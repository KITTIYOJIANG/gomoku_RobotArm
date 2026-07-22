"""Real-time black/white Gomoku piece center detection with OpenCV.

Run:
    python piece_center_detect.py

Press q to quit.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Tuple

import cv2
import numpy as np


PieceType = str
Point = Tuple[int, int]
Detection = Tuple[PieceType, np.ndarray, Point]


@dataclass
class DetectionTrack:
    piece_type: PieceType
    contour: np.ndarray
    center: np.ndarray
    hits: int = 1
    missed: int = 0


@dataclass(frozen=True)
class DetectionConfig:
    """Thresholds and contour filters that can be tuned for the scene."""

    method: str = "hybrid"
    black_v_max: int = 80
    black_diff: float = 22.0
    black_p20_max: float = 105.0
    black_dark_ratio_min: float = 0.24
    black_rescue_enabled: bool = True
    black_blob_v_max: int = 125
    black_blob_min_distance: float = 8.0
    white_s_max: int = 115
    white_v_min: int = 135
    white_diff: float = 6.0
    min_area: float = 250.0
    max_area: float = 8000.0
    min_circularity: float = 0.45
    min_aspect_ratio: float = 0.55
    max_aspect_ratio: float = 1.80
    min_radius: int = 13
    max_radius: int = 32
    hough_dp: float = 1.2
    hough_min_dist: int = 32
    hough_param1: int = 90
    hough_param2: int = 30
    blur_size: int = 5
    morph_size: int = 5
    roi: Optional[Tuple[int, int, int, int]] = None


def _make_odd(value: int) -> int:
    """OpenCV blur kernels must be odd and positive."""

    value = max(1, int(value))
    return value if value % 2 == 1 else value + 1


def _clean_mask(mask: np.ndarray, morph_size: int) -> np.ndarray:
    """Remove small mask noise and fill small gaps inside stones."""

    kernel_size = max(1, int(morph_size))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
    cleaned = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_CLOSE, kernel, iterations=2)
    return cleaned


def _is_piece_like(contour: np.ndarray, config: DetectionConfig) -> bool:
    """Reject tiny noise, huge background regions, and very non-round objects."""

    area = cv2.contourArea(contour)
    if area < config.min_area or area > config.max_area:
        return False

    x, y, w, h = cv2.boundingRect(contour)
    if w == 0 or h == 0:
        return False

    aspect_ratio = w / float(h)
    if aspect_ratio < config.min_aspect_ratio or aspect_ratio > config.max_aspect_ratio:
        return False

    perimeter = cv2.arcLength(contour, True)
    if perimeter <= 0:
        return False

    circularity = 4.0 * np.pi * area / (perimeter * perimeter)
    return circularity >= config.min_circularity


def _find_filtered_contours(mask: np.ndarray, config: DetectionConfig) -> List[np.ndarray]:
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    filtered = [contour for contour in contours if _is_piece_like(contour, config)]
    return sorted(filtered, key=cv2.contourArea, reverse=True)


def _clip_patch(image: np.ndarray, x: int, y: int, radius: int) -> Tuple[np.ndarray, np.ndarray]:
    h, w = image.shape[:2]
    x0 = max(0, x - radius)
    x1 = min(w, x + radius + 1)
    y0 = max(0, y - radius)
    y1 = min(h, y + radius + 1)
    patch = image[y0:y1, x0:x1]
    yy, xx = np.ogrid[y0 - y : y1 - y, x0 - x : x1 - x]
    distance_sq = xx * xx + yy * yy
    return patch, distance_sq


def _masked_mean(values: np.ndarray) -> float:
    if values.size == 0:
        return 0.0
    return float(np.mean(values))


def _classify_circle(
    gray: np.ndarray,
    hsv: np.ndarray,
    x: int,
    y: int,
    radius: int,
    config: DetectionConfig,
) -> Optional[PieceType]:
    inner_radius = max(4, int(radius * 0.58))
    outer_radius = max(inner_radius + 3, int(radius * 1.75))

    gray_patch, distance_sq = _clip_patch(gray, x, y, outer_radius)
    hsv_patch, _ = _clip_patch(hsv, x, y, outer_radius)
    if gray_patch.size == 0 or hsv_patch.size == 0:
        return None

    inner_mask = distance_sq <= inner_radius * inner_radius
    ring_mask = (distance_sq >= int(radius * 1.15) ** 2) & (
        distance_sq <= outer_radius * outer_radius
    )
    if not np.any(inner_mask) or not np.any(ring_mask):
        return None

    center_gray = _masked_mean(gray_patch[inner_mask])
    bg_gray = _masked_mean(gray_patch[ring_mask])
    center_hsv = hsv_patch[inner_mask]
    center_sat = _masked_mean(center_hsv[:, 1])
    center_val = _masked_mean(center_hsv[:, 2])
    center_val_p20 = float(np.percentile(center_hsv[:, 2], 20))
    dark_ratio = float(np.mean(center_hsv[:, 2] <= config.black_blob_v_max))
    diff = center_gray - bg_gray

    # Black stones can have bright highlights. Use the darker percentile and
    # dark-pixel ratio so clustered stones still classify as black.
    if (
        center_val <= config.black_v_max
        or diff <= -config.black_diff
        or center_val_p20 <= config.black_p20_max
        or dark_ratio >= config.black_dark_ratio_min
    ):
        return "Black"

    # White stones may be only slightly brighter than the board under USB lighting,
    # so use local contrast plus saturation/brightness instead of a global white mask.
    if (
        center_val >= config.white_v_min
        and center_sat <= config.white_s_max
        and diff >= config.white_diff
    ):
        return "White"

    return None


def _circle_to_contour(x: int, y: int, radius: int) -> np.ndarray:
    points = cv2.ellipse2Poly((x, y), (radius, radius), 0, 0, 360, 10)
    return points.reshape(-1, 1, 2)


def _dedupe_circles(circles: List[Tuple[int, int, int]]) -> List[Tuple[int, int, int]]:
    deduped: List[Tuple[int, int, int]] = []
    for x, y, radius in sorted(circles, key=lambda item: item[2], reverse=True):
        too_close = False
        for kept_x, kept_y, kept_radius in deduped:
            distance = np.hypot(x - kept_x, y - kept_y)
            if distance < max(radius, kept_radius) * 0.75:
                too_close = True
                break
        if not too_close:
            deduped.append((x, y, radius))
    return deduped


def _has_nearby_contour(
    contours: Iterable[np.ndarray], x: int, y: int, min_distance: float
) -> bool:
    for contour in contours:
        cx, cy = get_center(contour)
        if np.hypot(x - cx, y - cy) < min_distance:
            return True
    return False


def _detect_black_blob_contours(
    frame: np.ndarray,
    config: DetectionConfig,
    existing_contours: Iterable[np.ndarray],
) -> List[np.ndarray]:
    """Find dark stone centers when several black stones touch each other.

    Thin board lines also appear dark, but their distance-transform peaks are
    shallow. Real stones create much larger dark blobs, even when clustered.
    """

    blur_size = _make_odd(config.blur_size)
    blurred = cv2.GaussianBlur(frame, (blur_size, blur_size), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
    value = hsv[:, :, 2]

    dark_mask = cv2.inRange(value, 0, config.black_blob_v_max)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    dark_mask = cv2.morphologyEx(dark_mask, cv2.MORPH_OPEN, kernel, iterations=1)
    dark_mask = cv2.morphologyEx(dark_mask, cv2.MORPH_CLOSE, kernel, iterations=2)

    distance = cv2.distanceTransform(dark_mask, cv2.DIST_L2, 5)
    local_kernel = cv2.getStructuringElement(
        cv2.MORPH_RECT,
        (
            max(3, config.min_radius),
            max(3, config.min_radius),
        ),
    )
    local_max = cv2.dilate(distance, local_kernel)
    peaks = (
        (distance >= config.black_blob_min_distance)
        & (distance == local_max)
    ).astype(np.uint8)

    count, labels, stats, _centroids = cv2.connectedComponentsWithStats(peaks, 8)
    contours: List[np.ndarray] = []
    used_centers: List[np.ndarray] = list(existing_contours)
    duplicate_distance = max(config.min_radius, config.hough_min_dist * 0.45)

    for label in range(1, count):
        if stats[label, cv2.CC_STAT_AREA] <= 0:
            continue

        ys, xs = np.where(labels == label)
        if xs.size == 0:
            continue

        best_index = int(np.argmax(distance[ys, xs]))
        x = int(xs[best_index])
        y = int(ys[best_index])
        peak_radius = float(distance[y, x])
        if peak_radius < config.black_blob_min_distance:
            continue
        if _has_nearby_contour(used_centers, x, y, duplicate_distance):
            continue

        draw_radius = int(np.clip(peak_radius * 1.55, config.min_radius, config.max_radius))
        if _classify_circle(gray, hsv, x, y, draw_radius, config) != "Black":
            continue

        contour = _circle_to_contour(x, y, draw_radius)
        if _is_piece_like(contour, config):
            contours.append(contour)
            used_centers.append(contour)

    return contours


def _offset_contours(
    contours_by_type: Dict[PieceType, List[np.ndarray]], dx: int, dy: int
) -> Dict[PieceType, List[np.ndarray]]:
    if dx == 0 and dy == 0:
        return contours_by_type
    offset = np.array([[[dx, dy]]], dtype=np.int32)
    return {
        piece_type: [contour + offset for contour in contours]
        for piece_type, contours in contours_by_type.items()
    }


def _detect_circle_contours(
    frame: np.ndarray, config: DetectionConfig
) -> Dict[PieceType, List[np.ndarray]]:
    blur_size = _make_odd(config.blur_size)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (blur_size, blur_size), 0)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    circles = cv2.HoughCircles(
        gray,
        cv2.HOUGH_GRADIENT,
        dp=config.hough_dp,
        minDist=config.hough_min_dist,
        param1=config.hough_param1,
        param2=config.hough_param2,
        minRadius=config.min_radius,
        maxRadius=config.max_radius,
    )

    result: Dict[PieceType, List[np.ndarray]] = {"Black": [], "White": []}
    if circles is None:
        return result

    rounded = np.round(circles[0, :]).astype("int")
    for x, y, radius in _dedupe_circles([(int(x), int(y), int(r)) for x, y, r in rounded]):
        piece_type = _classify_circle(gray, hsv, x, y, radius, config)
        if piece_type is None:
            continue
        contour = _circle_to_contour(x, y, radius)
        if _is_piece_like(contour, config):
            result[piece_type].append(contour)

    return result


def _detect_hybrid_contours(
    frame: np.ndarray, config: DetectionConfig
) -> Dict[PieceType, List[np.ndarray]]:
    result = _detect_circle_contours(frame, config)
    if config.black_rescue_enabled:
        black_rescue = _detect_black_blob_contours(
            frame, config, result["Black"] + result["White"]
        )
        result["Black"].extend(black_rescue)
    return result


def _detect_mask_contours(
    frame: np.ndarray, config: DetectionConfig
) -> Dict[PieceType, List[np.ndarray]]:
    """HSV mask fallback. Circle mode is preferred for grid-line scenes."""

    blur_size = _make_odd(config.blur_size)
    blurred = cv2.GaussianBlur(frame, (blur_size, blur_size), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    black_lower = np.array([0, 0, 0], dtype=np.uint8)
    black_upper = np.array([179, 255, config.black_v_max], dtype=np.uint8)
    black_mask = cv2.inRange(hsv, black_lower, black_upper)

    white_lower = np.array([0, 0, config.white_v_min], dtype=np.uint8)
    white_upper = np.array([179, config.white_s_max, 255], dtype=np.uint8)
    white_mask = cv2.inRange(hsv, white_lower, white_upper)

    black_mask = _clean_mask(black_mask, config.morph_size)
    white_mask = _clean_mask(white_mask, config.morph_size)

    return {
        "Black": _find_filtered_contours(black_mask, config),
        "White": _find_filtered_contours(white_mask, config),
    }


def get_piece_contours(
    frame: np.ndarray, config: DetectionConfig
) -> Dict[PieceType, List[np.ndarray]]:
    """Return contours for black and white stones in the current frame."""

    dx = dy = 0
    detect_frame = frame
    if config.roi is not None:
        x, y, w, h = config.roi
        frame_h, frame_w = frame.shape[:2]
        x0 = max(0, x)
        y0 = max(0, y)
        x1 = min(frame_w, x + w)
        y1 = min(frame_h, y + h)
        detect_frame = frame[y0:y1, x0:x1]
        dx, dy = x0, y0

    if detect_frame.size == 0:
        return {"Black": [], "White": []}

    if config.method == "contour":
        contours_by_type = _detect_mask_contours(detect_frame, config)
    elif config.method == "circle":
        contours_by_type = _detect_circle_contours(detect_frame, config)
    else:
        contours_by_type = _detect_hybrid_contours(detect_frame, config)

    return _offset_contours(contours_by_type, dx, dy)


def get_center(contour: np.ndarray) -> Point:
    """Compute contour center with image moments, fallback to bounding box center."""

    moments = cv2.moments(contour)
    if moments["m00"] != 0:
        cx = int(moments["m10"] / moments["m00"])
        cy = int(moments["m01"] / moments["m00"])
        return cx, cy

    x, y, w, h = cv2.boundingRect(contour)
    return x + w // 2, y + h // 2


