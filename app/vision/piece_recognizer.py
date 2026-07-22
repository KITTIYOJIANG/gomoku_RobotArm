from __future__ import annotations

from dataclasses import dataclass
import logging
import time

import cv2
import numpy as np

from app.config import PieceRecognitionConfig

from .overlay import grid_points_from_homography
from .piece_center_detect import DetectionConfig, get_center, get_piece_contours
from .stone_detector import BLACK, EMPTY, WHITE


LOGGER = logging.getLogger(__name__)


@dataclass(frozen=True)
class PieceRecognitionResult:
    board_matrix: tuple[tuple[int, ...], ...]
    black_count: int
    white_count: int
    empty_count: int
    timestamp: float
    method: str = "hybrid_circle"

    @property
    def summary(self) -> str:
        return (
            f"BLACK={self.black_count} WHITE={self.white_count} "
            f"EMPTY={self.empty_count} ({self.method})"
        )


class PieceRecognizer:
    """Assign stones with original hybrid circle/HSV detector + nearest grid snap."""

    def __init__(self, config: PieceRecognitionConfig) -> None:
        self.config = config
        # Defaults tuned for the integrated overhead camera; ROI/radius adapted
        # from board geometry each frame when possible.
        self.detection_config = DetectionConfig(
            method="hybrid",
            black_v_max=80,
            black_diff=22.0,
            black_p20_max=105.0,
            black_dark_ratio_min=0.24,
            black_rescue_enabled=True,
            black_blob_v_max=125,
            black_blob_min_distance=8.0,
            white_s_max=140,
            white_v_min=120,
            white_diff=3.0,
            min_area=250.0,
            max_area=8000.0,
            min_circularity=0.45,
            min_aspect_ratio=0.55,
            max_aspect_ratio=1.80,
            min_radius=13,
            max_radius=32,
            hough_dp=1.2,
            hough_min_dist=32,
            hough_param1=90,
            hough_param2=30,
            blur_size=5,
            morph_size=5,
            roi=None,
        )

    def recognize(
        self,
        detection_frame: np.ndarray,
        *,
        homography: np.ndarray,
        board_size: int,
    ) -> PieceRecognitionResult:
        grid = grid_points_from_homography(homography, board_size)
        detect_config = self._config_for_board(grid, detection_frame.shape[:2])

        contours_by_type = get_piece_contours(detection_frame, detect_config)
        matrix = np.zeros((board_size, board_size), dtype=np.int32)

        # Local spacing for snap threshold.
        spacing = _median_grid_spacing(grid)
        max_snap = max(6.0, spacing * 0.35)

        assignments: list[tuple[float, int, int, int]] = []
        for piece_name, code in (("Black", BLACK), ("White", WHITE)):
            for contour in contours_by_type.get(piece_name, []):
                cx, cy = get_center(contour)
                # Reject detections clearly outside board polygon.
                if not _point_in_board(grid, cx, cy):
                    continue
                row, col, dist = _nearest_intersection(grid, cx, cy)
                if dist > max_snap:
                    continue
                assignments.append((dist, row, col, code))

        # Prefer closer detections when two stones compete for one cell.
        assignments.sort(key=lambda item: item[0])
        occupied: set[tuple[int, int]] = set()
        for _dist, row, col, code in assignments:
            key = (row, col)
            if key in occupied:
                continue
            matrix[row, col] = code
            occupied.add(key)

        immutable = tuple(tuple(int(v) for v in row) for row in matrix)
        flat = [v for row in immutable for v in row]
        result = PieceRecognitionResult(
            board_matrix=immutable,
            black_count=flat.count(BLACK),
            white_count=flat.count(WHITE),
            empty_count=flat.count(EMPTY),
            timestamp=time.time(),
            method=f"hybrid:{detect_config.method}",
        )
        LOGGER.info(
            "PIECE HYBRID %s snap<=%.1f spacing=%.1f rawB=%d rawW=%d",
            result.summary,
            max_snap,
            spacing,
            len(contours_by_type.get("Black", [])),
            len(contours_by_type.get("White", [])),
        )
        return result

    def _config_for_board(
        self,
        grid: np.ndarray,
        frame_shape: tuple[int, int],
    ) -> DetectionConfig:
        spacing = _median_grid_spacing(grid)
        # Stone radius roughly 0.35-0.45 of grid spacing on this setup.
        min_r = max(8, int(round(spacing * 0.28)))
        max_r = max(min_r + 4, int(round(spacing * 0.55)))
        min_dist = max(min_r + 2, int(round(spacing * 0.75)))
        min_area = float(max(120.0, np.pi * (min_r * 0.7) ** 2))
        max_area = float(max(min_area + 100.0, np.pi * (max_r * 1.15) ** 2))

        # Board axis-aligned ROI with margin.
        xs = grid[:, :, 0].reshape(-1)
        ys = grid[:, :, 1].reshape(-1)
        x0 = int(max(0, np.floor(xs.min() - spacing)))
        y0 = int(max(0, np.floor(ys.min() - spacing)))
        x1 = int(min(frame_shape[1], np.ceil(xs.max() + spacing)))
        y1 = int(min(frame_shape[0], np.ceil(ys.max() + spacing)))
        roi = (x0, y0, max(1, x1 - x0), max(1, y1 - y0))

        base = self.detection_config
        return DetectionConfig(
            method=base.method,
            black_v_max=base.black_v_max,
            black_diff=base.black_diff,
            black_p20_max=base.black_p20_max,
            black_dark_ratio_min=base.black_dark_ratio_min,
            black_rescue_enabled=base.black_rescue_enabled,
            black_blob_v_max=base.black_blob_v_max,
            black_blob_min_distance=max(4.0, spacing * 0.25),
            white_s_max=base.white_s_max,
            white_v_min=base.white_v_min,
            white_diff=base.white_diff,
            min_area=min_area,
            max_area=max_area,
            min_circularity=base.min_circularity,
            min_aspect_ratio=base.min_aspect_ratio,
            max_aspect_ratio=base.max_aspect_ratio,
            min_radius=min_r,
            max_radius=max_r,
            hough_dp=base.hough_dp,
            hough_min_dist=min_dist,
            hough_param1=base.hough_param1,
            hough_param2=base.hough_param2,
            blur_size=base.blur_size,
            morph_size=base.morph_size,
            roi=roi,
        )


def _median_grid_spacing(grid: np.ndarray) -> float:
    samples: list[float] = []
    rows, cols = grid.shape[:2]
    for r in range(rows):
        for c in range(cols - 1):
            samples.append(float(np.linalg.norm(grid[r, c + 1] - grid[r, c])))
    for r in range(rows - 1):
        for c in range(cols):
            samples.append(float(np.linalg.norm(grid[r + 1, c] - grid[r, c])))
    if not samples:
        return 20.0
    return float(np.median(samples))


def _nearest_intersection(grid: np.ndarray, x: float, y: float) -> tuple[int, int, float]:
    flat = grid.reshape(-1, 2)
    d = np.linalg.norm(flat - np.array([x, y], dtype=np.float32), axis=1)
    idx = int(np.argmin(d))
    cols = grid.shape[1]
    return idx // cols, idx % cols, float(d[idx])


def _point_in_board(grid: np.ndarray, x: float, y: float) -> bool:
    corners = np.array(
        [grid[0, 0], grid[0, -1], grid[-1, -1], grid[-1, 0]],
        dtype=np.float32,
    )
    # cv2.pointPolygonTest: >=0 inside or edge
    return cv2.pointPolygonTest(corners.reshape(-1, 1, 2), (float(x), float(y)), False) >= 0
