from __future__ import annotations

from dataclasses import asdict, dataclass, field
import logging
import time
from typing import Any

import cv2
import numpy as np

from app.config import PieceRecognitionConfig

from .overlay import grid_points_from_homography
from .piece_center_detect import DetectionConfig, get_center, get_piece_contours
from .stone_detector import BLACK, EMPTY, WHITE


LOGGER = logging.getLogger(__name__)


@dataclass(frozen=True)
class PieceCandidateDiag:
    candidate_id: int
    center_x: float
    center_y: float
    radius: float
    gray_mean: float
    hsv_mean: tuple[float, float, float]
    classified_color: str
    nearest_row: int | None
    nearest_col: int | None
    distance_px: float | None
    local_grid_spacing: float
    normalized_distance: float | None
    accepted: bool
    reject_reason: str

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class PieceRecognitionResult:
    board_matrix: tuple[tuple[int, ...], ...]
    black_count: int
    white_count: int
    empty_count: int
    timestamp: float
    method: str = "hybrid_circle"
    diagnostics: tuple[PieceCandidateDiag, ...] = ()
    spacing: float = 0.0
    hough_min_dist: float = 0.0
    max_snap: float = 0.0

    @property
    def summary(self) -> str:
        return (
            f"BLACK={self.black_count} WHITE={self.white_count} "
            f"EMPTY={self.empty_count} ({self.method})"
        )


class PieceRecognizer:
    """Hybrid circle/HSV detector with full candidate diagnostics."""

    def __init__(self, config: PieceRecognitionConfig) -> None:
        self.config = config
        self.diagnostic_mode = True
        self.last_diagnostics: tuple[PieceCandidateDiag, ...] = ()
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
            hough_param2=26,
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
        """Run on pristine detection_frame (no overlays)."""
        if detection_frame is None or detection_frame.size == 0:
            empty = tuple(tuple(0 for _ in range(board_size)) for _ in range(board_size))
            return PieceRecognitionResult(empty, 0, 0, board_size * board_size, time.time())

        grid = grid_points_from_homography(homography, board_size)
        spacing = _median_grid_spacing(grid)
        detect_config = self._config_for_board(grid, detection_frame.shape[:2], spacing)
        max_snap = max(6.0, spacing * 0.38)

        # Collect raw candidates from hybrid detector as classified contours,
        # plus low-level Hough candidates for diagnostics (even if unclassified).
        raw_circles = _hough_candidates(detection_frame, detect_config)
        contours_by_type = get_piece_contours(detection_frame, detect_config)

        diagnostics: list[PieceCandidateDiag] = []
        candidate_id = 0

        # Diagnostic for every Hough circle (pre-snap).
        gray = cv2.cvtColor(detection_frame, cv2.COLOR_BGR2GRAY)
        hsv = cv2.cvtColor(detection_frame, cv2.COLOR_BGR2HSV)
        for x, y, radius in raw_circles:
            gmean, hmean = _local_means(gray, hsv, x, y, radius)
            color = _classify_from_means(gmean, hmean, detect_config, gray, x, y, radius)
            row, col, dist = _nearest_intersection(grid, x, y)
            norm = dist / max(spacing, 1e-6)
            in_board = _point_in_board(grid, x, y)
            accepted = False
            reason = "pending"
            if not in_board:
                reason = "outside_board"
            elif color is None:
                reason = "unclassified_color"
            elif dist > max_snap:
                reason = "snap_too_far"
            else:
                reason = "candidate_ok"
            diagnostics.append(
                PieceCandidateDiag(
                    candidate_id=candidate_id,
                    center_x=float(x),
                    center_y=float(y),
                    radius=float(radius),
                    gray_mean=gmean,
                    hsv_mean=hmean,
                    classified_color=color or "None",
                    nearest_row=row if in_board else None,
                    nearest_col=col if in_board else None,
                    distance_px=dist,
                    local_grid_spacing=spacing,
                    normalized_distance=norm,
                    accepted=False,
                    reject_reason=reason,
                )
            )
            candidate_id += 1
            LOGGER.info(
                "[PIECE][CANDIDATE] id=%s xy=(%.1f,%.1f) r=%.1f gray=%.1f hsv=(%.1f,%.1f,%.1f) "
                "color=%s nearest=(%s,%s) dist=%.2f spacing=%.2f norm=%.3f reason=%s",
                candidate_id - 1,
                x,
                y,
                radius,
                gmean,
                hmean[0],
                hmean[1],
                hmean[2],
                color or "None",
                row,
                col,
                dist,
                spacing,
                norm,
                reason,
            )

        # Build assignments from classified contours (hybrid path, includes black rescue).
        matrix = np.zeros((board_size, board_size), dtype=np.int32)
        assignments: list[tuple[float, int, int, int, float, float, float]] = []
        for piece_name, code in (("Black", BLACK), ("White", WHITE)):
            for contour in contours_by_type.get(piece_name, []):
                cx, cy = get_center(contour)
                # approximate radius from contour area
                area = float(cv2.contourArea(contour))
                radius = max(1.0, (area / np.pi) ** 0.5)
                if not _point_in_board(grid, cx, cy):
                    LOGGER.info(
                        "[PIECE][SNAP] color=%s xy=(%.1f,%.1f) rejected=outside_board",
                        piece_name,
                        cx,
                        cy,
                    )
                    continue
                row, col, dist = _nearest_intersection(grid, cx, cy)
                norm = dist / max(spacing, 1e-6)
                if dist > max_snap:
                    LOGGER.info(
                        "[PIECE][SNAP] color=%s xy=(%.1f,%.1f) nearest=(%d,%d) dist=%.2f "
                        "norm=%.3f rejected=snap_too_far max_snap=%.2f",
                        piece_name,
                        cx,
                        cy,
                        row,
                        col,
                        dist,
                        norm,
                        max_snap,
                    )
                    continue
                assignments.append((dist, row, col, code, cx, cy, radius))

        assignments.sort(key=lambda item: item[0])
        occupied: dict[tuple[int, int], tuple[float, int]] = {}
        accepted_cells: list[tuple[int, int, int, float, float, float]] = []
        for dist, row, col, code, cx, cy, radius in assignments:
            key = (row, col)
            if key in occupied:
                prev_dist, prev_code = occupied[key]
                LOGGER.info(
                    "[PIECE][SNAP] color=%s xy=(%.1f,%.1f) nearest=(%d,%d) dist=%.2f "
                    "rejected=cell_occupied prev_code=%s prev_dist=%.2f",
                    "Black" if code == BLACK else "White",
                    cx,
                    cy,
                    row,
                    col,
                    dist,
                    prev_code,
                    prev_dist,
                )
                # Keep closer one (already sorted); mark diagnostic if matching hough cand
                for i, diag in enumerate(diagnostics):
                    if abs(diag.center_x - cx) < 3 and abs(diag.center_y - cy) < 3:
                        diagnostics[i] = PieceCandidateDiag(
                            **{
                                **diag.to_dict(),
                                "accepted": False,
                                "reject_reason": "cell_occupied",
                            }
                        )
                continue
            matrix[row, col] = code
            occupied[key] = (dist, code)
            accepted_cells.append((row, col, code, cx, cy, radius))
            LOGGER.info(
                "[PIECE][SNAP] color=%s xy=(%.1f,%.1f) nearest=(%d,%d) dist=%.2f "
                "norm=%.3f accepted=1",
                "Black" if code == BLACK else "White",
                cx,
                cy,
                row,
                col,
                dist,
                dist / max(spacing, 1e-6),
            )
            for i, diag in enumerate(diagnostics):
                if abs(diag.center_x - cx) < 4 and abs(diag.center_y - cy) < 4:
                    diagnostics[i] = PieceCandidateDiag(
                        **{
                            **diag.to_dict(),
                            "accepted": True,
                            "reject_reason": "accepted",
                            "nearest_row": row,
                            "nearest_col": col,
                            "distance_px": dist,
                            "normalized_distance": dist / max(spacing, 1e-6),
                            "classified_color": "Black" if code == BLACK else "White",
                        }
                    )

        # Fallback: if hybrid contours missed a white Hough candidate that is OK, accept it.
        for i, diag in enumerate(diagnostics):
            if diag.accepted or diag.classified_color != "White":
                continue
            if diag.reject_reason not in {"candidate_ok", "pending"}:
                continue
            if diag.nearest_row is None or diag.nearest_col is None:
                continue
            key = (diag.nearest_row, diag.nearest_col)
            if key in occupied:
                diagnostics[i] = PieceCandidateDiag(
                    **{**diag.to_dict(), "reject_reason": "cell_occupied"}
                )
                continue
            if diag.distance_px is not None and diag.distance_px <= max_snap:
                matrix[diag.nearest_row, diag.nearest_col] = WHITE
                occupied[key] = (float(diag.distance_px), WHITE)
                diagnostics[i] = PieceCandidateDiag(
                    **{**diag.to_dict(), "accepted": True, "reject_reason": "accepted_hough_fallback"}
                )
                LOGGER.info(
                    "[PIECE][FALLBACK_WHITE] id=%s nearest=(%d,%d) dist=%.2f",
                    diag.candidate_id,
                    diag.nearest_row,
                    diag.nearest_col,
                    diag.distance_px,
                )

        immutable = tuple(tuple(int(v) for v in row) for row in matrix)
        flat = [v for row in immutable for v in row]
        diag_tuple = tuple(diagnostics)
        self.last_diagnostics = diag_tuple
        result = PieceRecognitionResult(
            board_matrix=immutable,
            black_count=flat.count(BLACK),
            white_count=flat.count(WHITE),
            empty_count=flat.count(EMPTY),
            timestamp=time.time(),
            method=f"hybrid:{detect_config.method}",
            diagnostics=diag_tuple,
            spacing=spacing,
            hough_min_dist=float(detect_config.hough_min_dist),
            max_snap=max_snap,
        )
        LOGGER.info(
            "[PIECE][SUMMARY] %s spacing=%.2f hough_minDist=%s max_snap=%.2f "
            "raw_hough=%d rawB=%d rawW=%d accepted=%d",
            result.summary,
            spacing,
            detect_config.hough_min_dist,
            max_snap,
            len(raw_circles),
            len(contours_by_type.get("Black", [])),
            len(contours_by_type.get("White", [])),
            sum(1 for d in diagnostics if d.accepted),
        )
        return result

    def _config_for_board(
        self,
        grid: np.ndarray,
        frame_shape: tuple[int, int],
        spacing: float,
    ) -> DetectionConfig:
        min_r = max(8, int(round(spacing * 0.28)))
        max_r = max(min_r + 4, int(round(spacing * 0.55)))
        # Critical: minDist must stay below one cell so adjacent stones are not merged.
        min_dist = max(min_r, int(round(spacing * 0.60)))
        min_area = float(max(120.0, np.pi * (min_r * 0.7) ** 2))
        max_area = float(max(min_area + 100.0, np.pi * (max_r * 1.15) ** 2))

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


def _hough_candidates(
    frame: np.ndarray, config: DetectionConfig
) -> list[tuple[int, int, int]]:
    detect = frame
    ox = oy = 0
    if config.roi is not None:
        x, y, w, h = config.roi
        fh, fw = frame.shape[:2]
        x0, y0 = max(0, x), max(0, y)
        x1, y1 = min(fw, x + w), min(fh, y + h)
        detect = frame[y0:y1, x0:x1]
        ox, oy = x0, y0
    if detect.size == 0:
        return []
    blur = config.blur_size if config.blur_size % 2 == 1 else config.blur_size + 1
    gray = cv2.GaussianBlur(cv2.cvtColor(detect, cv2.COLOR_BGR2GRAY), (blur, blur), 0)
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
    if circles is None:
        return []
    out: list[tuple[int, int, int]] = []
    for x, y, r in np.round(circles[0, :]).astype(int):
        out.append((int(x + ox), int(y + oy), int(r)))
    return out


def _local_means(
    gray: np.ndarray, hsv: np.ndarray, x: int, y: int, radius: int
) -> tuple[float, tuple[float, float, float]]:
    r = max(3, int(radius * 0.6))
    h, w = gray.shape[:2]
    x0, x1 = max(0, x - r), min(w, x + r + 1)
    y0, y1 = max(0, y - r), min(h, y + r + 1)
    g = gray[y0:y1, x0:x1]
    hv = hsv[y0:y1, x0:x1]
    if g.size == 0:
        return 0.0, (0.0, 0.0, 0.0)
    return float(np.mean(g)), (
        float(np.mean(hv[:, :, 0])),
        float(np.mean(hv[:, :, 1])),
        float(np.mean(hv[:, :, 2])),
    )


def _classify_from_means(
    gray_mean: float,
    hsv_mean: tuple[float, float, float],
    config: DetectionConfig,
    gray: np.ndarray,
    x: int,
    y: int,
    radius: int,
) -> str | None:
    # Local ring contrast
    outer = max(radius + 3, int(radius * 1.6))
    h, w = gray.shape[:2]
    yy, xx = np.ogrid[
        max(0, y - outer) - y : min(h, y + outer + 1) - y,
        max(0, x - outer) - x : min(w, x + outer + 1) - x,
    ]
    dist2 = xx * xx + yy * yy
    patch = gray[max(0, y - outer) : min(h, y + outer + 1), max(0, x - outer) : min(w, x + outer + 1)]
    if patch.size == 0:
        return None
    inner = dist2 <= max(4, int(radius * 0.55)) ** 2
    ring = (dist2 >= int(radius * 1.1) ** 2) & (dist2 <= outer * outer)
    if not np.any(inner) or not np.any(ring):
        return None
    center = float(np.mean(patch[inner]))
    bg = float(np.mean(patch[ring]))
    diff = center - bg
    _, sat, val = hsv_mean
    if val <= config.black_v_max or diff <= -config.black_diff:
        return "Black"
    if val >= config.white_v_min and sat <= config.white_s_max and diff >= config.white_diff:
        return "White"
    return None


def _median_grid_spacing(grid: np.ndarray) -> float:
    samples: list[float] = []
    rows, cols = grid.shape[:2]
    for r in range(rows):
        for c in range(cols - 1):
            samples.append(float(np.linalg.norm(grid[r, c + 1] - grid[r, c])))
    for r in range(rows - 1):
        for c in range(cols):
            samples.append(float(np.linalg.norm(grid[r + 1, c] - grid[r, c])))
    return float(np.median(samples)) if samples else 20.0


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
    return cv2.pointPolygonTest(corners.reshape(-1, 1, 2), (float(x), float(y)), False) >= 0
