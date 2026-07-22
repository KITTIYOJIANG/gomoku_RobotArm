from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from app.vision.overlay import grid_points_from_homography


@dataclass(frozen=True)
class ClickSelection:
    accepted: bool
    reason: str
    row: int | None = None
    col: int | None = None
    pixel_x: float | None = None
    pixel_y: float | None = None
    click_distance: float | None = None
    local_grid_spacing: float | None = None
    distance_ratio: float | None = None

    def to_dict(self) -> dict[str, Any]:
        return {
            "accepted": self.accepted,
            "reason": self.reason,
            "row": self.row,
            "col": self.col,
            "pixel_x": self.pixel_x,
            "pixel_y": self.pixel_y,
            "click_distance": self.click_distance,
            "local_grid_spacing": self.local_grid_spacing,
            "distance_ratio": self.distance_ratio,
        }


def build_intersection_grid(homography: np.ndarray, board_size: int = 15) -> np.ndarray:
    """Return intersection_points[row, col] = (x, y) in image pixels."""
    return grid_points_from_homography(np.asarray(homography, dtype=np.float32), int(board_size))


def local_grid_spacing(grid: np.ndarray, row: int, col: int) -> float:
    samples: list[float] = []
    rows, cols = grid.shape[:2]
    for dr, dc in ((0, 1), (0, -1), (1, 0), (-1, 0)):
        rr, cc = row + dr, col + dc
        if 0 <= rr < rows and 0 <= cc < cols:
            samples.append(float(np.linalg.norm(grid[row, col] - grid[rr, cc])))
    if not samples:
        return 1.0
    return float(np.mean(samples))


def select_intersection(
    grid: np.ndarray,
    click_xy: tuple[float, float] | np.ndarray,
    *,
    threshold_ratio: float = 0.32,
    board_size: int | None = None,
) -> ClickSelection:
    points = np.asarray(grid, dtype=np.float32)
    if points.ndim != 3 or points.shape[2] != 2:
        raise ValueError("grid must have shape (rows, cols, 2)")
    rows, cols = points.shape[:2]
    if board_size is not None and (rows != board_size or cols != board_size):
        raise ValueError(f"grid size {rows}x{cols} does not match board_size={board_size}")

    click = np.asarray(click_xy, dtype=np.float32).reshape(2)
    if not np.all(np.isfinite(click)):
        return ClickSelection(False, "CLICK_REJECTED_INVALID_COORDINATES")

    corners = np.array(
        [points[0, 0], points[0, -1], points[-1, -1], points[-1, 0]],
        dtype=np.float32,
    )
    if not _point_in_convex_quad(click, corners):
        return ClickSelection(False, "CLICK_REJECTED_OUTSIDE_BOARD")

    flat = points.reshape(-1, 2)
    distances = np.linalg.norm(flat - click[None, :], axis=1)
    nearest = int(np.argmin(distances))
    row = nearest // cols
    col = nearest % cols
    distance = float(distances[nearest])
    spacing = local_grid_spacing(points, row, col)
    ratio = distance / max(spacing, 1e-6)
    if ratio > float(threshold_ratio):
        return ClickSelection(
            False,
            "CLICK_REJECTED_NOT_NEAR_INTERSECTION",
            row=row,
            col=col,
            pixel_x=float(points[row, col, 0]),
            pixel_y=float(points[row, col, 1]),
            click_distance=distance,
            local_grid_spacing=spacing,
            distance_ratio=ratio,
        )
    return ClickSelection(
        True,
        "TARGET_SELECTED",
        row=row,
        col=col,
        pixel_x=float(points[row, col, 0]),
        pixel_y=float(points[row, col, 1]),
        click_distance=distance,
        local_grid_spacing=spacing,
        distance_ratio=ratio,
    )


def _point_in_convex_quad(point: np.ndarray, quad: np.ndarray) -> bool:
    pts = np.asarray(quad, dtype=np.float32).reshape(4, 2)
    p = np.asarray(point, dtype=np.float32).reshape(2)
    signs: list[float] = []
    for i in range(4):
        a = pts[i]
        b = pts[(i + 1) % 4]
        cross = (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])
        signs.append(float(cross))
    positive = all(s >= -1e-3 for s in signs)
    negative = all(s <= 1e-3 for s in signs)
    return positive or negative
