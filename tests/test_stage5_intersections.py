from __future__ import annotations

import numpy as np

from app.stage5.board_intersections import build_intersection_grid, select_intersection
from app.vision.overlay import grid_points_from_homography


def _square_homography(size: int = 400) -> np.ndarray:
    # Board [0,1]x[0,1] maps to a square image region.
    src = np.array([[0, 0], [1, 0], [1, 1], [0, 1]], dtype=np.float32)
    dst = np.array([[50, 50], [350, 50], [350, 350], [50, 350]], dtype=np.float32)
    import cv2

    return cv2.getPerspectiveTransform(src, dst)


def test_build_intersection_grid_shape():
    H = _square_homography()
    grid = build_intersection_grid(H, 15)
    assert grid.shape == (15, 15, 2)
    # Top-left near (50,50), bottom-right near (350,350)
    assert np.allclose(grid[0, 0], [50, 50], atol=1.0)
    assert np.allclose(grid[14, 14], [350, 350], atol=1.0)


def test_select_intersection_accepts_near_center():
    H = _square_homography()
    grid = build_intersection_grid(H, 15)
    target = grid[7, 7]
    selection = select_intersection(grid, target + np.array([1.0, -1.0]), threshold_ratio=0.32)
    assert selection.accepted
    assert selection.row == 7
    assert selection.col == 7


def test_select_intersection_rejects_far_click():
    H = _square_homography()
    grid = build_intersection_grid(H, 15)
    # Midway between cells, far enough when threshold is tight.
    a = grid[7, 7]
    b = grid[7, 8]
    mid = (a + b) / 2.0
    selection = select_intersection(grid, mid, threshold_ratio=0.20)
    assert not selection.accepted
    assert selection.reason == "CLICK_REJECTED_NOT_NEAR_INTERSECTION"


def test_select_intersection_rejects_outside_board():
    H = _square_homography()
    grid = build_intersection_grid(H, 15)
    selection = select_intersection(grid, (10.0, 10.0), threshold_ratio=0.32)
    assert not selection.accepted
    assert selection.reason == "CLICK_REJECTED_OUTSIDE_BOARD"
