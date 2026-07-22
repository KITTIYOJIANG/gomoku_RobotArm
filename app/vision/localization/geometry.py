from __future__ import annotations

import cv2
import numpy as np


def polygon_signed_area(points: np.ndarray) -> float:
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    x = pts[:, 0]
    y = pts[:, 1]
    return float(0.5 * np.sum(x * np.roll(y, -1) - y * np.roll(x, -1)))


def is_convex_quad(points: np.ndarray) -> bool:
    pts = np.asarray(points, dtype=np.float32).reshape(4, 2)
    contour = pts.reshape(-1, 1, 2)
    return bool(cv2.isContourConvex(contour))


def transform_points(points: np.ndarray, homography: np.ndarray) -> np.ndarray:
    pts = np.asarray(points, dtype=np.float64).reshape(1, -1, 2)
    return cv2.perspectiveTransform(pts, homography).reshape(-1, 2)


def reprojection_rmse(
    object_points: np.ndarray,
    image_points: np.ndarray,
    h_board_to_image: np.ndarray,
) -> float:
    projected = transform_points(object_points, h_board_to_image)
    errors = np.linalg.norm(projected - np.asarray(image_points, dtype=np.float64), axis=1)
    return float(np.sqrt(np.mean(errors * errors)))
