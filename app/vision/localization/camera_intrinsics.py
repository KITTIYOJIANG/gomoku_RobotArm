from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np


@dataclass(frozen=True)
class CameraIntrinsics:
    camera_name: str
    image_width: int
    image_height: int
    camera_matrix: np.ndarray
    dist_coeffs: np.ndarray
    calibration_rms: float | None = None

    @classmethod
    def from_file(cls, path: str | Path) -> "CameraIntrinsics":
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        if data.get("camera_matrix") is None or data.get("dist_coeffs") is None:
            raise ValueError("CAMERA_INTRINSICS_MISSING")
        matrix = np.asarray(data["camera_matrix"], dtype=np.float64)
        distortion = np.asarray(data["dist_coeffs"], dtype=np.float64).reshape(-1)
        if matrix.shape != (3, 3) or distortion.size < 4:
            raise ValueError("CAMERA_INTRINSICS_INVALID")
        return cls(
            camera_name=str(data.get("camera_name", "unknown")),
            image_width=int(data["image_width"]),
            image_height=int(data["image_height"]),
            camera_matrix=matrix,
            dist_coeffs=distortion,
            calibration_rms=(
                None if data.get("calibration_rms") is None else float(data["calibration_rms"])
            ),
        )

    def undistort(self, image: np.ndarray) -> np.ndarray:
        height, width = image.shape[:2]
        if (width, height) != (self.image_width, self.image_height):
            raise ValueError(
                "CAMERA_INTRINSICS_RESOLUTION_MISMATCH: "
                f"calibrated={self.image_width}x{self.image_height}, input={width}x{height}"
            )
        return cv2.undistort(image, self.camera_matrix, self.dist_coeffs)
