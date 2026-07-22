"""Reusable raw-frame to stable rectified-board localization pipeline."""

from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Protocol

import numpy as np

from .apriltag_detector import AprilTagDetector
from .board_localizer import BoardLocalizer
from .camera_intrinsics import CameraIntrinsics
from .layout import AprilTagBoardLayout
from .models import DetectionBatch, LocalizationUpdate
from .temporal_localizer import TemporalBoardLocalizer


class TagDetector(Protocol):
    def detect(self, image: np.ndarray, timestamp: float | None = None) -> DetectionBatch:
        ...


@dataclass
class RectifiedBoardFrame:
    timestamp: float
    working_image: np.ndarray
    detection_batch: DetectionBatch
    localization: LocalizationUpdate
    standard_board_image: np.ndarray | None


class BoardLocalizationPipeline:
    """Undistort, detect, stabilize and rectify in the required order."""

    def __init__(
        self,
        layout: AprilTagBoardLayout,
        *,
        intrinsics: CameraIntrinsics | None = None,
        detector: TagDetector | None = None,
    ) -> None:
        self.layout = layout
        self.intrinsics = intrinsics
        self.detector = AprilTagDetector(layout) if detector is None else detector
        self.localizer = BoardLocalizer(layout)
        self.temporal = TemporalBoardLocalizer(self.localizer)

    @property
    def arm_busy(self) -> bool:
        return self.temporal.arm_busy

    def set_arm_busy(self, busy: bool) -> None:
        self.temporal.set_arm_busy(busy)

    def process_frame(
        self,
        raw_image: np.ndarray,
        *,
        timestamp: float | None = None,
    ) -> RectifiedBoardFrame:
        if raw_image is None or raw_image.size == 0:
            raise ValueError("Input image is empty")
        observed_at = time.time() if timestamp is None else float(timestamp)
        working_image = (
            raw_image.copy()
            if self.intrinsics is None
            else self.intrinsics.undistort(raw_image)
        )
        batch = self.detector.detect(working_image, timestamp=observed_at)
        update = self.temporal.update(
            working_image,
            batch,
            timestamp=observed_at,
        )
        if self.intrinsics is None and "CAMERA_INTRINSICS_MISSING" not in update.raw_result.warnings:
            update.raw_result.warnings.append("CAMERA_INTRINSICS_MISSING")

        standard_board = None
        if update.formal_result is not None:
            standard_board = self.localizer.warp(working_image, update.formal_result)

        return RectifiedBoardFrame(
            timestamp=observed_at,
            working_image=working_image,
            detection_batch=batch,
            localization=update,
            standard_board_image=standard_board,
        )
