from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any

import numpy as np


class LocalizationStatus(str, Enum):
    TAG_SEARCHING = "TAG_SEARCHING"
    TAG_FULL = "TAG_FULL"
    TAG_PARTIAL = "TAG_PARTIAL"
    TAG_INSUFFICIENT = "TAG_INSUFFICIENT"
    TAG_LOST = "TAG_LOST"
    BOARD_LOCALIZED = "BOARD_LOCALIZED"
    LOCALIZATION_REJECTED = "LOCALIZATION_REJECTED"


@dataclass(frozen=True)
class TagDetection:
    tag_id: int
    center: np.ndarray
    corners: np.ndarray
    decision_margin: float
    hamming: int
    timestamp: float

    def to_dict(self) -> dict[str, Any]:
        return {
            "tag_id": self.tag_id,
            "center": self.center.astype(float).tolist(),
            "corners": self.corners.astype(float).tolist(),
            "decision_margin": self.decision_margin,
            "hamming": self.hamming,
            "timestamp": self.timestamp,
        }


@dataclass(frozen=True)
class RejectedTag:
    tag_id: int
    reason: str
    decision_margin: float
    hamming: int

    def to_dict(self) -> dict[str, Any]:
        return {
            "tag_id": self.tag_id,
            "reason": self.reason,
            "decision_margin": self.decision_margin,
            "hamming": self.hamming,
        }


@dataclass
class DetectionBatch:
    accepted: list[TagDetection] = field(default_factory=list)
    rejected: list[RejectedTag] = field(default_factory=list)
    duplicate_ids: list[int] = field(default_factory=list)


@dataclass
class LocalizationResult:
    status: LocalizationStatus
    detections: list[TagDetection]
    board_corners_image: np.ndarray | None = None
    h_image_to_board: np.ndarray | None = None
    h_board_to_image: np.ndarray | None = None
    reprojection_error: float | None = None
    localization_confidence: float = 0.0
    error_code: str | None = None
    warnings: list[str] = field(default_factory=list)
    rejected_tags: list[RejectedTag] = field(default_factory=list)

    @property
    def valid(self) -> bool:
        return self.status == LocalizationStatus.BOARD_LOCALIZED

    def to_dict(self) -> dict[str, Any]:
        return {
            "status": self.status.value,
            "valid": self.valid,
            "tag_count": len(self.detections),
            "detections": [item.to_dict() for item in self.detections],
            "rejected_tags": [item.to_dict() for item in self.rejected_tags],
            "board_corners_image": (
                None
                if self.board_corners_image is None
                else self.board_corners_image.astype(float).tolist()
            ),
            "h_image_to_board": (
                None if self.h_image_to_board is None else self.h_image_to_board.astype(float).tolist()
            ),
            "h_board_to_image": (
                None if self.h_board_to_image is None else self.h_board_to_image.astype(float).tolist()
            ),
            "reprojection_error": self.reprojection_error,
            "localization_confidence": self.localization_confidence,
            "error_code": self.error_code,
            "warnings": list(self.warnings),
        }


@dataclass
class LocalizationUpdate:
    """Current detections plus the separately committed formal localization."""

    raw_result: LocalizationResult
    tag_status: LocalizationStatus
    formal_result: LocalizationResult | None
    stable_frame_count: int
    required_stable_frames: int
    committed: bool
    used_last_valid_localization: bool
    arm_busy: bool
    recognition_allowed: bool
    transition_reason: str | None = None
    last_success_timestamp: float | None = None

    @property
    def board_localized(self) -> bool:
        return self.formal_result is not None and self.formal_result.valid

    def to_dict(self) -> dict[str, Any]:
        return {
            "tag_status": self.tag_status.value,
            "board_localized": self.board_localized,
            "stable_frame_count": self.stable_frame_count,
            "required_stable_frames": self.required_stable_frames,
            "committed": self.committed,
            "used_last_valid_localization": self.used_last_valid_localization,
            "arm_busy": self.arm_busy,
            "recognition_allowed": self.recognition_allowed,
            "transition_reason": self.transition_reason,
            "last_success_timestamp": self.last_success_timestamp,
            "raw_result": self.raw_result.to_dict(),
        }
