"""Temporal validation and arm-occlusion freezing for board localization."""

from __future__ import annotations

import time

import numpy as np

from .board_localizer import BoardLocalizer
from .models import (
    DetectionBatch,
    LocalizationResult,
    LocalizationStatus,
    LocalizationUpdate,
)


class TemporalBoardLocalizer:
    """Promote only consecutive geometrically consistent candidates."""

    def __init__(self, localizer: BoardLocalizer) -> None:
        self.localizer = localizer
        self.layout = localizer.layout
        self.candidate_homography: np.ndarray | None = None
        self.stable_homography: np.ndarray | None = None
        self.last_valid_homography: np.ndarray | None = None
        self.last_valid_pose = None
        self.localization_confidence = 0.0
        self.stable_frame_count = 0
        self._candidate_result: LocalizationResult | None = None
        self._stable_result: LocalizationResult | None = None
        self._arm_busy = False
        self._last_success_timestamp: float | None = None

    @property
    def arm_busy(self) -> bool:
        return self._arm_busy

    @property
    def stable_result(self) -> LocalizationResult | None:
        return self._stable_result

    def set_arm_busy(self, busy: bool) -> None:
        busy = bool(busy)
        if busy != self._arm_busy:
            self._reset_candidate()
        self._arm_busy = busy

    def update(
        self,
        image: np.ndarray,
        batch: DetectionBatch,
        *,
        timestamp: float | None = None,
    ) -> LocalizationUpdate:
        observed_at = time.time() if timestamp is None else float(timestamp)
        raw_result = self.localizer.localize(image, batch)
        tag_status = self._tag_status(raw_result, len(batch.accepted))

        if self._arm_busy:
            self._reset_candidate()
            return self._build_update(
                raw_result,
                tag_status,
                committed=False,
                recognition_allowed=False,
                reason="ARM_BUSY_FROZEN",
            )

        if not raw_result.valid:
            self._reset_candidate()
            return self._build_update(
                raw_result,
                tag_status,
                committed=False,
                recognition_allowed=False,
                reason=raw_result.error_code or tag_status.value,
            )

        reason = "CANDIDATE_STABILIZING"
        if self._candidate_result is None:
            self._set_candidate(raw_result, 1)
        else:
            jump = board_corner_jump_px(self._candidate_result, raw_result)
            if jump <= self.layout.homography_jump_threshold:
                self._set_candidate(raw_result, self.stable_frame_count + 1)
            else:
                self._set_candidate(raw_result, 1)
                reason = "CANDIDATE_RESET_HOMOGRAPHY_JUMP"

        committed = self.stable_frame_count >= self.layout.stable_frame_count
        if committed:
            self._commit_candidate(observed_at)
            reason = "LOCALIZATION_COMMITTED"

        recognition_allowed = self._recognition_allowed(raw_result, committed)
        return self._build_update(
            raw_result,
            tag_status,
            committed=committed,
            recognition_allowed=recognition_allowed,
            reason=reason,
        )

    def _recognition_allowed(
        self,
        raw_result: LocalizationResult,
        committed: bool,
    ) -> bool:
        if self._stable_result is None:
            return False
        if committed:
            return True
        return (
            raw_result.valid
            and board_corner_jump_px(self._stable_result, raw_result)
            <= self.layout.homography_jump_threshold
        )

    def _set_candidate(self, result: LocalizationResult, frame_count: int) -> None:
        self._candidate_result = result
        self.candidate_homography = _copy_matrix(result.h_image_to_board)
        self.stable_frame_count = frame_count

    def _commit_candidate(self, timestamp: float) -> None:
        assert self._candidate_result is not None
        self._stable_result = self._candidate_result
        self.stable_homography = _copy_matrix(self._candidate_result.h_image_to_board)
        self.last_valid_homography = _copy_matrix(self._candidate_result.h_image_to_board)
        self.localization_confidence = self._candidate_result.localization_confidence
        self._last_success_timestamp = timestamp
        self.stable_frame_count = self.layout.stable_frame_count

    def _reset_candidate(self) -> None:
        self._candidate_result = None
        self.candidate_homography = None
        self.stable_frame_count = 0

    def _build_update(
        self,
        raw_result: LocalizationResult,
        tag_status: LocalizationStatus,
        *,
        committed: bool,
        recognition_allowed: bool,
        reason: str,
    ) -> LocalizationUpdate:
        using_last = self._stable_result is not None and not committed
        return LocalizationUpdate(
            raw_result=raw_result,
            tag_status=tag_status,
            formal_result=self._stable_result,
            stable_frame_count=self.stable_frame_count,
            required_stable_frames=self.layout.stable_frame_count,
            committed=committed,
            used_last_valid_localization=using_last,
            arm_busy=self._arm_busy,
            recognition_allowed=recognition_allowed,
            transition_reason=reason,
            last_success_timestamp=self._last_success_timestamp,
        )

    @staticmethod
    def _tag_status(
        result: LocalizationResult,
        accepted_count: int,
    ) -> LocalizationStatus:
        if result.status == LocalizationStatus.LOCALIZATION_REJECTED:
            return LocalizationStatus.LOCALIZATION_REJECTED
        if accepted_count >= 4:
            return LocalizationStatus.TAG_FULL
        if accepted_count == 3:
            return LocalizationStatus.TAG_PARTIAL
        if accepted_count in (1, 2):
            return LocalizationStatus.TAG_INSUFFICIENT
        return LocalizationStatus.TAG_LOST


def board_corner_jump_px(
    previous: LocalizationResult,
    current: LocalizationResult,
) -> float:
    if previous.board_corners_image is None or current.board_corners_image is None:
        return float("inf")
    difference = np.asarray(current.board_corners_image) - np.asarray(previous.board_corners_image)
    return float(np.max(np.linalg.norm(difference, axis=1)))


def _copy_matrix(matrix: np.ndarray | None) -> np.ndarray | None:
    return None if matrix is None else np.asarray(matrix, dtype=np.float64).copy()
