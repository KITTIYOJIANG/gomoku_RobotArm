from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import time

import cv2
import numpy as np

from .localization.models import LocalizationUpdate


CORNER_LABELS = ("TL", "TR", "BR", "BL")


class BoardTrackState(str, Enum):
    LOCKED = "LOCKED"
    FROZEN = "FROZEN"
    LOST = "LOST"


@dataclass(frozen=True)
class BoardTrackSnapshot:
    state: BoardTrackState
    smoothed_corners: np.ndarray | None
    homography: np.ndarray | None
    failure_count: int
    last_success_timestamp: float | None

    @property
    def displayable(self) -> bool:
        return (
            self.state in (BoardTrackState.LOCKED, BoardTrackState.FROZEN)
            and self.smoothed_corners is not None
            and self.homography is not None
        )

    @property
    def placement_ready(self) -> bool:
        return self.state == BoardTrackState.LOCKED and self.displayable

    @property
    def display_status(self) -> str:
        if self.state == BoardTrackState.LOCKED:
            return "BOARD LOCKED"
        if self.state == BoardTrackState.FROZEN:
            return "BOARD LOCKED · FROZEN"
        return "BOARD LOST"

    @property
    def corner_status(self) -> str:
        count = 4 if self.displayable else 0
        return f"{count}/4 {self.state.value}"


class BoardTracker:
    """Caches one ordered TL/TR/BR/BL pose for every display overlay."""

    def __init__(
        self,
        *,
        smoothing_alpha: float = 0.35,
        failure_limit: int = 5,
        lost_timeout_seconds: float = 2.0,
    ) -> None:
        self.smoothing_alpha = float(smoothing_alpha)
        self.failure_limit = int(failure_limit)
        self.lost_timeout_seconds = float(lost_timeout_seconds)
        if not 0.0 < self.smoothing_alpha <= 1.0:
            raise ValueError("smoothing_alpha must be in (0, 1]")
        if self.failure_limit < 1:
            raise ValueError("failure_limit must be at least one")
        if self.lost_timeout_seconds <= 0:
            raise ValueError("lost_timeout_seconds must be positive")
        self._state = BoardTrackState.LOST
        self._corners: np.ndarray | None = None
        self._homography: np.ndarray | None = None
        self._failure_count = 0
        self._last_success_timestamp: float | None = None

    def reset(self) -> BoardTrackSnapshot:
        self._state = BoardTrackState.LOST
        self._corners = None
        self._homography = None
        self._failure_count = 0
        self._last_success_timestamp = None
        return self.snapshot()

    def mark_relocalizing(self) -> BoardTrackSnapshot:
        self._failure_count = 0
        self._state = BoardTrackState.FROZEN if self._corners is not None else BoardTrackState.LOST
        return self.snapshot()

    def set_arm_busy(self, busy: bool) -> BoardTrackSnapshot:
        if busy and self._corners is not None:
            self._state = BoardTrackState.FROZEN
        return self.snapshot()

    def update(
        self,
        localization: LocalizationUpdate | None,
        *,
        detection_performed: bool,
        arm_busy: bool,
        timestamp: float | None = None,
    ) -> BoardTrackSnapshot:
        observed_at = time.monotonic() if timestamp is None else float(timestamp)
        if arm_busy:
            if self._corners is not None:
                self._state = BoardTrackState.FROZEN
            return self.snapshot()
        if not detection_performed:
            return self.snapshot()

        raw_valid = bool(localization is not None and localization.raw_result.valid)
        formal = None if localization is None else localization.formal_result
        candidate = None
        if raw_valid and formal is not None and formal.valid and formal.board_corners_image is not None:
            candidate = np.asarray(formal.board_corners_image, dtype=np.float32).reshape(4, 2)

        if candidate is not None and np.all(np.isfinite(candidate)):
            if self._corners is None:
                self._corners = candidate.copy()
            else:
                alpha = self.smoothing_alpha
                self._corners = (1.0 - alpha) * self._corners + alpha * candidate
            self._homography = _homography_from_ordered_corners(self._corners)
            self._failure_count = 0
            self._last_success_timestamp = observed_at
            self._state = BoardTrackState.LOCKED
            return self.snapshot()

        # A geometrically valid raw candidate that is still passing temporal
        # stabilization is not a localization failure. Preserve the last pose.
        if raw_valid and self._corners is not None:
            self._state = BoardTrackState.FROZEN
            return self.snapshot()

        self._failure_count += 1
        elapsed = (
            float("inf")
            if self._last_success_timestamp is None
            else observed_at - self._last_success_timestamp
        )
        if (
            self._corners is not None
            and self._failure_count < self.failure_limit
            and elapsed <= self.lost_timeout_seconds
        ):
            self._state = BoardTrackState.FROZEN
        else:
            self._state = BoardTrackState.LOST
            self._corners = None
            self._homography = None
        return self.snapshot()

    def snapshot(self) -> BoardTrackSnapshot:
        return BoardTrackSnapshot(
            state=self._state,
            smoothed_corners=None if self._corners is None else self._corners.copy(),
            homography=None if self._homography is None else self._homography.copy(),
            failure_count=self._failure_count,
            last_success_timestamp=self._last_success_timestamp,
        )


def _homography_from_ordered_corners(corners: np.ndarray) -> np.ndarray:
    normalized = np.array([[0, 0], [1, 0], [1, 1], [0, 1]], dtype=np.float32)
    return cv2.getPerspectiveTransform(normalized, np.asarray(corners, dtype=np.float32))
