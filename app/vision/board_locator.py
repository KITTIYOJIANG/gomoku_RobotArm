from __future__ import annotations

from dataclasses import dataclass
import logging
from pathlib import Path

import numpy as np

from app.config import PieceRecognitionConfig

from .board_tracker import BoardTrackSnapshot, BoardTrackState, BoardTracker
from .calibration import load_camera_intrinsics
from .localization import AprilTagBoardLayout, BoardLocalizationPipeline
from .localization.models import LocalizationUpdate
from .overlay import draw_board_overlay
from .piece_recognizer import PieceRecognitionResult, PieceRecognizer


LOGGER = logging.getLogger(__name__)


@dataclass
class BoardObservation:
    image: np.ndarray
    board_locked: bool
    target_visible: bool
    reason: str
    track_state: BoardTrackState
    corner_status: str
    corners: np.ndarray | None = None
    homography: np.ndarray | None = None
    update: LocalizationUpdate | None = None
    piece_result: PieceRecognitionResult | None = None

    @property
    def board_display_status(self) -> str:
        if self.track_state == BoardTrackState.LOCKED:
            return "BOARD LOCKED"
        if self.track_state == BoardTrackState.FROZEN:
            return "BOARD LOCKED · FROZEN"
        return "BOARD LOST"


class BoardLocator:
    """Runs detection on pristine pixels and overlays one stable tracked pose."""

    def __init__(
        self,
        *,
        layout_path: Path,
        intrinsics_path: Path,
        intrinsics_fallback_path: Path,
        piece_recognition_config: PieceRecognitionConfig,
        detection_interval_frames: int = 4,
        board_size: int = 15,
        target_row: int = 7,
        target_col: int = 7,
        target_name: str = "P77",
        corner_smoothing_alpha: float = 0.35,
        tracker_failure_limit: int = 5,
        tracker_lost_timeout_seconds: float = 2.0,
    ) -> None:
        self.layout_path = Path(layout_path)
        self.intrinsics_path = Path(intrinsics_path)
        self.intrinsics_fallback_path = Path(intrinsics_fallback_path)
        self.detection_interval_frames = max(1, int(detection_interval_frames))
        self.board_size = int(board_size)
        self.target_row = int(target_row)
        self.target_col = int(target_col)
        self.target_name = str(target_name)
        self._frame_count = 0
        self._force_next = True
        self._last_update: LocalizationUpdate | None = None
        self._pipeline: BoardLocalizationPipeline | None = None
        self._startup_error: str | None = None
        self._intrinsics_warning: str | None = None
        self._arm_busy = False
        self._show_corners = True
        self._show_corner_coordinates = False
        self._selected_row: int | None = None
        self._selected_col: int | None = None
        self.tracker = BoardTracker(
            smoothing_alpha=corner_smoothing_alpha,
            failure_limit=tracker_failure_limit,
            lost_timeout_seconds=tracker_lost_timeout_seconds,
        )
        self.piece_recognizer = PieceRecognizer(piece_recognition_config)
        self._create_pipeline()

    @property
    def startup_error(self) -> str | None:
        return self._startup_error

    def _create_pipeline(self) -> None:
        try:
            layout = AprilTagBoardLayout.from_file(self.layout_path)
            intrinsics, warning = load_camera_intrinsics(
                self.intrinsics_path,
                self.intrinsics_fallback_path,
            )
            self._intrinsics_warning = warning
            self._pipeline = BoardLocalizationPipeline(layout, intrinsics=intrinsics)
            self._startup_error = None
        except Exception as exc:
            self._pipeline = None
            self._startup_error = str(exc)
            LOGGER.error("APRILTAG INITIALIZATION FAILED: %s", exc)

    def reset(self, *, preserve_track: bool = True) -> None:
        self._frame_count = 0
        self._force_next = True
        self._last_update = None
        self._create_pipeline()
        if preserve_track:
            self.tracker.mark_relocalizing()
        else:
            self.tracker.reset()

    def set_arm_busy(self, busy: bool) -> None:
        self._arm_busy = bool(busy)
        if self._pipeline is not None:
            self._pipeline.set_arm_busy(self._arm_busy)
        self.tracker.set_arm_busy(self._arm_busy)

    def set_overlay_options(self, *, show_corners: bool, show_coordinates: bool) -> None:
        self._show_corners = bool(show_corners)
        self._show_corner_coordinates = bool(show_coordinates)

    def set_selected_target(self, row: int | None, col: int | None) -> None:
        self._selected_row = None if row is None else int(row)
        self._selected_col = None if col is None else int(col)

    def process(
        self,
        frame: np.ndarray,
        *,
        recognize_pieces: bool = False,
    ) -> BoardObservation:
        if frame is None or frame.size == 0:
            raise ValueError("Camera frame is empty")
        self._frame_count += 1

        # Detection and display buffers are deliberately independent. No overlay
        # operation can feed red corner pixels back into AprilTag or stone input.
        raw_frame = np.ascontiguousarray(frame)
        detection_frame = raw_frame.copy()
        display_frame = raw_frame.copy()
        should_detect = self._force_next or self._frame_count % self.detection_interval_frames == 0
        detection_error: str | None = None

        if self._pipeline is None:
            track = self.tracker.update(
                None,
                detection_performed=should_detect,
                arm_busy=self._arm_busy,
            )
            return self._observation(
                display_frame,
                track,
                self._startup_error or "APRILTAG UNAVAILABLE",
            )

        if should_detect:
            self._force_next = False
            try:
                result = self._pipeline.process_frame(detection_frame)
                self._last_update = result.localization
            except Exception as exc:
                detection_error = str(exc)
                LOGGER.warning("APRILTAG FRAME FAILED: %s", exc)

        track = self.tracker.update(
            None if detection_error else self._last_update,
            detection_performed=should_detect,
            arm_busy=self._arm_busy,
        )
        if track.displayable:
            assert track.smoothed_corners is not None
            assert track.homography is not None
            display_frame = draw_board_overlay(
                display_frame,
                corners=track.smoothed_corners,
                homography=track.homography,
                track_state=track.state,
                board_size=self.board_size,
                target_row=self.target_row,
                target_col=self.target_col,
                target_name=self.target_name,
                show_corners=self._show_corners,
                show_corner_coordinates=self._show_corner_coordinates,
            )

        piece_result = None
        if recognize_pieces and track.placement_ready and track.homography is not None:
            piece_result = self.piece_recognizer.recognize(
                detection_frame,
                homography=track.homography,
                board_size=self.board_size,
            )
            LOGGER.info("PIECE RECOGNITION %s", piece_result.summary)

        reason = self._reason(track, detection_error)
        observation = self._observation(display_frame, track, reason)
        observation.piece_result = piece_result
        observation.update = self._last_update
        return observation

    def _reason(self, track: BoardTrackSnapshot, detection_error: str | None) -> str:
        if detection_error:
            reason = f"APRILTAG ERROR: {detection_error}"
        elif self._arm_busy and track.displayable:
            reason = "ARM_BUSY_FROZEN"
        elif self._last_update is not None:
            reason = self._last_update.transition_reason or self._last_update.tag_status.value
        else:
            reason = "TAG SEARCHING"
        if self._intrinsics_warning and "INTRINSICS" not in reason:
            reason = f"{reason}; {self._intrinsics_warning}"
        return reason

    @staticmethod
    def _observation(
        image: np.ndarray,
        track: BoardTrackSnapshot,
        reason: str,
    ) -> BoardObservation:
        return BoardObservation(
            image=image,
            board_locked=track.placement_ready,
            target_visible=track.displayable,
            reason=reason,
            track_state=track.state,
            corner_status=track.corner_status,
            corners=track.smoothed_corners,
            homography=track.homography,
        )
