from __future__ import annotations

import logging
import os
import threading
import time

import cv2
import numpy as np
from PySide6.QtCore import QThread, Signal

from app.config import AppConfig

from .board_locator import BoardLocator
from .camera_selector import resolve_preferred_camera_id
from .overlay import draw_runtime_status, make_test_pattern


LOGGER = logging.getLogger(__name__)


class CameraWorker(QThread):
    """Owns camera acquisition and AprilTag processing outside the GUI thread."""

    frame_ready = Signal(object, float)
    camera_status = Signal(str)
    board_status = Signal(bool, str, bool, str, str)
    board_geometry = Signal(object)
    piece_status = Signal(str, object)
    error = Signal(str)

    def __init__(self, config: AppConfig, *, test_pattern: bool = False, dry_run: bool = False, parent=None):
        super().__init__(parent)
        self.config = config
        self.test_pattern = bool(test_pattern)
        self.dry_run = bool(dry_run)
        self._stop = threading.Event()
        self._relocalize = threading.Event()
        self._recognize_pieces = threading.Event()
        self._state_lock = threading.Lock()
        self._arm_busy = False
        self._show_corners = True
        self._show_corner_coordinates = False
        self._selected_row: int | None = None
        self._selected_col: int | None = None
        self._capture = None

    def stop(self) -> None:
        self._stop.set()

    def request_relocalize(self) -> None:
        self._relocalize.set()

    def request_piece_recognition(self) -> None:
        self._recognize_pieces.set()

    def set_arm_busy(self, busy: bool) -> None:
        with self._state_lock:
            self._arm_busy = bool(busy)

    def set_corner_overlay_options(self, show_corners: bool, show_coordinates: bool) -> None:
        with self._state_lock:
            self._show_corners = bool(show_corners)
            self._show_corner_coordinates = bool(show_coordinates)

    def set_selected_target(self, row: int | None, col: int | None) -> None:
        with self._state_lock:
            self._selected_row = None if row is None else int(row)
            self._selected_col = None if col is None else int(col)

    def run(self) -> None:
        locator = BoardLocator(
            layout_path=self.config.vision.layout_path,
            intrinsics_path=self.config.vision.intrinsics_path,
            intrinsics_fallback_path=self.config.vision.intrinsics_fallback_path,
            detection_interval_frames=self.config.vision.detection_interval_frames,
            board_size=self.config.vision.board_size,
            target_row=self.config.vision.target_row,
            target_col=self.config.vision.target_col,
            target_name=self.config.vision.target_name,
            piece_recognition_config=self.config.vision.piece_recognition,
            corner_smoothing_alpha=self.config.vision.tracker.corner_smoothing_alpha,
            tracker_failure_limit=self.config.vision.tracker.consecutive_failure_limit,
            tracker_lost_timeout_seconds=self.config.vision.tracker.lost_timeout_seconds,
        )
        if locator.startup_error:
            self.error.emit(locator.startup_error)
        try:
            if self.test_pattern:
                self.camera_status.emit("TEST PATTERN")
                self._run_frames(locator, None)
            else:
                capture = self._open_camera()
                self._capture = capture
                self.camera_status.emit("CONNECTED")
                self._run_frames(locator, capture)
        except Exception as exc:
            LOGGER.exception("CAMERA ERROR")
            self.error.emit(str(exc))
        finally:
            capture = self._capture
            self._capture = None
            if capture is not None:
                capture.release()
            self.camera_status.emit("DISCONNECTED")
            self.board_status.emit(False, "BOARD LOST", False, "BOARD LOST", "0/4 LOST")

    def _open_camera(self):
        camera_id = resolve_preferred_camera_id(
            self.config.camera.preferred_name,
            fallback=self.config.camera.index,
        )
        backend = cv2.CAP_DSHOW if os.name == "nt" else cv2.CAP_ANY
        capture = cv2.VideoCapture(camera_id, backend)
        if not capture.isOpened():
            capture.release()
            raise RuntimeError(f"Cannot open camera index {camera_id}")
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.camera.width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.camera.height)
        capture.set(cv2.CAP_PROP_FPS, self.config.camera.fps)
        LOGGER.info("CAMERA CONNECTED index=%d", camera_id)
        return capture

    def _run_frames(self, locator: BoardLocator, capture) -> None:
        frame_number = 0
        sample_started = time.monotonic()
        sample_frames = 0
        fps = 0.0
        consecutive_failures = 0
        last_board = None
        while not self._stop.is_set():
            if self._relocalize.is_set():
                self._relocalize.clear()
                locator.reset()
                LOGGER.info("APRILTAG RELOCALIZATION REQUESTED")

            with self._state_lock:
                arm_busy = self._arm_busy
                show_corners = self._show_corners
                show_corner_coordinates = self._show_corner_coordinates
                selected_row = self._selected_row
                selected_col = self._selected_col
            locator.set_arm_busy(arm_busy)
            locator.set_overlay_options(
                show_corners=show_corners,
                show_coordinates=show_corner_coordinates,
            )
            locator.set_selected_target(selected_row, selected_col)

            if capture is None:
                frame = make_test_pattern(
                    self.config.camera.width,
                    self.config.camera.height,
                    frame_number,
                )
                time.sleep(1.0 / max(1, self.config.camera.fps))
            else:
                ok, frame = capture.read()
                if not ok or frame is None:
                    consecutive_failures += 1
                    if consecutive_failures >= 30:
                        raise RuntimeError("Camera stopped returning frames")
                    time.sleep(0.02)
                    continue
                consecutive_failures = 0

            frame_number += 1
            sample_frames += 1
            now = time.monotonic()
            elapsed = now - sample_started
            if elapsed >= 1.0:
                fps = sample_frames / elapsed
                sample_started = now
                sample_frames = 0

            observation = locator.process(
                np.ascontiguousarray(frame),
                recognize_pieces=self._recognize_pieces.is_set(),
            )
            if observation.piece_result is not None:
                self._recognize_pieces.clear()
                self.piece_status.emit(
                    observation.piece_result.summary,
                    observation.piece_result.board_matrix,
                )
            annotated = draw_runtime_status(
                observation.image,
                fps=fps,
                board_status=observation.board_display_status,
                reason=observation.reason,
                dry_run=self.dry_run,
            )
            board_key = (
                observation.board_locked,
                observation.reason,
                observation.target_visible,
                observation.board_display_status,
                observation.corner_status,
            )
            if board_key != last_board:
                self.board_status.emit(*board_key)
                last_board = board_key
            if observation.homography is not None and observation.corners is not None:
                self.board_geometry.emit(
                    {
                        "homography": observation.homography.copy(),
                        "corners": observation.corners.copy(),
                        "board_locked": bool(observation.board_locked),
                        "track_state": observation.track_state.value,
                    }
                )
            self.frame_ready.emit(annotated, fps)
