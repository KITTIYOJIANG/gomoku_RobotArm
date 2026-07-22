from types import SimpleNamespace

import cv2
import numpy as np

from app.config import AppConfig
from app.vision.board_locator import BoardLocator
from app.vision.board_tracker import BoardTrackState, BoardTracker, CORNER_LABELS
from app.vision.localization.models import (
    LocalizationResult,
    LocalizationStatus,
    LocalizationUpdate,
)
from app.vision.overlay import draw_board_overlay


CORNERS = np.array([[30, 30], [270, 30], [270, 270], [30, 270]], dtype=np.float32)


def localized_update(corners=CORNERS) -> LocalizationUpdate:
    result = LocalizationResult(
        status=LocalizationStatus.BOARD_LOCALIZED,
        detections=[],
        board_corners_image=np.asarray(corners, dtype=np.float64),
        h_image_to_board=np.eye(3),
        h_board_to_image=np.eye(3),
        localization_confidence=1.0,
    )
    return LocalizationUpdate(
        raw_result=result,
        tag_status=LocalizationStatus.TAG_FULL,
        formal_result=result,
        stable_frame_count=3,
        required_stable_frames=3,
        committed=True,
        used_last_valid_localization=False,
        arm_busy=False,
        recognition_allowed=True,
    )


def lost_update() -> LocalizationUpdate:
    result = LocalizationResult(
        status=LocalizationStatus.TAG_LOST,
        detections=[],
        error_code="TAG_LOST",
    )
    return LocalizationUpdate(
        raw_result=result,
        tag_status=LocalizationStatus.TAG_LOST,
        formal_result=None,
        stable_frame_count=0,
        required_stable_frames=3,
        committed=False,
        used_last_valid_localization=False,
        arm_busy=False,
        recognition_allowed=False,
    )


def test_tracker_keeps_ordered_corners_on_skipped_frames_and_arm_freeze():
    tracker = BoardTracker(smoothing_alpha=0.5, failure_limit=3, lost_timeout_seconds=10.0)
    locked = tracker.update(
        localized_update(),
        detection_performed=True,
        arm_busy=False,
        timestamp=1.0,
    )
    assert CORNER_LABELS == ("TL", "TR", "BR", "BL")
    assert locked.state == BoardTrackState.LOCKED
    assert locked.corner_status == "4/4 LOCKED"
    skipped = tracker.update(None, detection_performed=False, arm_busy=False, timestamp=1.1)
    assert skipped.state == BoardTrackState.LOCKED
    np.testing.assert_allclose(skipped.smoothed_corners, locked.smoothed_corners)
    frozen = tracker.update(None, detection_performed=False, arm_busy=True, timestamp=1.2)
    assert frozen.state == BoardTrackState.FROZEN
    assert frozen.corner_status == "4/4 FROZEN"
    np.testing.assert_allclose(frozen.smoothed_corners, locked.smoothed_corners)


def test_tracker_smooths_relock_and_only_loses_after_failure_limit():
    tracker = BoardTracker(smoothing_alpha=0.5, failure_limit=3, lost_timeout_seconds=10.0)
    first = tracker.update(localized_update(), detection_performed=True, arm_busy=False, timestamp=1.0)
    moved = CORNERS + np.array([10.0, 4.0], dtype=np.float32)
    second = tracker.update(localized_update(moved), detection_performed=True, arm_busy=False, timestamp=1.1)
    np.testing.assert_allclose(second.smoothed_corners, CORNERS + np.array([5.0, 2.0]))
    assert tracker.update(lost_update(), detection_performed=True, arm_busy=False, timestamp=1.2).state == BoardTrackState.FROZEN
    assert tracker.update(lost_update(), detection_performed=True, arm_busy=False, timestamp=1.3).state == BoardTrackState.FROZEN
    lost = tracker.update(lost_update(), detection_performed=True, arm_busy=False, timestamp=1.4)
    assert lost.state == BoardTrackState.LOST
    assert lost.corner_status == "0/4 LOST"


def test_red_corner_overlay_is_display_only_and_uses_expected_bgr_points():
    source = np.zeros((300, 300, 3), dtype=np.uint8)
    pristine = source.copy()
    normalized = np.array([[0, 0], [1, 0], [1, 1], [0, 1]], dtype=np.float32)
    homography = cv2.getPerspectiveTransform(normalized, CORNERS)
    display = draw_board_overlay(
        source,
        corners=CORNERS,
        homography=homography,
        track_state=BoardTrackState.FROZEN,
        board_size=15,
        show_corners=True,
        show_corner_coordinates=True,
    )
    np.testing.assert_array_equal(source, pristine)
    assert not np.array_equal(display, pristine)
    for x, y in CORNERS.astype(int):
        np.testing.assert_array_equal(display[y, x], np.array([0, 0, 255], dtype=np.uint8))


class FakePipeline:
    def __init__(self, update):
        self.update = update
        self.frames = []
        self.arm_busy = False

    def process_frame(self, frame):
        self.frames.append(frame.copy())
        return SimpleNamespace(localization=self.update)

    def set_arm_busy(self, busy):
        self.arm_busy = bool(busy)


def test_board_locator_uses_pristine_detection_frame_and_persists_overlay_between_detections():
    config = AppConfig.load()
    locator = BoardLocator(
        layout_path=config.vision.layout_path,
        intrinsics_path=config.vision.intrinsics_path,
        intrinsics_fallback_path=config.vision.intrinsics_fallback_path,
        piece_recognition_config=config.vision.piece_recognition,
        detection_interval_frames=4,
        board_size=config.vision.board_size,
        target_row=7,
        target_col=7,
        corner_smoothing_alpha=1.0,
        tracker_failure_limit=3,
        tracker_lost_timeout_seconds=10.0,
    )
    fake = FakePipeline(localized_update())
    locator._pipeline = fake
    raw = np.zeros((300, 300, 3), dtype=np.uint8)
    first = locator.process(raw, recognize_pieces=True)
    assert first.track_state == BoardTrackState.LOCKED
    assert first.piece_result is not None
    assert first.piece_result.empty_count == 225
    assert len(fake.frames) == 1
    np.testing.assert_array_equal(fake.frames[0], raw)
    np.testing.assert_array_equal(raw, np.zeros_like(raw))
    second = locator.process(raw)
    assert len(fake.frames) == 1  # frame 2 is intentionally not an AprilTag frame
    assert second.track_state == BoardTrackState.LOCKED
    for x, y in CORNERS.astype(int):
        np.testing.assert_array_equal(second.image[y, x], np.array([0, 0, 255], dtype=np.uint8))
    locator.set_arm_busy(True)
    frozen = locator.process(raw)
    assert frozen.track_state == BoardTrackState.FROZEN
    assert frozen.corner_status == "4/4 FROZEN"
    for x, y in CORNERS.astype(int):
        np.testing.assert_array_equal(frozen.image[y, x], np.array([0, 0, 255], dtype=np.uint8))
    locator.set_overlay_options(show_corners=False, show_coordinates=False)
    hidden = locator.process(raw)
    for x, y in CORNERS.astype(int):
        assert not np.array_equal(hidden.image[y, x], np.array([0, 0, 255], dtype=np.uint8))
