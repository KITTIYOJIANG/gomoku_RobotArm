from __future__ import annotations

import numpy as np

from app.vision.board_tracker import BoardTrackState, BoardTracker


def test_frozen_still_placement_ready() -> None:
    tr = BoardTracker(failure_limit=3, lost_timeout_seconds=4.0)
    corners = np.array([[10, 10], [100, 10], [100, 100], [10, 100]], dtype=np.float32)
    tr._corners = corners.copy()
    tr._homography = np.eye(3)
    tr._state = BoardTrackState.LOCKED
    assert tr.snapshot().placement_ready is True

    tr.set_arm_busy(True)
    snap = tr.snapshot()
    assert snap.state == BoardTrackState.FROZEN
    assert snap.placement_ready is True
    assert snap.display_status.startswith("BOARD LOCKED")

    # While busy, update must not LOST even with no localization
    snap2 = tr.update(None, detection_performed=True, arm_busy=True)
    assert snap2.state == BoardTrackState.FROZEN
    assert snap2.placement_ready is True

    tr.set_arm_busy(False)
    snap3 = tr.snapshot()
    assert snap3.state == BoardTrackState.FROZEN
    assert snap3.placement_ready is True


def test_repeated_not_busy_updates_do_not_restart_relocalization_grace() -> None:
    tr = BoardTracker(failure_limit=3, lost_timeout_seconds=4.0)
    tr._corners = np.array(
        [[10, 10], [100, 10], [100, 100], [10, 100]],
        dtype=np.float32,
    )
    tr._homography = np.eye(3)
    tr._state = BoardTrackState.LOCKED

    tr.set_arm_busy(True)
    tr.set_arm_busy(False)
    first_failure = tr.update(None, detection_performed=True, arm_busy=False)
    assert first_failure.failure_count == 1

    # CameraWorker publishes the current flag every frame. Repeating False is
    # not a new busy->idle edge and must not erase accumulated failures.
    unchanged = tr.set_arm_busy(False)
    assert unchanged.failure_count == 1
    tr.update(None, detection_performed=True, arm_busy=False)
    lost = tr.update(None, detection_performed=True, arm_busy=False)
    assert lost.state == BoardTrackState.LOST
