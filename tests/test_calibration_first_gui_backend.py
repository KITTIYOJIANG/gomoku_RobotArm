from __future__ import annotations

import json

from app.arm.actions import ActionLibrary
from app.arm.controller import SerialArmController
from app.stage7.pick_poses import PickPoseStore


def test_apply_all_command_never_touches_pump_or_reserved_outputs() -> None:
    controller = SerialArmController(dry_run=True)
    controller.connect("MOCK_COM")

    command = controller.send_spatial_pose(
        {"000": 1510, "001": 1840, "002": 1260, "003": 1720, "004": 1480},
        time_ms=1000,
    )

    assert command.startswith("{#000P1510T1000!")
    assert command.endswith("#004P1480T1000!}")
    assert "#005" not in command
    assert "#006" not in command
    assert "#007" not in command
    assert controller.dry_run_commands == [("APPLY_SPATIAL_POSE", command)]


def test_pick_pose_candidate_preserves_stable_actions_and_is_unverified(tmp_path) -> None:
    library = ActionLibrary()
    stable_before = library.get("SOURCE_TOUCH_IDLE").command
    path = tmp_path / "pick_poses.json"
    store = PickPoseStore(path, library)
    baseline = store.get("PICK_ABOVE")["baseline_pwm"]
    edited = dict(baseline)
    edited["001"] += 5

    record = store.save_candidate(
        "PICK_ABOVE", edited, calibration_session="offline-test"
    )

    persisted = json.loads(path.read_text(encoding="utf-8"))
    assert record["source"] == "DIRECT_CANDIDATE"
    assert record["verified"] is False
    assert record["delta_pwm"]["001"] == 5
    assert persisted["stable_action_table_unchanged"] is True
    assert library.get("SOURCE_TOUCH_IDLE").command == stable_before
