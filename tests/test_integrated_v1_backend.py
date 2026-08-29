from __future__ import annotations

import json

import pytest

from app.arm.actions import ActionLibrary
from app.arm.controller import SerialArmController
from app.integrated_v1.golden import GOLDEN_ABOVE, SPATIAL_KEYS, assert_golden_above
from app.integrated_v1.movel import DropStatus, MoveLPlanner
from app.integrated_v1.points import PointRef, format_point_id, parse_point_id
from app.integrated_v1.profile import CalibrationProfileManager, ProfileError
from app.integrated_v1.robot_controller import RobotController, RobotState


EXPECTED_GOLDEN = {
    "P33": [1589, 1136, 1101, 1084, 1500],
    "P311": [1432, 1199, 1157, 1042, 1500],
    "P77": [1500, 1230, 870, 1230, 1500],
    "P113": [1630, 1264, 588, 1424, 1500],
    "P1111": [1382, 1258, 639, 1410, 1500],
}


def make_profile(tmp_path):
    manager = CalibrationProfileManager(profile_path=tmp_path / "profile.json")
    manager.create_from_stable_baseline()
    return manager


def golden_vectors(manager: CalibrationProfileManager) -> dict[str, list[int]]:
    result = {}
    for anchor in GOLDEN_ABOVE.values():
        pwm = manager.above_pwm(anchor.point)
        result[anchor.legacy_id] = [pwm[key] for key in SPATIAL_KEYS]
    return result


def test_point_id_boundary_is_unambiguous():
    assert parse_point_id("P77") == PointRef(7, 7)
    assert parse_point_id("P311") == PointRef(3, 11)
    assert parse_point_id("P113") == PointRef(11, 3)
    assert parse_point_id("P11_1") == PointRef(11, 1)
    assert parse_point_id("P#112") == PointRef(7, 7)
    assert format_point_id(3, 11) == "P3_11"
    with pytest.raises(ValueError, match="ambiguous"):
        parse_point_id("P112")


def test_user_confirmed_golden_overlay_records_all_stage5_conflicts(tmp_path):
    manager = make_profile(tmp_path)
    assert golden_vectors(manager) == EXPECTED_GOLDEN
    assert len(manager.data["golden_anchor_conflicts"]) == 5
    assert_golden_above(manager.data["above"]["points"])


def test_golden_survives_generate_fast_save_reload_and_restart(tmp_path):
    manager = make_profile(tmp_path)
    planner = MoveLPlanner(manager)
    summary = planner.generate_all(persist=False)
    assert summary.requested == 225
    assert summary.golden_started == 5
    anchors = {}
    for coord in GOLDEN_ABOVE:
        pwm = manager.above_pwm(coord)
        pwm["000"] += 20
        anchors[coord] = pwm
    manager.apply_fast_calibration("FAST_5", anchors)
    assert golden_vectors(manager) == EXPECTED_GOLDEN
    planner.generate_all(persist=False)
    assert golden_vectors(manager) == EXPECTED_GOLDEN
    manager.save()

    reloaded = CalibrationProfileManager(profile_path=manager.profile_path)
    reloaded.load()
    assert golden_vectors(reloaded) == EXPECTED_GOLDEN
    assert_golden_above(reloaded.data["above"]["points"])


def test_fast_9_and_direct_anchor_priority_do_not_overwrite_direct(tmp_path):
    manager = make_profile(tmp_path)
    direct = manager.above_pwm((2, 2))
    direct["000"] += 7
    manager.save_direct_anchor((2, 2), direct)
    anchors = {}
    for coord in ((0, 0), (0, 7), (0, 14), (7, 0), (7, 7), (7, 14), (14, 0), (14, 7), (14, 14)):
        pwm = manager.above_pwm(coord)
        pwm["001"] += 5
        anchors[coord] = pwm
    result = manager.apply_fast_calibration("FAST_9", anchors)
    assert result["method"] == "piecewise_bilinear_clamped"
    assert manager.above_pwm((2, 2)) == direct
    assert manager.above_record((2, 2))["source"] == "user_direct_anchor"
    assert golden_vectors(manager) == EXPECTED_GOLDEN


def test_protected_golden_requires_explicit_revalidation_request(tmp_path):
    manager = make_profile(tmp_path)
    requested = manager.above_pwm("P77")
    requested["000"] += 1
    with pytest.raises(ProfileError, match="protected Golden"):
        manager.save_direct_anchor("P77", requested)
    manager.request_golden_override(
        "P77", requested, confirmation_note="operator requests a future bounded revalidation"
    )
    assert manager.above_pwm("P77")["000"] == 1500
    assert manager.data["pending_golden_overrides"]["P77"]["status"] == "PENDING_REVALIDATION"


def test_movel_starts_at_exact_golden_and_uses_cartesian_waypoints(tmp_path):
    manager = make_profile(tmp_path)
    planner = MoveLPlanner(manager, target_descent_mm=25, step_mm=5)
    record = planner.generate_point("P77", persist=False)
    assert record["status"] == DropStatus.PENDING_VERIFY.value
    assert record["waypoints"][0]["pwm"] == GOLDEN_ABOVE[(7, 7)].pwm_map
    assert [item["descent_mm"] for item in record["waypoints"]] == [0, 5, 10, 15, 20, 25]
    above_pose = record["waypoints"][0]["cartesian_pose"]
    for item in record["waypoints"]:
        pose = item["cartesian_pose"]
        assert pose["x"] == pytest.approx(above_pose["x"])
        assert pose["y"] == pytest.approx(above_pose["y"])
        assert pose["alpha"] == pytest.approx(above_pose["alpha"])
    assert record["reverse_ascent_indices"] == [4, 3, 2, 1, 0]


def test_movel_unreachable_saves_failed_waypoint_and_best_safe_pose(tmp_path):
    manager = make_profile(tmp_path)
    planner = MoveLPlanner(manager, max_waypoint_joint_delta_pwm=1)
    record = planner.generate_point("P77", persist=False)
    assert record["status"] == DropStatus.MOVE_L_UNREACHABLE.value
    assert record["failed_waypoint"]["index"] >= 1
    assert record["best_safe_pose"]["status"] == "SUGGESTED_NOT_VERIFIED"
    assert not record["verified"]


def test_drop_correction_composes_without_changing_auto(tmp_path):
    manager = make_profile(tmp_path)
    planner = MoveLPlanner(manager)
    record = planner.generate_point("P77", persist=False)
    auto = dict(record["drop_auto_pwm"])
    correction = {joint: 0 for joint in SPATIAL_KEYS}
    correction["000"] = 1
    corrected = manager.save_drop_correction("P77", correction)
    assert corrected["drop_auto_pwm"] == auto
    assert corrected["drop_final_pwm"]["000"] == auto["000"] + 1
    assert corrected["status"] == DropStatus.MANUAL_CORRECTED.value


def test_profile_startup_route_requires_real_valid_profile(tmp_path):
    manager = make_profile(tmp_path)
    assert manager.status().route == "FIRST_SETUP"
    MoveLPlanner(manager).generate_point("P77", persist=False)
    manager.mark_drop_verified("P77", notes="offline regression only")
    manager.promote_valid()
    assert manager.status().route == "GAME"
    raw = json.loads(manager.profile_path.read_text(encoding="utf-8"))
    raw["above"]["points"]["P77"]["final_above_pwm"]["000"] = 1630
    manager.profile_path.write_text(json.dumps(raw), encoding="utf-8")
    broken = CalibrationProfileManager(profile_path=manager.profile_path)
    with pytest.raises(ProfileError, match="Golden ABOVE changed"):
        broken.load()


def test_valid_profile_exports_new_golden_baseline_without_overwrite(tmp_path):
    manager = make_profile(tmp_path)
    MoveLPlanner(manager).generate_point("P77", persist=False)
    manager.mark_drop_verified("P77")
    manager.promote_valid()
    destination = manager.export_golden_baseline(tmp_path / "baseline")
    payload = json.loads(destination.read_text(encoding="utf-8"))
    assert payload["baseline_kind"] == "GOLDEN_CALIBRATION_BASELINE"
    assert payload["immutable_intent"] is True
    assert_golden_above(payload["above"]["points"])


def test_robot_place_piece_uses_single_controller_and_full_dry_run(tmp_path):
    actions = ActionLibrary()
    manager = CalibrationProfileManager(profile_path=tmp_path / "profile.json", library=actions)
    manager.create_from_stable_baseline()
    planner = MoveLPlanner(manager)
    planner.generate_point("P77", persist=False)
    manager.mark_drop_verified("P77", notes="offline dry-run gate")
    manager.data["valid"] = True
    serial = SerialArmController(dry_run=True)
    serial.connect("V1_DRY_RUN")
    robot = RobotController(
        serial_controller=serial,
        action_library=actions,
        profile=manager,
        planner=planner,
    )
    result = robot.place_piece("P77")
    assert result.accepted
    assert result.reason == "DRY RUN PASS"
    assert robot.state == RobotState.IDLE
    labels = [label for label, _command in serial.dry_run_commands]
    assert labels[:4] == [
        "SOURCE_TOUCH_IDLE",
        "SOURCE_TOUCH_HOLD",
        "OBSERVE_HOLD",
        "CARRY_HIGH_P77_HOLD",
    ]
    assert labels[-2:] == ["CARRY_HIGH_P77_IDLE", "OBSERVE_IDLE"]
    assert "V1_P07_07_DROP_FINAL_RELEASE" in labels
    descent = [label for label in labels if label.endswith("_HOLD") and "_WP_" in label]
    ascent = [label for label in labels if label.endswith("_IDLE") and "_WP_" in label]
    assert [label.replace("_HOLD", "_IDLE") for label in descent[-2::-1]] == ascent


def test_robot_gate_and_estop_are_shared_high_level_boundaries(tmp_path):
    actions = ActionLibrary()
    manager = CalibrationProfileManager(profile_path=tmp_path / "profile.json", library=actions)
    manager.create_from_stable_baseline()
    serial = SerialArmController(dry_run=True)
    robot = RobotController(serial_controller=serial, action_library=actions, profile=manager)
    blocked = robot.place_piece("P77")
    assert not blocked.accepted
    assert "Serial disconnected" in blocked.reason
    serial.connect("V1_DRY_RUN")
    robot.emergency_stop()
    assert robot.state == RobotState.STOPPED
    assert serial.dry_run_commands[-1][0] == "ESTOP"


def test_live_gate_rejects_offline_verified_drop(tmp_path):
    class FakeOpenSerial:
        is_open = True

    actions = ActionLibrary()
    manager = CalibrationProfileManager(profile_path=tmp_path / "profile.json", library=actions)
    manager.create_from_stable_baseline()
    MoveLPlanner(manager).generate_point("P77", persist=False)
    manager.mark_drop_verified("P77", hardware_confirmed=False)
    manager.data["valid"] = True
    serial = SerialArmController(dry_run=False)
    serial.port = "FAKE_READ_ONLY_GATE"
    serial._connection = FakeOpenSerial()
    robot = RobotController(
        serial_controller=serial,
        action_library=actions,
        profile=manager,
        worker=None,
    )
    gate = robot.gate("P77")
    assert not gate.allowed
    assert "HARDWARE VERIFIED" in "; ".join(gate.reasons)


def test_legacy_hardware_successful_p77_sequence_is_unchanged(tmp_path):
    actions = ActionLibrary()
    manager = CalibrationProfileManager(profile_path=tmp_path / "profile.json", library=actions)
    manager.create_from_stable_baseline()
    robot = RobotController(
        serial_controller=SerialArmController(dry_run=True),
        action_library=actions,
        profile=manager,
    )
    assert robot.legacy_p77_full_cycle().action_names == (
        "OBSERVE_IDLE",
        "SOURCE_TOUCH_IDLE",
        "SOURCE_TOUCH_HOLD",
        "OBSERVE_HOLD",
        "CARRY_HIGH_P77_HOLD",
        "P77_ABOVE_HOLD",
        "P77_TOUCH_HOLD",
        "P77_TOUCH_RELEASE",
        "P77_ABOVE_IDLE",
        "CARRY_HIGH_P77_IDLE",
        "OBSERVE_IDLE",
    )


def test_calibration_drop_sequence_keeps_pump_off(tmp_path):
    actions = ActionLibrary()
    manager = CalibrationProfileManager(profile_path=tmp_path / "profile.json", library=actions)
    manager.create_from_stable_baseline()
    MoveLPlanner(manager).generate_point("P77", persist=False)
    robot = RobotController(
        serial_controller=SerialArmController(dry_run=True),
        action_library=actions,
        profile=manager,
    )
    sequence = robot.move_to_drop_for_calibration("P77")
    assert len(sequence.action_names) >= 2
    for name in sequence.action_names:
        assert actions.get(name).target(5).pwm == 1500
