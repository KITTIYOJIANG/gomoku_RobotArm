from __future__ import annotations

import json
import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication
from PySide6.QtTest import QSignalSpy
import pytest

from app.arm.actions import ActionLibrary
from app.arm.controller import SerialArmController
from app.arm.ordered_motion import action_pwm
from app.arm.sequences import ActionStep
from app.calibration_lite.drop_v1 import LiteDropSequenceBuilder, LiteDropStore
from app.calibration_lite.manual_movel import P77ManualMoveLStore
from app.calibration_lite.observe_pose import build_action
from app.calibration_lite.main import build_parser
from app.calibration_lite.window import CalibrationLiteWindow
from app.config import AppConfig
from app.integrated_v1.golden import GOLDEN_ABOVE, SPATIAL_KEYS
from app.integrated_v1.movel import DropStatus, MoveLPlanner
from app.integrated_v1.points import all_points
from app.integrated_v1.profile import CalibrationProfileManager
from app.integrated_v1.profile import ProfileError
from app.stage6.kinematics import ArmKinematics, KinematicsConfig


@pytest.fixture(scope="module")
def qt_app() -> QApplication:
    return QApplication.instance() or QApplication([])


def _store(tmp_path) -> tuple[LiteDropStore, ActionLibrary]:
    library = ActionLibrary()
    source = CalibrationProfileManager(
        profile_path=tmp_path / "unused_integrated.json",
        library=library,
    )
    source.create_from_stable_baseline(profile_id="lite-test-source")
    above = {
        point.as_tuple(): source.above_pwm(point)
        for point in all_points()
    }
    store = LiteDropStore(tmp_path / "lite_drop.json", library=library)
    store.load_or_initialize(above, source={"test": True})
    return store, library


def _planner(store: LiteDropStore) -> MoveLPlanner:
    config = KinematicsConfig.load()
    assert config.pump_joint_id == 5
    assert set(config.joints) == {0, 1, 2, 3, 4}
    return MoveLPlanner(
        store,
        kinematics=ArmKinematics(config),
        target_descent_mm=25.0,
        step_mm=5.0,
        max_waypoint_joint_delta_pwm=400,
    )


def test_lite_store_preserves_five_golden_above_and_excludes_j5(tmp_path) -> None:
    store, _library = _store(tmp_path)
    for point, golden in GOLDEN_ABOVE.items():
        record = store.above_record(point)
        assert record["final_above_pwm"] == golden.pwm_map
        assert record["protected"] is True
        assert record["verification_level"] == "HARDWARE VERIFIED"
        assert set(record["final_above_pwm"]) == set(SPATIAL_KEYS)
        assert "005" not in record["final_above_pwm"]

    store.save()
    raw = json.loads(store.path.read_text(encoding="utf-8"))
    assert raw["safety"] == {
        "j5_pump_joint": "005",
        "j5_in_kinematics": False,
        "auto_hardware_batch": False,
        "golden_above_mutable": False,
    }


def test_p77_manual_drop_overlay_survives_movel_recalculation(tmp_path) -> None:
    store, library = _store(tmp_path)
    planner = _planner(store)
    planner.generate_point("P77", persist=False)

    manual = P77ManualMoveLStore(
        tmp_path / "p77_manual.json",
        joint_limits=store.joint_limits,
    )
    manual.load_or_initialize()
    manual.confirm_step(0)
    manual.create_next_step(0)
    step_1 = {"J0": 1500, "J1": 1170, "J2": 830, "J3": 1280, "J4": 1500}
    manual.save_step(1, step_1)
    manual.confirm_step(1)
    manual.create_next_step(1)
    step_2 = {"J0": 1500, "J1": 1110, "J2": 790, "J3": 1330, "J4": 1500}
    manual.save_step(2, step_2)
    manual.confirm_step(2)
    manual.create_next_step(2)
    final = {"J0": 1500, "J1": 1050, "J2": 760, "J3": 1370, "J4": 1500}
    manual.save_step(3, final)
    manual.confirm_step(3)
    manual.set_as_drop(3)

    applied = store.apply_p77_manual_movel_calibration(
        manual.calibration_payload()
    )
    expected_final = {f"{joint:03d}": final[f"J{joint}"] for joint in range(5)}
    assert applied["drop_final_pwm"] == expected_final
    assert applied["manual_movel_calibration"]["source"] == "manual_movel_tuning"
    assert applied["manual_movel_calibration"]["hardware_verified"] is False

    store.save()
    reloaded = LiteDropStore(store.path, library=library)
    reloaded.load()
    recalculated = _planner(reloaded).generate_point("P77", persist=False)
    assert recalculated["drop_final_pwm"] == expected_final
    assert recalculated["manual_movel_calibration"]["final_pwm"] == expected_final
    assert recalculated["waypoints"][0]["pwm"] == GOLDEN_ABOVE[(7, 7)].pwm_map
    assert reloaded.above_pwm("P77") == GOLDEN_ABOVE[(7, 7)].pwm_map


def test_movel_uses_cartesian_z_waypoints_continuous_ik_and_exact_reverse(tmp_path) -> None:
    store, _library = _store(tmp_path)
    planner = _planner(store)
    record = planner.generate_point("P07_07", persist=False)
    assert record["status"] == DropStatus.PENDING_VERIFY.value
    assert record["verification_level"] == "NOT VERIFIED"
    assert record["generation_method"].endswith("CONTINUOUS_IK_TO_PWM")
    assert [item["descent_mm"] for item in record["waypoints"]] == [
        0.0,
        5.0,
        10.0,
        15.0,
        20.0,
        25.0,
    ]
    poses = [item["cartesian_pose"] for item in record["waypoints"]]
    assert {round(item["x"], 6) for item in poses} == {round(poses[0]["x"], 6)}
    assert {round(item["y"], 6) for item in poses} == {round(poses[0]["y"], 6)}
    assert {round(item["alpha"], 6) for item in poses} == {
        round(poses[0]["alpha"], 6)
    }
    assert record["reverse_ascent_indices"] == [4, 3, 2, 1, 0]
    assert record["waypoints"][0]["pwm"] == GOLDEN_ABOVE[(7, 7)].pwm_map
    for previous, current in zip(record["waypoints"], record["waypoints"][1:]):
        assert max(
            abs(current["pwm"][joint] - previous["pwm"][joint])
            for joint in SPATIAL_KEYS
        ) <= 400
        assert current["pwm"]["004"] == previous["pwm"]["004"]


def test_p77_move_above_uses_j1_last_with_safe_above_staging_dry_run(tmp_path) -> None:
    store, library = _store(tmp_path)
    observe_spatial = [
        action_pwm(library.get("OBSERVE_IDLE"))[joint] for joint in range(5)
    ]
    builder = LiteDropSequenceBuilder(actions=library, store=store)

    golden_before = dict(GOLDEN_ABOVE[(7, 7)].pwm_map)
    sequence = builder.build_move_above("P07_07")

    assert len(sequence.steps) == 3
    assert all(isinstance(step, ActionStep) for step in sequence.steps)
    assert sequence.action_names == (
        "LITE_DROP_MOVE_ABOVE_P77_OFF_00_J1_HELD",
        "LITE_DROP_MOVE_ABOVE_P77_OFF_SAFE_ABOVE",
        "LITE_DROP_MOVE_ABOVE_P77_OFF_00_J1_LAST",
    )
    phase_a = action_pwm(library.get(sequence.action_names[0]))
    safe_above = action_pwm(library.get(sequence.action_names[1]))
    phase_b = action_pwm(library.get(sequence.action_names[2]))
    assert [phase_a[joint] for joint in range(5)] == [
        1500,
        observe_spatial[1],
        870,
        1230,
        1500,
    ]
    assert [safe_above[joint] for joint in range(5)] == [1500, 1260, 870, 1230, 1500]
    assert [phase_b[joint] for joint in range(5)] == [1500, 1230, 870, 1230, 1500]
    assert phase_a[1] == observe_spatial[1]
    assert all(phase_a[joint] == phase_b[joint] for joint in (0, 2, 3, 4))
    assert [joint for joint in range(5) if phase_a[joint] != phase_b[joint]] == [1]
    assert safe_above[1] == phase_b[1] + 30
    assert all(safe_above[joint] == phase_b[joint] for joint in (0, 2, 3, 4))
    assert phase_a[5] == safe_above[5] == phase_b[5] == 1500
    assert all(
        target.time_ms == 1000
        for action_name in sequence.action_names
        for target in library.get(action_name).targets
    )

    hold_sequence = builder.move_observation_to_above("P07_07", pump_state="HOLD")
    assert len(hold_sequence.action_names) == 3
    hold_phase_a = action_pwm(library.get(hold_sequence.action_names[0]))
    hold_safe_above = action_pwm(library.get(hold_sequence.action_names[1]))
    hold_phase_b = action_pwm(library.get(hold_sequence.action_names[2]))
    assert [hold_phase_a[joint] for joint in range(5)] == [
        1500,
        observe_spatial[1],
        870,
        1230,
        1500,
    ]
    assert [hold_safe_above[joint] for joint in range(5)] == [
        1500,
        1260,
        870,
        1230,
        1500,
    ]
    assert [hold_phase_b[joint] for joint in range(5)] == [
        1500,
        1230,
        870,
        1230,
        1500,
    ]
    assert hold_phase_a[5] == hold_safe_above[5] == hold_phase_b[5] == 2500
    with pytest.raises(ProfileError, match="OFF or HOLD"):
        builder.move_observation_to_above("P07_07", pump_state="UNKNOWN")

    controller = SerialArmController(dry_run=True)
    controller.connect("P77_MOVE_ABOVE_DRY_RUN")
    for step in sequence.steps:
        controller.send_action(library.get(step.action_name))
    assert [name for name, _command in controller.dry_run_commands] == list(
        sequence.action_names
    )
    print("PHASE_A_J1_HELD")
    print("target=[" + ",".join(str(phase_a[joint]) for joint in range(5)) + "]")
    print("SAFE_ABOVE_J1_PLUS_30")
    print("target=[" + ",".join(str(safe_above[joint]) for joint in range(5)) + "]")
    print("PHASE_B_J1_FINAL")
    print("target=[" + ",".join(str(phase_b[joint]) for joint in range(5)) + "]")
    print("AT_ABOVE")
    for name, command in controller.dry_run_commands:
        print(f"P77_DRY_RUN {name} {command}")
    assert GOLDEN_ABOVE[(7, 7)].pwm_map == golden_before
    assert store.above_pwm("P07_07") == golden_before


def test_non_golden_move_above_is_exactly_j1_held_then_j1_last(tmp_path) -> None:
    store, library = _store(tmp_path)
    builder = LiteDropSequenceBuilder(actions=library, store=store)

    sequence = builder.build_move_above("P07_08")

    assert sequence.action_names == (
        "LITE_DROP_MOVE_ABOVE_P78_OFF_00_J1_HELD",
        "LITE_DROP_MOVE_ABOVE_P78_OFF_00_J1_LAST",
    )
    phase_a = action_pwm(library.get(sequence.action_names[0]))
    phase_b = action_pwm(library.get(sequence.action_names[1]))
    observe = action_pwm(library.get("OBSERVE_IDLE"))
    assert phase_a[1] == observe[1]
    assert all(phase_a[joint] == phase_b[joint] for joint in (0, 2, 3, 4))
    assert [joint for joint in range(5) if phase_a[joint] != phase_b[joint]] == [1]
    assert phase_a[5] == phase_b[5] == observe[5]


def test_p77_safe_return_from_above_moves_j1_first(tmp_path) -> None:
    store, library = _store(tmp_path)
    builder = LiteDropSequenceBuilder(actions=library, store=store)

    golden = {int(key): int(value) for key, value in GOLDEN_ABOVE[(7, 7)].pwm_map.items()}
    sequence = builder.build_safe_return_from_above("P07_07")

    names = sequence.action_names
    assert names == (
        "LITE_DROP_SAFE_RETURN_P77_00_J1_FIRST",
        "LITE_DROP_SAFE_RETURN_P77_00_J1_HELD",
    )
    observe = action_pwm(library.get("OBSERVE_IDLE"))

    j1_first_observe = action_pwm(library.get(names[0]))
    assert j1_first_observe[1] == observe[1]
    assert all(
        j1_first_observe[joint] == golden[joint] for joint in (0, 2, 3, 4)
    )
    assert action_pwm(library.get(names[1])) == observe

    assert all(library.get(name).target(5).pwm == 1500 for name in names)
    assert all(
        target.time_ms == 1500
        for name in names
        for target in library.get(name).targets
    )

    hold_sequence = builder.build_safe_return_from_above(
        "P07_07",
        pump_state="HOLD",
    )
    assert hold_sequence.action_names == (
        "LITE_DROP_SAFE_RETURN_P77_HOLD_00_J1_FIRST",
        "LITE_DROP_SAFE_RETURN_P77_HOLD_00_J1_HELD",
    )
    assert all(
        library.get(name).target(5).pwm == 2500
        for name in hold_sequence.action_names
    )
    with pytest.raises(ProfileError, match="OFF or HOLD"):
        builder.build_safe_return_from_above("P07_07", pump_state="UNKNOWN")


def test_p77_safe_return_from_drop_reverses_to_above_then_moves_j1_first(
    tmp_path,
) -> None:
    store, library = _store(tmp_path)
    record = _planner(store).generate_point("P07_07", persist=False)
    builder = LiteDropSequenceBuilder(actions=library, store=store)

    reverse = builder.build_retract("P07_07").action_names
    sequence = builder.build_safe_return_from_drop("P07_07")

    assert sequence.action_names[:-2] == reverse
    assert sequence.action_names[-2:] == (
        "LITE_DROP_SAFE_RETURN_P77_00_J1_FIRST",
        "LITE_DROP_SAFE_RETURN_P77_00_J1_HELD",
    )
    assert sequence.action_names[len(reverse) - 1].endswith("_WP_00_IDLE")
    above = {int(key): int(value) for key, value in record["above_pwm"].items()}
    observe = action_pwm(library.get("OBSERVE_IDLE"))
    phase_a = action_pwm(library.get(sequence.action_names[-2]))
    phase_b = action_pwm(library.get(sequence.action_names[-1]))
    assert phase_a[1] == observe[1]
    assert all(phase_a[joint] == above[joint] for joint in (0, 2, 3, 4))
    assert phase_b == observe
    assert all(library.get(name).target(5).pwm == 1500 for name in sequence.action_names)
    assert all(
        target.time_ms == 1500
        for name in sequence.action_names[-2:]
        for target in library.get(name).targets
    )

    controller = SerialArmController(dry_run=True)
    controller.connect("P77_DROP_RETURN_DRY_RUN")
    for step in sequence.steps:
        controller.send_action(library.get(step.action_name))
    assert [name for name, _command in controller.dry_run_commands] == list(
        sequence.action_names
    )
    for name, command in controller.dry_run_commands:
        print(f"P77_DRY_RUN {name} {command}")


def test_unreachable_waypoint_is_blocked_before_motion(tmp_path) -> None:
    store, _library = _store(tmp_path)
    planner = MoveLPlanner(
        store,
        kinematics=ArmKinematics(KinematicsConfig.load()),
        target_descent_mm=25.0,
        step_mm=5.0,
        max_waypoint_joint_delta_pwm=1,
    )
    record = planner.generate_point("P07_07", persist=False)
    assert record["status"] == DropStatus.MOVE_L_UNREACHABLE.value
    assert record["drop_final_pwm"] is None
    assert record["verification_level"] == "NOT VERIFIED"
    assert record["failed_waypoint"]["index"] >= 1


def test_auto_drop_correction_and_verification_are_separate(tmp_path) -> None:
    store, _library = _store(tmp_path)
    planner = _planner(store)
    generated = planner.generate_point("P07_07", persist=False)
    auto = dict(generated["drop_auto_pwm"])
    correction = {joint: 0 for joint in SPATIAL_KEYS}
    correction["001"] = -7
    saved = store.save_correction("P07_07", correction)
    assert saved["drop_auto_pwm"] == auto
    assert saved["drop_correction_pwm"] == correction
    assert saved["drop_final_pwm"]["001"] == auto["001"] - 7
    assert saved["verification_level"] == "NOT VERIFIED"
    assert saved["verified"] is False

    verified = store.mark_verified("P07_07", hardware_confirmed=False)
    assert verified["verification_level"] == "OFFLINE VERIFIED"
    assert verified["status"] == DropStatus.VERIFIED.value


def test_sequences_lock_j5_and_test_place_retracts_exact_path(tmp_path) -> None:
    store, library = _store(tmp_path)
    planner = _planner(store)
    record = planner.generate_point("P07_07", persist=False)
    builder = LiteDropSequenceBuilder(actions=library, store=store)

    too_high = 2451 - int(record["drop_auto_pwm"]["000"])
    with pytest.raises(ProfileError, match="outside 550..2450"):
        builder.apply_joint_target("P07_07", 0, too_high)
    with pytest.raises(ProfileError, match="J0..J4"):
        builder.apply_joint_target("P07_07", 5, 0)

    move = builder.build_move_drop("P07_07")
    retract = builder.build_retract("P07_07")
    waypoint = builder.build_move_waypoint("P07_07", 1)
    partial_retract = builder.build_retract_from_waypoint("P07_07", 3)
    test_place = builder.build_test_place("P07_07")
    assert waypoint.action_names == ("LITE_DROP_P07_07_WP_01_IDLE",)
    assert [int(name.split("_WP_")[1][:2]) for name in partial_retract.action_names] == [
        2,
        1,
        0,
    ]
    assert len(move.action_names) == len(record["waypoints"])
    assert retract.action_names[-1].endswith("_WP_00_IDLE")
    assert [int(name.split("_WP_")[1][:2]) for name in retract.action_names] == [
        4,
        3,
        2,
        1,
        0,
    ]
    names = test_place.action_names
    assert names.index("SOURCE_TOUCH_HOLD") < names.index("CARRY_HIGH_P77_HOLD")
    release_index = next(i for i, name in enumerate(names) if name.endswith("DROP_FINAL_RELEASE"))
    assert names[-1].endswith("_WP_00_IDLE")
    for index, name in enumerate(names):
        pump = library.get(name).target(5).pwm
        if index < release_index and name != "SOURCE_TOUCH_IDLE":
            assert pump == 2500
        elif index >= release_index:
            assert pump == 1500
    assert not hasattr(planner, "controller")
    assert not hasattr(builder, "controller")


def test_previous_waypoint_leaves_corrected_drop_through_auto_final(tmp_path) -> None:
    store, library = _store(tmp_path)
    record = _planner(store).generate_point("P07_07", persist=False)
    correction = {joint: 0 for joint in SPATIAL_KEYS}
    correction["001"] = -7
    store.save_correction("P07_07", correction)
    builder = LiteDropSequenceBuilder(actions=library, store=store)

    last = len(record["waypoints"]) - 1
    sequence = builder.build_move_waypoint(
        "P07_07",
        last - 1,
        from_final_correction=True,
    )
    assert [int(name.split("_WP_")[1][:2]) for name in sequence.action_names] == [
        last,
        last - 1,
    ]


def test_wizard_reveals_only_current_step_and_enforces_place_gate(
    qt_app: QApplication,
    tmp_path,
) -> None:
    from app.calibration_lite.drop_v1_view import LiteDropV1Panel

    store, _library = _store(tmp_path)
    record = _planner(store).generate_point("P07_07", persist=False)
    panel = LiteDropV1Panel()
    try:
        panel.set_record(store.above_record("P07_07"), record)
        assert not panel.pwm_section.isChecked()
        assert not panel.advanced_section.isChecked()

        panel.set_workflow_state(step=2, completed_steps={1}, waypoint_count=6)
        panel.set_motion_state(
            connected=True,
            busy=False,
            estop=False,
            pose_state="SAFE",
            verification_eligible=False,
            dry_run=False,
            wizard_step=2,
            waypoint_count=6,
        )
        assert panel.workflow_pages.currentIndex() == 1
        assert panel.move_above_button.isEnabled()
        assert not panel.above_confirm_button.isEnabled()
        assert not panel.test_place_button.isEnabled()

        panel.set_motion_state(
            connected=True,
            busy=False,
            estop=False,
            pose_state="ABOVE",
            verification_eligible=False,
            dry_run=False,
            wizard_step=2,
            waypoint_count=6,
        )
        assert panel.above_confirm_button.isEnabled()
        assert panel.safe_return_button.isEnabled()

        panel.set_workflow_state(step=3, completed_steps={1, 2}, waypoint_count=6)
        panel.set_motion_state(
            connected=True,
            busy=False,
            estop=False,
            pose_state="ABOVE",
            verification_eligible=False,
            dry_run=False,
            wizard_step=3,
            above_confirmed=True,
            waypoint_index=0,
            waypoint_count=6,
        )
        assert panel.next_waypoint_button.isEnabled()
        assert panel.move_drop_button.isEnabled()
        assert not panel.drop_accurate_button.isEnabled()

        correction_spy = QSignalSpy(panel.correction_mode_requested)
        panel.set_motion_state(
            connected=True,
            busy=False,
            estop=False,
            pose_state="DROP",
            verification_eligible=False,
            dry_run=False,
            wizard_step=3,
            above_confirmed=True,
            waypoint_index=5,
            waypoint_count=6,
        )
        assert panel.drop_accurate_button.isEnabled()
        assert panel.open_correction_button.isEnabled()
        panel.open_correction_button.click()
        assert correction_spy.count() == 1
        assert panel.pwm_section.isChecked()

        panel.set_workflow_state(step=5, completed_steps={1, 2, 3, 4})
        panel.set_motion_state(
            connected=True,
            busy=False,
            estop=False,
            pose_state="SAFE",
            verification_eligible=False,
            dry_run=False,
            wizard_step=5,
            above_confirmed=True,
            drop_accuracy_confirmed=True,
            test_place_succeeded=False,
            waypoint_count=6,
        )
        assert panel.test_place_button.isEnabled()
        assert not panel.verify_button.isEnabled()

        panel.set_motion_state(
            connected=True,
            busy=False,
            estop=False,
            pose_state="ABOVE",
            verification_eligible=True,
            dry_run=False,
            wizard_step=5,
            above_confirmed=True,
            drop_accuracy_confirmed=True,
            test_place_succeeded=True,
            waypoint_count=6,
        )
        assert panel.verify_button.isEnabled()
        panel.set_motion_state(
            connected=True,
            busy=True,
            estop=False,
            pose_state="ABOVE",
            verification_eligible=True,
            dry_run=False,
            wizard_step=5,
            test_place_succeeded=True,
            waypoint_count=6,
        )
        assert panel.emergency_button.isEnabled()
    finally:
        panel.close()


def test_generate_225_is_offline_and_records_per_point_failures(tmp_path) -> None:
    store, _library = _store(tmp_path)
    planner = _planner(store)
    summary = planner.generate_all(persist=False)
    assert summary.requested == 225
    assert summary.success + summary.unreachable + summary.invalid + summary.skipped == 225
    assert summary.golden_started == 5
    assert store.data["drop"]["last_generate_all"]["execution"] == (
        "OFFLINE_ONLY_NO_CONTROLLER_REFERENCE"
    )


def test_lite_cli_and_window_support_dry_run_without_auto_connect(
    qt_app: QApplication,
) -> None:
    args = build_parser().parse_args(["--dry-run"])
    assert args.dry_run is True
    window = CalibrationLiteWindow(AppConfig.load(), dry_run=True)
    try:
        assert window.dry_run is True
        assert "DRY RUN" in window.windowTitle()
        assert not window.controller.is_connected
        assert window.drop_store is not None
        assert window.lite.drop_v1_panel.generate_all_button.text().endswith(
            "OFFLINE ONLY"
        )
        assert len(window.lite.drop_v1_panel._corrections) == 5
        panel = window.lite.drop_v1_panel
        panel.set_motion_state(
            connected=False,
            busy=False,
            estop=False,
            pose_state="SAFE",
            verification_eligible=False,
            dry_run=True,
        )
        panel.advanced_section.expand()
        assert panel.generate_all_button.isEnabled()
        assert not panel.save_correction_button.isEnabled()
        panel.set_motion_state(
            connected=True,
            busy=False,
            estop=False,
            pose_state="DROP",
            verification_eligible=False,
            dry_run=True,
            wizard_step=4,
            above_confirmed=True,
            waypoint_index=5,
            waypoint_count=6,
        )
        panel.pwm_section.expand()
        assert panel.save_correction_button.isEnabled()
        assert all(button.isEnabled() for button in panel._correction_apply_buttons)
    finally:
        window.close()
        qt_app.processEvents()


def test_window_refuses_verification_before_test_place(
    qt_app: QApplication,
) -> None:
    window = CalibrationLiteWindow(AppConfig.load(), dry_run=True)
    warnings: list[tuple[str, str]] = []
    try:
        window.controller.connect("DRY-RUN")
        point = "P07_07"
        before = json.dumps(
            window.drop_store.drop_record(point),
            ensure_ascii=False,
            sort_keys=True,
        )
        window._warn = lambda title, message: warnings.append((title, message))
        window._drop_wizard_step = 5
        window._drop_test_place_succeeded = False

        window.verify_drop_v1(point)

        after = json.dumps(
            window.drop_store.drop_record(point),
            ensure_ascii=False,
            sort_keys=True,
        )
        assert after == before
        assert warnings == [
            (
                "Save DROP verification",
                "Confirm verification only after Step 5 Test PLACE succeeds.",
            )
        ]
    finally:
        window.controller.disconnect()
        window.close()
        qt_app.processEvents()


def test_safe_return_from_midflow_resumes_at_above_confirmation(
    qt_app: QApplication,
) -> None:
    window = CalibrationLiteWindow(AppConfig.load(), dry_run=True)
    submitted = []
    try:
        window._submit_drop_v1_sequence = submitted.append
        window._drop_pose_state = "ABOVE"
        window._drop_active_point = "P77"
        window._drop_wizard_step = 3
        window._drop_after_safe_return_step = None

        window.safe_return_drop_v1("P77")

        assert [sequence.name for sequence in submitted] == [
            "MANUAL:LITE_DROP_SAFE_RETURN:P77"
        ]
        assert window._drop_after_safe_return_step == 2
    finally:
        window.close()
        qt_app.processEvents()


def test_safe_return_button_from_drop_submits_reverse_then_j1_first(
    qt_app: QApplication,
) -> None:
    window = CalibrationLiteWindow(AppConfig.load(), dry_run=True)
    submitted = []
    try:
        window._submit_drop_v1_sequence = submitted.append
        window._drop_pose_state = "DROP"
        window._drop_active_point = "P77"
        window._drop_wizard_step = 3
        window._drop_after_safe_return_step = None

        window.safe_return_drop_v1("P77")

        assert len(submitted) == 1
        sequence = submitted[0]
        assert sequence.name == "MANUAL:LITE_DROP_SAFE_RETURN:P77"
        assert sequence.action_names[-2:] == (
            "LITE_DROP_SAFE_RETURN_P77_00_J1_FIRST",
            "LITE_DROP_SAFE_RETURN_P77_00_J1_HELD",
        )
        assert sequence.action_names[-3].endswith("_WP_00_IDLE")
        assert window._drop_after_safe_return_step == 2
    finally:
        window.close()
        qt_app.processEvents()
