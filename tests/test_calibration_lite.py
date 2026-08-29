from __future__ import annotations

from dataclasses import replace
import hashlib
import json
import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication, QLabel
from PySide6.QtTest import QSignalSpy
import pytest

from app.calibration_lite.context import load_calibration_summary
from app.calibration_lite.observe_pose import (
    action_pwm,
    action_times,
    build_action,
    load_observe_override,
    save_observe_override,
)
from app.calibration_lite.place_pose import (
    derive_place_contact,
    load_place_override,
    save_place_override,
)
from app.calibration_lite.p77_point import load_p77_point, save_p77_point
from app.calibration_lite.view import CalibrationLiteView
from app.calibration_lite.window import CalibrationLiteWindow
from app.calibration_lite.wizard import ANCHOR_SPECS, LiteWizardState, WizardPhase
from app.config import AppConfig, PROJECT_ROOT
from app.arm.actions import ActionLibrary
from app.arm.controller import SerialArmController
from app.arm.sequences import ActionStep, SequenceDefinition, pick_piece, retry_pick_piece
from app.stage7.settings import Stage7Settings


@pytest.fixture(scope="module")
def qt_app() -> QApplication:
    return QApplication.instance() or QApplication([])


def test_quick_five_wizard_uses_required_order_and_automatic_transitions() -> None:
    wizard = LiteWizardState()
    assert wizard.start() == ANCHOR_SPECS[0]
    visited = [wizard.current_anchor.label]
    while wizard.phase is WizardPhase.ANCHOR:
        next_anchor = wizard.mark_anchor_saved()
        if next_anchor is not None:
            visited.append(next_anchor.label)

    assert visited == ["P77", "P00", "P014", "P140", "P1414"]
    assert wizard.phase is WizardPhase.GENERATE
    assert len(wizard.completed_anchor_labels) == 5

    assert wizard.begin_test().label == "P77"
    tested = [wizard.current_test.label]
    while wizard.phase is WizardPhase.TEST:
        next_test = wizard.mark_test_accurate()
        if next_test is not None:
            tested.append(next_test.label)
    assert tested == visited
    assert wizard.phase is WizardPhase.COMPLETE


def test_power_on_unknown_to_observe_uses_dedicated_five_second_action(
    qt_app: QApplication,
) -> None:
    window = CalibrationLiteWindow(AppConfig.load(), dry_run=True)
    submitted = []
    try:
        regular_observe = window.actions.get("OBSERVE_IDLE")
        regular_duration = regular_observe.duration_ms
        window.controller.connect("POWER_ON_OBSERVE_DRY_RUN")
        window.state_machine.connect()
        window._start_sequence = lambda sequence, begin: submitted.append(
            (sequence, begin)
        )

        window.start_return_to_observe()

        assert len(submitted) == 1
        sequence, begin = submitted[0]
        assert sequence.name == "RETURN_TO_OBSERVE"
        assert sequence.action_names == ("LITE_POWER_ON_TO_OBSERVE_IDLE",)
        action = window.actions.get(sequence.action_names[0])
        assert action.duration_ms == 5000
        assert all(target.time_ms == 5000 for target in action.targets)
        assert action_pwm(action) == action_pwm(regular_observe)
        assert begin == window.state_machine.begin_return_to_observe
        assert window.actions.get("OBSERVE_IDLE").duration_ms == regular_duration

        window.controller.send_action(action)
        assert window.controller.dry_run_commands[-1][0] == action.name
        print("POWER_ON_UPRIGHT_TO_OBSERVE T5000")
        print("AT_OBSERVE")
    finally:
        window.close()
        qt_app.processEvents()


def test_inaccurate_test_returns_only_current_point_to_anchor_correction() -> None:
    wizard = LiteWizardState()
    wizard.start()
    for _ in ANCHOR_SPECS:
        wizard.mark_anchor_saved()
    wizard.begin_test()
    wizard.mark_test_accurate()

    target = wizard.correct_current_test()
    assert target.label == "P00"
    assert wizard.phase is WizardPhase.ANCHOR
    assert wizard.current_anchor == target
    assert wizard.mark_anchor_saved() is None
    assert wizard.phase is WizardPhase.GENERATE


def test_lite_view_hides_advanced_controls_and_locks_pump(qt_app: QApplication) -> None:
    view = CalibrationLiteView(default_port="COM_TEST")
    try:
        assert not view.advanced_group.isChecked()
        assert view.advanced_group.body.isHidden()
        assert not view.pwm_group.isChecked()
        assert view.pwm_group.body.isHidden()
        assert any("J5" in label.text() for label in view.pwm_group.body.findChildren(QLabel))
        assert view.pages.count() == 9
        assert view.quick_button.text() == "快速标定"
        assert view.relocalize_button.text() == "快速重新定位"
        assert not view.relocalize_button.isEnabled()
        pwm_row = view._pwm_rows[0]
        button_texts = [button.text() for button in pwm_row.findChildren(type(view.quick_button))]
        assert "-50" in button_texts
        assert "+50" in button_texts
        assert "-1" not in button_texts
        assert "+1" not in button_texts
        spy = QSignalSpy(view.camera_anchor_pick_requested)
        view.generate_add_anchor.setChecked(True)
        camera_button = view.generate_add_anchor.body.findChildren(type(view.quick_button))[0]
        camera_button.click()
        assert spy.count() == 1
        pump_spy = QSignalSpy(view.pump_toggle_requested)
        view.pump_toggle.click()
        assert pump_spy.count() == 1
        assert pump_spy.at(0)[0] is True
    finally:
        view.close()


def test_summary_ignores_empty_deployment_session_path(tmp_path) -> None:
    baseline = tmp_path / "baseline.json"
    baseline.write_text(json.dumps({"updated_at": "2026-08-25", "anchors": {}}), encoding="utf-8")
    deployment = tmp_path / "deployment.json"
    deployment.write_text(json.dumps({"session_path": "", "points": {}}), encoding="utf-8")
    sessions = tmp_path / "sessions"
    sessions.mkdir()
    settings = replace(
        Stage7Settings.load(),
        baseline_path=baseline,
        sessions_dir=sessions,
        current_deployment_path=deployment,
    )

    summary = load_calibration_summary(settings)
    assert summary.status == "Stable baseline"
    assert summary.session_path is None
    assert summary.generated_points == 225


def test_lite_window_never_auto_connects_hardware(qt_app: QApplication) -> None:
    window = CalibrationLiteWindow(AppConfig.load())
    try:
        assert not window.controller.is_connected
        assert window.camera_worker is None
        assert window.windowTitle() == "Gomoku Robot — Calibration Lite V1"
        assert window.centralWidget() is window.lite
        assert window._legacy_root is not window.centralWidget()
        assert window.stage7.session is not None
        assert len(window.stage7.session.generated_points) == 225
        calibrated_p77 = window._authoritative_point_pwm(7, 7)
        runtime_p77 = action_pwm(window.actions.get("P77_ABOVE_IDLE"))
        assert all(
            runtime_p77[f"{joint:03d}"] == calibrated_p77[f"{joint:03d}"]
            for joint in range(5)
        )
        carry_idle = action_pwm(window.actions.get("CARRY_HIGH_P77_IDLE"))
        carry_hold = action_pwm(window.actions.get("CARRY_HIGH_P77_HOLD"))
        for joint in (0, 2, 3, 4):
            key = f"{joint:03d}"
            assert carry_idle[key] == calibrated_p77[key]
            assert carry_hold[key] == calibrated_p77[key]
        assert carry_idle["001"] == calibrated_p77["001"] + 60
        assert carry_hold["001"] == calibrated_p77["001"] + 60
        assert carry_idle["005"] == 1500
        assert carry_hold["005"] == 2500
        assert window.stage5.planner.carry_lift_001 == 0
        pick_record = window.stage7.pick_poses.get("PICK_DOWN")
        if pick_record.get("updated_at"):
            runtime_pick_idle = action_pwm(window.actions.get("SOURCE_TOUCH_IDLE"))
            runtime_pick = action_pwm(window.actions.get("SOURCE_TOUCH_HOLD"))
            assert all(
                runtime_pick[f"{joint:03d}"]
                == pick_record["new_pwm"][f"{joint:03d}"]
                for joint in range(5)
            )
            assert all(
                runtime_pick_idle[f"{joint:03d}"]
                == runtime_pick[f"{joint:03d}"]
                for joint in range(5)
            )
            ordered_pick = window._prepare_pick_calibration_sequence(
                SequenceDefinition(
                    name="MANUAL:LITE_PICK_CALIBRATE",
                    display_name="test",
                    steps=(ActionStep("SOURCE_TOUCH_IDLE"),),
                )
            )
            assert ordered_pick.action_names == (
                "LITE_PICK_DOWN_00_J1_HELD",
                "LITE_PICK_DOWN_00_J1_LAST",
            )
            pick_held = action_pwm(
                window.actions.get("LITE_PICK_DOWN_00_J1_HELD")
            )
            assert pick_held["001"] == action_pwm(
                window.actions.get("OBSERVE_IDLE")
            )["001"]
            assert action_pwm(
                window.actions.get("LITE_PICK_DOWN_00_J1_LAST")
            ) == runtime_pick_idle
            normal_pick = window._prepare_pick_execution_sequence(pick_piece())
            assert normal_pick.action_names == (
                "LITE_PICK_EXEC_00_J1_HELD",
                "LITE_PICK_EXEC_00_J1_LAST",
                "SOURCE_TOUCH_HOLD",
                "OBSERVE_HOLD",
            )
            retry_pick = window._prepare_pick_execution_sequence(retry_pick_piece())
            assert retry_pick.action_names == (
                "OBSERVE_IDLE",
                "LITE_PICK_EXEC_00_J1_HELD",
                "LITE_PICK_EXEC_00_J1_LAST",
                "SOURCE_TOUCH_HOLD",
                "OBSERVE_HOLD",
            )
            pick_exec_held = action_pwm(
                window.actions.get("LITE_PICK_EXEC_00_J1_HELD")
            )
            assert pick_exec_held["001"] == action_pwm(
                window.actions.get("OBSERVE_IDLE")
            )["001"]
            assert action_pwm(
                window.actions.get("LITE_PICK_EXEC_00_J1_LAST")
            ) == runtime_pick_idle
        observe_idle = action_pwm(window.actions.get("OBSERVE_IDLE"))
        observe_hold = action_pwm(window.actions.get("OBSERVE_HOLD"))
        assert all(
            observe_hold[f"{joint:03d}"] == observe_idle[f"{joint:03d}"]
            for joint in range(5)
        )
        sequence = window._build_lite_stage7_return_sequence(
            row=7,
            col=7,
            parked_pwm=calibrated_p77,
        )
        assert sequence.action_names == (
            "LITE_RETURN_00_J1_FIRST",
            "LITE_RETURN_00_J1_HELD",
        )
        observe_idle = action_pwm(window.actions.get("OBSERVE_IDLE"))
        j1_first_pose = action_pwm(window.actions.get("LITE_RETURN_00_J1_FIRST"))
        held_observe = action_pwm(window.actions.get("LITE_RETURN_00_J1_HELD"))
        assert j1_first_pose["001"] == observe_idle["001"]
        assert all(
            j1_first_pose[key] == calibrated_p77[key]
            for key in ("000", "002", "003", "004")
        )
        assert held_observe == observe_idle
        assert all(
            target.time_ms == 1500
            for name in sequence.action_names
            for target in window.actions.get(name).targets
        )
        move_sequence = window._prepare_stage7_above_sequence(
            SequenceDefinition(
                name="STAGE7_MOVE_ABOVE",
                display_name="test",
                steps=(
                    ActionStep("CARRY_HIGH_P77_IDLE"),
                    ActionStep("P77_ABOVE_IDLE"),
                ),
            )
        )
        assert move_sequence.action_names == (
            "LITE_ABOVE_00_J1_HELD",
            "LITE_ABOVE_00_J1_LAST",
        )
        move_held = action_pwm(window.actions.get("LITE_ABOVE_00_J1_HELD"))
        assert move_held["001"] == observe_idle["001"]
        move_final = action_pwm(window.actions.get("LITE_ABOVE_00_J1_LAST"))
        assert all(
            move_final[f"{joint:03d}"] == calibrated_p77[f"{joint:03d}"]
            for joint in range(5)
        )
        assert move_held["005"] == move_final["005"] == observe_idle["005"]
        window.continue_calibration()
        assert window._current_spec is not None
        assert window.lite.pwm_values() == window._authoritative_point_pwm(
            window._current_spec.row, window._current_spec.col
        )
        assert set(window.lite.pwm_values().values()) != {550}
    finally:
        window.close()
        qt_app.processEvents()


def test_lite_pose_guard_rejects_uninitialized_all_minimum_pwm(
    qt_app: QApplication,
) -> None:
    window = CalibrationLiteWindow(AppConfig.load())
    try:
        spec = ANCHOR_SPECS[0]
        window._show_anchor(spec)
        for row in window.lite._pwm_rows.values():
            row.target.setValue(550)
        with pytest.raises(RuntimeError, match="跳变过大|异常相同|软件极限"):
            window._validated_lite_move_pwm(spec)
    finally:
        window.close()
        qt_app.processEvents()


def test_arbitrary_anchor_pwm_is_not_blocked_by_p77_carry_or_large_manual_delta(
    qt_app: QApplication,
    caplog,
) -> None:
    window = CalibrationLiteWindow(AppConfig.load(), dry_run=True)
    try:
        spec = ANCHOR_SPECS[1]
        assert spec.label == "P00"
        window._show_anchor(spec)
        window._last_sent_pwm.clear()
        expected = window._authoritative_point_pwm(spec.row, spec.col)

        unchanged = window._validated_lite_move_pwm(spec)
        assert unchanged == expected
        assert set(unchanged) == {"000", "001", "002", "003", "004"}

        for joint in range(5):
            current = int(expected[f"{joint:03d}"])
            lower = int(window.stage7.limits.joint_min[joint])
            upper = int(window.stage7.limits.joint_max[joint])
            if current + 200 < upper:
                target_joint, target = joint, current + 200
                break
            if current - 200 > lower:
                target_joint, target = joint, current - 200
                break
        else:  # pragma: no cover - configured limits always have room
            raise AssertionError("no joint has room for a 200 PWM manual change")

        window.lite._pwm_rows[target_joint].target.setValue(target)
        with caplog.at_level("WARNING", logger="app.calibration_lite.window"):
            validated = window._validated_lite_move_pwm(spec)
        assert validated[f"{target_joint:03d}"] == target
        assert "[LITE][ANCHOR_LARGE_MANUAL_CHANGE]" in caplog.text
        assert "已禁止整姿态发送" not in caplog.text
    finally:
        window.close()
        qt_app.processEvents()


def test_arbitrary_anchor_still_rejects_joint_software_limit(
    qt_app: QApplication,
) -> None:
    window = CalibrationLiteWindow(AppConfig.load(), dry_run=True)
    try:
        spec = ANCHOR_SPECS[1]
        window._show_anchor(spec)
        window._last_sent_pwm.clear()
        window.lite._pwm_rows[0].target.setValue(
            int(window.stage7.limits.joint_min[0])
        )
        with pytest.raises(RuntimeError, match="软件极限"):
            window._validated_lite_move_pwm(spec)
    finally:
        window.close()
        qt_app.processEvents()


def test_observe_override_preserves_stable_table_and_shares_pick_above(tmp_path) -> None:
    stable_path = PROJECT_ROOT / "config" / "arm_actions.json"
    before = hashlib.sha256(stable_path.read_bytes()).hexdigest()
    library = ActionLibrary(stable_path)
    stable = {name: library.get(name) for name in ("OBSERVE_IDLE", "OBSERVE_HOLD")}
    desired = action_pwm(stable["OBSERVE_IDLE"])
    desired["000"] += 20
    desired["003"] -= 10
    output = tmp_path / "observe.json"

    save_observe_override(
        output,
        library=library,
        stable_actions=stable,
        observe_idle_pwm=desired,
    )
    assert hashlib.sha256(stable_path.read_bytes()).hexdigest() == before
    idle = action_pwm(library.get("OBSERVE_IDLE"))
    hold = action_pwm(library.get("OBSERVE_HOLD"))
    assert idle["000"] == desired["000"]
    assert all(hold[f"{joint:03d}"] == idle[f"{joint:03d}"] for joint in range(5))
    assert hold["005"] == action_pwm(stable["OBSERVE_HOLD"])["005"]

    fresh = ActionLibrary(stable_path)
    assert load_observe_override(output, fresh)
    assert action_pwm(fresh.get("OBSERVE_IDLE"))["000"] == desired["000"]
    assert all(
        action_pwm(fresh.get("OBSERVE_HOLD"))[f"{joint:03d}"]
        == action_pwm(fresh.get("OBSERVE_IDLE"))[f"{joint:03d}"]
        for joint in range(5)
    )


def test_place_contact_uses_latest_above_and_persists_only_correction(tmp_path) -> None:
    stable_path = PROJECT_ROOT / "config" / "arm_actions.json"
    before = hashlib.sha256(stable_path.read_bytes()).hexdigest()
    library = ActionLibrary(stable_path)
    stable_above = library.get("P77_ABOVE_IDLE")
    stable_hold = library.get("P77_TOUCH_HOLD")
    stable_release = library.get("P77_TOUCH_RELEASE")
    above = action_pwm(stable_above)
    above["000"] += 35
    above["002"] -= 20
    library.register_runtime(
        build_action("P77_ABOVE_IDLE", above, action_times(stable_above))
    )
    derived = derive_place_contact(
        above, action_pwm(stable_above), action_pwm(stable_hold)
    )
    requested = dict(derived)
    requested["001"] -= 10
    output = tmp_path / "place.json"
    record = save_place_override(
        output,
        library=library,
        stable_above=stable_above,
        stable_touch_hold=stable_hold,
        stable_touch_release=stable_release,
        requested_pwm=requested,
        calibration_session="test-session",
    )
    assert record["correction_delta"]["001"] == -10
    assert record["motion_kind"].endswith("not_cartesian_movel")
    assert hashlib.sha256(stable_path.read_bytes()).hexdigest() == before
    fresh = ActionLibrary(stable_path)
    fresh.register_runtime(
        build_action("P77_ABOVE_IDLE", above, action_times(stable_above))
    )
    loaded = load_place_override(
        output,
        library=fresh,
        stable_above=stable_above,
        stable_touch_hold=stable_hold,
        stable_touch_release=stable_release,
    )
    assert loaded is not None
    assert action_pwm(fresh.get("P77_TOUCH_HOLD"))["001"] == requested["001"]


def test_manual_pump_commands_have_explicit_on_and_off_states() -> None:
    controller = SerialArmController(dry_run=True)
    controller.connect("COM_TEST")
    controller.pump_on()
    controller.pump_off()
    assert controller.dry_run_commands[-2:] == [
        ("PUMP_ON", "#005P2500T0500!"),
        ("PUMP_OFF", "#005P1500T0500!"),
    ]


def test_p77_dedicated_file_contains_new_pwm_only(tmp_path) -> None:
    path = tmp_path / "p77.json"
    expected = {
        "000": 1500,
        "001": 1170,
        "002": 870,
        "003": 1230,
        "004": 1500,
    }
    save_p77_point(path, expected)
    raw = json.loads(path.read_text(encoding="utf-8"))
    assert raw["new_pwm"] == expected
    assert "baseline_pwm" not in raw
    assert "delta_pwm" not in raw
    assert load_p77_point(path) == expected
