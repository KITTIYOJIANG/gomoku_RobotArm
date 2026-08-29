from __future__ import annotations

import json

import pytest
from PySide6.QtWidgets import QApplication, QPushButton

from app.arm.actions import ActionLibrary
from app.arm.controller import SerialArmController
from app.calibration_lite.manual_movel import (
    FINAL_DROP_J1_REFERENCE_PWM,
    FINAL_DROP_STEP_INDEX,
    P77_GOLDEN_ABOVE,
    P77ManualMoveLSequenceBuilder,
    P77ManualMoveLStore,
)
from app.calibration_lite.manual_movel_view import P77ManualMoveLPanel
from app.calibration_lite.window import CalibrationLiteWindow
from app.config import AppConfig
from app.integrated_v1.profile import ProfileError


@pytest.fixture(scope="module")
def qt_app() -> QApplication:
    return QApplication.instance() or QApplication([])


def _populated_store(tmp_path):
    store = P77ManualMoveLStore(tmp_path / "p77_manual_movel.json")
    store.load_or_initialize()
    store.confirm_step(0)
    inherited_1 = store.create_next_step(0)
    step_1 = {"J0": 1500, "J1": 1170, "J2": 830, "J3": 1280, "J4": 1500}
    store.save_step(1, step_1)
    store.confirm_step(1)
    inherited_2 = store.create_next_step(1)
    step_2 = {"J0": 1500, "J1": 1110, "J2": 790, "J3": 1330, "J4": 1500}
    store.save_step(2, step_2)
    store.confirm_step(2)
    inherited_3 = store.create_next_step(2)
    step_3 = {"J0": 1500, "J1": 1050, "J2": 760, "J3": 1370, "J4": 1500}
    store.save_step(3, step_3)
    store.confirm_step(3)
    return (
        store,
        inherited_1,
        inherited_2,
        inherited_3,
        step_1,
        step_2,
        step_3,
    )


def test_step0_is_immutable_p77_golden_above(tmp_path) -> None:
    store = P77ManualMoveLStore(tmp_path / "manual.json")
    store.load_or_initialize()

    assert store.step(0)["final_pwm"] == P77_GOLDEN_ABOVE == {
        "J0": 1500,
        "J1": 1230,
        "J2": 870,
        "J3": 1230,
        "J4": 1500,
    }
    with pytest.raises(ProfileError, match="immutable"):
        store.save_step(0, {**P77_GOLDEN_ABOVE, "J1": 1229})


def test_next_step_inherits_previous_saved_final_pwm(tmp_path) -> None:
    (
        store,
        inherited_1,
        inherited_2,
        inherited_3,
        step_1,
        step_2,
        _step_3,
    ) = _populated_store(tmp_path)

    assert inherited_1["final_pwm"] == P77_GOLDEN_ABOVE
    assert inherited_2["final_pwm"] == step_1
    assert inherited_2["inherited_from_step"] == 1
    assert inherited_3["final_pwm"] == step_2
    assert inherited_3["inherited_from_step"] == 2
    assert inherited_3["final_drop_reference"]["J1_approx_pwm"] == 1050
    assert inherited_2["direction_suggestion"]["J2"] == "small decrease"
    assert "kinematic truth" in inherited_2["direction_suggestion"]["note"]
    operator_override = {
        "J0": 1510,
        "J1": 1235,
        "J2": 875,
        "J3": 1238,
        "J4": 1490,
    }
    saved = store.save_step(2, operator_override)
    assert saved["final_pwm"] == operator_override
    assert saved["correction_pwm"] == {
        joint: operator_override[joint] - step_1[joint]
        for joint in P77_GOLDEN_ABOVE
    }
    assert all(value > 0 for value in saved["final_pwm"].values())

    with pytest.raises(ProfileError, match="ends at Step3"):
        store.create_next_step(FINAL_DROP_STEP_INDEX)


def test_manual_steps_save_and_load_without_hardware_verification(tmp_path) -> None:
    store, *_unused, step_3 = _populated_store(tmp_path)
    with pytest.raises(ProfileError, match="only Step3"):
        store.set_as_drop(2)
    drop = store.set_as_drop(3)
    above_before = dict(store.step(0)["final_pwm"])
    store.save()

    loaded = P77ManualMoveLStore(store.path)
    loaded.load()

    assert loaded.step_count() == 4
    assert loaded.step(3)["final_pwm"] == step_3
    assert all(
        item["hardware_verified"] is False
        for item in loaded.data["steps"]
    )
    assert drop == loaded.drop_candidate()
    assert drop["source"] == "manual_movel_tuning"
    assert drop["operator_confirmed"] is True
    assert drop["hardware_verified"] is False
    assert loaded.step(0)["final_pwm"] == above_before == P77_GOLDEN_ABOVE
    raw = json.loads(store.path.read_text(encoding="utf-8"))
    assert raw["golden_above_pwm"] == P77_GOLDEN_ABOVE


def test_reverse_uses_saved_manual_steps_and_never_commands_j5(tmp_path) -> None:
    store, _i1, _i2, _i3, step_1, step_2, _step_3 = _populated_store(tmp_path)
    library = ActionLibrary()
    builder = P77ManualMoveLSequenceBuilder(actions=library, store=store)

    sequence = builder.build_return_above(3)

    assert sequence.action_names == (
        "P77_MANUAL_MOVEL_STEP_02_SAVED",
        "P77_MANUAL_MOVEL_STEP_01_SAVED",
        "P77_MANUAL_MOVEL_STEP_00_SAVED",
    )
    assert [target.pwm for target in library.get(sequence.action_names[0]).targets] == [
        step_2[f"J{joint}"] for joint in range(5)
    ]
    assert [target.pwm for target in library.get(sequence.action_names[1]).targets] == [
        step_1[f"J{joint}"] for joint in range(5)
    ]
    assert [target.pwm for target in library.get(sequence.action_names[2]).targets] == [
        P77_GOLDEN_ABOVE[f"J{joint}"] for joint in range(5)
    ]
    for name in sequence.action_names:
        action = library.get(name)
        assert [target.servo_id for target in action.targets] == [0, 1, 2, 3, 4]
        assert "#005" not in action.command


def test_back_from_step0_returns_observation_with_j1_first(tmp_path) -> None:
    store = P77ManualMoveLStore(tmp_path / "manual.json")
    store.load_or_initialize()
    library = ActionLibrary()
    builder = P77ManualMoveLSequenceBuilder(actions=library, store=store)

    sequence = builder.build_return_observation()

    assert sequence.action_names == (
        "P77_MANUAL_RETURN_OBSERVATION_00_J1_FIRST",
        "P77_MANUAL_RETURN_OBSERVATION_00_J1_HELD",
    )
    observe = library.get("OBSERVE_IDLE")
    phase_a = library.get(sequence.action_names[0])
    phase_b = library.get(sequence.action_names[1])
    assert phase_a.target(1).pwm == observe.target(1).pwm
    assert [phase_a.target(joint).pwm for joint in (0, 2, 3, 4)] == [
        P77_GOLDEN_ABOVE[f"J{joint}"] for joint in (0, 2, 3, 4)
    ]
    assert [target.pwm for target in phase_b.targets] == [
        target.pwm for target in observe.targets
    ]
    assert all(
        target.time_ms == 1500
        for name in sequence.action_names
        for target in library.get(name).targets
    )
    assert all(
        library.get(name).target(5).pwm == 1500
        for name in sequence.action_names
    )


def test_ui_edits_absolute_final_pwm_and_exposes_required_controls(
    qt_app: QApplication,
) -> None:
    panel = P77ManualMoveLPanel()
    try:
        panel.set_step(
            {
                "step_index": 0,
                "final_pwm": P77_GOLDEN_ABOVE,
                "operator_confirmed": False,
                "hardware_verified": False,
            }
        )
        assert panel.final_pwm() == P77_GOLDEN_ABOVE
        assert panel.step_label.text() == "Step 0"
        assert not hasattr(panel, "_corrections")
        assert all(not field.isEnabled() for field in panel._fields.values())
        button_texts = {button.text() for button in panel.findChildren(QPushButton)}
        assert {
            "-50",
            "-20",
            "-10",
            "+10",
            "+20",
            "+50",
            "Move Current Step",
            "Save Current Step",
            "Confirm Step",
            "Next Step",
            "Previous Step",
            "Return Previous Step",
            "Return ABOVE",
            "Set Step 3 As DROP",
            "Run P77 Pick & Place Full Flow",
            "Back to Home",
            "Emergency Stop",
        } <= button_texts
        assert any(
            "J5" in label.text() for label in panel.findChildren(type(panel.step_label))
        )
        panel.set_step(
            {
                "step_index": 3,
                "final_pwm": {
                    "J0": 1500,
                    "J1": 1050,
                    "J2": 760,
                    "J3": 1370,
                    "J4": 1500,
                },
                "operator_confirmed": True,
                "hardware_verified": False,
            }
        )
        assert panel.step_label.text() == "Step 3 / FINAL DROP"
        assert str(FINAL_DROP_J1_REFERENCE_PWM) in panel.suggestion_label.text()
        assert set(panel.final_pwm()) == {"J0", "J1", "J2", "J3", "J4"}
        panel.set_controls(
            connected=True,
            busy=False,
            estop=False,
            pose_index=3,
            record={
                "final_pwm": panel.final_pwm(),
                "operator_confirmed": True,
            },
            step_count=4,
        )
        assert not panel.next_button.isEnabled()
        assert panel.set_drop_button.isEnabled()
    finally:
        panel.close()
        qt_app.processEvents()


def test_p77_manual_movel_dry_run_never_opens_hardware(tmp_path) -> None:
    store, *_unused = _populated_store(tmp_path)
    library = ActionLibrary()
    builder = P77ManualMoveLSequenceBuilder(actions=library, store=store)
    controller = SerialArmController(dry_run=True)
    controller.connect("P77_MANUAL_MOVEL_DRY_RUN")
    operator_override = {
        "J0": 1510,
        "J1": 1240,
        "J2": 860,
        "J3": 1220,
        "J4": 1490,
    }

    enter = builder.build_enter_above(
        {"J0": 1770, "J1": 1300, "J2": 920, "J3": 1170, "J4": 1500}
    )
    move = builder.build_move_candidate(1, operator_override)
    reverse = builder.build_return_above(1)
    for sequence in (enter, move, reverse):
        for action_name in sequence.action_names:
            controller.send_action(library.get(action_name))

    assert controller._connection is None
    assert controller.dry_run is True
    assert len(controller.dry_run_commands) == 5
    assert all("#005" not in command for _name, command in controller.dry_run_commands)
    assert enter.action_names == (
        "P77_MANUAL_MOVEL_ENTER_J1_HELD",
        "P77_MANUAL_MOVEL_SAFE_ABOVE",
        "P77_MANUAL_MOVEL_STEP_00",
    )
    assert [
        target.pwm for target in library.get(move.action_names[0]).targets
    ] == [operator_override[f"J{joint}"] for joint in range(5)]
    assert [
        target.servo_id for target in library.get(move.action_names[0]).targets
    ] == [0, 1, 2, 3, 4]
    assert store.step(1)["final_pwm"] != operator_override
    assert reverse.action_names == ("P77_MANUAL_MOVEL_STEP_00_SAVED",)
    for name, command in controller.dry_run_commands:
        print(f"P77_MANUAL_MOVEL_DRY_RUN {name} {command}")


def test_p77_full_pick_place_uses_confirmed_manual_path_and_exact_reverse(
    tmp_path,
) -> None:
    store, _i1, _i2, _i3, step_1, step_2, step_3 = _populated_store(tmp_path)
    store.set_as_drop(3)
    library = ActionLibrary()
    builder = P77ManualMoveLSequenceBuilder(actions=library, store=store)

    sequence = builder.build_full_pick_place()
    names = sequence.action_names

    assert sequence.requires_board is True
    assert names[:4] == (
        "SOURCE_TOUCH_IDLE",
        "SOURCE_TOUCH_HOLD",
        "OBSERVE_HOLD",
        "P77_MANUAL_FULL_ENTER_J1_HELD_HOLD",
    )
    assert names[4:9] == (
        "P77_MANUAL_FULL_SAFE_ABOVE_HOLD",
        "P77_MANUAL_FULL_STEP_00_HOLD",
        "P77_MANUAL_FULL_STEP_01_HOLD",
        "P77_MANUAL_FULL_STEP_02_HOLD",
        "P77_MANUAL_FULL_STEP_03_HOLD",
    )
    release = names.index("P77_MANUAL_FULL_DROP_RELEASE")
    assert names[release + 1 : release + 4] == (
        "P77_MANUAL_FULL_STEP_02_IDLE",
        "P77_MANUAL_FULL_STEP_01_IDLE",
        "P77_MANUAL_FULL_STEP_00_IDLE",
    )
    assert names[-2:] == (
        "P77_MANUAL_FULL_RETURN_00_J1_FIRST",
        "P77_MANUAL_FULL_RETURN_00_J1_HELD",
    )
    assert [
        target.pwm
        for target in library.get("P77_MANUAL_FULL_STEP_01_HOLD").targets[:5]
    ] == [step_1[f"J{joint}"] for joint in range(5)]
    assert [
        target.pwm
        for target in library.get("P77_MANUAL_FULL_STEP_02_HOLD").targets[:5]
    ] == [step_2[f"J{joint}"] for joint in range(5)]
    assert [
        target.pwm
        for target in library.get("P77_MANUAL_FULL_STEP_03_HOLD").targets[:5]
    ] == [step_3[f"J{joint}"] for joint in range(5)]
    assert all(
        target.time_ms == 1500
        for name in names[-2:]
        for target in library.get(name).targets
    )

    released = False
    for name in names:
        if name == "P77_MANUAL_FULL_DROP_RELEASE":
            released = True
        if name in {"SOURCE_TOUCH_IDLE"}:
            expected_pump = 1500
        elif released:
            expected_pump = 1500
        else:
            expected_pump = 2500
        assert library.get(name).target(5).pwm == expected_pump

    controller = SerialArmController(dry_run=True)
    controller.connect("P77_FULL_FLOW_DRY_RUN")
    for name in names:
        controller.send_action(library.get(name))
    assert controller._connection is None
    assert [name for name, _command in controller.dry_run_commands] == list(names)
    for name, command in controller.dry_run_commands:
        print(f"P77_FULL_FLOW_DRY_RUN {name} {command}")

    from_above = builder.build_full_pick_place(start_from_above=True)
    assert from_above.action_names[:3] == (
        "P77_MANUAL_FULL_RETURN_00_J1_FIRST",
        "P77_MANUAL_FULL_RETURN_00_J1_HELD",
        "SOURCE_TOUCH_IDLE",
    )


def test_lite_window_exposes_p77_manual_page_without_auto_connect(
    qt_app: QApplication,
) -> None:
    window = CalibrationLiteWindow(AppConfig.load(), dry_run=True)
    try:
        assert not window.controller.is_connected
        assert window.camera_worker is None
        assert window.manual_movel_store is not None
        assert window.manual_movel_store.step(0)["final_pwm"] == P77_GOLDEN_ABOVE
        drop_before = window.manual_movel_store.drop_candidate()
        window.show_manual_movel()
        assert window.lite.pages.currentIndex() == window.lite.MANUAL_MOVEL
        assert window.lite.manual_movel_panel.final_pwm() == P77_GOLDEN_ABOVE
        assert window.manual_movel_store.drop_candidate() == drop_before
        window.back_from_manual_movel()
        assert window.lite.pages.currentIndex() == window.lite.HOME
    finally:
        window.close()
        qt_app.processEvents()
