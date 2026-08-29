from __future__ import annotations

import pytest
from PySide6.QtWidgets import QApplication, QLabel, QPushButton

from app.arm.actions import ActionLibrary
from app.arm.controller import SerialArmController
from app.calibration_lite.point_movel import (
    P77_DELTA_PREDICTION_SOURCE,
    P77_POINT_ID,
    PointMoveLSequenceBuilder,
    PointMoveLStore,
)
from app.calibration_lite.point_movel_view import PointMoveLPanel
from app.calibration_lite.window import CalibrationLiteWindow
from app.config import AppConfig
from app.integrated_v1.profile import ProfileError


P77_ABOVE = {"J0": 1500, "J1": 1230, "J2": 870, "J3": 1230, "J4": 1500}
P77_DROP = {"J0": 1500, "J1": 1090, "J2": 790, "J3": 1370, "J4": 1500}


@pytest.fixture(scope="module")
def qt_app() -> QApplication:
    return QApplication.instance() or QApplication([])


def _store(path) -> PointMoveLStore:
    store = PointMoveLStore(path)
    if store.point(P77_POINT_ID) is None:
        store.save_drop(
            point_id=P77_POINT_ID,
            board=(7, 7),
            above_pwm=P77_ABOVE,
            drop_pwm=P77_DROP,
            operator_confirmed=True,
            hardware_verified=True,
        )
    return store


def test_p77_delta_guess_and_generic_persistence(tmp_path) -> None:
    store = _store(tmp_path / "point_movel.json")
    above = {"J0": 1400, "J1": 1300, "J2": 900, "J3": 1200, "J4": 1500}
    predicted = store.initial_drop_from_p77_delta(above)
    assert predicted == {
        "J0": 1400,
        "J1": 1160,
        "J2": 820,
        "J3": 1340,
        "J4": 1500,
    }
    edited = {**predicted, "J1": 1150, "J2": 825}
    saved = store.save_drop(
        point_id="P03_03",
        board=(3, 3),
        above_pwm=above,
        drop_pwm=edited,
        predicted_drop_pwm=predicted,
        prediction_source=P77_DELTA_PREDICTION_SOURCE,
    )
    store.save()

    assert saved["operator_confirmed"] is False
    assert saved["hardware_verified"] is False
    assert saved["prediction_source"] == "p77_delta_v1"
    assert saved["model_prediction"]["predicted_drop_pwm"] == predicted
    assert saved["residual_pwm"] == {
        "J0": 0,
        "J1": -10,
        "J2": 5,
        "J3": 0,
        "J4": 0,
    }
    loaded = PointMoveLStore(store.path).point("P03_03")
    assert loaded == saved

    with pytest.raises(ProfileError, match="HARDWARE VERIFIED"):
        store.save_drop(
            point_id=P77_POINT_ID,
            board=(7, 7),
            above_pwm=P77_ABOVE,
            drop_pwm={**P77_DROP, "J1": 1089},
        )

    with pytest.raises(ProfileError, match="HARDWARE VERIFIED"):
        store.save_drop(
            point_id=P77_POINT_ID,
            board=(7, 7),
            above_pwm=P77_ABOVE,
            drop_pwm={**P77_DROP, "J1": 1089},
            allow_hardware_verified_overwrite=True,
        )


def test_hardware_confirmation_is_explicit(tmp_path) -> None:
    store = _store(tmp_path / "point_movel.json")
    above = {"J0": 1400, "J1": 1300, "J2": 900, "J3": 1200, "J4": 1500}
    drop = store.initial_drop_from_p77_delta(above)
    store.save_drop(
        point_id="P03_03",
        board=(3, 3),
        above_pwm=above,
        drop_pwm=drop,
    )
    assert store.point("P03_03")["hardware_verified"] is False
    confirmed = store.confirm_hardware("P03_03")
    assert confirmed["operator_confirmed"] is True
    assert confirmed["hardware_verified"] is True


def test_point_panel_canonicalizes_and_edits_only_j0_to_j4(
    qt_app: QApplication,
) -> None:
    panel = PointMoveLPanel()
    try:
        above = {"J0": 1400, "J1": 1300, "J2": 900, "J3": 1200, "J4": 1500}
        drop = {"J0": 1400, "J1": 1160, "J2": 820, "J3": 1340, "J4": 1500}
        panel.set_point(
            point_id="P03_03",
            board=(3, 3),
            above_pwm=above,
            drop_pwm=drop,
            record=None,
            prediction_source="p77_delta_v1",
        )
        assert panel.canonical_point_id(3, 3) == "P03_03"
        assert panel.resolved_label.text() == "Resolved: P03_03"
        assert panel.above_pwm == above
        assert panel.drop_pwm() == drop
        assert set(panel._drop_fields) == {"J0", "J1", "J2", "J3", "J4"}
        assert "UNVERIFIED INITIAL GUESS" in panel.guess_label.text()
        assert any(
            "J5" in label.text() and "excluded" in label.text()
            for label in panel.findChildren(QLabel)
        )
        buttons = {button.text() for button in panel.findChildren(QPushButton)}
        assert {
            "-50",
            "-20",
            "-10",
            "+10",
            "+20",
            "+50",
            "Move ABOVE",
            "Move Current DROP",
            "Save Current DROP",
            "Confirm Hardware",
            "Return ABOVE",
        } <= buttons
        panel._drop_fields["J1"].setValue(1150)
        panel._drop_fields["J2"].setValue(830)
        assert panel.drop_pwm()["J1"] == 1150
        assert panel.drop_pwm()["J2"] == 830
        panel.set_controls(
            connected=True,
            busy=False,
            estop=False,
            dry_run=True,
            pose_state="DROP",
            active_point="P03_03",
        )
        assert not panel.confirm_button.isEnabled()
        panel.row_field.setValue(4)
        assert panel.resolved_label.text() == "Resolved: P04_03"
        panel.set_controls(
            connected=True,
            busy=False,
            estop=False,
            dry_run=True,
            pose_state="SAFE",
            active_point=None,
        )
        assert not panel.move_above_button.isEnabled()
    finally:
        panel.close()
        qt_app.processEvents()


def test_hardware_verified_point_locks_editing_but_allows_saved_motion(
    qt_app: QApplication,
) -> None:
    panel = PointMoveLPanel()
    above = {"J0": 1589, "J1": 1136, "J2": 1101, "J3": 1084, "J4": 1500}
    drop = {"J0": 1579, "J1": 996, "J2": 1061, "J3": 1224, "J4": 1500}
    try:
        panel.set_point(
            point_id="P03_03",
            board=(3, 3),
            above_pwm=above,
            drop_pwm=drop,
            record={
                "point_id": "P03_03",
                "above_pwm": above,
                "drop_pwm": drop,
                "operator_confirmed": True,
                "hardware_verified": True,
            },
            prediction_source="p77_delta_v1",
        )

        assert "HARDWARE VERIFIED" in panel.guess_label.text()
        assert all(not field.isEnabled() for field in panel._drop_fields.values())
        assert all(not button.isEnabled() for button in panel._adjust_buttons)

        panel.set_controls(
            connected=True,
            busy=False,
            estop=False,
            dry_run=True,
            pose_state="SAFE",
            active_point=None,
        )
        assert panel.move_above_button.isEnabled()
        assert not panel.move_drop_button.isEnabled()
        assert not panel.return_above_button.isEnabled()
        assert panel.back_button.isEnabled()
        assert panel.recalibrate_button.isEnabled()

        panel.set_controls(
            connected=True,
            busy=False,
            estop=False,
            dry_run=True,
            pose_state="ABOVE",
            active_point="P03_03",
        )
        assert panel.move_above_button.isEnabled()
        assert panel.move_drop_button.isEnabled()
        assert not panel.return_above_button.isEnabled()
        assert not panel.save_button.isEnabled()
        assert not panel.confirm_button.isEnabled()

        panel.set_controls(
            connected=True,
            busy=False,
            estop=False,
            dry_run=True,
            pose_state="DROP",
            active_point="P03_03",
        )
        assert panel.return_above_button.isEnabled()
        assert not panel.save_button.isEnabled()
        assert not panel.confirm_button.isEnabled()

        panel.set_recalibration_unlocked(True)
        assert panel.recalibration_unlocked
        assert all(field.isEnabled() for field in panel._drop_fields.values())
        assert all(button.isEnabled() for button in panel._adjust_buttons)
        panel._drop_fields["J0"].setValue(1575)
        panel._drop_fields["J2"].setValue(1055)
        draft = panel.drop_pwm()

        panel.set_controls(
            connected=True,
            busy=False,
            estop=False,
            dry_run=True,
            pose_state="ABOVE",
            active_point="P03_03",
        )
        assert panel.drop_pwm() == draft
        assert panel.move_drop_button.isEnabled()
        assert not panel.recalibrate_button.isEnabled()

        for connected, busy, estop, active_point in (
            (False, False, False, "P03_03"),
            (True, True, False, "P03_03"),
            (True, False, True, "P03_03"),
            (True, False, False, "P04_04"),
        ):
            panel.set_controls(
                connected=connected,
                busy=busy,
                estop=estop,
                dry_run=True,
                pose_state="ABOVE",
                active_point=active_point,
            )
            assert not panel.move_drop_button.isEnabled()

        panel.set_controls(
            connected=True,
            busy=False,
            estop=False,
            dry_run=True,
            pose_state="DROP",
            active_point="P03_03",
        )
        assert not panel.move_drop_button.isEnabled()
        assert panel.return_above_button.isEnabled()
        assert panel.drop_pwm() == draft
        assert panel.save_button.isEnabled()
        assert not panel.confirm_button.isEnabled()
    finally:
        panel.close()
        qt_app.processEvents()


def test_direct_drop_and_return_send_one_full_spatial_pose_without_j5(tmp_path) -> None:
    store = _store(tmp_path / "point_movel.json")
    library = ActionLibrary()
    builder = PointMoveLSequenceBuilder(actions=library, store=store)
    above = {"J0": 1400, "J1": 1300, "J2": 900, "J3": 1200, "J4": 1500}
    drop = store.initial_drop_from_p77_delta(above)
    descent = builder.build_move_drop("P03_03", drop)
    reverse = builder.build_return_above("P03_03", above)
    controller = SerialArmController(dry_run=True)
    controller.connect("POINT_MOVEL_DRY_RUN")
    for sequence in (descent, reverse):
        assert len(sequence.action_names) == 1
        action = library.get(sequence.action_names[0])
        assert [target.servo_id for target in action.targets] == [0, 1, 2, 3, 4]
        assert "#005" not in action.command
        controller.send_action(action)
    assert controller._connection is None
    assert len(controller.dry_run_commands) == 2


def test_verified_recalibration_draft_does_not_overwrite_until_save(
    qt_app: QApplication, tmp_path
) -> None:
    window = CalibrationLiteWindow(AppConfig.load(), dry_run=True)
    try:
        path = tmp_path / "point_movel.json"
        store = _store(path)
        above = window._point_movel_pwm(window.drop_store.above_pwm("P03_03"))
        drop = store.initial_drop_from_p77_delta(above)
        store.save_drop(
            point_id="P03_03",
            board=(3, 3),
            above_pwm=above,
            drop_pwm=drop,
            operator_confirmed=True,
            hardware_verified=True,
        )
        store.save()
        original_bytes = path.read_bytes()
        window.point_movel_store = store
        window.load_point_movel(3, 3)
        panel = window.lite.point_movel_panel
        window._point_movel_pose_state = "DROP"
        window._point_movel_active_point = "P03_03"

        window.begin_point_movel_recalibration("P03_03")
        assert panel.recalibration_unlocked
        assert path.read_bytes() == original_bytes

        panel._drop_fields["J0"].setValue(drop["J0"] - 4)
        panel._drop_fields["J2"].setValue(drop["J2"] - 6)
        edited = panel.drop_pwm()
        window._refresh_lite_status()
        assert panel.save_button.isEnabled()
        assert not panel.confirm_button.isEnabled()
        assert path.read_bytes() == original_bytes

        window.save_point_movel_drop("P03_03", edited)
        saved = store.point("P03_03")
        assert saved["drop_pwm"] == edited
        assert saved["operator_confirmed"] is False
        assert saved["hardware_verified"] is False
        assert not panel.recalibration_unlocked
        assert path.read_bytes() != original_bytes
    finally:
        window.close()
        qt_app.processEvents()


def test_p03_03_window_flow_uses_saved_above_and_dry_run_save(
    qt_app: QApplication,
    tmp_path,
) -> None:
    window = CalibrationLiteWindow(AppConfig.load(), dry_run=True)
    warnings: list[tuple[str, str]] = []
    submitted = []
    try:
        store = _store(tmp_path / "point_movel.json")
        window.point_movel_store = store
        window.point_movel_sequences = PointMoveLSequenceBuilder(
            actions=window.actions,
            store=store,
        )
        window._warn = lambda title, message: warnings.append((title, message))
        window._submit_point_movel_sequence = submitted.append

        window.show_point_movel()
        panel = window.lite.point_movel_panel
        assert window.lite.pages.currentIndex() == window.lite.POINT_MOVEL
        assert panel.point_id == "P03_03"
        expected_above = window._point_movel_pwm(
            window.drop_store.above_pwm("P03_03")
        )
        assert panel.above_pwm == expected_above
        assert panel.drop_pwm() == store.initial_drop_from_p77_delta(expected_above)

        window.move_point_movel_above("P03_03")
        assert submitted[-1].name == "MANUAL:POINT_MOVEL:ABOVE:P03_03"
        assert len(submitted[-1].action_names) == 3
        assert submitted[-1].action_names[0].endswith("_J1_HELD")
        assert submitted[-1].action_names[1].endswith("_SAFE_ABOVE")
        assert submitted[-1].action_names[2].endswith("_J1_LAST")
        window._point_movel_pose_state = "ABOVE"
        window._point_movel_active_point = "P03_03"

        edited = panel.drop_pwm()
        edited["J1"] -= 10
        edited["J2"] += 5
        panel._drop_fields["J1"].setValue(edited["J1"])
        panel._drop_fields["J2"].setValue(edited["J2"])
        window.move_point_movel_drop("P03_03", edited)
        assert submitted[-1].name == "MANUAL:POINT_MOVEL:DROP:P03_03"
        action = window.actions.get(submitted[-1].action_names[0])
        assert [target.servo_id for target in action.targets] == [0, 1, 2, 3, 4]
        window._point_movel_pose_state = "DROP"

        window.save_point_movel_drop("P03_03", edited)
        saved = store.point("P03_03")
        assert saved["drop_pwm"] == edited
        assert saved["operator_confirmed"] is False
        assert saved["hardware_verified"] is False
        assert saved["prediction_source"] == "p77_delta_v1"

        window.confirm_point_movel_drop("P03_03")
        assert warnings[-1][1] == "Dry Run cannot create HARDWARE VERIFIED evidence."
        assert store.point("P03_03")["hardware_verified"] is False

        window.return_point_movel_above("P03_03")
        assert submitted[-1].name == "MANUAL:POINT_MOVEL:RETURN_ABOVE:P03_03"
        window._point_movel_pose_state = "ABOVE"
        window.back_from_point_movel()
        assert submitted[-1].name == (
            "MANUAL:POINT_MOVEL:RETURN_OBSERVATION:P03_03"
        )
        assert submitted[-1].action_names[0].endswith("_J1_FIRST")
        assert submitted[-1].action_names[1].endswith("_J1_HELD")
        assert window.manual_movel_sequences is not None
    finally:
        window.close()
        qt_app.processEvents()
