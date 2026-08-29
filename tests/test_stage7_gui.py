from __future__ import annotations

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

from app.arm.state import ArmState
from app.gui.control_panel import ControlPanel
from app.gui.rapid_calibration_panel import RapidCalibrationPanel
from app.stage7.baseline import point_id


def qt_app() -> QApplication:
    return QApplication.instance() or QApplication([])


def test_stage7_panel_has_225_grid_points_and_live_defaults() -> None:
    qt_app()
    panel = RapidCalibrationPanel()

    assert len(panel._grid_buttons) == 225
    assert len(panel._joint_edits) == 5
    assert panel.step() == 5
    assert not hasattr(panel, "dry_run_checkbox")
    assert "SERIAL NOT CONNECTED" in panel.live_status_label.text()
    assert "J5=气泵" in panel._joint_edits[0].parentWidget().title()


def test_point_selector_uses_flat_index_and_row_col() -> None:
    qt_app()
    panel = RapidCalibrationPanel()
    selected: list[tuple[int, int]] = []
    panel.point_selected.connect(lambda row, col: selected.append((row, col)))

    panel.point_index.setValue(137)

    assert panel.current_point == (9, 2)
    assert selected[-1] == (9, 2)
    assert "P137" in panel.current_point_label.text()
    assert "(9,2)" in panel.current_point_label.text()


def test_joint_buttons_emit_selected_step_without_sending_hardware() -> None:
    qt_app()
    panel = RapidCalibrationPanel()
    events: list[tuple[int, int]] = []
    panel.jog_requested.connect(lambda joint, delta: events.append((joint, delta)))

    # Invoke the same intent method used by buttons/shortcuts; the panel owns no controller.
    panel._emit_jog(2, panel.step())

    assert events == [(2, 5)]
    assert panel._selected_joint == 2


def test_grid_provenance_prioritizes_verified_state() -> None:
    qt_app()
    panel = RapidCalibrationPanel()
    records = {
        point_id(0, 0): {"source": "DIRECT", "verified": False},
        point_id(0, 1): {"source": "INTERPOLATED", "verified": False},
        point_id(0, 2): {"source": "INTERPOLATED", "verified": True},
    }

    panel.update_grid(records)

    assert "DIRECT" in panel._grid_buttons[(0, 0)].toolTip()
    assert "INTERPOLATED" in panel._grid_buttons[(0, 1)].toolTip()
    assert "VERIFIED" in panel._grid_buttons[(0, 2)].toolTip()


def test_absolute_input_emits_only_after_enter_and_targets_one_joint() -> None:
    qt_app()
    panel = RapidCalibrationPanel()
    events: list[tuple[int, int, int]] = []
    panel.apply_joint_requested.connect(
        lambda joint, pwm, time_ms: events.append((joint, pwm, time_ms))
    )

    panel._joint_edits[2].setValue(1450)
    assert events == []

    panel._joint_edits[2].lineEdit().returnPressed.emit()
    assert events == [(2, 1450, 1000)]


def test_apply_all_is_separate_and_speed_defaults_are_safe() -> None:
    qt_app()
    panel = RapidCalibrationPanel()
    events: list[tuple[dict[str, int], int]] = []
    panel.apply_all_requested.connect(
        lambda pwm, time_ms: events.append((dict(pwm), time_ms))
    )
    panel.set_pwm_values({f"{joint:03d}": 1200 + joint for joint in range(5)})

    panel.apply_all_button.click()

    assert panel.step() == 5
    assert panel.speed_time_ms() == 1000
    assert events == [
        ({"000": 1200, "001": 1201, "002": 1202, "003": 1203, "004": 1204}, 1000)
    ]


def test_dry_run_target_flash_is_visible_in_the_grid() -> None:
    qt_app()
    panel = RapidCalibrationPanel()
    panel.select_point(7, 7, emit=False)

    panel.flash_point(7, 7)

    assert "#fb923c" in panel._grid_buttons[(7, 7)].styleSheet()


def test_original_control_panel_mounts_one_slim_task_ui_and_diagnostics_only() -> None:
    qt_app()
    control = ControlPanel(dry_run=True)
    advanced_widgets = [
        control._advanced_content.layout().itemAt(index).widget()
        for index in range(control._advanced_content.layout().count())
    ]

    assert control.rapid_calibration_panel.parentWidget() is not None
    assert control.stage5_panel not in advanced_widgets
    assert control.stage6_panel not in advanced_widgets
    assert control.cross_anchor_panel not in advanced_widgets
    assert control.hover_learning_panel not in advanced_widgets
    assert len(advanced_widgets) == 3  # status, vision diagnostics, serial log


def test_runtime_return_button_exposes_required_observe_start_action() -> None:
    qt_app()
    control = ControlPanel()
    requested: list[bool] = []
    control.return_observe_requested.connect(lambda: requested.append(True))

    control.rapid_calibration_panel.runtime_return_button.setEnabled(True)
    control.rapid_calibration_panel.runtime_return_button.click()

    assert requested == [True]
    assert "观察位" in control.rapid_calibration_panel.runtime_return_button.text()


def test_recovery_button_is_only_enabled_for_a_latched_estop() -> None:
    qt_app()
    control = ControlPanel()

    control.update_controls(
        connected=True,
        state=ArmState.UNKNOWN,
        busy=False,
        board_locked=False,
        target_visible=False,
        estop_latched=False,
    )
    assert control.recover_button.isEnabled() is False

    control.update_controls(
        connected=True,
        state=ArmState.ERROR,
        busy=False,
        board_locked=False,
        target_visible=False,
        estop_latched=True,
    )
    assert control.recover_button.isEnabled() is True
    assert control.rapid_calibration_panel.runtime_return_button.isEnabled() is False
