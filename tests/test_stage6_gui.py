from __future__ import annotations

import inspect
import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

from app.gui.control_panel import ControlPanel
from app.gui.stage6_panel import Stage6Panel


def qt_app() -> QApplication:
    return QApplication.instance() or QApplication([])


def test_stage6_panel_exposes_required_controls_and_small_nudges() -> None:
    qt_app()
    panel = Stage6Panel()
    assert panel.target() == (7, 7)
    assert panel.level_combo.count() == 5
    assert panel.generate_current_button.text() == "生成当前点下降轨迹"
    assert panel.generate_all_button.text() == "生成全棋盘候选轨迹"
    assert panel.return_above_button.text() == "沿原路径安全返回 ABOVE"
    buttons = [button.text() for button in panel.findChildren(type(panel.generate_current_button))]
    for amount in ("-10", "-5", "-1", "+1", "+5", "+10"):
        assert buttons.count(amount) == 5


def test_stage6_low_position_lock_disables_target_selection() -> None:
    qt_app()
    panel = Stage6Panel()
    panel.set_lock_state(True, "BELOW_ABOVE_LOCKED_TO_P(7,7)")
    assert not panel.row_spin.isEnabled()
    assert not panel.col_spin.isEnabled()
    assert panel.lock_label.text() == "BELOW_ABOVE_LOCKED_TO_P(7,7)"


def test_stage6_panel_is_collapsed_and_does_not_build_serial_commands() -> None:
    qt_app()
    control = ControlPanel(dry_run=True)
    assert not control._stage6_group.isChecked()
    assert not control.stage6_panel.isVisible()
    source = inspect.getsource(Stage6Panel)
    assert "controller.write" not in source
    assert "#000P" not in source
