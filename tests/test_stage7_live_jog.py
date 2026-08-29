from __future__ import annotations

from dataclasses import replace
import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

from app.arm.actions import ActionLibrary
from app.arm.sequences import return_to_observe
from app.arm.state import ArmState, ArmStateMachine
from app.config import AppConfig
from app.main_window import MainWindow
from app.stage7.coordinator import RapidCalibrationCoordinator
from app.stage7.settings import Stage7Settings


class MockLiveController:
    dry_run = False
    is_connected = True
    port = "MOCK_COM"

    def __init__(self) -> None:
        self.calls: list[tuple[int, int, int]] = []

    def send_joint_pwm(self, joint_id: int, pwm: int, *, time_ms: int = 200) -> str:
        self.calls.append((joint_id, pwm, time_ms))
        return f"#{joint_id:03d}P{pwm:04d}T{time_ms:04d}!"


def qt_app() -> QApplication:
    return QApplication.instance() or QApplication([])


def test_g_dry_run_jog_changes_gui_but_never_records_serial_tx() -> None:
    qt_app()
    window = MainWindow(AppConfig.load(), dry_run=True)
    try:
        panel = window.control_panel.rapid_calibration_panel
        panel.set_joint_pwm(2, 1265)

        window.stage7_jog(2, 5)

        assert panel.joint_pwm(2) == 1270
        assert window.controller.dry_run_commands == []
        assert "NOT SENT" in panel.live_status_label.text()
    finally:
        window.close()
        QApplication.processEvents()


def test_dry_run_move_above_flashes_target_and_never_sends() -> None:
    qt_app()
    window = MainWindow(AppConfig.load(), dry_run=True)
    try:
        panel = window.control_panel.rapid_calibration_panel
        panel.select_point(7, 7)

        window.stage7_move_above()

        assert window.controller.dry_run_commands == []
        assert "SIMULATED / NOT SENT" in panel.live_status_label.text()
        assert "target flashing" in panel.workflow_label.text()
    finally:
        window.close()
        QApplication.processEvents()


def test_h_one_jog_sends_only_one_joint_and_preserves_other_axes() -> None:
    controller = MockLiveController()
    settings = replace(
        Stage7Settings.load(),
        default_dry_run=False,
        force_dry_run=False,
    )
    coordinator = RapidCalibrationCoordinator(
        controller=controller,  # type: ignore[arg-type]
        library=ActionLibrary(),
        settings=settings,
    )
    coordinator.set_dry_run(False)

    queued = coordinator.queue_live_jog(2, 1270)
    sent = coordinator.flush_one_jog()

    assert queued.queued is True
    assert sent is not None and sent.sent is True
    assert controller.calls == [(2, 1270, settings.live_jog_time_ms)]
    assert sent.command == "#002P1270T0200!"
    assert all(joint == 2 for joint, _pwm, _time in controller.calls)


def test_rapid_clicks_coalesce_to_latest_value_per_joint() -> None:
    controller = MockLiveController()
    settings = replace(Stage7Settings.load(), default_dry_run=False)
    coordinator = RapidCalibrationCoordinator(
        controller=controller,  # type: ignore[arg-type]
        library=ActionLibrary(),
        settings=settings,
    )
    coordinator.set_dry_run(False)

    coordinator.queue_live_jog(1, 1205)
    coordinator.queue_live_jog(1, 1210)
    coordinator.queue_live_jog(1, 1215)

    assert coordinator.pending_jog_count == 1
    coordinator.flush_one_jog()
    assert controller.calls == [(1, 1215, settings.live_jog_time_ms)]


def test_stage7_above_state_requires_explicit_safe_return() -> None:
    state = ArmStateMachine()
    state.connect()
    state.mark_observe_idle()

    state.begin_stage7_hover()
    state.complete_stage7_hover()
    assert state.state == ArmState.HOVERING

    state.begin_stage7_return()
    state.complete_stage7_return()
    assert state.state == ArmState.OBSERVE_IDLE


def test_normal_application_mode_is_live_but_never_auto_connects() -> None:
    qt_app()
    window = MainWindow(AppConfig.load(), dry_run=False)
    try:
        assert window.controller.dry_run is False
        assert window.stage7.dry_run is False
        assert window.controller.is_connected is False
        assert "SERIAL NOT CONNECTED" in window.stage7.live_status
    finally:
        window.close()
        QApplication.processEvents()


def test_stage5_estop_latch_blocks_every_shared_sequence_path(monkeypatch) -> None:
    qt_app()
    window = MainWindow(AppConfig.load(), dry_run=True)
    try:
        window.controller.connect("COM6")
        window.state_machine.connect()
        window.stage5.estop()
        warnings: list[str] = []
        monkeypatch.setattr(
            "app.main_window.QMessageBox.warning",
            lambda _parent, _title, message, *args: warnings.append(str(message)),
        )

        window._start_sequence(
            return_to_observe(),
            window.state_machine.begin_return_to_observe,
        )

        assert warnings and "急停后恢复" in warnings[-1]
        assert window.controller.dry_run_commands == []
        assert window.state_machine.state == ArmState.UNKNOWN
    finally:
        window.close()
        QApplication.processEvents()
