from __future__ import annotations

from app.arm.controller import SerialArmController
from app.arm.state import ArmState, ArmStateMachine
from app.arm.worker import ArmSequenceWorker
from app.arm.actions import ActionLibrary
from app.config import Stage5Config
from app.stage5.coordinator import Stage5Coordinator, _normalize_arm_state
from app.stage5.state_machine import Stage5State, Stage5StateMachine
from app.gui.stage5_panel import Stage5Panel
from pathlib import Path


def test_initial_disconnected():
    sm = Stage5StateMachine(dry_run=True)
    assert sm.state == Stage5State.DISCONNECTED


def test_serial_true_board_false():
    sm = Stage5StateMachine(dry_run=True)
    sm.sync_context(serial_connected=True, board_locked=False, arm_ready=True, estop=False)
    assert sm.state == Stage5State.BOARD_NOT_LOCKED


def test_ready_when_serial_board_observe():
    sm = Stage5StateMachine(dry_run=True)
    sm.sync_context(serial_connected=True, board_locked=True, arm_ready=True, estop=False)
    assert sm.state == Stage5State.READY


def test_normalize_arm_state_enum():
    assert _normalize_arm_state(ArmState.OBSERVE_IDLE) == "OBSERVE_IDLE"
    assert _normalize_arm_state("OBSERVE_IDLE") == "OBSERVE_IDLE"
    assert _normalize_arm_state(ArmState.OBSERVE_HOLD) == "OBSERVE_HOLD"


def test_coordinator_late_creation_after_com_connected(tmp_path: Path):
    """Coordinator created while COM already connected must become READY after update_context."""
    lib = ActionLibrary()
    controller = SerialArmController(dry_run=True)
    controller.connect("COM6")
    assert controller.is_connected
    arm = ArmStateMachine()
    arm.connect()
    arm.begin_return_to_observe()
    arm.complete_return_to_observe()
    assert arm.state == ArmState.OBSERVE_IDLE
    worker = ArmSequenceWorker(controller, lib)
    worker.start()
    try:
        cfg = Stage5Config(True, True, 0.32, tmp_path / "c.json", 8, False)
        coord = Stage5Coordinator(
            config=cfg,
            actions=lib,
            controller=controller,
            arm_state=arm,
            worker=worker,
            action_wait_margin_ms=50,
            logs_dir=tmp_path,
        )
        # Still disconnected until context push (simulates missed historical signal)
        assert coord.stage_state.state == Stage5State.DISCONNECTED
        coord.update_context(
            serial_connected=controller.is_connected,
            board_locked=True,
            arm_state=arm.state,
            emergency_stopped=False,
        )
        assert coord.stage_state.state == Stage5State.READY
        assert coord.controller is controller
    finally:
        worker.shutdown()
        controller.close()


def test_shared_controller_identity(tmp_path: Path):
    lib = ActionLibrary()
    controller = SerialArmController(dry_run=True)
    arm = ArmStateMachine()
    worker = ArmSequenceWorker(controller, lib)
    cfg = Stage5Config(True, True, 0.32, tmp_path / "c.json", 8, False)
    coord = Stage5Coordinator(
        config=cfg,
        actions=lib,
        controller=controller,
        arm_state=arm,
        worker=worker,
        action_wait_margin_ms=50,
        logs_dir=tmp_path,
    )
    assert coord.controller is controller
    assert id(coord.controller) == id(controller)


def test_dry_run_checkbox_enabled_when_disconnected():
    from PySide6.QtWidgets import QApplication
    import sys
    app = QApplication.instance() or QApplication(sys.argv[:1])
    panel = Stage5Panel(default_dry_run=True)
    assert panel.dry_run_checkbox.isChecked() is True
    panel.set_enabled_state(
        serial_connected=False,
        board_locked=False,
        busy=False,
        can_hover=False,
        can_return=False,
        has_target=False,
        estop=False,
    )
    assert panel.dry_run_checkbox.isEnabled() is True
    assert panel.dry_run_checkbox.isChecked() is True


def test_disconnect_to_disconnected():
    sm = Stage5StateMachine(dry_run=True)
    sm.sync_context(serial_connected=True, board_locked=True, arm_ready=True, estop=False)
    assert sm.state == Stage5State.READY
    sm.on_serial_disconnected()
    assert sm.state == Stage5State.DISCONNECTED


def test_reconnect_to_ready():
    sm = Stage5StateMachine(dry_run=True)
    sm.sync_context(serial_connected=True, board_locked=True, arm_ready=True, estop=False)
    sm.on_serial_disconnected()
    assert sm.state == Stage5State.DISCONNECTED
    sm.sync_context(serial_connected=True, board_locked=True, arm_ready=True, estop=False)
    assert sm.state == Stage5State.READY
