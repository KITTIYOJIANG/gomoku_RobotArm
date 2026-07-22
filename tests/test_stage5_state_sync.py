from __future__ import annotations

from app.stage5.state_machine import Stage5State, Stage5StateMachine


def test_stage5_stays_disconnected_without_serial():
    sm = Stage5StateMachine(dry_run=True)
    sm.on_board_lock_changed(True)
    sm.on_arm_state("OBSERVE_IDLE")
    # Board/arm alone cannot leave DISCONNECTED.
    assert sm.state == Stage5State.DISCONNECTED


def test_stage5_board_not_locked_when_serial_only():
    sm = Stage5StateMachine(dry_run=True)
    sm.on_serial_connected(board_locked=False)
    sm.on_arm_state("OBSERVE_IDLE")
    assert sm.state == Stage5State.BOARD_NOT_LOCKED


def test_stage5_idle_when_serial_locked_but_arm_not_ready():
    sm = Stage5StateMachine(dry_run=True)
    sm.sync_context(serial_connected=True, board_locked=True, arm_ready=False, estop=False)
    assert sm.state == Stage5State.IDLE


def test_stage5_ready_only_when_all_three_conditions_met():
    sm = Stage5StateMachine(dry_run=True)
    # Partial conditions insufficient.
    sm.sync_context(serial_connected=True, board_locked=False, arm_ready=True, estop=False)
    assert sm.state != Stage5State.READY
    sm.sync_context(serial_connected=True, board_locked=True, arm_ready=False, estop=False)
    assert sm.state != Stage5State.READY
    sm.sync_context(serial_connected=False, board_locked=True, arm_ready=True, estop=False)
    assert sm.state == Stage5State.DISCONNECTED
    # All three true.
    state = sm.sync_context(serial_connected=True, board_locked=True, arm_ready=True, estop=False)
    assert state == Stage5State.READY
    snap = sm.snapshot()
    assert snap.serial_connected is True
    assert snap.board_locked is True
    assert snap.arm_ready is True


def test_stage5_disconnect_resets():
    sm = Stage5StateMachine(dry_run=True)
    sm.sync_context(serial_connected=True, board_locked=True, arm_ready=True, estop=False)
    assert sm.state == Stage5State.READY
    sm.on_serial_disconnected()
    assert sm.state == Stage5State.DISCONNECTED


def test_stage5_p77_select_becomes_dry_run_ready():
    sm = Stage5StateMachine(dry_run=True)
    sm.sync_context(serial_connected=True, board_locked=True, arm_ready=True, estop=False)
    sm.select_target(7, 7, board_locked=True, calibrated=True, in_region=True)
    assert sm.state == Stage5State.DRY_RUN_READY
