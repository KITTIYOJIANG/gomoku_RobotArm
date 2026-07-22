import pytest

from app.arm.state import ArmState, ArmStateMachine, InvalidTransition


def connected_unknown_machine() -> ArmStateMachine:
    machine = ArmStateMachine()
    machine.connect()
    assert machine.state == ArmState.UNKNOWN
    return machine


def test_unknown_cannot_pick_and_observe_idle_can_pick():
    machine = connected_unknown_machine()
    with pytest.raises(InvalidTransition):
        machine.begin_pick()
    machine.begin_return_to_observe()
    machine.complete_return_to_observe()
    assert machine.can_pick()
    machine.begin_pick()
    machine.complete_pick()
    assert machine.state == ArmState.OBSERVE_HOLD


def test_only_observe_hold_with_board_lock_can_place():
    machine = connected_unknown_machine()
    machine.begin_return_to_observe()
    machine.complete_return_to_observe()
    assert not machine.can_place(board_locked=True)
    machine.begin_pick()
    machine.complete_pick()
    assert not machine.can_place(board_locked=False)
    assert machine.can_place(board_locked=True, target_visible=True)
    machine.begin_place(board_locked=True, target_visible=True)
    machine.complete_place()
    assert machine.state == ArmState.OBSERVE_IDLE


def test_second_action_is_rejected_while_busy():
    machine = connected_unknown_machine()
    machine.begin_return_to_observe()
    with pytest.raises(InvalidTransition, match="already running"):
        machine.begin_return_to_observe()


def test_estop_is_latched_and_never_restores_pick_permission_directly():
    machine = connected_unknown_machine()
    machine.begin_return_to_observe()
    machine.complete_return_to_observe()
    machine.estop()
    assert machine.state == ArmState.ESTOP
    assert not machine.can_pick()
    with pytest.raises(InvalidTransition):
        machine.connect()
    # Recovery requires an explicit user-commanded motion, not a state reset.
    machine.begin_return_to_observe()
    assert machine.state == ArmState.MOVING_TO_OBSERVE
    assert not machine.can_pick()
