from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from .models import DescentLevel


class Stage6MotionState(str, Enum):
    TARGET_ABOVE = "TARGET_ABOVE"
    DESCENDING_25 = "DESCENDING_25"
    DESCENDING_50 = "DESCENDING_50"
    DESCENDING_75 = "DESCENDING_75"
    TARGET_TOUCH = "TARGET_TOUCH"
    RELEASING = "RELEASING"
    RELEASE_DWELL = "RELEASE_DWELL"
    ASCENDING_75 = "ASCENDING_75"
    ASCENDING_50 = "ASCENDING_50"
    ASCENDING_25 = "ASCENDING_25"
    RETURNED_ABOVE = "RETURNED_ABOVE"
    CARRY_HIGH = "CARRY_HIGH"
    OBSERVE = "OBSERVE"
    ERROR = "ERROR"
    EMERGENCY_STOP = "EMERGENCY_STOP"


class Stage6TransitionError(RuntimeError):
    pass


DESCENT_TRANSITIONS = {
    Stage6MotionState.TARGET_ABOVE: (
        DescentLevel.DESCENT_25,
        Stage6MotionState.DESCENDING_25,
    ),
    Stage6MotionState.DESCENDING_25: (
        DescentLevel.DESCENT_50,
        Stage6MotionState.DESCENDING_50,
    ),
    Stage6MotionState.DESCENDING_50: (
        DescentLevel.DESCENT_75,
        Stage6MotionState.DESCENDING_75,
    ),
    Stage6MotionState.DESCENDING_75: (
        DescentLevel.TOUCH,
        Stage6MotionState.TARGET_TOUCH,
    ),
}

ASCENT_TRANSITIONS = {
    Stage6MotionState.TARGET_TOUCH: (
        DescentLevel.DESCENT_75,
        Stage6MotionState.ASCENDING_75,
    ),
    Stage6MotionState.RELEASE_DWELL: (
        DescentLevel.DESCENT_75,
        Stage6MotionState.ASCENDING_75,
    ),
    Stage6MotionState.ASCENDING_75: (
        DescentLevel.DESCENT_50,
        Stage6MotionState.ASCENDING_50,
    ),
    Stage6MotionState.ASCENDING_50: (
        DescentLevel.DESCENT_25,
        Stage6MotionState.ASCENDING_25,
    ),
    Stage6MotionState.ASCENDING_25: (
        DescentLevel.ABOVE,
        Stage6MotionState.RETURNED_ABOVE,
    ),
}


@dataclass(frozen=True)
class Stage6StateSnapshot:
    state: Stage6MotionState
    locked_target: tuple[int, int] | None
    below_above: bool
    emergency_stopped: bool

    @property
    def lock_label(self) -> str:
        if self.below_above and self.locked_target is not None:
            row, col = self.locked_target
            return f"BELOW_ABOVE_LOCKED_TO_P({row},{col})"
        return "ABOVE_OR_HIGH_UNLOCKED"


class Stage6StateMachine:
    def __init__(self) -> None:
        self.state = Stage6MotionState.OBSERVE
        self.locked_target: tuple[int, int] | None = None

    @property
    def below_above(self) -> bool:
        return self.state in {
            Stage6MotionState.DESCENDING_25,
            Stage6MotionState.DESCENDING_50,
            Stage6MotionState.DESCENDING_75,
            Stage6MotionState.TARGET_TOUCH,
            Stage6MotionState.RELEASING,
            Stage6MotionState.RELEASE_DWELL,
            Stage6MotionState.ASCENDING_75,
            Stage6MotionState.ASCENDING_50,
            Stage6MotionState.ASCENDING_25,
        }

    def establish_above(self, row: int, col: int) -> None:
        self._require_not_estopped()
        target = (int(row), int(col))
        if self.locked_target is not None and self.locked_target != target:
            raise Stage6TransitionError(self.snapshot().lock_label)
        if self.state not in {
            Stage6MotionState.CARRY_HIGH,
            Stage6MotionState.TARGET_ABOVE,
            Stage6MotionState.RETURNED_ABOVE,
        }:
            raise Stage6TransitionError(
                f"TARGET_ABOVE requires CARRY_HIGH, not {self.state.value}"
            )
        self.locked_target = target
        self.state = Stage6MotionState.TARGET_ABOVE

    def establish_carry_high(self) -> None:
        """Explicitly synchronize a physically confirmed high transit pose."""
        self._require_not_estopped()
        if self.below_above:
            raise Stage6TransitionError(self.snapshot().lock_label)
        if self.state not in {
            Stage6MotionState.OBSERVE,
            Stage6MotionState.CARRY_HIGH,
        }:
            raise Stage6TransitionError(
                f"cannot establish CARRY_HIGH from {self.state.value}"
            )
        self.state = Stage6MotionState.CARRY_HIGH
        self.locked_target = None

    def require_target(self, row: int, col: int) -> None:
        target = (int(row), int(col))
        if self.below_above and self.locked_target != target:
            raise Stage6TransitionError(self.snapshot().lock_label)
        if self.locked_target is not None and self.state not in {
            Stage6MotionState.OBSERVE,
            Stage6MotionState.CARRY_HIGH,
            Stage6MotionState.RETURNED_ABOVE,
        } and self.locked_target != target:
            raise Stage6TransitionError(f"active target is P{self.locked_target}")

    def descend(self, row: int, col: int, level: DescentLevel) -> None:
        self._require_not_estopped()
        self.require_target(row, col)
        expected = DESCENT_TRANSITIONS.get(self.state)
        if expected is None or expected[0] != level:
            raise Stage6TransitionError(
                f"illegal descent {self.state.value} -> {level.value}"
            )
        self.state = expected[1]

    def begin_release(self) -> None:
        if self.state != Stage6MotionState.TARGET_TOUCH:
            raise Stage6TransitionError("release requires TARGET_TOUCH")
        self.state = Stage6MotionState.RELEASING

    def begin_release_dwell(self) -> None:
        if self.state != Stage6MotionState.RELEASING:
            raise Stage6TransitionError("release dwell requires RELEASING")
        self.state = Stage6MotionState.RELEASE_DWELL

    def ascend(self, row: int, col: int, level: DescentLevel) -> None:
        self._require_not_estopped()
        self.require_target(row, col)
        expected = ASCENT_TRANSITIONS.get(self.state)
        if expected is None or expected[0] != level:
            raise Stage6TransitionError(
                f"illegal ascent {self.state.value} -> {level.value}"
            )
        self.state = expected[1]

    def move_to_carry_high(self) -> None:
        if self.state not in {
            Stage6MotionState.TARGET_ABOVE,
            Stage6MotionState.RETURNED_ABOVE,
        }:
            raise Stage6TransitionError("CARRY_HIGH is allowed only from ABOVE")
        self.state = Stage6MotionState.CARRY_HIGH
        self.locked_target = None

    def move_to_observe(self) -> None:
        if self.state not in {
            Stage6MotionState.CARRY_HIGH,
            Stage6MotionState.RETURNED_ABOVE,
            Stage6MotionState.OBSERVE,
        }:
            raise Stage6TransitionError("OBSERVE is forbidden below ABOVE")
        self.state = Stage6MotionState.OBSERVE
        self.locked_target = None

    def emergency_stop(self) -> None:
        self.state = Stage6MotionState.EMERGENCY_STOP

    def recover_pose(self, state: Stage6MotionState, row: int, col: int) -> None:
        if self.state != Stage6MotionState.EMERGENCY_STOP:
            raise Stage6TransitionError("recovery is only valid after emergency stop")
        if state not in set(Stage6MotionState) - {Stage6MotionState.EMERGENCY_STOP}:
            raise Stage6TransitionError("invalid recovery state")
        self.state = state
        self.locked_target = (int(row), int(col)) if state not in {
            Stage6MotionState.CARRY_HIGH,
            Stage6MotionState.OBSERVE,
        } else None

    def snapshot(self) -> Stage6StateSnapshot:
        return Stage6StateSnapshot(
            state=self.state,
            locked_target=self.locked_target,
            below_above=self.below_above,
            emergency_stopped=self.state == Stage6MotionState.EMERGENCY_STOP,
        )

    def _require_not_estopped(self) -> None:
        if self.state == Stage6MotionState.EMERGENCY_STOP:
            raise Stage6TransitionError("EMERGENCY_STOP requires explicit pose recovery")
