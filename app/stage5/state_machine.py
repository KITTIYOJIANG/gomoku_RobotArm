from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import threading


class Stage5State(str, Enum):
    DISCONNECTED = "DISCONNECTED"
    IDLE = "IDLE"
    BOARD_NOT_LOCKED = "BOARD_NOT_LOCKED"
    READY = "READY"
    TARGET_SELECTED = "TARGET_SELECTED"
    TARGET_UNCALIBRATED = "TARGET_UNCALIBRATED"
    TARGET_READY = "TARGET_READY"
    DRY_RUN_READY = "DRY_RUN_READY"
    PRE_MOVE_CHECK = "PRE_MOVE_CHECK"
    MOVING_TO_CARRY_HIGH = "MOVING_TO_CARRY_HIGH"
    MOVING_TO_TARGET_ABOVE = "MOVING_TO_TARGET_ABOVE"
    HOVERING = "HOVERING"
    RETURNING_TO_CARRY_HIGH = "RETURNING_TO_CARRY_HIGH"
    RETURNING_TO_OBSERVE = "RETURNING_TO_OBSERVE"
    EMERGENCY_STOP = "EMERGENCY_STOP"
    ERROR = "ERROR"


class Stage5Invalid(RuntimeError):
    pass


@dataclass(frozen=True)
class Stage5Snapshot:
    state: Stage5State
    target_row: int | None
    target_col: int | None
    dry_run: bool
    holding_piece: bool
    error: str | None


class Stage5StateMachine:
    """Hover workflow state machine. Does not send serial commands itself."""

    def __init__(self, *, dry_run: bool = True) -> None:
        self._lock = threading.RLock()
        self._state = Stage5State.DISCONNECTED
        self._target_row: int | None = None
        self._target_col: int | None = None
        self._dry_run = bool(dry_run)
        self._holding_piece = False
        self._error: str | None = None

    def snapshot(self) -> Stage5Snapshot:
        with self._lock:
            return Stage5Snapshot(
                state=self._state,
                target_row=self._target_row,
                target_col=self._target_col,
                dry_run=self._dry_run,
                holding_piece=self._holding_piece,
                error=self._error,
            )

    @property
    def state(self) -> Stage5State:
        return self.snapshot().state

    def set_dry_run(self, enabled: bool) -> None:
        with self._lock:
            if self._is_moving_locked():
                raise Stage5Invalid("Cannot change DRY RUN while moving")
            self._dry_run = bool(enabled)
            self._recompute_target_state()

    def on_serial_connected(self, *, board_locked: bool) -> Stage5State:
        with self._lock:
            if self._state == Stage5State.EMERGENCY_STOP:
                return self._state
            self._error = None
            if board_locked:
                self._state = Stage5State.READY if self._target_row is None else self._state
                if self._target_row is not None:
                    self._recompute_target_state()
                else:
                    self._state = Stage5State.READY
            else:
                self._state = Stage5State.BOARD_NOT_LOCKED
            return self._state

    def on_serial_disconnected(self) -> Stage5State:
        with self._lock:
            self._state = Stage5State.DISCONNECTED
            self._error = None
            return self._state

    def on_board_lock_changed(self, locked: bool) -> Stage5State:
        with self._lock:
            if self._state in {
                Stage5State.DISCONNECTED,
                Stage5State.EMERGENCY_STOP,
                Stage5State.MOVING_TO_CARRY_HIGH,
                Stage5State.MOVING_TO_TARGET_ABOVE,
                Stage5State.RETURNING_TO_CARRY_HIGH,
                Stage5State.RETURNING_TO_OBSERVE,
                Stage5State.HOVERING,
            }:
                return self._state
            if not locked:
                self._state = Stage5State.BOARD_NOT_LOCKED
                return self._state
            if self._target_row is None:
                self._state = Stage5State.READY
            else:
                self._recompute_target_state()
            return self._state

    def clear_target(self) -> Stage5State:
        with self._lock:
            if self._is_moving_locked():
                raise Stage5Invalid("Cannot clear target while moving")
            self._target_row = None
            self._target_col = None
            if self._state == Stage5State.DISCONNECTED:
                return self._state
            if self._state == Stage5State.EMERGENCY_STOP:
                return self._state
            self._state = Stage5State.READY
            self._error = None
            return self._state

    def select_target(
        self,
        row: int,
        col: int,
        *,
        board_locked: bool,
        calibrated: bool,
        in_region: bool,
    ) -> Stage5State:
        with self._lock:
            if self._state == Stage5State.DISCONNECTED:
                raise Stage5Invalid("Serial not connected")
            if self._state == Stage5State.EMERGENCY_STOP:
                raise Stage5Invalid("Emergency stop latched")
            if self._is_moving_locked():
                raise Stage5Invalid("Cannot change target while moving")
            if not board_locked:
                self._state = Stage5State.BOARD_NOT_LOCKED
                raise Stage5Invalid("BOARD LOCKED required to select target")
            self._target_row = int(row)
            self._target_col = int(col)
            self._error = None
            self._state = Stage5State.TARGET_SELECTED
            if not in_region or not calibrated:
                self._state = Stage5State.TARGET_UNCALIBRATED
            else:
                self._state = Stage5State.DRY_RUN_READY if self._dry_run else Stage5State.TARGET_READY
            return self._state

    def begin_hover(self, *, holding_piece: bool) -> Stage5State:
        with self._lock:
            if self._state not in {Stage5State.TARGET_READY, Stage5State.DRY_RUN_READY}:
                raise Stage5Invalid(f"Cannot hover from {self._state.value}")
            if self._target_row is None or self._target_col is None:
                raise Stage5Invalid("No target selected")
            self._holding_piece = bool(holding_piece)
            self._error = None
            self._state = Stage5State.PRE_MOVE_CHECK
            self._state = Stage5State.MOVING_TO_CARRY_HIGH
            return self._state

    def mark_moving_to_target(self) -> Stage5State:
        with self._lock:
            if self._state != Stage5State.MOVING_TO_CARRY_HIGH:
                raise Stage5Invalid("Not moving to carry-high")
            self._state = Stage5State.MOVING_TO_TARGET_ABOVE
            return self._state

    def complete_hover(self) -> Stage5State:
        with self._lock:
            if self._state not in {
                Stage5State.MOVING_TO_CARRY_HIGH,
                Stage5State.MOVING_TO_TARGET_ABOVE,
                Stage5State.PRE_MOVE_CHECK,
            }:
                raise Stage5Invalid(f"Cannot complete hover from {self._state.value}")
            self._state = Stage5State.HOVERING
            return self._state

    def begin_return(self) -> Stage5State:
        with self._lock:
            if self._state != Stage5State.HOVERING:
                raise Stage5Invalid("Safe return requires HOVERING")
            self._state = Stage5State.RETURNING_TO_CARRY_HIGH
            return self._state

    def mark_returning_to_observe(self) -> Stage5State:
        with self._lock:
            if self._state != Stage5State.RETURNING_TO_CARRY_HIGH:
                raise Stage5Invalid("Not returning via carry-high")
            self._state = Stage5State.RETURNING_TO_OBSERVE
            return self._state

    def complete_return(self, *, board_locked: bool) -> Stage5State:
        with self._lock:
            if self._state not in {
                Stage5State.RETURNING_TO_CARRY_HIGH,
                Stage5State.RETURNING_TO_OBSERVE,
            }:
                raise Stage5Invalid(f"Cannot complete return from {self._state.value}")
            self._holding_piece = False
            if board_locked and self._target_row is not None:
                self._recompute_target_state()
            elif board_locked:
                self._state = Stage5State.READY
            else:
                self._state = Stage5State.BOARD_NOT_LOCKED
            return self._state

    def estop(self) -> Stage5State:
        with self._lock:
            self._state = Stage5State.EMERGENCY_STOP
            self._error = "Emergency stop latched; user recovery required"
            return self._state

    def fail(self, message: str) -> Stage5State:
        with self._lock:
            self._state = Stage5State.ERROR
            self._error = str(message)
            return self._state

    def recover_from_estop(self, *, board_locked: bool, serial_connected: bool) -> Stage5State:
        with self._lock:
            if self._state not in {Stage5State.EMERGENCY_STOP, Stage5State.ERROR}:
                raise Stage5Invalid("Nothing to recover")
            self._error = None
            self._target_row = None
            self._target_col = None
            if not serial_connected:
                self._state = Stage5State.DISCONNECTED
            elif board_locked:
                self._state = Stage5State.READY
            else:
                self._state = Stage5State.BOARD_NOT_LOCKED
            return self._state

    def can_execute_hover(self) -> bool:
        snap = self.snapshot()
        return snap.state in {Stage5State.TARGET_READY, Stage5State.DRY_RUN_READY}

    def can_safe_return(self) -> bool:
        return self.snapshot().state == Stage5State.HOVERING

    def is_moving(self) -> bool:
        return self.snapshot().state in {
            Stage5State.PRE_MOVE_CHECK,
            Stage5State.MOVING_TO_CARRY_HIGH,
            Stage5State.MOVING_TO_TARGET_ABOVE,
            Stage5State.RETURNING_TO_CARRY_HIGH,
            Stage5State.RETURNING_TO_OBSERVE,
        }

    def _is_moving_locked(self) -> bool:
        return self._state in {
            Stage5State.PRE_MOVE_CHECK,
            Stage5State.MOVING_TO_CARRY_HIGH,
            Stage5State.MOVING_TO_TARGET_ABOVE,
            Stage5State.RETURNING_TO_CARRY_HIGH,
            Stage5State.RETURNING_TO_OBSERVE,
        }

    def _recompute_target_state(self) -> None:
        # Caller holds lock. Requires target already selected; calibrated flags
        # are re-checked by the coordinator when selecting/executing.
        if self._target_row is None:
            self._state = Stage5State.READY
            return
        # Keep TARGET_UNCALIBRATED sticky until a fresh select_target refreshes it.
        if self._state == Stage5State.TARGET_UNCALIBRATED:
            return
        self._state = Stage5State.DRY_RUN_READY if self._dry_run else Stage5State.TARGET_READY
