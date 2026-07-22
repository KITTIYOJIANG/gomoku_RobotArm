from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import logging
import threading


LOGGER = logging.getLogger(__name__)


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


MOVING_STATES = {
    Stage5State.PRE_MOVE_CHECK,
    Stage5State.MOVING_TO_CARRY_HIGH,
    Stage5State.MOVING_TO_TARGET_ABOVE,
    Stage5State.RETURNING_TO_CARRY_HIGH,
    Stage5State.RETURNING_TO_OBSERVE,
}


@dataclass(frozen=True)
class Stage5Snapshot:
    state: Stage5State
    target_row: int | None
    target_col: int | None
    dry_run: bool
    holding_piece: bool
    error: str | None
    serial_connected: bool
    board_locked: bool
    arm_ready: bool


class Stage5StateMachine:
    """Hover workflow state machine. Does not send serial commands itself."""

    def __init__(self, *, dry_run: bool = True) -> None:
        self._lock = threading.RLock()
        self._state = Stage5State.DISCONNECTED
        self._target_row: int | None = None
        self._target_col: int | None = None
        self._target_calibrated: bool = False
        self._target_in_region: bool = False
        self._dry_run = bool(dry_run)
        self._holding_piece = False
        self._error: str | None = None
        self._serial_connected = False
        self._board_locked = False
        self._arm_ready = False
        self._estop_latched = False

    def snapshot(self) -> Stage5Snapshot:
        with self._lock:
            return Stage5Snapshot(
                state=self._state,
                target_row=self._target_row,
                target_col=self._target_col,
                dry_run=self._dry_run,
                holding_piece=self._holding_piece,
                error=self._error,
                serial_connected=self._serial_connected,
                board_locked=self._board_locked,
                arm_ready=self._arm_ready,
            )

    @property
    def state(self) -> Stage5State:
        return self.snapshot().state

    def set_dry_run(self, enabled: bool) -> None:
        with self._lock:
            if self._is_moving_locked():
                raise Stage5Invalid("Cannot change DRY RUN while moving")
            self._dry_run = bool(enabled)
            if self._target_row is not None and self._state not in MOVING_STATES | {
                Stage5State.HOVERING,
                Stage5State.EMERGENCY_STOP,
                Stage5State.DISCONNECTED,
            }:
                self._apply_target_state_locked()

    def sync_context(
        self,
        *,
        serial_connected: bool | None = None,
        board_locked: bool | None = None,
        arm_ready: bool | None = None,
        estop: bool | None = None,
    ) -> Stage5State:
        """Recompute stage5 state from main-system facts. Safe to call frequently."""
        with self._lock:
            previous = self._state
            if serial_connected is not None:
                self._serial_connected = bool(serial_connected)
            if board_locked is not None:
                self._board_locked = bool(board_locked)
            if arm_ready is not None:
                self._arm_ready = bool(arm_ready)
            if estop is not None:
                self._estop_latched = bool(estop)
            self._recompute_from_context_locked()
            if self._state != previous:
                LOGGER.info(
                    "[STAGE5][STATE] %s -> %s serial=%s board=%s arm_ready=%s estop=%s",
                    previous.value,
                    self._state.value,
                    int(self._serial_connected),
                    int(self._board_locked),
                    int(self._arm_ready),
                    int(self._estop_latched),
                )
            return self._state

    def on_serial_connected(self, *, board_locked: bool) -> Stage5State:
        LOGGER.info("[STAGE5][SERIAL_SYNC] connected=1")
        return self.sync_context(serial_connected=True, board_locked=board_locked, estop=False)

    def on_serial_disconnected(self) -> Stage5State:
        LOGGER.info("[STAGE5][SERIAL_SYNC] connected=0")
        with self._lock:
            self._serial_connected = False
            previous = self._state
            if not self._is_moving_locked():
                self._state = Stage5State.DISCONNECTED
                self._error = None
            else:
                # Still mark disconnected; motion path will fail safely.
                self._state = Stage5State.DISCONNECTED
                self._error = "Serial disconnected"
            if self._state != previous:
                LOGGER.info("[STAGE5][STATE] %s -> %s", previous.value, self._state.value)
            return self._state

    def on_board_lock_changed(self, locked: bool) -> Stage5State:
        LOGGER.info("[STAGE5][BOARD_SYNC] locked=%s", int(bool(locked)))
        return self.sync_context(board_locked=bool(locked))

    def on_arm_state(self, arm_state_name: str) -> Stage5State:
        ready = arm_state_name in {"OBSERVE_IDLE", "OBSERVE_HOLD"}
        estop = arm_state_name == "ESTOP"
        LOGGER.info("[STAGE5][ARM_SYNC] state=%s arm_ready=%s", arm_state_name, int(ready))
        return self.sync_context(arm_ready=ready, estop=estop if estop else None)

    def clear_target(self) -> Stage5State:
        with self._lock:
            if self._is_moving_locked():
                raise Stage5Invalid("Cannot clear target while moving")
            self._target_row = None
            self._target_col = None
            self._target_calibrated = False
            self._target_in_region = False
            previous = self._state
            self._recompute_from_context_locked()
            if self._state != previous:
                LOGGER.info("[STAGE5][STATE] %s -> %s", previous.value, self._state.value)
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
            if not self._serial_connected or self._state == Stage5State.DISCONNECTED:
                raise Stage5Invalid("Serial not connected")
            if self._estop_latched or self._state == Stage5State.EMERGENCY_STOP:
                raise Stage5Invalid("Emergency stop latched")
            if self._is_moving_locked():
                raise Stage5Invalid("Cannot change target while moving")
            self._board_locked = bool(board_locked)
            if not board_locked:
                self._state = Stage5State.BOARD_NOT_LOCKED
                raise Stage5Invalid("BOARD LOCKED required to select target")
            self._target_row = int(row)
            self._target_col = int(col)
            self._target_calibrated = bool(calibrated)
            self._target_in_region = bool(in_region)
            self._error = None
            previous = self._state
            self._apply_target_state_locked()
            if self._state != previous:
                LOGGER.info("[STAGE5][STATE] %s -> %s", previous.value, self._state.value)
            return self._state

    def begin_hover(self, *, holding_piece: bool) -> Stage5State:
        with self._lock:
            if self._state not in {Stage5State.TARGET_READY, Stage5State.DRY_RUN_READY}:
                raise Stage5Invalid(f"Cannot hover from {self._state.value}")
            if self._target_row is None or self._target_col is None:
                raise Stage5Invalid("No target selected")
            if not self._arm_ready:
                raise Stage5Invalid("Arm must be OBSERVE_IDLE or OBSERVE_HOLD")
            self._holding_piece = bool(holding_piece)
            self._error = None
            previous = self._state
            self._state = Stage5State.PRE_MOVE_CHECK
            self._state = Stage5State.MOVING_TO_CARRY_HIGH
            LOGGER.info("[STAGE5][STATE] %s -> %s", previous.value, self._state.value)
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
            previous = self._state
            self._state = Stage5State.HOVERING
            LOGGER.info("[STAGE5][STATE] %s -> %s", previous.value, self._state.value)
            return self._state

    def begin_return(self) -> Stage5State:
        with self._lock:
            if self._state != Stage5State.HOVERING:
                raise Stage5Invalid("Safe return requires HOVERING")
            previous = self._state
            self._state = Stage5State.RETURNING_TO_CARRY_HIGH
            LOGGER.info("[STAGE5][STATE] %s -> %s", previous.value, self._state.value)
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
            previous = self._state
            self._holding_piece = False
            self._board_locked = bool(board_locked)
            self._arm_ready = True
            self._recompute_from_context_locked()
            LOGGER.info("[STAGE5][STATE] %s -> %s", previous.value, self._state.value)
            return self._state

    def estop(self) -> Stage5State:
        with self._lock:
            previous = self._state
            self._estop_latched = True
            self._state = Stage5State.EMERGENCY_STOP
            self._error = "Emergency stop latched; user recovery required"
            LOGGER.info("[STAGE5][STATE] %s -> %s", previous.value, self._state.value)
            return self._state

    def fail(self, message: str) -> Stage5State:
        with self._lock:
            previous = self._state
            self._state = Stage5State.ERROR
            self._error = str(message)
            LOGGER.info("[STAGE5][STATE] %s -> %s", previous.value, self._state.value)
            return self._state

    def recover_from_estop(self, *, board_locked: bool, serial_connected: bool) -> Stage5State:
        with self._lock:
            if self._state not in {Stage5State.EMERGENCY_STOP, Stage5State.ERROR}:
                raise Stage5Invalid("Nothing to recover")
            previous = self._state
            self._error = None
            self._estop_latched = False
            self._target_row = None
            self._target_col = None
            self._target_calibrated = False
            self._target_in_region = False
            self._serial_connected = bool(serial_connected)
            self._board_locked = bool(board_locked)
            self._arm_ready = False
            self._recompute_from_context_locked()
            LOGGER.info("[STAGE5][STATE] %s -> %s", previous.value, self._state.value)
            return self._state

    def can_execute_hover(self) -> bool:
        snap = self.snapshot()
        return (
            snap.state in {Stage5State.TARGET_READY, Stage5State.DRY_RUN_READY}
            and snap.arm_ready
            and snap.serial_connected
            and snap.board_locked
        )

    def can_safe_return(self) -> bool:
        return self.snapshot().state == Stage5State.HOVERING

    def is_moving(self) -> bool:
        return self.snapshot().state in MOVING_STATES

    def _is_moving_locked(self) -> bool:
        return self._state in MOVING_STATES

    def _recompute_from_context_locked(self) -> None:
        """Caller holds lock. Do not clobber active motion/hover/error without cause."""
        if self._is_moving_locked() or self._state == Stage5State.HOVERING:
            return
        if self._state == Stage5State.ERROR and self._error:
            # Keep ERROR until explicit recover/new successful path.
            if self._serial_connected and not self._estop_latched:
                return
        if self._estop_latched:
            self._state = Stage5State.EMERGENCY_STOP
            return
        if not self._serial_connected:
            self._state = Stage5State.DISCONNECTED
            return
        if not self._board_locked:
            self._state = Stage5State.BOARD_NOT_LOCKED
            return
        if self._target_row is not None:
            self._apply_target_state_locked()
            return
        if self._arm_ready:
            self._state = Stage5State.READY
        else:
            # Connected + locked, but arm not yet confirmed at observe.
            self._state = Stage5State.IDLE

    def _apply_target_state_locked(self) -> None:
        if self._target_row is None:
            self._state = Stage5State.READY if self._arm_ready else Stage5State.IDLE
            return
        if not self._target_in_region or not self._target_calibrated:
            self._state = Stage5State.TARGET_UNCALIBRATED
            return
        self._state = Stage5State.DRY_RUN_READY if self._dry_run else Stage5State.TARGET_READY

    # Back-compat aliases used by older coordinator paths
    def _recompute_target_state(self) -> None:
        self._apply_target_state_locked()
