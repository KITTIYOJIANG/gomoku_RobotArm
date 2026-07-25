from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import threading


class ArmState(str, Enum):
    DISCONNECTED = "DISCONNECTED"
    UNKNOWN = "UNKNOWN"
    MOVING_TO_OBSERVE = "MOVING_TO_OBSERVE"
    OBSERVE_IDLE = "OBSERVE_IDLE"
    PICKING = "PICKING"
    OBSERVE_HOLD = "OBSERVE_HOLD"
    PLACING_P77 = "PLACING_P77"
    MOVING_TO_HOVER = "MOVING_TO_HOVER"
    HOVERING = "HOVERING"
    RETURNING_FROM_HOVER = "RETURNING_FROM_HOVER"
    ERROR = "ERROR"
    ESTOP = "ESTOP"


class InvalidTransition(RuntimeError):
    pass


@dataclass(frozen=True)
class StateSnapshot:
    state: ArmState
    busy: bool
    current_action: str | None
    error: str | None


class ArmStateMachine:
    """Explicit V0.1 arm state with latched ERROR/ESTOP recovery."""

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._state = ArmState.DISCONNECTED
        self._busy = False
        self._current_action: str | None = None
        self._error: str | None = None

    @property
    def state(self) -> ArmState:
        with self._lock:
            return self._state

    @property
    def busy(self) -> bool:
        with self._lock:
            return self._busy

    def snapshot(self) -> StateSnapshot:
        with self._lock:
            return StateSnapshot(self._state, self._busy, self._current_action, self._error)

    def connect(self) -> tuple[ArmState, ArmState]:
        with self._lock:
            if self._state != ArmState.DISCONNECTED:
                raise InvalidTransition(f"Cannot connect from {self._state.value}")
            return self._set_state(ArmState.UNKNOWN)

    def disconnect(self) -> tuple[ArmState, ArmState]:
        with self._lock:
            self._busy = False
            self._current_action = None
            self._error = None
            return self._set_state(ArmState.DISCONNECTED)

    def can_return_to_observe(self) -> bool:
        with self._lock:
            return not self._busy and self._state != ArmState.DISCONNECTED

    def can_pick(self) -> bool:
        with self._lock:
            return not self._busy and self._state == ArmState.OBSERVE_IDLE

    def can_place(self, *, board_locked: bool, target_visible: bool = True) -> bool:
        with self._lock:
            return (
                not self._busy
                and self._state == ArmState.OBSERVE_HOLD
                and bool(board_locked)
                and bool(target_visible)
            )

    def begin_return_to_observe(self) -> tuple[ArmState, ArmState]:
        with self._lock:
            self._require_idle()
            if self._state == ArmState.DISCONNECTED:
                raise InvalidTransition("Connect COM before returning to observe")
            self._busy = True
            self._current_action = "RETURN_TO_OBSERVE"
            self._error = None
            return self._set_state(ArmState.MOVING_TO_OBSERVE)

    def complete_return_to_observe(self) -> tuple[ArmState, ArmState]:
        return self._complete("RETURN_TO_OBSERVE", ArmState.MOVING_TO_OBSERVE, ArmState.OBSERVE_IDLE)

    def begin_pick(self) -> tuple[ArmState, ArmState]:
        with self._lock:
            self._require_idle()
            if self._state != ArmState.OBSERVE_IDLE:
                raise InvalidTransition("Pick requires OBSERVE_IDLE")
            self._busy = True
            self._current_action = "PICK_PIECE"
            return self._set_state(ArmState.PICKING)

    def complete_pick(self) -> tuple[ArmState, ArmState]:
        return self._complete("PICK_PIECE", ArmState.PICKING, ArmState.OBSERVE_HOLD)

    def begin_place(
        self,
        *,
        board_locked: bool,
        target_visible: bool,
    ) -> tuple[ArmState, ArmState]:
        with self._lock:
            self._require_idle()
            if self._state != ArmState.OBSERVE_HOLD:
                raise InvalidTransition("Place to P77 requires OBSERVE_HOLD")
            if not board_locked or not target_visible:
                raise InvalidTransition("Place to P77 requires BOARD LOCKED and visible P77")
            self._busy = True
            self._current_action = "PLACE_TO_P77"
            return self._set_state(ArmState.PLACING_P77)

    def complete_place(self) -> tuple[ArmState, ArmState]:
        return self._complete("PLACE_TO_P77", ArmState.PLACING_P77, ArmState.OBSERVE_IDLE)

    def begin_full_cycle(
        self,
        *,
        board_locked: bool,
        target_visible: bool,
    ) -> tuple[ArmState, ArmState]:
        with self._lock:
            self._require_idle()
            if self._state != ArmState.OBSERVE_IDLE:
                raise InvalidTransition("Full cycle requires OBSERVE_IDLE")
            if not board_locked or not target_visible:
                raise InvalidTransition("Full cycle requires BOARD LOCKED and visible P77")
            self._busy = True
            self._current_action = "FULL_CYCLE"
            return self._set_state(ArmState.PICKING)

    def mark_full_cycle_placing(self) -> tuple[ArmState, ArmState]:
        with self._lock:
            if not self._busy or self._current_action != "FULL_CYCLE" or self._state != ArmState.PICKING:
                raise InvalidTransition("Full cycle is not in its picking phase")
            return self._set_state(ArmState.PLACING_P77)

    def complete_full_cycle(self) -> tuple[ArmState, ArmState]:
        return self._complete("FULL_CYCLE", ArmState.PLACING_P77, ArmState.OBSERVE_IDLE)

    def begin_manual(self, action_name: str) -> None:
        with self._lock:
            self._require_idle()
            if self._state == ArmState.DISCONNECTED:
                raise InvalidTransition("Connect COM before manual movement")
            self._busy = True
            self._current_action = f"MANUAL:{action_name}"

    def complete_manual(self) -> tuple[ArmState, ArmState]:
        with self._lock:
            if not self._busy or not (self._current_action or "").startswith("MANUAL:"):
                raise InvalidTransition("No manual action is active")
            self._busy = False
            self._current_action = None
            self._error = None
            return self._set_state(ArmState.UNKNOWN)


    def can_hover(self, *, board_locked: bool) -> bool:
        with self._lock:
            return (
                not self._busy
                and self._state in {ArmState.OBSERVE_IDLE, ArmState.OBSERVE_HOLD, ArmState.HOVERING}
                and bool(board_locked)
            )

    def begin_hover(self, *, board_locked: bool, holding_piece: bool | None = None) -> tuple[ArmState, ArmState]:
        with self._lock:
            self._require_idle()
            if self._state not in {ArmState.OBSERVE_IDLE, ArmState.OBSERVE_HOLD}:
                raise InvalidTransition("Hover requires OBSERVE_IDLE or OBSERVE_HOLD")
            if not board_locked:
                raise InvalidTransition("Hover requires BOARD LOCKED")
            self._busy = True
            self._current_action = "HOVER_TO_TARGET"
            self._error = None
            return self._set_state(ArmState.MOVING_TO_HOVER)

    def complete_hover(self) -> tuple[ArmState, ArmState]:
        return self._complete("HOVER_TO_TARGET", ArmState.MOVING_TO_HOVER, ArmState.HOVERING)

    def begin_return_from_hover(self) -> tuple[ArmState, ArmState]:
        with self._lock:
            self._require_idle()
            if self._state != ArmState.HOVERING:
                raise InvalidTransition("Safe return-from-hover requires HOVERING")
            self._busy = True
            self._current_action = "SAFE_RETURN_FROM_HOVER"
            self._error = None
            return self._set_state(ArmState.RETURNING_FROM_HOVER)

    def complete_return_from_hover(self, *, holding_piece: bool) -> tuple[ArmState, ArmState]:
        final = ArmState.OBSERVE_HOLD if holding_piece else ArmState.OBSERVE_IDLE
        return self._complete("SAFE_RETURN_FROM_HOVER", ArmState.RETURNING_FROM_HOVER, final)
    def mark_observe_idle(self) -> tuple[ArmState, ArmState]:
        """Software mark after a sequence that already commanded OBSERVE_IDLE."""
        with self._lock:
            self._busy = False
            self._current_action = None
            self._error = None
            return self._set_state(ArmState.OBSERVE_IDLE)

    def estop(self) -> tuple[ArmState, ArmState]:
        with self._lock:
            self._busy = False
            self._current_action = None
            self._error = "Emergency stop latched; user recovery required"
            return self._set_state(ArmState.ESTOP)

    def fail(self, message: str) -> tuple[ArmState, ArmState]:
        with self._lock:
            self._busy = False
            self._current_action = None
            self._error = str(message)
            if self._state == ArmState.DISCONNECTED:
                return self._state, self._state
            return self._set_state(ArmState.ERROR)

    def _complete(
        self,
        expected_action: str,
        expected_state: ArmState,
        final_state: ArmState,
    ) -> tuple[ArmState, ArmState]:
        with self._lock:
            if not self._busy or self._current_action != expected_action or self._state != expected_state:
                raise InvalidTransition(f"{expected_action} is not active")
            self._busy = False
            self._current_action = None
            self._error = None
            return self._set_state(final_state)

    def _require_idle(self) -> None:
        if self._busy:
            raise InvalidTransition("Another arm action is already running")

    def _set_state(self, target: ArmState) -> tuple[ArmState, ArmState]:
        previous = self._state
        self._state = target
        return previous, target

