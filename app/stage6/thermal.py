from __future__ import annotations

from dataclasses import dataclass
from time import monotonic


class ThermalLockout(RuntimeError):
    pass


@dataclass(frozen=True)
class ThermalSnapshot:
    overheated: bool
    continuous_actions: int
    tweaks: int
    dwell_warning: str | None


class ThermalGuard:
    def __init__(
        self,
        *,
        max_above_dwell_seconds: float,
        max_touch_dwell_seconds: float,
        max_tweaks_per_session: int,
        max_continuous_actions: int,
    ) -> None:
        self.max_above_dwell_seconds = float(max_above_dwell_seconds)
        self.max_touch_dwell_seconds = float(max_touch_dwell_seconds)
        self.max_tweaks_per_session = int(max_tweaks_per_session)
        self.max_continuous_actions = int(max_continuous_actions)
        self.overheated = False
        self.continuous_actions = 0
        self.tweaks = 0
        self._dwell_kind: str | None = None
        self._dwell_started: float | None = None

    def require_available(self) -> None:
        if self.overheated:
            raise ThermalLockout(
                "OVERHEAT_LOCKED: support the arm, power it off, and allow it to cool"
            )
        if self.continuous_actions >= self.max_continuous_actions:
            raise ThermalLockout("COOLING_REQUIRED: continuous action limit reached")

    def record_action(self) -> None:
        self.require_available()
        self.continuous_actions += 1

    def record_tweak(self) -> None:
        self.require_available()
        if self.tweaks >= self.max_tweaks_per_session:
            raise ThermalLockout("COOLING_REQUIRED: tweak limit reached")
        self.tweaks += 1

    def enter_dwell(self, kind: str) -> None:
        self._dwell_kind = str(kind).upper()
        self._dwell_started = monotonic()

    def leave_dwell(self) -> None:
        self._dwell_kind = None
        self._dwell_started = None

    def dwell_warning(self, *, now: float | None = None) -> str | None:
        if self._dwell_started is None or self._dwell_kind is None:
            return None
        elapsed = (monotonic() if now is None else float(now)) - self._dwell_started
        limit = (
            self.max_touch_dwell_seconds
            if self._dwell_kind == "TOUCH"
            else self.max_above_dwell_seconds
        )
        if elapsed > limit:
            return (
                f"{self._dwell_kind}_DWELL_TIMEOUT: {elapsed:.1f}s > {limit:.1f}s; "
                "request a user-confirmed safe return"
            )
        return None

    def report_overheat(self) -> None:
        self.overheated = True

    def reset_after_physical_cooldown(self, *, user_confirmed: bool) -> None:
        if not user_confirmed:
            raise ThermalLockout("physical cooldown confirmation required")
        self.overheated = False
        self.continuous_actions = 0
        self.tweaks = 0
        self.leave_dwell()

    def snapshot(self) -> ThermalSnapshot:
        return ThermalSnapshot(
            overheated=self.overheated,
            continuous_actions=self.continuous_actions,
            tweaks=self.tweaks,
            dwell_warning=self.dwell_warning(),
        )
