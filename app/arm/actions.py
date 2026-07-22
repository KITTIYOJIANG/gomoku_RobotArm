from __future__ import annotations

from dataclasses import dataclass
import json
import logging
from pathlib import Path
import re
from typing import Iterable

from app.config import PROJECT_ROOT


LOGGER = logging.getLogger(__name__)
SERVO_PATTERN = re.compile(r"#(?P<id>\d{3})P(?P<pwm>\d{4})T(?P<time>\d{4})!")


@dataclass(frozen=True)
class ServoTarget:
    servo_id: int
    pwm: int
    time_ms: int


@dataclass(frozen=True)
class Action:
    name: str
    command: str
    targets: tuple[ServoTarget, ...]

    @property
    def duration_ms(self) -> int:
        return max(target.time_ms for target in self.targets)

    def target(self, servo_id: int) -> ServoTarget:
        for target in self.targets:
            if target.servo_id == servo_id:
                return target
        raise KeyError(f"Action {self.name} does not contain servo {servo_id:03d}")

    def describe(self) -> str:
        details = ", ".join(
            f"{item.servo_id:03d}=P{item.pwm:04d}/T{item.time_ms:04d}"
            for item in self.targets
        )
        return f"{self.name}: {details}"


class ActionLibrary:
    """Loads and validates the single authoritative V0.1 action table."""

    def __init__(self, path: str | Path | None = None) -> None:
        self.path = Path(path) if path is not None else PROJECT_ROOT / "config" / "arm_actions.json"
        data = json.loads(self.path.read_text(encoding="utf-8"))
        self.pwm_min = int(data["pwm_min"])
        self.pwm_max = int(data["pwm_max"])
        self.time_min_ms = int(data["time_min_ms"])
        self.time_max_ms = int(data["time_max_ms"])
        self._actions = {
            str(name).upper(): self._parse(str(name).upper(), str(command))
            for name, command in data["actions"].items()
        }
        if not self._actions:
            raise ValueError("Action table is empty")

    @property
    def names(self) -> tuple[str, ...]:
        return tuple(self._actions)

    def __iter__(self) -> Iterable[Action]:
        return iter(self._actions.values())

    def get(self, name: str) -> Action:
        key = name.strip().upper()
        try:
            action = self._actions[key]
        except KeyError as exc:
            raise KeyError(f"Unknown arm action: {name}") from exc
        LOGGER.info("ACTION %s", action.describe())
        return action

    def _parse(self, name: str, command: str) -> Action:
        if not command.startswith("{") or not command.endswith("}"):
            raise ValueError(f"{name}: multi-servo command must have complete braces")
        if not command.isascii():
            raise ValueError(f"{name}: command is not ASCII")
        body = command[1:-1]
        matches = list(SERVO_PATTERN.finditer(body))
        if not matches or "".join(match.group(0) for match in matches) != body:
            raise ValueError(f"{name}: malformed servo command")

        targets = tuple(
            ServoTarget(
                servo_id=int(match.group("id")),
                pwm=int(match.group("pwm")),
                time_ms=int(match.group("time")),
            )
            for match in matches
        )
        ids = [target.servo_id for target in targets]
        if ids != list(range(8)):
            raise ValueError(f"{name}: expected servo IDs 000..007 exactly once, got {ids}")
        for target in targets:
            if not self.pwm_min <= target.pwm <= self.pwm_max:
                raise ValueError(
                    f"{name}: servo {target.servo_id:03d} PWM {target.pwm} outside "
                    f"{self.pwm_min}..{self.pwm_max}"
                )
            if not self.time_min_ms <= target.time_ms <= self.time_max_ms:
                raise ValueError(
                    f"{name}: servo {target.servo_id:03d} time {target.time_ms} outside "
                    f"{self.time_min_ms}..{self.time_max_ms}"
                )
        return Action(name=name, command=command, targets=targets)
