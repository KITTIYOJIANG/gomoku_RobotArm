from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Mapping


SPATIAL_KEYS = ("000", "001", "002", "003", "004")


def normalize_spatial_pwm(values: Mapping[str | int, int]) -> dict[str, int]:
    normalized = {f"{int(key):03d}": int(value) for key, value in values.items()}
    missing = [key for key in SPATIAL_KEYS if key not in normalized]
    if missing:
        raise ValueError(f"missing spatial PWM: {missing}")
    return {key: normalized[key] for key in SPATIAL_KEYS}


class DescentLevel(str, Enum):
    ABOVE = "above"
    DESCENT_25 = "descent_25"
    DESCENT_50 = "descent_50"
    DESCENT_75 = "descent_75"
    TOUCH = "touch"

    @property
    def fraction(self) -> float:
        return {
            DescentLevel.ABOVE: 0.0,
            DescentLevel.DESCENT_25: 0.25,
            DescentLevel.DESCENT_50: 0.50,
            DescentLevel.DESCENT_75: 0.75,
            DescentLevel.TOUCH: 1.0,
        }[self]


DESCENT_LEVELS = (
    DescentLevel.ABOVE,
    DescentLevel.DESCENT_25,
    DescentLevel.DESCENT_50,
    DescentLevel.DESCENT_75,
    DescentLevel.TOUCH,
)


class LevelStatus(str, Enum):
    COMPUTED = "COMPUTED"
    TESTED = "TESTED"
    VERIFIED = "VERIFIED"
    REJECTED = "REJECTED"


class VerificationStage(str, Enum):
    COMPUTED = "COMPUTED"
    DRY_RUN_PASSED = "DRY_RUN_PASSED"
    EMPTY_TOOL_TESTED = "EMPTY_TOOL_TESTED"
    VERTICAL_VERIFIED = "VERTICAL_VERIFIED"
    RELEASE_VERIFIED = "RELEASE_VERIFIED"
    FULLY_VERIFIED = "FULLY_VERIFIED"


@dataclass(frozen=True)
class ToolPose:
    x: float
    y: float
    z: float
    alpha: float
    auxiliary_004_pwm: int = 1500

    def to_dict(self) -> dict[str, float | int]:
        return {
            "x": float(self.x),
            "y": float(self.y),
            "z": float(self.z),
            "alpha": float(self.alpha),
            "auxiliary_004_pwm": int(self.auxiliary_004_pwm),
        }


@dataclass(frozen=True)
class DescentLevelPose:
    level: DescentLevel
    source: str
    tool_pose: ToolPose
    computed_pwm: dict[str, int]
    manual_delta_pwm: dict[str, int] = field(
        default_factory=lambda: {key: 0 for key in SPATIAL_KEYS}
    )
    status: LevelStatus = LevelStatus.COMPUTED
    warnings: tuple[str, ...] = ()

    @property
    def final_pwm(self) -> dict[str, int]:
        return {
            key: int(self.computed_pwm[key]) + int(self.manual_delta_pwm.get(key, 0))
            for key in SPATIAL_KEYS
        }

    def to_dict(self) -> dict[str, Any]:
        return {
            "level": self.level.value,
            "source": self.source,
            "tool_pose": self.tool_pose.to_dict(),
            "computed_pwm": dict(self.computed_pwm),
            "manual_delta_pwm": dict(self.manual_delta_pwm),
            "final_pwm": self.final_pwm,
            "status": self.status.value,
            "verified": self.status == LevelStatus.VERIFIED,
            "warnings": list(self.warnings),
        }


@dataclass(frozen=True)
class DescentProfile:
    row: int
    col: int
    above_source: str
    above_verified: bool
    levels: tuple[DescentLevelPose, ...]
    reverse_ascent: tuple[DescentLevel, ...]
    model_valid: bool
    verification_stage: VerificationStage = VerificationStage.COMPUTED
    reverse_ascent_verified: bool = False
    warnings: tuple[str, ...] = ()

    def level(self, level: DescentLevel | str) -> DescentLevelPose:
        target = level if isinstance(level, DescentLevel) else DescentLevel(level)
        for item in self.levels:
            if item.level == target:
                return item
        raise KeyError(target.value)

    @property
    def verified(self) -> bool:
        return self.verification_stage == VerificationStage.FULLY_VERIFIED

    def to_dict(self) -> dict[str, Any]:
        return {
            "target": {"row": int(self.row), "col": int(self.col)},
            "above": self.level(DescentLevel.ABOVE).to_dict(),
            "levels": [item.to_dict() for item in self.levels],
            "touch": self.level(DescentLevel.TOUCH).to_dict(),
            "reverse_ascent": [level.value for level in self.reverse_ascent],
            "model_valid": bool(self.model_valid),
            "above_source": self.above_source,
            "above_verified": bool(self.above_verified),
            "verification_stage": self.verification_stage.value,
            "verified": self.verified,
            "reverse_ascent_verified": bool(self.reverse_ascent_verified),
            "warnings": list(self.warnings),
        }
