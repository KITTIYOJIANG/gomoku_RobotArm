from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Mapping

from app.arm.actions import ActionLibrary


SPATIAL_JOINT_IDS = (0, 1, 2, 3, 4)
PUMP_JOINT_ID = 5
REFERENCE_ACTIONS = (
    "HOME_IDLE",
    "OBSERVE_IDLE",
    "OBSERVE_HOLD",
    "SOURCE_TOUCH_IDLE",
    "SOURCE_TOUCH_HOLD",
    "CARRY_HIGH_P77_IDLE",
    "CARRY_HIGH_P77_HOLD",
    "P77_ABOVE_IDLE",
    "P77_ABOVE_HOLD",
    "P77_TOUCH_HOLD",
    "P77_TOUCH_RELEASE",
)


@dataclass(frozen=True)
class PwmSafetyLimits:
    pwm_min: int
    pwm_max: int
    joint_min: dict[int, int]
    joint_max: dict[int, int]
    max_adjacent_delta: dict[int, int]

    def contains(self, joint_id: int, pwm: int) -> bool:
        if joint_id not in self.joint_min:
            return self.pwm_min <= pwm <= self.pwm_max
        return self.joint_min[joint_id] <= pwm <= self.joint_max[joint_id]


def derive_pwm_safety_limits(library: ActionLibrary, *, board_span_cells: int = 8) -> PwmSafetyLimits:
    """Derive PWM envelopes and adjacent-cell continuity thresholds from known poses."""
    joint_values: dict[int, list[int]] = {jid: [] for jid in SPATIAL_JOINT_IDS}
    for name in REFERENCE_ACTIONS:
        try:
            action = library.get(name)
        except KeyError:
            continue
        for jid in SPATIAL_JOINT_IDS:
            joint_values[jid].append(int(action.target(jid).pwm))

    joint_min: dict[int, int] = {}
    joint_max: dict[int, int] = {}
    max_adjacent: dict[int, int] = {}
    span = max(1, int(board_span_cells))
    for jid, values in joint_values.items():
        if not values:
            joint_min[jid] = library.pwm_min
            joint_max[jid] = library.pwm_max
            max_adjacent[jid] = 80
            continue
        lo = min(values)
        hi = max(values)
        margin = max(20, int(math.ceil((hi - lo) * 0.15)) if hi > lo else 40)
        joint_min[jid] = max(library.pwm_min, lo - margin)
        joint_max[jid] = min(library.pwm_max, hi + margin)
        pose_range = hi - lo
        per_cell = max(15, int(math.ceil(pose_range / span * 1.75))) if pose_range else 30
        max_adjacent[jid] = per_cell
    return PwmSafetyLimits(
        pwm_min=library.pwm_min,
        pwm_max=library.pwm_max,
        joint_min=joint_min,
        joint_max=joint_max,
        max_adjacent_delta=max_adjacent,
    )


def validate_spatial_pwm(pwm: Mapping[int | str, int], limits: PwmSafetyLimits) -> list[str]:
    errors: list[str] = []
    for jid in SPATIAL_JOINT_IDS:
        if jid in pwm:
            key: int | str = jid
        elif f"{jid:03d}" in pwm:
            key = f"{jid:03d}"
        else:
            errors.append(f"missing joint {jid:03d}")
            continue
        value = int(pwm[key])
        if not limits.contains(jid, value):
            errors.append(
                f"joint {jid:03d} PWM {value} outside safe range "
                f"{limits.joint_min[jid]}..{limits.joint_max[jid]}"
            )
    return errors
