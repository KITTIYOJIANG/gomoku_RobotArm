from __future__ import annotations

from typing import Mapping


SPATIAL_JOINTS = ("J0", "J1", "J2", "J3", "J4")

# Hardware-verified P77 baseline:
#
# ABOVE = [1500, 1230, 870, 1230, 1500]
# DROP  = [1500, 1090, 790, 1370, 1500]
#
# DROP - ABOVE
P77_DROP_DELTA = {
    "J0": 0,
    "J1": -140,
    "J2": -80,
    "J3": 140,
    "J4": 0,
}


def normalize_pose(pwm: Mapping[str, int]) -> dict[str, int]:
    """Return a J0-J4 pose and reject incomplete input."""
    result: dict[str, int] = {}

    for joint in SPATIAL_JOINTS:
        if joint not in pwm:
            raise ValueError(f"missing joint: {joint}")

        result[joint] = int(pwm[joint])

    return result


def compute_delta(
    above_pwm: Mapping[str, int],
    drop_pwm: Mapping[str, int],
) -> dict[str, int]:
    """Compute joint-space DROP - ABOVE delta."""
    above = normalize_pose(above_pwm)
    drop = normalize_pose(drop_pwm)

    return {
        joint: drop[joint] - above[joint]
        for joint in SPATIAL_JOINTS
    }


def apply_delta(
    above_pwm: Mapping[str, int],
    delta_pwm: Mapping[str, int],
) -> dict[str, int]:
    """Apply a joint-space delta to ABOVE."""
    above = normalize_pose(above_pwm)
    delta = normalize_pose(delta_pwm)

    return {
        joint: above[joint] + delta[joint]
        for joint in SPATIAL_JOINTS
    }


def compute_residual(
    real_drop_pwm: Mapping[str, int],
    predicted_drop_pwm: Mapping[str, int],
) -> dict[str, int]:
    """Compute REAL DROP - PREDICTED DROP."""
    real = normalize_pose(real_drop_pwm)
    predicted = normalize_pose(predicted_drop_pwm)

    return {
        joint: real[joint] - predicted[joint]
        for joint in SPATIAL_JOINTS
    }


def predict_drop_p77_delta(
    above_pwm: Mapping[str, int],
) -> dict[str, int]:
    """V1 baseline prediction using the hardware-verified P77 delta."""
    return apply_delta(
        above_pwm,
        P77_DROP_DELTA,
    )