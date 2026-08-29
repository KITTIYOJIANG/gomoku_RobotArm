from __future__ import annotations

from datetime import datetime
import json
from pathlib import Path
from typing import Mapping

from app.arm.actions import Action, ActionLibrary

from .observe_pose import action_pwm, action_times, build_action


SPATIAL_KEYS = tuple(f"{joint:03d}" for joint in range(5))


def derive_place_contact(
    above_pwm: Mapping[str, int],
    stable_above_pwm: Mapping[str, int],
    stable_touch_pwm: Mapping[str, int],
    correction_delta: Mapping[str, int] | None = None,
) -> dict[str, int]:
    """Apply the stable P77 descent vector to the latest calibrated ABOVE."""
    correction = correction_delta or {}
    return {
        key: int(above_pwm[key])
        + int(stable_touch_pwm[key])
        - int(stable_above_pwm[key])
        + int(correction.get(key, 0))
        for key in SPATIAL_KEYS
    }


def save_place_override(
    path: Path,
    *,
    library: ActionLibrary,
    stable_above: Action,
    stable_touch_hold: Action,
    stable_touch_release: Action,
    requested_pwm: Mapping[str, int],
    calibration_session: str | None,
) -> dict:
    above = action_pwm(library.get("P77_ABOVE_IDLE"))
    stable_above_pwm = action_pwm(stable_above)
    stable_touch_pwm = action_pwm(stable_touch_hold)
    derived = derive_place_contact(above, stable_above_pwm, stable_touch_pwm)
    requested = {key: int(requested_pwm[key]) for key in SPATIAL_KEYS}
    correction = {key: requested[key] - derived[key] for key in SPATIAL_KEYS}
    record = {
        "schema_version": 1,
        "updated_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "calibration_session": calibration_session,
        "source": "quick_calibration_P77_ABOVE_plus_stable_descent_vector",
        "motion_kind": "layered_pwm_descent_candidate_not_cartesian_movel",
        "verified": False,
        "verification_level": "NOT VERIFIED",
        "operator_position_confirmed": True,
        "above_pwm": {key: int(above[key]) for key in SPATIAL_KEYS},
        "reference_descent_delta": {
            key: int(stable_touch_pwm[key]) - int(stable_above_pwm[key])
            for key in SPATIAL_KEYS
        },
        "correction_delta": correction,
        "new_pwm": requested,
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(record, ensure_ascii=False, indent=2), encoding="utf-8")
    apply_place_runtime(
        library,
        stable_above=stable_above,
        stable_touch_hold=stable_touch_hold,
        stable_touch_release=stable_touch_release,
        correction_delta=correction,
    )
    return record


def load_place_override(
    path: Path,
    *,
    library: ActionLibrary,
    stable_above: Action,
    stable_touch_hold: Action,
    stable_touch_release: Action,
) -> dict | None:
    if not path.is_file():
        return None
    record = json.loads(path.read_text(encoding="utf-8"))
    correction = record.get("correction_delta") or {}
    if set(correction) != set(SPATIAL_KEYS):
        raise ValueError("place calibration correction_delta must contain J0..J4")
    new_pwm = apply_place_runtime(
        library,
        stable_above=stable_above,
        stable_touch_hold=stable_touch_hold,
        stable_touch_release=stable_touch_release,
        correction_delta=correction,
    )
    result = dict(record)
    result["new_pwm"] = new_pwm
    return result


def apply_place_runtime(
    library: ActionLibrary,
    *,
    stable_above: Action,
    stable_touch_hold: Action,
    stable_touch_release: Action,
    correction_delta: Mapping[str, int] | None = None,
) -> dict[str, int]:
    above = action_pwm(library.get("P77_ABOVE_IDLE"))
    target = derive_place_contact(
        above,
        action_pwm(stable_above),
        action_pwm(stable_touch_hold),
        correction_delta,
    )
    for key, value in target.items():
        if not library.pwm_min <= int(value) <= library.pwm_max:
            raise ValueError(
                f"place target {key}={value} outside "
                f"{library.pwm_min}..{library.pwm_max}"
            )
    for name, stable in (
        ("P77_TOUCH_HOLD", stable_touch_hold),
        ("P77_TOUCH_RELEASE", stable_touch_release),
    ):
        pwm = action_pwm(stable)
        pwm.update(target)
        if name == "P77_TOUCH_RELEASE":
            pwm["005"] = 1500
        library.register_runtime(build_action(name, pwm, action_times(stable)))
    return target
