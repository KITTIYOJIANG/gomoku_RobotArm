from __future__ import annotations

from datetime import datetime, timezone
import json
from pathlib import Path
from typing import Mapping

from app.arm.actions import Action, ActionLibrary, ServoTarget


ACTION_NAMES = ("OBSERVE_IDLE", "OBSERVE_HOLD")


def action_pwm(action: Action) -> dict[str, int]:
    return {f"{target.servo_id:03d}": int(target.pwm) for target in action.targets}


def action_times(action: Action) -> dict[str, int]:
    return {f"{target.servo_id:03d}": int(target.time_ms) for target in action.targets}


def build_action(
    name: str,
    pwm: Mapping[str, int],
    times: Mapping[str, int],
) -> Action:
    targets = tuple(
        ServoTarget(joint, int(pwm[f"{joint:03d}"]), int(times[f"{joint:03d}"]))
        for joint in range(8)
    )
    command = "{" + "".join(
        f"#{target.servo_id:03d}P{target.pwm:04d}T{target.time_ms:04d}!"
        for target in targets
    ) + "}"
    return Action(name=name, command=command, targets=targets)


def load_observe_override(path: Path, library: ActionLibrary) -> bool:
    if not path.is_file():
        return False
    data = json.loads(path.read_text(encoding="utf-8"))
    actions = data.get("actions") or {}
    idle_record = actions["OBSERVE_IDLE"]
    library.register_runtime(
        build_action("OBSERVE_IDLE", idle_record["pwm"], idle_record["time_ms"])
    )
    hold_record = actions["OBSERVE_HOLD"]
    hold_pwm = dict(hold_record["pwm"])
    for joint in range(5):
        key = f"{joint:03d}"
        hold_pwm[key] = int(idle_record["pwm"][key])
    library.register_runtime(
        build_action("OBSERVE_HOLD", hold_pwm, hold_record["time_ms"])
    )
    return True


def save_observe_override(
    path: Path,
    *,
    library: ActionLibrary,
    stable_actions: Mapping[str, Action],
    observe_idle_pwm: Mapping[str, int],
) -> Path:
    stable_idle = stable_actions["OBSERVE_IDLE"]
    idle_pwm = action_pwm(stable_idle)
    delta: dict[str, int] = {}
    for joint in range(5):
        key = f"{joint:03d}"
        requested = int(observe_idle_pwm[key])
        if not library.pwm_min <= requested <= library.pwm_max:
            raise ValueError(
                f"{key} PWM {requested} outside {library.pwm_min}..{library.pwm_max}"
            )
        delta[key] = requested - idle_pwm[key]
        idle_pwm[key] = requested

    actions: dict[str, dict[str, dict[str, int]]] = {
        "OBSERVE_IDLE": {
            "pwm": idle_pwm,
            "time_ms": action_times(stable_idle),
        }
    }
    stable_hold = stable_actions["OBSERVE_HOLD"]
    hold_pwm = action_pwm(stable_hold)
    for joint in range(5):
        key = f"{joint:03d}"
        hold_pwm[key] = idle_pwm[key]
    actions["OBSERVE_HOLD"] = {
        "pwm": hold_pwm,
        "time_ms": action_times(stable_hold),
    }

    payload = {
        "schema_version": 1,
        "updated_at": datetime.now(timezone.utc).astimezone().isoformat(),
        "description": "Calibration Lite OBSERVE / PICK_ABOVE shared runtime override",
        "observe_is_pick_above_and_post_pick_hover": True,
        "stable_action_table_unchanged": True,
        "delta_pwm": delta,
        "actions": actions,
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)
    for name in ACTION_NAMES:
        record = actions[name]
        library.register_runtime(build_action(name, record["pwm"], record["time_ms"]))
    return path
