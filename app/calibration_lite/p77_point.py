from __future__ import annotations

from datetime import datetime
import json
from pathlib import Path
from typing import Mapping


SPATIAL_KEYS = tuple(f"{joint:03d}" for joint in range(5))


def load_p77_point(path: Path) -> dict[str, int] | None:
    if not path.is_file():
        return None
    data = json.loads(path.read_text(encoding="utf-8"))
    if data.get("point_name") != "P77":
        raise ValueError("P77 point file must declare point_name=P77")
    pwm = data.get("new_pwm") or {}
    if set(pwm) != set(SPATIAL_KEYS):
        raise ValueError("P77 new_pwm must contain exactly J0..J4")
    result = {key: int(pwm[key]) for key in SPATIAL_KEYS}
    if any(not 550 < value < 2450 for value in result.values()):
        raise ValueError("P77 new_pwm touches or exceeds the software rail")
    if len(set(result.values())) == 1:
        raise ValueError("P77 J0..J4 cannot all have the same PWM")
    return result


def save_p77_point(path: Path, pwm: Mapping[str, int]) -> Path:
    values = {key: int(pwm[key]) for key in SPATIAL_KEYS}
    if any(not 550 < value < 2450 for value in values.values()):
        raise ValueError("P77 new_pwm touches or exceeds the software rail")
    if len(set(values.values())) == 1:
        raise ValueError("P77 J0..J4 cannot all have the same PWM")
    payload = {
        "schema_version": 1,
        "point_name": "P77",
        "board_row": 7,
        "board_col": 7,
        "source": "operator_confirmed_new_pwm_only",
        "updated_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "stable_action_table_unchanged": True,
        "new_pwm": values,
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)
    return path
