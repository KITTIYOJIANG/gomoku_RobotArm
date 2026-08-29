from __future__ import annotations

from datetime import datetime, timezone
import json
import os
from pathlib import Path
import tempfile
from typing import Any, Mapping

from app.arm.actions import ActionLibrary

from .baseline import SPATIAL_KEYS


PICK_ACTIONS = {
    "PICK_ABOVE": "SOURCE_TOUCH_IDLE",
    "PICK_DOWN": "SOURCE_TOUCH_HOLD",
}


class PickPoseStore:
    """Candidate pick calibration kept separate from the stable action table."""

    def __init__(self, path: str | Path, library: ActionLibrary) -> None:
        self.path = Path(path)
        self.library = library
        self._data = self._load_or_seed()

    def _baseline_pwm(self, pose_name: str) -> dict[str, int]:
        action = self.library.get(PICK_ACTIONS[pose_name])
        return {key: action.target(int(key)).pwm for key in SPATIAL_KEYS}

    def _seed(self) -> dict[str, Any]:
        return {
            "schema_version": 1,
            "status": "OFFLINE / NOT HARDWARE VERIFIED",
            "stable_action_table_unchanged": True,
            "poses": {
                name: {
                    "pose_name": name,
                    "stable_source_action": action,
                    "baseline_pwm": self._baseline_pwm(name),
                    "new_pwm": self._baseline_pwm(name),
                    "delta_pwm": {key: 0 for key in SPATIAL_KEYS},
                    "source": "BASELINE_CANDIDATE",
                    "verified": False,
                    "updated_at": None,
                    "calibration_session": None,
                }
                for name, action in PICK_ACTIONS.items()
            },
        }

    def _load_or_seed(self) -> dict[str, Any]:
        if not self.path.exists():
            return self._seed()
        data = json.loads(self.path.read_text(encoding="utf-8"))
        if int(data.get("schema_version", 0)) != 1:
            raise ValueError("unsupported pick pose schema")
        return data

    def get(self, pose_name: str) -> dict[str, Any]:
        key = str(pose_name).upper()
        if key not in PICK_ACTIONS:
            raise KeyError(f"unknown pick pose: {pose_name}")
        return dict(self._data["poses"][key])

    def save_candidate(
        self,
        pose_name: str,
        pwm: Mapping[int | str, int],
        *,
        calibration_session: str | None,
    ) -> dict[str, Any]:
        key = str(pose_name).upper()
        baseline = self._baseline_pwm(key)
        new_pwm: dict[str, int] = {}
        for jid, servo_key in enumerate(SPATIAL_KEYS):
            raw = pwm[servo_key] if servo_key in pwm else pwm[jid]
            value = int(raw)
            if not self.library.pwm_min <= value <= self.library.pwm_max:
                raise ValueError(
                    f"{key} J{jid} PWM {value} outside "
                    f"{self.library.pwm_min}..{self.library.pwm_max}"
                )
            new_pwm[servo_key] = value
        record = {
            "pose_name": key,
            "stable_source_action": PICK_ACTIONS[key],
            "baseline_pwm": baseline,
            "new_pwm": new_pwm,
            "delta_pwm": {
                servo_key: new_pwm[servo_key] - baseline[servo_key]
                for servo_key in SPATIAL_KEYS
            },
            "source": "DIRECT_CANDIDATE",
            "verified": False,
            "updated_at": datetime.now(timezone.utc).isoformat(),
            "calibration_session": calibration_session,
        }
        self._data["poses"][key] = record
        self._atomic_write()
        return dict(record)

    def _atomic_write(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        handle, temp_name = tempfile.mkstemp(
            prefix=f".{self.path.name}.", suffix=".tmp", dir=self.path.parent
        )
        try:
            with os.fdopen(handle, "w", encoding="utf-8", newline="\n") as stream:
                json.dump(self._data, stream, ensure_ascii=False, indent=2)
                stream.write("\n")
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temp_name, self.path)
        except Exception:
            try:
                os.unlink(temp_name)
            except FileNotFoundError:
                pass
            raise
