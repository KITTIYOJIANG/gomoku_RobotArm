from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Any

from app.config import PROJECT_ROOT


@dataclass(frozen=True)
class Stage6Settings:
    path: Path
    force_dry_run: bool
    board_size: int
    above_calibration_path: Path
    descent_calibration_path: Path
    kinematics_path: Path
    move_time_ms: int
    release_dwell_ms: int
    pump_hold_pwm: int
    pump_off_pwm: int
    unused_pwm: int
    max_layer_joint_delta_pwm: int
    max_neighbor_position_jump_mm: float
    max_neighbor_z_jump_mm: float
    max_neighbor_alpha_jump_deg: float
    max_above_dwell_seconds: float
    max_touch_dwell_seconds: float
    max_tweaks_per_session: int
    max_continuous_actions: int
    raw: dict[str, Any]

    @classmethod
    def load(
        cls,
        path: str | Path = PROJECT_ROOT / "config" / "stage6_descent.json",
    ) -> "Stage6Settings":
        source = Path(path)
        if not source.is_absolute():
            source = PROJECT_ROOT / source
        data = json.loads(source.read_text(encoding="utf-8"))
        motion = data["motion"]
        validation = data["model_validation"]
        thermal = data["thermal"]

        def project(value: str) -> Path:
            candidate = Path(value)
            return candidate if candidate.is_absolute() else PROJECT_ROOT / candidate

        settings = cls(
            path=source,
            force_dry_run=bool(data.get("force_dry_run", True)),
            board_size=int(data["board_size"]),
            above_calibration_path=project(data["above_calibration_path"]),
            descent_calibration_path=project(data["descent_calibration_path"]),
            kinematics_path=project(data["kinematics_path"]),
            move_time_ms=int(motion["move_time_ms"]),
            release_dwell_ms=int(motion["release_dwell_ms"]),
            pump_hold_pwm=int(motion["pump_hold_pwm"]),
            pump_off_pwm=int(motion["pump_off_pwm"]),
            unused_pwm=int(motion["unused_pwm"]),
            max_layer_joint_delta_pwm=int(motion["max_layer_joint_delta_pwm"]),
            max_neighbor_position_jump_mm=float(
                validation["max_neighbor_position_jump_mm"]
            ),
            max_neighbor_z_jump_mm=float(validation["max_neighbor_z_jump_mm"]),
            max_neighbor_alpha_jump_deg=float(
                validation["max_neighbor_alpha_jump_deg"]
            ),
            max_above_dwell_seconds=float(thermal["max_above_dwell_seconds"]),
            max_touch_dwell_seconds=float(thermal["max_touch_dwell_seconds"]),
            max_tweaks_per_session=int(thermal["max_tweaks_per_session"]),
            max_continuous_actions=int(thermal["max_continuous_actions"]),
            raw=data,
        )
        settings.validate()
        return settings

    def validate(self) -> None:
        if self.board_size != 15:
            raise ValueError("Stage6 currently requires the stable 15x15 board")
        if not 500 <= self.pump_off_pwm <= 2500:
            raise ValueError("pump_off_pwm outside protocol range")
        if not 500 <= self.pump_hold_pwm <= 2500:
            raise ValueError("pump_hold_pwm outside protocol range")
        if self.move_time_ms <= 0 or self.release_dwell_ms < 0:
            raise ValueError("invalid Stage6 timing")
        if self.max_layer_joint_delta_pwm <= 0:
            raise ValueError("max layer joint delta must be positive")
