from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path

from app.config import PROJECT_ROOT


@dataclass(frozen=True)
class IntegratedV1Settings:
    path: Path
    profile_path: Path
    baseline_path: Path
    kinematics_path: Path
    target_descent_mm: float
    waypoint_step_mm: float
    max_waypoint_joint_delta_pwm: int
    move_time_ms: int
    release_ms: int
    default_dry_run: bool
    force_dry_run: bool

    @classmethod
    def load(
        cls,
        path: str | Path = PROJECT_ROOT / "config" / "integrated_v1.json",
    ) -> "IntegratedV1Settings":
        source = Path(path)
        if not source.is_absolute():
            source = PROJECT_ROOT / source
        data = json.loads(source.read_text(encoding="utf-8"))

        def project(raw: str) -> Path:
            candidate = Path(raw)
            return candidate if candidate.is_absolute() else PROJECT_ROOT / candidate

        motion = data.get("motion") or {}
        settings = cls(
            path=source,
            profile_path=project(str(data["profile_path"])),
            baseline_path=project(str(data["baseline_path"])),
            kinematics_path=project(str(data["kinematics_path"])),
            target_descent_mm=float(motion.get("target_descent_mm", 25.0)),
            waypoint_step_mm=float(motion.get("waypoint_step_mm", 5.0)),
            max_waypoint_joint_delta_pwm=int(
                motion.get("max_waypoint_joint_delta_pwm", 400)
            ),
            move_time_ms=int(motion.get("move_time_ms", 1000)),
            release_ms=int(motion.get("release_ms", 700)),
            default_dry_run=bool(data.get("default_dry_run", True)),
            force_dry_run=bool(data.get("force_dry_run", False)),
        )
        settings.validate()
        return settings

    def validate(self) -> None:
        if self.target_descent_mm <= 0 or self.waypoint_step_mm <= 0:
            raise ValueError("V1 descent and waypoint step must be positive")
        if self.waypoint_step_mm > self.target_descent_mm:
            raise ValueError("V1 waypoint step cannot exceed target descent")
        if self.max_waypoint_joint_delta_pwm <= 0:
            raise ValueError("V1 waypoint joint-delta guard must be positive")
        if not 100 <= self.move_time_ms <= 9999:
            raise ValueError("V1 move time outside protocol limits")
        if self.release_ms < 0:
            raise ValueError("V1 release dwell cannot be negative")
