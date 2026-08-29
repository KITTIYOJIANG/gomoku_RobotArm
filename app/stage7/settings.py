from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path

from app.config import PROJECT_ROOT


@dataclass(frozen=True)
class Stage7Settings:
    path: Path
    baseline_path: Path
    sessions_dir: Path
    current_deployment_path: Path
    pick_pose_path: Path
    board_size: int
    default_dry_run: bool
    force_dry_run: bool
    live_jog_interval_ms: int
    live_jog_time_ms: int

    @classmethod
    def load(
        cls,
        path: str | Path = PROJECT_ROOT / "config" / "stage7_rapid_calibration.json",
    ) -> "Stage7Settings":
        source = Path(path)
        if not source.is_absolute():
            source = PROJECT_ROOT / source
        data = json.loads(source.read_text(encoding="utf-8"))

        def project(value: str) -> Path:
            candidate = Path(value)
            return candidate if candidate.is_absolute() else PROJECT_ROOT / candidate

        settings = cls(
            path=source,
            baseline_path=project(str(data["baseline_path"])),
            sessions_dir=project(str(data["sessions_dir"])),
            current_deployment_path=project(str(data["current_deployment_path"])),
            pick_pose_path=project(
                str(data.get("pick_pose_path", "calibration/stage7_pick_poses.json"))
            ),
            board_size=int(data.get("board_size", 15)),
            default_dry_run=bool(data.get("default_dry_run", True)),
            force_dry_run=bool(data.get("force_dry_run", False)),
            live_jog_interval_ms=int(data.get("live_jog_interval_ms", 80)),
            live_jog_time_ms=int(data.get("live_jog_time_ms", 200)),
        )
        settings.validate()
        return settings

    def validate(self) -> None:
        if self.board_size != 15:
            raise ValueError("Stage 7 currently requires the stable 15x15 board")
        if self.live_jog_interval_ms < 50:
            raise ValueError("live jog interval must be at least 50ms")
        if not 100 <= self.live_jog_time_ms <= 9999:
            raise ValueError("live jog movement time outside protocol range")
