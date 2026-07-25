from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def project_path(value: str | Path) -> Path:
    path = Path(value)
    return path if path.is_absolute() else PROJECT_ROOT / path


def load_json(path: str | Path) -> dict[str, Any]:
    source = project_path(path)
    return json.loads(source.read_text(encoding="utf-8"))


@dataclass(frozen=True)
class SerialConfig:
    default_port: str
    baudrate: int
    write_timeout_seconds: float


@dataclass(frozen=True)
class CameraConfig:
    index: int
    preferred_name: str
    width: int
    height: int
    fps: int


@dataclass(frozen=True)
class Stage5Config:
    enabled: bool
    default_dry_run: bool
    click_threshold_ratio: float
    calibration_path: Path
    board_span_cells: int
    allow_motion_without_camera: bool
    force_dry_run: bool = False
    cross_anchor_required_runs: int = 3
    cross_draft_path: Path | None = None
    hover_samples_path: Path | None = None


@dataclass(frozen=True)
class TimingConfig:
    action_wait_margin_ms: int
    vacuum_build_ms: int
    release_ms: int


@dataclass(frozen=True)
class TrackerConfig:
    corner_smoothing_alpha: float
    consecutive_failure_limit: int
    lost_timeout_seconds: float


@dataclass(frozen=True)
class PieceRecognitionConfig:
    roi_radius: int
    background_radius: int
    black_diff: float
    white_diff: float
    black_area_ratio: float
    white_area_ratio: float


@dataclass(frozen=True)
class VisionConfig:
    detection_interval_frames: int
    board_size: int
    target_name: str
    target_row: int
    target_col: int
    layout_path: Path
    grid_path: Path
    intrinsics_path: Path
    intrinsics_fallback_path: Path
    tracker: TrackerConfig
    piece_recognition: PieceRecognitionConfig


@dataclass(frozen=True)
class AppConfig:
    serial: SerialConfig
    camera: CameraConfig
    timing: TimingConfig
    vision: VisionConfig
    stage5: Stage5Config
    logs_dir: Path

    @classmethod
    def load(cls, path: str | Path = "config/app_config.json") -> "AppConfig":
        data = load_json(path)
        serial = data["serial"]
        camera = data["camera"]
        timing = data["timing"]
        vision = load_json(data["vision_config_path"])
        grid_path = project_path(vision["grid_config"])
        grid = load_json(grid_path)
        target = vision["target"]
        tracker = vision["tracker"]
        piece = vision["piece_recognition"]
        result = cls(
            serial=SerialConfig(
                default_port=str(serial["default_port"]),
                baudrate=int(serial["baudrate"]),
                write_timeout_seconds=float(serial["write_timeout_seconds"]),
            ),
            camera=CameraConfig(
                index=int(camera["index"]),
                preferred_name=str(camera["preferred_name"]),
                width=int(camera["width"]),
                height=int(camera["height"]),
                fps=int(camera["fps"]),
            ),
            timing=TimingConfig(
                action_wait_margin_ms=int(timing["action_wait_margin_ms"]),
                vacuum_build_ms=int(timing["vacuum_build_ms"]),
                release_ms=int(timing["release_ms"]),
            ),
            vision=VisionConfig(
                detection_interval_frames=int(vision["detection_interval_frames"]),
                board_size=int(grid["board_size"]),
                target_name=str(target["name"]),
                target_row=int(target["row"]),
                target_col=int(target["col"]),
                layout_path=project_path(vision["apriltag_layout"]),
                grid_path=grid_path,
                intrinsics_path=project_path(vision["camera_intrinsics"]),
                intrinsics_fallback_path=project_path(vision["camera_intrinsics_fallback"]),
                tracker=TrackerConfig(
                    corner_smoothing_alpha=float(tracker["corner_smoothing_alpha"]),
                    consecutive_failure_limit=int(tracker["consecutive_failure_limit"]),
                    lost_timeout_seconds=float(tracker["lost_timeout_seconds"]),
                ),
                piece_recognition=PieceRecognitionConfig(
                    roi_radius=int(piece["roi_radius"]),
                    background_radius=int(piece["background_radius"]),
                    black_diff=float(piece["black_diff"]),
                    white_diff=float(piece["white_diff"]),
                    black_area_ratio=float(piece["black_area_ratio"]),
                    white_area_ratio=float(piece["white_area_ratio"]),
                ),
            ),
            stage5=Stage5Config(
                enabled=bool((data.get("stage5") or {}).get("enabled", True)),
                default_dry_run=bool((data.get("stage5") or {}).get("default_dry_run", True)),
                click_threshold_ratio=float((data.get("stage5") or {}).get("click_threshold_ratio", 0.32)),
                calibration_path=project_path((data.get("stage5") or {}).get("calibration_path", "calibration/stage5_board_calibration.json")),
                board_span_cells=int((data.get("stage5") or {}).get("board_span_cells", 8)),
                allow_motion_without_camera=bool((data.get("stage5") or {}).get("allow_motion_without_camera", False)),
                force_dry_run=bool((data.get("stage5") or {}).get("force_dry_run", False)),
                cross_anchor_required_runs=int((data.get("stage5") or {}).get("cross_anchor_required_runs", 3)),
                cross_draft_path=project_path((data.get("stage5") or {}).get("cross_draft_path", "calibration/stage5_cross_anchor_drafts.json")),
                hover_samples_path=project_path((data.get("stage5") or {}).get("hover_samples_path", "datasets/hover_pose/verified_samples.jsonl")),
            ),
            logs_dir=project_path(data["logs_dir"]),
        )
        result.validate()
        return result

    def validate(self) -> None:
        if self.serial.baudrate != 115200:
            raise ValueError("V0.1 requires a fixed baudrate of 115200")
        if self.timing.vacuum_build_ms < 0 or self.timing.release_ms < 0:
            raise ValueError("Wait durations cannot be negative")
        if self.vision.detection_interval_frames < 1:
            raise ValueError("AprilTag detection interval must be at least one frame")
        if not 0.0 < self.vision.tracker.corner_smoothing_alpha <= 1.0:
            raise ValueError("Corner smoothing alpha must be in (0, 1]")
        if self.vision.tracker.consecutive_failure_limit < 1:
            raise ValueError("Tracker failure limit must be at least one")
        if self.vision.tracker.lost_timeout_seconds <= 0:
            raise ValueError("Tracker lost timeout must be positive")
        if self.vision.board_size < 2:
            raise ValueError("Vision board size must be at least two")
        if not 0.1 <= self.stage5.click_threshold_ratio <= 0.5:
            raise ValueError('stage5 click_threshold_ratio must be in [0.1, 0.5]')
        if self.stage5.board_span_cells < 1:
            raise ValueError('stage5 board_span_cells must be positive')
        if not (
            0 <= self.vision.target_row < self.vision.board_size
            and 0 <= self.vision.target_col < self.vision.board_size
        ):
            raise ValueError("P77 target must be inside the configured board")

