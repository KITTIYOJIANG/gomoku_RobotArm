from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
from pathlib import Path

from app.arm.actions import ActionLibrary
from app.stage5.calibration_store import CalibrationStore
from app.stage5.pwm_interpolator import resolve_target_pwm

from .kinematics import ArmKinematics, KinematicsError
from .models import ToolPose, normalize_spatial_pwm
from .settings import Stage6Settings


class AboveSourceError(RuntimeError):
    pass


@dataclass(frozen=True)
class AbovePoseRecord:
    row: int
    col: int
    pwm: dict[str, int]
    source: str
    anchors_used: tuple[str, ...]
    verified: bool
    tool_pose: ToolPose
    model_valid: bool
    warnings: tuple[str, ...] = ()


class ReadOnlyAboveSource:
    """Resolves the Stage5 ABOVE asset and proves that the source file is unchanged."""

    def __init__(
        self,
        settings: Stage6Settings,
        kinematics: ArmKinematics,
        *,
        library: ActionLibrary | None = None,
    ) -> None:
        self.settings = settings
        self.kinematics = kinematics
        self.library = library or ActionLibrary()
        self.path = settings.above_calibration_path
        self._assert_p77_matches_before_store_load()
        before = self.sha256()
        self.store = CalibrationStore(self.path, library=self.library, safety_limits=None)
        after = self.sha256()
        if before != after:
            raise AboveSourceError(
                "Stage5 ABOVE file changed while opening it; refusing Stage6 startup"
            )
        self.source_sha256 = before
        self._records: dict[tuple[int, int], AbovePoseRecord] = {}

    def _assert_p77_matches_before_store_load(self) -> None:
        data = json.loads(self.path.read_text(encoding="utf-8"))
        raw = (data.get("anchors") or {}).get("7,7")
        if raw is None:
            raise AboveSourceError("Stage5 file has no P77 anchor")
        stored = normalize_spatial_pwm(raw.get("pwm") or {})
        action = self.library.get("P77_ABOVE_IDLE")
        stable = {
            f"{joint_id:03d}": action.target(joint_id).pwm for joint_id in range(5)
        }
        if stored != stable:
            raise AboveSourceError(
                "P77 ABOVE in Stage5 data differs from stable action library"
            )

    def sha256(self) -> str:
        return hashlib.sha256(self.path.read_bytes()).hexdigest().upper()

    def resolve_all(self) -> dict[tuple[int, int], AbovePoseRecord]:
        if self._records:
            return dict(self._records)
        if self.sha256() != self.source_sha256:
            raise AboveSourceError("Stage5 ABOVE source changed after Stage6 startup")
        raw_records: dict[tuple[int, int], AbovePoseRecord] = {}
        for row in range(self.settings.board_size):
            for col in range(self.settings.board_size):
                resolved = resolve_target_pwm(
                    self.store,
                    row,
                    col,
                    limits=None,
                    allow_star_seed=True,
                    allow_outer_seed=True,
                )
                pwm = normalize_spatial_pwm(resolved.pwm)
                pose = self.kinematics.forward_kinematics(pwm)
                anchor = self.store.get_anchor(row, col)
                direct_verified = bool(
                    resolved.source == "direct_anchor"
                    and anchor is not None
                    and anchor.calibrated
                    and anchor.verified_runs > 0
                )
                raw_records[(row, col)] = AbovePoseRecord(
                    row=row,
                    col=col,
                    pwm=pwm,
                    source=resolved.source,
                    anchors_used=tuple(resolved.anchors_used),
                    verified=direct_verified,
                    tool_pose=pose,
                    model_valid=True,
                )
        self._records = self._validate_continuity(raw_records)
        if self.sha256() != self.source_sha256:
            raise AboveSourceError("Stage5 ABOVE source was modified during resolution")
        return dict(self._records)

    def get(self, row: int, col: int) -> AbovePoseRecord:
        records = self.resolve_all()
        try:
            return records[(int(row), int(col))]
        except KeyError as exc:
            raise AboveSourceError(f"P({row},{col}) outside board") from exc

    def _validate_continuity(
        self,
        records: dict[tuple[int, int], AbovePoseRecord],
    ) -> dict[tuple[int, int], AbovePoseRecord]:
        result: dict[tuple[int, int], AbovePoseRecord] = {}
        for key, record in records.items():
            warnings: list[str] = []
            for other_key in (
                (record.row - 1, record.col),
                (record.row + 1, record.col),
                (record.row, record.col - 1),
                (record.row, record.col + 1),
            ):
                other = records.get(other_key)
                if other is None:
                    continue
                dx = other.tool_pose.x - record.tool_pose.x
                dy = other.tool_pose.y - record.tool_pose.y
                planar = (dx * dx + dy * dy) ** 0.5
                dz = abs(other.tool_pose.z - record.tool_pose.z)
                da = abs(other.tool_pose.alpha - record.tool_pose.alpha)
                if planar > self.settings.max_neighbor_position_jump_mm:
                    warnings.append(f"neighbor {other_key} XY jump {planar:.1f}mm")
                if dz > self.settings.max_neighbor_z_jump_mm:
                    warnings.append(f"neighbor {other_key} Z jump {dz:.1f}mm")
                if da > self.settings.max_neighbor_alpha_jump_deg:
                    warnings.append(f"neighbor {other_key} alpha jump {da:.1f}deg")
            result[key] = AbovePoseRecord(
                row=record.row,
                col=record.col,
                pwm=dict(record.pwm),
                source=record.source,
                anchors_used=record.anchors_used,
                verified=record.verified,
                tool_pose=record.tool_pose,
                model_valid=not warnings,
                warnings=tuple(sorted(set(warnings))),
            )
        return result
