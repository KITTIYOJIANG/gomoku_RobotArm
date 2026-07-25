from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
from datetime import datetime
import json
import logging
from pathlib import Path
import shutil
from typing import Any, Mapping

from app.arm.actions import ActionLibrary
from app.config import PROJECT_ROOT
from app.stage5.safety import SPATIAL_JOINT_IDS, PwmSafetyLimits, validate_spatial_pwm


LOGGER = logging.getLogger(__name__)
SCHEMA_VERSION = 1
DEFAULT_ANCHOR_ROWS = (3, 7, 11)
DEFAULT_ANCHOR_COLS = (3, 7, 11)


@dataclass(frozen=True)
class AnchorPose:
    row: int
    col: int
    pose_type: str
    pwm: dict[str, int]
    time_ms: int
    calibrated: bool
    verified_runs: int
    notes: str = ""

    @property
    def key(self) -> str:
        return f"{self.row},{self.col}"

    def spatial_pwm(self) -> dict[int, int]:
        return {int(jid): int(value) for jid, value in self.pwm.items() if int(jid) in SPATIAL_JOINT_IDS}

    def to_dict(self) -> dict[str, Any]:
        return {
            "row": self.row,
            "col": self.col,
            "pose_type": self.pose_type,
            "pwm": {f"{int(k):03d}": int(v) for k, v in self.pwm.items()},
            "time_ms": self.time_ms,
            "calibrated": bool(self.calibrated),
            "verified_runs": int(self.verified_runs),
            "notes": self.notes,
        }

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "AnchorPose":
        pwm_raw = data.get("pwm") or {}
        pwm: dict[str, int] = {}
        for key, value in pwm_raw.items():
            if value is None:
                raise ValueError(f"anchor pwm contains null for joint {key}")
            pwm[f"{int(key):03d}"] = int(value)
        return cls(
            row=int(data["row"]),
            col=int(data["col"]),
            pose_type=str(data.get("pose_type", "TARGET_ABOVE")),
            pwm=pwm,
            time_ms=int(data.get("time_ms", 1000)),
            calibrated=bool(data.get("calibrated", False)),
            verified_runs=int(data.get("verified_runs", 0)),
            notes=str(data.get("notes", "")),
        )


class CalibrationStore:
    """Persistent TARGET_ABOVE anchor manager with atomic save and backups."""

    def __init__(
        self,
        path: str | Path | None = None,
        *,
        library: ActionLibrary | None = None,
        safety_limits: PwmSafetyLimits | None = None,
    ) -> None:
        self.path = Path(path) if path is not None else PROJECT_ROOT / "calibration" / "stage5_board_calibration.json"
        self.library = library or ActionLibrary()
        self.safety_limits = safety_limits
        self._data: dict[str, Any] = {}
        self._anchors: dict[str, AnchorPose] = {}
        self.load_error: str | None = None
        self._load_or_bootstrap()

    @property
    def valid(self) -> bool:
        return self.load_error is None and bool(self._data)

    @property
    def anchor_rows(self) -> tuple[int, ...]:
        return tuple(int(v) for v in self._data.get("anchor_rows", DEFAULT_ANCHOR_ROWS))

    @property
    def anchor_cols(self) -> tuple[int, ...]:
        return tuple(int(v) for v in self._data.get("anchor_cols", DEFAULT_ANCHOR_COLS))

    @property
    def board_size(self) -> int:
        return int(self._data.get("board_size", 15))

    @property
    def allowed_region(self) -> dict[str, int]:
        rows = self.anchor_rows
        cols = self.anchor_cols
        return {
            "row_min": min(rows) if rows else 0,
            "row_max": max(rows) if rows else self.board_size - 1,
            "col_min": min(cols) if cols else 0,
            "col_max": max(cols) if cols else self.board_size - 1,
        }

    def anchors(self) -> dict[str, AnchorPose]:
        return dict(self._anchors)

    def get_anchor(self, row: int, col: int) -> AnchorPose | None:
        return self._anchors.get(f"{int(row)},{int(col)}")

    def is_anchor_point(self, row: int, col: int) -> bool:
        return int(row) in self.anchor_rows and int(col) in self.anchor_cols

    def expand_anchor_grid(self, row: int, col: int) -> None:
        """Include a taught intersection in the interpolation grid (rows/cols)."""
        r = int(row)
        c = int(col)
        if not (0 <= r < self.board_size and 0 <= c < self.board_size):
            raise ValueError(f"({r},{c}) outside board")
        rows = sorted(set(self.anchor_rows) | {r})
        cols = sorted(set(self.anchor_cols) | {c})
        self._data["anchor_rows"] = rows
        self._data["anchor_cols"] = cols


    def calibrated_anchors(self) -> dict[str, AnchorPose]:
        return {key: anchor for key, anchor in self._anchors.items() if anchor.calibrated}

    def _load_or_bootstrap(self) -> None:
        if self.path.exists():
            try:
                self._data = json.loads(self.path.read_text(encoding="utf-8"))
                self._anchors = {
                    key: AnchorPose.from_dict(value)
                    for key, value in (self._data.get("anchors") or {}).items()
                }
                self._ensure_p77_from_library(save=True)
                self.load_error = None
                return
            except Exception as exc:
                self.load_error = f"Failed to load calibration: {exc}"
                LOGGER.exception("STAGE5 calibration load failed")
                self._data = {}
                self._anchors = {}
                return
        self._bootstrap_from_library()

    def _bootstrap_from_library(self) -> None:
        now = datetime.now().isoformat(timespec="seconds")
        self._data = {
            "schema_version": SCHEMA_VERSION,
            "board_size": 15,
            "coordinate_system": {"row_0": "top", "col_0": "left"},
            "joint_ids": [f"{jid:03d}" for jid in SPATIAL_JOINT_IDS],
            "pump_joint_id": "005",
            "anchor_rows": list(DEFAULT_ANCHOR_ROWS),
            "anchor_cols": list(DEFAULT_ANCHOR_COLS),
            "anchors": {},
            "limits": {},
            "metadata": {
                "created_at": now,
                "updated_at": now,
                "camera_setup": "",
                "board_setup": "",
            },
        }
        self._anchors = {}
        self._ensure_p77_from_library(save=True)
        self.load_error = None

    def _ensure_p77_from_library(self, *, save: bool) -> None:
        action = self.library.get("P77_ABOVE_IDLE")
        pwm = {f"{jid:03d}": int(action.target(jid).pwm) for jid in SPATIAL_JOINT_IDS}
        existing = self._anchors.get("7,7")
        notes = "Imported from stable action library P77_ABOVE_IDLE"
        if existing is not None and existing.calibrated and existing.spatial_pwm() == {int(k): v for k, v in pwm.items()}:
            return
        anchor = AnchorPose(
            row=7,
            col=7,
            pose_type="TARGET_ABOVE",
            pwm=pwm,
            time_ms=int(action.duration_ms),
            calibrated=True,
            verified_runs=max(1, existing.verified_runs if existing else 1),
            notes=notes,
        )
        self._anchors["7,7"] = anchor
        self._data.setdefault("anchors", {})["7,7"] = anchor.to_dict()
        if save:
            self.save()

    def reload(self) -> None:
        self._load_or_bootstrap()

    def save(self) -> Path:
        if not self._data:
            raise RuntimeError("No calibration data to save")
        self._data["schema_version"] = SCHEMA_VERSION
        self._data["anchors"] = {key: anchor.to_dict() for key, anchor in self._anchors.items()}
        metadata = dict(self._data.get("metadata") or {})
        now = datetime.now().isoformat(timespec="seconds")
        metadata.setdefault("created_at", now)
        metadata["updated_at"] = now
        self._data["metadata"] = metadata
        self._atomic_write(self.path, self._data)
        self._write_backup(self._data)
        LOGGER.info("STAGE5 calibration saved %s anchors=%d", self.path, len(self._anchors))
        return self.path

    def export_to(self, destination: str | Path) -> Path:
        dest = Path(destination)
        self._atomic_write(dest, self._data if self._data else {"anchors": {}})
        return dest

    def restore_latest_backup(self) -> Path:
        backup_dir = self.path.parent / "backups"
        candidates = sorted(backup_dir.glob(f"{self.path.stem}_*.json"), reverse=True)
        if not candidates:
            raise FileNotFoundError("No calibration backup found")
        source = candidates[0]
        data = json.loads(source.read_text(encoding="utf-8"))
        self._data = data
        self._anchors = {key: AnchorPose.from_dict(value) for key, value in (data.get("anchors") or {}).items()}
        self.load_error = None
        self.save()
        LOGGER.info("STAGE5 calibration restored from %s", source)
        return source

    def upsert_anchor(
        self,
        row: int,
        col: int,
        pwm: Mapping[str | int, int],
        *,
        time_ms: int = 1000,
        notes: str = "",
        calibrated: bool | None = None,
        verified_runs: int | None = None,
        require_anchor_set: bool = True,
        expand_grid: bool = False,
        safety_limits: object | None = None,
        skip_envelope_check: bool = False,
    ) -> AnchorPose:
        if expand_grid:
            self.expand_anchor_grid(row, col)
        if require_anchor_set and not self.is_anchor_point(row, col):
            raise ValueError(f"({row},{col}) is not in the configured anchor set")
        normalized = {f"{int(k):03d}": int(v) for k, v in pwm.items() if int(k) in SPATIAL_JOINT_IDS}
        limits = safety_limits if safety_limits is not None else self.safety_limits
        if not skip_envelope_check and limits is not None:
            errors = validate_spatial_pwm(normalized, limits)
            if errors:
                raise ValueError("; ".join(errors))
        # Absolute protocol bounds always
        for jid in SPATIAL_JOINT_IDS:
            key = f"{jid:03d}"
            if key not in normalized:
                continue
            v = int(normalized[key])
            if not (500 <= v <= 2500):
                raise ValueError(f"joint {key} PWM {v} outside protocol 500..2500")
        for jid in SPATIAL_JOINT_IDS:
            key = f"{jid:03d}"
            if key not in normalized:
                raise ValueError(f"missing joint {key}")
        existing = self.get_anchor(row, col)
        if calibrated is None:
            calibrated_flag = bool(existing.calibrated) if existing else False
        else:
            calibrated_flag = bool(calibrated)
        if verified_runs is None:
            runs = int(existing.verified_runs) if existing else 0
        else:
            runs = int(verified_runs)
        anchor = AnchorPose(
            row=int(row),
            col=int(col),
            pose_type="TARGET_ABOVE",
            pwm=normalized,
            time_ms=int(time_ms),
            calibrated=calibrated_flag,
            verified_runs=runs,
            notes=notes or (existing.notes if existing else ""),
        )
        self._anchors[anchor.key] = anchor
        return anchor

    def confirm_anchor_safe(self, row: int, col: int) -> AnchorPose:
        anchor = self.get_anchor(row, col)
        if anchor is None:
            raise KeyError(f"No anchor data for ({row},{col})")
        updated = AnchorPose(
            row=anchor.row,
            col=anchor.col,
            pose_type=anchor.pose_type,
            pwm=dict(anchor.pwm),
            time_ms=anchor.time_ms,
            calibrated=True,
            verified_runs=int(anchor.verified_runs) + 1,
            notes=anchor.notes,
        )
        self._anchors[updated.key] = updated
        self.save()
        return updated

    def revoke_anchor_safe(self, row: int, col: int) -> AnchorPose:
        anchor = self.get_anchor(row, col)
        if anchor is None:
            raise KeyError(f"No anchor data for ({row},{col})")
        updated = AnchorPose(
            row=anchor.row,
            col=anchor.col,
            pose_type=anchor.pose_type,
            pwm=dict(anchor.pwm),
            time_ms=anchor.time_ms,
            calibrated=False,
            verified_runs=int(anchor.verified_runs),
            notes=anchor.notes,
        )
        self._anchors[updated.key] = updated
        self.save()
        return updated

    def load_pwm_from_action(self, action_name: str = "P77_ABOVE_IDLE") -> dict[str, int]:
        action = self.library.get(action_name)
        return {f"{jid:03d}": int(action.target(jid).pwm) for jid in SPATIAL_JOINT_IDS}

    def to_public_dict(self) -> dict[str, Any]:
        return deepcopy(self._data)

    def _atomic_write(self, path: Path, data: dict[str, Any]) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.with_suffix(path.suffix + ".tmp")
        tmp.write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        tmp.replace(path)

    def _write_backup(self, data: dict[str, Any]) -> Path:
        backup_dir = self.path.parent / "backups"
        backup_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        backup = backup_dir / f"{self.path.stem}_{stamp}.json"
        backup.write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        # Keep only the latest 20 backups.
        existing = sorted(backup_dir.glob(f"{self.path.stem}_*.json"), reverse=True)
        for stale in existing[20:]:
            try:
                stale.unlink()
            except OSError:
                pass
        return backup
