from __future__ import annotations

from copy import deepcopy
from datetime import datetime
import json
from pathlib import Path
from typing import Any, Mapping

from app.stage5.safety import PwmSafetyLimits, validate_spatial_pwm

from .models import (
    DESCENT_LEVELS,
    SPATIAL_KEYS,
    DescentLevel,
    DescentProfile,
    LevelStatus,
    VerificationStage,
)


SCHEMA_VERSION = 1


class Stage6CalibrationError(RuntimeError):
    pass


def _zero_delta() -> dict[str, int]:
    return {key: 0 for key in SPATIAL_KEYS}


class Stage6CalibrationStore:
    """Independent candidate/residual store. It never opens Stage5 data for writing."""

    def __init__(
        self,
        path: str | Path,
        *,
        safety_limits: PwmSafetyLimits | None = None,
    ) -> None:
        self.path = Path(path)
        self.safety_limits = safety_limits
        self._data = self._load()

    def _load(self) -> dict[str, Any]:
        if not self.path.exists():
            now = datetime.now().isoformat(timespec="seconds")
            return {
                "schema_version": SCHEMA_VERSION,
                "board_size": 15,
                "source_above": {
                    "path": "calibration/stage5_board_calibration.json",
                    "readonly": True,
                    "sha256": "",
                },
                "profiles": {},
                "history": [],
                "metadata": {"created_at": now, "updated_at": now},
            }
        data = json.loads(self.path.read_text(encoding="utf-8"))
        if int(data.get("schema_version", -1)) != SCHEMA_VERSION:
            raise Stage6CalibrationError("unsupported Stage6 calibration schema")
        return data

    def set_source_fingerprint(self, path: Path, sha256: str) -> None:
        source = dict(self._data.get("source_above") or {})
        previous = str(source.get("sha256") or "")
        if previous and previous != sha256:
            raise Stage6CalibrationError(
                "Stage5 ABOVE fingerprint changed; explicit migration is required"
            )
        self._data["source_above"] = {
            "path": str(path),
            "readonly": True,
            "sha256": str(sha256),
        }

    def upsert_profile(self, profile: DescentProfile, *, save: bool = False) -> None:
        key = self.point_key(profile.row, profile.col)
        old = (self._data.get("profiles") or {}).get(key) or {}
        old_levels = old.get("levels") or {}
        encoded = profile.to_dict()
        level_map: dict[str, Any] = {}
        for item in encoded["levels"]:
            level_name = str(item["level"])
            previous = old_levels.get(level_name) or {}
            if level_name == DescentLevel.ABOVE.value:
                delta = _zero_delta()
            elif previous:
                delta = {
                    key_name: int(
                        (previous.get("manual_delta_pwm") or {}).get(key_name, 0)
                    )
                    for key_name in SPATIAL_KEYS
                }
            else:
                delta = {
                    key_name: int(
                        (item.get("manual_delta_pwm") or {}).get(key_name, 0)
                    )
                    for key_name in SPATIAL_KEYS
                }
            item["manual_delta_pwm"] = delta
            item["final_pwm"] = {
                joint: int(item["computed_pwm"][joint]) + delta[joint]
                for joint in SPATIAL_KEYS
            }
            self._validate_final(item["final_pwm"])
            if previous.get("status") in {status.value for status in LevelStatus}:
                item["status"] = previous["status"]
                item["verified"] = previous["status"] == LevelStatus.VERIFIED.value
            level_map[level_name] = item
        encoded["levels"] = level_map
        encoded["above"] = level_map[DescentLevel.ABOVE.value]
        encoded["touch"] = level_map[DescentLevel.TOUCH.value]
        if old.get("verification_stage") in {
            stage.value for stage in VerificationStage
        }:
            encoded["verification_stage"] = old["verification_stage"]
        encoded["verified"] = (
            encoded["verification_stage"] == VerificationStage.FULLY_VERIFIED.value
        )
        encoded["reverse_ascent_verified"] = bool(
            old.get("reverse_ascent_verified", False)
        )
        self._data.setdefault("profiles", {})[key] = encoded
        if save:
            self.save()

    def profile(self, row: int, col: int) -> dict[str, Any]:
        key = self.point_key(row, col)
        try:
            return deepcopy(self._data["profiles"][key])
        except KeyError as exc:
            raise Stage6CalibrationError(f"no profile for P({row},{col})") from exc

    def profiles(self) -> dict[str, Any]:
        return deepcopy(self._data.get("profiles") or {})

    def apply_delta(
        self,
        row: int,
        col: int,
        level: DescentLevel | str,
        delta_change: Mapping[str | int, int],
        *,
        save: bool = True,
    ) -> dict[str, Any]:
        target = level if isinstance(level, DescentLevel) else DescentLevel(level)
        if target == DescentLevel.ABOVE:
            raise Stage6CalibrationError("existing ABOVE is immutable")
        item = self._level_ref(row, col, target)
        current = {
            key: int((item.get("manual_delta_pwm") or {}).get(key, 0))
            for key in SPATIAL_KEYS
        }
        for raw_key, raw_value in delta_change.items():
            key = f"{int(raw_key):03d}"
            if key not in SPATIAL_KEYS:
                raise Stage6CalibrationError(f"joint {key} is not a spatial joint")
            current[key] = int(current.get(key, 0)) + int(raw_value)
        final = {
            joint: int(item["computed_pwm"][joint]) + int(current.get(joint, 0))
            for joint in SPATIAL_KEYS
        }
        self._validate_final(final)
        self._append_history(row, col, target, item, reason="delta_change")
        item["manual_delta_pwm"] = current
        item["final_pwm"] = final
        item["status"] = LevelStatus.COMPUTED.value
        item["verified"] = False
        if save:
            self.save()
        return deepcopy(item)

    def reset_delta(
        self,
        row: int,
        col: int,
        level: DescentLevel | str,
        *,
        save: bool = True,
    ) -> dict[str, Any]:
        target = level if isinstance(level, DescentLevel) else DescentLevel(level)
        if target == DescentLevel.ABOVE:
            raise Stage6CalibrationError("existing ABOVE is immutable")
        item = self._level_ref(row, col, target)
        self._append_history(row, col, target, item, reason="reset_delta")
        item["manual_delta_pwm"] = _zero_delta()
        item["final_pwm"] = dict(item["computed_pwm"])
        item["status"] = LevelStatus.COMPUTED.value
        item["verified"] = False
        if save:
            self.save()
        return deepcopy(item)

    def copy_neighbor_delta(
        self,
        source_row: int,
        source_col: int,
        target_row: int,
        target_col: int,
        level: DescentLevel | str,
    ) -> dict[str, Any]:
        target = level if isinstance(level, DescentLevel) else DescentLevel(level)
        source = self._level_ref(source_row, source_col, target)
        destination = self._level_ref(target_row, target_col, target)
        self._append_history(
            target_row, target_col, target, destination, reason="copy_neighbor_delta"
        )
        destination["manual_delta_pwm"] = {
            key: int((source.get("manual_delta_pwm") or {}).get(key, 0))
            for key in SPATIAL_KEYS
        }
        destination["final_pwm"] = {
            key: int(destination["computed_pwm"][key])
            + int(destination["manual_delta_pwm"][key])
            for key in SPATIAL_KEYS
        }
        self._validate_final(destination["final_pwm"])
        destination["status"] = LevelStatus.COMPUTED.value
        destination["verified"] = False
        self.save()
        return deepcopy(destination)

    def undo_last(self, row: int, col: int, level: DescentLevel | str) -> dict[str, Any]:
        target = level if isinstance(level, DescentLevel) else DescentLevel(level)
        key = self.point_key(row, col)
        history = self._data.setdefault("history", [])
        for index in range(len(history) - 1, -1, -1):
            entry = history[index]
            if entry["point"] == key and entry["level"] == target.value:
                restored = deepcopy(entry["previous"])
                self._data["profiles"][key]["levels"][target.value] = restored
                history.pop(index)
                self.save()
                return deepcopy(restored)
        raise Stage6CalibrationError("no prior version for this point/level")

    def mark_level(
        self,
        row: int,
        col: int,
        level: DescentLevel | str,
        status: LevelStatus,
    ) -> None:
        target = level if isinstance(level, DescentLevel) else DescentLevel(level)
        item = self._level_ref(row, col, target)
        self._append_history(row, col, target, item, reason="mark_level")
        item["status"] = status.value
        item["verified"] = status == LevelStatus.VERIFIED
        self.save()

    def set_verification_stage(
        self,
        row: int,
        col: int,
        stage: VerificationStage,
        *,
        reverse_ascent_verified: bool | None = None,
    ) -> None:
        profile = self._profile_ref(row, col)
        profile["verification_stage"] = stage.value
        profile["verified"] = stage == VerificationStage.FULLY_VERIFIED
        if reverse_ascent_verified is not None:
            profile["reverse_ascent_verified"] = bool(reverse_ascent_verified)
        self.save()

    def final_pwm(
        self, row: int, col: int, level: DescentLevel | str
    ) -> dict[str, int]:
        item = self._level_ref(
            row,
            col,
            level if isinstance(level, DescentLevel) else DescentLevel(level),
        )
        return {key: int(item["final_pwm"][key]) for key in SPATIAL_KEYS}

    def save(self) -> Path:
        metadata = self._data.setdefault("metadata", {})
        metadata.setdefault("created_at", datetime.now().isoformat(timespec="seconds"))
        metadata["updated_at"] = datetime.now().isoformat(timespec="seconds")
        self._data["schema_version"] = SCHEMA_VERSION
        self._atomic_write(self.path, self._data)
        self._backup()
        return self.path

    def record_batch_result(
        self,
        *,
        requested: int,
        generated_points: list[str],
        rejected: Mapping[str, str],
        save: bool = True,
    ) -> None:
        self._data["last_batch_generation"] = {
            "timestamp": datetime.now().isoformat(timespec="seconds"),
            "requested": int(requested),
            "generated_count": len(generated_points),
            "generated_points": list(generated_points),
            "rejected_count": len(rejected),
            "rejected": dict(rejected),
            "all_candidates_verified": False,
        }
        if save:
            self.save()

    def record_above_snapshot(
        self,
        records: Mapping[str, Mapping[str, Any]],
        *,
        save: bool = True,
    ) -> None:
        if len(records) != 225:
            raise Stage6CalibrationError(
                f"ABOVE snapshot must contain 225 points, got {len(records)}"
            )
        self._data["above_fk_snapshot"] = deepcopy(dict(records))
        if save:
            self.save()

    def _append_history(
        self,
        row: int,
        col: int,
        level: DescentLevel,
        previous: Mapping[str, Any],
        *,
        reason: str,
    ) -> None:
        self._data.setdefault("history", []).append(
            {
                "timestamp": datetime.now().isoformat(timespec="milliseconds"),
                "point": self.point_key(row, col),
                "level": level.value,
                "reason": reason,
                "previous": deepcopy(dict(previous)),
            }
        )

    def _validate_final(self, pwm: Mapping[str, int]) -> None:
        if self.safety_limits is None:
            return
        errors = validate_spatial_pwm(pwm, self.safety_limits)
        if errors:
            raise Stage6CalibrationError("; ".join(errors))

    def _profile_ref(self, row: int, col: int) -> dict[str, Any]:
        key = self.point_key(row, col)
        try:
            return self._data["profiles"][key]
        except KeyError as exc:
            raise Stage6CalibrationError(f"no profile for P({row},{col})") from exc

    def _level_ref(
        self, row: int, col: int, level: DescentLevel
    ) -> dict[str, Any]:
        profile = self._profile_ref(row, col)
        try:
            return profile["levels"][level.value]
        except KeyError as exc:
            raise Stage6CalibrationError(
                f"no level {level.value} for P({row},{col})"
            ) from exc

    @staticmethod
    def point_key(row: int, col: int) -> str:
        row_i, col_i = int(row), int(col)
        if not 0 <= row_i < 15 or not 0 <= col_i < 15:
            raise Stage6CalibrationError(f"P({row},{col}) outside 15x15 board")
        return f"{row_i},{col_i}"

    @staticmethod
    def _atomic_write(path: Path, data: Mapping[str, Any]) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(path.suffix + ".tmp")
        temporary.write_text(
            json.dumps(data, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        temporary.replace(path)

    def _backup(self) -> None:
        backup_dir = self.path.parent / "backups"
        backup_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        destination = backup_dir / f"{self.path.stem}_{stamp}.json"
        destination.write_text(
            json.dumps(self._data, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        for stale in sorted(
            backup_dir.glob(f"{self.path.stem}_*.json"), reverse=True
        )[50:]:
            stale.unlink(missing_ok=True)
