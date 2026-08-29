from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import tempfile
from typing import Any, Mapping

from app.arm.actions import ActionLibrary
from app.config import PROJECT_ROOT
from app.stage7.baseline import BaselineSnapshot

from .golden import (
    FAST_9,
    GOLDEN_ABOVE,
    GOLDEN_FAST_5,
    SPATIAL_KEYS,
    assert_golden_above,
    golden_for,
    normalize_spatial,
)
from .points import BOARD_SIZE, PointRef, all_points, format_point_id, parse_point_id


PROFILE_SCHEMA_VERSION = 1
CALIBRATION_VERSION = "1.0"
DEFAULT_PROFILE_PATH = PROJECT_ROOT / "calibration" / "profiles" / "integrated_v1_current.json"
DEFAULT_BASELINE_PATH = PROJECT_ROOT / "calibration" / "stage5_board_calibration.json"
DEFAULT_GOLDEN_BASELINE_DIR = PROJECT_ROOT / "calibration" / "baseline"


class ProfileError(RuntimeError):
    pass


@dataclass(frozen=True)
class ProfileStatus:
    exists: bool
    schema_valid: bool
    compatible: bool
    valid: bool
    reasons: tuple[str, ...]

    @property
    def route(self) -> str:
        return "GAME" if self.valid else "FIRST_SETUP"


def _now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def _atomic_json(path: Path, payload: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    handle, raw_temp = tempfile.mkstemp(prefix=path.name + ".", suffix=".tmp", dir=path.parent)
    temp = Path(raw_temp)
    try:
        with os.fdopen(handle, "w", encoding="utf-8", newline="\n") as stream:
            json.dump(payload, stream, ensure_ascii=False, indent=2, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temp, path)
    finally:
        if temp.exists():
            temp.unlink()


class CalibrationProfileManager:
    """Versioned V1 profile that overlays, but never edits, the Stage 5 baseline."""

    def __init__(
        self,
        *,
        profile_path: str | Path = DEFAULT_PROFILE_PATH,
        baseline_path: str | Path = DEFAULT_BASELINE_PATH,
        library: ActionLibrary | None = None,
    ) -> None:
        self.profile_path = Path(profile_path)
        self.library = library or ActionLibrary()
        self.baseline = BaselineSnapshot(
            Path(baseline_path), board_size=BOARD_SIZE, library=self.library
        )
        self.data: dict[str, Any] | None = None

    @property
    def is_loaded(self) -> bool:
        return self.data is not None

    def create_from_stable_baseline(self, *, profile_id: str | None = None) -> dict[str, Any]:
        self.baseline.assert_unchanged()
        timestamp = _now()
        points: dict[str, dict[str, Any]] = {}
        conflicts: list[dict[str, Any]] = []
        for point in all_points():
            baseline_point = self.baseline.get(point.row, point.col)
            baseline_pwm = dict(baseline_point.pwm)
            golden = golden_for(point.row, point.col)
            if golden is None:
                final = baseline_pwm
                source = baseline_point.source
                verified = bool(baseline_point.verified)
                verification_level = (
                    "HARDWARE VERIFIED" if verified else "NOT VERIFIED"
                )
                protected = False
                legacy_id = None
            else:
                final = golden.pwm_map
                source = "golden_direct_anchor"
                verified = True
                verification_level = "HARDWARE VERIFIED"
                protected = True
                legacy_id = golden.legacy_id
                if baseline_pwm != final:
                    conflicts.append(
                        {
                            "point": golden.legacy_id,
                            "board": [point.row, point.col],
                            "old_stage5_pwm": baseline_pwm,
                            "golden_pwm": final,
                            "reason": "user-confirmed hardware Golden overrides the V1 runtime view; stable Stage 5 file is unchanged",
                        }
                    )
            key = point.point_id
            points[key] = {
                "point_id": key,
                "flat_id": point.flat_id,
                "legacy_id": legacy_id,
                "board": [point.row, point.col],
                "baseline_above_pwm": baseline_pwm,
                "generated_above_pwm": baseline_pwm,
                "fast_correction_pwm": {joint: 0 for joint in SPATIAL_KEYS},
                "final_above_pwm": final,
                "source": source,
                "baseline_source": baseline_point.source,
                "verified": verified,
                "verification_level": verification_level,
                "protected": protected,
            }
        assert_golden_above(points)
        self.data = {
            "schema_version": PROFILE_SCHEMA_VERSION,
            "calibration_version": CALIBRATION_VERSION,
            "profile_id": profile_id or f"integrated-v1-{timestamp.replace(':', '').replace('+', '_')}",
            "created_at": timestamp,
            "updated_at": timestamp,
            "valid": False,
            "verification_level": "NOT VERIFIED",
            "baseline": {
                "path": str(self.baseline.path),
                "sha256": self.baseline.source_sha256,
                "immutable": True,
            },
            "robot": {
                "model": "J1",
                "spatial_joint_ids": list(SPATIAL_KEYS),
                "pump_joint_id": "005",
                "serial_protocol": "existing_ascii_pwm_115200",
            },
            "board": {
                "size": BOARD_SIZE,
                "row_0": "top",
                "col_0": "left",
            },
            "pickup": {
                "source": "stable_action_table",
                "valid": True,
                "verification_level": "HARDWARE VERIFIED",
            },
            "above": {"points": points},
            "drop": {"points": {}},
            "fast_calibration": {
                "mode": None,
                "anchors": {},
                "method": None,
            },
            "golden_anchor_conflicts": conflicts,
            "golden_baseline": None,
            "pending_golden_overrides": {},
            "verification": {
                "verified_drop_points": [],
                "last_verified_at": None,
            },
            "history": [
                {
                    "timestamp": timestamp,
                    "event": "PROFILE_CREATED_FROM_STABLE_BASELINE",
                    "baseline_sha256": self.baseline.source_sha256,
                    "golden_conflict_count": len(conflicts),
                }
            ],
        }
        return self.data

    def load(self) -> dict[str, Any]:
        if not self.profile_path.exists():
            raise ProfileError(f"calibration profile does not exist: {self.profile_path}")
        try:
            payload = json.loads(self.profile_path.read_text(encoding="utf-8"))
        except Exception as exc:
            raise ProfileError(f"cannot read calibration profile: {exc}") from exc
        if not isinstance(payload, dict):
            raise ProfileError("calibration profile root must be an object")
        self.data = payload
        status = self.status()
        if not status.schema_valid or not status.compatible:
            raise ProfileError("; ".join(status.reasons))
        return self.data

    def load_or_create(self) -> dict[str, Any]:
        if self.profile_path.exists():
            return self.load()
        return self.create_from_stable_baseline()

    def status(self) -> ProfileStatus:
        if self.data is None:
            if not self.profile_path.exists():
                return ProfileStatus(False, False, False, False, ("profile missing",))
            try:
                payload = json.loads(self.profile_path.read_text(encoding="utf-8"))
            except Exception as exc:
                return ProfileStatus(True, False, False, False, (f"profile unreadable: {exc}",))
        else:
            payload = self.data
        reasons: list[str] = []
        schema_valid = True
        if int(payload.get("schema_version", -1)) != PROFILE_SCHEMA_VERSION:
            schema_valid = False
            reasons.append("unsupported schema_version")
        if str(payload.get("calibration_version")) != CALIBRATION_VERSION:
            schema_valid = False
            reasons.append("unsupported calibration_version")
        if int((payload.get("board") or {}).get("size", -1)) != BOARD_SIZE:
            schema_valid = False
            reasons.append("board size must be 15")
        points = ((payload.get("above") or {}).get("points") or {})
        if len(points) != BOARD_SIZE * BOARD_SIZE:
            schema_valid = False
            reasons.append("225 ABOVE points are required")
        compatible = True
        baseline_hash = str((payload.get("baseline") or {}).get("sha256", ""))
        if baseline_hash != self.baseline.source_sha256:
            compatible = False
            reasons.append("stable baseline hash mismatch")
        try:
            assert_golden_above(points)
        except Exception as exc:
            compatible = False
            reasons.append(str(exc))
        if not bool((payload.get("pickup") or {}).get("valid", False)):
            reasons.append("pickup calibration invalid")
        declared_valid = bool(payload.get("valid", False))
        verified_drop_ids = list(
            ((payload.get("verification") or {}).get("verified_drop_points") or [])
        )
        drop_points = ((payload.get("drop") or {}).get("points") or {})
        verified_drop_valid = bool(verified_drop_ids) and all(
            point_id in drop_points
            and bool(drop_points[point_id].get("verified"))
            and str(drop_points[point_id].get("status")) == "VERIFIED"
            for point_id in verified_drop_ids
        )
        if declared_valid and not verified_drop_valid:
            reasons.append("valid profile requires at least one internally consistent verified DROP")
        valid = (
            schema_valid
            and compatible
            and declared_valid
            and verified_drop_valid
            and "pickup calibration invalid" not in reasons
        )
        if not declared_valid:
            reasons.append("profile has not been explicitly promoted valid")
        return ProfileStatus(True, schema_valid, compatible, valid, tuple(reasons))

    def save(self) -> Path:
        data = self._require_data()
        self.baseline.assert_unchanged()
        assert_golden_above(data["above"]["points"])
        data["updated_at"] = _now()
        _atomic_json(self.profile_path, data)
        return self.profile_path

    def promote_valid(self) -> Path:
        data = self._require_data()
        assert_golden_above(data["above"]["points"])
        if not bool(data["pickup"].get("valid")):
            raise ProfileError("pickup must be valid before profile promotion")
        verified = data["verification"].get("verified_drop_points") or []
        if not verified:
            raise ProfileError("verify at least one DROP point before profile promotion")
        data["valid"] = True
        data["verification_level"] = "OFFLINE VERIFIED"
        data["history"].append(
            {"timestamp": _now(), "event": "PROFILE_PROMOTED_VALID", "verified_drop_count": len(verified)}
        )
        return self.save()

    def export_golden_baseline(
        self, directory: str | Path = DEFAULT_GOLDEN_BASELINE_DIR
    ) -> Path:
        """Write a new immutable-intent baseline artifact; never overwrite one."""

        data = self._require_data()
        if not self.status().valid:
            raise ProfileError("only an explicitly valid profile can become a Golden Baseline")
        assert_golden_above(data["above"]["points"])
        timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        target_dir = Path(directory)
        target_dir.mkdir(parents=True, exist_ok=True)
        destination = target_dir / f"gomoku_v1_golden_{timestamp}.json"
        if destination.exists():
            raise ProfileError(f"Golden Baseline already exists: {destination}")
        payload = json.loads(json.dumps(data))
        payload["baseline_kind"] = "GOLDEN_CALIBRATION_BASELINE"
        payload["baseline_created_at"] = _now()
        payload["source_profile_path"] = str(self.profile_path)
        payload["immutable_intent"] = True
        _atomic_json(destination, payload)
        data["golden_baseline"] = {
            "path": str(destination),
            "created_at": payload["baseline_created_at"],
            "immutable_intent": True,
        }
        data["history"].append(
            {
                "timestamp": payload["baseline_created_at"],
                "event": "GOLDEN_BASELINE_EXPORTED",
                "path": str(destination),
            }
        )
        self.save()
        return destination

    def above_record(self, point_id: str | PointRef | tuple[int, int]) -> dict[str, Any]:
        point = parse_point_id(point_id)
        key = point.point_id
        try:
            return self._require_data()["above"]["points"][key]
        except KeyError as exc:
            raise ProfileError(f"profile has no ABOVE for {key}") from exc

    def above_pwm(self, point_id: str | PointRef | tuple[int, int]) -> dict[str, int]:
        return normalize_spatial(self.above_record(point_id)["final_above_pwm"])

    def request_golden_override(
        self,
        point_id: str | PointRef | tuple[int, int],
        pwm: Mapping[str | int, int],
        *,
        confirmation_note: str,
    ) -> None:
        point = parse_point_id(point_id)
        golden = golden_for(point.row, point.col)
        if golden is None:
            raise ProfileError(f"{point.point_id} is not a Golden ABOVE")
        note = str(confirmation_note).strip()
        if not note:
            raise ProfileError("Golden override request requires an operator confirmation note")
        requested = self._validate_pwm(pwm)
        data = self._require_data()
        data["pending_golden_overrides"][golden.legacy_id] = {
            "board": [point.row, point.col],
            "current_golden_pwm": golden.pwm_map,
            "requested_pwm": requested,
            "confirmation_note": note,
            "status": "PENDING_REVALIDATION",
            "created_at": _now(),
        }
        data["history"].append(
            {"timestamp": _now(), "event": "GOLDEN_OVERRIDE_REQUESTED", "point": golden.legacy_id}
        )

    def save_direct_anchor(
        self,
        point_id: str | PointRef | tuple[int, int],
        pwm: Mapping[str | int, int],
    ) -> dict[str, Any]:
        point = parse_point_id(point_id)
        golden = golden_for(point.row, point.col)
        if golden is not None:
            raise ProfileError(
                f"{golden.legacy_id} is a protected Golden ABOVE; create an explicit revalidation request instead"
            )
        requested = self._validate_pwm(pwm)
        record = self.above_record(point)
        record["generated_above_pwm"] = dict(requested)
        record["fast_correction_pwm"] = {
            joint: requested[joint] - int(record["baseline_above_pwm"][joint])
            for joint in SPATIAL_KEYS
        }
        record["final_above_pwm"] = dict(requested)
        record["source"] = "user_direct_anchor"
        record["verified"] = False
        record["verification_level"] = "NOT VERIFIED"
        record["protected"] = True
        self._invalidate_drop(point, reason="ABOVE_DIRECT_ANCHOR_CHANGED")
        self._require_data()["history"].append(
            {"timestamp": _now(), "event": "DIRECT_ABOVE_SAVED", "point_id": point.point_id}
        )
        return record

    def apply_fast_calibration(
        self,
        mode: str,
        anchors: Mapping[str | tuple[int, int] | PointRef, Mapping[str | int, int]],
    ) -> dict[str, Any]:
        selected = str(mode).strip().upper()
        required = GOLDEN_FAST_5 if selected in {"5", "FAST_5", "QUICK_5"} else FAST_9
        normalized: dict[tuple[int, int], dict[str, int]] = {}
        for raw_point, raw_pwm in anchors.items():
            point = parse_point_id(raw_point)
            normalized[point.as_tuple()] = self._validate_pwm(raw_pwm)
        missing = [format_point_id(*coord) for coord in required if coord not in normalized]
        if missing:
            raise ProfileError("missing fast-calibration anchors: " + ", ".join(missing))
        anchor_deltas: dict[tuple[int, int], dict[str, int]] = {}
        for coord in required:
            base = self.above_pwm(coord)
            anchor_deltas[coord] = {
                joint: normalized[coord][joint] - base[joint] for joint in SPATIAL_KEYS
            }
        nodes = self._fast_nodes(selected, anchor_deltas)
        data = self._require_data()
        changed = 0
        for point in all_points():
            record = self.above_record(point)
            if bool(record.get("protected")):
                # Golden and user-direct anchors are authoritative and are never
                # overwritten by a generated correction field.
                continue
            delta = self._bilinear_field(nodes, point.row, point.col)
            base = normalize_spatial(record["baseline_above_pwm"])
            final = self._validate_pwm(
                {joint: base[joint] + delta[joint] for joint in SPATIAL_KEYS}
            )
            record["generated_above_pwm"] = dict(final)
            record["fast_correction_pwm"] = dict(delta)
            record["final_above_pwm"] = dict(final)
            record["source"] = f"fast_calibration_{len(required)}_point"
            record["verified"] = False
            record["verification_level"] = "NOT VERIFIED"
            self._invalidate_drop(point, reason="FAST_CALIBRATION_CHANGED_ABOVE")
            changed += 1
        assert_golden_above(data["above"]["points"])
        data["fast_calibration"] = {
            "mode": f"FAST_{len(required)}",
            "anchors": {
                format_point_id(*coord): {
                    "new_anchor_pwm": normalized[coord],
                    "anchor_correction_pwm": anchor_deltas[coord],
                }
                for coord in required
            },
            "method": "piecewise_bilinear_clamped",
            "applied_at": _now(),
            "changed_generated_points": changed,
        }
        data["valid"] = False
        data["history"].append(
            {
                "timestamp": _now(),
                "event": "FAST_CALIBRATION_APPLIED",
                "mode": f"FAST_{len(required)}",
                "golden_points_preserved": len(GOLDEN_ABOVE),
            }
        )
        return data["fast_calibration"]

    def drop_record(self, point_id: str | PointRef | tuple[int, int]) -> dict[str, Any] | None:
        point = parse_point_id(point_id)
        return self._require_data()["drop"]["points"].get(point.point_id)

    def set_drop_record(
        self,
        point_id: str | PointRef | tuple[int, int],
        record: Mapping[str, Any],
    ) -> dict[str, Any]:
        point = parse_point_id(point_id)
        payload = dict(record)
        payload["point_id"] = point.point_id
        payload["board"] = [point.row, point.col]
        self._require_data()["drop"]["points"][point.point_id] = payload
        return payload

    def save_drop_correction(
        self,
        point_id: str | PointRef | tuple[int, int],
        correction: Mapping[str | int, int],
    ) -> dict[str, Any]:
        point = parse_point_id(point_id)
        record = self.drop_record(point)
        if record is None or not record.get("drop_auto_pwm"):
            raise ProfileError(f"generate DROP before correcting {point.point_id}")
        delta = normalize_spatial(correction)
        auto = normalize_spatial(record["drop_auto_pwm"])
        final = self._validate_pwm({joint: auto[joint] + delta[joint] for joint in SPATIAL_KEYS})
        record["drop_correction_pwm"] = delta
        record["drop_final_pwm"] = final
        record["status"] = "MANUAL_CORRECTED"
        record["verified"] = False
        record["verification_level"] = "NOT VERIFIED"
        record["updated_at"] = _now()
        self._require_data()["history"].append(
            {"timestamp": _now(), "event": "DROP_CORRECTION_SAVED", "point_id": point.point_id}
        )
        return record

    def reset_drop_correction(self, point_id: str | PointRef | tuple[int, int]) -> dict[str, Any]:
        point = parse_point_id(point_id)
        record = self.drop_record(point)
        if record is None or not record.get("drop_auto_pwm"):
            raise ProfileError(f"generate DROP before resetting {point.point_id}")
        record["drop_correction_pwm"] = {joint: 0 for joint in SPATIAL_KEYS}
        record["drop_final_pwm"] = dict(record["drop_auto_pwm"])
        record["status"] = "PENDING_VERIFY"
        record["verified"] = False
        record["verification_level"] = "NOT VERIFIED"
        record["updated_at"] = _now()
        return record

    def mark_drop_verified(
        self,
        point_id: str | PointRef | tuple[int, int],
        *,
        notes: str = "",
        hardware_confirmed: bool = False,
    ) -> dict[str, Any]:
        point = parse_point_id(point_id)
        record = self.drop_record(point)
        if record is None or record.get("status") in {"MOVE_L_UNREACHABLE", "INVALID", "NOT_GENERATED"}:
            raise ProfileError(f"{point.point_id} has no verifiable DROP")
        record["status"] = "VERIFIED"
        record["verified"] = True
        record["verification_time"] = _now()
        record["verification_level"] = (
            "HARDWARE VERIFIED" if hardware_confirmed else "OFFLINE VERIFIED"
        )
        record["notes"] = str(notes)
        verification = self._require_data()["verification"]
        points = set(verification.get("verified_drop_points") or [])
        points.add(point.point_id)
        verification["verified_drop_points"] = sorted(points)
        verification["last_verified_at"] = record["verification_time"]
        return record

    def statistics(self) -> dict[str, int]:
        drops = self._require_data()["drop"]["points"]
        statuses = [str(item.get("status", "NOT_GENERATED")) for item in drops.values()]
        return {
            "Total": BOARD_SIZE * BOARD_SIZE,
            "Generated": sum(status not in {"NOT_GENERATED", "INVALID", "MOVE_L_UNREACHABLE"} for status in statuses),
            "Verified": sum(bool(item.get("verified")) for item in drops.values()),
            "Pending": statuses.count("PENDING_VERIFY"),
            "Unreachable": statuses.count("MOVE_L_UNREACHABLE"),
            "Invalid": statuses.count("INVALID"),
            "Manual": statuses.count("MANUAL_CORRECTED"),
            "Golden": len(GOLDEN_ABOVE),
        }

    def _require_data(self) -> dict[str, Any]:
        if self.data is None:
            raise ProfileError("load or create a calibration profile first")
        return self.data

    def _validate_pwm(self, pwm: Mapping[str | int, int]) -> dict[str, int]:
        values = normalize_spatial(pwm)
        for joint, value in values.items():
            if not self.library.pwm_min <= value <= self.library.pwm_max:
                raise ProfileError(
                    f"joint {joint} PWM {value} outside {self.library.pwm_min}..{self.library.pwm_max}"
                )
        return values

    def _invalidate_drop(self, point: PointRef, *, reason: str) -> None:
        record = self.drop_record(point)
        if record is None:
            return
        record["status"] = "NOT_GENERATED"
        record["verified"] = False
        record["verification_level"] = "NOT VERIFIED"
        record["stale_reason"] = reason
        verified = set(self._require_data()["verification"].get("verified_drop_points") or [])
        verified.discard(point.point_id)
        self._require_data()["verification"]["verified_drop_points"] = sorted(verified)

    @staticmethod
    def _fast_nodes(
        mode: str,
        deltas: Mapping[tuple[int, int], Mapping[str, int]],
    ) -> dict[tuple[int, int], dict[str, float]]:
        if mode in {"5", "FAST_5", "QUICK_5"}:
            tl, tr = deltas[(3, 3)], deltas[(3, 11)]
            center = deltas[(7, 7)]
            bl, br = deltas[(11, 3)], deltas[(11, 11)]

            def avg(a: Mapping[str, int], b: Mapping[str, int]) -> dict[str, float]:
                return {joint: (a[joint] + b[joint]) / 2.0 for joint in SPATIAL_KEYS}

            return {
                (3, 3): dict(tl),
                (3, 7): avg(tl, tr),
                (3, 11): dict(tr),
                (7, 3): avg(tl, bl),
                (7, 7): dict(center),
                (7, 11): avg(tr, br),
                (11, 3): dict(bl),
                (11, 7): avg(bl, br),
                (11, 11): dict(br),
            }
        return {coord: {joint: float(value) for joint, value in delta.items()} for coord, delta in deltas.items()}

    @staticmethod
    def _bilinear_field(
        nodes: Mapping[tuple[int, int], Mapping[str, float]], row: int, col: int
    ) -> dict[str, int]:
        rows = sorted({coord[0] for coord in nodes})
        cols = sorted({coord[1] for coord in nodes})

        def bounds(value: int, axis: list[int]) -> tuple[int, int]:
            clamped = max(axis[0], min(axis[-1], int(value)))
            for low, high in zip(axis, axis[1:]):
                if low <= clamped <= high:
                    return low, high
            return axis[-2], axis[-1]

        r0, r1 = bounds(row, rows)
        c0, c1 = bounds(col, cols)
        rr = max(rows[0], min(rows[-1], row))
        cc = max(cols[0], min(cols[-1], col))
        v = 0.0 if r1 == r0 else (rr - r0) / float(r1 - r0)
        u = 0.0 if c1 == c0 else (cc - c0) / float(c1 - c0)
        result: dict[str, int] = {}
        for joint in SPATIAL_KEYS:
            top = (1.0 - u) * nodes[(r0, c0)][joint] + u * nodes[(r0, c1)][joint]
            bottom = (1.0 - u) * nodes[(r1, c0)][joint] + u * nodes[(r1, c1)][joint]
            result[joint] = int(round((1.0 - v) * top + v * bottom))
        return result
