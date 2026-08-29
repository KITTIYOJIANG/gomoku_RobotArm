from __future__ import annotations

from datetime import datetime
from enum import Enum
import json
import os
from pathlib import Path
from typing import Any, Mapping

from app.stage5.pwm_interpolator import bilinear_interpolate_values
from app.stage5.safety import PwmSafetyLimits

from .baseline import BaselineSnapshot, SPATIAL_KEYS, point_id


QUICK_COORDS = ((0, 0), (0, 14), (7, 7), (14, 0), (14, 14))
STANDARD_COORDS = (
    (0, 0),
    (0, 7),
    (0, 14),
    (7, 0),
    (7, 7),
    (7, 14),
    (14, 0),
    (14, 7),
    (14, 14),
)


class CalibrationMode(str, Enum):
    QUICK_5 = "QUICK_5"
    STANDARD_9 = "STANDARD_9"


class SessionError(RuntimeError):
    pass


def _now() -> str:
    return datetime.now().astimezone().isoformat(timespec="seconds")


def _normalize_pwm(pwm: Mapping[int | str, int]) -> dict[str, int]:
    result: dict[str, int] = {}
    for index, key in enumerate(SPATIAL_KEYS):
        if key in pwm:
            raw = pwm[key]
        elif index in pwm:
            raw = pwm[index]
        else:
            raise SessionError(f"missing spatial joint {key}")
        result[key] = int(raw)
    return result


def _atomic_json(path: Path, payload: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, path)


class RapidCalibrationSession:
    """One deployment session; it never mutates the stable baseline file."""

    schema_version = 1

    def __init__(
        self,
        *,
        baseline: BaselineSnapshot,
        mode: CalibrationMode | str,
        session_id: str,
        path: str | Path,
        limits: PwmSafetyLimits,
        created_at: str | None = None,
    ) -> None:
        self.baseline = baseline
        self.mode = CalibrationMode(mode)
        self.session_id = str(session_id)
        self.path = Path(path)
        self.limits = limits
        self.created_at = created_at or _now()
        self.updated_at = self.created_at
        self.anchors: dict[str, dict[str, Any]] = {}
        self.generated_points: dict[str, dict[str, Any]] = {}
        self.verified_points: set[str] = set()
        self.history: list[dict[str, Any]] = []
        self.candidate_revision = 0
        self.candidate_stale = True
        self.committed_at: str | None = None

    @property
    def required_coordinates(self) -> tuple[tuple[int, int], ...]:
        return QUICK_COORDS if self.mode == CalibrationMode.QUICK_5 else STANDARD_COORDS

    @property
    def missing_required(self) -> tuple[str, ...]:
        return tuple(
            point_id(row, col, self.baseline.board_size)
            for row, col in self.required_coordinates
            if point_id(row, col, self.baseline.board_size) not in self.anchors
        )

    def point_pwm(self, row: int, col: int) -> dict[str, int]:
        key = point_id(row, col, self.baseline.board_size)
        if key in self.anchors:
            return dict(self.anchors[key]["new_pwm"])
        if key in self.generated_points and not self.candidate_stale:
            return dict(self.generated_points[key]["new_pwm"])
        return dict(self.baseline.get(row, col).pwm)

    def save_anchor(
        self,
        row: int,
        col: int,
        requested_pwm: Mapping[int | str, int],
    ) -> dict[str, Any]:
        baseline_point = self.baseline.get(row, col)
        normalized = _normalize_pwm(requested_pwm)
        applied: dict[str, int] = {}
        clamp_log: dict[str, dict[str, int]] = {}
        for index, key in enumerate(SPATIAL_KEYS):
            requested = normalized[key]
            lo = int(self.limits.joint_min[index])
            hi = int(self.limits.joint_max[index])
            value = max(lo, min(hi, requested))
            applied[key] = value
            if value != requested:
                clamp_log[key] = {"requested": requested, "applied": value}
        key = baseline_point.point_id
        timestamp = _now()
        replaced = self.anchors.get(key)
        anchor = {
            "point_id": key,
            "board_row": int(row),
            "board_col": int(col),
            "baseline_pwm": dict(baseline_point.pwm),
            "new_pwm": applied,
            "delta_pwm": {
                joint: applied[joint] - baseline_point.pwm[joint]
                for joint in SPATIAL_KEYS
            },
            "source": "DIRECT",
            "timestamp": timestamp,
            "calibration_session_id": self.session_id,
            "clamped": bool(clamp_log),
            "clamp_log": clamp_log,
        }
        self.anchors[key] = anchor
        self.verified_points.discard(key)
        self.candidate_stale = True
        self.updated_at = timestamp
        self.history.append(
            {
                "timestamp": timestamp,
                "event": "ANCHOR_OVERWRITTEN" if replaced else "ANCHOR_SAVED",
                "point_id": key,
                "previous_new_pwm": None if replaced is None else replaced["new_pwm"],
                "new_pwm": dict(applied),
                "clamp_log": clamp_log,
            }
        )
        return dict(anchor)

    def recalculate(self) -> dict[str, dict[str, Any]]:
        self.baseline.assert_unchanged()
        missing = self.missing_required
        if missing:
            raise SessionError("missing required anchors: " + ", ".join(missing))

        has_standard = all(
            point_id(row, col, self.baseline.board_size) in self.anchors
            for row, col in STANDARD_COORDS
        )
        effective_mode = CalibrationMode.STANDARD_9 if has_standard else CalibrationMode.QUICK_5
        if has_standard and self.mode == CalibrationMode.QUICK_5:
            self.mode = CalibrationMode.STANDARD_9
            self.history.append(
                {"timestamp": _now(), "event": "MODE_UPGRADED", "mode": self.mode.value}
            )

        core_nodes = self._core_nodes(effective_mode)
        field: dict[tuple[int, int], dict[str, float]] = {}
        core_used: dict[tuple[int, int], tuple[str, ...]] = {}
        for row in range(self.baseline.board_size):
            for col in range(self.baseline.board_size):
                delta, used = self._interpolate_core(core_nodes, row, col)
                field[(row, col)] = delta
                core_used[(row, col)] = used

        core_ids = {
            point_id(row, col, self.baseline.board_size)
            for row, col in (STANDARD_COORDS if effective_mode == CalibrationMode.STANDARD_9 else QUICK_COORDS)
        }
        local_anchors = [anchor for key, anchor in self.anchors.items() if key not in core_ids]
        local_used: dict[tuple[int, int], list[str]] = {
            key: [] for key in field
        }
        for anchor in local_anchors:
            anchor_coord = (int(anchor["board_row"]), int(anchor["board_col"]))
            residual = {
                joint: float(anchor["delta_pwm"][joint]) - field[anchor_coord][joint]
                for joint in SPATIAL_KEYS
            }
            r_lo, r_hi = _cell_bounds(anchor_coord[0])
            c_lo, c_hi = _cell_bounds(anchor_coord[1])
            for (row, col), values in field.items():
                weight = _axis_tent(row, anchor_coord[0], r_lo, r_hi) * _axis_tent(
                    col, anchor_coord[1], c_lo, c_hi
                )
                if weight <= 0.0:
                    continue
                for joint in SPATIAL_KEYS:
                    values[joint] += weight * residual[joint]
                local_used[(row, col)].append(str(anchor["point_id"]))

        generated: dict[str, dict[str, Any]] = {}
        for (row, col), delta_float in field.items():
            baseline_point = self.baseline.get(row, col)
            key = baseline_point.point_id
            delta = {joint: int(round(delta_float[joint])) for joint in SPATIAL_KEYS}
            new_pwm: dict[str, int] = {}
            clamp_log: dict[str, dict[str, int]] = {}
            for index, joint in enumerate(SPATIAL_KEYS):
                requested = baseline_point.pwm[joint] + delta[joint]
                lo, hi = self.limits.joint_min[index], self.limits.joint_max[index]
                applied = max(int(lo), min(int(hi), requested))
                new_pwm[joint] = applied
                delta[joint] = applied - baseline_point.pwm[joint]
                if applied != requested:
                    clamp_log[joint] = {"requested": requested, "applied": applied}

            source = "DIRECT" if key in self.anchors else "INTERPOLATED"
            if source == "DIRECT":
                # Direct values are authoritative after safety clamp and may never
                # be changed by interpolation or a later local residual overlay.
                new_pwm = dict(self.anchors[key]["new_pwm"])
                delta = dict(self.anchors[key]["delta_pwm"])
                clamp_log = dict(self.anchors[key].get("clamp_log") or {})
            generated[key] = {
                "point_id": key,
                "board_row": row,
                "board_col": col,
                "baseline_pwm": dict(baseline_point.pwm),
                "new_pwm": new_pwm,
                "delta_pwm": delta,
                "source": source,
                "verified": key in self.verified_points,
                "baseline_source": baseline_point.source,
                "anchors_used": list(core_used[(row, col)] + tuple(local_used[(row, col)])),
                "clamped": bool(clamp_log),
                "clamp_log": clamp_log,
            }

        if len(generated) != self.baseline.board_size**2:
            raise SessionError("candidate generation did not produce 225 points")
        self.generated_points = generated
        self.candidate_revision += 1
        self.candidate_stale = False
        self.updated_at = _now()
        self.history.append(
            {
                "timestamp": self.updated_at,
                "event": "CANDIDATE_RECALCULATED",
                "revision": self.candidate_revision,
                "mode": effective_mode.value,
                "anchor_count": len(self.anchors),
                "local_anchor_count": len(local_anchors),
            }
        )
        return dict(self.generated_points)

    def verify(self, row: int, col: int) -> dict[str, Any]:
        if self.candidate_stale or not self.generated_points:
            raise SessionError("recalculate the candidate before verification")
        key = point_id(row, col, self.baseline.board_size)
        try:
            record = self.generated_points[key]
        except KeyError as exc:
            raise SessionError(f"candidate has no {key}") from exc
        self.verified_points.add(key)
        record["verified"] = True
        self.updated_at = _now()
        self.history.append(
            {"timestamp": self.updated_at, "event": "POINT_VERIFIED", "point_id": key}
        )
        return dict(record)

    def rollback_candidate_to_baseline(self) -> dict[str, dict[str, Any]]:
        self.baseline.assert_unchanged()
        self.verified_points.clear()
        self.generated_points = {}
        for (row, col), baseline_point in self.baseline.all_points().items():
            self.generated_points[baseline_point.point_id] = {
                "point_id": baseline_point.point_id,
                "board_row": row,
                "board_col": col,
                "baseline_pwm": dict(baseline_point.pwm),
                "new_pwm": dict(baseline_point.pwm),
                "delta_pwm": {joint: 0 for joint in SPATIAL_KEYS},
                "source": "BASELINE",
                "verified": False,
                "baseline_source": baseline_point.source,
                "anchors_used": [],
                "clamped": False,
                "clamp_log": {},
            }
        self.candidate_revision += 1
        self.candidate_stale = False
        self.updated_at = _now()
        self.history.append(
            {
                "timestamp": self.updated_at,
                "event": "CANDIDATE_ROLLED_BACK_TO_BASELINE",
                "revision": self.candidate_revision,
            }
        )
        return dict(self.generated_points)

    def save(self) -> Path:
        self.baseline.assert_unchanged()
        self.updated_at = _now()
        _atomic_json(self.path, self.to_dict())
        return self.path

    def commit(self, current_deployment_path: str | Path) -> Path:
        if self.candidate_stale or len(self.generated_points) != self.baseline.board_size**2:
            raise SessionError("candidate is missing or stale; recalculate before commit")
        self.baseline.assert_unchanged()
        self.committed_at = _now()
        self.updated_at = self.committed_at
        self.history.append(
            {
                "timestamp": self.committed_at,
                "event": "CALIBRATION_COMMITTED",
                "revision": self.candidate_revision,
            }
        )
        self.save()
        destination = Path(current_deployment_path)
        payload = {
            "schema_version": self.schema_version,
            "deployment_state": "CANDIDATE_COMMITTED",
            "calibration_session_id": self.session_id,
            "session_path": str(self.path),
            "committed_at": self.committed_at,
            "baseline": self._baseline_metadata(),
            "mode": self.mode.value,
            "candidate_revision": self.candidate_revision,
            "points": self.generated_points,
        }
        _atomic_json(destination, payload)
        return destination

    def commit_rollback(self, current_deployment_path: str | Path) -> Path:
        self.rollback_candidate_to_baseline()
        destination = Path(current_deployment_path)
        timestamp = _now()
        payload = {
            "schema_version": self.schema_version,
            "deployment_state": "ROLLED_BACK_TO_BASELINE",
            "calibration_session_id": self.session_id,
            "session_path": str(self.path),
            "rolled_back_at": timestamp,
            "baseline": self._baseline_metadata(),
            "points": self.generated_points,
        }
        self.history.append(
            {"timestamp": timestamp, "event": "DEPLOYMENT_ROLLED_BACK_TO_BASELINE"}
        )
        self.save()
        _atomic_json(destination, payload)
        return destination

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "calibration_session_id": self.session_id,
            "baseline": self._baseline_metadata(),
            "mode": self.mode.value,
            "anchors": self.anchors,
            "generated_points": self.generated_points,
            "verified_points": sorted(self.verified_points),
            "candidate_revision": self.candidate_revision,
            "candidate_stale": self.candidate_stale,
            "committed_at": self.committed_at,
            "created_at": self.created_at,
            "updated_at": self.updated_at,
            "history": self.history,
        }

    @classmethod
    def load(
        cls,
        path: str | Path,
        *,
        baseline: BaselineSnapshot,
        limits: PwmSafetyLimits,
    ) -> "RapidCalibrationSession":
        source = Path(path)
        data = json.loads(source.read_text(encoding="utf-8"))
        baseline_meta = data.get("baseline") or {}
        if str(baseline_meta.get("sha256")) != baseline.source_sha256:
            raise SessionError("session baseline hash differs from the loaded stable baseline")
        session = cls(
            baseline=baseline,
            mode=data["mode"],
            session_id=data["calibration_session_id"],
            path=source,
            limits=limits,
            created_at=data.get("created_at"),
        )
        session.updated_at = str(data.get("updated_at") or session.created_at)
        session.anchors = dict(data.get("anchors") or {})
        session.generated_points = dict(data.get("generated_points") or {})
        session.verified_points = set(data.get("verified_points") or [])
        session.history = list(data.get("history") or [])
        session.candidate_revision = int(data.get("candidate_revision", 0))
        session.candidate_stale = bool(data.get("candidate_stale", True))
        session.committed_at = data.get("committed_at")
        return session

    def _baseline_metadata(self) -> dict[str, Any]:
        return {
            "path": str(self.baseline.path),
            "sha256": self.baseline.source_sha256,
            "board_size": self.baseline.board_size,
            "resolved_point_count": self.baseline.board_size**2,
            "direct_anchor_count": self.baseline.direct_count,
        }

    def _core_nodes(
        self, mode: CalibrationMode
    ) -> dict[tuple[int, int], dict[str, float]]:
        direct = {
            (int(anchor["board_row"]), int(anchor["board_col"])): {
                joint: float(anchor["delta_pwm"][joint]) for joint in SPATIAL_KEYS
            }
            for anchor in self.anchors.values()
        }
        if mode == CalibrationMode.STANDARD_9:
            return {coord: direct[coord] for coord in STANDARD_COORDS}

        tl, tr, center, bl, br = (direct[coord] for coord in QUICK_COORDS)

        def average(a: Mapping[str, float], b: Mapping[str, float]) -> dict[str, float]:
            return {joint: (a[joint] + b[joint]) / 2.0 for joint in SPATIAL_KEYS}

        return {
            (0, 0): tl,
            (0, 7): average(tl, tr),
            (0, 14): tr,
            (7, 0): average(tl, bl),
            (7, 7): center,
            (7, 14): average(tr, br),
            (14, 0): bl,
            (14, 7): average(bl, br),
            (14, 14): br,
        }

    def _interpolate_core(
        self,
        nodes: Mapping[tuple[int, int], Mapping[str, float]],
        row: int,
        col: int,
    ) -> tuple[dict[str, float], tuple[str, ...]]:
        r1, r2 = (0, 7) if row <= 7 else (7, 14)
        c1, c2 = (0, 7) if col <= 7 else (7, 14)
        u = 0.0 if c1 == c2 else (col - c1) / float(c2 - c1)
        v = 0.0 if r1 == r2 else (row - r1) / float(r2 - r1)
        values = bilinear_interpolate_values(
            nodes[(r1, c1)],
            nodes[(r1, c2)],
            nodes[(r2, c1)],
            nodes[(r2, c2)],
            u,
            v,
        )
        used_values: list[str] = []
        for r, c in ((r1, c1), (r1, c2), (r2, c1), (r2, c2)):
            key = point_id(r, c, self.baseline.board_size)
            used_values.append(key if key in self.anchors else f"VIRTUAL:{key}")
        used = tuple(used_values)
        return ({str(key): float(value) for key, value in values.items()}, used)


def _cell_bounds(value: int) -> tuple[int, int]:
    return (0, 7) if int(value) <= 7 else (7, 14)


def _axis_tent(position: int, anchor: int, lo: int, hi: int) -> float:
    pos, peak = float(position), float(anchor)
    if pos < lo or pos > hi:
        return 0.0
    if pos == peak:
        return 1.0
    if pos < peak:
        if peak == lo:
            return 0.0
        return max(0.0, (pos - lo) / (peak - lo))
    if peak == hi:
        return 0.0
    return max(0.0, (hi - pos) / (hi - peak))
