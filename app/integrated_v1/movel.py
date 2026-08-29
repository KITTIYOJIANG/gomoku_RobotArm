from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
from datetime import datetime, timezone
from enum import Enum
import math
from typing import Any, Mapping

from app.stage6.kinematics import ArmKinematics, KinematicsError
from app.stage6.models import ToolPose

from .golden import SPATIAL_KEYS, golden_for, normalize_spatial
from .points import BOARD_SIZE, PointRef, all_points, parse_point_id
from .profile import CalibrationProfileManager, ProfileError


class DropStatus(str, Enum):
    NOT_GENERATED = "NOT_GENERATED"
    GENERATED = "GENERATED"
    PENDING_VERIFY = "PENDING_VERIFY"
    VERIFIED = "VERIFIED"
    MOVE_L_UNREACHABLE = "MOVE_L_UNREACHABLE"
    MANUAL_CORRECTED = "MANUAL_CORRECTED"
    INVALID = "INVALID"


@dataclass(frozen=True)
class GenerateAllSummary:
    requested: int
    success: int
    unreachable: int
    invalid: int
    skipped: int
    golden_started: int

    def to_dict(self) -> dict[str, int]:
        return {
            "Requested": self.requested,
            "Success": self.success,
            "Unreachable": self.unreachable,
            "Invalid": self.invalid,
            "Skipped": self.skipped,
            "Golden Started": self.golden_started,
        }


def _now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


class MoveLPlanner:
    """Offline Cartesian-Z descent generator with continuous seeded IK.

    The planner has no controller/serial dependency. Generate All therefore
    cannot move hardware by construction.
    """

    def __init__(
        self,
        profile: CalibrationProfileManager,
        *,
        kinematics: ArmKinematics | None = None,
        target_descent_mm: float = 25.0,
        step_mm: float = 5.0,
        max_waypoint_joint_delta_pwm: int = 400,
    ) -> None:
        self.profile = profile
        self.kinematics = kinematics or ArmKinematics()
        self.target_descent_mm = float(target_descent_mm)
        self.step_mm = float(step_mm)
        self.max_waypoint_joint_delta_pwm = int(max_waypoint_joint_delta_pwm)
        if not math.isfinite(self.target_descent_mm) or self.target_descent_mm <= 0:
            raise ValueError("target_descent_mm must be finite and positive")
        if not math.isfinite(self.step_mm) or self.step_mm <= 0:
            raise ValueError("step_mm must be finite and positive")
        if self.max_waypoint_joint_delta_pwm <= 0:
            raise ValueError("max_waypoint_joint_delta_pwm must be positive")

    def generate_point(
        self,
        point_id: str | PointRef | tuple[int, int],
        *,
        persist: bool = True,
    ) -> dict[str, Any]:
        point = parse_point_id(point_id)
        above = self.profile.above_pwm(point)
        previous_drop = self.profile.drop_record(point) or {}
        manual_calibration = previous_drop.get("manual_movel_calibration")
        if manual_calibration is not None:
            if point.as_tuple() != (7, 7):
                raise ProfileError("manual MoveL calibration is supported only for P77")
            if manual_calibration.get("source") != "manual_movel_tuning":
                raise ProfileError("P77 manual MoveL calibration source is invalid")
            if not bool(manual_calibration.get("operator_confirmed", False)):
                raise ProfileError("P77 manual MoveL calibration is not confirmed")
            if bool(manual_calibration.get("hardware_verified", False)):
                raise ProfileError(
                    "P77 manual MoveL calibration cannot self-assert hardware verification"
                )
        correction = normalize_spatial(
            previous_drop.get("drop_correction_pwm")
            or {joint: 0 for joint in SPATIAL_KEYS}
        )
        timestamp = _now()
        base_record: dict[str, Any] = {
            "point_id": point.point_id,
            "board": [point.row, point.col],
            "above_pwm": dict(above),
            "drop_auto_pwm": None,
            "drop_correction_pwm": correction,
            "drop_final_pwm": None,
            "status": DropStatus.GENERATED.value,
            "verified": False,
            "verification_level": "NOT VERIFIED",
            "generation_method": "PWM_TO_JOINT_FK_CARTESIAN_Z_WAYPOINTS_CONTINUOUS_IK_TO_PWM",
            "target_descent_mm": self.target_descent_mm,
            "step_mm": self.step_mm,
            "max_safe_descent_mm": 0.0,
            "failed_waypoint": None,
            "reason": None,
            "waypoints": [],
            "reverse_ascent_indices": [],
            "golden_started": golden_for(point.row, point.col) is not None,
            "generation_time": timestamp,
            "verification_time": None,
            "notes": "",
        }
        if manual_calibration is not None:
            base_record["manual_movel_calibration"] = deepcopy(manual_calibration)
        try:
            above_pose = self.kinematics.forward_kinematics(above)
            self._assert_finite_pose(above_pose)
        except Exception as exc:
            record = self._failed_record(
                base_record,
                status=DropStatus.INVALID,
                waypoint_index=0,
                descent_mm=0.0,
                reason=self._reason(exc),
            )
            return self._store(point, record, persist)

        base_record["above_cartesian_pose"] = above_pose.to_dict()
        base_record["waypoints"].append(
            self._waypoint(0, 0.0, above_pose, above, source="final_above")
        )
        seed = dict(above)
        distances = self._descent_distances()
        for index, descent in enumerate(distances, start=1):
            target = ToolPose(
                x=above_pose.x,
                y=above_pose.y,
                z=above_pose.z - descent,
                alpha=above_pose.alpha,
                auxiliary_004_pwm=above_pose.auxiliary_004_pwm,
            )
            try:
                self._assert_finite_pose(target)
                solved = self.kinematics.inverse_kinematics(target, seed_pwm=seed)
                solved = normalize_spatial(solved)
                self._assert_continuity(seed, solved, point, index)
            except Exception as exc:
                record = self._failed_record(
                    base_record,
                    status=DropStatus.MOVE_L_UNREACHABLE,
                    waypoint_index=index,
                    descent_mm=descent,
                    reason=self._reason(exc),
                )
                if base_record["waypoints"]:
                    last = base_record["waypoints"][-1]
                    record["best_safe_pose"] = {
                        "status": "SUGGESTED_NOT_VERIFIED",
                        "descent_mm": last["descent_mm"],
                        "pwm": dict(last["pwm"]),
                        "cartesian_pose": dict(last["cartesian_pose"]),
                    }
                return self._store(point, record, persist)
            base_record["waypoints"].append(
                self._waypoint(index, descent, target, solved, source="continuous_seeded_ik")
            )
            base_record["max_safe_descent_mm"] = descent
            seed = solved

        auto = dict(seed)
        if manual_calibration is not None:
            manual_final = self.profile._validate_pwm(
                manual_calibration["final_pwm"]
            )
            correction = {
                joint: int(manual_final[joint]) - int(auto[joint])
                for joint in SPATIAL_KEYS
            }
            base_record["drop_correction_pwm"] = correction
        try:
            final = self.profile._validate_pwm(  # central profile limit policy
                {joint: auto[joint] + correction[joint] for joint in SPATIAL_KEYS}
            )
        except ProfileError as exc:
            record = self._failed_record(
                base_record,
                status=DropStatus.INVALID,
                waypoint_index=len(base_record["waypoints"]),
                descent_mm=self.target_descent_mm,
                reason=f"DROP_CORRECTION_INVALID:{exc}",
            )
            return self._store(point, record, persist)
        base_record["drop_auto_pwm"] = auto
        base_record["drop_final_pwm"] = final
        base_record["status"] = (
            DropStatus.MANUAL_CORRECTED.value
            if any(correction.values())
            else DropStatus.PENDING_VERIFY.value
        )
        # Exact saved reverse path; no recomputation or shortcut is permitted.
        base_record["reverse_ascent_indices"] = list(
            range(len(base_record["waypoints"]) - 2, -1, -1)
        )
        record = self._store(point, base_record, persist)
        if base_record["golden_started"]:
            golden = golden_for(point.row, point.col)
            assert golden is not None
            if record["waypoints"][0]["pwm"] != golden.pwm_map:
                raise AssertionError(f"MoveL did not start from {golden.legacy_id} Golden ABOVE")
        return record

    def generate_all(self, *, persist: bool = True) -> GenerateAllSummary:
        success = unreachable = invalid = skipped = golden_started = 0
        for point in all_points():
            try:
                record = self.generate_point(point, persist=False)
            except Exception as exc:  # defensive: retain a per-point INVALID record
                record = {
                    "point_id": point.point_id,
                    "board": [point.row, point.col],
                    "status": DropStatus.INVALID.value,
                    "verified": False,
                    "verification_level": "NOT VERIFIED",
                    "reason": self._reason(exc),
                    "generation_time": _now(),
                }
                self.profile.set_drop_record(point, record)
            status = record.get("status")
            if status == DropStatus.MOVE_L_UNREACHABLE.value:
                unreachable += 1
            elif status == DropStatus.INVALID.value:
                invalid += 1
            elif status == DropStatus.NOT_GENERATED.value:
                skipped += 1
            else:
                success += 1
            golden_started += int(bool(record.get("golden_started")))
        summary = GenerateAllSummary(
            requested=BOARD_SIZE * BOARD_SIZE,
            success=success,
            unreachable=unreachable,
            invalid=invalid,
            skipped=skipped,
            golden_started=golden_started,
        )
        data = self.profile._require_data()
        data["drop"]["last_generate_all"] = {
            **summary.to_dict(),
            "timestamp": _now(),
            "execution": "OFFLINE_ONLY_NO_CONTROLLER_REFERENCE",
        }
        data["history"].append(
            {"timestamp": _now(), "event": "GENERATE_ALL_DROP_OFFLINE", **summary.to_dict()}
        )
        if persist:
            self.profile.save()
        return summary

    def preview(self, point_id: str | PointRef | tuple[int, int]) -> dict[str, Any]:
        point = parse_point_id(point_id)
        record = self.profile.drop_record(point)
        if record is None:
            raise ProfileError(f"DROP not generated for {point.point_id}")
        waypoints = list(record.get("waypoints") or [])
        reverse = [waypoints[index] for index in record.get("reverse_ascent_indices") or []]
        return {
            "point_id": point.point_id,
            "status": record.get("status"),
            "hardware_execution": False,
            "descent": waypoints,
            "drop_final_pwm": record.get("drop_final_pwm"),
            "reverse_ascent": reverse,
            "reason": record.get("reason"),
        }

    def _store(self, point: PointRef, record: dict[str, Any], persist: bool) -> dict[str, Any]:
        stored = self.profile.set_drop_record(point, record)
        if persist:
            self.profile.save()
        return stored

    def _descent_distances(self) -> tuple[float, ...]:
        count = int(math.floor(self.target_descent_mm / self.step_mm))
        values = [self.step_mm * index for index in range(1, count + 1)]
        if not values or not math.isclose(values[-1], self.target_descent_mm, abs_tol=1e-9):
            values.append(self.target_descent_mm)
        return tuple(float(value) for value in values)

    def _assert_continuity(
        self,
        previous: Mapping[str, int],
        current: Mapping[str, int],
        point: PointRef,
        waypoint_index: int,
    ) -> None:
        jumps = {
            joint: abs(int(current[joint]) - int(previous[joint])) for joint in SPATIAL_KEYS
        }
        worst_joint = max(jumps, key=jumps.get)
        if jumps[worst_joint] > self.max_waypoint_joint_delta_pwm:
            raise KinematicsError(
                "IK_DISCONTINUITY",
                f"{point.point_id} waypoint {waypoint_index} joint {worst_joint} "
                f"jump {jumps[worst_joint]} exceeds {self.max_waypoint_joint_delta_pwm}",
            )
        if int(current["004"]) != int(previous["004"]):
            raise KinematicsError("J4_DISCONTINUITY", "auxiliary joint 004 must stay constant")

    @staticmethod
    def _assert_finite_pose(pose: ToolPose) -> None:
        if not all(math.isfinite(value) for value in (pose.x, pose.y, pose.z, pose.alpha)):
            raise KinematicsError("NONFINITE_POSE", "Cartesian pose contains NaN or infinity")

    @staticmethod
    def _waypoint(
        index: int,
        descent_mm: float,
        pose: ToolPose,
        pwm: Mapping[str, int],
        *,
        source: str,
    ) -> dict[str, Any]:
        return {
            "index": int(index),
            "descent_mm": float(descent_mm),
            "cartesian_pose": pose.to_dict(),
            "pwm": normalize_spatial(pwm),
            "source": source,
        }

    @staticmethod
    def _failed_record(
        record: dict[str, Any],
        *,
        status: DropStatus,
        waypoint_index: int,
        descent_mm: float,
        reason: str,
    ) -> dict[str, Any]:
        record["status"] = status.value
        record["verified"] = False
        record["verification_level"] = "NOT VERIFIED"
        record["failed_waypoint"] = {
            "index": int(waypoint_index),
            "descent_mm": float(descent_mm),
        }
        record["reason"] = reason
        return record

    @staticmethod
    def _reason(exc: Exception) -> str:
        code = getattr(exc, "code", exc.__class__.__name__)
        return f"{code}:{exc}"
