from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any, Mapping

from app.config import PROJECT_ROOT
from app.stage5.safety import PwmSafetyLimits, validate_spatial_pwm

from .models import ToolPose, normalize_spatial_pwm


PLANAR_JOINTS = (0, 1, 2, 3)
AUXILIARY_JOINT = 4


class KinematicsError(ValueError):
    def __init__(self, code: str, message: str) -> None:
        super().__init__(message)
        self.code = str(code)


@dataclass(frozen=True)
class JointCalibration:
    joint_id: int
    role: str
    zero_pwm: float
    direction: float
    degrees_per_pwm: float
    angle_bias_deg: float
    kinematic_offset_deg: float
    pwm_min: int
    pwm_max: int
    participates: bool = True

    def pwm_to_angle(self, pwm: float) -> float:
        return (
            self.direction * (float(pwm) - self.zero_pwm) * self.degrees_per_pwm
            + self.angle_bias_deg
            + self.kinematic_offset_deg
        )

    def angle_to_pwm(self, angle_deg: float) -> int:
        denominator = self.direction * self.degrees_per_pwm
        if abs(denominator) < 1e-12:
            raise KinematicsError(
                "INVALID_JOINT_SCALE",
                f"joint {self.joint_id:03d} has zero PWM/angle scale",
            )
        raw = self.zero_pwm + (
            float(angle_deg) - self.angle_bias_deg - self.kinematic_offset_deg
        ) / denominator
        return int(round(raw))


@dataclass(frozen=True)
class KinematicsConfig:
    path: Path
    model_name: str
    model_status: str
    links_mm: dict[str, float]
    joints: dict[int, JointCalibration]
    pump_joint_id: int
    alpha_min_deg: float
    alpha_max_deg: float
    reach_tolerance_mm: float
    position_tolerance_mm: float
    alpha_tolerance_deg: float
    raw: dict[str, Any]

    @classmethod
    def load(
        cls,
        path: str | Path = PROJECT_ROOT / "config" / "arm_kinematics.json",
    ) -> "KinematicsConfig":
        source = Path(path)
        if not source.is_absolute():
            source = PROJECT_ROOT / source
        data = json.loads(source.read_text(encoding="utf-8"))
        links = {key: float(value) for key, value in data["links_mm"].items()}
        if set(links) != {"L0", "L1", "L2", "L3"}:
            raise KinematicsError("INVALID_LINK_CONFIG", "links must contain L0..L3")
        if any(value <= 0 for value in links.values()):
            raise KinematicsError("INVALID_LINK_CONFIG", "all link lengths must be positive")

        joints: dict[int, JointCalibration] = {}
        for raw_id, item in data["joints"].items():
            joint_id = int(raw_id)
            calibration = JointCalibration(
                joint_id=joint_id,
                role=str(item["role"]),
                zero_pwm=float(item["zero_pwm"]),
                direction=float(item["direction"]),
                degrees_per_pwm=float(item["degrees_per_pwm"]),
                angle_bias_deg=float(item.get("angle_bias_deg", 0.0)),
                kinematic_offset_deg=float(item.get("kinematic_offset_deg", 0.0)),
                pwm_min=int(item["pwm_min"]),
                pwm_max=int(item["pwm_max"]),
                participates=bool(item.get("participates_in_planar_kinematics", True)),
            )
            if calibration.direction not in (-1.0, 1.0):
                raise KinematicsError(
                    "INVALID_JOINT_DIRECTION",
                    f"joint {joint_id:03d} direction must be -1 or +1",
                )
            if calibration.degrees_per_pwm <= 0:
                raise KinematicsError(
                    "INVALID_JOINT_SCALE",
                    f"joint {joint_id:03d} degrees_per_pwm must be positive",
                )
            joints[joint_id] = calibration
        if set(joints) != {0, 1, 2, 3, 4}:
            raise KinematicsError("INVALID_JOINT_CONFIG", "joints must contain 000..004")

        alpha = data["tool_alpha_range_deg"]
        tolerance = data["numeric_tolerance"]
        return cls(
            path=source,
            model_name=str(data["model_name"]),
            model_status=str(data["model_status"]),
            links_mm=links,
            joints=joints,
            pump_joint_id=int(data["pump_joint_id"]),
            alpha_min_deg=float(alpha["min"]),
            alpha_max_deg=float(alpha["max"]),
            reach_tolerance_mm=float(tolerance["reach_mm"]),
            position_tolerance_mm=float(tolerance["roundtrip_position_mm"]),
            alpha_tolerance_deg=float(tolerance["roundtrip_alpha_deg"]),
            raw=data,
        )

    def safety_limits(self) -> PwmSafetyLimits:
        return PwmSafetyLimits(
            pwm_min=min(item.pwm_min for item in self.joints.values()),
            pwm_max=max(item.pwm_max for item in self.joints.values()),
            joint_min={joint_id: item.pwm_min for joint_id, item in self.joints.items()},
            joint_max={joint_id: item.pwm_max for joint_id, item in self.joints.items()},
            max_adjacent_delta={joint_id: 500 for joint_id in self.joints},
        )


@dataclass(frozen=True)
class _Candidate:
    pwm: dict[str, int]
    position_error_mm: float
    alpha_error_deg: float
    seed_distance: float


class ArmKinematics:
    """Factory-compatible 4-DOF Cartesian model plus passthrough joint 004."""

    def __init__(self, config: KinematicsConfig | None = None) -> None:
        self.config = config or KinematicsConfig.load()
        self.limits = self.config.safety_limits()

    def pwm_to_joint_angles(
        self,
        pwm_pose: Mapping[str | int, int],
    ) -> dict[int, float]:
        pwm = normalize_spatial_pwm(pwm_pose)
        return {
            joint_id: calibration.pwm_to_angle(pwm[f"{joint_id:03d}"])
            for joint_id, calibration in self.config.joints.items()
        }

    def joint_angles_to_pwm(
        self,
        angles: Mapping[int, float],
        *,
        auxiliary_004_pwm: int = 1500,
    ) -> dict[str, int]:
        result: dict[str, int] = {}
        for joint_id in PLANAR_JOINTS:
            if joint_id not in angles:
                raise KinematicsError(
                    "MISSING_JOINT_ANGLE",
                    f"missing joint angle {joint_id:03d}",
                )
            result[f"{joint_id:03d}"] = self.config.joints[joint_id].angle_to_pwm(
                float(angles[joint_id])
            )
        result["004"] = int(auxiliary_004_pwm)
        errors = validate_spatial_pwm(result, self.limits)
        if errors:
            raise KinematicsError("PWM_OUT_OF_RANGE", "; ".join(errors))
        return result

    def forward_kinematics(
        self,
        pwm_pose: Mapping[str | int, int],
    ) -> ToolPose:
        pwm = normalize_spatial_pwm(pwm_pose)
        angles = self.pwm_to_joint_angles(pwm)
        base = angles[0]
        shoulder = angles[1]
        elbow = angles[2]
        wrist = angles[3]
        alpha = shoulder - elbow + wrist

        l0 = self.config.links_mm["L0"]
        l1 = self.config.links_mm["L1"]
        l2 = self.config.links_mm["L2"]
        l3 = self.config.links_mm["L3"]
        base_r = math.radians(base)
        shoulder_r = math.radians(shoulder)
        forearm_r = math.radians(shoulder - elbow)
        alpha_r = math.radians(alpha)

        radius = (
            l1 * math.cos(shoulder_r)
            + l2 * math.cos(forearm_r)
            + l3 * math.cos(alpha_r)
        )
        z = (
            l0
            + l1 * math.sin(shoulder_r)
            + l2 * math.sin(forearm_r)
            + l3 * math.sin(alpha_r)
        )
        pose = ToolPose(
            x=radius * math.sin(base_r),
            y=radius * math.cos(base_r),
            z=z,
            alpha=alpha,
            auxiliary_004_pwm=int(pwm["004"]),
        )
        if not all(math.isfinite(value) for value in (pose.x, pose.y, pose.z, pose.alpha)):
            raise KinematicsError("NONFINITE_FK", "forward kinematics produced non-finite pose")
        return pose

    def inverse_kinematics(
        self,
        tool_pose: ToolPose,
        *,
        seed_pwm: Mapping[str | int, int] | None = None,
    ) -> dict[str, int]:
        if not all(
            math.isfinite(value)
            for value in (tool_pose.x, tool_pose.y, tool_pose.z, tool_pose.alpha)
        ):
            raise KinematicsError("NONFINITE_TARGET", "IK target contains non-finite value")
        if not self.config.alpha_min_deg <= tool_pose.alpha <= self.config.alpha_max_deg:
            raise KinematicsError(
                "ALPHA_OUT_OF_RANGE",
                f"alpha {tool_pose.alpha:.3f} outside "
                f"{self.config.alpha_min_deg}..{self.config.alpha_max_deg}",
            )

        l0 = self.config.links_mm["L0"]
        l1 = self.config.links_mm["L1"]
        l2 = self.config.links_mm["L2"]
        l3 = self.config.links_mm["L3"]
        alpha_r = math.radians(tool_pose.alpha)
        base_deg = math.degrees(math.atan2(tool_pose.x, tool_pose.y))
        wrist_radius = math.hypot(tool_pose.x, tool_pose.y) - l3 * math.cos(alpha_r)
        wrist_z = tool_pose.z - l0 - l3 * math.sin(alpha_r)
        distance_sq = wrist_radius * wrist_radius + wrist_z * wrist_z
        denominator = 2.0 * l1 * l2
        cosine = (distance_sq - l1 * l1 - l2 * l2) / denominator
        tolerance = self.config.reach_tolerance_mm
        if cosine < -1.0 - tolerance or cosine > 1.0 + tolerance:
            raise KinematicsError(
                "UNREACHABLE",
                f"target wrist distance {math.sqrt(distance_sq):.3f} mm is unreachable",
            )
        cosine = max(-1.0, min(1.0, cosine))
        relative_options = (math.acos(cosine), -math.acos(cosine))
        seed = (
            normalize_spatial_pwm(seed_pwm)
            if seed_pwm is not None
            else {f"{joint_id:03d}": 1500 for joint_id in range(5)}
        )

        candidates: list[_Candidate] = []
        rejected: list[str] = []
        for relative in relative_options:
            shoulder_r = math.atan2(wrist_z, wrist_radius) - math.atan2(
                l2 * math.sin(relative),
                l1 + l2 * math.cos(relative),
            )
            elbow_deg = -math.degrees(relative)
            shoulder_deg = math.degrees(shoulder_r)
            wrist_deg = tool_pose.alpha - shoulder_deg + elbow_deg
            try:
                pwm = self.joint_angles_to_pwm(
                    {
                        0: base_deg,
                        1: shoulder_deg,
                        2: elbow_deg,
                        3: wrist_deg,
                    },
                    auxiliary_004_pwm=tool_pose.auxiliary_004_pwm,
                )
            except KinematicsError as exc:
                rejected.append(f"{exc.code}:{exc}")
                continue
            roundtrip = self.forward_kinematics(pwm)
            position_error = math.sqrt(
                (roundtrip.x - tool_pose.x) ** 2
                + (roundtrip.y - tool_pose.y) ** 2
                + (roundtrip.z - tool_pose.z) ** 2
            )
            alpha_error = abs(roundtrip.alpha - tool_pose.alpha)
            seed_distance = sum(
                (int(pwm[key]) - int(seed[key])) ** 2 for key in ("000", "001", "002", "003")
            )
            candidates.append(
                _Candidate(
                    pwm=pwm,
                    position_error_mm=position_error,
                    alpha_error_deg=alpha_error,
                    seed_distance=float(seed_distance),
                )
            )

        if not candidates:
            detail = "; ".join(rejected) if rejected else "no elbow branch"
            raise KinematicsError("NO_SAFE_IK_SOLUTION", detail)
        best = min(
            candidates,
            key=lambda item: (
                item.seed_distance,
                item.position_error_mm,
                item.alpha_error_deg,
            ),
        )
        if (
            best.position_error_mm > self.config.position_tolerance_mm
            or best.alpha_error_deg > self.config.alpha_tolerance_deg
        ):
            raise KinematicsError(
                "IK_ROUNDTRIP_ERROR",
                f"position_error={best.position_error_mm:.3f}mm "
                f"alpha_error={best.alpha_error_deg:.3f}deg",
            )
        return dict(best.pwm)
