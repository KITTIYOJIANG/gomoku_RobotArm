from __future__ import annotations

import math

import pytest

from app.arm.actions import ActionLibrary
from app.stage6.kinematics import ArmKinematics, KinematicsConfig, KinematicsError
from app.stage6.models import ToolPose


def spatial(action_name: str) -> dict[str, int]:
    action = ActionLibrary().get(action_name)
    return {
        f"{joint_id:03d}": action.target(joint_id).pwm for joint_id in range(5)
    }


def test_stage6_fk_input_output_is_finite() -> None:
    kin = ArmKinematics()
    pose = kin.forward_kinematics(spatial("P77_ABOVE_IDLE"))
    assert all(math.isfinite(value) for value in (pose.x, pose.y, pose.z, pose.alpha))
    assert pose.auxiliary_004_pwm == 1500


def test_stage6_pwm_angle_conversions_are_inverse() -> None:
    config = KinematicsConfig.load()
    for joint_id, calibration in config.joints.items():
        for pwm in (calibration.pwm_min, 1000, 1500, 2000, calibration.pwm_max):
            if calibration.pwm_min <= pwm <= calibration.pwm_max:
                angle = calibration.pwm_to_angle(pwm)
                assert calibration.angle_to_pwm(angle) == pwm, joint_id


def test_stage6_fk_ik_roundtrip_reproduces_p77_endpoints() -> None:
    kin = ArmKinematics()
    for action_name in ("P77_ABOVE_IDLE", "P77_TOUCH_HOLD"):
        pwm = spatial(action_name)
        pose = kin.forward_kinematics(pwm)
        assert kin.inverse_kinematics(pose, seed_pwm=pwm) == pwm


def test_stage6_p77_calibration_has_vertical_xy_and_measured_drop() -> None:
    kin = ArmKinematics()
    above = kin.forward_kinematics(spatial("P77_ABOVE_IDLE"))
    touch = kin.forward_kinematics(spatial("P77_TOUCH_HOLD"))
    assert touch.x == pytest.approx(above.x, abs=1e-9)
    assert touch.y == pytest.approx(above.y, abs=1e-9)
    assert touch.z - above.z == pytest.approx(-51.3028747911, abs=1e-6)
    assert touch.alpha - above.alpha == pytest.approx(-12.15, abs=1e-6)


def test_stage6_unreachable_target_is_explicitly_rejected() -> None:
    kin = ArmKinematics()
    with pytest.raises(KinematicsError, match="unreachable") as caught:
        kin.inverse_kinematics(ToolPose(10000.0, 10000.0, 10000.0, -55.0))
    assert caught.value.code == "UNREACHABLE"


def test_stage6_pump_is_not_part_of_kinematics() -> None:
    kin = ArmKinematics()
    pwm = spatial("P77_ABOVE_IDLE")
    pose_without = kin.forward_kinematics(pwm)
    pose_with = kin.forward_kinematics({**pwm, "005": 2500})
    assert pose_with == pose_without
    solved = kin.inverse_kinematics(pose_with, seed_pwm=pwm)
    assert set(solved) == {"000", "001", "002", "003", "004"}
