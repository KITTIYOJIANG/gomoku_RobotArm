"""Gomoku Robot Integrated V1 application services.

This package is deliberately layered on top of the stable Stage 5/arm code.
It never rewrites the stable calibration or action-table assets in place.
"""

from .golden import GOLDEN_ABOVE, assert_golden_above
from .movel import DropStatus, GenerateAllSummary, MoveLPlanner
from .points import PointRef, format_point_id, parse_point_id
from .profile import CalibrationProfileManager, ProfileError, ProfileStatus
from .robot_controller import RobotController, RobotExecutionBlocked, RobotState
from .settings import IntegratedV1Settings

__all__ = [
    "CalibrationProfileManager",
    "GOLDEN_ABOVE",
    "DropStatus",
    "GenerateAllSummary",
    "IntegratedV1Settings",
    "MoveLPlanner",
    "PointRef",
    "ProfileError",
    "ProfileStatus",
    "RobotController",
    "RobotExecutionBlocked",
    "RobotState",
    "assert_golden_above",
    "format_point_id",
    "parse_point_id",
]
