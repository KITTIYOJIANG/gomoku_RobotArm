"""Player UI to robot-control interface contract.

The player UI may import this module, but it must not import PWM, IK, serial,
pump, MoveL, or calibration-storage internals.  The integrated adapter delegates
all real work to the existing high-level controller and composition-root
callbacks, preserving the single serial owner and worker.
"""

from __future__ import annotations

from collections.abc import Callable
from typing import Any, Protocol

from app.integrated_v1.points import PointRef, parse_point_id


IDLE = "IDLE"
BUSY = "BUSY"
CALIBRATION_REQUIRED = "CALIBRATION_REQUIRED"
UNREACHABLE = "UNREACHABLE"
SERIAL_ERROR = "SERIAL_ERROR"
STOPPED = "STOPPED"
SUCCESS = "SUCCESS"

NOT_CONNECTED = "NOT_CONNECTED"
INVALID_POINT = "INVALID_POINT"
CALIBRATION_MISSING = "CALIBRATION_MISSING"
MOTION_TIMEOUT = "MOTION_TIMEOUT"

DISCONNECTED = "DISCONNECTED"
CONNECTING = "CONNECTING"
MOVING = "MOVING"
PICKING = "PICKING"
PLACING = "PLACING"
CALIBRATING = "CALIBRATING"
ERROR = "ERROR"


def row_col_to_point_id(row: int, col: int) -> str:
    """Return the canonical public ID: ``P{row:02d}_{col:02d}``."""

    point = PointRef(int(row), int(col))
    return f"P{point.row:02d}_{point.col:02d}"


def point_id_to_row_col(point_id: str) -> tuple[int, int] | None:
    """Parse canonical IDs and the integrated controller's read-only aliases."""

    try:
        point = parse_point_id(point_id)
    except (TypeError, ValueError):
        return None
    return point.as_tuple()


class RobotInterface(Protocol):
    """The only eight robot operations visible to the player UI."""

    def connect(self) -> bool: ...

    def disconnect(self) -> bool: ...

    def get_status(self) -> dict[str, Any]: ...

    def place_piece(self, point_id: str) -> str: ...

    def move_above(self, point_id: str) -> str: ...

    def home(self) -> str: ...

    def stop(self) -> str: ...

    def calibration_ready(self) -> bool: ...


class IntegratedRobotInterface:
    """Adapt the existing guarded controller to the frozen UI contract.

    Motion callbacks are supplied by ``MainWindow`` so confirmations, ordered
    paths, state-machine checks, and worker serialization remain authoritative.
    This class never constructs a serial controller and never formats PWM.
    """

    def __init__(
        self,
        *,
        controller: Any,
        robot: Any,
        calibration_ready: Callable[[], bool],
        default_port: str,
        connect_action: Callable[[str], Any] | None = None,
        disconnect_action: Callable[[], Any] | None = None,
        move_above_action: Callable[[str], Any] | None = None,
        home_action: Callable[[], Any] | None = None,
        stop_action: Callable[[], Any] | None = None,
    ) -> None:
        self._controller = controller
        self._robot = robot
        self._calibration_ready = calibration_ready
        self._default_port = str(default_port)
        self._connect_action = connect_action
        self._disconnect_action = disconnect_action
        self._move_above_action = move_above_action
        self._home_action = home_action
        self._stop_action = stop_action
        self._error: str | None = None

    def connect(self) -> bool:
        if bool(self._controller.is_connected):
            return True
        try:
            if self._connect_action is not None:
                result = self._connect_action(self._default_port)
            else:
                result = self._controller.connect(self._default_port)
            connected = bool(self._controller.is_connected)
            if result is False or not connected:
                self._error = "Robot connection was not established"
                return False
            self._error = None
            return True
        except Exception as exc:
            self._error = str(exc)
            return False

    def disconnect(self) -> bool:
        try:
            if self._disconnect_action is not None:
                result = self._disconnect_action()
            else:
                result = self._controller.disconnect()
            if result is False:
                self._error = "Robot disconnect was rejected"
                return False
            self._error = None
            return True
        except Exception as exc:
            self._error = str(exc)
            return False

    def get_status(self) -> dict[str, Any]:
        state = str(getattr(getattr(self._robot, "state", IDLE), "value", getattr(self._robot, "state", IDLE)))
        busy = bool(getattr(self._robot, "is_busy", False))
        worker = getattr(self._robot, "worker", None)
        busy = busy or bool(worker is not None and worker.busy)
        active = getattr(self._robot, "active_point", None)
        current_point = None
        if active is not None:
            current_point = row_col_to_point_id(active.row, active.col)
        error = self._error or getattr(self._robot, "last_error", None)
        return {
            "connected": bool(self._controller.is_connected),
            "state": state if self._controller.is_connected else DISCONNECTED,
            "busy": busy,
            "calibration_ready": self.calibration_ready(),
            "current_point": current_point,
            "error": error,
        }

    def calibration_ready(self) -> bool:
        try:
            return bool(self._calibration_ready())
        except Exception as exc:
            self._error = str(exc)
            return False

    def place_piece(self, point_id: str) -> str:
        coord = point_id_to_row_col(point_id)
        if coord is None:
            self._error = f"Invalid point_id: {point_id!r}"
            return INVALID_POINT
        if not self._controller.is_connected:
            self._error = "Serial disconnected"
            return NOT_CONNECTED
        try:
            request = self._robot.place_piece(coord, target_available=True)
        except Exception as exc:
            self._error = str(exc)
            return SERIAL_ERROR
        if bool(request.accepted):
            self._error = None
            return SUCCESS
        self._error = str(request.reason)
        return self._map_rejection(self._error)

    def move_above(self, point_id: str) -> str:
        coord = point_id_to_row_col(point_id)
        if coord is None:
            self._error = f"Invalid point_id: {point_id!r}"
            return INVALID_POINT
        if not self._controller.is_connected:
            self._error = "Serial disconnected"
            return NOT_CONNECTED
        if self._move_above_action is None:
            self._error = "Safe Move ABOVE dispatcher is not bound"
            return SERIAL_ERROR
        canonical = row_col_to_point_id(*coord)
        try:
            result = self._move_above_action(canonical)
        except Exception as exc:
            self._error = str(exc)
            return SERIAL_ERROR
        if result is False:
            self._error = "Move ABOVE was rejected or cancelled"
            return BUSY if self.get_status()["busy"] else STOPPED
        self._error = None
        return SUCCESS

    def home(self) -> str:
        if not self._controller.is_connected:
            self._error = "Serial disconnected"
            return NOT_CONNECTED
        if self._home_action is None:
            self._error = "Safe Home dispatcher is not bound"
            return SERIAL_ERROR
        try:
            result = self._home_action()
        except Exception as exc:
            self._error = str(exc)
            return SERIAL_ERROR
        if result is False:
            self._error = "Home was rejected or cancelled"
            return BUSY if self.get_status()["busy"] else STOPPED
        self._error = None
        return SUCCESS

    def stop(self) -> str:
        try:
            if self._stop_action is not None:
                self._stop_action()
            else:
                self._robot.emergency_stop()
            self._error = "Emergency stop latched"
            return STOPPED
        except Exception as exc:
            self._error = str(exc)
            return SERIAL_ERROR

    @staticmethod
    def _map_rejection(reason: str) -> str:
        lowered = reason.casefold()
        if "disconnect" in lowered:
            return NOT_CONNECTED
        if "busy" in lowered:
            return BUSY
        if "unreachable" in lowered:
            return UNREACHABLE
        if "calibration" in lowered or "drop not generated" in lowered or "pickup" in lowered:
            return CALIBRATION_REQUIRED
        if "stop" in lowered:
            return STOPPED
        return SERIAL_ERROR
