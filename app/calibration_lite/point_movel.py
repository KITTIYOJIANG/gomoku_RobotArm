from __future__ import annotations

from copy import deepcopy
from datetime import datetime, timezone
import json
from pathlib import Path
from typing import Any, Mapping

from app.arm.actions import Action, ActionLibrary, ServoTarget
from app.arm.sequences import ActionStep, SequenceDefinition
from app.integrated_v1.profile import ProfileError


SPATIAL_JOINTS = ("J0", "J1", "J2", "J3", "J4")
P77_POINT_ID = "P07_07"
P77_DELTA_PREDICTION_SOURCE = "p77_delta_v1"


def _now() -> str:
    return datetime.now(timezone.utc).astimezone().isoformat(timespec="seconds")


class PointMoveLStore:
    """Manual ABOVE -> DROP calibration for arbitrary board points."""

    def __init__(
        self,
        path: str | Path,
        *,
        joint_limits: Mapping[str, tuple[int, int]] | None = None,
    ) -> None:
        self.path = Path(path)

        self.joint_limits = dict(
            joint_limits
            or {joint: (550, 2450) for joint in SPATIAL_JOINTS}
        )

        self.data: dict[str, Any] = {
            "schema_version": 1,
            "product": "J1 Gomoku Point MoveL Calibration",
            "updated_at": _now(),
            "points": {},
        }

        if self.path.is_file():
            self.load()

    def _normalize_pwm(
        self,
        pwm: Mapping[str, int],
    ) -> dict[str, int]:
        result: dict[str, int] = {}

        for joint in SPATIAL_JOINTS:
            if joint not in pwm:
                raise ProfileError(f"missing {joint}")

            value = int(pwm[joint])
            low, high = self.joint_limits[joint]

            if not low <= value <= high:
                raise ProfileError(
                    f"{joint} PWM {value} outside [{low}, {high}]"
                )

            result[joint] = value

        return result

    def load(self) -> None:
        raw = json.loads(self.path.read_text(encoding="utf-8"))

        if int(raw.get("schema_version", 0)) != 1:
            raise ProfileError("invalid Point MoveL calibration schema")
        if raw.get("product") != "J1 Gomoku Point MoveL Calibration":
            raise ProfileError("invalid Point MoveL calibration product")
        if not isinstance(raw.get("points"), dict):
            raise ProfileError("Point MoveL calibration points must be an object")

        self.data = raw

    def save(self) -> Path:
        self.data["updated_at"] = _now()

        self.path.parent.mkdir(parents=True, exist_ok=True)

        temp = self.path.with_suffix(self.path.suffix + ".tmp")

        temp.write_text(
            json.dumps(
                self.data,
                ensure_ascii=False,
                indent=2,
            )
            + "\n",
            encoding="utf-8",
        )

        temp.replace(self.path)

        return self.path

    def point(
        self,
        point_id: str,
    ) -> dict[str, Any] | None:
        record = self.data["points"].get(point_id)

        return None if record is None else deepcopy(record)

    def save_drop(
        self,
        *,
        point_id: str,
        board: tuple[int, int],
        above_pwm: Mapping[str, int],
        drop_pwm: Mapping[str, int],
        operator_confirmed: bool = False,
        hardware_verified: bool = False,
        predicted_drop_pwm: Mapping[str, int] | None = None,
        prediction_source: str | None = None,
        allow_hardware_verified_overwrite: bool = False,
    ) -> dict[str, Any]:
        existing = self.point(point_id)
        if (
            existing is not None
            and bool(existing.get("hardware_verified", False))
            and (
                point_id == P77_POINT_ID
                or not bool(allow_hardware_verified_overwrite)
            )
        ):
            raise ProfileError(
                f"{point_id} is HARDWARE VERIFIED and cannot be overwritten"
            )
        above = self._normalize_pwm(above_pwm)
        drop = self._normalize_pwm(drop_pwm)

        delta = {
            joint: drop[joint] - above[joint]
            for joint in SPATIAL_JOINTS
        }

        record = {
            "point_id": point_id,
            "board": [int(board[0]), int(board[1])],
            "above_pwm": above,
            "drop_pwm": drop,
            "delta_pwm": delta,
            "operator_confirmed": bool(operator_confirmed),
            "hardware_verified": bool(hardware_verified),
            "saved_at": _now(),
        }

        if predicted_drop_pwm is not None:
            predicted = self._normalize_pwm(predicted_drop_pwm)
            predicted_delta = {
                joint: predicted[joint] - above[joint]
                for joint in SPATIAL_JOINTS
            }
            record["prediction_source"] = str(
                prediction_source or P77_DELTA_PREDICTION_SOURCE
            )
            record["model_prediction"] = {
                "predicted_drop_pwm": predicted,
                "predicted_delta_pwm": predicted_delta,
            }
            record["residual_pwm"] = {
                joint: drop[joint] - predicted[joint]
                for joint in SPATIAL_JOINTS
            }

        self.data["points"][point_id] = record

        return deepcopy(record)

    def initial_drop_from_p77_delta(
        self,
        above_pwm: Mapping[str, int],
    ) -> dict[str, int]:
        """Return an unverified guess using the saved Golden P77 delta."""
        above = self._normalize_pwm(above_pwm)
        p77 = self.point(P77_POINT_ID)
        if p77 is None:
            raise ProfileError("Golden P07_07 is required for the V1 initial guess")
        delta = p77.get("delta_pwm") or {}
        if set(delta) != set(SPATIAL_JOINTS):
            raise ProfileError("Golden P07_07 delta must contain J0..J4")
        return self._normalize_pwm(
            {
                joint: above[joint] + int(delta[joint])
                for joint in SPATIAL_JOINTS
            }
        )

    def confirm_hardware(self, point_id: str) -> dict[str, Any]:
        record = self.data["points"].get(str(point_id))
        if record is None:
            raise ProfileError(f"save {point_id} before hardware confirmation")
        record["operator_confirmed"] = True
        record["hardware_verified"] = True
        record["confirmed_at"] = _now()
        return deepcopy(record)


class PointMoveLSequenceBuilder:
    """Direct arbitrary-point DROP/return actions through the shared worker."""

    def __init__(
        self,
        *,
        actions: ActionLibrary,
        store: PointMoveLStore,
        move_time_ms: int = 1000,
    ) -> None:
        self.actions = actions
        self.store = store
        self.move_time_ms = int(move_time_ms)

    def build_move_drop(
        self,
        point_id: str,
        drop_pwm: Mapping[str, int],
    ) -> SequenceDefinition:
        name = f"POINT_MOVEL_{point_id}_DROP_CANDIDATE"
        self._register_spatial(name, drop_pwm)
        return SequenceDefinition(
            name=f"MANUAL:POINT_MOVEL:DROP:{point_id}",
            display_name=f"Point MoveL direct DROP {point_id}",
            steps=(ActionStep(name),),
            requires_board=True,
        )

    def build_return_above(
        self,
        point_id: str,
        above_pwm: Mapping[str, int],
    ) -> SequenceDefinition:
        name = f"POINT_MOVEL_{point_id}_RETURN_ABOVE"
        self._register_spatial(name, above_pwm)
        return SequenceDefinition(
            name=f"MANUAL:POINT_MOVEL:RETURN_ABOVE:{point_id}",
            display_name=f"Point MoveL return to ABOVE {point_id}",
            steps=(ActionStep(name),),
            requires_board=True,
        )

    def _register_spatial(
        self,
        name: str,
        pwm: Mapping[str, int],
    ) -> None:
        values = self.store._normalize_pwm(pwm)
        targets = tuple(
            ServoTarget(joint, values[f"J{joint}"], self.move_time_ms)
            for joint in range(5)
        )
        command = "{" + "".join(
            f"#{target.servo_id:03d}P{target.pwm:04d}T{target.time_ms:04d}!"
            for target in targets
        ) + "}"
        self.actions.register_runtime(Action(name=name, command=command, targets=targets))
