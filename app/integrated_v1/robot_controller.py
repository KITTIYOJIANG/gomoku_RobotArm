from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import logging
from typing import Any, Mapping

from app.arm.actions import Action, ActionLibrary, ServoTarget
from app.arm.controller import SerialArmController
from app.arm.sequences import (
    ActionStep,
    SequenceDefinition,
    WaitStep,
    pick_piece,
    run_full_cycle,
)
from app.arm.worker import ArmSequenceWorker

from .golden import SPATIAL_KEYS, normalize_spatial
from .movel import DropStatus, MoveLPlanner
from .points import PointRef, parse_point_id
from .profile import CalibrationProfileManager, ProfileError


LOGGER = logging.getLogger(__name__)


class RobotState(str, Enum):
    IDLE = "IDLE"
    PICKING = "PICKING"
    LIFTING = "LIFTING"
    MOVING_TO_ABOVE = "MOVING_TO_ABOVE"
    DESCENDING = "DESCENDING"
    RELEASING = "RELEASING"
    RETRACTING = "RETRACTING"
    ERROR = "ERROR"
    STOPPED = "STOPPED"


class RobotExecutionBlocked(RuntimeError):
    pass


@dataclass(frozen=True)
class RobotGate:
    allowed: bool
    reasons: tuple[str, ...]


@dataclass(frozen=True)
class PlacementRequest:
    point: PointRef
    accepted: bool
    dry_run: bool
    state: RobotState
    reason: str
    sequence: SequenceDefinition | None = None


class RobotController:
    """The single high-level robot API shared by game and calibration UIs.

    It owns no second serial implementation. All transmission continues through
    the caller-provided stable SerialArmController and ArmSequenceWorker.
    """

    def __init__(
        self,
        *,
        serial_controller: SerialArmController,
        action_library: ActionLibrary,
        profile: CalibrationProfileManager,
        planner: MoveLPlanner | None = None,
        worker: ArmSequenceWorker | None = None,
        dry_run: bool | None = None,
        move_time_ms: int = 1000,
        vacuum_build_ms: int = 700,
        release_ms: int = 700,
        pump_hold_pwm: int = 2500,
        pump_off_pwm: int = 1500,
    ) -> None:
        self.serial = serial_controller
        self.actions = action_library
        self.profile = profile
        self.planner = planner or MoveLPlanner(profile)
        self.worker = worker
        self.dry_run = self.serial.dry_run if dry_run is None else bool(dry_run)
        if self.dry_run != self.serial.dry_run:
            raise ValueError("RobotController dry_run must match the single SerialArmController")
        self.move_time_ms = int(move_time_ms)
        self.vacuum_build_ms = int(vacuum_build_ms)
        self.release_ms = int(release_ms)
        self.pump_hold_pwm = int(pump_hold_pwm)
        self.pump_off_pwm = int(pump_off_pwm)
        self.state = RobotState.IDLE
        self.last_error: str | None = None
        self.active_point: PointRef | None = None
        self.last_sequence: SequenceDefinition | None = None
        if self.worker is not None:
            if self.worker.controller is not self.serial:
                raise ValueError("RobotController worker must share the exact serial controller instance")
            if self.worker.actions is not self.actions:
                raise ValueError("RobotController worker must share the exact action library instance")
            self.worker.step_started.connect(self._on_step_started)
            self.worker.sequence_finished.connect(self._on_sequence_finished)

    @property
    def is_busy(self) -> bool:
        return self.state not in {RobotState.IDLE, RobotState.ERROR, RobotState.STOPPED}

    def gate(self, point_id: str | PointRef | tuple[int, int], *, target_available: bool = True) -> RobotGate:
        point = parse_point_id(point_id)
        reasons: list[str] = []
        if not self.serial.is_connected:
            reasons.append("Serial disconnected")
        if self.state == RobotState.STOPPED:
            reasons.append("Emergency stop is latched")
        elif self.state == RobotState.ERROR:
            reasons.append("Robot is in ERROR; recover explicitly")
        elif self.is_busy or (self.worker is not None and self.worker.busy):
            reasons.append("Robot busy")
        status = self.profile.status()
        if not status.valid:
            reasons.append("Calibration profile invalid: " + "; ".join(status.reasons))
        data = self.profile._require_data()
        if not bool(data.get("pickup", {}).get("valid")):
            reasons.append("Pickup calibration invalid")
        if not target_available:
            reasons.append(f"Target {point.point_id} is unavailable or occupied")
        drop = self.profile.drop_record(point)
        if drop is None:
            reasons.append(f"DROP not generated for {point.point_id}")
        else:
            drop_status = str(drop.get("status"))
            blocked = {
                DropStatus.NOT_GENERATED.value,
                DropStatus.MOVE_L_UNREACHABLE.value,
                DropStatus.INVALID.value,
            }
            if drop_status in blocked:
                reasons.append(f"DROP {drop_status}: {drop.get('reason') or '-'}")
            elif not self.dry_run and (
                not bool(drop.get("verified"))
                or str(drop.get("verification_level")) != "HARDWARE VERIFIED"
            ):
                reasons.append("Live placement requires a HARDWARE VERIFIED DROP")
        return RobotGate(not reasons, tuple(reasons))

    def preview_drop(self, point_id: str | PointRef | tuple[int, int]) -> dict[str, Any]:
        return self.planner.preview(point_id)

    def move_to_above(self, point_id: str | PointRef | tuple[int, int]) -> SequenceDefinition:
        point = parse_point_id(point_id)
        above = self.profile.above_pwm(point)
        name = f"V1_{point.row:02d}_{point.col:02d}_ABOVE_IDLE"
        self.actions.register_runtime(
            self._pose_action(name, above, pump_pwm=self.pump_off_pwm)
        )
        sequence = SequenceDefinition(
            name=f"V1_MOVE_ABOVE:{point.point_id}",
            display_name=f"Move ABOVE {point.point_id}",
            steps=(ActionStep("CARRY_HIGH_P77_IDLE"), ActionStep(name)),
            requires_board=True,
        )
        self._validate_sequence(sequence)
        return sequence

    def retract(self, point_id: str | PointRef | tuple[int, int]) -> SequenceDefinition:
        point = parse_point_id(point_id)
        drop = self._require_executable_drop(point, allow_unverified=self.dry_run)
        self._register_drop_actions(point, drop)
        prefix = self._prefix(point)
        reverse_names = self._reverse_action_names(point, drop, pump_pwm=self.pump_off_pwm)
        sequence = SequenceDefinition(
            name=f"V1_RETRACT:{point.point_id}",
            display_name=f"Retract {point.point_id}",
            steps=tuple(ActionStep(name) for name in reverse_names)
            + (ActionStep("CARRY_HIGH_P77_IDLE"), ActionStep("OBSERVE_IDLE")),
            requires_board=True,
        )
        self._validate_sequence(sequence)
        return sequence

    def move_to_drop_for_calibration(
        self, point_id: str | PointRef | tuple[int, int]
    ) -> SequenceDefinition:
        """Build an empty-tool/pump-off descent for explicit point verification."""

        point = parse_point_id(point_id)
        drop = self._require_executable_drop(point, allow_unverified=True)
        self._register_drop_actions(point, drop)
        prefix = self._prefix(point)
        steps = [
            ActionStep(f"{prefix}_WP_{int(item['index']):02d}_IDLE")
            for item in drop["waypoints"]
        ]
        if drop["drop_final_pwm"] != drop["drop_auto_pwm"]:
            steps.append(ActionStep(f"{prefix}_DROP_FINAL_RELEASE"))
        sequence = SequenceDefinition(
            name=f"V1_CAL_MOVE_DROP:{point.point_id}",
            display_name=f"Calibration Move DROP {point.point_id}",
            steps=tuple(steps),
            requires_board=True,
        )
        if len(sequence.action_names) < 2:
            raise RobotExecutionBlocked("calibration MoveL requires ABOVE and at least one DROP waypoint")
        return sequence

    def calibration_full_place_sequence(
        self, point_id: str | PointRef | tuple[int, int]
    ) -> SequenceDefinition:
        """Explicit operator-test sequence; it may use an unverified DROP.

        This method only builds a sequence. The UI must still show a live-motion
        confirmation and submit it through the one shared worker.
        """

        point = parse_point_id(point_id)
        base = self._build_place_sequence(point, allow_unverified=True)
        return SequenceDefinition(
            name=f"V1_CAL_FULL_PLACE:{point.point_id}",
            display_name=f"Calibration full place {point.point_id}",
            steps=base.steps,
            requires_board=base.requires_board,
        )

    def place_piece(
        self,
        point_id: str | PointRef | tuple[int, int],
        *,
        target_available: bool = True,
    ) -> PlacementRequest:
        point = parse_point_id(point_id)
        gate = self.gate(point, target_available=target_available)
        if not gate.allowed:
            reason = "; ".join(gate.reasons)
            LOGGER.warning("[V1][PLACE_BLOCKED] point=%s reasons=%s", point.point_id, reason)
            return PlacementRequest(point, False, self.dry_run, self.state, reason)
        try:
            sequence = self.build_place_sequence(point)
            self.active_point = point
            self.last_sequence = sequence
            self.state = RobotState.PICKING
            if self.worker is not None:
                if not self.worker.submit(sequence):
                    self.state = RobotState.IDLE
                    return PlacementRequest(
                        point, False, self.dry_run, self.state, "Arm worker rejected busy request"
                    )
            elif self.dry_run:
                self._execute_dry_run_immediately(sequence)
            else:
                raise RobotExecutionBlocked("live execution requires ArmSequenceWorker")
        except Exception as exc:
            self.state = RobotState.ERROR
            self.last_error = str(exc)
            LOGGER.exception("[V1][PLACE_FAILED] point=%s", point.point_id)
            return PlacementRequest(point, False, self.dry_run, self.state, str(exc))
        return PlacementRequest(
            point,
            True,
            self.dry_run,
            self.state,
            "DRY RUN PASS" if self.dry_run and self.worker is None else "accepted",
            sequence,
        )

    def build_place_sequence(
        self, point_id: str | PointRef | tuple[int, int]
    ) -> SequenceDefinition:
        return self._build_place_sequence(
            parse_point_id(point_id), allow_unverified=self.dry_run
        )

    def _build_place_sequence(
        self, point: PointRef, *, allow_unverified: bool
    ) -> SequenceDefinition:
        drop = self._require_executable_drop(point, allow_unverified=allow_unverified)
        self._register_drop_actions(point, drop)
        prefix = self._prefix(point)
        descent_names = [f"{prefix}_WP_{item['index']:02d}_HOLD" for item in drop["waypoints"]]
        # ABOVE is the first waypoint and must follow carry-high.
        steps: list[ActionStep | WaitStep] = list(pick_piece(self.vacuum_build_ms).steps)
        steps.append(ActionStep("CARRY_HIGH_P77_HOLD"))
        steps.extend(ActionStep(name) for name in descent_names)
        if drop["drop_final_pwm"] != drop["drop_auto_pwm"]:
            steps.append(ActionStep(f"{prefix}_DROP_FINAL_HOLD"))
        steps.append(ActionStep(f"{prefix}_DROP_FINAL_RELEASE"))
        steps.append(WaitStep("VACUUM RELEASE", self.release_ms))
        steps.extend(
            ActionStep(name)
            for name in self._reverse_action_names(point, drop, pump_pwm=self.pump_off_pwm)
        )
        steps.extend((ActionStep("CARRY_HIGH_P77_IDLE"), ActionStep("OBSERVE_IDLE")))
        sequence = SequenceDefinition(
            name=f"V1_PLACE:{point.point_id}",
            display_name=f"Place piece at {point.point_id}",
            steps=tuple(steps),
            requires_board=True,
        )
        self._validate_sequence(sequence)
        return sequence

    def legacy_p77_full_cycle(self) -> SequenceDefinition:
        """Expose the unchanged hardware-successful P77 sequence as a fallback."""

        return run_full_cycle(self.vacuum_build_ms, self.release_ms)

    def emergency_stop(self) -> None:
        if self.worker is not None:
            self.worker.cancel_pending()
        try:
            if self.serial.is_connected:
                self.serial.emergency_stop()
        finally:
            self.state = RobotState.STOPPED
            self.last_error = "Emergency stop latched"

    def recover(self) -> None:
        if self.state not in {RobotState.STOPPED, RobotState.ERROR}:
            raise RobotExecutionBlocked("robot recovery is only valid from STOPPED or ERROR")
        if self.worker is not None and self.worker.busy:
            raise RobotExecutionBlocked("cannot recover while worker is busy")
        self.state = RobotState.IDLE
        self.last_error = None
        self.active_point = None

    def _register_drop_actions(self, point: PointRef, drop: Mapping[str, Any]) -> None:
        prefix = self._prefix(point)
        waypoints = list(drop.get("waypoints") or [])
        if not waypoints:
            raise RobotExecutionBlocked(f"{point.point_id} has no MoveL waypoints")
        for item in waypoints:
            name = f"{prefix}_WP_{int(item['index']):02d}_HOLD"
            self.actions.register_runtime(
                self._pose_action(name, item["pwm"], pump_pwm=self.pump_hold_pwm)
            )
            idle_name = f"{prefix}_WP_{int(item['index']):02d}_IDLE"
            self.actions.register_runtime(
                self._pose_action(idle_name, item["pwm"], pump_pwm=self.pump_off_pwm)
            )
        self.actions.register_runtime(
            self._pose_action(
                f"{prefix}_DROP_FINAL_HOLD",
                drop["drop_final_pwm"],
                pump_pwm=self.pump_hold_pwm,
            )
        )
        self.actions.register_runtime(
            self._pose_action(
                f"{prefix}_DROP_FINAL_RELEASE",
                drop["drop_final_pwm"],
                pump_pwm=self.pump_off_pwm,
            )
        )

    def _reverse_action_names(
        self, point: PointRef, drop: Mapping[str, Any], *, pump_pwm: int
    ) -> list[str]:
        del pump_pwm  # actions were registered with both HOLD and IDLE variants
        prefix = self._prefix(point)
        names: list[str] = []
        if drop["drop_final_pwm"] != drop["drop_auto_pwm"]:
            last_index = int(drop["waypoints"][-1]["index"])
            names.append(f"{prefix}_WP_{last_index:02d}_IDLE")
        names.extend(
            f"{prefix}_WP_{int(index):02d}_IDLE"
            for index in drop.get("reverse_ascent_indices") or []
        )
        return names

    def _pose_action(
        self, name: str, pwm: Mapping[str | int, int], *, pump_pwm: int
    ) -> Action:
        spatial = normalize_spatial(pwm)
        values = [spatial[key] for key in SPATIAL_KEYS] + [int(pump_pwm), 1500, 1500]
        targets = tuple(
            ServoTarget(servo_id=index, pwm=value, time_ms=self.move_time_ms)
            for index, value in enumerate(values)
        )
        body = "".join(
            f"#{target.servo_id:03d}P{target.pwm:04d}T{target.time_ms:04d}!"
            for target in targets
        )
        return Action(name=name, command="{" + body + "}", targets=targets)

    def _require_executable_drop(
        self, point: PointRef, *, allow_unverified: bool
    ) -> dict[str, Any]:
        drop = self.profile.drop_record(point)
        if drop is None:
            raise RobotExecutionBlocked(f"DROP not generated for {point.point_id}")
        status = str(drop.get("status"))
        if status in {
            DropStatus.NOT_GENERATED.value,
            DropStatus.MOVE_L_UNREACHABLE.value,
            DropStatus.INVALID.value,
        }:
            raise RobotExecutionBlocked(f"DROP {status}: {drop.get('reason') or '-'}")
        if not allow_unverified and (
            not bool(drop.get("verified"))
            or str(drop.get("verification_level")) != "HARDWARE VERIFIED"
        ):
            raise RobotExecutionBlocked("live DROP is not HARDWARE VERIFIED")
        return drop

    @staticmethod
    def _prefix(point: PointRef) -> str:
        return f"V1_P{point.row:02d}_{point.col:02d}"

    @staticmethod
    def _validate_sequence(sequence: SequenceDefinition) -> None:
        names = sequence.action_names
        if not names:
            raise RobotExecutionBlocked("empty robot sequence")
        if sequence.name.startswith("V1_PLACE"):
            carry_hold = names.index("CARRY_HIGH_P77_HOLD")
            first_above = next(
                index for index, name in enumerate(names) if name.endswith("_WP_00_HOLD")
            )
            release = next(
                index for index, name in enumerate(names) if name.endswith("_DROP_FINAL_RELEASE")
            )
            carry_idle = names.index("CARRY_HIGH_P77_IDLE")
            observe = names.index("OBSERVE_IDLE", carry_idle)
            if not carry_hold < first_above < release < carry_idle < observe:
                raise RobotExecutionBlocked("unsafe V1 place ordering")
            ascent = [name for name in names[release + 1 : carry_idle] if "_WP_" in name]
            if not ascent:
                raise RobotExecutionBlocked("V1 place has no saved reverse ascent")

    def _execute_dry_run_immediately(self, sequence: SequenceDefinition) -> None:
        for step in sequence.steps:
            if isinstance(step, ActionStep):
                self._on_step_started(sequence.name, step.action_name)
                self.serial.send_action(self.actions.get(step.action_name))
        self.state = RobotState.IDLE
        self.active_point = None

    def _on_step_started(self, _sequence_name: str, action_name: str) -> None:
        name = str(action_name).upper()
        if "SOURCE_TOUCH" in name:
            self.state = RobotState.PICKING
        elif name == "OBSERVE_HOLD" or name == "CARRY_HIGH_P77_HOLD":
            self.state = RobotState.LIFTING
        elif name.endswith("_WP_00_HOLD"):
            self.state = RobotState.MOVING_TO_ABOVE
        elif "_WP_" in name and name.endswith("_HOLD"):
            self.state = RobotState.DESCENDING
        elif name.endswith("_DROP_FINAL_RELEASE"):
            self.state = RobotState.RELEASING
        elif "_WP_" in name and name.endswith("_IDLE"):
            self.state = RobotState.RETRACTING

    def _on_sequence_finished(self, name: str, success: bool, message: str) -> None:
        if not str(name).startswith("V1_"):
            return
        if success:
            self.state = RobotState.IDLE
            self.last_error = None
        else:
            self.state = RobotState.ERROR
            self.last_error = str(message)
        self.active_point = None
