from __future__ import annotations

from dataclasses import dataclass
import logging
import time
from typing import Any, Mapping

from app.arm.actions import ActionLibrary
from app.arm.controller import SerialArmController
from app.stage5.safety import validate_spatial_pwm

from .above_source import ReadOnlyAboveSource
from .calibration_store import Stage6CalibrationError, Stage6CalibrationStore
from .kinematics import ArmKinematics, KinematicsConfig, KinematicsError
from .models import (
    DESCENT_LEVELS,
    SPATIAL_KEYS,
    DescentLevel,
    DescentLevelPose,
    DescentProfile,
    LevelStatus,
    ToolPose,
    VerificationStage,
)
from .residuals import ResidualCorrector
from .settings import Stage6Settings
from .state_machine import Stage6MotionState, Stage6StateMachine
from .thermal import ThermalGuard


LOGGER = logging.getLogger(__name__)
P77 = (7, 7)


class Stage6PlanningError(RuntimeError):
    pass


class Stage6ExecutionBlocked(RuntimeError):
    pass


@dataclass(frozen=True)
class BatchGenerationResult:
    requested: int
    generated: dict[tuple[int, int], DescentProfile]
    rejected: dict[tuple[int, int], str]

    @property
    def generated_count(self) -> int:
        return len(self.generated)

    @property
    def rejected_count(self) -> int:
        return len(self.rejected)


class Stage6DescentPlanner:
    """Single Stage6 API for generation, preview and guarded execution."""

    def __init__(
        self,
        *,
        controller: SerialArmController,
        library: ActionLibrary | None = None,
        settings: Stage6Settings | None = None,
        calibration_store: Stage6CalibrationStore | None = None,
    ) -> None:
        self.controller = controller
        self.library = library or ActionLibrary()
        self.settings = settings or Stage6Settings.load()
        self.kinematics = ArmKinematics(
            KinematicsConfig.load(self.settings.kinematics_path)
        )
        self.above_source = ReadOnlyAboveSource(
            self.settings, self.kinematics, library=self.library
        )
        self.store = calibration_store or Stage6CalibrationStore(
            self.settings.descent_calibration_path,
            safety_limits=self.kinematics.limits,
        )
        self.store.safety_limits = self.kinematics.limits
        self.store.set_source_fingerprint(
            self.settings.above_calibration_path,
            self.above_source.source_sha256,
        )
        self.state = Stage6StateMachine()
        self.thermal = ThermalGuard(
            max_above_dwell_seconds=self.settings.max_above_dwell_seconds,
            max_touch_dwell_seconds=self.settings.max_touch_dwell_seconds,
            max_tweaks_per_session=self.settings.max_tweaks_per_session,
            max_continuous_actions=self.settings.max_continuous_actions,
        )
        self._reference = self._calculate_p77_reference()
        self.last_p77_dry_run_events: list[str] = []

    @property
    def reference_drop(self) -> dict[str, float]:
        return dict(self._reference)

    def _calculate_p77_reference(self) -> dict[str, float]:
        above = self._action_spatial("P77_ABOVE_IDLE")
        touch = self._action_spatial("P77_TOUCH_HOLD")
        pose_above = self.kinematics.forward_kinematics(above)
        pose_touch = self.kinematics.forward_kinematics(touch)
        return {
            "dx": pose_touch.x - pose_above.x,
            "dy": pose_touch.y - pose_above.y,
            "dz": pose_touch.z - pose_above.z,
            "dalpha": pose_touch.alpha - pose_above.alpha,
        }

    def generate_descent_profile(
        self,
        row: int,
        col: int,
        *,
        persist: bool = True,
        use_residuals: bool = False,
    ) -> DescentProfile:
        row_i, col_i = int(row), int(col)
        above = self.above_source.get(row_i, col_i)
        if not above.model_valid:
            raise Stage6PlanningError(
                f"MODEL_INVALID P({row_i},{col_i}): {'; '.join(above.warnings)}"
            )
        levels: list[DescentLevelPose] = []
        warnings = list(above.warnings)
        seed = dict(above.pwm)
        residual_corrector = (
            ResidualCorrector(self.store.profiles()) if use_residuals else None
        )
        for level in DESCENT_LEVELS:
            if level == DescentLevel.ABOVE:
                computed = dict(above.pwm)
                target_pose = above.tool_pose
                source = (
                    "existing_verified_above"
                    if above.verified
                    else f"existing_resolved_above:{above.source}"
                )
                status = (
                    LevelStatus.VERIFIED if above.verified else LevelStatus.COMPUTED
                )
            elif (row_i, col_i) == P77 and level == DescentLevel.TOUCH:
                computed = self._action_spatial("P77_TOUCH_HOLD")
                target_pose = self.kinematics.forward_kinematics(computed)
                source = "stable_p77_touch_action"
                status = LevelStatus.VERIFIED
            else:
                fraction = level.fraction
                alpha = above.tool_pose.alpha
                if (row_i, col_i) == P77:
                    alpha += fraction * self._reference["dalpha"]
                target_pose = ToolPose(
                    x=above.tool_pose.x,
                    y=above.tool_pose.y,
                    z=above.tool_pose.z + fraction * self._reference["dz"],
                    alpha=alpha,
                    auxiliary_004_pwm=above.pwm["004"],
                )
                try:
                    computed = self.kinematics.inverse_kinematics(
                        target_pose, seed_pwm=seed
                    )
                except KinematicsError as exc:
                    raise Stage6PlanningError(
                        f"{exc.code} P({row_i},{col_i}) {level.value}: {exc}"
                    ) from exc
                source = "kinematics_generated"
                status = LevelStatus.COMPUTED
            self._check_layer(seed, computed, level)
            layer_warnings: list[str] = []
            manual_delta = {key: 0 for key in SPATIAL_KEYS}
            if residual_corrector is not None and level != DescentLevel.ABOVE:
                estimate = residual_corrector.estimate(row_i, col_i, level)
                if estimate is not None:
                    manual_delta = estimate.delta_pwm
                    source = "kinematics_plus_residual"
                    layer_warnings.append(
                        f"residual {estimate.method} from {estimate.anchors_used}"
                    )
                    residual_final = {
                        key: int(computed[key]) + int(manual_delta[key])
                        for key in SPATIAL_KEYS
                    }
                    errors = validate_spatial_pwm(
                        residual_final, self.kinematics.limits
                    )
                    if errors:
                        raise Stage6PlanningError(
                            "residual correction outside safety envelope: "
                            + "; ".join(errors)
                        )
            levels.append(
                DescentLevelPose(
                    level=level,
                    source=source,
                    tool_pose=target_pose,
                    computed_pwm=computed,
                    manual_delta_pwm=manual_delta,
                    status=status,
                    warnings=tuple(layer_warnings),
                )
            )
            seed = computed
        profile = DescentProfile(
            row=row_i,
            col=col_i,
            above_source=above.source,
            above_verified=above.verified,
            levels=tuple(levels),
            reverse_ascent=(
                DescentLevel.DESCENT_75,
                DescentLevel.DESCENT_50,
                DescentLevel.DESCENT_25,
                DescentLevel.ABOVE,
            ),
            model_valid=True,
            verification_stage=VerificationStage.COMPUTED,
            reverse_ascent_verified=False,
            warnings=tuple(sorted(set(warnings))),
        )
        if persist:
            self.store.upsert_profile(profile, save=True)
            self._log_stored_profile(row_i, col_i)
        else:
            LOGGER.info(
                "[STAGE6][GENERATE] P(%d,%d) computed=%s delta=%s final=%s",
                row_i,
                col_i,
                {item.level.value: item.computed_pwm for item in profile.levels},
                {item.level.value: item.manual_delta_pwm for item in profile.levels},
                {item.level.value: item.final_pwm for item in profile.levels},
            )
        return profile

    def generate_all_descent_profiles(
        self, *, persist: bool = True, use_residuals: bool = False
    ) -> BatchGenerationResult:
        above_records = self.above_source.resolve_all()
        generated: dict[tuple[int, int], DescentProfile] = {}
        rejected: dict[tuple[int, int], str] = {}
        for row in range(self.settings.board_size):
            for col in range(self.settings.board_size):
                try:
                    generated[(row, col)] = self.generate_descent_profile(
                        row,
                        col,
                        persist=False,
                        use_residuals=use_residuals,
                    )
                except (Stage6PlanningError, KinematicsError) as exc:
                    rejected[(row, col)] = str(exc)
        if persist:
            self.store.record_above_snapshot(
                {
                    f"{row},{col}": {
                        "row": record.row,
                        "col": record.col,
                        "pwm": dict(record.pwm),
                        "source": record.source,
                        "anchors_used": list(record.anchors_used),
                        "verified": record.verified,
                        "tool_pose": record.tool_pose.to_dict(),
                        "model_valid": record.model_valid,
                        "warnings": list(record.warnings),
                    }
                    for (row, col), record in sorted(above_records.items())
                },
                save=False,
            )
            for profile in generated.values():
                self.store.upsert_profile(profile, save=False)
            self.store.record_batch_result(
                requested=self.settings.board_size**2,
                generated_points=[
                    f"{row},{col}" for row, col in sorted(generated)
                ],
                rejected={
                    f"{row},{col}": reason
                    for (row, col), reason in sorted(rejected.items())
                },
                save=True,
            )
        result = BatchGenerationResult(
            requested=self.settings.board_size**2,
            generated=generated,
            rejected=rejected,
        )
        LOGGER.info(
            "[STAGE6][BATCH] requested=%d generated=%d rejected=%d",
            result.requested,
            result.generated_count,
            result.rejected_count,
        )
        return result

    def preview_descent_profile(self, row: int, col: int) -> dict[str, Any]:
        profile = self._ensure_profile(row, col)
        commands: list[dict[str, Any]] = []
        for item in profile.levels:
            pwm = self._stored_final_pwm(row, col, item.level)
            commands.append(
                {
                    "phase": "descent",
                    "level": item.level.value,
                    "pwm": pwm,
                    "command": self._pose_command(
                        pwm, self.settings.pump_hold_pwm
                    ),
                }
            )
        commands.append(
            {
                "phase": "release",
                "level": DescentLevel.TOUCH.value,
                "command": f"#005P{self.settings.pump_off_pwm:04d}T0500!",
                "dwell_ms": self.settings.release_dwell_ms,
            }
        )
        for level in profile.reverse_ascent:
            pwm = self._stored_final_pwm(row, col, level)
            commands.append(
                {
                    "phase": "reverse_ascent",
                    "level": level.value,
                    "pwm": pwm,
                    "command": self._pose_command(
                        pwm, self.settings.pump_off_pwm
                    ),
                }
            )
        reverse = [
            item["level"]
            for item in commands
            if item["phase"] == "reverse_ascent"
        ]
        result = {
            "target": {"row": int(row), "col": int(col)},
            "dry_run_only": bool(self.settings.force_dry_run),
            "commands": commands,
            "reverse_is_exact": reverse
            == [level.value for level in profile.reverse_ascent],
            "warnings": list(profile.warnings),
        }
        self.store.set_verification_stage(
            int(row), int(col), VerificationStage.DRY_RUN_PASSED
        )
        return result

    def execute_descent_step(
        self, row: int, col: int, level: DescentLevel | str
    ) -> dict[str, int]:
        target = level if isinstance(level, DescentLevel) else DescentLevel(level)
        self._require_execution_channel()
        self.thermal.require_available()
        self._ensure_profile(row, col)
        pwm = self._stored_final_pwm(row, col, target)
        if target == DescentLevel.ABOVE:
            self.state.establish_above(row, col)
            self.thermal.enter_dwell("ABOVE")
        else:
            self.state.descend(row, col, target)
            if target == DescentLevel.TOUCH:
                self.thermal.enter_dwell("TOUCH")
        self._send_pose(
            pwm,
            pump_pwm=self.settings.pump_hold_pwm,
            label=f"STAGE6_P{row}_{col}_{target.value.upper()}_HOLD",
        )
        return pwm

    def execute_reverse_ascent(self, row: int, col: int) -> list[dict[str, int]]:
        self._require_execution_channel()
        profile = self._ensure_profile(row, col)
        remaining_by_state = {
            Stage6MotionState.TARGET_TOUCH: profile.reverse_ascent,
            Stage6MotionState.RELEASE_DWELL: profile.reverse_ascent,
            Stage6MotionState.ASCENDING_75: profile.reverse_ascent[1:],
            Stage6MotionState.ASCENDING_50: profile.reverse_ascent[2:],
            Stage6MotionState.ASCENDING_25: profile.reverse_ascent[3:],
        }
        try:
            remaining = remaining_by_state[self.state.state]
        except KeyError as exc:
            raise Stage6ExecutionBlocked(
                f"reverse ascent unavailable from {self.state.state.value}"
            ) from exc
        sent: list[dict[str, int]] = []
        for level in remaining:
            sent.append(self.execute_ascent_step(row, col, level))
        self.thermal.enter_dwell("ABOVE")
        return sent

    def execute_ascent_step(
        self, row: int, col: int, level: DescentLevel | str
    ) -> dict[str, int]:
        target = level if isinstance(level, DescentLevel) else DescentLevel(level)
        self._require_execution_channel()
        self.thermal.require_available()
        self._ensure_profile(row, col)
        self.state.ascend(row, col, target)
        pwm = self._stored_final_pwm(row, col, target)
        self._send_pose(
            pwm,
            pump_pwm=self.settings.pump_off_pwm,
            label=f"STAGE6_P{row}_{col}_ASCEND_{target.value.upper()}",
        )
        if target == DescentLevel.ABOVE:
            self.thermal.enter_dwell("ABOVE")
        return pwm

    def place_piece_at(
        self,
        row: int,
        col: int,
        *,
        target_empty: bool,
        arm_holding: bool,
        board_locked: bool,
    ) -> None:
        profile = self._ensure_profile(row, col)
        stored = self.store.profile(row, col)
        reasons: list[str] = []
        if not profile.above_verified:
            reasons.append("ABOVE not verified")
        if not all(
            item.get("status") == LevelStatus.VERIFIED.value
            for item in stored["levels"].values()
        ):
            reasons.append("not all descent levels verified")
        if stored.get("verification_stage") != VerificationStage.FULLY_VERIFIED.value:
            reasons.append("profile not FULLY_VERIFIED")
        if not stored.get("reverse_ascent_verified"):
            reasons.append("reverse ascent not verified")
        if not target_empty:
            reasons.append("target is occupied")
        if not arm_holding:
            reasons.append("arm is not HOLDING")
        if self.state.state != Stage6MotionState.CARRY_HIGH:
            reasons.append("Stage6 pose is not confirmed CARRY_HIGH")
        if not board_locked:
            reasons.append("BOARD not LOCKED")
        if not self.controller.is_connected:
            reasons.append("serial disconnected")
        if self.state.snapshot().emergency_stopped:
            reasons.append("emergency stop active")
        try:
            self.thermal.require_available()
        except Exception as exc:
            reasons.append(str(exc))
        if reasons:
            raise Stage6ExecutionBlocked("; ".join(reasons))
        self.execute_descent_step(row, col, DescentLevel.ABOVE)
        for level in (
            DescentLevel.DESCENT_25,
            DescentLevel.DESCENT_50,
            DescentLevel.DESCENT_75,
            DescentLevel.TOUCH,
        ):
            self.execute_descent_step(row, col, level)
        self.state.begin_release()
        self.controller.pump_off()
        self.state.begin_release_dwell()
        time.sleep(self.settings.release_dwell_ms / 1000.0)
        self.execute_reverse_ascent(row, col)

    def dry_run_p77_regression(self) -> list[tuple[str, str]]:
        if not self.controller.dry_run:
            raise Stage6ExecutionBlocked("P77 regression requires a DRY RUN controller")
        if not self.controller.is_connected:
            self.controller.connect("STAGE6_DRY_RUN")
        start = len(self.controller.dry_run_commands)
        events: list[str] = ["CARRY_HIGH_HOLD"]
        self.controller.send_action(self.library.get("CARRY_HIGH_P77_HOLD"))
        self.state.establish_carry_high()
        self.execute_descent_step(7, 7, DescentLevel.ABOVE)
        events.append("P77_ABOVE_HOLD")
        for level in (
            DescentLevel.DESCENT_25,
            DescentLevel.DESCENT_50,
            DescentLevel.DESCENT_75,
            DescentLevel.TOUCH,
        ):
            self.execute_descent_step(7, 7, level)
            events.append(level.value.upper())
        self.state.begin_release()
        self.controller.pump_off()
        events.append("PUMP_OFF")
        self.state.begin_release_dwell()
        events.append(f"RELEASE_DWELL_{self.settings.release_dwell_ms}MS")
        self.execute_reverse_ascent(7, 7)
        events.extend(
            [
                "ASCENDING_75",
                "ASCENDING_50",
                "ASCENDING_25",
                "P77_ABOVE_IDLE",
            ]
        )
        self.state.move_to_carry_high()
        self.controller.send_action(self.library.get("CARRY_HIGH_P77_IDLE"))
        events.append("CARRY_HIGH_IDLE")
        self.state.move_to_observe()
        self.controller.send_action(self.library.get("OBSERVE_IDLE"))
        events.append("OBSERVE_IDLE")
        self.last_p77_dry_run_events = events
        self.store.set_verification_stage(7, 7, VerificationStage.DRY_RUN_PASSED)
        LOGGER.info("[STAGE6][P77_DRY_RUN] %s", " -> ".join(events))
        return list(self.controller.dry_run_commands[start:])

    def report_overheat(self) -> None:
        self.thermal.report_overheat()

    def emergency_stop(self) -> None:
        self.state.emergency_stop()
        self.controller.emergency_stop()

    def _ensure_profile(self, row: int, col: int) -> DescentProfile:
        try:
            self.store.profile(row, col)
        except Stage6CalibrationError:
            return self.generate_descent_profile(row, col)
        return self.generate_descent_profile(row, col, persist=False)

    def _stored_final_pwm(
        self, row: int, col: int, level: DescentLevel
    ) -> dict[str, int]:
        pwm = self.store.final_pwm(row, col, level)
        errors = validate_spatial_pwm(pwm, self.kinematics.limits)
        if errors:
            raise Stage6ExecutionBlocked("; ".join(errors))
        return pwm

    def _check_layer(
        self,
        previous: Mapping[str, int],
        current: Mapping[str, int],
        level: DescentLevel,
    ) -> None:
        errors = validate_spatial_pwm(current, self.kinematics.limits)
        if errors:
            raise Stage6PlanningError("; ".join(errors))
        if level != DescentLevel.ABOVE:
            for key in SPATIAL_KEYS:
                delta = abs(int(current[key]) - int(previous[key]))
                if delta > self.settings.max_layer_joint_delta_pwm:
                    raise Stage6PlanningError(
                        f"layer jump {key}={delta} exceeds "
                        f"{self.settings.max_layer_joint_delta_pwm}"
                    )

    def _send_pose(
        self, pwm: Mapping[str, int], *, pump_pwm: int, label: str
    ) -> None:
        self._require_execution_channel()
        self.thermal.record_action()
        self.controller.write(self._pose_command(pwm, pump_pwm), label=label)
        LOGGER.info("[STAGE6][SEND] %s final=%s pump=%d", label, dict(pwm), pump_pwm)

    def _pose_command(self, pwm: Mapping[str, int], pump_pwm: int) -> str:
        values = {key: int(pwm[key]) for key in SPATIAL_KEYS}
        errors = validate_spatial_pwm(values, self.kinematics.limits)
        if errors:
            raise Stage6ExecutionBlocked("; ".join(errors))
        all_values = {
            **values,
            "005": int(pump_pwm),
            "006": self.settings.unused_pwm,
            "007": self.settings.unused_pwm,
        }
        body = "".join(
            f"#{joint}P{all_values[joint]:04d}T{self.settings.move_time_ms:04d}!"
            for joint in (f"{index:03d}" for index in range(8))
        )
        return "{" + body + "}"

    def _require_execution_channel(self) -> None:
        if self.settings.force_dry_run and not self.controller.dry_run:
            raise Stage6ExecutionBlocked(
                "FORCE_DRY_RUN is enabled; real serial writes are blocked"
            )
        if not self.controller.is_connected:
            raise Stage6ExecutionBlocked("serial controller is not connected")

    def _action_spatial(self, name: str) -> dict[str, int]:
        action = self.library.get(name)
        return {
            f"{joint_id:03d}": int(action.target(joint_id).pwm)
            for joint_id in range(5)
        }

    def _log_stored_profile(self, row: int, col: int) -> None:
        profile = self.store.profile(row, col)
        computed = {
            level: item["computed_pwm"]
            for level, item in profile["levels"].items()
        }
        delta = {
            level: item["manual_delta_pwm"]
            for level, item in profile["levels"].items()
        }
        final = {
            level: item["final_pwm"]
            for level, item in profile["levels"].items()
        }
        LOGGER.info(
            "[STAGE6][PROFILE] P(%d,%d) computed=%s delta=%s final=%s",
            row,
            col,
            computed,
            delta,
            final,
        )
