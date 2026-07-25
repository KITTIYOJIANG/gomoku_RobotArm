from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from app.arm.actions import Action, ActionLibrary, ServoTarget
from app.arm.sequences import ActionStep, SequenceDefinition, validate_safe_sequence
from app.stage5.calibration_store import CalibrationStore
from app.stage5.pwm_interpolator import InterpolationError, InterpolationResult, interpolate_target_pwm, resolve_target_pwm
from app.stage5.safety import PUMP_JOINT_ID, SPATIAL_JOINT_IDS, PwmSafetyLimits


RUNTIME_TARGET_IDLE = "TARGET_ABOVE_IDLE"
RUNTIME_TARGET_HOLD = "TARGET_ABOVE_HOLD"
RUNTIME_CARRY_LIFTED_IDLE = "CARRY_HIGH_LIFTED_IDLE"
RUNTIME_CARRY_LIFTED_HOLD = "CARRY_HIGH_LIFTED_HOLD"
DEFAULT_CARRY_LIFT_001 = 60


@dataclass(frozen=True)
class HoverPlan:
    target_row: int
    target_col: int
    source: str
    holding_piece: bool
    dry_run: bool
    interpolation: InterpolationResult
    sequence: SequenceDefinition
    serial_commands: tuple[tuple[str, str], ...]
    estimated_duration_ms: int
    safety_checks: tuple[str, ...]
    target_action: Action

    def to_dict(self) -> dict[str, Any]:
        return {
            "target": {"row": self.target_row, "col": self.target_col},
            "source": self.source,
            "holding_piece": self.holding_piece,
            "dry_run": self.dry_run,
            "steps": [
                {"name": step.action_name} if isinstance(step, ActionStep) else {"wait": step.label}
                for step in self.sequence.steps
            ],
            "estimated_duration_ms": self.estimated_duration_ms,
            "safety_checks": list(self.safety_checks),
            "serial_commands": [{"label": label, "command": command} for label, command in self.serial_commands],
            "interpolation": self.interpolation.to_dict(),
        }


@dataclass
class HoverPlanner:
    library: ActionLibrary
    store: CalibrationStore
    limits: PwmSafetyLimits
    action_wait_margin_ms: int = 200
    carry_lift_001: int = DEFAULT_CARRY_LIFT_001

    def plan_hover_to(
        self,
        row: int,
        col: int,
        *,
        holding_piece: bool,
        dry_run: bool,
    ) -> HoverPlan:
        # Priority: calibrated taught > default bilinear > star seed. No tight-limit reject
        # on default points — user only fine-tunes where needed.
        interpolation = resolve_target_pwm(
            self.store, row, col, limits=None, allow_star_seed=True
        )
        target_action = self.build_target_action(interpolation, holding_piece=holding_piece)
        target_name = RUNTIME_TARGET_HOLD if holding_piece else RUNTIME_TARGET_IDLE
        # Register runtime target so the existing worker/action path can send it.
        self.library.register_runtime(target_action)
        ref_001 = int(interpolation.pwm.get(1, 0)) or None
        carry_action = build_lifted_carry_action(
            self.library,
            holding_piece=holding_piece,
            reference_001=ref_001,
            lift_001=self.carry_lift_001,
        )
        carry_name = carry_action.name

        sequence = SequenceDefinition(
            name="HOVER_TO_TARGET",
            display_name=f"悬停到 P({row},{col})",
            requires_board=True,
            steps=(
                ActionStep(carry_name),
                ActionStep(target_name),
            ),
        )
        validate_safe_sequence(sequence)
        safety_checks = [
            "path_via_carry_high_lifted",
            "no_target_touch",
            "no_release",
            "spatial_joints_only_interpolated",
            f"source={interpolation.source}",
            f"carry_001={carry_action.target(1).pwm}",
        ]
        serial_commands = (
            (carry_name, carry_action.command),
            (target_name, target_action.command),
        )
        duration = sum(self.library.get(step.action_name).duration_ms for step in sequence.steps if isinstance(step, ActionStep))
        duration += self.action_wait_margin_ms * len(sequence.action_names)
        return HoverPlan(
            target_row=int(row),
            target_col=int(col),
            source=interpolation.source,
            holding_piece=bool(holding_piece),
            dry_run=bool(dry_run),
            interpolation=interpolation,
            sequence=sequence,
            serial_commands=serial_commands,
            estimated_duration_ms=int(duration),
            safety_checks=tuple(safety_checks),
            target_action=target_action,
        )

    def plan_hover_with_pwm(
        self,
        row: int,
        col: int,
        pwm_by_str: dict[str, int],
        *,
        holding_piece: bool,
        dry_run: bool,
        source: str = "user_edited",
    ) -> HoverPlan:
        """Hover using explicit PWM (fine-tune path); does not re-interpolate."""
        spatial = {int(k): int(v) for k, v in pwm_by_str.items() if int(k) in SPATIAL_JOINT_IDS}
        for jid in SPATIAL_JOINT_IDS:
            if jid not in spatial:
                raise ValueError(f"missing joint {jid:03d}")
            if not (500 <= spatial[jid] <= 2500):
                raise ValueError(f"joint {jid:03d} PWM {spatial[jid]} out of 500..2500")
        interpolation = InterpolationResult(
            row=int(row),
            col=int(col),
            source=source,
            pwm=spatial,
            time_ms=1000,
            anchors_used=(),
            u=None,
            v=None,
            details={"manual_edit": True},
        )
        target_action = self.build_target_action(interpolation, holding_piece=holding_piece)
        target_name = RUNTIME_TARGET_HOLD if holding_piece else RUNTIME_TARGET_IDLE
        self.library.register_runtime(target_action)
        carry_action = build_lifted_carry_action(
            self.library,
            holding_piece=holding_piece,
            reference_001=int(spatial[1]),
            lift_001=self.carry_lift_001,
        )
        carry_name = carry_action.name
        sequence = SequenceDefinition(
            name="HOVER_TO_TARGET",
            display_name=f"悬停到 P({row},{col}) [微调PWM]",
            requires_board=True,
            steps=(ActionStep(carry_name), ActionStep(target_name)),
        )
        validate_safe_sequence(sequence)
        serial_commands = (
            (carry_name, carry_action.command),
            (target_name, target_action.command),
        )
        duration = sum(
            self.library.get(step.action_name).duration_ms
            for step in sequence.steps
            if isinstance(step, ActionStep)
        )
        duration += self.action_wait_margin_ms * len(sequence.action_names)
        return HoverPlan(
            target_row=int(row),
            target_col=int(col),
            source=source,
            holding_piece=bool(holding_piece),
            dry_run=bool(dry_run),
            interpolation=interpolation,
            sequence=sequence,
            serial_commands=serial_commands,
            estimated_duration_ms=int(duration),
            safety_checks=("path_via_carry_high", "no_target_touch", "user_pwm_override"),
            target_action=target_action,
        )

    def plan_return_to_observe(
        self,
        *,
        holding_piece: bool,
        dry_run: bool,
        reference_001: int | None = None,
    ) -> SequenceDefinition:
        carry_action = build_lifted_carry_action(
            self.library,
            holding_piece=holding_piece,
            reference_001=reference_001,
            lift_001=self.carry_lift_001,
        )
        carry_name = carry_action.name
        observe_name = "OBSERVE_HOLD" if holding_piece else "OBSERVE_IDLE"
        sequence = SequenceDefinition(
            name="SAFE_RETURN_FROM_HOVER",
            display_name="安全返回观察位(抬高运输)",
            steps=(
                ActionStep(carry_name),
                ActionStep(observe_name),
            ),
        )
        validate_safe_sequence(sequence)
        return sequence

    def build_target_action(self, interpolation: InterpolationResult, *, holding_piece: bool) -> Action:
        name = RUNTIME_TARGET_HOLD if holding_piece else RUNTIME_TARGET_IDLE
        pump_pwm = 2500 if holding_piece else 1500
        # Preserve unused channels 006/007 as neutral 1500 from stable poses.
        pwm_map = {jid: int(interpolation.pwm[jid]) for jid in SPATIAL_JOINT_IDS}
        pwm_map[PUMP_JOINT_ID] = pump_pwm
        pwm_map[6] = 1500
        pwm_map[7] = 1500
        return build_action_from_pwm(name, pwm_map, time_ms=int(interpolation.time_ms))


def build_action_from_pwm(name: str, pwm_by_id: dict[int, int], *, time_ms: int = 1000) -> Action:
    if sorted(pwm_by_id) != list(range(8)):
        # Allow partial spatial maps by filling neutrals.
        filled = {jid: 1500 for jid in range(8)}
        filled.update({int(k): int(v) for k, v in pwm_by_id.items()})
        pwm_by_id = filled
    time_token = max(100, min(9999, int(time_ms)))
    parts = []
    targets = []
    for jid in range(8):
        pwm = int(pwm_by_id[jid])
        if not 500 <= pwm <= 2500:
            raise ValueError(f"PWM out of protocol range for joint {jid:03d}: {pwm}")
        parts.append(f"#{jid:03d}P{pwm:04d}T{time_token:04d}!")
        targets.append(ServoTarget(servo_id=jid, pwm=pwm, time_ms=time_token))
    command = "{" + "".join(parts) + "}"
    return Action(name=name.upper(), command=command, targets=tuple(targets))


def build_lifted_carry_action(
    library: ActionLibrary,
    *,
    holding_piece: bool,
    reference_001: int | None = None,
    lift_001: int = DEFAULT_CARRY_LIFT_001,
    time_ms: int = 1000,
) -> Action:
    """Transit pose based on CARRY_HIGH_P77 but with a higher joint-001.

    Stock return path is TARGET -> CARRY_HIGH_P77 -> OBSERVE. If CARRY_HIGH 001 is
    lower than the hover pose, the elbow/shoulder can dip and scrape the board.
    Users often raise taught 001 as insurance; this bakes a safer transit into
    software without requiring every target to be taught high.
    """
    base_name = "CARRY_HIGH_P77_HOLD" if holding_piece else "CARRY_HIGH_P77_IDLE"
    base = library.get(base_name)
    pwm_map = {jid: 1500 for jid in range(8)}
    for target in base.targets:
        pwm_map[int(target.servo_id)] = int(target.pwm)
    base_001 = int(pwm_map.get(1, 1180))
    ref = int(reference_001) if reference_001 is not None else base_001
    lifted = max(base_001 + int(lift_001), ref + 20, base_001)
    pwm_map[1] = max(500, min(2500, lifted))
    name = RUNTIME_CARRY_LIFTED_HOLD if holding_piece else RUNTIME_CARRY_LIFTED_IDLE
    action = build_action_from_pwm(name, pwm_map, time_ms=time_ms)
    library.register_runtime(action)
    return action
