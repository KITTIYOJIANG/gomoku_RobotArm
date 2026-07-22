from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum
import logging
from typing import Any, Callable

from app.arm.actions import ActionLibrary
from app.stage5.calibration_store import CalibrationStore
from app.stage5.candidate_store import CandidateStore, CandidateStoreError
from app.stage5.constants import (
    CROSS_ANCHORS,
    DEFAULT_REQUIRED_RUNS,
    FORCE_STAGE5_DRY_RUN,
    P77_KEY,
    PROTECTED_ANCHOR,
    SPATIAL_JOINTS,
    anchor_key,
    move_confirm_token,
)
from app.stage5.hover_planner import build_action_from_pwm
from app.stage5.safety import SPATIAL_JOINT_IDS, derive_pwm_safety_limits, validate_spatial_pwm
from app.learning.hover_sample_store import HoverSampleStore


LOGGER = logging.getLogger(__name__)


class CalibrationWizardState(str, Enum):
    IDLE = "IDLE"
    ANCHOR_DRAFT = "ANCHOR_DRAFT"
    ANCHOR_CANDIDATE_READY = "ANCHOR_CANDIDATE_READY"
    ANCHOR_DRY_RUN_READY = "ANCHOR_DRY_RUN_READY"
    WAITING_CARRY_CONFIRM = "WAITING_CARRY_CONFIRM"
    AT_CARRY_HIGH = "AT_CARRY_HIGH"
    WAITING_TARGET_CONFIRM = "WAITING_TARGET_CONFIRM"
    TESTING_TARGET_ABOVE = "TESTING_TARGET_ABOVE"
    AWAITING_USER_VERIFICATION = "AWAITING_USER_VERIFICATION"
    RETURNING_FROM_ANCHOR = "RETURNING_FROM_ANCHOR"
    ANCHOR_VERIFIED_ONCE = "ANCHOR_VERIFIED_ONCE"
    ANCHOR_COMPLETED = "ANCHOR_COMPLETED"
    EMERGENCY_STOP = "EMERGENCY_STOP"
    ERROR = "ERROR"


class UserTestResult(str, Enum):
    SAFE_OK = "SAFE_OK"
    SAFE_WITH_OFFSET = "SAFE_WITH_OFFSET"
    UNSAFE = "UNSAFE"
    ESTOP = "ESTOP"
    INCOMPLETE = "INCOMPLETE"


@dataclass
class ValidationIssue:
    level: str  # INFO / WARNING / BLOCKING_ERROR
    code: str
    message: str


@dataclass
class StepPlan:
    name: str
    display_name: str
    action_names: tuple[str, ...]
    serial_commands: tuple[tuple[str, str], ...]
    estimated_duration_ms: int
    pump_off: bool = True


@dataclass
class CrossAnchorWizard:
    library: ActionLibrary
    calibration: CalibrationStore
    drafts: CandidateStore
    samples: HoverSampleStore
    required_runs: int = DEFAULT_REQUIRED_RUNS
    mock_tx: list[tuple[str, str]] = field(default_factory=list)
    real_serial_write_count: int = 0
    force_dry_run: bool = FORCE_STAGE5_DRY_RUN

    def __post_init__(self) -> None:
        self.state = CalibrationWizardState.IDLE
        self.index = 0
        self.reference_pwm = self._load_p77_pwm()
        self.limits = derive_pwm_safety_limits(self.library)
        self.candidate_pwm: dict[str, int] = dict(self.reference_pwm)
        self.undo_stack: list[dict[str, int]] = []
        self.last_validation: list[ValidationIssue] = []
        self.last_plan: StepPlan | None = None
        self._safe_return_done = False
        self._estop = False
        self._load_current_draft_into_candidate()

    # ---- navigation ----
    @property
    def current(self) -> tuple[int, int, str, str]:
        return CROSS_ANCHORS[self.index]

    @property
    def current_row(self) -> int:
        return self.current[0]

    @property
    def current_col(self) -> int:
        return self.current[1]

    @property
    def progress_text(self) -> str:
        row, col, label, cn = self.current
        return f"中心十字标定：{self.index + 1} / {len(CROSS_ANCHORS)}  当前：P({row},{col}) {cn}"

    def select_index(self, index: int) -> None:
        if not 0 <= index < len(CROSS_ANCHORS):
            raise ValueError("anchor index out of range")
        self.index = int(index)
        self._load_current_draft_into_candidate()
        self.state = CalibrationWizardState.ANCHOR_DRAFT
        LOGGER.info(
            "[STAGE5][ANCHOR_SELECTED] target=P%s%s direction=%s",
            self.current_row,
            self.current_col,
            self.current[2],
        )

    def next_anchor(self) -> None:
        self.select_index(min(self.index + 1, len(CROSS_ANCHORS) - 1))

    def prev_anchor(self) -> None:
        self.select_index(max(self.index - 1, 0))

    def select_by_rc(self, row: int, col: int) -> None:
        for i, (r, c, _l, _cn) in enumerate(CROSS_ANCHORS):
            if r == row and c == col:
                self.select_index(i)
                return
        raise ValueError(f"P({row},{col}) is not a cross-anchor target")

    # ---- PWM editing ----
    def _load_p77_pwm(self) -> dict[str, int]:
        anchor = self.calibration.get_anchor(7, 7)
        if anchor is not None and anchor.calibrated:
            return {f"{k:03d}" if isinstance(k, int) else str(k).zfill(3): int(v) for k, v in anchor.pwm.items()}
        action = self.library.get("P77_ABOVE_IDLE")
        return {f"{jid:03d}": int(action.target(jid).pwm) for jid in SPATIAL_JOINT_IDS}

    def _load_current_draft_into_candidate(self) -> None:
        draft = self.drafts.get(self.current_row, self.current_col)
        pwm = draft.get("candidate_pwm") or {}
        if all(pwm.get(j) is not None for j in SPATIAL_JOINTS):
            self.candidate_pwm = {j: int(pwm[j]) for j in SPATIAL_JOINTS}
        else:
            self.candidate_pwm = dict(self.reference_pwm)
        self.undo_stack.clear()

    def set_joint(self, joint_id: str, value: int) -> None:
        jid = f"{int(joint_id):03d}"
        if jid not in SPATIAL_JOINTS:
            raise ValueError("pump and non-spatial joints cannot be edited")
        self.undo_stack.append(dict(self.candidate_pwm))
        old = self.candidate_pwm[jid]
        self.candidate_pwm[jid] = int(value)
        self.state = CalibrationWizardState.ANCHOR_DRAFT
        LOGGER.info("[STAGE5][CANDIDATE_EDIT] joint=%s old=%s new=%s delta=%s", jid, old, value, int(value) - old)

    def nudge(self, joint_id: str, delta: int) -> None:
        jid = f"{int(joint_id):03d}"
        self.set_joint(jid, self.candidate_pwm[jid] + int(delta))

    def undo(self) -> bool:
        if not self.undo_stack:
            return False
        self.candidate_pwm = self.undo_stack.pop()
        return True

    def reset_to_p77(self) -> None:
        self.undo_stack.append(dict(self.candidate_pwm))
        self.candidate_pwm = dict(self.reference_pwm)

    def deltas(self) -> dict[str, int]:
        return {j: self.candidate_pwm[j] - self.reference_pwm[j] for j in SPATIAL_JOINTS}

    # ---- validation ----
    def validate_candidate(self) -> list[ValidationIssue]:
        issues: list[ValidationIssue] = []
        row, col = self.current_row, self.current_col
        if (row, col) == PROTECTED_ANCHOR:
            issues.append(ValidationIssue("BLOCKING_ERROR", "PROTECTED", "P(7,7) is protected and cannot be recalibrated here"))
        for jid in SPATIAL_JOINTS:
            if jid not in self.candidate_pwm or self.candidate_pwm[jid] is None:
                issues.append(ValidationIssue("BLOCKING_ERROR", "NULL_PWM", f"joint {jid} is empty/null"))
        if any(i.code == "NULL_PWM" for i in issues):
            self.last_validation = issues
            return issues
        # envelope from stable actions
        errors = validate_spatial_pwm({int(j): int(self.candidate_pwm[j]) for j in SPATIAL_JOINTS}, self.limits)
        for err in errors:
            issues.append(ValidationIssue("BLOCKING_ERROR", "ENVELOPE", err))
        # delta vs P77
        for jid, delta in self.deltas().items():
            adj = self.limits.max_adjacent_delta[int(jid)]
            # allow larger than one cell because anchors are 4 cells away, use *4 margin
            budget = max(40, adj * 5)
            if abs(delta) > budget:
                issues.append(
                    ValidationIssue(
                        "BLOCKING_ERROR",
                        "DELTA_TOO_LARGE",
                        f"joint {jid} delta {delta} exceeds conservative budget ±{budget}",
                    )
                )
            elif abs(delta) > budget * 0.7:
                issues.append(
                    ValidationIssue("WARNING", "DELTA_LARGE", f"joint {jid} delta {delta} is large")
                )
        # direction trend (info only)
        if row < 7:
            issues.append(ValidationIssue("INFO", "TREND", "P(3,7) is center-up (row-); do not auto-guess PWM sign"))
        elif row > 7:
            issues.append(ValidationIssue("INFO", "TREND", "P(11,7) is center-down (row+); do not auto-guess PWM sign"))
        elif col < 7:
            issues.append(ValidationIssue("INFO", "TREND", "P(7,3) is center-left (col-); do not auto-guess PWM sign"))
        elif col > 7:
            issues.append(ValidationIssue("INFO", "TREND", "P(7,11) is center-right (col+); do not auto-guess PWM sign"))
        # pump must not be in candidate
        issues.append(ValidationIssue("INFO", "PUMP", "Pump channel 005 is forced OFF (1500) and not editable"))
        self.last_validation = issues
        blocking = [i for i in issues if i.level == "BLOCKING_ERROR"]
        LOGGER.info(
            "[STAGE5][CANDIDATE_VALIDATION] result=%s warnings=%s",
            "FAIL" if blocking else "PASS",
            len([i for i in issues if i.level == "WARNING"]),
        )
        if not blocking:
            self.state = CalibrationWizardState.ANCHOR_CANDIDATE_READY
        return issues

    def has_blocking_errors(self) -> bool:
        return any(i.level == "BLOCKING_ERROR" for i in self.validate_candidate())

    # ---- plans ----
    def _register_candidate_action(self) -> str:
        name = f"TARGET_ABOVE_CANDIDATE_P{self.current_row}{self.current_col}"
        pwm_map = {int(j): int(self.candidate_pwm[j]) for j in SPATIAL_JOINTS}
        pwm_map[5] = 1500  # pump OFF
        pwm_map[6] = 1500
        pwm_map[7] = 1500
        action = build_action_from_pwm(name, pwm_map, time_ms=1000)
        self.library.register_runtime(action)
        return name

    def plan_carry_high_test(self) -> StepPlan:
        if self.has_blocking_errors():
            raise RuntimeError("candidate has blocking validation errors")
        carry = self.library.get("CARRY_HIGH_P77_IDLE")
        observe = self.library.get("OBSERVE_IDLE")
        plan = StepPlan(
            name="CROSS_CARRY_HIGH",
            display_name="测试运输高位",
            action_names=("OBSERVE_IDLE", "CARRY_HIGH_P77_IDLE"),
            serial_commands=(("OBSERVE_IDLE", observe.command), ("CARRY_HIGH_P77_IDLE", carry.command)),
            estimated_duration_ms=observe.duration_ms + carry.duration_ms + 400,
            pump_off=True,
        )
        self._assert_safe_plan(plan)
        self.last_plan = plan
        self.state = CalibrationWizardState.ANCHOR_DRY_RUN_READY
        LOGGER.info("[STAGE5][DRY_RUN_PLAN] target=P%s%s steps=OBSERVE,CARRY_HIGH", self.current_row, self.current_col)
        return plan

    def plan_target_above_test(self) -> StepPlan:
        if self.has_blocking_errors():
            raise RuntimeError("candidate has blocking validation errors")
        name = self._register_candidate_action()
        carry = self.library.get("CARRY_HIGH_P77_IDLE")
        target = self.library.get(name)
        plan = StepPlan(
            name="CROSS_TARGET_ABOVE",
            display_name="测试目标上方",
            action_names=("CARRY_HIGH_P77_IDLE", name),
            serial_commands=(("CARRY_HIGH_P77_IDLE", carry.command), (name, target.command)),
            estimated_duration_ms=carry.duration_ms + target.duration_ms + 400,
            pump_off=True,
        )
        self._assert_safe_plan(plan)
        self.last_plan = plan
        self.state = CalibrationWizardState.ANCHOR_DRY_RUN_READY
        LOGGER.info("[STAGE5][DRY_RUN_PLAN] target=P%s%s steps=CARRY_HIGH,TARGET_ABOVE", self.current_row, self.current_col)
        return plan

    def plan_safe_return(self) -> StepPlan:
        name = self._register_candidate_action()
        carry = self.library.get("CARRY_HIGH_P77_IDLE")
        observe = self.library.get("OBSERVE_IDLE")
        # return path may start from target or carry; always go carry then observe
        plan = StepPlan(
            name="CROSS_SAFE_RETURN",
            display_name="安全返回",
            action_names=("CARRY_HIGH_P77_IDLE", "OBSERVE_IDLE"),
            serial_commands=(("CARRY_HIGH_P77_IDLE", carry.command), ("OBSERVE_IDLE", observe.command)),
            estimated_duration_ms=carry.duration_ms + observe.duration_ms + 400,
            pump_off=True,
        )
        self._assert_safe_plan(plan)
        self.last_plan = plan
        return plan

    def _assert_safe_plan(self, plan: StepPlan) -> None:
        names = plan.action_names
        if any("TOUCH" in n or "RELEASE" in n for n in names):
            raise RuntimeError("plan must not include TOUCH/RELEASE")
        if plan.name == "CROSS_TARGET_ABOVE" and not any(n.startswith("CARRY_HIGH") for n in names):
            raise RuntimeError("target test must pass through CARRY_HIGH")
        for _label, command in plan.serial_commands:
            if "#005P2500" in command:
                raise RuntimeError("plan must keep pump OFF")

    def execute_plan_mock(self, plan: StepPlan, *, gui_dry_run_checked: bool = True) -> list[tuple[str, str]]:
        """Always mock when FORCE_STAGE5_DRY_RUN is True. Never increments real write count."""
        if self._estop:
            raise RuntimeError("estop latched")
        effective_dry = True if self.force_dry_run else bool(gui_dry_run_checked)
        if not effective_dry:
            # Live path is intentionally blocked this sprint.
            raise RuntimeError("FORCE_STAGE5_DRY_RUN blocks live serial writes")
        sent: list[tuple[str, str]] = []
        for label, command in plan.serial_commands:
            self.mock_tx.append((label, command))
            sent.append((label, command))
            LOGGER.info("[STAGE5][MOCK_TX] %s %s", label, command)
        # real_serial_write_count stays 0
        if plan.name == "CROSS_CARRY_HIGH":
            self.state = CalibrationWizardState.AT_CARRY_HIGH
        elif plan.name == "CROSS_TARGET_ABOVE":
            self.state = CalibrationWizardState.AWAITING_USER_VERIFICATION
        elif plan.name == "CROSS_SAFE_RETURN":
            self.state = CalibrationWizardState.RETURNING_FROM_ANCHOR
            self._safe_return_done = True
        return sent

    def live_confirm_token(self) -> str:
        return move_confirm_token(self.current_row, self.current_col)

    def validate_live_confirm(self, text: str) -> bool:
        return text.strip().upper() == self.live_confirm_token()

    # ---- draft / confirm ----
    def save_draft(self, notes: str = "") -> dict[str, Any]:
        issues = self.validate_candidate()
        if any(i.level == "BLOCKING_ERROR" and i.code == "PROTECTED" for i in issues):
            raise RuntimeError("cannot save draft for protected P77")
        entry = self.drafts.set_candidate_pwm(
            self.current_row,
            self.current_col,
            self.candidate_pwm,
            status="DRAFT",
            notes=notes,
        )
        self.state = CalibrationWizardState.ANCHOR_DRAFT
        return entry

    def load_draft(self) -> dict[str, Any]:
        entry = self.drafts.get(self.current_row, self.current_col)
        self._load_current_draft_into_candidate()
        return entry

    def record_user_result(self, result: UserTestResult) -> dict[str, Any]:
        if self._estop or result == UserTestResult.ESTOP:
            self._estop = True
            self.state = CalibrationWizardState.EMERGENCY_STOP
            entry = self.drafts.record_test_result(
                self.current_row,
                self.current_col,
                result=UserTestResult.ESTOP.value,
                safe_return_completed=False,
                emergency_stop=True,
                increment_verified=False,
            )
            LOGGER.info("[STAGE5][USER_RESULT] target=P%s%s result=ESTOP", self.current_row, self.current_col)
            return entry

        increment = (
            result == UserTestResult.SAFE_OK
            and self._safe_return_done
            and not self._estop
        )
        entry = self.drafts.record_test_result(
            self.current_row,
            self.current_col,
            result=result.value,
            safe_return_completed=self._safe_return_done,
            emergency_stop=False,
            increment_verified=increment,
        )
        LOGGER.info(
            "[STAGE5][USER_RESULT] target=P%s%s result=%s verified_runs=%s",
            self.current_row,
            self.current_col,
            result.value,
            entry.get("verified_runs"),
        )
        if increment:
            LOGGER.info(
                "[STAGE5][VERIFIED_RUN] target=P%s%s runs=%s required=%s",
                self.current_row,
                self.current_col,
                entry.get("verified_runs"),
                self.required_runs,
            )
            self.state = CalibrationWizardState.ANCHOR_VERIFIED_ONCE
        return entry

    def mark_safe_return_completed(self) -> None:
        self._safe_return_done = True

    def reset_test_session_flags(self) -> None:
        self._safe_return_done = False
        self._estop = False

    def complete_anchor(self, *, write_calibration: bool = True) -> dict[str, Any]:
        entry = self.drafts.get(self.current_row, self.current_col)
        runs = int(entry.get("verified_runs", 0))
        if runs < self.required_runs:
            raise RuntimeError(f"verified_runs {runs} < required {self.required_runs}")
        if not entry.get("safe_return_completed"):
            # last successful cycle must have returned
            raise RuntimeError("safe return not completed")
        if entry.get("emergency_stop"):
            raise RuntimeError("cannot complete after estop without clean verification")
        raw_pwm = entry.get("candidate_pwm") or {}
        if not all(raw_pwm.get(j) is not None for j in SPATIAL_JOINTS):
            # Prefer in-memory candidate if draft never persisted.
            if all(self.candidate_pwm.get(j) is not None for j in SPATIAL_JOINTS):
                self.save_draft(notes="auto-save before complete")
                entry = self.drafts.get(self.current_row, self.current_col)
                raw_pwm = entry.get("candidate_pwm") or {}
            else:
                raise RuntimeError("candidate_pwm incomplete; save draft first")
        pwm = {j: int(raw_pwm[j]) for j in SPATIAL_JOINTS}
        # re-validate
        self.candidate_pwm = dict(pwm)
        if self.has_blocking_errors():
            raise RuntimeError("candidate no longer passes validation")

        completed = self.drafts.mark_completed(self.current_row, self.current_col)
        if write_calibration:
            # Never overwrite P77
            if (self.current_row, self.current_col) == PROTECTED_ANCHOR:
                raise RuntimeError("refusing to overwrite protected P77")
            self.calibration.upsert_anchor(
                self.current_row,
                self.current_col,
                pwm,
                time_ms=1000,
                notes="cross-anchor wizard manual calibration",
                calibrated=True,
                verified_runs=runs,
                require_anchor_set=True,
            )
            self.calibration.save()

        version = datetime.now().strftime("%Y%m%d%H%M%S")
        sample = self.samples.add_sample(
            row=self.current_row,
            col=self.current_col,
            pwm=pwm,
            verified_runs=runs,
            safe_return_completed=True,
            emergency_stop=False,
            calibration_version=version,
        )
        self.state = CalibrationWizardState.ANCHOR_COMPLETED
        LOGGER.info(
            "[STAGE5][ANCHOR_COMPLETED] target=P%s%s calibrated=1 sample=%s",
            self.current_row,
            self.current_col,
            None if sample is None else sample.get("sample_id"),
        )
        return {"anchor": completed, "sample": sample}

    def status_snapshot(self) -> dict[str, Any]:
        statuses = self.drafts.list_status()
        row, col, label, cn = self.current
        draft = self.drafts.get(row, col)
        return {
            "progress": self.progress_text,
            "index": self.index,
            "row": row,
            "col": col,
            "label": label,
            "direction_cn": cn,
            "state": self.state.value,
            "reference_pwm": dict(self.reference_pwm),
            "candidate_pwm": dict(self.candidate_pwm),
            "deltas": self.deltas(),
            "verified_runs": int(draft.get("verified_runs", 0)),
            "required_runs": self.required_runs,
            "status_map": statuses,
            "force_dry_run": self.force_dry_run,
            "real_serial_write_count": self.real_serial_write_count,
            "mock_tx_count": len(self.mock_tx),
            "live_confirm_token": self.live_confirm_token(),
        }
