from __future__ import annotations

from dataclasses import dataclass
import logging
from typing import Any

import numpy as np

from app.arm.actions import ActionLibrary
from app.arm.controller import SerialArmController
from app.arm.sequences import SequenceDefinition
from app.arm.state import ArmState, ArmStateMachine
from app.arm.worker import ArmSequenceWorker
from app.config import Stage5Config
from app.stage5.board_intersections import ClickSelection, build_intersection_grid, select_intersection
from app.stage5.calibration_store import CalibrationStore
from app.stage5.hover_planner import HoverPlan, HoverPlanner
from app.stage5.logger import Stage5Logger
from app.stage5.pwm_interpolator import InterpolationError, interpolate_target_pwm
from app.stage5.safety import derive_pwm_safety_limits
from app.stage5.state_machine import Stage5State, Stage5StateMachine


LOGGER = logging.getLogger(__name__)


@dataclass
class TargetView:
    row: int | None = None
    col: int | None = None
    pixel_x: float | None = None
    pixel_y: float | None = None
    calibrated_text: str = "-"
    region_text: str = "-"
    source: str = "-"
    pwm_text: str = "-"
    verified_runs: int = 0
    in_region: bool = False
    calibrated: bool = False
    pwm: dict[str, int] | None = None


class Stage5Coordinator:
    """Pure orchestration for stage-5 selection, planning, and dry-run logging."""

    def __init__(
        self,
        *,
        config: Stage5Config,
        actions: ActionLibrary,
        controller: SerialArmController,
        arm_state: ArmStateMachine,
        worker: ArmSequenceWorker,
        action_wait_margin_ms: int,
        logs_dir,
    ) -> None:
        self.config = config
        self.actions = actions
        self.controller = controller
        self.arm_state = arm_state
        self.worker = worker
        self.limits = derive_pwm_safety_limits(actions, board_span_cells=config.board_span_cells)
        self.store = CalibrationStore(config.calibration_path, library=actions, safety_limits=self.limits)
        self.planner = HoverPlanner(
            library=actions,
            store=self.store,
            limits=self.limits,
            action_wait_margin_ms=action_wait_margin_ms,
        )
        self.stage_state = Stage5StateMachine(dry_run=config.default_dry_run)
        self.logger = Stage5Logger(logs_dir / "stage5" if hasattr(logs_dir, "__truediv__") else logs_dir)
        self.last_homography: np.ndarray | None = None
        self.last_corners: np.ndarray | None = None
        self.board_locked = False
        self.target = TargetView()
        self.last_plan: HoverPlan | None = None
        self._holding_piece = False
        self._active_sequence: str | None = None

    def set_dry_run(self, enabled: bool) -> None:
        self.stage_state.set_dry_run(enabled)
        self.logger.log("DRY_RUN_SET", dry_run=bool(enabled))

    def on_serial_connected(self) -> None:
        self.stage_state.on_serial_connected(board_locked=self.board_locked)
        self.logger.log("SERIAL_CONNECTED", port=self.controller.port, board_locked=self.board_locked)

    def on_serial_disconnected(self) -> None:
        self.stage_state.on_serial_disconnected()
        self.logger.log("SERIAL_DISCONNECTED")

    def on_board_lock_changed(self, locked: bool) -> None:
        self.board_locked = bool(locked)
        self.stage_state.on_board_lock_changed(bool(locked))

    def update_geometry(self, payload: dict[str, Any]) -> None:
        if self.stage_state.is_moving() or self.stage_state.state == Stage5State.HOVERING:
            # Freeze geometry used for the selected target during motion/hover.
            return
        homography = payload.get("homography")
        corners = payload.get("corners")
        if homography is not None:
            self.last_homography = np.asarray(homography, dtype=np.float32)
        if corners is not None:
            self.last_corners = np.asarray(corners, dtype=np.float32)
        locked = bool(payload.get("board_locked", False))
        if locked != self.board_locked:
            self.on_board_lock_changed(locked)

    def handle_click(self, image_x: float, image_y: float, board_size: int) -> ClickSelection:
        if not self.board_locked or self.last_homography is None:
            selection = ClickSelection(False, "BOARD_NOT_LOCKED")
            self.logger.log("CLICK_REJECTED", reason=selection.reason, x=image_x, y=image_y)
            return selection
        if self.stage_state.is_moving():
            selection = ClickSelection(False, "ARM_BUSY")
            self.logger.log("CLICK_REJECTED", reason=selection.reason)
            return selection
        grid = build_intersection_grid(self.last_homography, board_size)
        selection = select_intersection(
            grid,
            (image_x, image_y),
            threshold_ratio=self.config.click_threshold_ratio,
            board_size=board_size,
        )
        self.logger.log(
            "CLICK",
            **selection.to_dict(),
            board_locked=self.board_locked,
            port=self.controller.port,
        )
        if not selection.accepted or selection.row is None or selection.col is None:
            return selection
        self._apply_target(selection.row, selection.col, selection.pixel_x, selection.pixel_y)
        return selection

    def select_target_programmatically(self, row: int, col: int, board_size: int) -> None:
        pixel_x = pixel_y = None
        if self.last_homography is not None:
            grid = build_intersection_grid(self.last_homography, board_size)
            pixel_x = float(grid[row, col, 0])
            pixel_y = float(grid[row, col, 1])
        self._apply_target(row, col, pixel_x, pixel_y)

    def clear_target(self) -> None:
        self.stage_state.clear_target()
        self.target = TargetView()
        self.last_plan = None
        self.logger.log("TARGET_CLEARED")

    def _apply_target(
        self,
        row: int,
        col: int,
        pixel_x: float | None,
        pixel_y: float | None,
    ) -> None:
        region = self.store.allowed_region
        in_region = (
            region["row_min"] <= row <= region["row_max"]
            and region["col_min"] <= col <= region["col_max"]
        )
        calibrated = False
        source = "-"
        pwm_text = "-"
        pwm: dict[str, int] | None = None
        verified = 0
        try:
            result = interpolate_target_pwm(self.store, row, col, limits=self.limits)
            calibrated = True
            source = result.source
            pwm = result.pwm_str_keys()
            pwm_text = ", ".join(f"{k}:{v}" for k, v in pwm.items())
        except InterpolationError as exc:
            source = exc.code
            pwm_text = str(exc)
            calibrated = False
        anchor = self.store.get_anchor(row, col)
        if anchor is not None:
            verified = anchor.verified_runs
            if not calibrated and anchor.calibrated:
                calibrated = True
        self.stage_state.select_target(
            row,
            col,
            board_locked=self.board_locked,
            calibrated=calibrated and in_region,
            in_region=in_region,
        )
        self.target = TargetView(
            row=row,
            col=col,
            pixel_x=pixel_x,
            pixel_y=pixel_y,
            calibrated_text="YES" if calibrated else "NO",
            region_text="INSIDE" if in_region else "OUTSIDE",
            source=source,
            pwm_text=pwm_text,
            verified_runs=verified,
            in_region=in_region,
            calibrated=calibrated,
            pwm=pwm,
        )
        self.logger.log(
            "TARGET_SELECTED",
            row=row,
            col=col,
            pixel_x=pixel_x,
            pixel_y=pixel_y,
            calibrated=calibrated,
            in_region=in_region,
            source=source,
            pwm=pwm,
        )

    def plan_hover(self, *, holding_piece: bool) -> HoverPlan:
        if self.target.row is None or self.target.col is None:
            raise RuntimeError("No target selected")
        if not self.store.valid:
            raise RuntimeError(self.store.load_error or "Calibration invalid")
        if not self.board_locked:
            raise RuntimeError("BOARD LOCKED required")
        arm = self.arm_state.snapshot()
        if arm.busy or self.worker.busy:
            raise RuntimeError("Arm is busy")
        if arm.state not in {ArmState.OBSERVE_IDLE, ArmState.OBSERVE_HOLD}:
            raise RuntimeError(f"Arm state {arm.state.value} cannot start hover")
        dry_run = self.stage_state.snapshot().dry_run or self.controller.dry_run
        plan = self.planner.plan_hover_to(
            self.target.row,
            self.target.col,
            holding_piece=holding_piece,
            dry_run=dry_run,
        )
        self.last_plan = plan
        self.logger.log("HOVER_PLAN", **plan.to_dict())
        return plan

    def begin_hover_execution(self, plan: HoverPlan) -> tuple[bool, str]:
        """Return (submitted_to_worker, message). Dry-run may not submit real TX."""
        self._holding_piece = plan.holding_piece
        self.stage_state.begin_hover(holding_piece=plan.holding_piece)
        self.arm_state.begin_hover(board_locked=self.board_locked)
        self._active_sequence = plan.sequence.name
        if plan.dry_run:
            for label, command in plan.serial_commands:
                self.logger.log("DRY_RUN_TX", label=label, command=command)
                LOGGER.info("STAGE5 DRY-RUN TX %s %s", label, command)
            self.logger.log(
                "COMMAND_SENT",
                mode="DRY_RUN",
                commands=list(plan.serial_commands),
                estimated_duration_ms=plan.estimated_duration_ms,
            )
            return False, "dry_run"
        if not self.controller.is_connected:
            self.stage_state.fail("Serial not connected")
            self.arm_state.fail("Serial not connected")
            raise RuntimeError("Serial not connected")
        ok = self.worker.submit(plan.sequence)
        if not ok:
            self.stage_state.fail("Worker busy")
            self.arm_state.fail("Worker busy")
            raise RuntimeError("Arm worker rejected sequence")
        self.logger.log(
            "COMMAND_SENT",
            mode="LIVE",
            commands=list(plan.serial_commands),
            estimated_duration_ms=plan.estimated_duration_ms,
        )
        return True, "submitted"

    def complete_hover_dry_run(self) -> None:
        self.stage_state.complete_hover()
        self.arm_state.complete_hover()
        self._active_sequence = None
        self.logger.log("ESTIMATED_MOTION_COMPLETE", mode="DRY_RUN", phase="HOVER")

    def begin_safe_return(self) -> tuple[SequenceDefinition, bool]:
        if not self.stage_state.can_safe_return():
            raise RuntimeError("Safe return only from HOVERING")
        holding = self._holding_piece
        dry_run = self.stage_state.snapshot().dry_run or self.controller.dry_run
        sequence = self.planner.plan_return_to_observe(holding_piece=holding, dry_run=dry_run)
        self.stage_state.begin_return()
        self.arm_state.begin_return_from_hover()
        self._active_sequence = sequence.name
        commands = [(step.action_name, self.actions.get(step.action_name).command) for step in sequence.steps]
        if dry_run:
            for label, command in commands:
                self.logger.log("DRY_RUN_TX", label=label, command=command)
            self.logger.log("COMMAND_SENT", mode="DRY_RUN", phase="SAFE_RETURN", commands=commands)
            return sequence, False
        ok = self.worker.submit(sequence)
        if not ok:
            self.stage_state.fail("Worker busy")
            self.arm_state.fail("Worker busy")
            raise RuntimeError("Arm worker rejected return sequence")
        self.logger.log("COMMAND_SENT", mode="LIVE", phase="SAFE_RETURN", commands=commands)
        return sequence, True

    def complete_return_dry_run(self) -> None:
        self.stage_state.complete_return(board_locked=self.board_locked)
        self.arm_state.complete_return_from_hover(holding_piece=self._holding_piece)
        self._active_sequence = None
        self.logger.log("ESTIMATED_MOTION_COMPLETE", mode="DRY_RUN", phase="SAFE_RETURN")

    def on_sequence_finished(self, name: str, success: bool, message: str) -> bool:
        """Handle worker completion for stage5 sequences. Returns True if consumed."""
        if name not in {"HOVER_TO_TARGET", "SAFE_RETURN_FROM_HOVER"}:
            return False
        if success:
            if name == "HOVER_TO_TARGET":
                self.stage_state.complete_hover()
                self.arm_state.complete_hover()
                self.logger.log("ESTIMATED_MOTION_COMPLETE", mode="LIVE", phase="HOVER")
            else:
                self.stage_state.complete_return(board_locked=self.board_locked)
                self.arm_state.complete_return_from_hover(holding_piece=self._holding_piece)
                self.logger.log("ESTIMATED_MOTION_COMPLETE", mode="LIVE", phase="SAFE_RETURN")
        else:
            self.stage_state.fail(message or "sequence failed")
            self.arm_state.fail(message or "sequence failed")
            self.logger.log("SEQUENCE_FAILED", name=name, message=message)
        self._active_sequence = None
        return True

    def estop(self) -> None:
        self.stage_state.estop()
        self.logger.log("ESTOP")

    def recover(self) -> None:
        self.stage_state.recover_from_estop(
            board_locked=self.board_locked,
            serial_connected=self.controller.is_connected,
        )
        self.clear_target()
        self.logger.log("RECOVERED_FROM_ESTOP")
