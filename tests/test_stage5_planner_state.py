from __future__ import annotations

from pathlib import Path

import pytest

from app.arm.actions import ActionLibrary
from app.arm.controller import SerialArmController
from app.arm.state import ArmState, ArmStateMachine
from app.arm.worker import ArmSequenceWorker
from app.config import Stage5Config
from app.stage5.coordinator import Stage5Coordinator
from app.stage5.hover_planner import HoverPlanner
from app.stage5.safety import derive_pwm_safety_limits
from app.stage5.state_machine import Stage5State, Stage5StateMachine


def test_hover_plan_path_uses_carry_high_and_no_touch(tmp_path: Path):
    lib = ActionLibrary()
    limits = derive_pwm_safety_limits(lib)
    from app.stage5.calibration_store import CalibrationStore

    store = CalibrationStore(tmp_path / "c.json", library=lib, safety_limits=limits)
    planner = HoverPlanner(lib, store, limits)
    plan = planner.plan_hover_to(7, 7, holding_piece=False, dry_run=True)
    names = plan.sequence.action_names
    assert names[0].startswith("CARRY_HIGH")
    assert names[1].startswith("TARGET_ABOVE")
    assert all("TOUCH" not in n and "RELEASE" not in n for n in names)
    # Pump remains idle on non-holding path.
    assert "#005P1500" in plan.target_action.command
    assert plan.interpolation.source == "direct_anchor"
    assert plan.interpolation.pwm[0] == 1560


def test_hover_hold_keeps_pump_on(tmp_path: Path):
    lib = ActionLibrary()
    limits = derive_pwm_safety_limits(lib)
    from app.stage5.calibration_store import CalibrationStore

    store = CalibrationStore(tmp_path / "c.json", library=lib, safety_limits=limits)
    planner = HoverPlanner(lib, store, limits)
    plan = planner.plan_hover_to(7, 7, holding_piece=True, dry_run=True)
    assert plan.sequence.action_names[0] == "CARRY_HIGH_P77_HOLD"
    assert "#005P2500" in plan.target_action.command


def test_stage5_state_machine_click_does_not_move():
    sm = Stage5StateMachine(dry_run=True)
    sm.on_serial_connected(board_locked=True)
    sm.on_arm_state("OBSERVE_IDLE")
    assert sm.state == Stage5State.READY
    sm.select_target(7, 7, board_locked=True, calibrated=True, in_region=True)
    assert sm.state == Stage5State.DRY_RUN_READY
    # Still not moving until begin_hover.
    assert not sm.is_moving()


def test_coordinator_dry_run_hover(tmp_path: Path):
    lib = ActionLibrary()
    controller = SerialArmController(dry_run=True)
    controller.connect("COM6")
    arm = ArmStateMachine()
    arm.connect()
    # Put arm into observe idle as if user returned to observe.
    arm.begin_return_to_observe()
    arm.complete_return_to_observe()
    worker = ArmSequenceWorker(controller, lib)
    worker.start()
    try:
        cfg = Stage5Config(
            enabled=True,
            default_dry_run=True,
            click_threshold_ratio=0.32,
            calibration_path=tmp_path / "c.json",
            board_span_cells=8,
            allow_motion_without_camera=False,
        )
        coord = Stage5Coordinator(
            config=cfg,
            actions=lib,
            controller=controller,
            arm_state=arm,
            worker=worker,
            action_wait_margin_ms=50,
            logs_dir=tmp_path,
        )
        coord.on_serial_connected()
        coord.on_board_lock_changed(True)
        coord.on_arm_state("OBSERVE_IDLE")
        coord.select_target_programmatically(7, 7, 15)
        plan = coord.plan_hover(holding_piece=False)
        submitted, mode = coord.begin_hover_execution(plan)
        assert mode == "dry_run"
        assert submitted is False
        coord.complete_hover_dry_run()
        assert coord.stage_state.state == Stage5State.HOVERING
        assert arm.snapshot().state == ArmState.HOVERING
        # No live serial commands should have been written in dry-run path.
        assert controller.dry_run_commands == []
    finally:
        worker.shutdown()
        controller.close()
