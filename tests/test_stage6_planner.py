from __future__ import annotations

from dataclasses import replace
import hashlib
import logging
from pathlib import Path

import pytest

from app.arm.actions import ActionLibrary
from app.arm.controller import SerialArmController
from app.stage6.calibration_store import (
    Stage6CalibrationError,
    Stage6CalibrationStore,
)
from app.stage6.models import DESCENT_LEVELS, DescentLevel
from app.stage6.planner import Stage6DescentPlanner, Stage6ExecutionBlocked
from app.stage6.settings import Stage6Settings
from app.stage6.state_machine import Stage6MotionState, Stage6TransitionError
from app.stage6.thermal import ThermalLockout


@pytest.fixture()
def planner(tmp_path: Path) -> Stage6DescentPlanner:
    path = tmp_path / "stage6_descent_calibration.json"
    settings = replace(
        Stage6Settings.load(),
        descent_calibration_path=path,
    )
    controller = SerialArmController(dry_run=True)
    store = Stage6CalibrationStore(path)
    return Stage6DescentPlanner(
        controller=controller,
        settings=settings,
        calibration_store=store,
    )


def action_spatial(name: str) -> dict[str, int]:
    action = ActionLibrary().get(name)
    return {
        f"{joint_id:03d}": action.target(joint_id).pwm for joint_id in range(5)
    }


def test_stage6_reads_all_above_without_modifying_source(
    planner: Stage6DescentPlanner,
) -> None:
    source = planner.settings.above_calibration_path
    before = hashlib.sha256(source.read_bytes()).hexdigest()
    records = planner.above_source.resolve_all()
    after = hashlib.sha256(source.read_bytes()).hexdigest()
    assert len(records) == 225
    assert before == after
    assert sum(item.verified for item in records.values()) == 30


def test_stage6_batch_attempts_all_225_and_records_rejections(
    planner: Stage6DescentPlanner,
) -> None:
    result = planner.generate_all_descent_profiles()
    assert result.requested == 225
    assert result.generated_count + result.rejected_count == 225
    assert result.generated_count > 0
    saved = planner.store._data["last_batch_generation"]
    assert saved["requested"] == 225
    assert saved["generated_count"] == result.generated_count
    assert saved["rejected_count"] == result.rejected_count
    assert saved["all_candidates_verified"] is False
    assert len(planner.store._data["above_fk_snapshot"]) == 225


def test_stage6_every_generated_trajectory_has_five_complete_levels(
    planner: Stage6DescentPlanner,
) -> None:
    result = planner.generate_all_descent_profiles(persist=False)
    expected = tuple(DESCENT_LEVELS)
    assert result.generated
    for profile in result.generated.values():
        assert tuple(item.level for item in profile.levels) == expected
        assert all(set(item.computed_pwm) == {"000", "001", "002", "003", "004"} for item in profile.levels)


def test_stage6_reverse_ascent_is_exact_descent_reverse(
    planner: Stage6DescentPlanner,
) -> None:
    planner.generate_descent_profile(7, 7)
    preview = planner.preview_descent_profile(7, 7)
    assert preview["reverse_is_exact"] is True
    descent = [
        item["pwm"]
        for item in preview["commands"]
        if item["phase"] == "descent"
    ]
    ascent = [
        item["pwm"]
        for item in preview["commands"]
        if item["phase"] == "reverse_ascent"
    ]
    assert ascent == list(reversed(descent[:-1]))


def test_stage6_low_position_target_switch_is_rejected(
    planner: Stage6DescentPlanner,
) -> None:
    planner.state.establish_above(7, 7)
    planner.state.descend(7, 7, DescentLevel.DESCENT_25)
    with pytest.raises(Stage6TransitionError, match="BELOW_ABOVE_LOCKED"):
        planner.state.establish_above(7, 8)


def test_stage6_above_to_other_above_requires_carry_high(
    planner: Stage6DescentPlanner,
) -> None:
    planner.state.establish_above(7, 7)
    with pytest.raises(Stage6TransitionError):
        planner.state.establish_above(7, 8)
    planner.state.move_to_carry_high()
    planner.state.establish_above(7, 8)
    assert planner.state.locked_target == (7, 8)


def test_stage6_touch_cannot_move_directly_to_observe(
    planner: Stage6DescentPlanner,
) -> None:
    planner.state.establish_above(7, 7)
    for level in (
        DescentLevel.DESCENT_25,
        DescentLevel.DESCENT_50,
        DescentLevel.DESCENT_75,
        DescentLevel.TOUCH,
    ):
        planner.state.descend(7, 7, level)
    with pytest.raises(Stage6TransitionError, match="forbidden below ABOVE"):
        planner.state.move_to_observe()


def test_stage6_manual_delta_adds_and_recompute_preserves_it(
    planner: Stage6DescentPlanner,
) -> None:
    profile = planner.generate_descent_profile(7, 7)
    base = profile.level(DescentLevel.DESCENT_50).computed_pwm["001"]
    planner.store.apply_delta(7, 7, DescentLevel.DESCENT_50, {"001": 5})
    assert (
        planner.store.final_pwm(7, 7, DescentLevel.DESCENT_50)["001"]
        == base + 5
    )
    planner.generate_descent_profile(7, 7)
    stored = planner.store.profile(7, 7)["levels"]["descent_50"]
    assert stored["manual_delta_pwm"]["001"] == 5
    assert stored["final_pwm"]["001"] == stored["computed_pwm"]["001"] + 5


def test_stage6_unsafe_final_delta_is_rejected_transactionally(
    planner: Stage6DescentPlanner,
) -> None:
    planner.generate_descent_profile(7, 7)
    before = planner.store.final_pwm(7, 7, DescentLevel.TOUCH)
    with pytest.raises(Stage6CalibrationError, match="outside safe range"):
        planner.store.apply_delta(
            7, 7, DescentLevel.TOUCH, {"001": -10000}
        )
    assert planner.store.final_pwm(7, 7, DescentLevel.TOUCH) == before


def test_stage6_unverified_profile_cannot_place_piece(
    planner: Stage6DescentPlanner,
) -> None:
    planner.generate_descent_profile(7, 7)
    with pytest.raises(Stage6ExecutionBlocked, match="not all descent levels verified"):
        planner.place_piece_at(
            7,
            7,
            target_empty=True,
            arm_holding=True,
            board_locked=True,
        )


def test_stage6_p77_uses_stable_above_and_touch_endpoints(
    planner: Stage6DescentPlanner,
) -> None:
    profile = planner.generate_descent_profile(7, 7)
    assert profile.level(DescentLevel.ABOVE).computed_pwm == action_spatial(
        "P77_ABOVE_IDLE"
    )
    assert profile.level(DescentLevel.TOUCH).computed_pwm == action_spatial(
        "P77_TOUCH_HOLD"
    )


def test_stage6_dry_run_records_commands_without_real_connection(
    planner: Stage6DescentPlanner,
) -> None:
    planner.generate_descent_profile(7, 7)
    commands = planner.dry_run_p77_regression()
    assert len(commands) == 13
    assert planner.controller._connection is None
    assert planner.controller.dry_run_commands == commands
    assert "RELEASE_DWELL_700MS" in planner.last_p77_dry_run_events
    assert planner.state.state == Stage6MotionState.OBSERVE


def test_stage6_logs_computed_delta_and_final(
    planner: Stage6DescentPlanner, caplog: pytest.LogCaptureFixture
) -> None:
    caplog.set_level(logging.INFO)
    planner.generate_descent_profile(7, 7)
    text = "\n".join(record.getMessage() for record in caplog.records)
    assert "computed=" in text
    assert "delta=" in text
    assert "final=" in text


def test_stage6_overheat_lock_rejects_new_task(
    planner: Stage6DescentPlanner,
) -> None:
    planner.report_overheat()
    with pytest.raises(ThermalLockout, match="OVERHEAT_LOCKED"):
        planner.thermal.require_available()
    assert planner.controller.dry_run_commands == []


def test_stage6_touch_dwell_timeout_requests_safe_return(
    planner: Stage6DescentPlanner,
) -> None:
    planner.thermal.enter_dwell("TOUCH")
    started = planner.thermal._dwell_started
    assert started is not None
    warning = planner.thermal.dwell_warning(
        now=started + planner.settings.max_touch_dwell_seconds + 0.1
    )
    assert warning is not None
    assert "TOUCH_DWELL_TIMEOUT" in warning
    assert "safe return" in warning
