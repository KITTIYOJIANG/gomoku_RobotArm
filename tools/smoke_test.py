from __future__ import annotations

import hashlib
import json
from pathlib import Path
import shutil
import sys
import tempfile
from typing import Callable


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
sys.dont_write_bytecode = True


def emit(text: str = "") -> None:
    line = str(text) + "\n"
    try:
        sys.stdout.write(line)
    except UnicodeEncodeError:
        stream = getattr(sys.stdout, "buffer", None)
        if stream is not None:
            stream.write(line.encode(getattr(sys.stdout, "encoding", None) or "utf-8", "replace"))


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest().upper()


def load_json(path: Path) -> dict:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise AssertionError(f"{path.name}: top level must be an object")
    return value


def check_imports_and_config() -> str:
    from app.config import AppConfig
    from app.arm.actions import ActionLibrary
    from app.arm.controller import SerialArmController
    from app.arm.sequences import place_to_p77
    from app.stage5.calibration_store import CalibrationStore
    from app.stage5.pwm_interpolator import resolve_target_pwm

    del SerialArmController, place_to_p77, CalibrationStore, resolve_target_pwm
    config = AppConfig.load(ROOT / "config" / "app_config.json")
    library = ActionLibrary(ROOT / "config" / "arm_actions.json")
    assert config.serial.baudrate == 115200
    assert config.stage5.default_dry_run is True
    assert len(library.names) == 11
    return "core config/calibration/arm modules import; Stage 5 defaults dry-run"


def check_stable_assets() -> str:
    state = load_json(ROOT / "project_state.json")
    baseline_info = state["baseline_calibration"]
    action_info = state["stable_action_table"]
    baseline = ROOT / baseline_info["path"]
    actions = ROOT / action_info["path"]
    assert sha256(baseline) == baseline_info["sha256"]
    assert sha256(actions) == action_info["sha256"]
    return "baseline and stable action hashes match project_state.json"


def check_baseline_resolution() -> str:
    from app.arm.actions import ActionLibrary
    from app.stage5.calibration_store import CalibrationStore
    from app.stage5.pwm_interpolator import resolve_target_pwm

    source = ROOT / "calibration" / "stage5_board_calibration.json"
    source_before = sha256(source)
    with tempfile.TemporaryDirectory(prefix="gomoku_smoke_") as directory:
        copy = Path(directory) / source.name
        shutil.copy2(source, copy)
        library = ActionLibrary(ROOT / "config" / "arm_actions.json")
        store = CalibrationStore(copy, library=library, safety_limits=None)
        sources: dict[str, int] = {}
        for row in range(15):
            for col in range(15):
                resolved = resolve_target_pwm(
                    store,
                    row,
                    col,
                    limits=None,
                    allow_star_seed=True,
                    allow_outer_seed=True,
                )
                pwm = resolved.pwm_str_keys()
                assert set(pwm) == {"000", "001", "002", "003", "004"}
                assert all(type(value) is int and 500 <= value <= 2500 for value in pwm.values())
                sources[resolved.source] = sources.get(resolved.source, 0) + 1
        assert sum(sources.values()) == 225
        assert sources.get("direct_anchor") == 30
    assert sha256(source) == source_before
    return f"225 ABOVE points resolved on a temporary baseline copy: {sources}"


def check_board_indexing() -> str:
    baseline = load_json(ROOT / "calibration" / "stage5_board_calibration.json")
    coordinates = baseline["coordinate_system"]
    assert coordinates == {"row_0": "top", "col_0": "left"}

    expected = {
        (0, 0): "P000",
        (0, 14): "P014",
        (7, 7): "P112",
        (14, 0): "P210",
        (14, 14): "P224",
    }
    for (row, col), label in expected.items():
        assert f"P{row * 15 + col:03d}" == label

    stage7_baseline = ROOT / "app" / "stage7" / "baseline.py"
    if stage7_baseline.exists():
        from app.stage7.baseline import point_id

        for (row, col), label in expected.items():
            assert point_id(row, col) == label
        suffix = "; Stage 7 point_id agrees"
    else:
        suffix = "; Stage 7 module absent"
    return "row-down/col-right and P000/P014/P112/P210/P224 mapping valid" + suffix


def check_action_pwm_and_dry_run() -> str:
    from app.arm.actions import ActionLibrary
    from app.arm.controller import SerialArmController
    from app.arm.sequences import ActionStep, place_to_p77

    library = ActionLibrary(ROOT / "config" / "arm_actions.json")
    for action in library:
        assert [target.servo_id for target in action.targets] == list(range(8))
        assert all(500 <= target.pwm <= 2500 for target in action.targets)

    controller = SerialArmController(dry_run=True)
    controller.connect("SMOKE_SIMULATED_PORT")
    assert controller.is_connected and controller._connection is None
    sequence = place_to_p77()
    for step in sequence.steps:
        if isinstance(step, ActionStep):
            controller.send_action(library.get(step.action_name))
    labels = [label for label, _command in controller.dry_run_commands]
    assert labels == [
        "CARRY_HIGH_P77_HOLD",
        "P77_ABOVE_HOLD",
        "P77_TOUCH_HOLD",
        "P77_TOUCH_RELEASE",
        "P77_ABOVE_IDLE",
        "CARRY_HIGH_P77_IDLE",
        "OBSERVE_IDLE",
    ]
    assert controller._connection is None
    controller.close()
    return f"{len(labels)} guarded P77 actions recorded by simulated controller; no serial object"


def check_stage6_offline_gate() -> str:
    config = load_json(ROOT / "config" / "stage6_descent.json")
    calibration = load_json(ROOT / "calibration" / "stage6_descent_calibration.json")
    batch = calibration["last_batch_generation"]
    assert config["force_dry_run"] is True
    assert len(calibration["above_fk_snapshot"]) == 225
    assert len(calibration["profiles"]) == 153
    assert batch["rejected_count"] == 72
    assert batch["all_candidates_verified"] is False
    return "force_dry_run=true; 225 snapshot / 153 profiles / 72 rejected"


def check_integrated_v1_golden_and_movel() -> str:
    from app.integrated_v1.golden import GOLDEN_ABOVE, assert_golden_above
    from app.integrated_v1.movel import MoveLPlanner
    from app.integrated_v1.profile import CalibrationProfileManager

    stable_baseline = ROOT / "calibration" / "stage5_board_calibration.json"
    stable_actions = ROOT / "config" / "arm_actions.json"
    before = (sha256(stable_baseline), sha256(stable_actions))
    with tempfile.TemporaryDirectory(prefix="gomoku_v1_smoke_") as directory:
        manager = CalibrationProfileManager(profile_path=Path(directory) / "profile.json")
        manager.create_from_stable_baseline()
        assert len(manager.data["above"]["points"]) == 225
        assert_golden_above(manager.data["above"]["points"])
        record = MoveLPlanner(manager).generate_point("P77", persist=False)
        assert record["waypoints"][0]["pwm"] == GOLDEN_ABOVE[(7, 7)].pwm_map
        assert record["reverse_ascent_indices"] == [4, 3, 2, 1, 0]
        assert not record["verified"]
    assert before == (sha256(stable_baseline), sha256(stable_actions))
    return "225 ABOVE initialized; five Golden anchors exact; P77 MoveL/reverse generated offline"


def check_integrated_v1_generate_all_offline() -> str:
    from app.integrated_v1.movel import MoveLPlanner
    from app.integrated_v1.profile import CalibrationProfileManager

    with tempfile.TemporaryDirectory(prefix="gomoku_v1_batch_smoke_") as directory:
        manager = CalibrationProfileManager(profile_path=Path(directory) / "profile.json")
        manager.create_from_stable_baseline()
        summary = MoveLPlanner(manager).generate_all(persist=False)
        assert summary.requested == 225
        assert summary.success + summary.unreachable + summary.invalid + summary.skipped == 225
        assert summary.golden_started == 5
    return (
        f"225 planned without controller/serial: generated={summary.success}, "
        f"unreachable={summary.unreachable}, invalid={summary.invalid}, skipped={summary.skipped}"
    )


def _new_lite_store(directory: str):
    from app.arm.actions import ActionLibrary
    from app.calibration_lite.drop_v1 import LiteDropStore
    from app.integrated_v1.points import all_points
    from app.integrated_v1.profile import CalibrationProfileManager

    library = ActionLibrary(ROOT / "config" / "arm_actions.json")
    source = CalibrationProfileManager(
        profile_path=Path(directory) / "unused_integrated.json",
        library=library,
    )
    source.create_from_stable_baseline(profile_id="lite-smoke-source")
    above = {
        point.as_tuple(): source.above_pwm(point)
        for point in all_points()
    }
    store = LiteDropStore(Path(directory) / "lite_drop.json", library=library)
    store.load_or_initialize(above, source={"smoke": True})
    return store, library


def check_lite_drop_v1_movel_and_dry_run() -> str:
    from app.arm.controller import SerialArmController
    from app.arm.sequences import ActionStep
    from app.calibration_lite.drop_v1 import LiteDropSequenceBuilder
    from app.integrated_v1.golden import GOLDEN_ABOVE
    from app.integrated_v1.movel import MoveLPlanner

    with tempfile.TemporaryDirectory(prefix="gomoku_lite_v1_smoke_") as directory:
        store, library = _new_lite_store(directory)
        planner = MoveLPlanner(store)
        record = planner.generate_point("P07_07", persist=False)
        assert record["waypoints"][0]["pwm"] == GOLDEN_ABOVE[(7, 7)].pwm_map
        assert [item["descent_mm"] for item in record["waypoints"]] == [
            0.0, 5.0, 10.0, 15.0, 20.0, 25.0
        ]
        assert record["reverse_ascent_indices"] == [4, 3, 2, 1, 0]
        assert record["verification_level"] == "NOT VERIFIED"
        builder = LiteDropSequenceBuilder(actions=library, store=store)
        sequence = builder.build_test_place("P07_07")
        assert sequence.action_names[-1].endswith("_WP_00_IDLE")
        controller = SerialArmController(dry_run=True)
        controller.connect("LITE_SMOKE_SIMULATED_PORT")
        for step in sequence.steps:
            if isinstance(step, ActionStep):
                controller.send_action(library.get(step.action_name))
        assert controller._connection is None
        assert library.get(sequence.action_names[-1]).target(5).pwm == 1500
        controller.close()
    return "P77 Cartesian-Z Test PLACE/retract recorded by Dry Run; no serial object"


def check_lite_drop_v1_generate_all_offline() -> str:
    from app.integrated_v1.movel import MoveLPlanner

    with tempfile.TemporaryDirectory(prefix="gomoku_lite_v1_batch_") as directory:
        store, _library = _new_lite_store(directory)
        planner = MoveLPlanner(store)
        summary = planner.generate_all(persist=False)
        assert summary.requested == 225
        assert summary.success + summary.unreachable + summary.invalid + summary.skipped == 225
        assert summary.golden_started == 5
        assert store.data["drop"]["last_generate_all"]["execution"] == (
            "OFFLINE_ONLY_NO_CONTROLLER_REFERENCE"
        )
        assert not hasattr(planner, "controller")
    return (
        f"225 Lite DROP candidates planned offline: generated={summary.success}, "
        f"unreachable={summary.unreachable}, invalid={summary.invalid}"
    )


def main() -> int:
    checks: list[tuple[str, Callable[[], str]]] = [
        ("Core imports/config", check_imports_and_config),
        ("Stable asset hashes", check_stable_assets),
        ("Baseline resolution", check_baseline_resolution),
        ("Board indexing", check_board_indexing),
        ("PWM parse + dry-run", check_action_pwm_and_dry_run),
        ("Stage 6 offline gate", check_stage6_offline_gate),
        ("V1 Golden + MoveL", check_integrated_v1_golden_and_movel),
        ("V1 225-point offline batch", check_integrated_v1_generate_all_offline),
        ("Lite V1 MoveL + Dry Run", check_lite_drop_v1_movel_and_dry_run),
        ("Lite V1 225-point offline batch", check_lite_drop_v1_generate_all_offline),
    ]

    emit("=" * 64)
    emit("GOMOKU ROBOT SAFE SMOKE TEST")
    emit("No camera, real serial port, worker thread, or hardware motion")
    emit("=" * 64)
    failures = 0
    for label, check in checks:
        try:
            detail = check()
            emit(f"[PASS] {label}: {detail}")
        except Exception as exc:
            failures += 1
            emit(f"[FAIL] {label}: {type(exc).__name__}: {exc}")
    emit()
    if failures:
        emit(f"SMOKE TEST FAILED: {failures} check(s)")
        return 1
    emit(f"SMOKE TEST PASSED: {len(checks)} checks")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
