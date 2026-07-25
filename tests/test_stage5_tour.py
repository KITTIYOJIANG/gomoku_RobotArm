from __future__ import annotations

import json
from pathlib import Path

from app.arm.actions import ActionLibrary
from app.arm.controller import SerialArmController
from app.stage5.calibration_store import CalibrationStore
from app.stage5.safety import derive_calibration_limits
from app.stage5.tour_planner import (
    build_board_reachable_tour,
    build_cross_reverify_tour,
    order_stops_cross_axes,
    sync_completed_drafts_into_calibration,
    list_reachable_board_stops,
    TourStop,
)


def _write_cal(path: Path) -> None:
    data = {
        "schema_version": 1,
        "board_size": 15,
        "coordinate_system": {"row_0": "top", "col_0": "left"},
        "joint_ids": ["000", "001", "002", "003", "004"],
        "pump_joint_id": "005",
        "anchor_rows": [3, 7, 11],
        "anchor_cols": [3, 7, 11],
        "anchors": {
            "7,7": {
                "row": 7,
                "col": 7,
                "pose_type": "TARGET_ABOVE",
                "pwm": {"000": 1560, "001": 1170, "002": 990, "003": 1170, "004": 1500},
                "time_ms": 1000,
                "calibrated": True,
                "verified_runs": 1,
                "notes": "p77",
            },
            "3,7": {
                "row": 3,
                "col": 7,
                "pose_type": "TARGET_ABOVE",
                "pwm": {"000": 1553, "001": 1140, "002": 1180, "003": 1125, "004": 1500},
                "time_ms": 1000,
                "calibrated": True,
                "verified_runs": 3,
                "notes": "up",
            },
        },
        "limits": {},
        "metadata": {},
    }
    path.write_text(json.dumps(data, indent=2), encoding="utf-8")


def test_cross_reverify_tour_builds_sequence(tmp_path: Path) -> None:
    cal_path = tmp_path / "cal.json"
    _write_cal(cal_path)
    lib = ActionLibrary()
    store = CalibrationStore(cal_path, library=lib)
    plan = build_cross_reverify_tour(lib, store, dwell_ms=100, action_wait_margin_ms=50)
    assert plan.name == "CROSS_REVERIFY_TOUR"
    assert len(plan.stops) == 1
    assert plan.stops[0].row == 3 and plan.stops[0].col == 7
    assert plan.sequence.action_names[0] in {"CARRY_HIGH_P77_IDLE", "CARRY_HIGH_LIFTED_IDLE"}
    assert plan.sequence.action_names[-1] == "OBSERVE_IDLE"
    assert "TARGET_TOUR_P37" in plan.sequence.action_names


def test_sync_completed_draft_promotes_missing(tmp_path: Path) -> None:
    cal_path = tmp_path / "cal.json"
    _write_cal(cal_path)
    lib = ActionLibrary()
    store = CalibrationStore(cal_path, library=lib)
    drafts = {
        "version": "1.0",
        "anchors": {
            "7,11": {
                "row": 7,
                "col": 11,
                "status": "COMPLETED",
                "verified_runs": 3,
                "candidate_pwm": {"000": 1470, "001": 1200, "002": 990, "003": 1190, "004": 1500},
            }
        },
    }
    written = sync_completed_drafts_into_calibration(
        store, drafts, safety_limits=derive_calibration_limits(lib)
    )
    assert written == ["7,11"]
    store.reload()
    assert store.get_anchor(7, 11) is not None
    assert store.get_anchor(7, 11).calibrated


def test_board_tour_includes_interpolated_cross_line(tmp_path: Path) -> None:
    cal_path = tmp_path / "cal.json"
    _write_cal(cal_path)
    lib = ActionLibrary()
    store = CalibrationStore(cal_path, library=lib)
    store.upsert_anchor(
        7,
        3,
        {"000": 1650, "001": 1200, "002": 1000, "003": 1190, "004": 1500},
        calibrated=True,
        verified_runs=3,
        safety_limits=derive_calibration_limits(lib),
    )
    store.upsert_anchor(
        7,
        11,
        {"000": 1470, "001": 1200, "002": 990, "003": 1190, "004": 1500},
        calibrated=True,
        verified_runs=3,
        safety_limits=derive_calibration_limits(lib),
    )
    store.save()
    plan = build_board_reachable_tour(
        lib,
        store,
        limits=derive_calibration_limits(lib),
        dwell_ms=50,
        action_wait_margin_ms=20,
        cool_every_n=4,
        cool_ms=100,
        path_mode="segment",
    )
    assert plan.name == "BOARD_HOVER_TOUR"
    coords = [(s.row, s.col) for s in plan.stops]
    assert (7, 7) in coords
    assert (7, 5) in coords
    assert (3, 7) in coords
    # Vertical arm starts at top, not center-first jump
    assert coords[0] == (3, 7)
    assert plan.sequence.action_names[-1] == "OBSERVE_IDLE"
    # Cooling injects OBSERVE mid-tour
    assert plan.sequence.action_names.count("OBSERVE_IDLE") >= 2


def test_board_tour_direct_only_skips_interpolation(tmp_path: Path) -> None:
    cal_path = tmp_path / "cal.json"
    _write_cal(cal_path)
    lib = ActionLibrary()
    store = CalibrationStore(cal_path, library=lib)
    store.upsert_anchor(
        7,
        3,
        {"000": 1650, "001": 1200, "002": 1000, "003": 1190, "004": 1500},
        calibrated=True,
        verified_runs=3,
        safety_limits=derive_calibration_limits(lib),
    )
    store.save()
    plan = build_board_reachable_tour(
        lib,
        store,
        limits=derive_calibration_limits(lib),
        direct_only=True,
        cool_every_n=0,
        cool_ms=0,
        path_mode="carry_each",
    )
    assert all(s.source == "direct_anchor" for s in plan.stops)
    assert {(s.row, s.col) for s in plan.stops} == {(7, 7), (3, 7), (7, 3)}


def test_order_stops_cross_axes() -> None:
    stops = [
        TourStop(7, 11, "r", "direct_anchor", {"000": 1, "001": 1, "002": 1, "003": 1, "004": 1}),
        TourStop(3, 7, "u", "direct_anchor", {"000": 1, "001": 1, "002": 1, "003": 1, "004": 1}),
        TourStop(7, 7, "c", "direct_anchor", {"000": 1, "001": 1, "002": 1, "003": 1, "004": 1}),
        TourStop(5, 7, "m", "bilinear_interpolation", {"000": 1, "001": 1, "002": 1, "003": 1, "004": 1}),
    ]
    ordered = order_stops_cross_axes(stops)
    # Vertical top→bottom first, then remaining horizontal.
    assert [(s.row, s.col) for s in ordered] == [(3, 7), (5, 7), (7, 7), (7, 11)]


def test_controller_beep_dry_run() -> None:
    ctl = SerialArmController(dry_run=True)
    ctl.connect("COM_TEST")
    sent = ctl.beep(1, 100)
    assert "$BEEP:1,100!" in sent
    assert any(c.startswith("beep,1") for c in sent)
    assert all(label == "BEEP" for label, _ in ctl.dry_run_commands[-len(sent) :])
