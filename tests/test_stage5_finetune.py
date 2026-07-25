from __future__ import annotations

import json
from pathlib import Path

from app.arm.actions import ActionLibrary
from app.stage5.calibration_store import CalibrationStore
from app.stage5.hover_planner import HoverPlanner
from app.stage5.pwm_interpolator import interpolate_target_pwm
from app.stage5.safety import derive_calibration_limits, derive_pwm_safety_limits


def _write_base(path: Path) -> None:
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
                "row": 7, "col": 7, "pose_type": "TARGET_ABOVE",
                "pwm": {"000": 1560, "001": 1170, "002": 990, "003": 1170, "004": 1500},
                "time_ms": 1000, "calibrated": True, "verified_runs": 1, "notes": "",
            },
            "3,7": {
                "row": 3, "col": 7, "pose_type": "TARGET_ABOVE",
                "pwm": {"000": 1553, "001": 1140, "002": 1180, "003": 1125, "004": 1500},
                "time_ms": 1000, "calibrated": True, "verified_runs": 3, "notes": "",
            },
            "11,7": {
                "row": 11, "col": 7, "pose_type": "TARGET_ABOVE",
                "pwm": {"000": 1590, "001": 1230, "002": 720, "003": 1370, "004": 1500},
                "time_ms": 1000, "calibrated": True, "verified_runs": 3, "notes": "",
            },
        },
        "limits": {},
        "metadata": {},
    }
    path.write_text(json.dumps(data, indent=2), encoding="utf-8")


def test_expand_grid_and_direct_midpoint(tmp_path: Path) -> None:
    path = tmp_path / "cal.json"
    _write_base(path)
    lib = ActionLibrary()
    store = CalibrationStore(path, library=lib)
    # inferred mid
    mid = interpolate_target_pwm(store, 5, 7)
    assert mid.source == "bilinear_interpolation"
    taught = {"000": 1550, "001": 1150, "002": 1100, "003": 1140, "004": 1500}
    store.upsert_anchor(
        5, 7, taught,
        calibrated=True,
        verified_runs=1,
        require_anchor_set=False,
        expand_grid=True,
        safety_limits=derive_calibration_limits(lib),
    )
    store.save()
    store.reload()
    assert 5 in store.anchor_rows
    assert 7 in store.anchor_cols
    direct = interpolate_target_pwm(store, 5, 7)
    assert direct.source == "direct_anchor"
    assert direct.pwm[0] == 1550


def test_plan_hover_with_user_pwm(tmp_path: Path) -> None:
    path = tmp_path / "cal.json"
    _write_base(path)
    lib = ActionLibrary()
    store = CalibrationStore(path, library=lib)
    planner = HoverPlanner(
        library=lib,
        store=store,
        limits=derive_pwm_safety_limits(lib),
        action_wait_margin_ms=50,
    )
    plan = planner.plan_hover_with_pwm(
        4, 7,
        {"000": 1555, "001": 1148, "002": 1130, "003": 1136, "004": 1500},
        holding_piece=False,
        dry_run=True,
    )
    assert plan.source == "user_edited"
    assert plan.target_row == 4
    assert "user_pwm_override" in plan.safety_checks
