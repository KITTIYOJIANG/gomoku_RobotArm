from __future__ import annotations

import json
from pathlib import Path

from app.arm.actions import ActionLibrary
from app.stage5.calibration_store import CalibrationStore
from app.stage5.pwm_interpolator import (
    estimate_star_corner_pwm,
    interpolate_target_pwm,
    resolve_target_pwm,
)
from app.stage5.safety import derive_calibration_limits


def _base(path: Path) -> None:
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
            "7,3": {
                "row": 7, "col": 3, "pose_type": "TARGET_ABOVE",
                "pwm": {"000": 1650, "001": 1200, "002": 1000, "003": 1190, "004": 1500},
                "time_ms": 1000, "calibrated": True, "verified_runs": 3, "notes": "",
            },
            "7,11": {
                "row": 7, "col": 11, "pose_type": "TARGET_ABOVE",
                "pwm": {"000": 1470, "001": 1200, "002": 990, "003": 1190, "004": 1500},
                "time_ms": 1000, "calibrated": True, "verified_runs": 3, "notes": "",
            },
        },
        "limits": {},
        "metadata": {},
    }
    path.write_text(json.dumps(data, indent=2), encoding="utf-8")


def test_priority_taught_over_default(tmp_path: Path) -> None:
    path = tmp_path / "c.json"
    _base(path)
    lib = ActionLibrary()
    store = CalibrationStore(path, library=lib)
    mid = interpolate_target_pwm(store, 5, 7)
    assert mid.source == "bilinear_interpolation"
    taught = {"000": 1500, "001": 1111, "002": 1000, "003": 1100, "004": 1500}
    store.upsert_anchor(
        5, 7, taught, calibrated=True, require_anchor_set=False, expand_grid=True,
        safety_limits=derive_calibration_limits(lib),
    )
    store.save()
    store.reload()
    direct = interpolate_target_pwm(store, 5, 7)
    assert direct.source == "direct_anchor"
    assert direct.pwm[1] == 1111


def test_star_seed_parallelogram(tmp_path: Path) -> None:
    path = tmp_path / "c.json"
    _base(path)
    store = CalibrationStore(path, library=ActionLibrary())
    seed = estimate_star_corner_pwm(store, 3, 3)
    assert seed.source == "star_parallelogram_seed"
    # 000: 1553 + 1650 - 1560 = 1643
    assert seed.pwm[0] == 1643
    # resolve falls back to seed when no bilinear cell
    resolved = resolve_target_pwm(store, 3, 3, allow_star_seed=True)
    assert resolved.source == "star_parallelogram_seed"


def test_star_corners_enable_interior(tmp_path: Path) -> None:
    path = tmp_path / "c.json"
    _base(path)
    lib = ActionLibrary()
    store = CalibrationStore(path, library=lib)
    for r, c in [(3, 3), (3, 11), (11, 3), (11, 11)]:
        seed = estimate_star_corner_pwm(store, r, c)
        store.upsert_anchor(
            r, c, seed.pwm_str_keys(),
            calibrated=True, require_anchor_set=False, expand_grid=True,
            safety_limits=derive_calibration_limits(lib),
        )
    store.save()
    store.reload()
    interior = interpolate_target_pwm(store, 5, 5)
    assert interior.source == "bilinear_interpolation"
