from __future__ import annotations

import json
from pathlib import Path

from app.arm.actions import ActionLibrary
from app.stage5.calibration_store import CalibrationStore
from app.stage5.pwm_interpolator import estimate_outer_ring_pwm, interpolate_target_pwm, resolve_target_pwm
from app.stage5.safety import derive_calibration_limits


def _write(path: Path) -> None:
    anchors = {
        "7,7": {"row": 7, "col": 7, "pwm": {"000": 1560, "001": 1170, "002": 990, "003": 1170, "004": 1500}},
        "3,7": {"row": 3, "col": 7, "pwm": {"000": 1553, "001": 1140, "002": 1180, "003": 1125, "004": 1500}},
        "11,7": {"row": 11, "col": 7, "pwm": {"000": 1590, "001": 1230, "002": 720, "003": 1370, "004": 1500}},
        "7,3": {"row": 7, "col": 3, "pwm": {"000": 1650, "001": 1200, "002": 1000, "003": 1190, "004": 1500}},
        "7,11": {"row": 7, "col": 11, "pwm": {"000": 1470, "001": 1200, "002": 990, "003": 1190, "004": 1500}},
        "3,3": {"row": 3, "col": 3, "pwm": {"000": 1628, "001": 1170, "002": 1190, "003": 1145, "004": 1500}},
        "3,11": {"row": 3, "col": 11, "pwm": {"000": 1483, "001": 1170, "002": 1180, "003": 1145, "004": 1500}},
        "11,3": {"row": 11, "col": 3, "pwm": {"000": 1680, "001": 1260, "002": 740, "003": 1390, "004": 1500}},
        "11,11": {"row": 11, "col": 11, "pwm": {"000": 1465, "001": 1260, "002": 720, "003": 1390, "004": 1500}},
    }
    full = {}
    for k, v in anchors.items():
        full[k] = {
            **v,
            "pose_type": "TARGET_ABOVE",
            "time_ms": 1000,
            "calibrated": True,
            "verified_runs": 1,
            "notes": "",
        }
    data = {
        "schema_version": 1,
        "board_size": 15,
        "anchor_rows": [3, 7, 11],
        "anchor_cols": [3, 7, 11],
        "anchors": full,
        "limits": {},
        "metadata": {},
    }
    path.write_text(json.dumps(data), encoding="utf-8")


def test_outer_mid_seed_and_full_cover(tmp_path: Path) -> None:
    path = tmp_path / "c.json"
    _write(path)
    lib = ActionLibrary()
    store = CalibrationStore(path, library=lib)
    top = estimate_outer_ring_pwm(store, 0, 7)
    assert top.source == "outer_ring_seed"
    # teach outer ring
    for r, c in [(0, 7), (14, 7), (7, 0), (7, 14), (0, 0), (0, 14), (14, 0), (14, 14)]:
        seed = estimate_outer_ring_pwm(store, r, c)
        store.upsert_anchor(
            r, c, seed.pwm_str_keys(),
            calibrated=True, require_anchor_set=False, expand_grid=True,
            safety_limits=derive_calibration_limits(lib),
            skip_envelope_check=True,
        )
    store.save()
    store.reload()
    assert store.allowed_region["row_min"] == 0
    assert store.allowed_region["row_max"] == 14
    # corner of board should resolve
    res = interpolate_target_pwm(store, 0, 0)
    assert res.source == "direct_anchor"
    mid = interpolate_target_pwm(store, 2, 2)
    assert mid.source == "bilinear_interpolation"
