from __future__ import annotations

from pathlib import Path

import pytest

from app.arm.actions import ActionLibrary
from app.stage5.calibration_store import CalibrationStore
from app.stage5.pwm_interpolator import InterpolationError, interpolate_target_pwm
from app.stage5.safety import derive_pwm_safety_limits


@pytest.fixture()
def store(tmp_path: Path) -> CalibrationStore:
    lib = ActionLibrary()
    limits = derive_pwm_safety_limits(lib)
    path = tmp_path / "stage5_board_calibration.json"
    s = CalibrationStore(path, library=lib, safety_limits=limits)
    # Seed a minimal calibrated 2x2 cell around P77 using small deltas from P77.
    base = s.get_anchor(7, 7)
    assert base is not None
    bp = base.spatial_pwm()
    samples = {
        (3, 3): {0: bp[0] + 40, 1: bp[1] + 20, 2: bp[2] - 10, 3: bp[3] + 10, 4: bp[4]},
        (3, 7): {0: bp[0] + 20, 1: bp[1] + 10, 2: bp[2] - 5, 3: bp[3] + 5, 4: bp[4]},
        (3, 11): {0: bp[0] + 0, 1: bp[1] + 0, 2: bp[2] + 0, 3: bp[3] + 0, 4: bp[4]},
        (7, 3): {0: bp[0] + 30, 1: bp[1] + 15, 2: bp[2] - 8, 3: bp[3] + 8, 4: bp[4]},
        (7, 11): {0: bp[0] - 10, 1: bp[1] - 5, 2: bp[2] + 4, 3: bp[3] - 4, 4: bp[4]},
        (11, 3): {0: bp[0] + 10, 1: bp[1] + 5, 2: bp[2] - 2, 3: bp[3] + 2, 4: bp[4]},
        (11, 7): {0: bp[0] - 5, 1: bp[1] - 2, 2: bp[2] + 2, 3: bp[3] - 2, 4: bp[4]},
        (11, 11): {0: bp[0] - 20, 1: bp[1] - 10, 2: bp[2] + 6, 3: bp[3] - 6, 4: bp[4]},
    }
    for (r, c), pwm in samples.items():
        s.upsert_anchor(r, c, {f"{k:03d}": v for k, v in pwm.items()}, calibrated=True, verified_runs=1)
    s.save()
    return s


def test_p77_is_direct_anchor(store: CalibrationStore):
    limits = derive_pwm_safety_limits(ActionLibrary())
    result = interpolate_target_pwm(store, 7, 7, limits=limits)
    assert result.source == "direct_anchor"
    lib = ActionLibrary()
    for jid in range(5):
        assert result.pwm[jid] == lib.get("P77_ABOVE_IDLE").target(jid).pwm


def test_bilinear_center_of_cell(store: CalibrationStore):
    limits = derive_pwm_safety_limits(ActionLibrary())
    # With full 3x3 anchors, (5,5) is interior of top-left cell.
    result = interpolate_target_pwm(store, 5, 5, limits=limits)
    assert result.source == "bilinear_interpolation"
    assert result.u is not None and result.v is not None
    assert 0.0 <= result.u <= 1.0
    assert 0.0 <= result.v <= 1.0


def test_outside_region_rejected(store: CalibrationStore):
    limits = derive_pwm_safety_limits(ActionLibrary())
    with pytest.raises(InterpolationError) as exc:
        interpolate_target_pwm(store, 0, 0, limits=limits)
    assert exc.value.code == "TARGET_OUTSIDE_CALIBRATED_REGION"


def test_missing_corner_uncalibrated(tmp_path: Path):
    lib = ActionLibrary()
    limits = derive_pwm_safety_limits(lib)
    s = CalibrationStore(tmp_path / "c.json", library=lib, safety_limits=limits)
    # Only P77 calibrated.
    with pytest.raises(InterpolationError) as exc:
        interpolate_target_pwm(s, 5, 5, limits=limits)
    assert exc.value.code in {"TARGET_UNCALIBRATED", "TARGET_OUTSIDE_CALIBRATED_REGION"}
