from __future__ import annotations

from pathlib import Path

from app.arm.actions import ActionLibrary
from app.stage5.calibration_store import CalibrationStore
from app.stage5.safety import derive_pwm_safety_limits


def test_atomic_save_and_backup(tmp_path: Path):
    lib = ActionLibrary()
    limits = derive_pwm_safety_limits(lib)
    path = tmp_path / "stage5_board_calibration.json"
    store = CalibrationStore(path, library=lib, safety_limits=limits)
    anchor = store.get_anchor(7, 7)
    assert anchor is not None and anchor.calibrated
    store.save()
    assert path.exists()
    backups = list((tmp_path / "backups").glob("stage5_board_calibration_*.json"))
    assert backups
    # Reload
    store2 = CalibrationStore(path, library=lib, safety_limits=limits)
    assert store2.get_anchor(7, 7).pwm["000"] == 1560
