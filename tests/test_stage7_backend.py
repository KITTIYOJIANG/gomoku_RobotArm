from __future__ import annotations

from pathlib import Path

import pytest

from app.arm.actions import ActionLibrary
from app.stage5.safety import derive_calibration_limits
from app.stage7.baseline import BaselineSnapshot, SPATIAL_KEYS, point_id
from app.stage7.session import (
    CalibrationMode,
    QUICK_COORDS,
    STANDARD_COORDS,
    RapidCalibrationSession,
)
from app.stage7.settings import Stage7Settings


@pytest.fixture(scope="module")
def stage7_assets() -> tuple[BaselineSnapshot, ActionLibrary]:
    settings = Stage7Settings.load()
    library = ActionLibrary()
    return (
        BaselineSnapshot(
            settings.baseline_path,
            board_size=settings.board_size,
            library=library,
        ),
        library,
    )


def make_session(
    tmp_path: Path,
    stage7_assets: tuple[BaselineSnapshot, ActionLibrary],
    mode: CalibrationMode = CalibrationMode.QUICK_5,
) -> RapidCalibrationSession:
    baseline, library = stage7_assets
    return RapidCalibrationSession(
        baseline=baseline,
        mode=mode,
        session_id="offline_test",
        path=tmp_path / "offline_test.json",
        limits=derive_calibration_limits(library),
    )


def save_delta(
    session: RapidCalibrationSession,
    row: int,
    col: int,
    deltas: dict[str, int],
) -> None:
    baseline = session.baseline.get(row, col).pwm
    session.save_anchor(
        row,
        col,
        {joint: baseline[joint] + int(deltas.get(joint, 0)) for joint in SPATIAL_KEYS},
    )


def test_a_zero_delta_reproduces_all_225_baseline_points(
    tmp_path: Path,
    stage7_assets: tuple[BaselineSnapshot, ActionLibrary],
) -> None:
    session = make_session(tmp_path, stage7_assets)
    for row, col in QUICK_COORDS:
        save_delta(session, row, col, {})

    generated = session.recalculate()

    assert len(generated) == 225
    for row in range(15):
        for col in range(15):
            record = generated[point_id(row, col)]
            assert record["new_pwm"] == session.baseline.get(row, col).pwm
            assert record["delta_pwm"] == {joint: 0 for joint in SPATIAL_KEYS}


def test_b_constant_joint_delta_stays_constant_inside_board(
    tmp_path: Path,
    stage7_assets: tuple[BaselineSnapshot, ActionLibrary],
) -> None:
    session = make_session(tmp_path, stage7_assets, CalibrationMode.STANDARD_9)
    for row, col in STANDARD_COORDS:
        save_delta(session, row, col, {"001": 5})

    generated = session.recalculate()

    for record in generated.values():
        assert record["delta_pwm"]["001"] == 5
        assert record["new_pwm"]["001"] == record["baseline_pwm"]["001"] + 5


def test_c_piecewise_bilinear_delta_is_mathematically_correct(
    tmp_path: Path,
    stage7_assets: tuple[BaselineSnapshot, ActionLibrary],
) -> None:
    session = make_session(tmp_path, stage7_assets, CalibrationMode.STANDARD_9)
    # A planar field is reproduced exactly by each of the four bilinear cells.
    for row, col in STANDARD_COORDS:
        save_delta(session, row, col, {"000": row + 2 * col})

    generated = session.recalculate()

    assert generated[point_id(3, 4)]["delta_pwm"]["000"] == 11
    assert generated[point_id(10, 12)]["delta_pwm"]["000"] == 34


def test_d_direct_local_anchor_is_never_changed_by_interpolation(
    tmp_path: Path,
    stage7_assets: tuple[BaselineSnapshot, ActionLibrary],
) -> None:
    session = make_session(tmp_path, stage7_assets)
    for row, col in QUICK_COORDS:
        save_delta(session, row, col, {})
    save_delta(session, 9, 2, {"001": 17, "002": -9})  # flat index P137

    generated = session.recalculate()
    direct = generated[point_id(9, 2)]

    assert direct["source"] == "DIRECT"
    assert direct["new_pwm"] == session.anchors[point_id(9, 2)]["new_pwm"]
    assert direct["delta_pwm"]["001"] == 17
    assert direct["delta_pwm"]["002"] == -9
    assert generated[point_id(9, 3)]["delta_pwm"]["001"] != 0


def test_e_provenance_and_verification_are_preserved(
    tmp_path: Path,
    stage7_assets: tuple[BaselineSnapshot, ActionLibrary],
) -> None:
    session = make_session(tmp_path, stage7_assets)
    for row, col in QUICK_COORDS:
        save_delta(session, row, col, {})
    generated = session.recalculate()

    assert generated[point_id(7, 7)]["source"] == "DIRECT"
    assert generated[point_id(4, 6)]["source"] == "INTERPOLATED"
    verified = session.verify(4, 6)
    assert verified["source"] == "INTERPOLATED"
    assert verified["verified"] is True
    assert point_id(4, 6) in session.verified_points


def test_f_out_of_range_anchor_is_clamped_and_audited(
    tmp_path: Path,
    stage7_assets: tuple[BaselineSnapshot, ActionLibrary],
) -> None:
    session = make_session(tmp_path, stage7_assets)
    requested = dict(session.baseline.get(0, 0).pwm)
    requested["002"] = 5000

    anchor = session.save_anchor(0, 0, requested)

    assert anchor["new_pwm"]["002"] == session.limits.joint_max[2] == 2450
    assert anchor["clamped"] is True
    assert anchor["clamp_log"]["002"] == {"requested": 5000, "applied": 2450}
    assert set(anchor) >= {
        "point_id",
        "board_row",
        "board_col",
        "baseline_pwm",
        "new_pwm",
        "delta_pwm",
        "source",
        "timestamp",
        "calibration_session_id",
    }


def test_session_round_trip_keeps_candidate_and_hash(
    tmp_path: Path,
    stage7_assets: tuple[BaselineSnapshot, ActionLibrary],
) -> None:
    session = make_session(tmp_path, stage7_assets)
    for row, col in QUICK_COORDS:
        save_delta(session, row, col, {"003": 3})
    session.recalculate()
    session.verify(4, 6)
    session.save()

    loaded = RapidCalibrationSession.load(
        session.path,
        baseline=session.baseline,
        limits=session.limits,
    )

    assert loaded.baseline.source_sha256 == session.baseline.source_sha256
    assert loaded.generated_points == session.generated_points
    assert loaded.verified_points == session.verified_points
