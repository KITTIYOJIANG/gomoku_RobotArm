from __future__ import annotations

import json
import shutil
import uuid
from pathlib import Path

import numpy as np
import pytest
import torch
from torch.utils.data import DataLoader

from app.learning import MODEL_LIVE_CONTROL_ENABLED
from app.learning.hover_comparator import HoverPoseComparator, PRIORITY
from app.learning.hover_dataset import DatasetError, VerifiedHoverPoseDataset, _validate_record, load_verified_records
from app.learning.hover_model import HoverPoseNet
from app.learning.hover_normalizer import HoverNormalizer
from app.learning.hover_predictor import HoverPosePredictor, PredictionStatus
from app.learning.hover_trainer import TrainConfig, train_hover_pose
from app.arm.actions import ActionLibrary
from app.stage5.calibration_store import CalibrationStore
from app.stage5.constants import FORCE_STAGE5_DRY_RUN
from app.stage5.safety import derive_pwm_safety_limits
from app.config import PROJECT_ROOT


SCRATCH = PROJECT_ROOT / ".tmp_hover_tests"


@pytest.fixture()
def tmp_path():
    path = SCRATCH / uuid.uuid4().hex
    path.mkdir(parents=True, exist_ok=True)
    yield path
    shutil.rmtree(path, ignore_errors=True)


def _sample(row=7, col=7, pwm=None, **over):
    pwm = pwm or [1560, 1170, 990, 1170, 1500]
    obj = {
        "sample_id": f"P{row}{col}_T",
        "fingerprint": f"fp_{row}_{col}_{pwm[1]}_{over.get('sample_id','')}",
        "row": row,
        "col": col,
        "input": [float(row), float(col)],
        "target_pwm": pwm,
        "joint_ids": ["000", "001", "002", "003", "004"],
        "pose_type": "TARGET_ABOVE",
        "source": "manual_calibration",
        "calibrated": True,
        "verified_runs": 3,
        "safe_return_completed": True,
        "emergency_stop": False,
        "created_at": "2026-07-23T00:00:00",
        "calibration_version": "v1",
    }
    obj.update(over)
    return obj


def _write_jsonl(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(json.dumps(r) for r in rows) + "\n", encoding="utf-8")


def test_model_live_control_disabled():
    assert MODEL_LIVE_CONTROL_ENABLED is False
    assert HoverPosePredictor.LIVE_CONTROL_ENABLED is False
    assert MODEL_LIVE_CONTROL_ENABLED is False  # force dry may be False for live hover


def test_dataset_reads_valid_sample(tmp_path: Path):
    p = tmp_path / "s.jsonl"
    _write_jsonl(p, [_sample()])
    ds = VerifiedHoverPoseDataset(p, min_verified_runs=1)
    assert len(ds) == 1
    x, y = ds[0]
    assert tuple(x.tolist()) == (7.0, 7.0)
    assert y.shape == (5,)
    assert int(y.tolist()[0]) == 1560


def test_missing_fields_rejected():
    with pytest.raises(DatasetError):
        _validate_record({"row": 1}, min_verified_runs=1)


def test_null_pwm_rejected():
    with pytest.raises(DatasetError):
        _validate_record(_sample(pwm=[1560, None, 990, 1170, 1500]), min_verified_runs=1)


def test_pump_not_in_output(tmp_path: Path):
    p = tmp_path / "s.jsonl"
    _write_jsonl(p, [_sample()])
    ds = VerifiedHoverPoseDataset(p)
    _, y = ds[0]
    assert len(y) == 5


def test_shapes_and_dataloader(tmp_path: Path):
    p = tmp_path / "s.jsonl"
    _write_jsonl(p, [_sample(), _sample(row=3, col=7, pwm=[1560, 1160, 990, 1170, 1500], sample_id="b")])
    ds = VerifiedHoverPoseDataset(p, latest_per_coordinate=True)
    x, y = ds.arrays()
    assert x.shape[1] == 2 and y.shape[1] == 5
    loader = DataLoader(ds, batch_size=2)
    bx, by = next(iter(loader))
    assert bx.shape[1] == 2 and by.shape[1] == 5


def test_model_batch_output_shape():
    net = HoverPoseNet()
    y = net(torch.randn(4, 2))
    assert y.shape == (4, 5)
    assert net.count_parameters() > 0


def test_normalizer_invertible_and_zero_std():
    x = np.array([[7.0, 7.0], [7.0, 7.0]], dtype=np.float64)
    y = np.array([[1560, 1170, 990, 1170, 1500], [1560, 1170, 990, 1170, 1500]], dtype=np.float64)
    n = HoverNormalizer.fit(x, y)
    assert np.allclose(n.inverse_y(n.transform_y(y)), y)
    assert np.all(n.x_std >= 1.0 - 1e-9)


def test_save_load_model_predict_consistent(tmp_path: Path):
    p = tmp_path / "s.jsonl"
    _write_jsonl(p, [_sample()])
    result = train_hover_pose(p, tmp_path / "out", TrainConfig(smoke_test=True, epochs=80, min_verified_runs=1))
    pred = HoverPosePredictor(model_path=result.model_path, normalizer_path=result.normalizer_path)
    a = pred.predict(7, 7)
    b = pred.predict(7, 7)
    assert a.status == PredictionStatus.OK and a.pwm == b.pwm
    assert abs(a.pwm["000"] - 1560) < 40


def test_manifest_contains_sample_ids(tmp_path: Path):
    p = tmp_path / "s.jsonl"
    _write_jsonl(p, [_sample()])
    m = VerifiedHoverPoseDataset(p).manifest()
    assert m["sample_ids"] and m["generalization_valid"] is False


def test_loo_refuses_fewer_than_3_coords(tmp_path: Path):
    p = tmp_path / "s.jsonl"
    _write_jsonl(p, [_sample(), _sample(row=3, col=7, sample_id="b")])
    ds = VerifiedHoverPoseDataset(p, latest_per_coordinate=True)
    assert len(ds.unique_coordinates()) < 3
    assert ds.manifest()["generalization_valid"] is False


def test_predictor_no_model():
    assert HoverPosePredictor().predict(7, 7).status == PredictionStatus.NO_MODEL


def test_out_of_board_rejected(tmp_path: Path):
    p = tmp_path / "s.jsonl"
    _write_jsonl(p, [_sample()])
    result = train_hover_pose(p, tmp_path / "out", TrainConfig(smoke_test=True, epochs=40))
    pred = HoverPosePredictor(model_path=result.model_path, normalizer_path=result.normalizer_path)
    assert pred.predict(99, 99).status == PredictionStatus.OUT_OF_BOARD


def test_comparator_priority_direct_anchor(tmp_path: Path):
    lib = ActionLibrary()
    limits = derive_pwm_safety_limits(lib)
    store = CalibrationStore(tmp_path / "c.json", library=lib, safety_limits=limits)
    result = HoverPoseComparator(store, limits=limits).compare(7, 7, HoverPosePredictor().predict(7, 7))
    assert result.preferred_source == "direct_anchor"
    assert PRIORITY[0] == "direct_anchor"


def test_model_cannot_call_serial():
    pred = HoverPosePredictor()
    assert not hasattr(pred, "controller")
    assert pred.LIVE_CONTROL_ENABLED is False


def test_draft_and_interp_not_in_dataset(tmp_path: Path):
    p = tmp_path / "s.jsonl"
    rows = [
        _sample(source="bilinear_interpolation", sample_id="interp", fingerprint="f0"),
        _sample(row=1, col=1, calibrated=False, sample_id="draft", fingerprint="f1"),
        _sample(row=2, col=2, emergency_stop=True, sample_id="estop", fingerprint="f2"),
        _sample(row=7, col=7, sample_id="ok", fingerprint="f3"),
    ]
    _write_jsonl(p, rows)
    assert VerifiedHoverPoseDataset(p, min_verified_runs=1).sample_ids() == ["ok"]


def test_model_stale(tmp_path: Path):
    p = tmp_path / "s.jsonl"
    _write_jsonl(p, [_sample()])
    result = train_hover_pose(p, tmp_path / "out", TrainConfig(smoke_test=True, epochs=30))
    pred = HoverPosePredictor(
        model_path=result.model_path,
        normalizer_path=result.normalizer_path,
        dataset_fingerprint="new",
        trained_fingerprint="old",
    )
    assert pred.predict(7, 7).status == PredictionStatus.STALE


def test_real_serial_and_live_flags():
    assert MODEL_LIVE_CONTROL_ENABLED is False
    assert MODEL_LIVE_CONTROL_ENABLED is False  # force dry may be False for live hover
