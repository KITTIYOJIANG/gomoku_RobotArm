from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Any, Iterable, Sequence

import numpy as np

try:
    import torch
    from torch.utils.data import Dataset
except ImportError:  # pragma: no cover
    torch = None
    Dataset = object  # type: ignore[misc,assignment]


JOINT_IDS = ("000", "001", "002", "003", "004")


class DatasetError(ValueError):
    pass


@dataclass(frozen=True)
class HoverSample:
    sample_id: str
    row: int
    col: int
    target_pwm: tuple[int, int, int, int, int]
    verified_runs: int
    fingerprint: str
    calibration_version: str
    raw: dict[str, Any]

    @property
    def x(self) -> np.ndarray:
        return np.asarray([float(self.row), float(self.col)], dtype=np.float32)

    @property
    def y(self) -> np.ndarray:
        return np.asarray(self.target_pwm, dtype=np.float32)


def _validate_record(obj: dict[str, Any], *, min_verified_runs: int) -> HoverSample:
    required = {
        "sample_id",
        "row",
        "col",
        "target_pwm",
        "joint_ids",
        "pose_type",
        "source",
        "calibrated",
        "verified_runs",
        "safe_return_completed",
        "emergency_stop",
    }
    missing = required - set(obj)
    if missing:
        raise DatasetError(f"missing fields: {sorted(missing)}")
    if not bool(obj.get("calibrated")):
        raise DatasetError("calibrated must be true")
    if int(obj.get("verified_runs", 0)) < min_verified_runs:
        raise DatasetError("verified_runs below threshold")
    if not bool(obj.get("safe_return_completed")):
        raise DatasetError("safe_return_completed must be true")
    if bool(obj.get("emergency_stop")):
        raise DatasetError("emergency_stop samples excluded")
    if str(obj.get("pose_type")) != "TARGET_ABOVE":
        raise DatasetError("pose_type must be TARGET_ABOVE")
    if str(obj.get("source")) != "manual_calibration":
        raise DatasetError("source must be manual_calibration")
    joints = list(obj.get("joint_ids") or [])
    if joints != list(JOINT_IDS):
        raise DatasetError(f"joint_ids must be {list(JOINT_IDS)}, got {joints}")
    pwm = obj.get("target_pwm")
    if not isinstance(pwm, (list, tuple)) or len(pwm) != 5:
        raise DatasetError("target_pwm must be length-5 list")
    if any(v is None for v in pwm):
        raise DatasetError("null PWM rejected")
    # pump channel must never appear as 6th output
    if len(pwm) != 5:
        raise DatasetError("pump joint must not enter target_pwm")
    values = tuple(int(v) for v in pwm)
    return HoverSample(
        sample_id=str(obj["sample_id"]),
        row=int(obj["row"]),
        col=int(obj["col"]),
        target_pwm=values,  # type: ignore[arg-type]
        verified_runs=int(obj["verified_runs"]),
        fingerprint=str(obj.get("fingerprint", "")),
        calibration_version=str(obj.get("calibration_version", "")),
        raw=obj,
    )


def load_verified_records(
    path: str | Path,
    *,
    min_verified_runs: int = 1,
    latest_per_coordinate: bool = False,
) -> list[HoverSample]:
    path = Path(path)
    if not path.exists():
        return []
    samples: list[HoverSample] = []
    for line_no, line in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
        line = line.strip()
        if not line:
            continue
        try:
            obj = json.loads(line)
        except json.JSONDecodeError as exc:
            raise DatasetError(f"line {line_no}: invalid JSON") from exc
        try:
            samples.append(_validate_record(obj, min_verified_runs=min_verified_runs))
        except DatasetError:
            # Skip records that fail filters (draft/unsafe/etc.)
            continue
    if latest_per_coordinate:
        best: dict[tuple[int, int], HoverSample] = {}
        for s in samples:
            key = (s.row, s.col)
            prev = best.get(key)
            if prev is None or s.raw.get("created_at", "") >= prev.raw.get("created_at", ""):
                best[key] = s
        samples = list(best.values())
    return samples


class VerifiedHoverPoseDataset(Dataset):
    """PyTorch dataset: x=[row,col], y=pwm000..004. Never includes pump 005."""

    def __init__(
        self,
        path: str | Path,
        *,
        min_verified_runs: int = 1,
        latest_per_coordinate: bool = False,
        samples: Sequence[HoverSample] | None = None,
    ) -> None:
        if samples is None:
            self.samples = load_verified_records(
                path,
                min_verified_runs=min_verified_runs,
                latest_per_coordinate=latest_per_coordinate,
            )
        else:
            self.samples = list(samples)
        self.path = Path(path)
        self.min_verified_runs = int(min_verified_runs)
        self.latest_per_coordinate = bool(latest_per_coordinate)

    def __len__(self) -> int:
        return len(self.samples)

    def __getitem__(self, index: int):
        sample = self.samples[index]
        x = sample.x
        y = sample.y
        if torch is None:
            return x, y
        return torch.from_numpy(x), torch.from_numpy(y)

    def arrays(self) -> tuple[np.ndarray, np.ndarray]:
        if not self.samples:
            return np.zeros((0, 2), dtype=np.float32), np.zeros((0, 5), dtype=np.float32)
        x = np.stack([s.x for s in self.samples], axis=0)
        y = np.stack([s.y for s in self.samples], axis=0)
        return x, y

    def unique_coordinates(self) -> set[tuple[int, int]]:
        return {(s.row, s.col) for s in self.samples}

    def sample_ids(self) -> list[str]:
        return [s.sample_id for s in self.samples]

    def manifest(self) -> dict[str, Any]:
        coords = sorted(self.unique_coordinates())
        return {
            "path": str(self.path),
            "n_samples": len(self.samples),
            "n_unique_coords": len(coords),
            "coordinates": [list(c) for c in coords],
            "sample_ids": self.sample_ids(),
            "min_verified_runs": self.min_verified_runs,
            "latest_per_coordinate": self.latest_per_coordinate,
            "generalization_valid": len(coords) >= 3,
        }
