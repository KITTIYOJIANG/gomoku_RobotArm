from __future__ import annotations

from dataclasses import asdict, dataclass
import json
from pathlib import Path
from typing import Any

import numpy as np


@dataclass
class HoverNormalizer:
    """Affine normalizer for [row,col] inputs and 5 PWM outputs.

    std=0 is replaced with 1.0 so inverse remains defined.
    """

    x_mean: np.ndarray
    x_std: np.ndarray
    y_mean: np.ndarray
    y_std: np.ndarray

    @classmethod
    def fit(cls, x: np.ndarray, y: np.ndarray) -> "HoverNormalizer":
        x = np.asarray(x, dtype=np.float64).reshape(-1, 2)
        y = np.asarray(y, dtype=np.float64).reshape(-1, 5)
        x_mean = x.mean(axis=0)
        y_mean = y.mean(axis=0)
        x_std = x.std(axis=0)
        y_std = y.std(axis=0)
        x_std = np.where(x_std < 1e-8, 1.0, x_std)
        y_std = np.where(y_std < 1e-8, 1.0, y_std)
        return cls(x_mean=x_mean, x_std=x_std, y_mean=y_mean, y_std=y_std)

    def transform_x(self, x: np.ndarray) -> np.ndarray:
        x = np.asarray(x, dtype=np.float64)
        return (x - self.x_mean) / self.x_std

    def transform_y(self, y: np.ndarray) -> np.ndarray:
        y = np.asarray(y, dtype=np.float64)
        return (y - self.y_mean) / self.y_std

    def inverse_y(self, y_norm: np.ndarray) -> np.ndarray:
        y_norm = np.asarray(y_norm, dtype=np.float64)
        return y_norm * self.y_std + self.y_mean

    def to_dict(self) -> dict[str, Any]:
        return {
            "x_mean": self.x_mean.tolist(),
            "x_std": self.x_std.tolist(),
            "y_mean": self.y_mean.tolist(),
            "y_std": self.y_std.tolist(),
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "HoverNormalizer":
        return cls(
            x_mean=np.asarray(data["x_mean"], dtype=np.float64),
            x_std=np.asarray(data["x_std"], dtype=np.float64),
            y_mean=np.asarray(data["y_mean"], dtype=np.float64),
            y_std=np.asarray(data["y_std"], dtype=np.float64),
        )

    def save(self, path: str | Path) -> Path:
        path = Path(path)
        path.write_text(json.dumps(self.to_dict(), indent=2) + "\n", encoding="utf-8")
        return path

    @classmethod
    def load(cls, path: str | Path) -> "HoverNormalizer":
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        return cls.from_dict(data)
