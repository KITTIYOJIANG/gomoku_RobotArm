from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Any

import numpy as np
import torch

from app.learning.hover_model import HoverPoseNet
from app.learning.hover_normalizer import HoverNormalizer
from app.stage5.safety import PwmSafetyLimits, validate_spatial_pwm


class PredictionStatus(str, Enum):
    OK = "OK"
    NO_MODEL = "NO_MODEL"
    OUT_OF_BOARD = "OUT_OF_BOARD"
    ENVELOPE_FAIL = "ENVELOPE_FAIL"
    STALE = "MODEL_STALE"
    DISABLED = "DISABLED"


@dataclass(frozen=True)
class ShadowPrediction:
    status: PredictionStatus
    row: int
    col: int
    pwm: dict[str, int] | None
    raw: np.ndarray | None
    message: str
    source: str = "pytorch_shadow"

    def to_dict(self) -> dict[str, Any]:
        return {
            "status": self.status.value,
            "row": self.row,
            "col": self.col,
            "pwm": self.pwm,
            "raw": None if self.raw is None else self.raw.tolist(),
            "message": self.message,
            "source": self.source,
            "MODEL_LIVE_CONTROL_ENABLED": False,
        }


class HoverPosePredictor:
    """Shadow predictor only. Never sends serial commands or registers runtime actions."""

    LIVE_CONTROL_ENABLED = False

    def __init__(
        self,
        *,
        model_path: str | Path | None = None,
        normalizer_path: str | Path | None = None,
        board_size: int = 15,
        limits: PwmSafetyLimits | None = None,
        dataset_fingerprint: str | None = None,
        trained_fingerprint: str | None = None,
    ) -> None:
        self.board_size = int(board_size)
        self.limits = limits
        self.dataset_fingerprint = dataset_fingerprint
        self.trained_fingerprint = trained_fingerprint
        self.model: HoverPoseNet | None = None
        self.normalizer: HoverNormalizer | None = None
        self.model_path = Path(model_path) if model_path else None
        self.normalizer_path = Path(normalizer_path) if normalizer_path else None
        if self.model_path and self.normalizer_path and self.model_path.exists() and self.normalizer_path.exists():
            self.load(self.model_path, self.normalizer_path)

    def load(self, model_path: str | Path, normalizer_path: str | Path) -> None:
        payload = torch.load(Path(model_path), map_location="cpu")
        hidden = int(payload.get("hidden", 64))
        model = HoverPoseNet(hidden=hidden)
        model.load_state_dict(payload["state_dict"])
        model.eval()
        self.model = model
        self.normalizer = HoverNormalizer.load(normalizer_path)
        self.model_path = Path(model_path)
        self.normalizer_path = Path(normalizer_path)

    def is_stale(self) -> bool:
        if self.dataset_fingerprint is None or self.trained_fingerprint is None:
            return False
        return self.dataset_fingerprint != self.trained_fingerprint

    def predict(self, row: int, col: int) -> ShadowPrediction:
        if self.model is None or self.normalizer is None:
            return ShadowPrediction(
                PredictionStatus.NO_MODEL,
                int(row),
                int(col),
                None,
                None,
                "No trained model loaded",
            )
        if self.is_stale():
            return ShadowPrediction(
                PredictionStatus.STALE,
                int(row),
                int(col),
                None,
                None,
                "Dataset changed since training (MODEL_STALE)",
            )
        if not (0 <= int(row) < self.board_size and 0 <= int(col) < self.board_size):
            return ShadowPrediction(
                PredictionStatus.OUT_OF_BOARD,
                int(row),
                int(col),
                None,
                None,
                "row/col outside board",
            )
        x = np.asarray([[float(row), float(col)]], dtype=np.float32)
        x_n = self.normalizer.transform_x(x).astype(np.float32)
        with torch.no_grad():
            y_n = self.model(torch.from_numpy(x_n)).cpu().numpy()
        y = self.normalizer.inverse_y(y_n)[0]
        pwm_int = {f"{i:03d}": int(round(float(y[i]))) for i in range(5)}
        if self.limits is not None:
            errors = validate_spatial_pwm({i: pwm_int[f"{i:03d}"] for i in range(5)}, self.limits)
            if errors:
                return ShadowPrediction(
                    PredictionStatus.ENVELOPE_FAIL,
                    int(row),
                    int(col),
                    pwm_int,
                    y,
                    "; ".join(errors),
                )
        return ShadowPrediction(
            PredictionStatus.OK,
            int(row),
            int(col),
            pwm_int,
            y,
            "shadow prediction only",
        )
