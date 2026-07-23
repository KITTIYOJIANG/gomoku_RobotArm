"""Hover pose learning layer (shadow only; never controls the arm)."""

MODEL_LIVE_CONTROL_ENABLED = False

from .hover_dataset import VerifiedHoverPoseDataset
from .hover_model import HoverPoseNet
from .hover_normalizer import HoverNormalizer
from .hover_predictor import HoverPosePredictor, PredictionStatus
from .hover_comparator import HoverPoseComparator

__all__ = [
    "MODEL_LIVE_CONTROL_ENABLED",
    "VerifiedHoverPoseDataset",
    "HoverPoseNet",
    "HoverNormalizer",
    "HoverPosePredictor",
    "PredictionStatus",
    "HoverPoseComparator",
]
