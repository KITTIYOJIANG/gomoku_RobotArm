from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from app.learning.hover_trainer import TrainConfig, train_hover_pose


def main() -> int:
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    p = argparse.ArgumentParser(description="Train shadow HoverPoseNet offline")
    p.add_argument("--dataset", default="datasets/hover_pose/verified_samples.jsonl")
    p.add_argument("--output", default="models/hover_pose")
    p.add_argument("--epochs", type=int, default=500)
    p.add_argument("--smoke-test", action="store_true")
    args = p.parse_args()
    result = train_hover_pose(
        ROOT / args.dataset,
        ROOT / args.output,
        TrainConfig(epochs=args.epochs, smoke_test=args.smoke_test, latest_per_coordinate=True),
    )
    print("model", result.model_path)
    print("normalizer", result.normalizer_path)
    print("manifest", result.manifest_path)
    print("final_loss", result.final_loss)
    print("n_samples", result.n_samples)
    print("n_unique_coords", result.n_unique_coords)
    print("generalization_valid", result.generalization_valid)
    print("parameters", result.parameter_count)
    print("MODEL_LIVE_CONTROL_ENABLED false")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
