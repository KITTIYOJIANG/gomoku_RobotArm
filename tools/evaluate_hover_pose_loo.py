from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import torch
from torch import nn

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from app.learning.hover_dataset import load_verified_records
from app.learning.hover_model import HoverPoseNet
from app.learning.hover_normalizer import HoverNormalizer


def main() -> int:
    p = argparse.ArgumentParser(description="Leave-one-coordinate-out eval")
    p.add_argument("--dataset", default="datasets/hover_pose/verified_samples.jsonl")
    p.add_argument("--epochs", type=int, default=300)
    p.add_argument("--min-coords", type=int, default=3)
    args = p.parse_args()
    samples = load_verified_records(ROOT / args.dataset, min_verified_runs=1, latest_per_coordinate=True)
    coords = sorted({(s.row, s.col) for s in samples})
    if len(coords) < args.min_coords:
        print(f"LOO refused: unique coords={len(coords)} < {args.min_coords}")
        print("generalization_valid=false")
        return 2
    folds = []
    for held in coords:
        train = [s for s in samples if (s.row, s.col) != held]
        test = [s for s in samples if (s.row, s.col) == held]
        x_tr = np.stack([s.x for s in train])
        y_tr = np.stack([s.y for s in train])
        norm = HoverNormalizer.fit(x_tr, y_tr)
        model = HoverPoseNet()
        opt = torch.optim.Adam(model.parameters(), lr=1e-2)
        loss_fn = nn.MSELoss()
        xt = torch.from_numpy(norm.transform_x(x_tr).astype(np.float32))
        yt = torch.from_numpy(norm.transform_y(y_tr).astype(np.float32))
        model.train()
        for _ in range(args.epochs):
            opt.zero_grad()
            loss = loss_fn(model(xt), yt)
            loss.backward()
            opt.step()
        model.eval()
        with torch.no_grad():
            for s in test:
                xn = torch.from_numpy(norm.transform_x(s.x.reshape(1, 2)).astype(np.float32))
                pred = norm.inverse_y(model(xn).numpy())[0]
                mae = float(np.mean(np.abs(pred - s.y)))
                folds.append({"coord": list(held), "mae": mae})
                print(f"holdout P{held} mae={mae:.2f}")
    mean_mae = float(np.mean([f["mae"] for f in folds])) if folds else None
    print(json.dumps({"mean_mae": mean_mae, "folds": folds, "generalization_valid": True}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
