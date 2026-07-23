from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
import json
import logging
from pathlib import Path
from typing import Any

import numpy as np
import torch
from torch import nn
from torch.utils.data import DataLoader

from app.learning.hover_dataset import VerifiedHoverPoseDataset
from app.learning.hover_model import HoverPoseNet
from app.learning.hover_normalizer import HoverNormalizer


LOGGER = logging.getLogger(__name__)


@dataclass
class TrainConfig:
    epochs: int = 500
    lr: float = 1e-2
    batch_size: int = 8
    hidden: int = 64
    smoke_test: bool = False
    latest_per_coordinate: bool = True
    min_verified_runs: int = 1
    seed: int = 42


@dataclass
class TrainResult:
    model_path: Path
    normalizer_path: Path
    manifest_path: Path
    final_loss: float
    epochs_run: int
    n_samples: int
    n_unique_coords: int
    generalization_valid: bool
    parameter_count: int


def train_hover_pose(
    dataset_path: str | Path,
    output_dir: str | Path,
    config: TrainConfig | None = None,
) -> TrainResult:
    cfg = config or TrainConfig()
    if cfg.smoke_test:
        cfg = TrainConfig(
            epochs=min(200, cfg.epochs),
            lr=cfg.lr,
            batch_size=1,
            hidden=cfg.hidden,
            smoke_test=True,
            latest_per_coordinate=cfg.latest_per_coordinate,
            min_verified_runs=cfg.min_verified_runs,
            seed=cfg.seed,
        )

    torch.manual_seed(cfg.seed)
    np.random.seed(cfg.seed)

    dataset = VerifiedHoverPoseDataset(
        dataset_path,
        min_verified_runs=cfg.min_verified_runs,
        latest_per_coordinate=cfg.latest_per_coordinate,
    )
    if len(dataset) == 0:
        raise RuntimeError("No verified samples available for training")

    x_np, y_np = dataset.arrays()
    normalizer = HoverNormalizer.fit(x_np, y_np)
    x_n = normalizer.transform_x(x_np).astype(np.float32)
    y_n = normalizer.transform_y(y_np).astype(np.float32)

    xs = torch.from_numpy(x_n)
    ys = torch.from_numpy(y_n)
    loader = DataLoader(
        list(zip(xs, ys)),
        batch_size=min(cfg.batch_size, len(dataset)),
        shuffle=True,
    )

    model = HoverPoseNet(hidden=cfg.hidden)
    opt = torch.optim.Adam(model.parameters(), lr=cfg.lr)
    loss_fn = nn.MSELoss()

    model.train()
    last_loss = float("inf")
    for epoch in range(cfg.epochs):
        epoch_loss = 0.0
        n_batches = 0
        for bx, by in loader:
            opt.zero_grad()
            pred = model(bx)
            loss = loss_fn(pred, by)
            loss.backward()
            opt.step()
            epoch_loss += float(loss.item())
            n_batches += 1
        last_loss = epoch_loss / max(1, n_batches)
        if cfg.smoke_test and epoch % 50 == 0:
            LOGGER.info("smoke epoch=%s loss=%.6f", epoch, last_loss)

    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    model_path = out / f"hover_pose_net_{stamp}.pt"
    normalizer_path = out / f"hover_normalizer_{stamp}.json"
    manifest_path = out / f"hover_train_manifest_{stamp}.json"
    latest_model = out / "hover_pose_net_latest.pt"
    latest_norm = out / "hover_normalizer_latest.json"
    latest_manifest = out / "hover_train_manifest_latest.json"

    torch.save({"state_dict": model.state_dict(), "hidden": cfg.hidden}, model_path)
    torch.save({"state_dict": model.state_dict(), "hidden": cfg.hidden}, latest_model)
    normalizer.save(normalizer_path)
    normalizer.save(latest_norm)

    coords = dataset.unique_coordinates()
    manifest: dict[str, Any] = {
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "dataset": dataset.manifest(),
        "dataset_path": str(Path(dataset_path).resolve()),
        "model_path": str(model_path.resolve()),
        "normalizer_path": str(normalizer_path.resolve()),
        "epochs": cfg.epochs,
        "final_loss": last_loss,
        "parameter_count": model.count_parameters(),
        "generalization_valid": len(coords) >= 3,
        "note": "Shadow model only. Never used for live arm control.",
        "MODEL_LIVE_CONTROL_ENABLED": False,
        "sample_ids": dataset.sample_ids(),
    }
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    latest_manifest.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")

    return TrainResult(
        model_path=model_path,
        normalizer_path=normalizer_path,
        manifest_path=manifest_path,
        final_loss=last_loss,
        epochs_run=cfg.epochs,
        n_samples=len(dataset),
        n_unique_coords=len(coords),
        generalization_valid=len(coords) >= 3,
        parameter_count=model.count_parameters(),
    )
