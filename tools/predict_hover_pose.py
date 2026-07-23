from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from app.learning.hover_predictor import HoverPosePredictor


def main() -> int:
    p = argparse.ArgumentParser(description="Shadow PWM predict (never controls arm)")
    p.add_argument("--row", type=int, required=True)
    p.add_argument("--col", type=int, required=True)
    p.add_argument("--model", default="models/hover_pose/hover_pose_net_latest.pt")
    p.add_argument("--normalizer", default="models/hover_pose/hover_normalizer_latest.json")
    args = p.parse_args()
    pred = HoverPosePredictor(
        model_path=ROOT / args.model,
        normalizer_path=ROOT / args.normalizer,
    )
    out = pred.predict(args.row, args.col)
    print(json.dumps(out.to_dict(), indent=2, ensure_ascii=False))
    print("MODEL_LIVE_CONTROL_ENABLED=false")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
