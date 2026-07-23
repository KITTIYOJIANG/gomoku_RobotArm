from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from app.learning.hover_sample_store import HoverSampleStore


def main() -> int:
    parser = argparse.ArgumentParser(description="Explicit sync calibration anchors to verified samples")
    parser.add_argument("--calibration", default="calibration/stage5_board_calibration.json")
    parser.add_argument("--samples", default="datasets/hover_pose/verified_samples.jsonl")
    parser.add_argument("--apply", action="store_true", help="Write samples; default is preview only")
    args = parser.parse_args()

    calib = json.loads((ROOT / args.calibration).read_text(encoding="utf-8"))
    store = HoverSampleStore(ROOT / args.samples)
    joints = ["000", "001", "002", "003", "004"]
    candidates = []
    for anchor in (calib.get("anchors") or {}).values():
        if not bool(anchor.get("calibrated")):
            continue
        pwm = anchor.get("pwm") or {}
        if any(pwm.get(j) is None for j in joints):
            continue
        candidates.append(anchor)

    print(f"candidates={len(candidates)} existing_samples={store.count()}")
    if not args.apply:
        print("PREVIEW only. Re-run with --apply to write.")
        for item in candidates:
            vals = [int(item["pwm"][j]) for j in joints]
            print(f"  would sync P({item['row']},{item['col']}) pwm={vals}")
        return 0

    added = 0
    for item in candidates:
        pwm = {j: int(item["pwm"][j]) for j in joints}
        version = f"calib_sync_{datetime.now():%Y%m%d}"
        rec = store.add_sample(
            row=int(item["row"]),
            col=int(item["col"]),
            pwm=pwm,
            verified_runs=max(int(item.get("verified_runs", 1)), 1),
            safe_return_completed=True,
            emergency_stop=False,
            calibration_version=version,
        )
        if rec is None:
            print(f"dup/skip P({item['row']},{item['col']})")
        else:
            added += 1
            print("added", rec["sample_id"])
    print(f"done added={added} total={store.count()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
