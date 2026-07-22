
from __future__ import annotations

import argparse
import json
from collections import Counter
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description="Inspect verified hover samples")
    parser.add_argument(
        "--path",
        default="datasets/hover_pose/verified_samples.jsonl",
        help="jsonl sample path",
    )
    args = parser.parse_args()
    path = Path(args.path)
    if not path.exists():
        print(f"missing: {path}")
        return 1
    rows = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.strip():
            rows.append(json.loads(line))
    print(f"samples: {len(rows)}")
    points = Counter((r.get("row"), r.get("col")) for r in rows)
    print("point distribution:", dict(points))
    fps = [r.get("fingerprint") for r in rows]
    print("unique fingerprints:", len(set(fps)), "duplicates:", len(fps) - len(set(fps)))
    required = {"sample_id", "row", "col", "input", "target_pwm", "joint_ids", "verified_runs"}
    incomplete = [r.get("sample_id") for r in rows if not required.issubset(r)]
    print("incomplete records:", incomplete)
    for r in rows:
        print(
            f"- {r.get('sample_id')} P({r.get('row')},{r.get('col')}) "
            f"pwm={r.get('target_pwm')} runs={r.get('verified_runs')}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
