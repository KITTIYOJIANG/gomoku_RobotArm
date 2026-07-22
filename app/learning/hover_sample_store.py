from __future__ import annotations

from datetime import datetime
import hashlib
import json
import logging
from pathlib import Path
from typing import Any, Mapping


LOGGER = logging.getLogger(__name__)


class HoverSampleStore:
    """Append-only verified hover samples for future learning (not used for control)."""

    def __init__(self, path: str | Path) -> None:
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        if not self.path.exists():
            self.path.touch()

    def _fingerprint(self, row: int, col: int, pwm: Mapping[str, int], version: str) -> str:
        payload = f"{row},{col}|{version}|" + ",".join(
            f"{k}:{int(pwm[k])}" for k in sorted(pwm)
        )
        return hashlib.sha1(payload.encode("utf-8")).hexdigest()[:16]

    def existing_fingerprints(self) -> set[str]:
        found: set[str] = set()
        if not self.path.exists():
            return found
        for line in self.path.read_text(encoding="utf-8").splitlines():
            line = line.strip()
            if not line:
                continue
            try:
                obj = json.loads(line)
            except json.JSONDecodeError:
                continue
            fp = obj.get("fingerprint")
            if fp:
                found.add(str(fp))
        return found

    def add_sample(
        self,
        *,
        row: int,
        col: int,
        pwm: Mapping[str, int],
        verified_runs: int,
        safe_return_completed: bool,
        emergency_stop: bool,
        calibration_version: str,
    ) -> dict[str, Any] | None:
        if emergency_stop or not safe_return_completed:
            LOGGER.info("[LEARNING][SAMPLE_SKIP] unsafe result")
            return None
        joints = ["000", "001", "002", "003", "004"]
        target = [int(pwm[j]) for j in joints]
        fp = self._fingerprint(row, col, {j: int(pwm[j]) for j in joints}, calibration_version)
        if fp in self.existing_fingerprints():
            LOGGER.info("[LEARNING][SAMPLE_DUP] fingerprint=%s", fp)
            return None
        stamp = datetime.now().strftime("%Y%m%d")
        sample_id = f"P{row}{col}_{stamp}_{fp[:3].upper()}"
        record = {
            "sample_id": sample_id,
            "fingerprint": fp,
            "row": int(row),
            "col": int(col),
            "input": [float(row), float(col)],
            "target_pwm": target,
            "joint_ids": joints,
            "pose_type": "TARGET_ABOVE",
            "source": "manual_calibration",
            "calibrated": True,
            "verified_runs": int(verified_runs),
            "safe_return_completed": True,
            "emergency_stop": False,
            "created_at": datetime.now().isoformat(timespec="seconds"),
            "calibration_version": calibration_version,
        }
        with self.path.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(record, ensure_ascii=False) + "\n")
        LOGGER.info("[LEARNING][SAMPLE_ADDED] sample_id=%s input=[%s,%s]", sample_id, row, col)
        return record

    def count(self) -> int:
        if not self.path.exists():
            return 0
        return sum(1 for line in self.path.read_text(encoding="utf-8").splitlines() if line.strip())
