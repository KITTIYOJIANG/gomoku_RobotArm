from __future__ import annotations

from copy import deepcopy
from datetime import datetime
import json
import logging
from pathlib import Path
from typing import Any

from app.stage5.constants import CROSS_ANCHORS, SPATIAL_JOINTS, anchor_key


LOGGER = logging.getLogger(__name__)


class CandidateStoreError(ValueError):
    pass


class CandidateStore:
    """Atomic draft store for cross-anchor candidate PWM values."""

    def __init__(self, path: str | Path) -> None:
        self.path = Path(path)
        self._data: dict[str, Any] = {"version": "1.0", "anchors": {}}
        if self.path.exists():
            self.reload()
        else:
            self._bootstrap()

    def _bootstrap(self) -> None:
        anchors: dict[str, Any] = {}
        for row, col, label, _cn in CROSS_ANCHORS:
            key = anchor_key(row, col)
            anchors[key] = self._empty_anchor(row, col, label)
        self._data = {"version": "1.0", "anchors": anchors}
        self.save()

    @staticmethod
    def _empty_anchor(row: int, col: int, label: str) -> dict[str, Any]:
        return {
            "row": int(row),
            "col": int(col),
            "label": label,
            "reference_anchor": "7,7",
            "candidate_pwm": {jid: None for jid in SPATIAL_JOINTS},
            "status": "EMPTY",
            "user_verified": False,
            "verified_runs": 0,
            "last_test_result": None,
            "safe_return_completed": False,
            "emergency_stop": False,
            "notes": "",
            "updated_at": "",
            "history": [],
        }

    def reload(self) -> None:
        try:
            raw = json.loads(self.path.read_text(encoding="utf-8"))
        except Exception as exc:
            raise CandidateStoreError(f"Failed to load draft JSON: {exc}") from exc
        if not isinstance(raw, dict) or "anchors" not in raw:
            raise CandidateStoreError("Invalid draft schema: missing anchors")
        anchors = raw.get("anchors") or {}
        if not isinstance(anchors, dict):
            raise CandidateStoreError("Invalid draft schema: anchors not object")
        for key, value in anchors.items():
            if not isinstance(value, dict):
                raise CandidateStoreError(f"Invalid draft entry {key}")
            pwm = value.get("candidate_pwm")
            if pwm is not None and not isinstance(pwm, dict):
                raise CandidateStoreError(f"Invalid candidate_pwm for {key}")
        self._data = raw
        # Ensure all four cross anchors exist.
        for row, col, label, _cn in CROSS_ANCHORS:
            key = anchor_key(row, col)
            if key not in self._data["anchors"]:
                self._data["anchors"][key] = self._empty_anchor(row, col, label)

    def save(self) -> Path:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_suffix(self.path.suffix + ".tmp")
        payload = json.dumps(self._data, indent=2, ensure_ascii=False) + "\n"
        tmp.write_text(payload, encoding="utf-8")
        tmp.replace(self.path)
        return self.path

    def get(self, row: int, col: int) -> dict[str, Any]:
        key = anchor_key(row, col)
        if key not in self._data["anchors"]:
            raise KeyError(key)
        return deepcopy(self._data["anchors"][key])

    def list_status(self) -> dict[str, str]:
        return {
            key: str(value.get("status", "EMPTY"))
            for key, value in self._data.get("anchors", {}).items()
        }

    def set_candidate_pwm(
        self,
        row: int,
        col: int,
        pwm: dict[str, int],
        *,
        status: str | None = None,
        notes: str = "",
    ) -> dict[str, Any]:
        key = anchor_key(row, col)
        entry = self._data["anchors"].setdefault(
            key, self._empty_anchor(row, col, f"P{row}{col}")
        )
        normalized = {jid: int(pwm[jid]) for jid in SPATIAL_JOINTS if jid in pwm}
        if len(normalized) != 5:
            raise CandidateStoreError("candidate_pwm must include joints 000..004")
        entry["candidate_pwm"] = normalized
        entry["status"] = status or "DRAFT"
        entry["notes"] = notes or entry.get("notes", "")
        entry["updated_at"] = datetime.now().isoformat(timespec="seconds")
        entry.setdefault("history", []).append(
            {"at": entry["updated_at"], "pwm": dict(normalized), "status": entry["status"]}
        )
        # Keep history bounded
        entry["history"] = entry["history"][-50:]
        self.save()
        return deepcopy(entry)

    def record_test_result(
        self,
        row: int,
        col: int,
        *,
        result: str,
        safe_return_completed: bool,
        emergency_stop: bool,
        increment_verified: bool,
    ) -> dict[str, Any]:
        key = anchor_key(row, col)
        entry = self._data["anchors"][key]
        entry["last_test_result"] = result
        entry["safe_return_completed"] = bool(safe_return_completed)
        entry["emergency_stop"] = bool(emergency_stop)
        if increment_verified:
            entry["verified_runs"] = int(entry.get("verified_runs", 0)) + 1
            entry["status"] = "VERIFIED_ONCE" if entry["verified_runs"] < 3 else entry.get("status", "VERIFIED_ONCE")
        entry["updated_at"] = datetime.now().isoformat(timespec="seconds")
        self.save()
        return deepcopy(entry)

    def mark_completed(self, row: int, col: int) -> dict[str, Any]:
        key = anchor_key(row, col)
        entry = self._data["anchors"][key]
        entry["status"] = "COMPLETED"
        entry["user_verified"] = True
        entry["updated_at"] = datetime.now().isoformat(timespec="seconds")
        self.save()
        return deepcopy(entry)
