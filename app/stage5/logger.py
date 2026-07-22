from __future__ import annotations

from datetime import datetime
import json
import logging
from pathlib import Path
from typing import Any

from app.config import PROJECT_ROOT


LOGGER = logging.getLogger(__name__)


class Stage5Logger:
    """Append structured stage-5 events to logs/stage5/ and the root logger."""

    def __init__(self, logs_dir: str | Path | None = None) -> None:
        base = Path(logs_dir) if logs_dir is not None else PROJECT_ROOT / "logs" / "stage5"
        self.logs_dir = base
        self.logs_dir.mkdir(parents=True, exist_ok=True)
        self.session_path = self.logs_dir / f"stage5_{datetime.now():%Y%m%d_%H%M%S}.jsonl"
        self.session_path.touch(exist_ok=True)
        LOGGER.info("STAGE5 LOG %s", self.session_path)

    def log(self, event: str, **payload: Any) -> None:
        record = {
            "time": datetime.now().isoformat(timespec="seconds"),
            "event": event,
            **payload,
        }
        line = json.dumps(record, ensure_ascii=False, default=str)
        with self.session_path.open("a", encoding="utf-8") as handle:
            handle.write(line + "\n")
        LOGGER.info("STAGE5 %s %s", event, line)
