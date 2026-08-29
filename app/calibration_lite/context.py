from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
import json
from pathlib import Path
from typing import Any

from app.config import PROJECT_ROOT
from app.stage7.settings import Stage7Settings


@dataclass(frozen=True)
class CalibrationSummary:
    date: str
    anchors: int
    generated_points: int
    status: str
    source_path: Path | None
    session_path: Path | None

    @property
    def valid(self) -> bool:
        return self.generated_points == 225 and self.status in {
            "Valid",
            "Committed",
            "Stable baseline",
            "Ready to test",
        }


def _read_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _date(value: object, fallback_path: Path) -> str:
    if value:
        return str(value)[:10]
    return datetime.fromtimestamp(fallback_path.stat().st_mtime).date().isoformat()


def _summary_from_session(path: Path, *, status: str | None = None) -> CalibrationSummary:
    data = _read_json(path)
    generated = len(data.get("generated_points") or {})
    anchors = len(data.get("anchors") or {})
    if status is None:
        if data.get("committed_at"):
            status = "Committed"
        elif generated == 225:
            status = "Ready to test"
        else:
            status = "In progress"
    return CalibrationSummary(
        date=_date(data.get("updated_at") or data.get("created_at"), path),
        anchors=anchors,
        generated_points=generated,
        status=status,
        source_path=path,
        session_path=path,
    )


def load_calibration_summary(
    settings: Stage7Settings | None = None,
) -> CalibrationSummary:
    stage7 = settings or Stage7Settings.load()
    deployment = stage7.current_deployment_path
    if deployment.exists():
        data = _read_json(deployment)
        raw_session_path = str(data.get("session_path") or "").strip()
        session_path = Path(raw_session_path) if raw_session_path else None
        if session_path is not None and not session_path.is_absolute():
            session_path = PROJECT_ROOT / session_path
        if session_path is not None and session_path.is_file():
            session = _summary_from_session(session_path, status="Committed")
            return CalibrationSummary(
                date=_date(data.get("committed_at") or data.get("rolled_back_at"), deployment),
                anchors=session.anchors,
                generated_points=len(data.get("points") or {}),
                status="Committed"
                if data.get("deployment_state") == "CANDIDATE_COMMITTED"
                else "Stable baseline",
                source_path=deployment,
                session_path=session_path,
            )

    sessions = sorted(
        stage7.sessions_dir.glob("*.json"),
        key=lambda item: item.stat().st_mtime,
        reverse=True,
    )
    if sessions:
        return _summary_from_session(sessions[0])

    baseline = stage7.baseline_path
    data = _read_json(baseline)
    anchors = len(data.get("anchors") or {})
    return CalibrationSummary(
        date=_date(data.get("updated_at") or data.get("created_at"), baseline),
        anchors=anchors,
        generated_points=225,
        status="Stable baseline",
        source_path=baseline,
        session_path=None,
    )
