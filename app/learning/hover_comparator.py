from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

from app.learning.hover_predictor import ShadowPrediction
from app.stage5.pwm_interpolator import InterpolationError, InterpolationResult, interpolate_target_pwm
from app.stage5.calibration_store import CalibrationStore
from app.stage5.safety import PwmSafetyLimits


PRIORITY = ("direct_anchor", "bilinear_interpolation", "pytorch_shadow")


@dataclass(frozen=True)
class ComparisonResult:
    row: int
    col: int
    sources: dict[str, dict[str, Any]]
    preferred_source: str | None
    max_abs_delta_vs_preferred: dict[str, int]
    notes: list[str]

    def to_dict(self) -> dict[str, Any]:
        return {
            "row": self.row,
            "col": self.col,
            "sources": self.sources,
            "preferred_source": self.preferred_source,
            "max_abs_delta_vs_preferred": self.max_abs_delta_vs_preferred,
            "notes": self.notes,
            "priority": list(PRIORITY),
            "MODEL_LIVE_CONTROL_ENABLED": False,
        }


class HoverPoseComparator:
    """Compare direct_anchor / bilinear / pytorch_shadow. Never sends motion."""

    def __init__(self, store: CalibrationStore, limits: PwmSafetyLimits | None = None) -> None:
        self.store = store
        self.limits = limits

    def compare(self, row: int, col: int, shadow: ShadowPrediction | None = None) -> ComparisonResult:
        sources: dict[str, dict[str, Any]] = {}
        notes: list[str] = []
        try:
            interp = interpolate_target_pwm(self.store, row, col, limits=self.limits)
            sources[interp.source] = {
                "pwm": interp.pwm_str_keys(),
                "ok": True,
                "detail": interp.to_dict(),
            }
        except InterpolationError as exc:
            notes.append(f"interp:{exc.code}:{exc}")
            sources["interpolation_error"] = {"ok": False, "code": exc.code, "message": str(exc)}

        if shadow is not None:
            sources["pytorch_shadow"] = {
                "ok": shadow.status.value == "OK",
                "status": shadow.status.value,
                "pwm": shadow.pwm,
                "message": shadow.message,
            }

        preferred = None
        for name in PRIORITY:
            if name in sources and sources[name].get("ok") and sources[name].get("pwm"):
                preferred = name
                break

        deltas: dict[str, int] = {}
        if preferred and sources[preferred].get("pwm"):
            base = sources[preferred]["pwm"]
            for name, payload in sources.items():
                if name == preferred or not payload.get("pwm"):
                    continue
                pwm = payload["pwm"]
                deltas[name] = max(abs(int(pwm[j]) - int(base[j])) for j in base)

        return ComparisonResult(
            row=int(row),
            col=int(col),
            sources=sources,
            preferred_source=preferred,
            max_abs_delta_vs_preferred=deltas,
            notes=notes,
        )
