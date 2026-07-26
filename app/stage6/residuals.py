from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

from .models import SPATIAL_KEYS, DescentLevel


@dataclass(frozen=True)
class ResidualEstimate:
    delta_pwm: dict[str, int]
    anchors_used: tuple[str, ...]
    method: str


class ResidualCorrector:
    """Bilinear residual interpolation from four explicitly verified anchors."""

    def __init__(self, profiles: Mapping[str, Any]) -> None:
        self.profiles = profiles

    def estimate(
        self, row: int, col: int, level: DescentLevel
    ) -> ResidualEstimate | None:
        verified: dict[tuple[int, int], dict[str, int]] = {}
        for key, profile in self.profiles.items():
            level_data = (profile.get("levels") or {}).get(level.value) or {}
            if level_data.get("status") != "VERIFIED":
                continue
            raw = level_data.get("manual_delta_pwm") or {}
            r, c = (int(value) for value in key.split(",", 1))
            verified[(r, c)] = {
                joint: int(raw.get(joint, 0)) for joint in SPATIAL_KEYS
            }
        if (row, col) in verified:
            return ResidualEstimate(
                dict(verified[(row, col)]),
                (f"{row},{col}",),
                "verified_anchor_residual",
            )
        rows = sorted({point[0] for point in verified})
        cols = sorted({point[1] for point in verified})
        lower_rows = [value for value in rows if value <= row]
        upper_rows = [value for value in rows if value >= row]
        lower_cols = [value for value in cols if value <= col]
        upper_cols = [value for value in cols if value >= col]
        if not lower_rows or not upper_rows or not lower_cols or not upper_cols:
            return None
        for r1 in reversed(lower_rows):
            for r2 in upper_rows:
                for c1 in reversed(lower_cols):
                    for c2 in upper_cols:
                        corners = ((r1, c1), (r1, c2), (r2, c1), (r2, c2))
                        if not all(point in verified for point in corners):
                            continue
                        u = 0.0 if c1 == c2 else (col - c1) / (c2 - c1)
                        v = 0.0 if r1 == r2 else (row - r1) / (r2 - r1)
                        delta: dict[str, int] = {}
                        for joint in SPATIAL_KEYS:
                            top = (
                                verified[(r1, c1)][joint] * (1.0 - u)
                                + verified[(r1, c2)][joint] * u
                            )
                            bottom = (
                                verified[(r2, c1)][joint] * (1.0 - u)
                                + verified[(r2, c2)][joint] * u
                            )
                            delta[joint] = int(
                                round(top * (1.0 - v) + bottom * v)
                            )
                        return ResidualEstimate(
                            delta,
                            tuple(f"{r},{c}" for r, c in corners),
                            "bilinear_verified_residual",
                        )
        return None
