from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

from .points import PointRef, format_point_id


SPATIAL_KEYS = ("000", "001", "002", "003", "004")


@dataclass(frozen=True)
class GoldenAboveAnchor:
    legacy_id: str
    point: PointRef
    pwm: tuple[int, int, int, int, int]

    @property
    def pwm_map(self) -> dict[str, int]:
        return dict(zip(SPATIAL_KEYS, self.pwm, strict=True))

    def to_record(self) -> dict[str, Any]:
        return {
            "point_id": format_point_id(self.point.row, self.point.col),
            "legacy_id": self.legacy_id,
            "board": [self.point.row, self.point.col],
            "above_pwm": self.pwm_map,
            "source": "golden_direct_anchor",
            "verified": True,
            "verification_level": "HARDWARE VERIFIED",
            "protected": True,
        }


GOLDEN_ABOVE: dict[tuple[int, int], GoldenAboveAnchor] = {
    (3, 3): GoldenAboveAnchor("P33", PointRef(3, 3), (1589, 1136, 1101, 1084, 1500)),
    (3, 11): GoldenAboveAnchor("P311", PointRef(3, 11), (1432, 1199, 1157, 1042, 1500)),
    (7, 7): GoldenAboveAnchor("P77", PointRef(7, 7), (1500, 1230, 870, 1230, 1500)),
    (11, 3): GoldenAboveAnchor("P113", PointRef(11, 3), (1630, 1264, 588, 1424, 1500)),
    (11, 11): GoldenAboveAnchor("P1111", PointRef(11, 11), (1382, 1258, 639, 1410, 1500)),
}

GOLDEN_FAST_5 = tuple(GOLDEN_ABOVE)
FAST_9 = (
    (0, 0),
    (0, 7),
    (0, 14),
    (7, 0),
    (7, 7),
    (7, 14),
    (14, 0),
    (14, 7),
    (14, 14),
)


def golden_for(row: int, col: int) -> GoldenAboveAnchor | None:
    return GOLDEN_ABOVE.get((int(row), int(col)))


def normalize_spatial(values: Mapping[str | int, int]) -> dict[str, int]:
    normalized = {f"{int(key):03d}": int(value) for key, value in values.items()}
    missing = [key for key in SPATIAL_KEYS if key not in normalized]
    if missing:
        raise ValueError(f"missing spatial PWM values: {missing}")
    return {key: normalized[key] for key in SPATIAL_KEYS}


def assert_golden_above(points: Mapping[str, Mapping[str, Any]]) -> None:
    """Raise if a profile lost or changed any protected Golden ABOVE value."""

    for anchor in GOLDEN_ABOVE.values():
        key = format_point_id(anchor.point.row, anchor.point.col)
        try:
            record = points[key]
        except KeyError as exc:
            raise ValueError(f"missing Golden ABOVE {anchor.legacy_id} ({key})") from exc
        final = normalize_spatial(record.get("final_above_pwm") or record.get("above_pwm") or {})
        if final != anchor.pwm_map:
            raise ValueError(
                f"Golden ABOVE changed for {anchor.legacy_id}: {final} != {anchor.pwm_map}"
            )
        if not bool(record.get("protected")):
            raise ValueError(f"Golden ABOVE {anchor.legacy_id} is not protected")
        if record.get("source") != "golden_direct_anchor":
            raise ValueError(f"Golden ABOVE {anchor.legacy_id} source is not authoritative")
