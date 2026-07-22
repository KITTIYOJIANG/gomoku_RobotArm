from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

from app.stage5.calibration_store import AnchorPose, CalibrationStore
from app.stage5.safety import SPATIAL_JOINT_IDS, PwmSafetyLimits, validate_spatial_pwm


class InterpolationError(ValueError):
    def __init__(self, code: str, message: str) -> None:
        super().__init__(message)
        self.code = code


@dataclass(frozen=True)
class InterpolationResult:
    row: int
    col: int
    source: str
    pwm: dict[int, int]
    time_ms: int
    anchors_used: tuple[str, ...]
    u: float | None
    v: float | None
    details: dict[str, Any]

    def pwm_str_keys(self) -> dict[str, int]:
        return {f"{jid:03d}": value for jid, value in self.pwm.items()}

    def to_dict(self) -> dict[str, Any]:
        return {
            "row": self.row,
            "col": self.col,
            "source": self.source,
            "pwm": self.pwm_str_keys(),
            "time_ms": self.time_ms,
            "anchors_used": list(self.anchors_used),
            "u": self.u,
            "v": self.v,
            "details": self.details,
        }


def interpolate_target_pwm(
    store: CalibrationStore,
    row: int,
    col: int,
    *,
    limits: PwmSafetyLimits | None = None,
) -> InterpolationResult:
    """Resolve TARGET_ABOVE PWM for (row, col) via direct anchor or bilinear cells."""
    target_row = int(row)
    target_col = int(col)
    board_size = store.board_size
    if not (0 <= target_row < board_size and 0 <= target_col < board_size):
        raise InterpolationError("TARGET_OUT_OF_BOARD", f"({target_row},{target_col}) outside board")

    region = store.allowed_region
    if not (
        region["row_min"] <= target_row <= region["row_max"]
        and region["col_min"] <= target_col <= region["col_max"]
    ):
        raise InterpolationError(
            "TARGET_OUTSIDE_CALIBRATED_REGION",
            f"({target_row},{target_col}) outside calibrated region "
            f"r{region['row_min']}..{region['row_max']} c{region['col_min']}..{region['col_max']}",
        )

    direct = store.get_anchor(target_row, target_col)
    if direct is not None and direct.calibrated:
        pwm = direct.spatial_pwm()
        if limits is not None:
            errors = validate_spatial_pwm(pwm, limits)
            if errors:
                raise InterpolationError("PWM_OUT_OF_RANGE", "; ".join(errors))
        return InterpolationResult(
            row=target_row,
            col=target_col,
            source="direct_anchor",
            pwm=pwm,
            time_ms=direct.time_ms,
            anchors_used=(direct.key,),
            u=None,
            v=None,
            details={"pose_type": direct.pose_type},
        )

    r1, r2, c1, c2 = _enclosing_cell(store.anchor_rows, store.anchor_cols, target_row, target_col)
    keys = (f"{r1},{c1}", f"{r1},{c2}", f"{r2},{c1}", f"{r2},{c2}")
    corners: dict[str, AnchorPose] = {}
    for key in keys:
        anchor = store.anchors().get(key)
        if anchor is None or not anchor.calibrated:
            raise InterpolationError(
                "TARGET_UNCALIBRATED",
                f"missing calibrated corner anchor {key} for ({target_row},{target_col})",
            )
        corners[key] = anchor

    if c2 == c1:
        u = 0.0
    else:
        u = (target_col - c1) / (c2 - c1)
    if r2 == r1:
        v = 0.0
    else:
        v = (target_row - r1) / (r2 - r1)
    if not (0.0 <= u <= 1.0 and 0.0 <= v <= 1.0):
        raise InterpolationError(
            "TARGET_OUTSIDE_CALIBRATED_REGION",
            f"({target_row},{target_col}) requires extrapolation u={u:.3f} v={v:.3f}",
        )

    q11 = corners[f"{r1},{c1}"].spatial_pwm()
    q12 = corners[f"{r1},{c2}"].spatial_pwm()
    q21 = corners[f"{r2},{c1}"].spatial_pwm()
    q22 = corners[f"{r2},{c2}"].spatial_pwm()

    pwm: dict[int, int] = {}
    for jid in SPATIAL_JOINT_IDS:
        value = (
            (1.0 - u) * (1.0 - v) * q11[jid]
            + u * (1.0 - v) * q12[jid]
            + (1.0 - u) * v * q21[jid]
            + u * v * q22[jid]
        )
        pwm[jid] = int(round(value))

    if limits is not None:
        errors = validate_spatial_pwm(pwm, limits)
        if errors:
            raise InterpolationError("PWM_OUT_OF_RANGE", "; ".join(errors))
        continuity = _continuity_errors(pwm, corners, limits)
        if continuity:
            raise InterpolationError("PWM_CONTINUITY_VIOLATION", "; ".join(continuity))

    time_ms = int(round(sum(a.time_ms for a in corners.values()) / 4.0))
    return InterpolationResult(
        row=target_row,
        col=target_col,
        source="bilinear_interpolation",
        pwm=pwm,
        time_ms=time_ms,
        anchors_used=keys,
        u=float(u),
        v=float(v),
        details={
            "r1": r1,
            "r2": r2,
            "c1": c1,
            "c2": c2,
        },
    )


def _enclosing_cell(
    anchor_rows: tuple[int, ...],
    anchor_cols: tuple[int, ...],
    row: int,
    col: int,
) -> tuple[int, int, int, int]:
    rows = sorted(int(v) for v in anchor_rows)
    cols = sorted(int(v) for v in anchor_cols)
    if not rows or not cols:
        raise InterpolationError("TARGET_UNCALIBRATED", "anchor grid is empty")
    if row < rows[0] or row > rows[-1] or col < cols[0] or col > cols[-1]:
        raise InterpolationError(
            "TARGET_OUTSIDE_CALIBRATED_REGION",
            f"({row},{col}) outside anchor span",
        )
    r1 = max(r for r in rows if r <= row)
    r2 = min(r for r in rows if r >= row)
    c1 = max(c for c in cols if c <= col)
    c2 = min(c for c in cols if c >= col)
    return r1, r2, c1, c2


def _continuity_errors(
    pwm: Mapping[int, int],
    corners: Mapping[str, AnchorPose],
    limits: PwmSafetyLimits,
) -> list[str]:
    """Reject results that jump farther from corner anchors than adjacent-cell budget allows."""
    errors: list[str] = []
    # Allow up to the diagonal of one calibration cell (row span + col span) in steps.
    # We approximate with 2 * max_adjacent for a cell interior point.
    for jid in SPATIAL_JOINT_IDS:
        budget = int(limits.max_adjacent_delta[jid]) * 2
        for anchor in corners.values():
            delta = abs(int(pwm[jid]) - int(anchor.spatial_pwm()[jid]))
            # Scale budget by cell size in board cells.
            # Conservative absolute cap still tied to derived adjacent threshold.
            if delta > max(budget, limits.max_adjacent_delta[jid]):
                # Only flag pathological jumps far outside the corner envelope.
                lo = min(a.spatial_pwm()[jid] for a in corners.values())
                hi = max(a.spatial_pwm()[jid] for a in corners.values())
                if not (lo - limits.max_adjacent_delta[jid] <= pwm[jid] <= hi + limits.max_adjacent_delta[jid]):
                    errors.append(
                        f"joint {jid:03d} interpolated {pwm[jid]} outside corner envelope "
                        f"{lo}..{hi} (adj_budget={limits.max_adjacent_delta[jid]})"
                    )
                    break
    return errors
