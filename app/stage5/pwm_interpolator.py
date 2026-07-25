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
    """Resolve TARGET_ABOVE PWM.

    Priority:
      1) direct calibrated taught point (user fine-tune / confirmed)
      2) bilinear on the tightest rectangle of *calibrated* anchors
      3) (not used here) caller may fall back to star-corner estimate
    Untaught points use default interpolation and are treated as accurate enough
    until the user overrides them.
    """
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

    # --- Priority 1: direct taught / calibrated anchor ---
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
            details={"pose_type": direct.pose_type, "priority": 1},
        )

    calibrated = store.calibrated_anchors()
    if not calibrated:
        raise InterpolationError("TARGET_UNCALIBRATED", "no calibrated anchors")

    cell = _enclosing_calibrated_cell(calibrated, target_row, target_col)
    if cell is None:
        # Fall back to configured grid (legacy) for compatibility.
        try:
            r1, r2, c1, c2 = _enclosing_cell(
                store.anchor_rows, store.anchor_cols, target_row, target_col
            )
            keys = (f"{r1},{c1}", f"{r1},{c2}", f"{r2},{c1}", f"{r2},{c2}")
            if not all(k in calibrated for k in keys):
                raise InterpolationError(
                    "TARGET_UNCALIBRATED",
                    f"missing calibrated corners for ({target_row},{target_col}); "
                    "teach star corners or nearby fine-tune points",
                )
            cell = (r1, r2, c1, c2)
        except InterpolationError:
            raise
        except Exception as exc:
            raise InterpolationError("TARGET_UNCALIBRATED", str(exc)) from exc

    r1, r2, c1, c2 = cell
    keys = (f"{r1},{c1}", f"{r1},{c2}", f"{r2},{c1}", f"{r2},{c2}")
    corners = {k: calibrated[k] for k in keys}

    if c2 == c1:
        u = 0.0
    else:
        u = (target_col - c1) / (c2 - c1)
    if r2 == r1:
        v = 0.0
    else:
        v = (target_row - r1) / (r2 - r1)

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
            "priority": 2,
            "note": "default_interpolated_not_user_taught",
        },
    )


def estimate_star_corner_pwm(
    store: CalibrationStore,
    row: int,
    col: int,
) -> InterpolationResult:
    """Parallelogram seed for a star corner from P77 + cross arms.

    For UL (3,3):  P(3,3) ≈ P(3,7) + P(7,3) - P(7,7)
    Similarly for other corners using the nearest cross mid-edge anchors.
    """
    r = int(row)
    c = int(col)
    # Choose reference edges on the cross (row center=7, col center=7).
    center = store.get_anchor(7, 7)
    edge_row = store.get_anchor(r, 7)  # same row as corner, center col
    edge_col = store.get_anchor(7, c)  # center row, same col as corner
    missing = []
    if center is None or not center.calibrated:
        missing.append("7,7")
    if edge_row is None or not edge_row.calibrated:
        missing.append(f"{r},7")
    if edge_col is None or not edge_col.calibrated:
        missing.append(f"7,{c}")
    if missing:
        raise InterpolationError(
            "STAR_SEED_MISSING",
            f"cannot seed P({r},{c}); need calibrated {', '.join(missing)}",
        )

    assert center is not None and edge_row is not None and edge_col is not None
    c_pwm = center.spatial_pwm()
    er_pwm = edge_row.spatial_pwm()
    ec_pwm = edge_col.spatial_pwm()
    pwm: dict[int, int] = {}
    for jid in SPATIAL_JOINT_IDS:
        # Parallelogram completion in joint PWM space (seed only).
        value = int(er_pwm[jid]) + int(ec_pwm[jid]) - int(c_pwm[jid])
        pwm[jid] = max(500, min(2500, value))
    return InterpolationResult(
        row=r,
        col=c,
        source="star_parallelogram_seed",
        pwm=pwm,
        time_ms=1000,
        anchors_used=(center.key, edge_row.key, edge_col.key),
        u=None,
        v=None,
        details={"method": "p_rc = p_r7 + p_7c - p_77", "priority": 0},
    )



def estimate_outer_ring_pwm(
    store: CalibrationStore,
    row: int,
    col: int,
) -> InterpolationResult:
    """Seed PWM for outer-ring points by linear extrapolation from center lattice.

    Preferred path:
      - edge mid (0,7)/(14,7)/(7,0)/(7,14): extrapolate along the cross from P77 + inner edge
      - board corners: parallelogram from outer edge mids + P77 once available,
        else from inner star corner + cross.
    """
    r, c = int(row), int(col)
    calibrated = store.calibrated_anchors()

    def need(rr: int, cc: int) -> AnchorPose:
        key = f"{rr},{cc}"
        a = calibrated.get(key) or store.get_anchor(rr, cc)
        if a is None or not a.calibrated:
            raise InterpolationError("OUTER_SEED_MISSING", f"need calibrated P({rr},{cc})")
        return a

    def spatial(a: AnchorPose) -> dict[int, int]:
        return a.spatial_pwm()

    def blend(a: dict[int, int], b: dict[int, int], t: float) -> dict[int, int]:
        # extrapolate: p = a + (a-b)*scale where t is factor on (a-b)
        out: dict[int, int] = {}
        for jid in SPATIAL_JOINT_IDS:
            val = int(round(a[jid] + (a[jid] - b[jid]) * t))
            out[jid] = max(550, min(2450, val))
        return out

    used: list[str] = []
    method = ""

    # --- cross outer mids ---
    if (r, c) == (0, 7):
        # from P(3,7) away from P(7,7): distance 3 beyond span 4 → factor 3/4
        inner, hub = need(3, 7), need(7, 7)
        pwm = blend(spatial(inner), spatial(hub), 3.0 / 4.0)
        used, method = [inner.key, hub.key], "extrap_top"
    elif (r, c) == (14, 7):
        inner, hub = need(11, 7), need(7, 7)
        pwm = blend(spatial(inner), spatial(hub), 3.0 / 4.0)
        used, method = [inner.key, hub.key], "extrap_bottom"
    elif (r, c) == (7, 0):
        inner, hub = need(7, 3), need(7, 7)
        pwm = blend(spatial(inner), spatial(hub), 3.0 / 4.0)
        used, method = [inner.key, hub.key], "extrap_left"
    elif (r, c) == (7, 14):
        inner, hub = need(7, 11), need(7, 7)
        pwm = blend(spatial(inner), spatial(hub), 3.0 / 4.0)
        used, method = [inner.key, hub.key], "extrap_right"
    # --- board corners: prefer outer mids parallelogram, else star+cross ---
    elif (r, c) == (0, 0):
        try:
            top, left, hub = need(0, 7), need(7, 0), need(7, 7)
            pwm = {
                j: max(550, min(2450, spatial(top)[j] + spatial(left)[j] - spatial(hub)[j]))
                for j in SPATIAL_JOINT_IDS
            }
            used, method = [top.key, left.key, hub.key], "para_outer_ul"
        except InterpolationError:
            star, er, ec, hub = need(3, 3), need(3, 7), need(7, 3), need(7, 7)
            # extrapolate star corner outward similarly on both axes
            # p00 ≈ p33 + (p33-p77) * (3/4) roughly via edges
            s, h = spatial(star), spatial(hub)
            pwm = {j: max(550, min(2450, int(round(s[j] + (s[j] - h[j]) * 0.75)))) for j in SPATIAL_JOINT_IDS}
            used, method = [star.key, hub.key], "extrap_star_ul"
    elif (r, c) == (0, 14):
        try:
            top, right, hub = need(0, 7), need(7, 14), need(7, 7)
            pwm = {
                j: max(550, min(2450, spatial(top)[j] + spatial(right)[j] - spatial(hub)[j]))
                for j in SPATIAL_JOINT_IDS
            }
            used, method = [top.key, right.key, hub.key], "para_outer_ur"
        except InterpolationError:
            star, hub = need(3, 11), need(7, 7)
            s, h = spatial(star), spatial(hub)
            pwm = {j: max(550, min(2450, int(round(s[j] + (s[j] - h[j]) * 0.75)))) for j in SPATIAL_JOINT_IDS}
            used, method = [star.key, hub.key], "extrap_star_ur"
    elif (r, c) == (14, 0):
        try:
            bot, left, hub = need(14, 7), need(7, 0), need(7, 7)
            pwm = {
                j: max(550, min(2450, spatial(bot)[j] + spatial(left)[j] - spatial(hub)[j]))
                for j in SPATIAL_JOINT_IDS
            }
            used, method = [bot.key, left.key, hub.key], "para_outer_dl"
        except InterpolationError:
            star, hub = need(11, 3), need(7, 7)
            s, h = spatial(star), spatial(hub)
            pwm = {j: max(550, min(2450, int(round(s[j] + (s[j] - h[j]) * 0.75)))) for j in SPATIAL_JOINT_IDS}
            used, method = [star.key, hub.key], "extrap_star_dl"
    elif (r, c) == (14, 14):
        try:
            bot, right, hub = need(14, 7), need(7, 14), need(7, 7)
            pwm = {
                j: max(550, min(2450, spatial(bot)[j] + spatial(right)[j] - spatial(hub)[j]))
                for j in SPATIAL_JOINT_IDS
            }
            used, method = [bot.key, right.key, hub.key], "para_outer_dr"
        except InterpolationError:
            star, hub = need(11, 11), need(7, 7)
            s, h = spatial(star), spatial(hub)
            pwm = {j: max(550, min(2450, int(round(s[j] + (s[j] - h[j]) * 0.75)))) for j in SPATIAL_JOINT_IDS}
            used, method = [star.key, hub.key], "extrap_star_dr"
    else:
        raise InterpolationError(
            "OUTER_SEED_UNSUPPORTED",
            f"P({r},{c}) is not a defined outer-ring seed point",
        )

    return InterpolationResult(
        row=r,
        col=c,
        source="outer_ring_seed",
        pwm=pwm,
        time_ms=1000,
        anchors_used=tuple(used),
        u=None,
        v=None,
        details={"method": method, "priority": 0},
    )



def resolve_target_pwm(
    store: CalibrationStore,
    row: int,
    col: int,
    *,
    limits: PwmSafetyLimits | None = None,
    allow_star_seed: bool = True,
    allow_outer_seed: bool = True,
) -> InterpolationResult:
    """Public resolver: taught > bilinear > star seed > outer-ring seed."""
    try:
        return interpolate_target_pwm(store, row, col, limits=limits)
    except InterpolationError:
        last: Exception | None = None
        if allow_star_seed:
            try:
                return estimate_star_corner_pwm(store, row, col)
            except InterpolationError as exc:
                last = exc
        if allow_outer_seed:
            try:
                return estimate_outer_ring_pwm(store, row, col)
            except InterpolationError as exc:
                last = exc
        if last is not None:
            raise last
        raise


def _enclosing_calibrated_cell(
    calibrated: Mapping[str, AnchorPose],
    row: int,
    col: int,
) -> tuple[int, int, int, int] | None:
    """Tightest rectangle whose four corners are all calibrated."""
    rows = sorted({a.row for a in calibrated.values()})
    cols = sorted({a.col for a in calibrated.values()})
    r_lo = [r for r in rows if r <= row]
    r_hi = [r for r in rows if r >= row]
    c_lo = [c for c in cols if c <= col]
    c_hi = [c for c in cols if c >= col]
    if not r_lo or not r_hi or not c_lo or not c_hi:
        return None
    # Closest bounds first → tightest cell that still has all four corners.
    for r1 in reversed(r_lo):
        for r2 in r_hi:
            if r2 < r1:
                continue
            for c1 in reversed(c_lo):
                for c2 in c_hi:
                    if c2 < c1:
                        continue
                    keys = (f"{r1},{c1}", f"{r1},{c2}", f"{r2},{c1}", f"{r2},{c2}")
                    if all(k in calibrated for k in keys):
                        return r1, r2, c1, c2
    return None


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
    errors: list[str] = []
    for jid in SPATIAL_JOINT_IDS:
        lo = min(a.spatial_pwm()[jid] for a in corners.values())
        hi = max(a.spatial_pwm()[jid] for a in corners.values())
        budget = int(limits.max_adjacent_delta[jid])
        if not (lo - budget <= pwm[jid] <= hi + budget):
            errors.append(
                f"joint {jid:03d} interpolated {pwm[jid]} outside corner envelope "
                f"{lo}..{hi} (adj_budget={budget})"
            )
    return errors
