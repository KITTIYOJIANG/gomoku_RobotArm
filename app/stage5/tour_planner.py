from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping, Sequence

from app.arm.actions import ActionLibrary
from app.arm.sequences import ActionStep, SequenceDefinition, WaitStep, validate_safe_sequence
from app.stage5.calibration_store import CalibrationStore
from app.stage5.constants import CROSS_ANCHORS, SPATIAL_JOINTS, anchor_key
from app.stage5.hover_planner import build_action_from_pwm, build_lifted_carry_action
from app.stage5.pwm_interpolator import InterpolationError, interpolate_target_pwm, resolve_target_pwm
from app.stage5.safety import SPATIAL_JOINT_IDS, PwmSafetyLimits


@dataclass(frozen=True)
class TourStop:
    row: int
    col: int
    label: str
    source: str
    pwm: dict[str, int]


@dataclass(frozen=True)
class TourPlan:
    name: str
    display_name: str
    stops: tuple[TourStop, ...]
    sequence: SequenceDefinition
    estimated_duration_ms: int
    notes: tuple[str, ...] = ()

    def to_public_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "display_name": self.display_name,
            "stop_count": len(self.stops),
            "stops": [
                {
                    "row": s.row,
                    "col": s.col,
                    "label": s.label,
                    "source": s.source,
                    "pwm": s.pwm,
                }
                for s in self.stops
            ],
            "estimated_duration_ms": self.estimated_duration_ms,
            "action_names": list(self.sequence.action_names),
            "notes": list(self.notes),
        }


def _pwm_str_map(pwm: Mapping[int | str, int]) -> dict[str, int]:
    out: dict[str, int] = {}
    for k, v in pwm.items():
        key = f"{int(k):03d}" if not isinstance(k, str) else str(k).zfill(3)
        if key in SPATIAL_JOINTS:
            out[key] = int(v)
    for jid in SPATIAL_JOINTS:
        if jid not in out:
            raise ValueError(f"missing joint {jid}")
    return out


def _full_joint_map(spatial: Mapping[str, int], *, holding: bool = False) -> dict[int, int]:
    filled = {jid: 1500 for jid in range(8)}
    for jid in SPATIAL_JOINT_IDS:
        filled[jid] = int(spatial[f"{jid:03d}"])
    filled[5] = 2500 if holding else 1500
    return filled


def _register_target(library: ActionLibrary, *, row: int, col: int, spatial: Mapping[str, int]) -> str:
    name = f"TARGET_TOUR_P{int(row)}{int(col)}"
    action = build_action_from_pwm(name, _full_joint_map(spatial), time_ms=1000)
    library.register_runtime(action)
    return action.name


def _estimate_ms(library: ActionLibrary, sequence: SequenceDefinition, margin_ms: int) -> int:
    total = 0
    for step in sequence.steps:
        if isinstance(step, ActionStep):
            total += library.get(step.action_name).duration_ms + margin_ms
        else:
            total += max(0, int(step.duration_ms))
    return int(total)


def list_completed_cross_stops(
    calibration: CalibrationStore,
    drafts: Mapping[str, Any] | None = None,
) -> list[TourStop]:
    """Prefer formal board calibration PWM; fall back to completed drafts."""
    draft_map: dict[str, Any] = {}
    if isinstance(drafts, Mapping):
        raw_anchors = drafts.get("anchors")
        if isinstance(raw_anchors, Mapping):
            draft_map = dict(raw_anchors)
        else:
            draft_map = dict(drafts)

    stops: list[TourStop] = []
    for row, col, label, cn in CROSS_ANCHORS:
        key = anchor_key(row, col)
        anchor = calibration.get_anchor(row, col)
        if anchor is not None and anchor.calibrated:
            pwm = _pwm_str_map(anchor.pwm)
            stops.append(TourStop(row, col, f"{cn}/{label}", "board_calibration", pwm))
            continue
        entry = (draft_map or {}).get(key) or {}
        raw = entry.get("candidate_pwm") or {}
        if entry.get("status") == "COMPLETED" and all(raw.get(j) is not None for j in SPATIAL_JOINTS):
            pwm = {j: int(raw[j]) for j in SPATIAL_JOINTS}
            stops.append(TourStop(row, col, f"{cn}/{label}", "draft_completed", pwm))
    return stops


def list_reachable_board_stops(
    calibration: CalibrationStore,
    *,
    limits: PwmSafetyLimits | None = None,
    direct_only: bool = False,
) -> tuple[list[TourStop], list[str]]:
    """Intersections resolvable by direct anchor and/or interpolation."""
    region = calibration.allowed_region
    stops: list[TourStop] = []
    notes: list[str] = []
    for row in range(region["row_min"], region["row_max"] + 1):
        for col in range(region["col_min"], region["col_max"] + 1):
            try:
                result = resolve_target_pwm(calibration, row, col, limits=limits, allow_star_seed=True)
            except InterpolationError:
                continue
            if direct_only and result.source != "direct_anchor":
                continue
            pwm = {f"{jid:03d}": int(result.pwm[jid]) for jid in SPATIAL_JOINT_IDS}
            stops.append(
                TourStop(
                    row=row,
                    col=col,
                    label=f"P({row},{col})",
                    source=result.source,
                    pwm=pwm,
                )
            )
    if not stops:
        notes.append("no reachable points under current calibration")
    return stops, notes


def order_stops_cross_axes(stops: Sequence[TourStop]) -> list[TourStop]:
    """Visit vertical arm top→bottom, then horizontal left→right (no center-first jump).

    Previous center-first order jumped P(7,7)→P(3,7) without a natural neighbor path,
    which made mid-points look worse. Continuous axes keep TARGET→TARGET hops short.
    """
    by_rc = {(s.row, s.col): s for s in stops}
    ordered: list[TourStop] = []
    seen: set[tuple[int, int]] = set()

    def take(row: int, col: int) -> None:
        key = (row, col)
        if key in by_rc and key not in seen:
            ordered.append(by_rc[key])
            seen.add(key)

    # Vertical first: top → bottom along col 7 (includes center when present)
    for row in sorted({s.row for s in stops if s.col == 7}):
        take(row, 7)
    # Horizontal: left → right along row 7 (center already visited)
    for col in sorted({s.col for s in stops if s.row == 7}):
        take(7, col)
    # Leftovers (future full-grid cells)
    for s in sorted(stops, key=lambda x: (x.row, x.col)):
        take(s.row, s.col)
    return ordered


def _grid_distance(a: TourStop, b: TourStop) -> int:
    return abs(int(a.row) - int(b.row)) + abs(int(b.col) - int(a.col))


def _can_skip_carry(prev: TourStop | None, stop: TourStop) -> bool:
    """Only allow TARGET→TARGET when moving one cell on the same row or column."""
    if prev is None:
        return False
    if not (prev.row == stop.row or prev.col == stop.col):
        return False
    return _grid_distance(prev, stop) == 1


def build_hover_tour(
    library: ActionLibrary,
    stops: Sequence[TourStop],
    *,
    name: str,
    display_name: str,
    dwell_ms: int = 450,
    action_wait_margin_ms: int = 200,
    notes: Sequence[str] = (),
    cool_every_n: int = 0,
    cool_ms: int = 0,
    path_mode: str = "carry_each",
) -> TourPlan:
    """Build a hover tour.

    path_mode:
      - carry_each: CARRY_HIGH before every target (max safety, more heat)
      - segment: CARRY_HIGH only when axis changes / after cool; TARGET→TARGET on same axis
    cool_every_n > 0 inserts OBSERVE + cool wait every N targets (thermal relief).
    """
    if not stops:
        raise ValueError("tour has no stops")
    ordered = list(stops)
    steps: list[ActionStep | WaitStep] = []
    prev: TourStop | None = None
    targets_since_cool = 0
    force_carry = True
    # Lift transit height using the tallest taught/inferred 001 on this tour.
    max_001 = 0
    for stop in ordered:
        try:
            max_001 = max(max_001, int(stop.pwm.get("001", 0)))
        except Exception:
            pass
    carry_action = build_lifted_carry_action(
        library,
        holding_piece=False,
        reference_001=max_001 or None,
    )
    carry_name = carry_action.name

    for stop in ordered:
        skip = (
            path_mode == "segment"
            and not force_carry
            and _can_skip_carry(prev, stop)
        )
        if not skip:
            steps.append(ActionStep(carry_name))

        target_name = _register_target(library, row=stop.row, col=stop.col, spatial=stop.pwm)
        steps.append(ActionStep(target_name))
        if dwell_ms > 0:
            steps.append(WaitStep(f"DWELL P({stop.row},{stop.col})", int(dwell_ms)))

        targets_since_cool += 1
        prev = stop
        force_carry = False

        if cool_every_n > 0 and cool_ms > 0 and targets_since_cool >= cool_every_n:
            steps.append(ActionStep(carry_name))
            steps.append(ActionStep("OBSERVE_IDLE"))
            steps.append(WaitStep(f"COOL after P({stop.row},{stop.col})", int(cool_ms)))
            targets_since_cool = 0
            force_carry = True
            prev = None  # force carry after cool

    steps.append(ActionStep(carry_name))
    steps.append(ActionStep("OBSERVE_IDLE"))
    sequence = SequenceDefinition(
        name=name,
        display_name=display_name,
        requires_board=True,
        steps=tuple(steps),
    )
    validate_safe_sequence(sequence)
    return TourPlan(
        name=name,
        display_name=display_name,
        stops=tuple(ordered),
        sequence=sequence,
        estimated_duration_ms=_estimate_ms(library, sequence, action_wait_margin_ms),
        notes=tuple(notes),
    )


def build_cross_reverify_tour(
    library: ActionLibrary,
    calibration: CalibrationStore,
    *,
    drafts: Mapping[str, Any] | None = None,
    dwell_ms: int = 450,
    action_wait_margin_ms: int = 200,
) -> TourPlan:
    stops = list_completed_cross_stops(calibration, drafts)
    if not stops:
        raise ValueError("没有已完成的十字锚点可复验（需要正式标定或 COMPLETED 草稿）")
    # Re-verify: few points, always carry each (same proven path as manual wizard).
    return build_hover_tour(
        library,
        stops,
        name="CROSS_REVERIFY_TOUR",
        display_name=f"十字锚点一键复验（{len(stops)}点）",
        dwell_ms=dwell_ms,
        action_wait_margin_ms=action_wait_margin_ms,
        path_mode="carry_each",
        notes=("via_carry_high", "no_touch", "ends_at_observe", "beep_on_success", "anchors_only"),
    )


def build_board_reachable_tour(
    library: ActionLibrary,
    calibration: CalibrationStore,
    *,
    limits: PwmSafetyLimits | None = None,
    dwell_ms: int = 500,
    action_wait_margin_ms: int = 200,
    direct_only: bool = False,
    cool_every_n: int = 3,
    cool_ms: int = 6000,
    path_mode: str = "segment",
) -> TourPlan:
    stops, notes = list_reachable_board_stops(
        calibration, limits=limits, direct_only=direct_only
    )
    if not stops:
        raise ValueError("当前标定下没有可插值/直达的交点")
    ordered = order_stops_cross_axes(stops)
    n_direct = sum(1 for s in ordered if s.source == "direct_anchor")
    n_interp = len(ordered) - n_direct
    notes = list(notes) + [
        f"reachable={len(ordered)}",
        f"direct={n_direct}",
        f"interpolated={n_interp}",
        f"path_mode={path_mode}",
        f"cool_every_n={cool_every_n}",
        f"cool_ms={cool_ms}",
        "ends_at_observe",
        "beep_on_success",
    ]
    if n_interp:
        notes.append("interp_points_are_estimated_not_taught")
    mode_label = "仅锚点" if direct_only else "可达交点"
    return build_hover_tour(
        library,
        ordered,
        name="BOARD_HOVER_TOUR",
        display_name=f"棋盘巡检·{mode_label}（{len(ordered)}点，分段冷却）",
        dwell_ms=dwell_ms,
        action_wait_margin_ms=action_wait_margin_ms,
        cool_every_n=cool_every_n,
        cool_ms=cool_ms,
        path_mode=path_mode,
        notes=notes,
    )


def sync_completed_drafts_into_calibration(
    calibration: CalibrationStore,
    drafts_data: Mapping[str, Any],
    *,
    safety_limits: PwmSafetyLimits | None = None,
) -> list[str]:
    """Promote COMPLETED draft anchors missing from formal calibration. Returns keys written."""
    written: list[str] = []
    anchors = drafts_data.get("anchors") or {}
    for key, entry in anchors.items():
        if entry.get("status") != "COMPLETED":
            continue
        row = int(entry.get("row", key.split(",")[0]))
        col = int(entry.get("col", key.split(",")[1]))
        existing = calibration.get_anchor(row, col)
        if existing is not None and existing.calibrated:
            continue
        raw = entry.get("candidate_pwm") or {}
        if not all(raw.get(j) is not None for j in SPATIAL_JOINTS):
            continue
        pwm = {j: int(raw[j]) for j in SPATIAL_JOINTS}
        calibration.upsert_anchor(
            row,
            col,
            pwm,
            time_ms=1000,
            notes="promoted from completed draft for tour",
            calibrated=True,
            verified_runs=int(entry.get("verified_runs") or 0),
            require_anchor_set=True,
            safety_limits=safety_limits,
            skip_envelope_check=safety_limits is None,
        )
        written.append(anchor_key(row, col))
    if written:
        calibration.save()
    return written
