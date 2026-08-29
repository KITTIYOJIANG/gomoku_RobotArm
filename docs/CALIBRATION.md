# Calibration

## Current Calibration Strategy

Robot calibration is board-indexed PWM, not a camera-to-robot metric transform. Vision maps pixels to zero-based board coordinates through a 2D homography. Stage 5 maps a board coordinate to an ABOVE spatial PWM pose using direct measured anchors where available and interpolation elsewhere. Stage 6 derives a separate descent candidate profile from that ABOVE pose.

```text
image pixel -> board (row,col)
board (row,col) -> stable/direct or interpolated ABOVE PWM
ABOVE PWM -> optional independent Stage 6 descent profile
```

The copied factory IK is used only for Stage 6 candidate generation. It does not replace the empirically taught Stage 5 PWM baseline.

## Baseline Calibration

Authoritative file: `calibration/stage5_board_calibration.json`

- Audited SHA-256: `21B4994D75E03ACCAF8D95B85A9577A9A41940370D2CE3C82F1F87AEB2EE8052`.
- Board size: 15×15 / 225 intersections.
- Persisted direct anchors: 30.
- Direct anchors with `calibrated=true`: 30.
- Direct anchors with `verified_runs>0`: 30.
- Runtime interpolated points: 195.
- Total readable ABOVE points: 225.

The `verified_runs` field and Stage 5 LIVE logs are hardware evidence for taught/used anchors, but they are not servo arrival telemetry. The 195 interpolated points must never be described as 195 independently persisted or verified poses.

P77 `(7,7)` is imported from the stable `P77_ABOVE_IDLE` action and is protected by Stage 5 calibration code. The baseline metadata does not record camera or board setup descriptions; deployment geometry remains unknown.

## Point Index Convention

The board coordinate system is zero-based, row-down and column-right.

- Stable code and UI commonly use `P77` to mean `(row=7,col=7)`.
- The Stage 7 draft converts `(row,col)` to flat index `row*15+col`, formatted with three digits.
- Therefore center `(7,7)` is flat `P112`, not flat `P077`.

Corner examples: `P000=(0,0)`, `P014=(0,14)`, `P210=(14,0)`, `P224=(14,14)`. Always include `(row,col)` when exchanging a point name across stages.

## ABOVE Calibration

Load path: `app/stage5/calibration_store.py::CalibrationStore`.

Resolution path: `app/stage5/pwm_interpolator.py::resolve_target_pwm`.

Resolution priority at the stable Stage 5 baseline is:

1. Direct calibrated anchor.
2. Bilinear interpolation inside the tightest calibrated rectangle.
3. Explicit star-position seed fallback.
4. Explicit outer-ring seed fallback.

Only spatial PWM `000..004` is interpolated. Pump `005` is attached later according to idle/hold state; `006..007` remain stable action values.

## Descent Calibration

Stage 6 is intentionally separate:

- Configuration: `config/stage6_descent.json` with `force_dry_run=true`.
- Candidate store: `calibration/stage6_descent_calibration.json`.
- Source baseline hash recorded in that file: the same `21B4994D...EE8052` Stage 5 hash.
- ABOVE FK snapshot: 225.
- Complete profiles: 153.
- Rejected points: 72.
- Verification: 152 `COMPUTED`, 1 `DRY_RUN_PASSED` (P77), 0 hardware verified.

Each complete profile has ABOVE, 25%, 50%, 75%, and TOUCH levels plus exact saved reverse-ascent use. Joint `004` is preserved through layers and pump `005` is excluded from IK. Rejected candidates retain their error instead of being PWM-clipped.

The configured link lengths and host biases are candidate model values. Factory comments conflict with executed setup values, installed EEPROM bias is unknown, and no real vertical descent has validated the model.

## Rapid Deployment Calibration

Status: **IN PROGRESS / WORKING-TREE DRAFT / NOT DEPLOYED**.

The current `app/stage7/` draft implements the intended relationship:

```text
stable Stage 5 baseline
  + direct anchor delta PWM
  + interpolation of the delta field
  = separate candidate deployment calibration
```

The baseline is hashed and checked for mutation while a Stage 7 session is active. Session and current-deployment outputs use separate paths. The current backend and GUI pass 11 dedicated offline tests and are wired into `MainWindow`. However, the implementation is uncommitted and `force_dry_run` is false even though `default_dry_run` is true. It must not be treated as a hardware-ready Stage 7 implementation.

## Calibration Safety Rules

1. Never overwrite the stable Stage 5 file to create a Stage 6 or Stage 7 candidate.
2. Verify the baseline hash before and after any read/generation operation.
3. Persist provenance: direct anchor, anchors used, interpolation method, verification level and session ID.
4. Keep pump and reserved outputs outside spatial interpolation/IK.
5. Reject out-of-envelope candidates; do not clip and call them verified.
6. A dry-run changes status only to offline/dry-run evidence, never hardware verified.
7. Preserve the exact reverse ascent and require carry-high before horizontal target changes.
8. Promote a new baseline only after explicit review, a backup/new file, focused hardware validation, updated state docs, and an appropriate stable tag.

## Integrated V1 Profile

V1 stores its candidate state at `calibration/profiles/integrated_v1_current.json`; absence of this file means First Setup, not failure. It resolves all 225 Stage 5 ABOVE points into a new profile, overlays the five user-confirmed Golden ABOVE anchors, and records the old/new conflict without editing the Stage 5 source.

For DROP, `auto`, per-joint `correction`, and `final` PWM are separate. The MoveL planner preserves Cartesian `x/y/alpha`, steps downward in `z`, rejects unsafe/unreachable solutions and saves exact reverse-ascent indices. A calculation or dry-run is **OFFLINE VERIFIED** at most. Only an explicitly completed live point test may produce **HARDWARE VERIFIED**, which is required by the live game gate.

Fast 5/9 creates a clamped piecewise-bilinear correction field. It never overwrites Golden or user-direct anchors, and it invalidates DROP data affected by an ABOVE change. Golden changes are staged as `PENDING_REVALIDATION` rather than applied. See `docs/INTEGRATED_V1_REPORT.md` and `docs/HARDWARE_TEST_CHECKLIST_V1.md`.

## Lite Calibration V1 DROP Profile (current scope)

Lite V1 stores `calibration/calibration_lite_drop_v1.json` independently and never rewrites the stable ABOVE/action assets. It synchronizes the saved 225-point Lite ABOVE snapshot, then force-overlays the five immutable Golden ABOVE values. Every pose field contains J0–J4 only; J5 remains the explicit pump channel outside FK/IK.

The DROP planner holds Cartesian x/y/alpha and steps Z down at 5 mm waypoints to 25 mm, solving each waypoint with previous-solution IK seeding. It blocks non-finite, unreachable, limit-breaking, J4-changing, and discontinuous solutions and records the deepest safe waypoint. Exact saved reverse indices are mandatory for Retract. Automatic DROP, manual correction, final DROP, failure state, and verification evidence remain separate. See `docs/CALIBRATION_LITE_V1_REPORT.md` and `docs/HARDWARE_TEST_CHECKLIST_LITE_V1.md`.
