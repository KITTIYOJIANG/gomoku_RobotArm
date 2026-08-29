# Stage 6 offline delivery report

Report date: 2026-07-26

Branch: `stage6-full-board-descent`

Frozen baseline: `7a1e1253f37c9614a8e528f2d4b01e099e03474f`

Stable tag: `J1_Gomoku_V0.2_Arbitrary_Hover_Stable`

This report covers the requested offline scope only. `force_dry_run` remains
enabled. No real serial connection, descent, or piece release was performed.

## 1. ABOVE data location and provenance

The stable source is `calibration/stage5_board_calibration.json`.

It contains 30 calibrated direct anchors. The remaining 195 intersections are
resolved by the Stage 5 bilinear interpolator, for a total of 225 readable
ABOVE poses. Stage 6 preserves this provenance instead of claiming that all 225
records are independently persisted or verified.

The source SHA-256 before and after Stage 6 generation was identical:

`21B4994D75E03ACCAF8D95B85A9577A9A41940370D2CE3C82F1F87AEB2EE8052`

## 2. FK/IK source

The IK geometry and PWM scale are based on the copied factory implementation
in `firmware/stm32_j1_reference/src/z_kinematics.c`. The host FK is the
configured mathematical forward form of that same four-axis planar model.

The copied factory source does not contain FK. The host implementation is in
`app/stage6/kinematics.py`.

## 3. Link parameters

The executed factory setup call provides:

- `L0 = 110 mm`
- `L1 = 105 mm`
- `L2 = 75 mm`
- `L3 = 190 mm`

The adjacent factory comment disagrees with this call. Therefore these remain
candidate geometry, not physically re-measured dimensions.

## 4. PWM/angle conversion

Factory scale: `270 deg / 2000 PWM = 0.135 deg/PWM`.

Each joint has one configured zero, direction, scale, angular bias, kinematic
offset, and PWM range in `config/arm_kinematics.json`. Conversion is:

`angle = direction * (pwm - zero_pwm) * 0.135 + bias + offset`

and its algebraic inverse. Joint `004` is passthrough and does not participate
in the planar IK. Pump `005` is excluded entirely.

## 5. P77 kinematic calibration

The candidate host biases were fitted so the stable P77 ABOVE and TOUCH share
FK X/Y while retaining both original PWM endpoints:

- joint `001` bias: `35.32595960670788 deg`
- joint `002` bias: `-14.534642111236085 deg`
- joint `003` bias: `-36.91060171794394 deg`
- P77 ABOVE alpha target: `-55 deg`

P77 ABOVE FK:

`x=-27.187401 mm, y=191.028637 mm, z=91.422041 mm, alpha=-55 deg`

P77 TOUCH FK:

`x=-27.187401 mm, y=191.028637 mm, z=41.358610 mm, alpha=-67.15 deg`

## 6. P77 delta

- `dx = 3.55e-15 mm` (numerical zero)
- `dy = -2.84e-14 mm` (numerical zero)
- `dz = -50.0634310305 mm`
- `dalpha = -12.15 deg`

Only `dz` is propagated to general points. P77 itself retains the stable
ABOVE/TOUCH actions; its intermediate alpha is fitted between the endpoints.

## 7. Candidate generation count

All 225 points were attempted:

- complete safe five-layer candidates: **153**
- explicitly rejected candidates: **72**
- automatically marked fully verified: **0**

The Stage 6 file also contains the FK snapshot and provenance for all 225
ABOVE poses. After the P77 DRY RUN, verification stages are:

- `DRY_RUN_PASSED`: 1 point (`P77`)
- `COMPUTED`: 152 complete candidates
- rejected: 72 entries in the batch rejection manifest

## 8. Rejected/abnormal point list

Layer joint jump over the configured 400 PWM limit (8):

`P(10,3), P(10,4), P(10,5), P(10,11), P(11,0), P(11,1), P(11,3), P(12,0)`

No IK branch remained inside the configured 550..2450 safety envelope (64):

`P(10,6), P(10,7), P(10,8), P(10,9), P(10,10), P(10,12), P(10,13), P(10,14),`
`P(11,2), P(11,4), P(11,5), P(11,6), P(11,7), P(11,8), P(11,9), P(11,10),`
`P(11,11), P(11,12), P(11,13), P(11,14), P(12,1), P(12,2), P(12,3),`
`P(12,4), P(12,5), P(12,6), P(12,7), P(12,8), P(12,9), P(12,10),`
`P(12,11), P(12,12), P(12,13), P(12,14), P(13,0), P(13,1), P(13,2),`
`P(13,3), P(13,4), P(13,5), P(13,6), P(13,7), P(13,8), P(13,9),`
`P(13,10), P(13,11), P(13,12), P(13,13), P(13,14), P(14,0), P(14,1),`
`P(14,2), P(14,3), P(14,4), P(14,5), P(14,6), P(14,7), P(14,8),`
`P(14,9), P(14,10), P(14,11), P(14,12), P(14,13), P(14,14)`

Their detailed layer and joint errors are retained in
`calibration/stage6_descent_calibration.json`. No fake or clipped PWM was
substituted.

## 9. New files

- `app/stage6/__init__.py`
- `app/stage6/above_source.py`
- `app/stage6/calibration_store.py`
- `app/stage6/kinematics.py`
- `app/stage6/models.py`
- `app/stage6/planner.py`
- `app/stage6/residuals.py`
- `app/stage6/settings.py`
- `app/stage6/state_machine.py`
- `app/stage6/thermal.py`
- `app/gui/stage6_panel.py`
- `config/arm_kinematics.json`
- `config/stage6_descent.json`
- `calibration/stage6_descent_calibration.json`
- `docs/STAGE6_ENGINEERING_AUDIT.md`
- `docs/STAGE6_OFFLINE_DELIVERY_REPORT.md`
- `tests/test_stage6_kinematics.py`
- `tests/test_stage6_planner.py`
- `tests/test_stage6_gui.py`

## 10. Modified files

- `app/gui/control_panel.py`: adds a collapsed Stage 6 panel.
- `app/main_window.py`: wires GUI signals only to the unified planner and adds
  dwell-time thermal warnings.

Stable files that were deliberately not modified:

- `config/arm_actions.json`
- `calibration/stage5_board_calibration.json`
- factory firmware reference
- existing fixed P77 sequences

## 11. Stage 6 calibration structure

The independent JSON contains:

- `source_above`: read-only path and SHA-256
- `above_fk_snapshot`: all 225 ABOVE PWM/provenance/tool poses
- `profiles`: complete generated points
- per level: `computed_pwm`, `manual_delta_pwm`, dynamic `final_pwm`, status
- `history`: timestamped prior versions
- `last_batch_generation`: counts, generated point list, full rejection map
- point verification stage and reverse-ascent verification flag

Regeneration replaces `computed_pwm` but preserves prior manual delta and
verification state. Existing ABOVE delta is hard-rejected.

## 12. GUI adjustment method

The collapsed “阶段六下降标定” panel selects one point and one of five levels.
It shows the stable ABOVE, computed, manual delta, final PWM, and status.

Each spatial joint has only `-10, -5, -1, +1, +5, +10` adjustments. Changes
apply only to the selected level, are safety checked, saved with history, and
can be zeroed or undone. The panel provides generation, batch generation,
preview, ordered descent, one-step ascent, full reverse return, E-stop,
pump-off, and overheat lock controls.

Offline output cannot be marked verified while `force_dry_run` is enabled.

## 13. State machine

Implemented states:

`TARGET_ABOVE, DESCENDING_25, DESCENDING_50, DESCENDING_75, TARGET_TOUCH,`
`RELEASING, RELEASE_DWELL, ASCENDING_75, ASCENDING_50, ASCENDING_25,`
`RETURNED_ABOVE, CARRY_HIGH, OBSERVE, ERROR, EMERGENCY_STOP`

Only the exact descent/ascent transition sequences are accepted. Emergency
stop requires explicit pose recovery; it never guesses a return path.

## 14. Low-position horizontal-move protection

`app/stage6/state_machine.py` owns the target lock. It rejects row/column
changes below ABOVE and also rejects direct `P1_ABOVE -> P2_ABOVE`; a different
target requires `current ABOVE -> CARRY_HIGH -> new ABOVE`.

The GUI mirrors this with disabled row/column selectors and:

`BELOW_ABOVE_LOCKED_TO_P(row,col)`

## 15. Reverse-ascent protection

Each profile stores:

`DESCENT_75 -> DESCENT_50 -> DESCENT_25 -> ABOVE`

The executor reads the exact saved final PWM for those prior descent levels.
It does not recalculate or linearly interpolate an ascent. Direct TOUCH to
OBSERVE and layer skipping are rejected.

## 16. Test result

Dedicated Stage 6 suite:

`25 passed`

This includes the 20 explicitly requested offline cases plus GUI
structure/locking and thermal-dwell checks.

Whole repository:

`147 passed, 5 failed`

All five failures are pre-existing baseline inconsistencies in unchanged
files. One test expects `P77_TOUCH_RELEASE` pump `1500`, while the stable action
required to remain unchanged actually contains `2500`. Other stale tests expect
older pick/Stage5 carry action orders. Stage 6 does not modify those stable
behaviors merely to make old assertions green.

## 17. P77 DRY RUN trajectory

Recorded event path:

`CARRY_HIGH_HOLD`
`-> P77_ABOVE_HOLD`
`-> DESCENT_25`
`-> DESCENT_50`
`-> DESCENT_75`
`-> TOUCH`
`-> PUMP_OFF`
`-> RELEASE_DWELL_700MS`
`-> ASCENDING_75`
`-> ASCENDING_50`
`-> ASCENDING_25`
`-> P77_ABOVE_IDLE`
`-> CARRY_HIGH_IDLE`
`-> OBSERVE_IDLE`

Thirteen protocol commands were recorded by a dry-run controller. The dwell is
a timed plan event, not a serial command.

## 18. Real serial sends

**0 real serial commands.**

The dry-run controller's real connection object remained `None`. There was no
automatic COM connection, camera connection, hardware descent, or release.

## 19. Only next real-world operation

After explicit user confirmation and with the arm physically supported and
cool, enable only the P77 test gate and perform one **empty-tool** P77
five-layer descent followed by the exact reverse ascent. Do not carry or
release a piece, and do not test any other board point.

The evidence gap this experiment addresses is whether the candidate factory
link geometry and configured joint biases reproduce a truly vertical path on
the deployed arm. Time-box the observation to one cycle and immediately return
to ABOVE; stop on any lateral cut, unexpected heat, or posture mismatch.
