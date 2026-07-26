# Stage 6 engineering audit

Audit date: 2026-07-25

Baseline commit: `7a1e1253f37c9614a8e528f2d4b01e099e03474f`

Stable tag: `J1_Gomoku_V0.2_Arbitrary_Hover_Stable`

Development branch: `stage6-full-board-descent`

This audit is read-only with respect to the stable Stage 5 calibration and the
factory P77 action table. No serial command was sent while producing it.

## Executive findings

- The repository does **not** persist 225 independent verified ABOVE records.
  It persists 30 calibrated `TARGET_ABOVE` anchors in
  `calibration/stage5_board_calibration.json`; the other 195 board points are
  resolved at runtime by bilinear interpolation. All 225 currently resolve,
  but their provenance must remain distinguishable.
- The persisted Stage 5 calibration file was hashed before and after the
  225-point read audit. Both hashes were
  `21B4994D75E03ACCAF8D95B85A9577A9A41940370D2CE3C82F1F87AEB2EE8052`.
- The copied factory source contains inverse kinematics in
  `firmware/stm32_j1_reference/src/z_kinematics.c`, but no forward-kinematics
  implementation.
- The factory kinematics initialization call uses link lengths
  `110, 105, 75, 190` mm. Its adjacent comment lists a conflicting set
  (`90, 105, 98, 150` mm), so the call arguments are the candidate source and
  the physical lengths remain an explicit real-world evidence gap.
- The host action table and controller agree that servo `005` is the pump:
  `2500` means hold/vacuum and `1500` means off/release.
- `P77_TOUCH_RELEASE` currently contains pump PWM `2500`, identical to
  `P77_TOUCH_HOLD`. This conflicts with its name and with the host pump-off
  convention. Stage 6 must not edit the stable action; it must use an explicit,
  separately generated pump-off step.
- Servo `004` is excluded from the factory IK. Factory PS2 mappings move it
  independently, while board ABOVE calibration values use it as an auxiliary
  wrist-orientation channel. Its exact physical axis is not proven by the
  available source.

## Required 17-point audit

### 1. ABOVE PWM location

The authoritative persisted anchors are in:

`calibration/stage5_board_calibration.json`

The host loader is:

`app/stage5/calibration_store.py::CalibrationStore`

There are 30 persisted calibrated anchors. A complete 15x15 set is resolved
without modifying the file by:

`app/stage5/pwm_interpolator.py::resolve_target_pwm`

Current resolution result:

- direct calibrated anchors: 30
- bilinear interpolation: 195
- total resolved points: 225
- failures: 0

### 2. ABOVE data structure

The data is a combination:

1. JSON-backed calibrated anchors (`AnchorPose`);
2. direct-anchor lookup when a calibrated point exists;
3. tightest-cell bilinear interpolation for intermediate points;
4. star/outer seed fallbacks for special expansion cases.

Stage 6 must snapshot the resolved input and its provenance. It must never
overwrite the Stage 5 JSON while generating descent candidates.

### 3. Actual P77 ABOVE PWM

Spatial pose (`000..004`):

`[1560, 1170, 990, 1170, 1500]`

`P77_ABOVE_IDLE` adds pump `005=1500`; `P77_ABOVE_HOLD` adds
`005=2500`. Channels `006` and `007` remain `1500`.

### 4. Actual P77 TOUCH HOLD PWM

Spatial pose (`000..004`):

`[1560, 1050, 980, 1210, 1500]`

Pump: `005=2500`.

### 5. Actual P77 TOUCH RELEASE PWM

Spatial pose (`000..004`):

`[1560, 1050, 980, 1210, 1500]`

Pump in the current action table: `005=2500`.

This is not an off/release value under the current host convention. The stable
action is preserved, and Stage 6 will generate a separate explicit pump-off
step with `005=1500`.

### 6. Current P77 ascent

After the named release step and release wait, `place_to_p77()` uses:

1. `P77_ABOVE_IDLE`
2. `CARRY_HIGH_P77_IDLE`
3. `OBSERVE_IDLE`

The current flow has no intermediate descent/ascent layers.

### 7. Current CARRY_HIGH actions

`CARRY_HIGH_P77_HOLD` spatial PWM:

`[1560, 1180, 980, 1170, 1500]`, pump `2500`

`CARRY_HIGH_P77_IDLE` spatial PWM:

`[1560, 1180, 980, 1170, 1500]`, pump `1500`

Stage 5 also generates runtime `CARRY_HIGH_LIFTED_*` actions by raising joint
`001`. These are runtime-only and do not change `config/arm_actions.json`.

### 8. Pump control

- pump servo ID: `005`
- vacuum/hold: PWM `2500`
- off/release: PWM `1500`
- explicit off command: `#005P1500T0500!`
- implementation: `SerialArmController.pump_off()`

Pump ID `005` must never enter FK or IK.

### 9. Serial sender

`app/arm/controller.py::SerialArmController`

Properties:

- 115200 baud;
- ASCII factory protocol;
- no automatic connection on construction;
- synchronized writes under an `RLock`;
- DRY RUN records commands without touching a serial connection;
- emergency stop command is `$DST!`.

### 10. Stage 5 interpolator

`app/stage5/pwm_interpolator.py`

Priority:

1. direct calibrated anchor;
2. bilinear interpolation from the tightest calibrated rectangle;
3. star seed;
4. outer-ring seed.

It interpolates only spatial PWM (`000..004`). It is an ABOVE resolver, not a
Cartesian descent planner.

### 11. Existing forward kinematics

No host or factory FK implementation was found.

Stage 6 must add FK as the mathematical inverse of the selected, configured
factory-compatible joint convention.

### 12. Existing inverse kinematics

Factory source:

`firmware/stm32_j1_reference/src/z_kinematics.c::kinematics_analysis`

It solves base rotation plus a planar shoulder/elbow/wrist chain. The factory
`kinematics_move()` scans tool angle `Alpha` from 0 to -135 degrees and writes
servos `000..003`.

### 13. Existing link lengths

The executed initialization call is:

`setup_kinematics(110, 105, 75, 190, &kinematics)`

Units are documented as millimetres. The contradictory nearby comment means
these values are candidates, not physically re-measured facts.

### 14. PWM/angle conversion

Factory conversion uses `2000 PWM = 270 degrees`:

- `000`: `pwm = 1500 - 2000 * angle / 270`
- `001`: `pwm = 1500 + 2000 * angle / 270`
- `002`: `pwm = 1500 + 2000 * angle / 270`
- `003`: `pwm = 1500 + 2000 * angle / 270`

The stable host poses indicate that at least one installed-axis convention may
differ from this copied reference. Stage 6 therefore makes zero, direction,
scale, and angular bias explicit configuration and labels the initial model as
candidate-only.

### 15. Bias/zero correction

The factory firmware stores `dj_bias_pwm[]` in EEPROM and adds it when commands
are parsed. The actual deployed EEPROM values are not available to the host.
The current Python host has no kinematic bias configuration.

Stage 6 must not pretend the unknown EEPROM values are zero-proven. Host model
biases will be explicit and versioned.

### 16. Actual role of ID004

Evidence:

- factory IK writes only `000..003`;
- PS2 mappings drive `004` independently;
- Stage 5 ABOVE calibration includes non-1500 values for `004`.

Inference: `004` is an auxiliary wrist orientation/rotation axis that does not
participate in the factory planar `x,y,z,alpha` solve. Its precise physical axis
requires operator or mechanical confirmation. Stage 6 preserves the existing
ABOVE value through every descent layer and excludes it from IK.

### 17. Current P77 complete flow and states

Host action sequence:

1. `CARRY_HIGH_P77_HOLD`
2. `P77_ABOVE_HOLD`
3. `P77_TOUCH_HOLD`
4. `P77_TOUCH_RELEASE`
5. `VACUUM RELEASE` wait
6. `P77_ABOVE_IDLE`
7. `CARRY_HIGH_P77_IDLE`
8. `OBSERVE_IDLE`

Arm state transition:

`OBSERVE_HOLD -> PLACING_P77 -> OBSERVE_IDLE`

There is no explicit low-level state, layer order enforcement, row/column lock,
reverse-path proof, thermal lock, or multi-layer vertical descent in the
current fixed P77 flow.

## Safety containment before implementation

- No factory firmware changes.
- No automatic connection or motion.
- No edits to `config/arm_actions.json`.
- No edits to existing Stage 5 ABOVE PWM data.
- Stage 6 live movement remains forced off until user confirmation.
- Generated candidates start at `COMPUTED`, never `VERIFIED`.
- P77 is the only permitted first real-world regression point after offline
  completion and explicit user confirmation.
