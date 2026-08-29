# CURRENT STATUS HANDOFF

Audit date: 2026-08-28 (Asia/Shanghai)

Audit basis: current filesystem, Git metadata, runtime code/configuration, saved calibration artifacts, current logs, a fresh full test run, focused test runs, `tools/project_doctor.py`, and `tools/smoke_test.py`. This document does not infer completion from prior chat statements.

Evidence labels used here:

- **HARDWARE VERIFIED**: explicitly human-confirmed physical result, scoped to the stated pose/action.
- **OFFLINE VERIFIED**: unit/integration/Dry Run/static validation only.
- **NOT VERIFIED**: computed, sent to hardware without trustworthy physical confirmation, incomplete, or otherwise lacking sufficient evidence.

## Executive status

The current Lite software can build and submit a P77 single-point flow, but it is not ready for unattended repeated operation. P77 Golden ABOVE is protected and **HARDWARE VERIFIED** as a pose value. The current P77 transit, Cartesian-Z DROP candidate, individual waypoints, exact reverse, current pickup override, and current full Lite Test PLACE are **NOT VERIFIED** as safe/accurate physical behaviors.

A real non-Dry-Run log on 2026-08-28 records COM6 commands for P77 ABOVE and a full P77 Test PLACE including WP00–WP05 and WP04–WP00 reverse. Those log entries prove command transmission plus fixed-time completion only. There is no servo arrival, position, torque, contact, or suction readback, and the saved Lite DROP record remains `PENDING_VERIFY`, `verified=false`, `verification_level=NOT VERIFIED`.

The next hardware step must remain one supervised, empty-tool transit only:

```text
known OBSERVE_IDLE -> current CARRY_HIGH_P77_IDLE -> P77 Golden ABOVE
```

No DROP, pump, pick, release, reverse, PLACE, or repetition is included.

# 1. Repository / Workspace

## Current repository identity

- Working directory: `D:\Projects\Embodied\RobotArms\J1_Gomoku_Integrated`
- Branch: `stage6-full-board-descent`
- HEAD: `f8411afc6f198a05b1646bebcb8bf0e6a761708e`
- HEAD tag: none
- Last stable tag: `J1_Gomoku_V0.2_Arbitrary_Hover_Stable`
- Last stable commit: `7a1e1253f37c9614a8e528f2d4b01e099e03474f`

Recent key commits:

| Commit | Date | Subject |
| --- | --- | --- |
| `f8411afc6f1` | 2026-07-26 | `fix: require carry-high before stage6 descent` |
| `d7a51cef9a98` | 2026-07-26 | `feat: add stage6 full-board descent candidates` |
| `7a1e1253f37c` | 2026-07-25 | `feat: full-board hover positioning` |
| `c41fe454bcb` | 2026-07-23 | `feat: Stage5 PyTorch shadow learning layer (dataset/train/predict/GUI, no live control)` |
| `70836aa4a86` | 2026-07-22 | `feat: Stage5 cross-anchor calibration wizard with forced dry-run and learning sample export` |

## Git status

The workspace is heavily dirty. Immediately before creating this handoff, `git status --short --untracked-files=all` contained 710 entries:

- 16 modified tracked files.
- 694 untracked files.
- This handoff adds one additional untracked file, so the expected count after creation is 711.

Modified tracked files:

```text
README.md
app/arm/controller.py
app/arm/sequences.py
app/arm/state.py
app/gui/camera_panel.py
app/gui/control_panel.py
app/main.py
app/main_window.py
app/stage5/pwm_interpolator.py
config/arm_kinematics.json
scripts/run_gui.bat
tests/test_action_commands.py
tests/test_safe_sequences.py
tests/test_stage5_planner_state.py
tests/test_stage6_kinematics.py
tests/test_state_machine.py
```

Important untracked groups before this handoff included approximately:

- `logs/`: 301 files.
- `firmware/`: 249 files.
- `app/`: 32 files, including all current `app/calibration_lite/` and `app/integrated_v1/` modules.
- `docs/`: 32 files.
- `calibration/`: 16 files, including the active Lite profile and runtime overrides.
- `tests/`: 9 files.
- `tools/`: 4 files.
- `.tmp*`: 15 files.

The entire current Lite/Integrated milestone is not represented by HEAD. In particular, the following operationally critical files are untracked:

```text
app/calibration_lite/drop_v1.py
app/calibration_lite/drop_v1_view.py
app/calibration_lite/window.py
app/integrated_v1/golden.py
app/integrated_v1/movel.py
app/integrated_v1/robot_controller.py
calibration/calibration_lite_drop_v1.json
calibration/p77.json
calibration/stage7_pick_poses.json
config/calibration_lite.json
CURRENT_STATUS_HANDOFF.md
```

Tracked working-tree diff alone is 16 files, 1,529 insertions and 104 deletions. Untracked files are additional and are not included in that diff statistic.

## Actual Lite launch commands

Hardware-capable Lite launcher:

```powershell
scripts\run_calibration_lite.bat
```

It executes:

```powershell
D:\Anaconda\python.exe -m app.calibration_lite.main
```

Hardware-isolated launcher:

```powershell
scripts\run_calibration_lite_dry_run.bat
```

It executes:

```powershell
D:\Anaconda\python.exe -m app.calibration_lite.main --dry-run
```

Compatibility Python wrapper:

```powershell
D:\Anaconda\python.exe calibration_lite.py [--dry-run]
```

Neither mode auto-connects the camera or serial port. Hardware mode becomes capable of motion only after an explicit COM connection and per-sequence default-No confirmation.

# 2. Current Architecture

## Active Lite runtime

`CalibrationLiteWindow` inherits `MainWindow`. Construction creates one shared `ActionLibrary`, one `SerialArmController`, one `ArmSequenceWorker`, and one `ArmStateMachine`. The Lite page does not own another hardware connection.

Actual Lite motion call chain:

```text
LiteDropV1Panel button/signal
  -> CalibrationLiteWindow handler
  -> LiteDropSequenceBuilder builds SequenceDefinition and runtime Action objects
  -> CalibrationLiteWindow._submit_drop_v1_sequence()
  -> MainWindow._start_sequence()
  -> ArmStateMachine.begin_manual()
  -> ArmSequenceWorker.submit()
  -> ArmSequenceWorker._execute()
  -> ActionLibrary.get()
  -> SerialArmController.send_action()
  -> SerialArmController.write()
  -> pyserial.write() + flush()
  -> ASCII PWM command at 115200 baud
  -> deployed STM32-compatible controller
  -> servos J0–J4 and pump J5
```

## RobotController reality

There are two different controller concepts:

1. `app/arm/controller.py::SerialArmController` is the actual serial owner and hardware write boundary used by Lite.
2. `app/integrated_v1/robot_controller.py::RobotController` is a high-level Integrated V1 placement controller. `MainWindow` instantiates it with the same serial controller, worker, and action library.

The Lite MoveL/PLACE page does **not** call the high-level `RobotController` for normal Lite movement. It calls `LiteDropSequenceBuilder` and the shared worker directly. The high-level controller is used by the Integrated game path and is also used by `MainWindow.emergency_stop()` to cancel and send ESTOP. Therefore the current project does not yet have one unified high-level RobotController API for both Lite and Integrated placement.

## Module responsibilities

| Area | Current implementation |
| --- | --- |
| Lite Calibration UI | `app/calibration_lite/drop_v1_view.py`; emits intent signals and manages button/page presentation only. |
| Lite orchestration | `app/calibration_lite/window.py`; workflow gates, runtime overrides, sequence submission, local pose state, evidence gates. |
| Motion / MoveL | `app/integrated_v1/movel.py::MoveLPlanner`; offline FK/IK candidate generation. `app/calibration_lite/drop_v1.py::LiteDropSequenceBuilder`; runtime sequence construction. |
| Calibration storage | `LiteDropStore` at `calibration/calibration_lite_drop_v1.json`; P77 override at `calibration/p77.json`; pickup override at `calibration/stage7_pick_poses.json`; observe override at `calibration/calibration_lite_observe_pose.json`. |
| Serial / STM32 | `app/arm/controller.py::SerialArmController`; explicit connect, ASCII write/flush, no inbound servo telemetry. |
| Pump | J5 (`005`), `2500=ON/HOLD`, `1500=OFF/RELEASE`; excluded from FK/IK/correction. |
| Worker | `app/arm/worker.py::ArmSequenceWorker`; one-slot queue, rejects concurrency, sends action then waits action duration plus margin. |
| State machine | `app/arm/state.py::ArmStateMachine` plus a separate Lite-local `_drop_pose_state` (`SAFE/ABOVE/WAYPOINT/DROP/UNKNOWN`) and high-level `RobotController.state`. |
| Dry Run | `SerialArmController(dry_run=True)` records commands without creating a pyserial connection. |
| Emergency Stop | Cancels pending worker steps, sends `$DST!` if connected, latches robot/main state and sets Lite pose to `UNKNOWN`. Physical power cutoff remains the final safety control. |

## Serial protocol

- Port hint: `COM6`.
- Baud: `115200` fixed.
- Write timeout: `1.0 s`.
- Multi-servo action example: `{#000P1500T1000!...#007P1500T1000!}`.
- Single-joint correction: `#00xPxxxxT1000!` in the current Lite correction path.
- Pump on: `#005P2500T0500!`.
- Pump off: `#005P1500T0500!`.
- Emergency stop: `$DST!`.
- There is no active read/ACK/parser path in `SerialArmController`.

# 3. Hardware-Verified Facts

## P77 Golden ABOVE

P77 means board `(row=7, col=7)`:

```text
J0 / 000 = 1500
J1 / 001 = 1230
J2 / 002 = 870
J3 / 003 = 1230
J4 / 004 = 1500
J5 / 005 = pump channel, LOCKED, not a spatial pose value
```

This exact P77 ABOVE pose is **HARDWARE VERIFIED**. That label applies to the pose value, not to every current transit used to reach it.

Current persistence/runtime sources:

1. `calibration/p77.json` stores the operator-confirmed P77 `new_pwm`. `CalibrationLiteWindow._load_p77_point_runtime()` loads this after the committed Stage 7 candidate and applies it to runtime P77/carry actions.
2. `app/integrated_v1/golden.py` contains the canonical five Golden ABOVE constants.
3. `calibration/calibration_lite_drop_v1.json` stores P77 under `above.points.P77` with `source=golden_direct_anchor`, `protected=true`, and `verification_level=HARDWARE VERIFIED`.
4. `LiteDropStore.sync_above()` forcibly overlays `golden_for(row,col)` after reading its 225-point source. `assert_golden_above()` is called on profile load and save.
5. `LiteDropSequenceBuilder.build_move_above()` reads `LiteDropStore.above_pwm(P77)`, so the current Lite Move ABOVE endpoint is the protected Golden value.

Regeneration/interpolation protection:

- Lite DROP candidate regeneration cannot change the P77 ABOVE record.
- Lite 225 ABOVE synchronization replaces any P77 source value with the Golden constant.
- Profile load/save fails if a Golden value/source/protected flag is missing or changed.
- The current protection is code/profile-level and is itself uncommitted, so losing the dirty working tree would lose this implementation.

Important conflict: the frozen stable Stage 5 anchor and stable `P77_ABOVE_IDLE` action still contain the older pose `[1560,1170,990,1170,1500]`. Lite intentionally overlays them at runtime with `calibration/p77.json` and the Golden constant. Any path that bypasses the Lite/Integrated overlay can still see the older stable value.

## Other Golden ABOVE anchors

| Legacy ID | Board | J0 | J1 | J2 | J3 | J4 | Evidence |
| --- | --- | ---: | ---: | ---: | ---: | ---: | --- |
| P33 | (3,3) | 1589 | 1136 | 1101 | 1084 | 1500 | **HARDWARE VERIFIED** ABOVE only |
| P311 | (3,11) | 1432 | 1199 | 1157 | 1042 | 1500 | **HARDWARE VERIFIED** ABOVE only |
| P77 | (7,7) | 1500 | 1230 | 870 | 1230 | 1500 | **HARDWARE VERIFIED** ABOVE only |
| P113 | (11,3) | 1630 | 1264 | 588 | 1424 | 1500 | **HARDWARE VERIFIED** ABOVE only |
| P1111 | (11,11) | 1382 | 1258 | 639 | 1410 | 1500 | **HARDWARE VERIFIED** ABOVE only |

No new Lite DROP is **HARDWARE VERIFIED**.

# 4. P77 Current MoveL Implementation

## Calculation path

Implementation locations:

- FK: `app/stage6/kinematics.py::ArmKinematics.forward_kinematics()`.
- IK: `app/stage6/kinematics.py::ArmKinematics.inverse_kinematics()`.
- PWM to angle: `JointCalibration.pwm_to_angle()` in the same file.
- Angle to PWM: `JointCalibration.angle_to_pwm()` in the same file.
- Waypoint generation and continuity: `app/integrated_v1/movel.py::MoveLPlanner.generate_point()` and `_assert_continuity()`.
- Runtime sequence construction: `app/calibration_lite/drop_v1.py::LiteDropSequenceBuilder`.

The active kinematic model is configured in `config/arm_kinematics.json`:

- L0=110 mm, L1=105 mm, L2=75 mm, L3=140 mm.
- Only L3 is described as user measured; L0/L1/L2 and fitted angle biases remain candidate parameters.
- J0–J3 participate in the planar/yaw model.
- J4 is a constant nozzle-rotation passthrough.
- J5 is not in the kinematic joint map.
- The config declares its P77 kinematic calibration `verified=false`.

Calculation steps:

1. Read the protected P77 ABOVE J0–J4 PWM.
2. Convert PWM to joint angles using per-joint zero, direction, `0.135 deg/PWM`, angle bias, and kinematic offset.
3. Run FK to compute P77 tool pose `(x,y,z,alpha)`.
4. Hold `x`, `y`, `alpha`, and J4 constant.
5. Generate descent distances 5, 10, 15, 20, and 25 mm.
6. For each target Z, solve both elbow IK branches.
7. Convert each safe IK solution back to PWM.
8. Select the candidate with minimum squared PWM distance from the previous waypoint seed; round-trip position/alpha errors are secondary tie-breakers.
9. Reject non-finite targets, unreachable IK, PWM/joint limits, round-trip error, J4 change, or a per-waypoint jump over 400 PWM.
10. Save the final auto candidate, correction, final PWM, and reverse indices separately.

Current saved P77 candidate:

| WP | dz mm | J0 | J1 | J2 | J3 | J4 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| WP00 | 0.0 | 1500 | 1230 | 870 | 1230 | 1500 |
| WP01 | 5.0 | 1500 | 1229 | 842 | 1258 | 1500 |
| WP02 | 10.0 | 1500 | 1228 | 816 | 1286 | 1500 |
| WP03 | 15.0 | 1500 | 1224 | 792 | 1314 | 1500 |
| WP04 | 20.0 | 1500 | 1220 | 769 | 1341 | 1500 |
| WP05 | 25.0 | 1500 | 1214 | 747 | 1369 | 1500 |

Saved profile state:

```text
drop_auto_pwm       = WP05
drop_correction_pwm = {J0:0, J1:0, J2:0, J3:0, J4:0}
drop_final_pwm      = WP05
status              = PENDING_VERIFY
verified            = false
verification_level  = NOT VERIFIED
reverse indices     = [4,3,2,1,0]
```

## Execution semantics

- Runtime motion time is `1000 ms` per action from `config/calibration_lite.json`.
- Worker wait is `action.duration_ms + 200 ms`, so normally 1.2 seconds before sending the next action.
- There is no configured velocity/acceleration profile beyond the protocol `T1000` field.
- Full DROP sends WP00, WP01, WP02, WP03, WP04, and WP05 as six individual multi-servo commands.
- Step mode sends one adjacent waypoint action per operator click. Leaving a corrected final DROP sends auto WP05 first, then the previous waypoint.
- Each endpoint was solved from Cartesian Z, but the host does not generate a continuous Cartesian trajectory between endpoints. It sends joint PWM endpoints; the deployed controller/servos perform whatever interpolation the `T1000` protocol implements. This is piecewise joint-space interpolation through Cartesian-solved nodes, not feedback-controlled continuous MoveL.

Exact Reverse:

- With zero correction, P77 reverse is WP04 -> WP03 -> WP02 -> WP01 -> WP00.
- With a non-zero final correction, reverse first sends the auto WP05, then WP04 -> WP03 -> WP02 -> WP01 -> WP00.
- Partial waypoint return uses `current-1 ... WP00` only.
- No IK recomputation or direct DROP-to-ABOVE shortcut is used.

Correction storage:

- `LiteDropStore.save_correction()` preserves `drop_auto_pwm`.
- It stores a separate J0–J4 `drop_correction_pwm`.
- It recomputes `drop_final_pwm = auto + correction` under joint limits.
- Saving correction clears prior verification and restores `NOT VERIFIED`.
- J5 cannot be corrected.

# 5. P77 Verification State

| Item | Software / Dry Run | Real-command evidence | Physical evidence state |
| --- | --- | --- | --- |
| P77 ABOVE pose value | Protected, profile-validated | Used by real commands | **HARDWARE VERIFIED** for the pose value only |
| Safe/current -> P77 ABOVE transit | Sequence construction/tests pass; Dry Run capable | COM6 sequence sent at 12:37:10 on 2026-08-28; log time-completed | **NOT VERIFIED**; user reported unresolved downward/board-strike risk |
| WP00 | Candidate/profile/tests pass | Sent during live Test PLACE | Same as P77 ABOVE endpoint; transit into it **NOT VERIFIED** |
| WP01 | FK/IK/continuity and Dry Run pass | Sent at 12:40:06 during live Test PLACE | **NOT VERIFIED** |
| WP02 | FK/IK/continuity and Dry Run pass | Sent at 12:40:07 during live Test PLACE | **NOT VERIFIED** |
| WP03 | FK/IK/continuity and Dry Run pass | Sent at 12:40:09 during live Test PLACE | **NOT VERIFIED** |
| WP04 | FK/IK/continuity and Dry Run pass | Sent at 12:40:10 during live Test PLACE | **NOT VERIFIED** |
| WP05 | FK/IK/continuity and Dry Run pass | Sent at 12:40:11 during live Test PLACE | **NOT VERIFIED** |
| P77 Golden DROP | No such verified artifact exists | WP05 candidate was sent | **NOT VERIFIED**; `drop_final_pwm` is a candidate, not Golden DROP |
| Exact Reverse | Sequence order and Dry Run pass | WP04 -> WP00 sent at 12:40:14–12:40:19 | **NOT VERIFIED** |
| Full current P77 Lite PLACE | Sequence construction and Dry Run pass | Full sequence sent on COM6 and log time-completed at 12:40:20 | **NOT VERIFIED** |

The real session log is `logs/session_20260828_120827.log`. Its own Lite evidence line records `evidence=NOT VERIFIED`.

`calibration/calibration_lite_place_pose.json` contains an older `verified=true` layered-PWM contact candidate. It is explicitly `motion_kind=layered_pwm_descent_candidate_not_cartesian_movel`, uses an older ABOVE value with J1=1220, and is not the current Cartesian MoveL DROP. It must not be called P77 Golden DROP.

# 6. Safety Problem Observed

The user observed a downward/smashing risk. Current evidence does not identify whether it occurred in A or B. Do not assign a cause without a bounded physical observation.

## A. Current Move ABOVE transit

For live Lite Move ABOVE, the software requires main state `OBSERVE_IDLE` and Lite-local pose `SAFE`. It then executes:

```text
current OBSERVE_IDLE
  -> CARRY_HIGH_P77_IDLE = [1500,1290,870,1230,1500], J5 off
  -> P77 Golden ABOVE    = [1500,1230,870,1230,1500], J5 off
```

The current saved OBSERVE override is `[1820,1400,710,1230,1500]` for J0–J4.

Properties of this transit:

- It is not Cartesian MoveL.
- It sends a simultaneous J0–J4 joint target to `CARRY_HIGH_P77_IDLE`, then a second simultaneous joint target to ABOVE.
- `CARRY_HIGH_P77_IDLE` is derived from Golden P77 by increasing only J1 by 60 PWM; the name does not prove a measured TCP clearance.
- There is no separate current-pose vertical lift before horizontal/joint reconfiguration.
- `build_move_above()` does not use the existing `j1_last_sequence()` staging helper.
- There is no minimum TCP Z/height check on the transit path.
- There is no collision model or swept-volume check.
- The real 12:37 transit started from an earlier OBSERVE override; the current OBSERVE override was edited later in that session. Therefore that run does not fully validate today's exact start-to-carry transition.

## B. Current ABOVE -> DROP descent

- Cartesian x/y/alpha are held in the offline endpoint calculations.
- Five 5 mm endpoints are generated below WP00.
- The endpoints have FK/IK continuity and PWM-limit checks.
- The physical interpolation between endpoints is not measured or controlled in Cartesian space.
- There is no minimum TCP height/contact-plane guard.
- There is no board/contact sensor.
- The configured kinematic geometry is still partly candidate/fitted and has `verified=false` for P77 calibration.

## Timing and completion

- Every Lite pose action uses `T1000`.
- The host waits 1,000 ms plus a 200 ms margin before sending the next action.
- The worker prevents two host sequences from being submitted concurrently.
- Serial `flush()` proves bytes were handed to the serial layer, not that a servo arrived.
- There is no servo position readback.
- There is no servo-arrival ACK.
- There is no velocity/trajectory feedback.
- There is no torque or load feedback.
- There is no suction/vacuum success feedback.
- If a loaded servo has not physically completed within the fixed 1.2-second host wait, the next waypoint can be sent while the mechanism is still moving. The code cannot detect this condition.

The code therefore does not determine whether the observed risk was caused by transit A, descent B, load/sag, model error, timing, or another physical factor.

# 7. Current Lite UI

| Function | Status | Current behavior |
| --- | --- | --- |
| Point selection | IMPLEMENTED | Row/column selection; switching is allowed only while Lite pose is `SAFE`. |
| Load ABOVE | IMPLEMENTED | Loads saved Lite ABOVE and advances to Step 2 if a candidate is executable. |
| Generate DROP | IMPLEMENTED | Offline planner operation; persists a candidate/failure record. |
| Preview MoveL | IMPLEMENTED | Displays XYZ, dz, PWM, and reverse path; no hardware action. |
| Move ABOVE | IMPLEMENTED | `CARRY_HIGH_P77_IDLE -> selected ABOVE`; live requires known `OBSERVE_IDLE` and confirmation. Safety limitations are listed above. |
| Move DROP | IMPLEMENTED | Full WP00–final sequence, only after ABOVE confirmation. |
| Retract | IMPLEMENTED | Exact partial/full saved reverse path. |
| PWM correction | IMPLEMENTED | J0–J4 single-joint live apply; J5 locked. |
| Save correction | IMPLEMENTED | Saves correction separately and invalidates prior verification. |
| Test PLACE | IMPLEMENTED | Pickup -> carry -> waypoints -> release -> exact reverse; ends at ABOVE. |
| Safe Return | IMPLEMENTED | ABOVE -> J1+60 lift -> carry -> OBSERVE; no minimum-TCP/readback validation. |
| Confirm Hardware Verified | IMPLEMENTED | Enabled only after Step 5 Test PLACE success and live eligibility; still human/open-loop evidence. |
| Waypoint step mode | IMPLEMENTED | Adjacent saved waypoint per click. |
| Next WP | IMPLEMENTED | Enabled from ABOVE/intermediate waypoint until final. |
| Previous WP | IMPLEMENTED | Enabled on path; corrected final first returns via auto final. |
| Emergency Stop | IMPLEMENTED | Visible outside wizard pages, cancels worker and sends `$DST!`; pose becomes unknown. |

All advanced PWM/candidate/Cartesian/225 statistics are collapsed by default. Only the current wizard step is displayed. UI implementation does not imply hardware safety verification.

# 8. Pick Sequence

## Standalone Lite Pick button

The current standalone pick execution is:

```text
START at OBSERVE_IDLE
  -> LITE_PICK_EXEC_00_J1_HELD
     target J0/J2/J3/J4 first while holding J1 at current OBSERVE J1
  -> LITE_PICK_EXEC_00_J1_LAST
     send final PICK_DOWN J1
  -> SOURCE_TOUCH_HOLD
     same spatial PICK_DOWN pose, J5=2500
  -> wait for action completion estimate
  -> VACUUM BUILD 700 ms
  -> OBSERVE_HOLD
     joint-space lift/return with J5=2500
END at OBSERVE_HOLD
```

Current pickup spatial source:

- File: `calibration/stage7_pick_poses.json`.
- Record: `PICK_DOWN.new_pwm = [1805,1080,595,1500,1300]` for J0–J4.
- Record status: `OFFLINE / NOT HARDWARE VERIFIED`, `verified=false`.
- Runtime: both `SOURCE_TOUCH_IDLE` and `SOURCE_TOUCH_HOLD` receive this same spatial PWM; J5 differs.
- The legacy stable pickup flow has prior hardware evidence, but the current runtime PICK_DOWN override differs from the stable table. The current override is **NOT VERIFIED**.

Pump timing:

- `SOURCE_TOUCH_HOLD` is a `T1000` action and worker waits 1.2 s.
- Then an explicit 700 ms `VACUUM BUILD` wait occurs.
- Total scheduled time from pump-on send to lift command is approximately 1.9 s.

Safe lift:

- The next action is `OBSERVE_HOLD`, a simultaneous multi-joint target.
- There is no vertical-only Cartesian lift, load feedback, or pickup confirmation.
- Suction is open-loop; no vacuum sensor confirms a piece was acquired.

Important divergence: `LiteDropSequenceBuilder.build_test_place()` inserts the raw `pick_piece()` steps and does not call `_prepare_pick_execution_sequence()`. Therefore current Test PLACE starts with direct `SOURCE_TOUCH_IDLE` rather than the standalone Pick button's J1-held/J1-last approach. Existing standalone pick ordering protection is not applied to the current Lite Test PLACE pickup approach.

# 9. Place Sequence

## Current Lite equivalent of place_piece(P77)

The active Lite page uses:

```text
CalibrationLiteWindow.test_place_drop_v1()
  -> LiteDropSequenceBuilder.build_test_place(P77)
  -> CalibrationLiteWindow._submit_drop_v1_sequence()
  -> shared ArmSequenceWorker
```

With the current zero correction, the exact action list is:

1. `SOURCE_TOUCH_IDLE` — current PICK_DOWN spatial pose, J5 off.
2. `SOURCE_TOUCH_HOLD` — same spatial pose, J5 on.
3. `VACUUM BUILD` — explicit 700 ms wait after the action's own 1.2-second scheduled wait.
4. `OBSERVE_HOLD` — pickup lift/return, J5 on.
5. `CARRY_HIGH_P77_HOLD` — `[1500,1290,870,1230,1500]`, J5 on.
6. `LITE_DROP_P07_07_WP_00_HOLD` — Golden ABOVE, J5 on.
7. `LITE_DROP_P07_07_WP_01_HOLD` — J5 on.
8. `LITE_DROP_P07_07_WP_02_HOLD` — J5 on.
9. `LITE_DROP_P07_07_WP_03_HOLD` — J5 on.
10. `LITE_DROP_P07_07_WP_04_HOLD` — J5 on.
11. `LITE_DROP_P07_07_WP_05_HOLD` — current auto/final DROP candidate, J5 on.
12. `LITE_DROP_P07_07_DROP_FINAL_RELEASE` — same final spatial pose, J5 off.
13. `VACUUM RELEASE` — explicit 700 ms wait after the release action's own 1.2-second scheduled wait.
14. `LITE_DROP_P07_07_WP_04_IDLE` — J5 off.
15. `LITE_DROP_P07_07_WP_03_IDLE` — J5 off.
16. `LITE_DROP_P07_07_WP_02_IDLE` — J5 off.
17. `LITE_DROP_P07_07_WP_01_IDLE` — J5 off.
18. `LITE_DROP_P07_07_WP_00_IDLE` — ends at P77 ABOVE, J5 off.

If a nonzero correction exists, an additional corrected `DROP_FINAL_HOLD` is inserted before release, and reverse first returns through auto WP05.

The Lite Test PLACE does not automatically execute `Safe Return`; it ends at P77 ABOVE. Safe Return is a separate operator action.

## Integrated `RobotController.place_piece(P77)`

The high-level game path exists at:

```text
MainWindow.v1_game_place(point_id)
  -> RobotController.place_piece(point_id)
  -> RobotController.gate()
  -> RobotController._build_place_sequence()
  -> shared ArmSequenceWorker
```

This path uses the separate Integrated profile, not `calibration/calibration_lite_drop_v1.json`. The current doctor reports no saved valid Integrated V1 profile (`FIRST_SETUP`), and live placement requires a `HARDWARE VERIFIED` DROP. Therefore live game `place_piece(P77)` is currently gated and is not the active Lite hardware test path.

# 10. Tests

Fresh audit results:

- Full suite: `225 passed in 25.77s` — **OFFLINE VERIFIED**, no real hardware.
- Lite DROP/safety/state focused run: `27 passed in 8.59s` — **OFFLINE VERIFIED**, no real hardware.
- Lite runtime/override focused run: `10 passed in 5.82s` — **OFFLINE VERIFIED**, no real hardware.
- Safe smoke: `10 checks passed` — **OFFLINE VERIFIED**, explicitly no camera, real serial, worker thread, or hardware motion.
- Project doctor: ready for safe development; stable hashes, Golden guards, Lite J5 lock, zero verified DROP points passed. Warnings remain for dirty tree, lack of project-wide force-dry-run, and missing camera intrinsics.

## Current Lite DROP tests

All tests below passed. None uses real hardware.

| Test name | What it verifies | Mode |
| --- | --- | --- |
| `test_lite_store_preserves_five_golden_above_and_excludes_j5` | Golden protection, profile safety flags, no J5 pose data | Offline |
| `test_movel_uses_cartesian_z_waypoints_continuous_ik_and_exact_reverse` | P77 waypoints, continuity limit, J4 constant, reverse indices | Offline |
| `test_unreachable_waypoint_is_blocked_before_motion` | Unsafe discontinuity becomes unreachable | Offline |
| `test_auto_drop_correction_and_verification_are_separate` | auto/correction/final/evidence separation | Offline |
| `test_sequences_lock_j5_and_test_place_retracts_exact_path` | J5 lock, waypoint/partial reverse, full Test PLACE pump/reverse order | Offline |
| `test_previous_waypoint_leaves_corrected_drop_through_auto_final` | Corrected final backs out through auto final | Offline |
| `test_wizard_reveals_only_current_step_and_enforces_place_gate` | Wizard/button/Test PLACE/final-verification gates and ESTOP availability | Offscreen UI |
| `test_generate_225_is_offline_and_records_per_point_failures` | Batch has no controller path | Offline |
| `test_lite_cli_and_window_support_dry_run_without_auto_connect` | Dry Run startup and no auto-connect | Dry Run/offscreen UI |
| `test_window_refuses_verification_before_test_place` | Final evidence cannot be saved before Test PLACE | Dry Run/offscreen UI |
| `test_safe_return_from_midflow_resumes_at_above_confirmation` | Safe Return resumes at Step 2 | Dry Run/offscreen UI |

## Current Lite runtime/override tests

All passed, no real hardware:

- `test_quick_five_wizard_uses_required_order_and_automatic_transitions`
- `test_inaccurate_test_returns_only_current_point_to_anchor_correction`
- `test_lite_view_hides_advanced_controls_and_locks_pump`
- `test_summary_ignores_empty_deployment_session_path`
- `test_lite_window_never_auto_connects_hardware`
- `test_lite_pose_guard_rejects_uninitialized_all_minimum_pwm`
- `test_observe_override_preserves_stable_table_and_shares_pick_above`
- `test_place_contact_uses_latest_above_and_persists_only_correction`
- `test_manual_pump_commands_have_explicit_on_and_off_states`
- `test_p77_dedicated_file_contains_new_pwm_only`

## Focused ordering/protocol/state tests

All passed, no real hardware:

- `test_j1_is_held_until_other_spatial_axes_reach_target`
- `test_j1_only_transition_is_not_duplicated`
- `test_pick_sequence_never_drops_vacuum_after_hold`
- `test_pick_sequence_rejects_skipping_the_safe_source_approach`
- `test_pick_retry_turns_pump_off_at_shared_above_before_descending`
- `test_place_sequence_enforces_safe_p77_approach_and_exit`
- `test_full_cycle_contains_no_direct_observe_touch_transition`
- `test_full_cycle_p77_commands_share_the_same_calibrated_base_pwm`
- `test_all_actions_are_complete_valid_ascii_commands`
- `test_hold_idle_release_and_reserved_channel_invariants`
- `test_all_calibrated_p77_path_actions_use_base_1560_and_never_1580`
- `test_unknown_cannot_pick_and_observe_idle_can_pick`
- `test_failed_pick_can_be_retried_repeatedly_from_observe_hold`
- `test_only_observe_hold_with_board_lock_can_place`
- `test_second_action_is_rejected_while_busy`
- `test_estop_is_latched_and_never_restores_pick_permission_directly`

Important test gaps:

- The stable action test deliberately checks legacy P77 base 1560, while current Lite P77 runtime is Golden base 1500.
- Ordered-motion tests prove the helper, but current Lite `build_move_above()` does not use that helper.
- Standalone Pick tests prove the J1-held/J1-last wrapper, but current Lite Test PLACE bypasses that wrapper.
- No automated test can validate servo sag, physical TCP height, collision clearance, suction success, real motion completion, or board contact.

# 11. Known Risks / TODO

Only currently evidenced risks are listed:

1. **Unsafe transit unresolved:** current `OBSERVE_IDLE -> CARRY_HIGH_P77 -> ABOVE` is a two-target joint-space transit with no minimum TCP height or swept collision validation.
2. **Observed downward/board-strike risk unresolved:** current evidence cannot distinguish transit A from MoveL descent B.
3. **Servo load/sag unobserved:** no position, load, torque, velocity, or current telemetry is consumed by the host.
4. **Fixed-time sequencing:** next commands are sent after 1.2 seconds regardless of physical arrival. Under load, mechanical movement can still be in progress.
5. **MoveL is waypoint-based, not continuous Cartesian servo control:** only endpoints are Cartesian-solved; interpolation between them is joint-space/protocol behavior.
6. **Kinematic model remains candidate:** L0/L1/L2 and biases are not fully physically validated; P77 kinematic calibration is marked `verified=false`.
7. **No Golden DROP:** current WP05/final is `PENDING_VERIFY`, not a physically confirmed endpoint.
8. **Pump is open-loop:** no vacuum sensor or piece-acquired feedback; pump timing is fixed.
9. **Current pickup override is not hardware verified:** `stage7_pick_poses.json` explicitly says offline/not verified.
10. **Test PLACE pick-path divergence:** standalone Pick uses J1-held/J1-last approach, but Test PLACE currently starts with raw `SOURCE_TOUCH_IDLE`.
11. **Multiple state layers:** high-level `RobotController`, main `ArmStateMachine`, and Lite-local pose state can disagree; manual Lite completion sets main state to `UNKNOWN` while local state becomes ABOVE/DROP.
12. **Serial completion semantics:** write/flush and elapsed time are treated as sequence completion; there is no firmware/servo arrival ACK.
13. **Recovery is manual after uncertainty/ESTOP:** Lite marks pose `UNKNOWN`; code cannot rediscover physical joint position.
14. **Source-of-truth divergence:** frozen Stage 5/stable P77 values differ from current Golden P77; paths must use the Lite/Integrated overlay consistently.
15. **Stale legacy place artifact:** `calibration_lite_place_pose.json` has `verified=true` but is not the current MoveL DROP and can be misread as current evidence.
16. **Large uncommitted workspace:** current operational code, configuration, calibration profiles, tests, firmware copy, and logs are mostly untracked. Reproducibility and review are at risk.
17. **No unattended-cycle implementation/evidence:** there is no current validated 3/5/10/20-cycle progression, no automatic fault detection, and no evidence supporting 20 uninterrupted cycles.
18. **UI verification is human/open-loop:** final confirmation can persist evidence only after software sequencing gates, but it cannot prove physical arrival or collision-free execution.

# 12. Recommended NEXT SMALLEST HARDWARE TEST

## One experiment only: known OBSERVE_IDLE -> P77 Golden ABOVE

The current Lite UI can isolate this command without executing DROP, pump, pick, place, or repetition. It cannot claim that the transit is automatically safe because it lacks minimum-TCP and arrival feedback.

Preconditions:

1. One operator, empty gripper, no piece, J5 confirmed off, empty board/work volume, physical power cutoff in hand.
2. Robot must already be physically confirmed at the current saved `OBSERVE_IDLE = [1820,1400,710,1230,1500]`. The software cannot safely infer or recover from an arbitrary current pose. If this pose is not known, stop; the missing capability is homing/position readback or a separately validated recovery move.
3. No camera is required. Do not start any Pick, Test PLACE, DROP, correction, Safe Return, or repeated cycle.
4. Record video from a side view that shows both the TCP and board plane, with timestamps sufficient to distinguish the first transit segment from the second.

Procedure:

1. Start `scripts\run_calibration_lite.bat`.
2. Explicitly connect only the intended COM port.
3. Open the single-point wizard.
4. Step 1: select `(7,7)` / P77 and load saved ABOVE. Confirm the displayed endpoint is `[1500,1230,870,1230,1500]` and `PROTECTED GOLDEN`.
5. Step 2: press `1. Move ABOVE` once and accept the default-No live confirmation only when the cutoff operator is ready.
6. Observe separately:
   - segment A1: `OBSERVE_IDLE -> CARRY_HIGH_P77_IDLE [1500,1290,870,1230,1500]`;
   - segment A2: `CARRY_HIGH_P77_IDLE -> P77 Golden ABOVE [1500,1230,870,1230,1500]`.
7. On any downward excursion toward the board, vibration, sag, unexpected lateral sweep, or delayed movement, use the physical cutoff/ESTOP and record which segment was active.
8. If it reaches P77 ABOVE, stop the experiment there. Do not click ABOVE confirmation, Next Waypoint, Full DROP, Safe Return, Pick, or Test PLACE as part of this experiment.

Pass evidence for this one experiment requires human observation that both transit segments were collision-free and the endpoint matched P77 Golden ABOVE. Even a pass validates only one supervised empty-tool transit. It does not validate MoveL, DROP, pump, Pick, PLACE, repeatability, or unattended operation.

Capabilities still missing before this transit can be called autonomously safe include a physically validated safe-transit trajectory or minimum TCP height envelope and trustworthy motion/arrival feedback. No 3/5/10/20-cycle test should start before the single transit, single waypoint progression, exact reverse, and single PLACE have separate bounded physical passes.

READY FOR CHATGPT TECH-LEAD REVIEW
