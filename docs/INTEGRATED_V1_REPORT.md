# Gomoku Robot Integrated V1 Delivery Report

Date: 2026-08-27

## Outcome

Integrated V1 is implemented as an offline-verified working-tree milestone. It combines the existing PySide6 vision/control host with the game board and AI, while retaining the existing successful fixed-P77 sequence and its controller/worker execution path. No camera, serial port, firmware or robot was used during this milestone.

First Setup render evidence: `docs/audits/integrated_v1_20260827/01_first_setup.png`.

Evidence boundary:

- **HARDWARE VERIFIED:** the user-confirmed five Golden ABOVE anchors and the inherited legacy fixed-P77 pick/place flow only.
- **OFFLINE VERIFIED:** V1 profile, point IDs, 225-point planning, safety gates, UI routing, game adapter and dry-run execution.
- **NOT VERIFIED:** every newly generated DROP, correction, fast-calibration result and V1 live game placement until the checklist is run point by point.

## Architecture

```text
Camera / vision matrix ───────────────┐
                                      v
GameSession ── explicit point_id ── RobotController ── ArmSequenceWorker ── SerialArmController
                                      ^                       ^                     ^
                                      │                       │                     │
15×15 calibration UI ── Profile ── MoveL planner              └──── same existing instances ────┘
                         │
                         ├─ 225 ABOVE records
                         ├─ DROP auto + correction + final
                         └─ verification/provenance/history
```

`RobotController` is the single high-level motion API. It receives the already-created action library, sequence worker and serial controller from `MainWindow`; it does not create a second serial path. `Generate All` is a pure planner operation with no controller reference.

## Existing Code Reused

- Stable pick/carry/release/return ordering from `app/arm/sequences.py`.
- `SerialArmController`, `ArmSequenceWorker`, action library and arm state handling.
- Stage 5 15×15 ABOVE resolution as the immutable source snapshot.
- Stage 6 forward/inverse kinematics and tool-pose types for Cartesian MoveL candidates.
- Existing PySide6 camera, board localization, piece recognition and application shell.
- Board rules and tactical AI adapted from `D:\Projects\Embodied\RobotArms\gomoku_project`; its PyQt5 UI and separate `STM32Controller` hardware adapter were deliberately not retained.

## Modified and Added Files

Core additions are under `app/integrated_v1/` (`golden.py`, `points.py`, `profile.py`, `movel.py`, `robot_controller.py`, `settings.py`), `app/game/`, `app/gui/game_panel.py`, and `app/gui/drop_calibration_panel.py`. Integration changes are in `app/main.py`, `app/main_window.py`, and `app/gui/rapid_calibration_panel.py`. Operator configuration/entry points are `config/integrated_v1.json`, `start_gomoku_robot.bat`, and `scripts/run_gomoku_robot_dry_run.bat`. Tests are in `tests/test_integrated_v1_backend.py` and `tests/test_integrated_v1_game_gui.py`.

The frozen `calibration/stage5_board_calibration.json` and `config/arm_actions.json` were not edited. Their hashes remain:

- Stage 5: `21B4994D75E03ACCAF8D95B85A9577A9A41940370D2CE3C82F1F87AEB2EE8052`
- Actions: `9B0A888C47DCFEC04C82A5D1C38D646334BBA46B3E2874DF630BBB142DA3FB7F`

## Golden ABOVE Anchors

The V1 runtime profile overlays the user-confirmed Golden values and records every mismatch with the immutable Stage 5 source. It never rewrites that source.

| Point | Board | Stage 5 source PWM | V1 Golden ABOVE PWM | Status |
| --- | ---: | --- | --- | --- |
| P33 | (3,3) | 1628,1170,1190,1145,1500 | 1589,1136,1101,1084,1500 | **HARDWARE VERIFIED**, protected |
| P311 | (3,11) | 1483,1170,1180,1145,1500 | 1432,1199,1157,1042,1500 | **HARDWARE VERIFIED**, protected |
| P77 | (7,7) | 1560,1170,990,1170,1500 | 1500,1230,870,1230,1500 | **HARDWARE VERIFIED**, protected |
| P113 | (11,3) | 1680,1260,740,1390,1500 | 1630,1264,588,1424,1500 | **HARDWARE VERIFIED**, protected |
| P1111 | (11,11) | 1465,1260,720,1390,1500 | 1382,1258,639,1410,1500 | **HARDWARE VERIFIED**, protected |

Generation, Fast 5/9, save, reload and restart invariants all assert these exact values. An attempted direct edit is rejected; an intentional change can only be staged as `PENDING_REVALIDATION` for a later bounded hardware procedure.

## MoveL and DROP Pipeline

For each point, the planner takes final ABOVE PWM, runs FK, holds Cartesian `x`, `y` and tool angle constant, lowers `z` in 5 mm increments to a configured 25 mm target, and solves seeded IK at every waypoint. It rejects non-finite, out-of-limit, discontinuous and unreachable candidates without clipping them into apparent success. Retraction uses the exact saved reverse waypoint indices.

Offline batch result on this machine: `225 requested / 199 success / 26 MOVE_L_UNREACHABLE / 0 invalid / 0 skipped`; all five Golden starts were exact. P77 produced 0/5/10/15/20/25 mm waypoints and exact reverse indices `4,3,2,1,0`. These are **OFFLINE VERIFIED**, not live clearance evidence.

## Calibration Data Model

Each point stores `ABOVE final`, `DROP auto`, `DROP correction`, and `DROP final` separately. PWM correction is applied per joint and saved atomically with history. The operator can preview, move ABOVE, descend with pump off, retract, run a full-place test, reset/save correction, and mark verification. Live hardware marking is blocked until the same point completes both DROP and exact saved Retract, or one complete Full Place sequence; dry-run marking is **OFFLINE VERIFIED** only. Live game execution accepts only `HARDWARE VERIFIED` DROP records.

The status grid exposes generated, pending, verified, unreachable, invalid, manual and Golden states with next/previous/next-pending navigation.

## Fast 5/9 and Direct Anchors

- Fast 5 uses the five protected Golden locations.
- Fast 9 uses `(0,0)`, `(0,7)`, `(0,14)`, `(7,0)`, `(7,7)`, `(7,14)`, `(14,0)`, `(14,7)`, `(14,14)`.
- Corrections are propagated by a clamped piecewise-bilinear field.
- Golden points and user direct anchors are not overwritten.
- Any changed ABOVE invalidates its previous DROP so it must be regenerated and verified.

## Game Integration and Startup

Game code emits only an unambiguous `point_id`; the integration call is `robot.place_piece(point_id)`. IDs such as `P3_11` are explicit, while ambiguous `P112` is rejected unless written as flat `P#112`. Human-vs-human never invokes the robot. Live robot moves remain pending until vision observes the target stone; dry-run completes without hardware vision.

Startup is profile-driven, not a one-time boolean: missing, malformed, incompatible or invalid profiles route to First Setup; a valid profile routes directly to Game. Advanced retains the legacy calibration/development surfaces. Neither normal nor dry-run startup auto-connects camera or serial.

## Verification Results

- **PASS / OFFLINE VERIFIED:** `214 passed in 21.07s` final full pytest suite.
- **PASS / OFFLINE VERIFIED:** `47 passed in 6.56s` focused integration and regression suite.
- **PASS / OFFLINE VERIFIED:** project doctor, including exact Golden constants, stable hashes and batch-motion guard.
- **PASS / OFFLINE VERIFIED:** 8/8 safe smoke checks, including the 225-point pure offline batch.
- **PASS / OFFLINE VERIFIED:** offscreen PySide6 startup/screenshot; no camera or serial opened.
- **NOT VERIFIED:** all new V1 real MoveL/DROP motion, live correction, live fast recalibration and game placement.
- **HARDWARE VERIFICATION REQUIRED:** follow `docs/HARDWARE_TEST_CHECKLIST_V1.md`; never execute 225 candidates automatically.

## Run Commands

```powershell
D:\Anaconda\python.exe tools\project_doctor.py
D:\Anaconda\python.exe tools\smoke_test.py
D:\Anaconda\python.exe -m pytest -q
scripts\run_gomoku_robot_dry_run.bat
start_gomoku_robot.bat
```

The final command exposes hardware-capable controls after explicit COM connection. Use it only under the checklist and physical supervision.
