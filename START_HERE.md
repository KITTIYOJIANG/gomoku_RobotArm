# START HERE

## Project Status

- Current stage: **J1 Gomoku Robot Lite Calibration V1 — software milestone**
- Status: **OFFLINE VERIFIED / HARDWARE VERIFICATION PENDING**
- Last stable branch: `main`
- Last stable tag: `J1_Gomoku_V0.2_Arbitrary_Hover_Stable`
- Last stable commit: `7a1e1253f37c9614a8e528f2d4b01e099e03474f`
- Current development branch/HEAD at audit: `stage6-full-board-descent` / `f8411afc6f198a05b1646bebcb8bf0e6a761708e`

Complex Integrated V1 expansion is paused. Calibration Lite now includes an isolated P77 Manual MoveL Tuner: Step0 is immutable Golden ABOVE, each new step inherits the previous saved final PWM, all editing is positive absolute J0–J4 PWM, J5 is omitted/locked during step tuning, and return uses saved manual steps in exact reverse order. J1-down/J2-up/J3-up is a suggestion only; no direction is enforced. `Set As P77 DROP + Apply To MoveL` persists the confirmed endpoint as the P77-only MoveL correction overlay, and the dedicated full flow executes pickup, confirmed manual descent, release, exact manual reverse, and J1-FIRST return. The old automatic WP01–WP05 remain reference-only for full-flow execution. Focused tests and an isolated Dry Run pass; no full suite was run. No manual DROP has been selected in the current data file and all manual steps remain **NOT VERIFIED** until operator hardware calibration.

## What Currently Works

- ✅ **Hardware exercised:** fixed-P77 PWM/action baseline and guarded P77 route; Stage 5 ABOVE teaching/hover has 30 persisted direct anchors and LIVE log evidence. Motion completion is time-estimated, not servo-ACK verified.
- 🧪 **Offline verified:** all 225 ABOVE intersections resolve from 30 direct anchors plus 195 interpolated points.
- 🧪 **Offline verified:** Stage 6 stores 153 complete five-layer descent candidates, rejects 72 points, and marks only P77 `DRY_RUN_PASSED`.
- 🧪 **Offline only:** the learning model is a shadow comparator and cannot drive live motion.
- 🧪 **OFFLINE VERIFIED:** Lite V1 loads 225 saved ABOVE records, overlays/protects five Golden anchors, and plans 193 DROP candidates while blocking 32 unreachable points.
- 🧪 **OFFLINE VERIFIED:** the shared five-step point wizard, adjacent waypoint/full DROP testing, exact partial/full retract, J0–J4 correction layers, Test PLACE/final-verification gates, ESTOP state clearing, Dry Run, and the offline-only 225 generator pass the full 225-test suite.
- ✅ **HARDWARE VERIFIED:** user-confirmed Golden ABOVE values P33/P311/P77/P113/P1111 and the inherited legacy pick/carry/fixed-P77 flow only.
- ❌ **NOT VERIFIED:** every new Lite MoveL/DROP and correction until the user completes a bounded real-arm point test.

## Current Goal

Complete and review J1 Gomoku Robot Lite Calibration V1 only. Game UI, vision closed loop, Fast 5/9, first-start flow, complex Integrated V1 UI, and automatic 225-point hardware execution remain paused.

## Next Task

1. Start Calibration Lite and open `P77 Manual MoveL Tuning`.
2. With explicit authorization and physical cutoff ready, calibrate P77 Step0 → StepN one saved/confirmed manual step at a time, select the confirmed landing step as P77 DROP, then run the dedicated P77 Pick & Place Full Flow.

## Critical Stable Assets

| Asset | Why it matters |
| --- | --- |
| `calibration/stage5_board_calibration.json` | Stable 15×15 ABOVE source: 30 direct anchors; audited SHA-256 `21B4994D...EE8052`. |
| `config/arm_actions.json` | Only persisted source for 11 stable action PWM commands; audited SHA-256 `9B0A888C...DA3FB7F`. |
| `calibration/stage6_descent_calibration.json` | Independent Stage 6 candidates and rejection evidence; must not overwrite Stage 5. |
| `config/stage6_descent.json` | Keeps Stage 6 `force_dry_run=true`. |
| `app/arm/sequences.py` | Enforces carry-high → ABOVE → TOUCH and guarded return ordering. |

## DO NOT TOUCH WITHOUT REVALIDATION

- Do not edit or regenerate the Stage 5 baseline in place.
- Do not change `config/arm_actions.json`, the P77 PWM values, pump semantics, or the fixed P77 sequence to satisfy stale tests.
- Do not remove carry-high, ABOVE, ordered descent, exact reverse ascent, state-machine, thermal, or emergency-stop guards.
- Do not set Stage 6 live or treat `COMPUTED`/`DRY_RUN_PASSED` as hardware verification.
- Do not compile or flash the copied `SAFE_STAGE1` firmware as a replacement for the deployed PWM controller.

## How to Start

Use a supported environment. On the audited machine, bare `python` resolves to an unsuitable MSYS2 Python 3.14; the known environment is `D:\Anaconda\python.exe` 3.12.7.

```powershell
python tools\project_doctor.py
python tools\resume_project.py
python tools\smoke_test.py
```

The operator GUI now uses the live lower-controller path:

```powershell
python -m app.main
```

Recommended entry points are `start_gomoku_robot.bat` and the hardware-isolated `scripts\run_gomoku_robot_dry_run.bat`.

For the current Lite milestone, use `scripts\run_calibration_lite_dry_run.bat`. The normal hardware-capable Lite entry is `scripts\run_calibration_lite.bat`; it still requires explicit COM connection and per-sequence confirmation.

The command does not auto-connect. After the user selects a COM port and clicks connect, enabled motion controls send real PWM through the existing limits and state machine.

## Safe Start

- Camera and serial never auto-connect.
- `--dry-run` uses the simulated serial boundary and never opens a real COM port.
- Normal startup is hardware-capable only after an explicit COM connection and action confirmation.
- Hidden legacy Stage 5/6 developer panels retain their internal test gates but are not user-facing.
- `--test-pattern` avoids opening a real camera.
- `Generate 225 OFFLINE ONLY` is planner-only and cannot execute hardware.
- Only explicit GUI actions after serial connection can move the arm. Physical clearance and power cutoff remain mandatory.

## Hardware Quick Setup

- Robot: J1 arm; spatial servo IDs `000..004`, suction `005`, reserved/held-neutral `006..007`.
- Controller: deployed STM32-compatible controller using the existing ASCII PWM protocol; exact firmware revision is UNKNOWN.
- Serial: selectable Windows COM port, default hint `COM6`, fixed `115200` baud, 0.1 s read timeout, 1.0 s write timeout.
- Camera: preferred name `USB 2.0 Camera`, index fallback `0`, configured `1280×720 @ 30 FPS`.
- Board: 15×15; Tag 15=TL, 16=TR, 17=BR, 18=BL; row increases downward and column increases rightward.

See [Hardware Setup](docs/HARDWARE_SETUP.md). No committed deployment photo or measured camera/board geometry is available.

## Important Files

| File | Purpose |
| --- | --- |
| `app/main.py` | GUI process entry point. |
| `app/main_window.py` | Runtime coordinator for camera, Stage 5/6, worker and controller. |
| `app/arm/controller.py` | The only host serial owner and hardware write boundary. |
| `app/stage5/` | Stable ABOVE selection, calibration and interpolation. |
| `app/stage6/` | Offline-first five-layer descent planner and safety state machine. |
| `app/stage7/` | Uncommitted rapid-calibration implementation wired into the current GUI; offline-tested, not stable or hardware verified. |
| `app/integrated_v1/` | Protected Golden profile, point IDs, MoveL planner and shared RobotController. |
| `app/calibration_lite/drop_v1.py` | Independent Lite DROP store and guarded shared-worker sequence builder. |
| `app/calibration_lite/drop_v1_view.py` | Five-step single-point ABOVE/DROP/correction/Test PLACE wizard with collapsed advanced details. |
| `app/calibration_lite/manual_movel.py` | P77-only manual step store and J0–J4 sequence builder. |
| `app/calibration_lite/manual_movel_view.py` | Absolute-PWM P77 Manual MoveL editor. |
| `calibration/p77_manual_movel.json` | Independent P77 manual steps and manual DROP candidate; Step0 is immutable Golden ABOVE. |
| `calibration/calibration_lite_drop_v1.json` | 225-point Lite candidate/failure records; currently zero verified DROP points. |
| `docs/CALIBRATION_LITE_V1_REPORT.md` | Current Lite algorithm, data format, files, and offline evidence. |
| `docs/HARDWARE_TEST_CHECKLIST_LITE_V1.md` | Required bounded real-arm verification and sign-off. |
| `app/game/` | Board rules, tactical AI and point-id-only game session. |
| `app/gui/drop_calibration_panel.py` | 15×15 DROP status, correction, Fast 5/9 and guarded test controls. |
| `docs/INTEGRATED_V1_REPORT.md` | Integrated V1 architecture, provenance and exact offline evidence. |
| `docs/HARDWARE_TEST_CHECKLIST_V1.md` | Required bounded live-test sequence and sign-off fields. |
| `docs/ARCHITECTURE.md` | Actual module/data flow and coordinate systems. |
| `docs/CALIBRATION.md` | Baseline, descent and Stage 7 provenance rules. |
| `docs/CALIBRATION_FIRST_GUI_REPORT.md` | Calibration-first GUI behavior, offline evidence and hardware-truth boundary. |
| `docs/TROUBLESHOOTING.md` | Known failures and recovery checks. |

## Instructions for AI Coding Agents

Repository-specific rules live in [AGENTS.md](AGENTS.md). Before changing code: read this file and `project_state.json`, read the architecture, run the doctor, inspect the full Git status, identify stable calibration data, and never modify a stable baseline unless explicitly requested.

## If You Only Have 10 Minutes

1. Run `git status --short --untracked-files=all`.
2. Run `python tools\project_doctor.py`.
3. Run `python tools\resume_project.py`.
4. Run `python tools\smoke_test.py`.
5. Read **Next Task**, then check [Development Workflow](docs/DEVELOPMENT_WORKFLOW.md).
