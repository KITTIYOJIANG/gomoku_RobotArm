# Architecture

## System Overview

The application is a Windows/PySide6 upper computer. `app.main` loads JSON configuration and constructs `MainWindow`; the window owns the camera worker, board/arm state, Stage 5/6 coordinators, the single arm worker, and the single serial controller.

The repository does **not** contain a Gomoku AI or an automatic move-selection engine. A user selects a board target in the GUI. Vision supplies board geometry and piece observations; it does not currently choose the robot move.

```text
Camera / test pattern
  -> CameraWorker (QThread)
  -> AprilTag BoardLocator
  -> BoardTracker (LOCKED / FROZEN / LOST)
  -> homography + 15x15 intersections + piece recognition
  -> GUI target selection
  -> Stage 5 ABOVE resolver / Stage 6 descent planner
  -> guarded SequenceDefinition
  -> ArmSequenceWorker (QThread)
  -> SerialArmController
  -> ASCII serial at 115200
  -> deployed STM32-compatible controller
  -> J1 servos
```

## Data Flow

### Vision and target selection

1. `CameraWorker` opens the configured camera only after an explicit GUI request, or produces a generated frame in test-pattern mode.
2. `BoardLocator` calls the copied AprilTag localization pipeline. Tag roles are 15=TL, 16=TR, 17=BR, 18=BL.
3. `BoardTracker` smooths corners in fixed TL/TR/BR/BL order and exposes `LOCKED`, temporary `FROZEN`, or `LOST` state.
4. The same homography feeds the displayed boundary, grid, P77 marker and board-intersection array. Drawing occurs on a display copy so overlays do not contaminate detector input.
5. `PieceRecognizer` consumes an undrawn detection frame. Its observations are diagnostic; there is no automatic game engine downstream.
6. A user click is mapped to the nearest `[row, col]` intersection and accepted only within the configured spacing-relative threshold.

### Stage 5 ABOVE motion

```text
selected row/col
  -> CalibrationStore
  -> direct anchor, tightest-cell bilinear interpolation, or guarded seed fallback
  -> spatial PWM 000..004
  -> runtime TARGET_ABOVE action (pump state added)
  -> CARRY_HIGH_LIFTED_* -> TARGET_ABOVE_*
  -> exact safe return via carry-high -> OBSERVE
```

`calibration/stage5_board_calibration.json` persists 30 direct anchors. It does not persist 225 independently measured poses. The Stage 5 resolver supplies the other 195 positions at runtime.

### Fixed P77 pick/place

The stable fixed-point flow in `app/arm/sequences.py` is:

```text
OBSERVE_IDLE
  -> source approach / suction / OBSERVE_HOLD
  -> CARRY_HIGH_P77_HOLD
  -> P77_ABOVE_HOLD
  -> P77_TOUCH_HOLD
  -> named release pose + release dwell
  -> P77_ABOVE_IDLE
  -> CARRY_HIGH_P77_IDLE
  -> OBSERVE_IDLE
```

The stable `P77_TOUCH_RELEASE` action still contains pump `005=2500`; Stage 6 therefore uses a separate explicit pump-off command instead of silently editing the stable action table.

### Stage 6 descent

Stage 6 reads the Stage 5 ABOVE baseline without modifying it, applies the configured candidate kinematic model, and stores independent five-layer profiles:

```text
ABOVE -> DESCENT_25 -> DESCENT_50 -> DESCENT_75 -> TOUCH
      -> pump off / dwell
      -> ASCENDING_75 -> ASCENDING_50 -> ASCENDING_25 -> ABOVE
      -> CARRY_HIGH -> OBSERVE
```

The state machine locks the selected row/column below ABOVE and requires carry-high before moving to a different target. The ascent reuses the saved reverse PWM levels rather than recomputing a shortcut. `config/stage6_descent.json` currently forces dry-run.

### Stage 7 draft

The uncommitted `app/stage7/` working-tree implementation snapshots all 225 Stage 5 ABOVE values, records new direct anchor PWM as deltas, interpolates the delta field, and creates a separate deployment candidate. `ControlPanel` embeds the rapid-calibration panel and `MainWindow` wires session, move-above, jog, save, recalculate, verify, commit, rollback and safe-return actions. Its 11 dedicated backend/GUI tests pass, but it remains an uncommitted, non-hardware-verified milestone; the repository-wide suite still has five older assertion failures.

## Main Modules

| Area | Real implementation |
| --- | --- |
| Entry/config | `app/main.py`, `app/config.py`, `config/*.json` |
| Runtime orchestration | `app/main_window.py` |
| Camera/vision | `app/vision/camera_worker.py`, `board_locator.py`, `board_tracker.py`, `piece_recognizer.py` |
| Board intersection UI | `app/stage5/board_intersections.py`, `app/gui/camera_panel.py` |
| ABOVE calibration | `app/stage5/calibration_store.py`, `pwm_interpolator.py` |
| Hover planning/safety | `app/stage5/hover_planner.py`, `state_machine.py`, `safety.py` |
| Descent candidates | `app/stage6/planner.py`, `calibration_store.py`, `state_machine.py`, `kinematics.py` |
| Stable actions/sequences | `app/arm/actions.py`, `sequences.py`, `state.py` |
| Hardware execution | `app/arm/worker.py`, `app/arm/controller.py` |
| Shadow learning | `app/learning/`; `MODEL_LIVE_CONTROL_ENABLED` is false |
| Stage 7 draft | `app/stage7/`, `app/gui/rapid_calibration_panel.py` in the current dirty tree |

## Coordinate Systems

### Image pixels

OpenCV frames use `(x, y)` pixels with origin at the image top-left. The GUI maps scaled widget clicks back into this original image space before target selection.

### Board coordinates

Board coordinates are zero-based `[row, col]`: row 0 is the top edge and grows downward; column 0 is the left edge and grows rightward. The Tag-defined corner order is TL, TR, BR, BL.

### Point names

Two conventions coexist and must be written with coordinates:

- Legacy `P77` means board coordinate `(7,7)`, the center.
- The Stage 7 draft uses flat zero-padded IDs: `P000=(0,0)`, `P014=(0,14)`, `P112=(7,7)`, `P210=(14,0)`, `P224=(14,14)`.

Never infer a coordinate from an unqualified `Pxxx` string; include `(row,col)` in logs and documents.

### Robot PWM and candidate tool pose

- Servo `000..004`: spatial pose PWM used by the host.
- Servo `005`: suction, conventionally 2500=hold and 1500=off.
- Servo `006..007`: retained at 1500 in stable actions; no host motion role is established.
- Stage 6 also computes candidate `(x,y,z,alpha)` values from copied factory geometry. These are model coordinates, not a measured camera/world/robot hand-eye frame.

## Safety Boundaries

- Only `SerialArmController.write()` can transmit host commands. Construction does not connect or move.
- `ArmSequenceWorker` is the normal action executor; waits are interruptible and sequence concurrency is rejected.
- GUI connection is explicit. Hardware-capable mode plus a connected COM port plus a confirmed action can move the robot.
- Stage 5 defaults to dry-run but is not globally forced. Stage 6 is force-locked to dry-run. The Stage 7 draft is not approved for hardware use.
- `$DST!` is the software emergency-stop command; physical power cutoff remains the final safety control.
- The copied `SAFE_STAGE1` firmware is a read-only reference whose protocol conflicts with the deployed raw PWM assumptions.

## Integrated V1 Layer

`app/integrated_v1/` adds a versioned profile, protected Golden anchors, Cartesian MoveL planning and one high-level `RobotController`. `MainWindow` injects its existing `SerialArmController`, `ActionLibrary` and `ArmSequenceWorker`; V1 does not own a second hardware backend. The game and 15×15 calibration panels call this shared API with explicit point IDs.

The V1 profile is independent of the stable Stage 5 file. Startup derives `FIRST_SETUP` or `GAME` from profile integrity and validity. Batch generation is planner-only and has no hardware execution path. Live game placement additionally requires a DROP carrying **HARDWARE VERIFIED** evidence.

## Lite Calibration V1 Layer (current scope)

Current development is intentionally limited to `app/calibration_lite/`. `CalibrationLiteWindow` inherits the existing runtime and therefore shares one controller, action library, worker, and state machine. `drop_v1_view.py` owns UI state only; `drop_v1.py` owns the independent profile and sequence construction; the pure `MoveLPlanner` owns Cartesian-Z/FK/seeded-IK computation and has no controller dependency.

The Lite sequence boundary is single-point and operator-driven. Its five-step wizard keeps transient interaction state (current step, completed steps, current waypoint, ABOVE confirmation, DROP accuracy confirmation, and Test PLACE result) separate from persisted calibration/evidence. Selecting any point, including P77, uses the same workflow. Advanced PWM and candidate/Cartesian/batch details are collapsed by default; the emergency and safe-return controls remain outside the step pages.

ABOVE/waypoint/DROP/return pose state prevents point switching below a known safe pose. A waypoint test moves only to an adjacent saved Cartesian waypoint; partial return uses the exact saved reverse points, and leaving a corrected final DROP first passes through the uncorrected final auto waypoint. A global Safe Return invalidates the local ABOVE confirmation and resumes at Step 2 unless the verified DROP flow has already advanced to Step 5. Live sequences use default-No confirmations. Final evidence confirmation remains locked until the full Test PLACE sequence succeeds; offline 225 generation saves candidates/failures but has no path to the worker. See `docs/CALIBRATION_LITE_V1_REPORT.md`.
