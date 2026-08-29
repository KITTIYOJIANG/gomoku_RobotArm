# J1 Gomoku Robot Lite Calibration V1

## Scope and evidence

This milestone intentionally pauses the game UI, vision closed loop, Fast 5/9, first-start wizard, complex Integrated V1 pages, and all automatic 225-point hardware execution. The operator works on one board point at a time in the existing Calibration Lite application.

- **HARDWARE VERIFIED:** inherited pick/carry/fixed-P77 flow and the five user-confirmed Golden ABOVE anchors only.
- **OFFLINE VERIFIED:** the Lite data model, Cartesian-Z planner, safety rejection, sequence construction, UI gating, Dry Run, and offline 225-point generation.
- **NOT VERIFIED:** every newly generated or corrected Lite DROP candidate until the user completes the bounded real-arm checklist for that point.

The frozen assets `calibration/stage5_board_calibration.json` and `config/arm_actions.json` remain byte-identical to their audited hashes.

## Runtime architecture

`CalibrationLiteWindow` still inherits the existing `MainWindow`, so it uses exactly one `SerialArmController`, one `ArmSequenceWorker`, one `ActionLibrary`, and one arm state machine. `LiteDropV1Panel` emits intents only and owns no serial/controller object. `LiteDropSequenceBuilder` registers runtime actions in the shared action library; all sequences are submitted through the existing worker.

The Lite DROP profile is independent at `calibration/calibration_lite_drop_v1.json`. Loading/synchronizing it never writes either stable asset. Five Golden ABOVE records are overlaid exactly and marked protected:

| Anchor | J0 | J1 | J2 | J3 | J4 |
| --- | ---: | ---: | ---: | ---: | ---: |
| P33 | 1589 | 1136 | 1101 | 1084 | 1500 |
| P311 | 1432 | 1199 | 1157 | 1042 | 1500 |
| P77 | 1500 | 1230 | 870 | 1230 | 1500 |
| P113 | 1630 | 1264 | 588 | 1424 | 1500 |
| P1111 | 1382 | 1258 | 639 | 1410 | 1500 |

J5 is pump joint `005`. The kinematic config contains only spatial joints `000..004`; Lite pose, waypoint, auto, correction, and final fields reject/omit J5. Runtime actions add J5 explicitly as either hold (`2500`) or off/release (`1500`).

## MoveL algorithm

For a selected saved ABOVE pose:

1. Normalize and limit-check J0–J4, then convert PWM to joint angles and run forward kinematics.
2. Keep Cartesian `x`, `y`, tool `alpha`, and J4 passthrough constant.
3. Build Z targets at 5 mm increments down to the configured 25 mm target: `0, 5, 10, 15, 20, 25 mm` descent.
4. Solve each target with inverse kinematics seeded by the previous waypoint. This gives branch continuity rather than six independent IK solutions.
5. Reject non-finite poses, IK failure, joint/PWM limit violations, J4 changes, or a per-waypoint PWM jump greater than 400.
6. Save the deepest safe waypoint and failure reason when a point is unreachable; blocked records cannot build a motion sequence.
7. Save the exact reverse indices and use them for Retract. No IK recomputation or direct DROP-to-ABOVE shortcut is used.

The automatically solved endpoint is `drop_auto_pwm`. Operator correction is a separate J0–J4 delta, and `drop_final_pwm = auto + correction`. Saving a correction invalidates prior DROP verification.

## Data format

The top-level profile contains `source`, `safety`, `above.points`, `drop.points`, `verification`, and `history`. A representative point record is:

```json
{
  "point_id": "P77",
  "board": [7, 7],
  "above_pwm": {"000": 1500, "001": 1230, "002": 870, "003": 1230, "004": 1500},
  "above_cartesian_pose": {"x": 0.0, "y": 177.5438, "z": 98.5418, "alpha": -55.0, "auxiliary_004_pwm": 1500},
  "waypoints": [
    {"index": 0, "descent_mm": 0.0, "cartesian_pose": {"x": 0.0, "y": 177.5438, "z": 98.5418, "alpha": -55.0}, "pwm": {"000": 1500, "001": 1230, "002": 870, "003": 1230, "004": 1500}, "source": "final_above"}
  ],
  "drop_auto_pwm": {"000": 1500, "001": 1214, "002": 747, "003": 1369, "004": 1500},
  "drop_correction_pwm": {"000": 0, "001": 0, "002": 0, "003": 0, "004": 0},
  "drop_final_pwm": {"000": 1500, "001": 1214, "002": 747, "003": 1369, "004": 1500},
  "reverse_ascent_indices": [4, 3, 2, 1, 0],
  "status": "PENDING_VERIFY",
  "verification_level": "NOT VERIFIED",
  "verified": false,
  "reason": null
}
```

This is an abbreviated copy of the saved P77 record; the profile contains all six Cartesian waypoint records. No pose field contains `005`.

## Operator actions

- The single-point page is a five-step wizard shared by P77 and every other point: select/load ABOVE → move and confirm ABOVE → waypoint/full DROP test → optional correction and accuracy gate → Test PLACE and final evidence confirmation.
- Only the current step's recommended controls are enabled. Emergency Stop, Return to ABOVE, and Safe Return remain visible outside the step pages.
- Step 3 supports `Next Waypoint`, `Previous Waypoint`, `Full DROP`, and `Return to ABOVE`. Intermediate return follows the exact saved reverse subset. If the final pose includes a correction, `Previous Waypoint` first returns through the saved auto-final waypoint.
- Advanced PWM correction is collapsed until the operator chooses correction/advanced settings. Candidate, waypoint, Cartesian XYZ, failure, and 225-point statistics are also collapsed by default.
- `Generate DROP` and `Preview MoveL` are offline operations.
- `Generate 225 OFFLINE ONLY` calls a planner that has no controller reference. It creates records and per-point failures but cannot submit motion.
- `Move ABOVE`, waypoint/full `Move DROP`, `Retract`, `Test PLACE`, and live J0–J4 correction use the shared controller and require an explicit connection. In live mode each sequence has a default-No confirmation dialog.
- Point switching is blocked while parked at ABOVE/waypoint/DROP. Leaving waypoint/DROP requires vertical Retract; leaving ABOVE requires Safe Return. A Safe Return during Steps 2–4 resumes at Step 2 so ABOVE must be moved to and confirmed again.
- `Test PLACE` is pickup → safe carry/lift → saved ABOVE → Cartesian-Z waypoints with pump held → corrected DROP → pump release → exact reverse vertical retract. It finishes at ABOVE so the operator can inspect before Safe Return.
- `Mark DROP Verified` in Step 4 is only the wizard's observed-accuracy gate; it does not write a persisted verification label. Step 5 remains locked until this gate and vertical retract complete.
- `Confirm HARDWARE VERIFIED` remains disabled until Test PLACE succeeds. In live mode a second human confirmation is then required before the profile can write **HARDWARE VERIFIED**.
- In Dry Run, verification writes **OFFLINE VERIFIED** only.

## Offline results (2026-08-28)

- Saved Lite ABOVE source: 225 requested, 193 executable candidates, 32 guarded unreachable, 0 invalid, 0 skipped, 5 Golden starts. This run applies the configured per-joint PWM limits as well as the continuity gate.
- Independent stable-baseline smoke source: 225 requested, 199 executable candidates, 26 guarded unreachable, 0 invalid, 0 skipped, 5 Golden starts.
- Lite-focused tests: `11 passed in 5.85s` for the current consolidated Lite DROP module, including wizard and verification gates.
- Full repository suite: `225 passed in 26.61s`.
- Safe smoke: `10 checks` passed.
- Project doctor: Lite batch gate, J5 lock, Golden integrity, profile parse, and stable hashes passed.
- Camera opened: no. Real serial opened: no. Hardware motion: no.

The result above is **OFFLINE VERIFIED**. The profile currently contains zero verified DROP points; all new candidates remain **NOT VERIFIED**.

## P77 Manual MoveL Tuner micro-milestone

The P77-only tuner is independent at `calibration/p77_manual_movel.json`. Step0 is the immutable Golden ABOVE `[1500,1230,870,1230,1500]`. Each new step initially copies the previous saved final PWM exactly; the UI then edits positive absolute J0–J4 targets with direct input and ±10/±20/±50 controls. J1 decrease and J2/J3 increase remain a conservative suggestion only; the store and sender do not enforce any delta direction or ratio.

Runtime step-tuning actions contain only servo IDs `000..004`, so J5 is locked by omission. `Confirm Step` records `operator_confirmed=true` while retaining `hardware_verified=false`. `Set As P77 DROP + Apply To MoveL` writes the separate `manual_movel_tuning` candidate and a P77-only correction overlay in the Lite DROP profile; save/load and subsequent P77 MoveL recalculation preserve the operator's exact final PWM without changing Golden ABOVE.

The dedicated P77 full flow starts at Observation (or safely returns from Step0 first), executes the calibrated pick actions, J1-held transit through runtime SAFE ABOVE, every confirmed manual descent step, pump release at the selected DROP, every saved step in exact reverse order, and the two-phase J1-FIRST Observation return. Its runtime path does not execute the old automatic WP01–WP05.

Targeted evidence: `12 passed in 4.83s`; isolated P77 full-flow Dry Run: `1 passed in 3.85s`. Camera opened: no. Real serial opened: no. Hardware motion: no. No full suite was run for this micro-development. The implementation is **OFFLINE VERIFIED** and the manual/full-flow path remains **NOT VERIFIED** on hardware.

## Commands

```powershell
scripts\run_calibration_lite_dry_run.bat
D:\Anaconda\python.exe tools\generate_lite_drop_candidates.py
D:\Anaconda\python.exe tools\project_doctor.py
D:\Anaconda\python.exe tools\smoke_test.py
D:\Anaconda\python.exe -m pytest -q
```

The generator is offline-only and imports no controller. For real-arm work, use the normal Lite launcher only after following `docs/HARDWARE_TEST_CHECKLIST_LITE_V1.md`.
