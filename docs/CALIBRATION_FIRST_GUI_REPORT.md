# Calibration-first GUI Delivery Report

Date: 2026-08-25  
Status: offline verified; new calibration controls are not hardware verified

## Audit evidence

- Before: `docs/audits/calibration_first_20260825/01_current_main_window.png`
- After, quick calibration: `docs/audits/calibration_first_20260825/03_new_quick_calibration.png`
- After, point verification: `docs/audits/calibration_first_20260825/04_new_point_verification.png`

## Delivery checklist

1. **Original GUI problem** — unrelated Stage 5/6/7, vision internals, learning, manual actions and logs competed in one long scrolling surface. The right side was visibly clipped at the audited viewport.
2. **Removed functions** — legacy fixed-P77 controls, Stage 5/6 panels, cross-anchor, learning, manual action lists and experimental buttons are removed from the user-facing GUI. `Advanced >>` now contains only status, camera diagnostics and the serial log.
3. **New structure** — the primary task tabs are `1 快速标定`, `2 点位验证`, and `3 运行`. Connection state, pump-off and emergency-stop remain globally visible; there is no user-facing DRY RUN switch.
4. **PWM input** — typing changes only the local spin-box value. A single joint is submitted only by Enter in that input or its `Apply` button.
5. **Live +/-** — Step defaults to 5; 1/5/10/20 are available. A click updates one spatial joint and emits one coalesced latest-target intent. It cannot address J5 (pump) or IDs 006/007.
6. **Apply versus Apply All** — `Apply` emits one `#00x...` spatial-joint command. `Apply All` emits one grouped J0–J4 pose and omits the pump/reserved outputs.
7. **PWM safety** — the existing Stage 5 teaching envelope is reused before the controller's 500..2500 protocol guard. Clamped values are shown as `OUT OF RANGE` in the workflow status.
8. **5/9 point calibration** — QUICK 5 and STANDARD 9 use the existing Stage 7 board-index mapping and required-coordinate definitions.
9. **Interpolation** — the stable Stage 5 bilinear helper remains the only interpolation primitive. Stage 7 applies per-joint baseline deltas and its existing local residual correction; direct anchors are never overwritten.
10. **Provenance** — persisted data remains `DIRECT`/`INTERPOLATED`/verified. The user-facing label maps interpolated points to `AUTO (INTERPOLATED)`.
11. **Fixed pick data** — `calibration/stage7_pick_poses.json` is an on-demand candidate file containing PICK_ABOVE/PICK_DOWN baseline/new/delta PWM, source, timestamp, session and `verified=false`. It does not alter `config/arm_actions.json`.
12. **One-click pick chain** — the button relays to `ControlPanel.pick_requested -> MainWindow.start_pick -> pick_piece -> ArmSequenceWorker -> SerialArmController.send_action`. The sequence is `SOURCE_TOUCH_IDLE -> SOURCE_TOUCH_HOLD -> wait -> OBSERVE_HOLD`, so it approaches the pick area, acquires the piece, then returns to hover at the observation point while holding vacuum. Candidate pick poses are not injected into this stable path.
13. **Live lower-controller path** — the slim operator GUI has no DRY RUN switch. After an explicit COM connection, enabled Stage 7 controls use the real controller path. Disconnected, busy, ESTOP and range guards still block transmission.
14. **Automated tests** — 35 focused calibration-first/Stage 7/pick/safety tests pass. The full suite is 177 passed, 2 failed; the remaining failures are pre-existing stale `P77_TOUCH_RELEASE` pump and lifted-carry assertions.
15. **Offline pass** — task navigation, PWM semantics, single/all command isolation, disconnected-state transmission blocking, safety range handling, session save, interpolation invariants, provenance, and pick candidate persistence.
16. **Hardware verified** — only the previously recorded fixed-P77/Stage 5 stable paths. No new PWM editor, Apply All, Stage 7 deployment, pick candidate or flash behavior is claimed as hardware verified.
17. **Next real-machine test** — with the tool empty and physical cutoff ready, connect COM, move to one already verified ABOVE point using the stable guarded route, and test exactly one J2 `+1` followed by safe return while observing that only J2 moves.
18. **ESTOP reconciliation** — every shared sequence and direct Stage 7 transmit path checks both the main arm ESTOP and the Stage 5 latched emergency state. Recovery is explicit, returns software state to `UNKNOWN`, and still requires a separate return-to-observe action.

## Keyboard controls

- Up/Down: selected joint +/- current Step
- Ctrl+Up/Down: selected joint +/-1
- Shift+Up/Down: selected joint +/-20
- Enter in a PWM input: Apply that joint only
- S: Save Point
- N: Next point
- Space: return to the calibration editor

## Motion-time presets

- Slow: 1500 ms — NOT HARDWARE TUNED
- Normal: 1000 ms — reused from the stable action-table default
- Fast: 700 ms — NOT HARDWARE TUNED
