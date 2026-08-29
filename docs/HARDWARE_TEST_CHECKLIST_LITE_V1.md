# Lite Calibration V1 Hardware Test Checklist

No Lite DROP is **HARDWARE VERIFIED** by software development or Dry Run. Complete this checklist manually for one point at a time. Begin with P77; do not run all 225 points automatically.

## 0. Stop conditions

Stop immediately and use the physical power cutoff/Emergency Stop for unexpected lateral motion below ABOVE, oscillation, joint-limit approach, cable/suction interference, board contact before DROP, or loss of operator visibility. After ESTOP, the pose is unknown; do not use DROP controls until the robot has been physically recovered to a known safe pose.

## 1. Pre-power checks

- [ ] Confirm point ID and row/column orientation.
- [ ] Confirm `tools/project_doctor.py` passes stable hashes and Lite guards.
- [ ] Confirm `calibration_lite_drop_v1.json` shows the selected point as `NOT VERIFIED` unless previously signed off.
- [ ] Inspect Preview: six Z waypoints (0–25 mm), constant x/y/alpha, valid status, exact reverse path.
- [ ] Clear the robot/board volume; secure base, board, hoses, and cables.
- [ ] Place physical cutoff and GUI Emergency Stop within reach.
- [ ] Start with no chess piece and reduced-risk power/speed where the hardware permits.
- [ ] Do not connect the camera; it is not required for this test.

## 2. Dry Run rehearsal — OFFLINE VERIFIED only

- [ ] Start `scripts\run_calibration_lite_dry_run.bat`.
- [ ] Connect the simulated controller; verify the title includes `DRY RUN`.
- [ ] Step 1: select P77 and load its saved ABOVE; inspect Preview under Advanced if needed.
- [ ] Step 2: click `1. Move ABOVE`, then confirm ABOVE correct.
- [ ] Step 3: exercise `Next Waypoint` and `Previous Waypoint`, then `2. Full DROP` and `3. Return to ABOVE`.
- [ ] Step 4: choose `Mark DROP Verified` only after visually accepting DROP; confirm this is a workflow gate and does not claim hardware evidence in Dry Run.
- [ ] Safe Return, then Step 5: run `4. Test PLACE` and confirm the displayed/logged order: pickup → carry high → ABOVE → Z waypoints → release → reverse waypoints.
- [ ] Confirm J5 stays hold before release and off through retract.
- [ ] If saved, confirm the point says **OFFLINE VERIFIED**, never **HARDWARE VERIFIED**.

## 3. Empty-tool live P77 ABOVE

- [ ] Close Dry Run and start `scripts\run_calibration_lite.bat`.
- [ ] Connect the intended COM port explicitly; verify no motion occurred on startup.
- [ ] Select P77 and compare saved ABOVE with Golden P77 `[1500,1230,870,1230,1500]`.
- [ ] Complete Step 1, click `1. Move ABOVE`, and accept the default-No live confirmation only when clear.
- [ ] Confirm carry-high then ABOVE clearance. Do not continue if P77 differs physically from the known Golden pose.
- [ ] Click the ABOVE-correct confirmation to unlock Step 3.

## 4. Empty-tool MoveL and retract

- [ ] With the arm parked at P77 ABOVE, test `Next Waypoint` one saved point at a time; use `Previous Waypoint` if necessary.
- [ ] After the stepped test is safe, click `2. Full DROP`.
- [ ] Observe every downward waypoint; x/y/tool attitude must remain visually stable.
- [ ] Stop before contact if clearance is wrong. Do not save a correction from an unsafe pose.
- [ ] If DROP is safe, click `3. Return to ABOVE` and verify the exact path returns vertically to ABOVE.
- [ ] Click Safe Return and verify OBSERVE/known-safe completion.

## 5. Optional J0–J4 correction

- [ ] Return to Step 3, move ABOVE and DROP again, then choose correction to enter Step 4.
- [ ] Adjust only one spatial joint at a time with small increments; J5 is locked.
- [ ] Apply every displayed J0–J4 correction target before Save Correction is enabled logically.
- [ ] Save and verify that auto, correction, and final remain separate in the profile.
- [ ] Use `Return to re-test DROP`; re-run DROP and Retract. Saving a correction must have reset persisted verification to **NOT VERIFIED**.
- [ ] Only after accepting the physical DROP, use `Mark DROP Verified` to unlock Step 5; this button alone must not write **HARDWARE VERIFIED**.

## 6. Complete Test PLACE

- [ ] Load exactly one piece in the known pickup source.
- [ ] Confirm Step 5 remains locked until ABOVE and DROP prerequisites have passed.
- [ ] Click `4. Test PLACE` only after the page enables it.
- [ ] Confirm suction pickup and safe lift before any board transit.
- [ ] Confirm saved ABOVE, waypoint descent, final DROP, pump release, and reverse retract in order.
- [ ] Confirm the piece was released and the tool ended at ABOVE without lateral motion below ABOVE.
- [ ] Safe Return to the known observe pose.

## 7. Verification sign-off

Only after the Step 5 Test PLACE sequence succeeds and the physical result above is observed may the operator click `Confirm HARDWARE VERIFIED` and answer Yes. The button must remain disabled beforehand.

- Point ID: __________
- Date/time: __________
- Operator: __________
- Candidate generation time: __________
- Correction J0/J1/J2/J3/J4: __________
- Empty-tool DROP/retract passed: Yes / No
- Test PLACE passed: Yes / No
- Emergency Stop tested separately: Yes / No
- Notes: ______________________________________________

Result label:

- [ ] **HARDWARE VERIFIED** — all required live steps passed and human confirmation was saved.
- [ ] **NOT VERIFIED** — any step failed, was skipped, or remains uncertain.

Proceed to another point only as a separate bounded test. Never use offline 225 generation as permission for 225-point hardware motion.
