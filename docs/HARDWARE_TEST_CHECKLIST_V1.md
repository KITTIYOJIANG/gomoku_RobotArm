# Integrated V1 Hardware Test Checklist

Status at delivery: **NOT VERIFIED** for all new V1 motion. This checklist requires an operator at the robot and explicit authorization. Never select or script all 225 points for live execution.

## 0. Evidence and Abort Rules

- [ ] Name the operator, date/time, robot/firmware identity, COM port, profile version and exact point under test.
- [ ] Put the physical power cutoff within reach; assign one person to it.
- [ ] Clear the whole swept volume and start with an empty tool/no stone.
- [ ] Confirm no other process owns the serial port.
- [ ] Stop immediately on unexpected direction, vibration, cable tension, board contact, suction error, loss of vision, UI freeze or uncertain state.
- [ ] After any abort, cut motion power if needed, preserve logs/profile, and do not resume from a mid-path assumption.

## 1. Offline Preflight — No Hardware

- [ ] Run `D:\Anaconda\python.exe tools\project_doctor.py`; require no critical failures.
- [ ] Run `D:\Anaconda\python.exe tools\smoke_test.py`; require 8/8 PASS.
- [ ] Confirm the stable hashes shown by doctor match the delivery report.
- [ ] Start `scripts\run_gomoku_robot_dry_run.bat` and inspect the selected point's ABOVE, MoveL waypoints, final DROP and reverse path.
- [ ] Confirm `Generate All DROP` is labelled `OFFLINE ONLY` and cannot send motion.

## 2. Connection and ESTOP — No Arm Travel

- [ ] Start `start_gomoku_robot.bat`; confirm camera and serial remain disconnected.
- [ ] Confirm First Setup or Game routing matches profile validity.
- [ ] Connect only the intended COM port at 115200.
- [ ] Test software ESTOP before any motion; verify commands are blocked afterward.
- [ ] Restore to a known safe observation pose only under the existing recovery procedure.

## 3. P77 Empty-Tool ABOVE Test

- [ ] Select P77 and verify displayed Golden ABOVE is exactly `1500,1230,870,1230,1500`.
- [ ] Keep pump off and command only `Move ABOVE`.
- [ ] Verify direction and clearance throughout; stop before descent if uncertain.
- [ ] Record observation and result. Do not modify the Golden ABOVE during this test.

## 4. P77 Incremental MoveL and Exact Retract

- [ ] Preview the 0/5/10/15/20/25 mm path.
- [ ] With empty tool and pump off, test one new waypoint at a time, beginning at 5 mm.
- [ ] After every descent increment, command the saved reverse path and verify return to exact P77 ABOVE.
- [ ] Advance only after the previous depth is observed safe.
- [ ] If the intended board gap is not reached safely, stop and save a correction candidate; do not bypass the planner guard.

## 5. P77 Release and Full Place

- [ ] Test suction hold/release at the source independently using the inherited verified sequence.
- [ ] Use one stone and run P77 only: pick → safe lift → Golden ABOVE → verified MoveL DROP → release → exact reverse → safe lift → observe.
- [ ] Verify the piece is released, the arm does not drag it, and vision confirms P77 occupancy.
- [ ] Only now mark P77 DROP **HARDWARE VERIFIED**, with notes and retained log evidence.

## 6. Golden and Representative Expansion

- [ ] Repeat the empty-tool ABOVE and incremental DROP procedure separately for P33, P311, P113 and P1111.
- [ ] Never edit a Golden ABOVE to compensate for DROP; use DROP correction fields.
- [ ] Test representative non-Golden points one at a time: corners/edges, inner quadrants, then interpolated regions.
- [ ] For every point, require successful retract and evidence before marking **HARDWARE VERIFIED**.
- [ ] Treat each of the 26 offline-unreachable points as blocked until separately re-taught or re-planned; never force or clip them.

## 7. Fast 5/9 Recalibration

- [ ] Capture and review Fast 5 anchors first; verify the five protected Golden values remain byte-for-byte unchanged.
- [ ] Apply Fast 5 offline, review affected ABOVE deltas and regenerated DROP status before any motion.
- [ ] If required, repeat with Fast 9 one anchor at a time.
- [ ] Confirm direct user anchors retain priority and every changed point's old DROP is invalidated.
- [ ] Re-run bounded representative hardware tests; a Fast 5/9 calculation is **NOT VERIFIED** by itself.

## 8. Game Validation

- [ ] Start with human-vs-human and confirm no robot command is emitted.
- [ ] In robot mode, test one known **HARDWARE VERIFIED** DROP point.
- [ ] Confirm game passes the displayed `point_id` to the shared `RobotController` and refuses unverified/unreachable targets.
- [ ] Confirm the move remains pending until vision observes the placed stone.
- [ ] Trigger ESTOP/recovery once in a safe no-motion condition and confirm game state does not silently advance.

## Sign-off

For each point record: `point_id`, ABOVE PWM, auto/correction/final DROP PWM, waypoint list, load, operator observation, pass/fail, verification level and evidence path. Promote/export a new Golden baseline only after explicit review. Do not claim all-board hardware verification from sampling.
