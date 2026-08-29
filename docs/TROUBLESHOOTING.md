# Troubleshooting

## `python` has no pytest or uses Python 3.14

**Symptoms:** `No module named pytest`, missing PySide6/OpenCV/serial, or a shell version different from the README-supported 3.10–3.12 range.

**Cause seen in this repository:** On the audited Windows machine, PATH resolves `python` to `C:\msys64\ucrt64\bin\python.exe` 3.14.5. The working environment is `D:\Anaconda\python.exe` 3.12.7.

**Check:**

1. Run `where python` and `python --version`.
2. Run `python tools\project_doctor.py`.
3. Use an intended 64-bit 3.10–3.12 environment and install `requirements.txt` there.

**Do not:** Install robotics dependencies into an unrelated system/MSYS2 Python just to make one command disappear.

## The full pytest suite is not green

**Symptoms:** Five assertion failures in the full suite even though the safe smoke and dedicated Stage 7 suite pass.

**Final result (2026-08-25):** `162 passed, 5 failed` under Anaconda Python 3.12.7. The dedicated Stage 7 backend/GUI tests report `11 passed`.

- The five failures expect older pump/pick/carry behavior; the Stage 6 delivery report already identifies these baseline inconsistencies.

**Check:** Run `python tools\smoke_test.py` first, then the dedicated tests for the area being changed, then the whole suite.

**Do not:** Change stable P77 PWM, pump semantics, or safe carry paths merely to satisfy stale assertions. Reconcile the test expectations with the accepted behavior instead.

## Robot does not move

**Symptoms:** GUI action logs as dry-run, is disabled, or returns a state/connection error.

**Check:**

1. Confirm this is an explicitly authorized hardware session.
2. Stage 5: inspect its dry-run checkbox and any `force_dry_run` setting.
3. Stage 6: current configuration intentionally has `force_dry_run=true`; it must not move.
4. Confirm the correct COM port is explicitly connected; startup never auto-connects.
5. Confirm arm state and board lock satisfy the selected action preconditions.
6. Read the session log for `DRY_RUN_TX`, state rejection, or serial error.

**Do not:** Disable a safety gate before understanding which stage and verification level it protects.

## Serial connection fails or opens the wrong controller

**Symptoms:** Cannot open COM6, port not present, access denied, or commands are ignored.

**Check:**

1. Use the GUI port dropdown or Windows Device Manager; `COM6` is only a default hint.
2. Confirm `pyserial` is installed in the Python actually launching the GUI.
3. Confirm no other process owns the port.
4. Confirm 115200 baud and that the attached controller accepts `{#...}` and `$DST!`.
5. Confirm it is not a newly flashed `SAFE_STAGE1` reference build, which rejects raw multi-servo PWM.

**Do not:** Probe compatibility by sending a motion command. Identify firmware/controller safely first.

## Camera cannot open or the wrong device is selected

**Symptoms:** Camera stays disconnected, index 0 opens another camera, or preferred-name resolution fails.

**Check:**

1. Confirm no other application owns the camera.
2. Set `GOMOKU_CAMERA_ID` to the desired numeric index if the preferred name cannot resolve.
3. The preferred-name path may fall back when ffmpeg device enumeration is unavailable.
4. Test the GUI first with `--dry-run --test-pattern`, then camera-only with `--dry-run`.

**Do not:** Connect serial while diagnosing camera selection.

## Board never reaches LOCKED or the overlay is mirrored

**Symptoms:** `FROZEN`/`LOST`, fewer than three tags, grid moves, or P77 appears on the wrong physical side.

**Check:**

1. Verify Tag 15=TL, 16=TR, 17=BR, 18=BL in the camera view.
2. Keep at least three tags and enough board area visible.
3. Check glare, occlusion, focus and board movement.
4. Confirm row grows downward and column grows rightward: P000 top-left, P014 top-right, P210 bottom-left, P224 bottom-right.
5. Remember the project intentionally has no real camera intrinsics file; localization is a 2D homography.

**Do not:** Edit the robot PWM baseline to fix a visual orientation problem.

## Calibration file is missing, unreadable, or hash-mismatched

**Symptoms:** Doctor reports baseline failure, Stage 6 source mismatch, invalid PWM, or the Stage 7 snapshot refuses to continue.

**Check:**

1. Stop before any motion.
2. Verify `calibration/stage5_board_calibration.json` exists and compare its SHA-256 with `project_state.json`.
3. Check Git diff/status and any deliberate calibration session output.
4. Restore only from an reviewed version/tag or an explicit backup; do not copy a test fixture over the baseline.

**Do not:** Regenerate or overwrite the baseline as an automatic repair.

## A point name selects the wrong location

**Symptoms:** `P77`, `P077`, `P112`, or a flat selector appears inconsistent.

**Cause:** Two point-name conventions coexist. Legacy `P77` means `(7,7)`; Stage 7 flat indexing makes `(7,7)` equal `P112`.

**Check:** Always log and confirm `(row,col)` and, when used, the flat index.

**Do not:** Parse a legacy coordinate label as a flat numeric ID.

## P77 release naming conflicts with pump PWM

**Symptoms:** A test expects `P77_TOUCH_RELEASE` to contain pump 1500, but the stable action contains 2500.

**Known behavior:** The stable action table is preserved. Stage 6 performs a separate explicit `#005P1500T0500!` pump-off step.

**Do not:** Edit the stable action table as a test-only fix.
