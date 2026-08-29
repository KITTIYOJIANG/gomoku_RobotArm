# Engineering Decisions

These records summarize decisions supported by current code, reports, and Git history. A decision marked **Planned** is not a claim of completed behavior.

## Decision 001 — Preserve calibrated PWM as the stable baseline

**Status:** Accepted

**Problem:** Copied factory kinematics contain conflicting geometry comments, installed EEPROM bias is unavailable, and the deployed arm has empirically taught poses.

**Decision:** Keep actual spatial PWM in `calibration/stage5_board_calibration.json` and stable fixed actions in `config/arm_actions.json`. Use IK only to generate separately reviewable candidates.

**Reason:** The repository has direct hardware-taught PWM evidence but no complete physical/kinematic identification.

**Consequences:** Baseline files are frozen assets. Model improvements cannot silently rewrite them.

## Decision 002 — Direct anchors plus interpolation for ABOVE coverage

**Status:** Accepted

**Problem:** Persisting and manually teaching all 225 points is costly, while one global IK model is not sufficiently evidenced.

**Decision:** Resolve a direct anchor first, otherwise use the tightest calibrated bilinear cell, with explicit guarded seed fallbacks for expansion.

**Reason:** This preserves measured local structure and provenance while providing full-board ABOVE lookup.

**Consequences:** “225 readable” does not mean “225 directly verified”; direct and interpolated provenance must stay visible.

## Decision 003 — Route through carry-high and ABOVE

**Status:** Accepted

**Problem:** Direct low horizontal moves or diagonal cuts can cross the board, fixtures, or pieces.

**Decision:** Target changes route through carry-high; approach reaches ABOVE before any descent; return follows the reverse low-level path before horizontal travel.

**Reason:** `app/arm/sequences.py`, Stage 5 hover planning, and the Stage 6 state machine all encode this safety boundary.

**Consequences:** Shorter direct paths are invalid even when their endpoints appear safe.

## Decision 004 — Separate ABOVE and descent calibration

**Status:** Accepted

**Problem:** A stable hovering board map should not be destabilized by an unverified Cartesian descent model.

**Decision:** Stage 6 reads Stage 5 as a hashed, read-only source and stores descent layers, manual deltas, verification and rejection evidence in a separate file.

**Reason:** It allows vertical-path experimentation without changing known ABOVE behavior.

**Consequences:** Stage 6 generation must never write the Stage 5 baseline; different verification levels coexist by design.

## Decision 005 — Force dry-run for unverified Stage 6 motion

**Status:** Accepted

**Problem:** Candidate geometry and PWM/angle bias are not physically proven over the board.

**Decision:** Keep `config/stage6_descent.json::force_dry_run` true and prevent offline output from being marked hardware verified.

**Reason:** The Stage 6 delivery explicitly records zero real serial commands and identifies P77 empty-tool descent as the only future experiment.

**Consequences:** Enabling live Stage 6 is a deliberate revalidation event, not a routine configuration tweak.

## Decision 006 — Decouple camera localization from robot PWM calibration

**Status:** Accepted

**Problem:** Camera/device placement affects image geometry, while robot pose teaching affects arm PWM. The repository has no measured hand-eye transform.

**Decision:** Vision produces a board `(row,col)` through homography; robot calibration independently maps that coordinate to PWM.

**Reason:** The existing code and configuration already expose this boundary and avoid claiming metric world coordinates.

**Consequences:** A camera remount requires visual/board revalidation, but must not automatically rewrite robot PWM. Metric pose remains unimplemented.

## Decision 007 — Keep orchestration on the upper computer

**Status:** Accepted

**Problem:** The system needs GUI confirmation, vision, board state, logging, cancellation, sequence ordering and calibration provenance, while the deployed controller interface is an ASCII servo target protocol.

**Decision:** The PC owns those policies and sends ordered servo commands; the controller/servo bus executes the received targets.

**Reason:** This matches `MainWindow`, `ArmSequenceWorker`, `SerialArmController` and the documented deployed protocol.

**Consequences:** Host timing is not real arrival feedback. The GUI state machine and physical operator remain safety-critical.

## Decision 008 — Treat the copied SAFE_STAGE1 firmware as reference only

**Status:** Accepted

**Problem:** The copied reference build rejects raw multi-servo PWM and defines suction ID 005 differently from the deployed host convention.

**Decision:** Do not compile, flash, or cite it as proof of deployed behavior.

**Reason:** `firmware/PROTOCOL_NOTES.md` documents the protocol conflict.

**Consequences:** Exact deployed firmware revision remains UNKNOWN and must be checked on hardware.

## Decision 009 — Overlay Stage 7 deployment deltas on the stable baseline

**Status:** Planned / working-tree draft

**Problem:** Re-deployment may shift the robot/board relationship without invalidating the stable nonlinear baseline shape.

**Decision:** Store direct anchor deltas, interpolate the delta field, add it to a hashed baseline snapshot, and commit to a separate deployment file with rollback.

**Reason:** This is the explicit design in the current `app/stage7` draft.

**Consequences:** It is not accepted as a stable runtime design until its files, integration, tests and dry-run lock are reviewed.
