# Development Workflow

## Re-entering the Project

1. Read `START_HERE.md` and `project_state.json`.
2. Run `git status --short --untracked-files=all`; preserve unrelated dirty-tree work.
3. Run `python tools/project_doctor.py` and resolve only critical read/config/environment failures.
4. Run `python tools/resume_project.py` and confirm the stable tag, current goal and one next task.
5. Run `python tools/smoke_test.py` before opening the hardware-capable GUI.
6. Read the stage-specific report and calibration rules for the task being resumed.

## Verification Levels

Use exactly these meanings:

- **HARDWARE VERIFIED:** a human-authorized real hardware run occurred, its exact scope/result is recorded, and evidence is retained. A serial send alone should be described more narrowly when there is no arrival feedback.
- **OFFLINE VERIFIED:** unit/integration/dry-run/simulation checks passed without real motion.
- **NOT VERIFIED:** planned, computed, incomplete, or lacking evidence.

`COMPUTED` and `DRY_RUN_PASSED` are offline states. They are never synonyms for hardware pass.

## Milestone Update Rule

After every important milestone:

1. Run the smoke test, focused tests and then the full suite in the supported Python environment.
2. Record exact pass/fail counts and whether camera, serial or motion was used.
3. Record verification level per feature; do not collapse mixed evidence into one “passed”.
4. Update `project_state.json`: stage, status, stable/current commits, features, issues, goal, one next task and date.
5. Update `START_HERE.md` so its summary matches the machine state.
6. Update calibration hashes/counts and known issues only from actual outputs.
7. Update the relevant report/decision/troubleshooting entry when behavior or a recovery fact changed.
8. Review `git diff --stat`, `git diff`, and `git status --short --untracked-files=all` for accidental calibration, PWM, log, firmware or temporary-file changes.
9. Create a stable tag only when the intended milestone is reviewable, reproducible and verified at its claimed level.

## Stable Asset Change Protocol

Changes to `calibration/stage5_board_calibration.json`, `config/arm_actions.json`, P77 sequences, Stage 6 force-dry-run, or serial protocol require explicit scope and revalidation.

Before such a change:

1. Identify the exact reason and expected physical effect.
2. Record the original Git revision and SHA-256.
3. Write a new candidate/session artifact when possible; do not overwrite in place.
4. Run offline validation and inspect the generated PWM/path.
5. Obtain explicit authorization before any hardware action.
6. Start with the lowest-risk single-point, empty-tool test and prepare physical power cutoff.
7. Record result, operator observation and final hash; roll back on any uncertainty.

## Branch and Tag Guidance

- `main` at `7a1e125...` with tag `J1_Gomoku_V0.2_Arbitrary_Hover_Stable` is the last stable baseline found in the audit.
- `stage6-full-board-descent` is a development branch; its Stage 6 result is offline-only.
- Do not tag the current Integrated V1 working tree until its intended files are selected, the bounded hardware checklist is reviewed, and verification claims remain scoped. The 2026-08-27 software milestone passes the full 214-test suite and 8 safe smoke checks; new V1 motion is still **NOT VERIFIED**.

## Evidence to Keep

Prefer small, reviewable evidence:

- JSON calibration/session files with provenance and hashes.
- A concise stage delivery report with exact counts.
- Test command and pass/fail summary.
- A hardware verification log that states point, load, path, operator and result.

Runtime logs and camera frames are currently untracked. If one becomes essential evidence, intentionally curate a small artifact and document it; do not commit an entire logs directory by accident.
