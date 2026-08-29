# Repository Instructions for Coding Agents

Before editing, read `START_HERE.md`, `project_state.json`, `docs/ARCHITECTURE.md`, and `docs/CALIBRATION.md`; then run `python tools/project_doctor.py` and inspect `git status --short --untracked-files=all`.

- Preserve the existing dirty working tree and never clean, move, or overwrite unrelated user work.
- Treat `calibration/stage5_board_calibration.json` and `config/arm_actions.json` as frozen stable assets unless the user explicitly requests revalidation work.
- Do not change P77 PWM, pump semantics, motion ordering, descent/reverse-ascent guards, serial protocol, or hardware safety defaults as incidental fixes.
- Do not connect a camera or serial port, send commands, compile/flash firmware, or run hardware-capable tests without explicit user authorization.
- Label evidence exactly as `HARDWARE VERIFIED`, `OFFLINE VERIFIED`, or `NOT VERIFIED`; dry-run is never a hardware pass.
- After a milestone, follow `docs/DEVELOPMENT_WORKFLOW.md` and update both `project_state.json` and `START_HERE.md`.
