from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys
from typing import Any


ROOT = Path(__file__).resolve().parents[1]


def emit(text: str = "") -> None:
    line = str(text) + "\n"
    try:
        sys.stdout.write(line)
    except UnicodeEncodeError:
        stream = getattr(sys.stdout, "buffer", None)
        if stream is not None:
            stream.write(line.encode(getattr(sys.stdout, "encoding", None) or "utf-8", "replace"))


def git(*args: str) -> str:
    try:
        result = subprocess.run(
            ["git", *args],
            cwd=ROOT,
            text=True,
            encoding="utf-8",
            errors="replace",
            capture_output=True,
            check=False,
            timeout=10,
        )
    except (OSError, subprocess.TimeoutExpired):
        return "UNKNOWN"
    return result.stdout.strip() if result.returncode == 0 and result.stdout.strip() else "UNKNOWN"


def read_state() -> dict[str, Any]:
    path = ROOT / "project_state.json"
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError("project_state.json must contain an object")
    return value


def section(title: str, value: str) -> None:
    emit()
    emit(title)
    emit(value)


def main() -> int:
    try:
        state = read_state()
    except Exception as exc:
        emit(f"Cannot resume project: {exc}")
        return 1

    branch = git("branch", "--show-current")
    latest = git("log", "-1", "--format=%h %s")
    porcelain = git("status", "--porcelain=v1", "--untracked-files=all")
    dirty_count = 0 if porcelain == "UNKNOWN" else len(porcelain.splitlines())
    stable = state.get("last_stable") or {}

    emit("Gomoku Robot — Resume Project")
    section("CURRENT", str(state.get("current_stage", "UNKNOWN")))
    emit(f"Branch: {branch}")
    emit(f"Latest: {latest}")
    emit(f"Working tree: {'UNKNOWN' if porcelain == 'UNKNOWN' else f'DIRTY ({dirty_count} entries)' if dirty_count else 'CLEAN'}")
    section("STATUS", str(state.get("status", "UNKNOWN")))
    section(
        "LAST STABLE",
        f"{stable.get('tag', 'UNKNOWN')}\nbranch {stable.get('branch', 'UNKNOWN')}\ncommit {stable.get('commit', 'UNKNOWN')}",
    )
    working = state.get("working_features") or []
    section("WORKING", "\n".join(f"- {item}" for item in working[:5]) or "- UNKNOWN")
    section("CURRENT TASK", str(state.get("current_goal", "UNKNOWN")))
    section("NEXT", str(state.get("next_task", "UNKNOWN")))

    warnings = state.get("known_issues") or []
    safety = state.get("safe_defaults") or {}
    warning_lines = [
        "- Use the dry-run entry before any hardware-capable GUI.",
        f"- Project-wide force dry-run: {safety.get('project_wide_force_dry_run', 'UNKNOWN')}",
    ]
    warning_lines.extend(f"- {item}" for item in warnings[:4])
    section("WARNING", "\n".join(warning_lines))
    section("READ", "START_HERE.md")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
