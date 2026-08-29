from __future__ import annotations

import subprocess
import sys

from app.config import PROJECT_ROOT


def open_quick_calibration() -> None:
    """Public entrypoint for the player/application UI.

    Opens Calibration Lite as an independent process.
    The caller must not depend on Calibration Lite internal classes.
    """
    subprocess.Popen(
        [
            sys.executable,
            "-m",
            "app.calibration_lite.main",
        ],
        cwd=str(PROJECT_ROOT),
    )