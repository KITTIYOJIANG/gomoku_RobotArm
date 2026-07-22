from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from .localization.camera_intrinsics import CameraIntrinsics


def load_camera_intrinsics(
    primary: Path,
    fallback: Path | None = None,
) -> tuple[CameraIntrinsics | None, str | None]:
    """Load real intrinsics if present; never invent values from the example."""

    candidates = [primary]
    if fallback is not None and fallback != primary:
        candidates.append(fallback)
    last_error = "CAMERA_INTRINSICS_MISSING"
    for path in candidates:
        if not path.exists():
            continue
        try:
            return CameraIntrinsics.from_file(path), None
        except (ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
            last_error = str(exc)
    return None, last_error


def load_legacy_board_corners(path: Path) -> np.ndarray | None:
    """Read the legacy manual corners for diagnostics; AprilTag remains authoritative."""

    if not path.exists():
        return None
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        corners = np.asarray(data["corners"], dtype=np.float32)
    except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError):
        return None
    return corners if corners.shape == (4, 2) else None
