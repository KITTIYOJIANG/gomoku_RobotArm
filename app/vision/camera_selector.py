from __future__ import annotations

import os
import re
import subprocess


PREFERRED_CAMERA_NAME = "USB 2.0 Camera"
SKIP_CAMERA_KEYWORDS = ("ivcam",)
FALLBACK_CAMERA_ID = 0


def list_dshow_video_devices() -> list[str]:
    """List DirectShow video device names in OpenCV index order."""

    try:
        result = subprocess.run(
            ["ffmpeg", "-hide_banner", "-list_devices", "true", "-f", "dshow", "-i", "dummy"],
            capture_output=True,
            timeout=10,
        )
    except (FileNotFoundError, subprocess.SubprocessError):
        return []

    stdout = (result.stdout or b"").decode("utf-8", errors="replace")
    stderr = (result.stderr or b"").decode("utf-8", errors="replace")
    text = stdout + "\n" + stderr

    names: list[str] = []
    for line in text.splitlines():
        match = re.search(r'\] "(.+)" \(video\)', line)
        if match:
            names.append(match.group(1))
    return names


def is_skipped_camera(name: str | None) -> bool:
    if not name:
        return False
    normalized = name.lower()
    return any(keyword in normalized for keyword in SKIP_CAMERA_KEYWORDS)


def find_camera_id_by_name(camera_name: str = PREFERRED_CAMERA_NAME) -> int | None:
    names = list_dshow_video_devices()
    target = camera_name.strip().lower()
    for index, name in enumerate(names):
        if name.strip().lower() == target and not is_skipped_camera(name):
            return index
    return None


def resolve_preferred_camera_id(
    camera_name: str | None = None,
    fallback: int = FALLBACK_CAMERA_ID,
) -> int:
    """Resolve the board camera ID by device name, with numeric fallback."""

    env_id = os.getenv("GOMOKU_CAMERA_ID")
    if env_id:
        try:
            return int(env_id)
        except ValueError:
            pass

    preferred_name = camera_name or os.getenv("GOMOKU_CAMERA_NAME") or PREFERRED_CAMERA_NAME
    resolved = find_camera_id_by_name(preferred_name)
    if resolved is not None:
        return resolved
    return fallback


def describe_camera_id(camera_id: int) -> str:
    names = list_dshow_video_devices()
    if 0 <= camera_id < len(names):
        return names[camera_id]
    return "unknown camera"
