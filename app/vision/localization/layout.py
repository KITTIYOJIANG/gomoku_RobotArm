from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np


ROLE_ORDER = ("top_left", "top_right", "bottom_right", "bottom_left")


@dataclass(frozen=True)
class TagPlacement:
    tag_id: int
    role: str
    center_board: tuple[float, float]
    rotation_deg: float


@dataclass(frozen=True)
class AprilTagBoardLayout:
    tag_family: str
    tag_size_mm: float | None
    board_width_mm: float | None
    board_height_mm: float | None
    tag_size_board_units: tuple[float, float]
    tags: dict[int, TagPlacement]
    playable_corners_board: np.ndarray
    warp_width_px: int
    warp_height_px: int
    minimum_tag_count: int
    minimum_decision_margin: float
    maximum_hamming: int
    maximum_reprojection_error: float
    minimum_board_area_ratio: float
    maximum_board_area_ratio: float
    pose_jump_threshold: float
    homography_jump_threshold: float
    stable_frame_count: int

    @classmethod
    def from_file(cls, path: str | Path) -> "AprilTagBoardLayout":
        source = Path(path)
        data = json.loads(source.read_text(encoding="utf-8"))
        return cls.from_dict(data)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "AprilTagBoardLayout":
        tag_entries = data.get("tags", {})
        tags: dict[int, TagPlacement] = {}
        roles: set[str] = set()
        for id_text, item in tag_entries.items():
            tag_id = int(id_text)
            role = str(item["role"])
            if role not in ROLE_ORDER:
                raise ValueError(f"Unsupported tag role: {role}")
            if role in roles:
                raise ValueError(f"Duplicate tag role: {role}")
            center = tuple(float(v) for v in item["center_board"])
            if len(center) != 2:
                raise ValueError(f"Tag {tag_id} center_board must contain x,y")
            tags[tag_id] = TagPlacement(
                tag_id=tag_id,
                role=role,
                center_board=(center[0], center[1]),
                rotation_deg=float(item.get("rotation_deg", 0.0)),
            )
            roles.add(role)

        if roles != set(ROLE_ORDER):
            raise ValueError(f"Layout must define exactly these roles: {ROLE_ORDER}")

        playable = np.asarray(data["playable_corners_board"], dtype=np.float64)
        if playable.shape != (4, 2):
            raise ValueError("playable_corners_board must have shape (4,2)")

        tag_size_units = tuple(float(v) for v in data["tag_size_board_units"])
        if len(tag_size_units) != 2 or min(tag_size_units) <= 0:
            raise ValueError("tag_size_board_units must contain two positive values")

        minimum_tag_count = int(data.get("minimum_tag_count", 3))
        if minimum_tag_count not in (3, 4):
            raise ValueError("minimum_tag_count must be 3 or 4")

        stable_frame_count = int(data.get("stable_frame_count", 3))
        if stable_frame_count < 1:
            raise ValueError("stable_frame_count must be at least 1")

        return cls(
            tag_family=str(data["tag_family"]),
            tag_size_mm=_optional_positive_float(data.get("tag_size_mm")),
            board_width_mm=_optional_positive_float(data.get("board_width_mm")),
            board_height_mm=_optional_positive_float(data.get("board_height_mm")),
            tag_size_board_units=(tag_size_units[0], tag_size_units[1]),
            tags=tags,
            playable_corners_board=playable,
            warp_width_px=int(data.get("warp_width_px", 600)),
            warp_height_px=int(data.get("warp_height_px", 600)),
            minimum_tag_count=minimum_tag_count,
            minimum_decision_margin=float(data.get("minimum_decision_margin", 20.0)),
            maximum_hamming=int(data.get("maximum_hamming", 0)),
            maximum_reprojection_error=float(data.get("maximum_reprojection_error", 3.0)),
            minimum_board_area_ratio=float(data.get("minimum_board_area_ratio", 0.05)),
            maximum_board_area_ratio=float(data.get("maximum_board_area_ratio", 0.95)),
            pose_jump_threshold=float(data.get("pose_jump_threshold", 15.0)),
            homography_jump_threshold=float(data.get("homography_jump_threshold", 25.0)),
            stable_frame_count=stable_frame_count,
        )

    @property
    def allowed_tag_ids(self) -> set[int]:
        return set(self.tags)

    def tag_object_corners(self, tag_id: int) -> np.ndarray:
        placement = self.tags[tag_id]
        half_x = self.tag_size_board_units[0] / 2.0
        half_y = self.tag_size_board_units[1] / 2.0
        # pupil-apriltags order: bottom-left, bottom-right, top-right, top-left.
        local = np.array(
            [[-half_x, half_y], [half_x, half_y], [half_x, -half_y], [-half_x, -half_y]],
            dtype=np.float64,
        )
        angle = np.deg2rad(placement.rotation_deg)
        rotation = np.array(
            [[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]],
            dtype=np.float64,
        )
        center = np.asarray(placement.center_board, dtype=np.float64)
        return local @ rotation.T + center


def _optional_positive_float(value: Any) -> float | None:
    if value is None:
        return None
    parsed = float(value)
    if parsed <= 0:
        raise ValueError("Physical measurements must be positive when provided")
    return parsed
