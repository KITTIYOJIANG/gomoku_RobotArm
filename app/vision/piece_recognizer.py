from __future__ import annotations

from dataclasses import dataclass
import time

import numpy as np

from app.config import PieceRecognitionConfig

from .overlay import grid_points_from_homography
from .stone_detector import BLACK, EMPTY, WHITE, detect_stones


@dataclass(frozen=True)
class PieceRecognitionResult:
    board_matrix: tuple[tuple[int, ...], ...]
    black_count: int
    white_count: int
    empty_count: int
    timestamp: float

    @property
    def summary(self) -> str:
        return f"BLACK={self.black_count} WHITE={self.white_count} EMPTY={self.empty_count}"


class PieceRecognizer:
    """Reuses the original detector on the pristine detection frame."""

    def __init__(self, config: PieceRecognitionConfig) -> None:
        self.config = config

    def recognize(
        self,
        detection_frame: np.ndarray,
        *,
        homography: np.ndarray,
        board_size: int,
    ) -> PieceRecognitionResult:
        grid = grid_points_from_homography(homography, board_size)
        matrix = detect_stones(
            detection_frame,
            grid,
            roi_radius=self.config.roi_radius,
            bg_radius=self.config.background_radius,
            black_diff=self.config.black_diff,
            white_diff=self.config.white_diff,
            black_area_ratio=self.config.black_area_ratio,
            white_area_ratio=self.config.white_area_ratio,
        )
        immutable = tuple(tuple(int(value) for value in row) for row in matrix)
        flat = [value for row in immutable for value in row]
        return PieceRecognitionResult(
            board_matrix=immutable,
            black_count=flat.count(BLACK),
            white_count=flat.count(WHITE),
            empty_count=flat.count(EMPTY),
            timestamp=time.time(),
        )
