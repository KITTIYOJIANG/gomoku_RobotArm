from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import random
from typing import Sequence

from app.integrated_v1.points import PointRef
from app.robot_api import row_col_to_point_id

from .ai import DIFFICULTY_CONFIG, HeuristicAI
from .core import GomokuBoard, Stone


class GameMode(str, Enum):
    HUMAN_HUMAN = "HUMAN_HUMAN"
    HUMAN_AI = "HUMAN_AI"


@dataclass(frozen=True)
class GameMove:
    point: PointRef
    stone: Stone
    point_id: str


class GameSession:
    """UI-independent game state; robot output is point_id only."""

    def __init__(self, *, mode: GameMode | str = GameMode.HUMAN_AI, seed: int = 0) -> None:
        self.mode = GameMode(mode)
        self.board = GomokuBoard()
        self.ai_stone = Stone.WHITE
        self.human_stone = Stone.BLACK
        self.seed = int(seed)
        self.difficulty = "standard"
        self.ai = self._build_ai()
        self.pending_robot_move: GameMove | None = None
        self.winner: Stone | None = None
        self.move_history: list[GameMove] = []

    def reset(self, *, mode: GameMode | str | None = None) -> None:
        if mode is not None:
            self.mode = GameMode(mode)
        self.board.reset()
        self.pending_robot_move = None
        self.winner = None
        self.move_history.clear()

    def set_ai_difficulty(self, difficulty: str) -> None:
        selected = str(difficulty).strip().lower()
        if selected not in DIFFICULTY_CONFIG:
            raise ValueError(f"unknown AI difficulty: {difficulty}")
        self.difficulty = selected
        self.ai = self._build_ai()

    def _build_ai(self) -> HeuristicAI:
        config = DIFFICULTY_CONFIG[self.difficulty]
        return HeuristicAI(
            self.ai_stone,
            search_radius=config["search_radius"],
            max_search_candidates=config["max_search_candidates"],
            rng=random.Random(self.seed),
        )

    def human_move(self, row: int, col: int) -> bool:
        if self.pending_robot_move is not None or self.winner is not None:
            return False
        stone = self.board.current_player
        if self.mode == GameMode.HUMAN_AI and stone != self.human_stone:
            return False
        if not self.board.place_stone(row, col, stone=stone):
            return False
        point = PointRef(int(row), int(col))
        self.move_history.append(
            GameMove(point, stone, row_col_to_point_id(point.row, point.col))
        )
        self.winner = self.board.check_winner((row, col))
        return True

    def choose_ai_move(self) -> GameMove | None:
        if self.mode != GameMode.HUMAN_AI or self.winner is not None:
            return None
        if self.pending_robot_move is not None:
            return self.pending_robot_move
        if self.board.current_player != self.ai_stone:
            return None
        move = self.ai.select_move(self.board.copy())
        if move is None:
            return None
        point = PointRef(*move)
        self.pending_robot_move = GameMove(
            point=point,
            stone=self.ai_stone,
            point_id=row_col_to_point_id(point.row, point.col),
        )
        return self.pending_robot_move

    def complete_robot_move(self, *, success: bool) -> bool:
        pending = self.pending_robot_move
        if pending is None:
            return False
        if not success:
            self.pending_robot_move = None
            return False
        placed = self.board.place_stone(
            pending.point.row, pending.point.col, stone=pending.stone
        )
        if placed:
            self.move_history.append(pending)
            self.winner = self.board.check_winner(pending.point.as_tuple())
        self.pending_robot_move = None
        return placed

    def apply_vision_matrix(self, matrix: Sequence[Sequence[int | Stone]]) -> None:
        """Reserved vision synchronization boundary; never emits robot PWM."""

        if self.pending_robot_move is not None:
            point = self.pending_robot_move.point
            observed = Stone(matrix[point.row][point.col])
            if observed == self.pending_robot_move.stone:
                self.complete_robot_move(success=True)
                return
        self.board.load_matrix(matrix)
