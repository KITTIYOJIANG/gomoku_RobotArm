"""Tactical heuristic AI reused from gomoku_project/gomoku/ai.py."""

from __future__ import annotations

import random
from typing import Optional

from .core import Coord, GomokuBoard, Stone


DIFFICULTY_CONFIG = {
    "easy": {"search_radius": 1, "max_search_candidates": 8},
    "standard": {"search_radius": 2, "max_search_candidates": 20},
    "hard": {"search_radius": 3, "max_search_candidates": 40},
}


class RandomAI:
    def __init__(self, stone: Stone, *, rng: random.Random | None = None) -> None:
        self.stone = Stone(stone)
        self.rng = rng or random.Random()

    def select_move(self, board: GomokuBoard) -> Optional[Coord]:
        moves = list(board.legal_moves())
        return self.rng.choice(moves) if moves else None


class HeuristicAI:
    SCORES = {1: 10, 2: 80, 3: 1500, 4: 20000, 5: 10000000}

    def __init__(
        self,
        stone: Stone,
        search_radius: int = 2,
        max_search_candidates: int = 20,
        *,
        rng: random.Random | None = None,
    ) -> None:
        self.stone = Stone(stone)
        self.search_radius = int(search_radius)
        self.max_search_candidates = int(max_search_candidates)
        self.rng = rng or random.Random()

    def select_move(self, board: GomokuBoard) -> Optional[Coord]:
        candidates = self._candidate_moves(board)
        if not candidates:
            return None
        for move in candidates:
            if self._is_winning_move(board, move, self.stone):
                return move
        opponent = self._opponent(self.stone)
        for move in candidates:
            if self._is_winning_move(board, move, opponent):
                return move
        danger = self._open_three_blocks(board, opponent)
        blocks = [move for move in candidates if move in danger]
        if blocks:
            return self._best_heuristic(board, blocks, self.stone)
        return self._best_heuristic(board, candidates, self.stone)

    def _best_heuristic(
        self, board: GomokuBoard, candidates: list[Coord], player: Stone
    ) -> Optional[Coord]:
        scored = [(self._evaluate_move(board, move, player), move) for move in candidates]
        if not scored:
            return None
        best = max(score for score, _move in scored)
        moves = [move for score, move in scored if score == best]
        return self.rng.choice(moves)

    def _candidate_moves(self, board: GomokuBoard) -> list[Coord]:
        stones = [
            (row, col)
            for row in range(board.size)
            for col in range(board.size)
            if board.grid[row][col] != Stone.EMPTY
        ]
        if not stones:
            center = board.size // 2
            return [(center, center)]
        min_row = max(min(row for row, _col in stones) - self.search_radius, 0)
        max_row = min(max(row for row, _col in stones) + self.search_radius, board.size - 1)
        min_col = max(min(col for _row, col in stones) - self.search_radius, 0)
        max_col = min(max(col for _row, col in stones) + self.search_radius, board.size - 1)
        moves = [
            (row, col)
            for row in range(min_row, max_row + 1)
            for col in range(min_col, max_col + 1)
            if board.grid[row][col] == Stone.EMPTY
        ]
        if len(moves) <= self.max_search_candidates:
            return moves
        pivot = board.last_move or (board.size // 2, board.size // 2)
        center = board.size // 2
        moves.sort(
            key=lambda move: (
                abs(move[0] - pivot[0]) + abs(move[1] - pivot[1]),
                abs(move[0] - center) + abs(move[1] - center),
                move,
            )
        )
        return moves[: self.max_search_candidates]

    def _is_winning_move(self, board: GomokuBoard, move: Coord, stone: Stone) -> bool:
        trial = board.copy()
        return trial.place_stone(*move, stone=stone) and trial.check_winner(move) == stone

    def _evaluate_move(self, board: GomokuBoard, move: Coord, player: Stone) -> int:
        trial = board.copy()
        if not trial.place_stone(*move, stone=player):
            return -(10**18)
        return self.evaluate_board(trial, player)

    def evaluate_board(self, board: GomokuBoard, player: Stone) -> int:
        total = 0
        opponent = self._opponent(player)
        for dr, dc in ((1, 0), (0, 1), (1, 1), (1, -1)):
            for row in range(board.size):
                for col in range(board.size):
                    if not board.is_on_board(row + 4 * dr, col + 4 * dc):
                        continue
                    line = [board.grid[row + k * dr][col + k * dc] for k in range(5)]
                    mine = line.count(player)
                    theirs = line.count(opponent)
                    if (mine and theirs) or (not mine and not theirs):
                        continue
                    if mine:
                        total += self.SCORES.get(mine, 0)
                    else:
                        base = self.SCORES.get(theirs, 0)
                        total -= base * (4 if theirs >= 4 else 2 if theirs == 3 else 1)
        return total

    def _open_three_blocks(self, board: GomokuBoard, stone: Stone) -> set[Coord]:
        result: set[Coord] = set()
        for dr, dc in ((1, 0), (0, 1), (1, 1), (1, -1)):
            for row in range(board.size):
                for col in range(board.size):
                    if not board.is_on_board(row + 4 * dr, col + 4 * dc):
                        continue
                    coords = [(row + k * dr, col + k * dc) for k in range(5)]
                    line = [board.grid[r][c] for r, c in coords]
                    if line == [Stone.EMPTY, stone, stone, stone, Stone.EMPTY]:
                        result.update((coords[0], coords[4]))
        return result

    @staticmethod
    def _opponent(stone: Stone) -> Stone:
        return Stone.BLACK if stone == Stone.WHITE else Stone.WHITE
