"""Gomoku rules reused from gomoku_project/gomoku/core.py."""

from __future__ import annotations

from enum import IntEnum
from typing import Iterable, Optional, Sequence, Tuple


BOARD_SIZE = 15
Coord = Tuple[int, int]


class Stone(IntEnum):
    EMPTY = 0
    BLACK = 1
    WHITE = 2


class GomokuBoard:
    def __init__(self, size: int = BOARD_SIZE) -> None:
        if int(size) != BOARD_SIZE:
            raise ValueError("Integrated V1 requires a 15x15 board")
        self.size = int(size)
        self.grid: list[list[Stone]] = []
        self.current_player = Stone.BLACK
        self.last_move: Optional[Coord] = None
        self.move_count = 0
        self.reset()

    def reset(self) -> None:
        self.grid = [[Stone.EMPTY for _ in range(self.size)] for _ in range(self.size)]
        self.current_player = Stone.BLACK
        self.last_move = None
        self.move_count = 0

    def copy(self) -> "GomokuBoard":
        result = GomokuBoard(self.size)
        result.grid = [list(row) for row in self.grid]
        result.current_player = self.current_player
        result.last_move = self.last_move
        result.move_count = self.move_count
        return result

    def load_matrix(self, matrix: Sequence[Sequence[int | Stone]]) -> None:
        if len(matrix) != self.size or any(len(row) != self.size for row in matrix):
            raise ValueError("vision board matrix must be exactly 15x15")
        grid = [[Stone(value) for value in row] for row in matrix]
        self.grid = grid
        self.move_count = sum(cell != Stone.EMPTY for row in grid for cell in row)
        black = sum(cell == Stone.BLACK for row in grid for cell in row)
        white = sum(cell == Stone.WHITE for row in grid for cell in row)
        self.current_player = Stone.BLACK if black <= white else Stone.WHITE

    def is_on_board(self, row: int, col: int) -> bool:
        return 0 <= int(row) < self.size and 0 <= int(col) < self.size

    def is_valid_move(self, row: int, col: int) -> bool:
        return self.is_on_board(row, col) and self.grid[int(row)][int(col)] == Stone.EMPTY

    def place_stone(self, row: int, col: int, *, stone: Stone | None = None) -> bool:
        row_i, col_i = int(row), int(col)
        if not self.is_valid_move(row_i, col_i):
            return False
        placed = self.current_player if stone is None else Stone(stone)
        self.grid[row_i][col_i] = placed
        self.last_move = row_i, col_i
        self.move_count += 1
        self.current_player = Stone.BLACK if placed == Stone.WHITE else Stone.WHITE
        return True

    def check_winner(self, coord: Optional[Coord] = None) -> Optional[Stone]:
        target = self.last_move if coord is None else coord
        if target is None or not self.is_on_board(*target):
            return None
        row, col = target
        player = self.grid[row][col]
        if player == Stone.EMPTY:
            return None
        for dr, dc in ((1, 0), (0, 1), (1, 1), (1, -1)):
            count = 1
            for sign in (-1, 1):
                r, c = row + sign * dr, col + sign * dc
                while self.is_on_board(r, c) and self.grid[r][c] == player:
                    count += 1
                    r += sign * dr
                    c += sign * dc
            if count >= 5:
                return player
        return None

    def is_full(self) -> bool:
        return self.move_count >= self.size * self.size

    def legal_moves(self) -> Iterable[Coord]:
        for row in range(self.size):
            for col in range(self.size):
                if self.is_valid_move(row, col):
                    yield row, col
