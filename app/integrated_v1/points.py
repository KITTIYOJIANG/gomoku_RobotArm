from __future__ import annotations

from dataclasses import dataclass
import re
from typing import Iterable


BOARD_SIZE = 15
_EXPLICIT = re.compile(r"^P?(?P<row>\d{1,2})\s*[_,:/]\s*(?P<col>\d{1,2})$", re.I)
_GOLDEN_ALIASES = {
    "P33": (3, 3),
    "P311": (3, 11),
    "P77": (7, 7),
    "P113": (11, 3),
    "P1111": (11, 11),
}


@dataclass(frozen=True, order=True)
class PointRef:
    row: int
    col: int

    def __post_init__(self) -> None:
        validate_coordinate(self.row, self.col)

    @property
    def point_id(self) -> str:
        return format_point_id(self.row, self.col)

    @property
    def flat_id(self) -> str:
        return f"P{self.row * BOARD_SIZE + self.col:03d}"

    def as_tuple(self) -> tuple[int, int]:
        return self.row, self.col


def validate_coordinate(row: int, col: int) -> tuple[int, int]:
    row_i, col_i = int(row), int(col)
    if not (0 <= row_i < BOARD_SIZE and 0 <= col_i < BOARD_SIZE):
        raise ValueError(f"point outside {BOARD_SIZE}x{BOARD_SIZE} board: ({row_i},{col_i})")
    return row_i, col_i


def format_point_id(row: int, col: int) -> str:
    """Return an unambiguous public point id while keeping familiar short ids.

    P77/P33 remain valid. Coordinates containing a two-digit axis use an
    underscore (for example P3_11) so P111 can never silently mean the wrong
    physical point.
    """

    row_i, col_i = validate_coordinate(row, col)
    if row_i < 10 and col_i < 10:
        return f"P{row_i}{col_i}"
    return f"P{row_i}_{col_i}"


def parse_point_id(value: str | PointRef | tuple[int, int] | list[int]) -> PointRef:
    if isinstance(value, PointRef):
        return value
    if isinstance(value, (tuple, list)) and len(value) == 2:
        return PointRef(*validate_coordinate(int(value[0]), int(value[1])))
    text = str(value).strip().upper()
    if text in _GOLDEN_ALIASES:
        return PointRef(*_GOLDEN_ALIASES[text])
    match = _EXPLICIT.fullmatch(text)
    if match:
        return PointRef(
            *validate_coordinate(int(match.group("row")), int(match.group("col")))
        )
    if re.fullmatch(r"P\d{2}", text):
        return PointRef(*validate_coordinate(int(text[1]), int(text[2])))
    if re.fullmatch(r"P#\d{3}", text) or re.fullmatch(r"F\d{3}", text):
        flat = int(text[-3:])
        if not 0 <= flat < BOARD_SIZE * BOARD_SIZE:
            raise ValueError(f"flat point id outside board: {text}")
        return PointRef(*divmod(flat, BOARD_SIZE))
    raise ValueError(
        f"ambiguous or invalid point_id {value!r}; use P7_11 or flat form P#116"
    )


def all_points() -> Iterable[PointRef]:
    for row in range(BOARD_SIZE):
        for col in range(BOARD_SIZE):
            yield PointRef(row, col)
