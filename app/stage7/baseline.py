from __future__ import annotations

from dataclasses import dataclass
import hashlib
from pathlib import Path
from typing import Any

from app.arm.actions import ActionLibrary
from app.stage5.calibration_store import CalibrationStore
from app.stage5.pwm_interpolator import resolve_target_pwm


SPATIAL_KEYS = ("000", "001", "002", "003", "004")


def point_id(row: int, col: int, board_size: int = 15) -> str:
    r, c, size = int(row), int(col), int(board_size)
    if not 0 <= r < size or not 0 <= c < size:
        raise ValueError(f"board point ({r},{c}) outside {size}x{size}")
    return f"P{r * size + c:03d}"


def point_label(row: int, col: int, board_size: int = 15) -> str:
    return f"{point_id(row, col, board_size)}  ({int(row)},{int(col)})"


def point_from_index(index: int, board_size: int = 15) -> tuple[int, int]:
    value, size = int(index), int(board_size)
    if not 0 <= value < size * size:
        raise ValueError(f"point index {value} outside 0..{size * size - 1}")
    return divmod(value, size)


@dataclass(frozen=True)
class BaselinePoint:
    row: int
    col: int
    point_id: str
    pwm: dict[str, int]
    source: str
    anchors_used: tuple[str, ...]
    verified: bool

    def to_dict(self) -> dict[str, Any]:
        return {
            "point_id": self.point_id,
            "board_row": self.row,
            "board_col": self.col,
            "pwm": dict(self.pwm),
            "source": self.source,
            "anchors_used": list(self.anchors_used),
            "verified": self.verified,
        }


class BaselineSnapshot:
    """Read-only resolved view of the stable Stage 5 ABOVE calibration."""

    def __init__(
        self,
        path: str | Path,
        *,
        board_size: int = 15,
        library: ActionLibrary | None = None,
    ) -> None:
        self.path = Path(path)
        self.board_size = int(board_size)
        self.library = library or ActionLibrary()
        before = self.sha256()
        store = CalibrationStore(self.path, library=self.library, safety_limits=None)
        if store.board_size != self.board_size:
            raise ValueError(
                f"baseline board size {store.board_size} != expected {self.board_size}"
            )
        records: dict[tuple[int, int], BaselinePoint] = {}
        for row in range(self.board_size):
            for col in range(self.board_size):
                resolved = resolve_target_pwm(
                    store,
                    row,
                    col,
                    limits=None,
                    allow_star_seed=True,
                    allow_outer_seed=True,
                )
                pwm = resolved.pwm_str_keys()
                if tuple(sorted(pwm)) != SPATIAL_KEYS:
                    raise ValueError(f"baseline P({row},{col}) has invalid joint keys")
                anchor = store.get_anchor(row, col)
                verified = bool(
                    resolved.source == "direct_anchor"
                    and anchor is not None
                    and anchor.calibrated
                    and anchor.verified_runs > 0
                )
                records[(row, col)] = BaselinePoint(
                    row=row,
                    col=col,
                    point_id=point_id(row, col, self.board_size),
                    pwm={key: int(pwm[key]) for key in SPATIAL_KEYS},
                    source=resolved.source,
                    anchors_used=tuple(resolved.anchors_used),
                    verified=verified,
                )
        after = self.sha256()
        if before != after:
            raise RuntimeError("baseline calibration changed while Stage 7 loaded it")
        if len(records) != self.board_size * self.board_size:
            raise RuntimeError("baseline did not resolve all board intersections")
        self.source_sha256 = before
        self._records = records

    def sha256(self) -> str:
        return hashlib.sha256(self.path.read_bytes()).hexdigest().upper()

    def assert_unchanged(self) -> None:
        if self.sha256() != self.source_sha256:
            raise RuntimeError("stable baseline changed after Stage 7 session creation")

    def get(self, row: int, col: int) -> BaselinePoint:
        try:
            return self._records[(int(row), int(col))]
        except KeyError as exc:
            raise ValueError(f"board point ({row},{col}) outside baseline") from exc

    def all_points(self) -> dict[tuple[int, int], BaselinePoint]:
        self.assert_unchanged()
        return dict(self._records)

    @property
    def direct_count(self) -> int:
        return sum(point.source == "direct_anchor" for point in self._records.values())
