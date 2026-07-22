from __future__ import annotations

# Hard safety gate for this development sprint.
# Even if the GUI dry-run checkbox is unchecked, no live arm TX is allowed.
FORCE_STAGE5_DRY_RUN = True

CROSS_ANCHORS: tuple[tuple[int, int, str, str], ...] = (
    (3, 7, "CENTER_UP", "中心上"),
    (11, 7, "CENTER_DOWN", "中心下"),
    (7, 3, "CENTER_LEFT", "中心左"),
    (7, 11, "CENTER_RIGHT", "中心右"),
)

PROTECTED_ANCHOR = (7, 7)
P77_KEY = "7,7"
SPATIAL_JOINTS = ("000", "001", "002", "003", "004")
DEFAULT_REQUIRED_RUNS = 3


def move_confirm_token(row: int, col: int) -> str:
    return f"MOVE P{int(row)}{int(col)}"


def anchor_key(row: int, col: int) -> str:
    return f"{int(row)},{int(col)}"
