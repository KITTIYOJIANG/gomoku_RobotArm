from __future__ import annotations

# When True, Stage5 hover/return never live-send (old learning-sprint lock).
# Default False: user controls live vs dry via Stage5 DRY RUN checkbox.
# Cross-anchor wizard can still force mock via its own force_dry_run flag.
FORCE_STAGE5_DRY_RUN = False

CROSS_ANCHORS: tuple[tuple[int, int, str, str], ...] = (
    (3, 7, "CENTER_UP", "中心上"),
    (11, 7, "CENTER_DOWN", "中心下"),
    (7, 3, "CENTER_LEFT", "中心左"),
    (7, 11, "CENTER_RIGHT", "中心右"),
)

# Diagonal corners of the center 3x3 anchor lattice (star positions).
STAR_CORNERS: tuple[tuple[int, int, str, str], ...] = (
    (3, 3, "STAR_UL", "星位左上"),
    (3, 11, "STAR_UR", "星位右上"),
    (11, 3, "STAR_DL", "星位左下"),
    (11, 11, "STAR_DR", "星位右下"),
)

PROTECTED_ANCHOR = (7, 7)
P77_KEY = "7,7"
SPATIAL_JOINTS = ("000", "001", "002", "003", "004")
DEFAULT_REQUIRED_RUNS = 3


def move_confirm_token(row: int, col: int) -> str:
    return f"MOVE P{int(row)}{int(col)}"


def anchor_key(row: int, col: int) -> str:
    return f"{int(row)},{int(col)}"


# Outer ring for full 15x15 coverage (beyond center r3..11 / c3..11).
# Teach these after star corners; expands allowed_region to full board.
OUTER_RING: tuple[tuple[int, int, str, str], ...] = (
    (0, 7, "OUTER_TOP", "外圈上"),
    (14, 7, "OUTER_BOTTOM", "外圈下"),
    (7, 0, "OUTER_LEFT", "外圈左"),
    (7, 14, "OUTER_RIGHT", "外圈右"),
    (0, 0, "OUTER_UL", "外圈左上"),
    (0, 14, "OUTER_UR", "外圈右上"),
    (14, 0, "OUTER_DL", "外圈左下"),
    (14, 14, "OUTER_DR", "外圈右下"),
)
