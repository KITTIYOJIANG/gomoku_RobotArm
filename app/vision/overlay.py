from __future__ import annotations

import cv2
import numpy as np

from .board_tracker import BoardTrackState, CORNER_LABELS


def project_points(homography: np.ndarray, points: np.ndarray) -> np.ndarray:
    values = np.asarray(points, dtype=np.float32).reshape(1, -1, 2)
    return cv2.perspectiveTransform(values, np.asarray(homography, dtype=np.float32)).reshape(-1, 2)


def grid_points_from_homography(homography: np.ndarray, board_size: int) -> np.ndarray:
    if board_size < 2:
        raise ValueError("board_size must be at least two")
    fractions = np.linspace(0.0, 1.0, board_size, dtype=np.float32)
    normalized = np.array(
        [(col, row) for row in fractions for col in fractions],
        dtype=np.float32,
    )
    return project_points(homography, normalized).reshape(board_size, board_size, 2)


def draw_board_corners(
    display_frame: np.ndarray,
    corners: np.ndarray,
    locked: bool,
    frozen: bool,
    *,
    show_coordinates: bool = False,
) -> np.ndarray:
    """Draw ordered TL/TR/BR/BL markers on a display-only frame copy."""

    output = display_frame.copy()
    if not locked and not frozen:
        return output
    _draw_board_corners_inplace(
        output,
        np.asarray(corners, dtype=np.float32).reshape(4, 2),
        show_coordinates=show_coordinates,
    )
    return output


def draw_board_overlay(
    display_frame: np.ndarray,
    *,
    corners: np.ndarray,
    homography: np.ndarray,
    track_state: BoardTrackState,
    board_size: int = 15,
    target_row: int = 7,
    target_col: int = 7,
    target_name: str = "P77",
    selected_row: int | None = None,
    selected_col: int | None = None,
    show_corners: bool = True,
    show_corner_coordinates: bool = False,
) -> np.ndarray:
    """Draw boundary, grid, fixed P77, optional selected target, and corners."""

    output = display_frame.copy()
    ordered = np.asarray(corners, dtype=np.float32).reshape(4, 2)
    h_board_to_image = np.asarray(homography, dtype=np.float32).reshape(3, 3)
    grid = grid_points_from_homography(h_board_to_image, board_size)

    boundary = np.rint(ordered).astype(np.int32).reshape(-1, 1, 2)
    boundary_color = (255, 130, 30) if track_state == BoardTrackState.LOCKED else (0, 190, 255)
    cv2.polylines(output, [boundary], True, boundary_color, 3, cv2.LINE_AA)

    for index in range(board_size):
        horizontal = np.rint(grid[index, (0, -1)]).astype(np.int32)
        vertical = np.rint(grid[(0, -1), index]).astype(np.int32)
        cv2.line(output, tuple(horizontal[0]), tuple(horizontal[1]), (90, 210, 255), 1, cv2.LINE_AA)
        cv2.line(output, tuple(vertical[0]), tuple(vertical[1]), (90, 210, 255), 1, cv2.LINE_AA)

    p77_point = tuple(np.rint(grid[target_row, target_col]).astype(int))
    cv2.circle(output, p77_point, 10, (0, 180, 255), 2, cv2.LINE_AA)
    cv2.putText(
        output,
        f"{target_name} ({target_row},{target_col})",
        _text_origin(output, p77_point, 16, -12),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (0, 180, 255),
        2,
        cv2.LINE_AA,
    )

    if selected_row is not None and selected_col is not None:
        draw_row = int(selected_row)
        draw_col = int(selected_col)
        if 0 <= draw_row < board_size and 0 <= draw_col < board_size:
            target_point = tuple(np.rint(grid[draw_row, draw_col]).astype(int))
            cv2.circle(output, target_point, 14, (0, 0, 255), 3, cv2.LINE_AA)
            cv2.circle(output, target_point, 5, (0, 255, 255), -1, cv2.LINE_AA)
            cv2.putText(
                output,
                f"TARGET P({draw_row},{draw_col})",
                _text_origin(output, target_point, 16, 18),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )
    if show_corners:
        _draw_board_corners_inplace(
            output,
            ordered,
            show_coordinates=show_corner_coordinates,
        )
    return output


def draw_board_grid(
    image: np.ndarray,
    board_corners: np.ndarray,
    *,
    board_size: int = 15,
    target_row: int = 7,
    target_col: int = 7,
    target_name: str = "P77",
) -> np.ndarray:
    """Compatibility wrapper for callers that do not have a tracker snapshot."""

    corners = np.asarray(board_corners, dtype=np.float32).reshape(4, 2)
    normalized = np.array([[0, 0], [1, 0], [1, 1], [0, 1]], dtype=np.float32)
    homography = cv2.getPerspectiveTransform(normalized, corners)
    return draw_board_overlay(
        image,
        corners=corners,
        homography=homography,
        track_state=BoardTrackState.LOCKED,
        board_size=board_size,
        target_row=target_row,
        target_col=target_col,
        target_name=target_name,
        show_corners=False,
    )


def draw_runtime_status(
    image: np.ndarray,
    *,
    fps: float,
    board_status: str,
    reason: str,
    dry_run: bool,
) -> np.ndarray:
    output = image.copy()
    height, _width = output.shape[:2]
    locked = board_status.startswith("BOARD LOCKED")
    frozen = "FROZEN" in board_status
    board_color = (0, 190, 255) if frozen else ((40, 220, 40) if locked else (30, 30, 235))
    # OpenCV Hershey fonts are ASCII-only; the Qt status panel keeps the exact
    # middle-dot form while the framebuffer uses a visible slash.
    framebuffer_status = board_status.replace(" · ", " / ")
    _outlined_text(output, framebuffer_status, (20, height - 54), board_color, 0.8)
    _outlined_text(output, f"FPS {fps:.1f}", (20, height - 20), (255, 255, 255), 0.7)
    if reason:
        _outlined_text(output, reason[:80], (210, height - 20), board_color, 0.55)
    if dry_run:
        _outlined_text(output, "DRY RUN", (20, 136), (0, 210, 255), 0.85)
    return output


def make_test_pattern(width: int, height: int, frame_number: int) -> np.ndarray:
    image = np.full((height, width, 3), (32, 36, 42), dtype=np.uint8)
    spacing = max(32, min(width, height) // 12)
    offset = frame_number % spacing
    for x in range(-spacing + offset, width, spacing):
        cv2.line(image, (x, 0), (x, height), (50, 58, 68), 1)
    for y in range(-spacing + offset, height, spacing):
        cv2.line(image, (0, y), (width, y), (50, 58, 68), 1)
    cv2.putText(
        image,
        "DRY-RUN TEST PATTERN - NO APRILTAGS",
        (max(20, width // 10), height // 2),
        cv2.FONT_HERSHEY_SIMPLEX,
        max(0.65, width / 1600.0),
        (80, 210, 255),
        2,
        cv2.LINE_AA,
    )
    return image


def _draw_board_corners_inplace(
    frame: np.ndarray,
    corners: np.ndarray,
    *,
    show_coordinates: bool,
) -> None:
    for label, value in zip(CORNER_LABELS, corners):
        point = tuple(np.rint(value).astype(int))
        cv2.circle(frame, point, 11, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.circle(frame, point, 7, (0, 0, 255), -1, cv2.LINE_AA)
        text = label
        if show_coordinates:
            text += f" ({point[0]}, {point[1]})"
        cv2.putText(
            frame,
            text,
            _text_origin(frame, point, 12, -12),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (255, 255, 255),
            3,
            cv2.LINE_AA,
        )
        cv2.putText(
            frame,
            text,
            _text_origin(frame, point, 12, -12),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (0, 0, 255),
            1,
            cv2.LINE_AA,
        )


def _text_origin(
    image: np.ndarray,
    point: tuple[int, int],
    dx: int,
    dy: int,
) -> tuple[int, int]:
    height, width = image.shape[:2]
    x = max(2, min(width - 140, point[0] + dx))
    y_candidate = point[1] + dy
    y = point[1] + 24 if y_candidate < 18 else y_candidate
    return x, max(18, min(height - 4, y))


def _outlined_text(
    image: np.ndarray,
    text: str,
    origin: tuple[int, int],
    color: tuple[int, int, int],
    scale: float,
) -> None:
    cv2.putText(image, text, origin, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(image, text, origin, cv2.FONT_HERSHEY_SIMPLEX, scale, color, 2, cv2.LINE_AA)


def draw_piece_matrix(
    display_frame: np.ndarray,
    *,
    homography: np.ndarray,
    board_matrix: object,
    board_size: int = 15,
    diagnostics: object | None = None,
) -> np.ndarray:
    """Overlay recognized stones and optional candidate diagnostics."""
    output = display_frame
    grid = grid_points_from_homography(np.asarray(homography, dtype=np.float32), board_size)

    # Diagnostic: draw all candidates first.
    if diagnostics:
        for diag in diagnostics:
            center = (int(round(diag.center_x)), int(round(diag.center_y)))
            radius = max(3, int(round(diag.radius)))
            if diag.accepted:
                color = (0, 255, 0)
            elif diag.classified_color in {"Black", "White"}:
                color = (0, 165, 255)  # rejected classified
            else:
                color = (128, 128, 128)  # unclassified
            cv2.circle(output, center, radius, color, 1, cv2.LINE_AA)
            if diag.nearest_row is not None and diag.nearest_col is not None:
                nearest = tuple(np.rint(grid[diag.nearest_row, diag.nearest_col]).astype(int))
                cv2.line(output, center, nearest, color, 1, cv2.LINE_AA)
            label = f"{diag.candidate_id}:{diag.reject_reason[:10]}"
            cv2.putText(
                output,
                label,
                (center[0] + 4, center[1] - 4),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.35,
                color,
                1,
                cv2.LINE_AA,
            )

    for row_index, row in enumerate(board_matrix):
        for col_index, value in enumerate(row):
            if int(value) == 0:
                continue
            point = tuple(np.rint(grid[row_index, col_index]).astype(int))
            if int(value) == 1:  # black
                cv2.circle(output, point, 10, (40, 40, 40), -1, cv2.LINE_AA)
                cv2.circle(output, point, 10, (0, 255, 0), 2, cv2.LINE_AA)
            elif int(value) == 2:  # white
                cv2.circle(output, point, 10, (240, 240, 240), -1, cv2.LINE_AA)
                cv2.circle(output, point, 10, (255, 128, 0), 2, cv2.LINE_AA)
    return output

