from __future__ import annotations

import cv2
import numpy as np

from .models import LocalizationResult, LocalizationUpdate


def draw_localization(image: np.ndarray, result: LocalizationResult) -> np.ndarray:
    output = image.copy()
    for detection in result.detections:
        corners = np.rint(detection.corners).astype(np.int32)
        center = tuple(np.rint(detection.center).astype(int))
        cv2.polylines(output, [corners.reshape(-1, 1, 2)], True, (0, 220, 0), 3)
        for index, point in enumerate(corners):
            cv2.circle(output, tuple(point), 5, (0, 0, 255), -1)
            cv2.putText(
                output,
                str(index),
                tuple(point + np.array([5, -5])),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (0, 0, 255),
                1,
                cv2.LINE_AA,
            )
        cv2.circle(output, center, 5, (255, 0, 255), -1)
        label = (
            f"ID {detection.tag_id} "
            f"M={detection.decision_margin:.1f} H={detection.hamming}"
        )
        cv2.putText(
            output,
            label,
            (center[0] + 10, center[1] - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )

    if result.board_corners_image is not None:
        board = np.rint(result.board_corners_image).astype(np.int32)
        cv2.polylines(output, [board.reshape(-1, 1, 2)], True, (255, 80, 0), 4)
        names = ("TL", "TR", "BR", "BL")
        for name, point in zip(names, board):
            cv2.circle(output, tuple(point), 7, (255, 80, 0), -1)
            cv2.putText(
                output,
                name,
                tuple(point + np.array([8, 22])),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (255, 80, 0),
                2,
                cv2.LINE_AA,
            )

    status = result.status.value
    if result.reprojection_error is not None:
        status += f" reproj={result.reprojection_error:.2f}px"
    status += f" confidence={result.localization_confidence:.2f}"
    _outlined_text(output, status, (20, 34), (0, 255, 0) if result.valid else (0, 0, 255))
    if result.error_code:
        _outlined_text(output, f"ERROR: {result.error_code}", (20, 68), (0, 0, 255))
    if result.warnings:
        _outlined_text(output, "WARN: " + ", ".join(result.warnings), (20, 102), (0, 180, 255))
    return output


def draw_localization_update(image: np.ndarray, update: LocalizationUpdate) -> np.ndarray:
    """Draw raw detections and the temporal/formal localization state."""

    output = draw_localization(image, update.raw_result)
    lines = [
        f"TAG={update.tag_status.value} stable={update.stable_frame_count}/{update.required_stable_frames}",
        (
            f"formal={'YES' if update.board_localized else 'NO'} "
            f"last_valid={'YES' if update.used_last_valid_localization else 'NO'}"
        ),
        (
            f"arm_busy={'YES' if update.arm_busy else 'NO'} "
            f"recognition={'ENABLED' if update.recognition_allowed else 'FROZEN'}"
        ),
        f"transition={update.transition_reason or '-'}",
        (
            "last_success=-"
            if update.last_success_timestamp is None
            else f"last_success={update.last_success_timestamp:.3f}"
        ),
    ]
    y = 136
    for line in lines:
        _outlined_text(output, line, (20, y), (255, 255, 0))
        y += 34
    return output


def _outlined_text(
    image: np.ndarray,
    text: str,
    origin: tuple[int, int],
    color: tuple[int, int, int],
) -> None:
    cv2.putText(image, text, origin, cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(image, text, origin, cv2.FONT_HERSHEY_SIMPLEX, 0.75, color, 2, cv2.LINE_AA)
