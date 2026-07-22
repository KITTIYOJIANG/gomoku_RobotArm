from __future__ import annotations

import cv2
import numpy as np

from .geometry import is_convex_quad, polygon_signed_area, reprojection_rmse, transform_points
from .layout import AprilTagBoardLayout
from .models import DetectionBatch, LocalizationResult, LocalizationStatus, TagDetection


class BoardLocalizer:
    def __init__(self, layout: AprilTagBoardLayout) -> None:
        self.layout = layout

    def localize(self, image: np.ndarray, batch: DetectionBatch) -> LocalizationResult:
        warnings: list[str] = []
        if (
            self.layout.tag_size_mm is None
            or self.layout.board_width_mm is None
            or self.layout.board_height_mm is None
        ):
            warnings.append("PHYSICAL_LAYOUT_UNCALIBRATED")

        detections = batch.accepted
        tag_count = len(detections)
        if batch.duplicate_ids:
            return self._rejected(
                detections,
                batch,
                "DUPLICATE_TAG_ID",
                warnings,
            )
        if tag_count == 0:
            return LocalizationResult(
                status=LocalizationStatus.TAG_LOST,
                detections=[],
                error_code="TAG_LOST",
                warnings=warnings,
                rejected_tags=batch.rejected,
            )
        if tag_count < self.layout.minimum_tag_count:
            return LocalizationResult(
                status=LocalizationStatus.TAG_INSUFFICIENT,
                detections=detections,
                error_code="TAG_INSUFFICIENT",
                warnings=warnings,
                rejected_tags=batch.rejected,
            )

        object_points, image_points = self._collect_correspondences(detections)
        h_board_to_image, inlier_mask = cv2.findHomography(
            object_points,
            image_points,
            cv2.RANSAC,
            self.layout.maximum_reprojection_error,
        )
        if h_board_to_image is None or not np.all(np.isfinite(h_board_to_image)):
            return self._rejected(detections, batch, "HOMOGRAPHY_INVALID", warnings)

        determinant = float(np.linalg.det(h_board_to_image))
        if abs(determinant) < 1e-12:
            return self._rejected(detections, batch, "HOMOGRAPHY_NOT_INVERTIBLE", warnings)

        h_image_to_board = np.linalg.inv(h_board_to_image)
        error = reprojection_rmse(object_points, image_points, h_board_to_image)
        if error > self.layout.maximum_reprojection_error:
            return self._rejected(
                detections,
                batch,
                "REPROJECTION_ERROR_EXCEEDED",
                warnings,
                reprojection_error=error,
            )

        board_corners = transform_points(
            self.layout.playable_corners_board,
            h_board_to_image,
        )
        geometry_error = self._validate_board_geometry(board_corners, image.shape)
        if geometry_error:
            return self._rejected(
                detections,
                batch,
                geometry_error,
                warnings,
                reprojection_error=error,
            )

        margins = [item.decision_margin for item in detections]
        margin_confidence = min(1.0, float(np.mean(margins)) / 100.0)
        error_confidence = max(
            0.0,
            1.0 - error / max(self.layout.maximum_reprojection_error, 1e-6),
        )
        tag_confidence = min(1.0, tag_count / 4.0)
        confidence = float(0.45 * margin_confidence + 0.35 * error_confidence + 0.20 * tag_confidence)

        return LocalizationResult(
            status=LocalizationStatus.BOARD_LOCALIZED,
            detections=detections,
            board_corners_image=board_corners,
            h_image_to_board=h_image_to_board,
            h_board_to_image=h_board_to_image,
            reprojection_error=error,
            localization_confidence=confidence,
            warnings=warnings,
            rejected_tags=batch.rejected,
        )

    def warp(self, image: np.ndarray, result: LocalizationResult) -> np.ndarray:
        if not result.valid or result.board_corners_image is None:
            raise ValueError("Cannot warp an image without a valid localization")
        width = self.layout.warp_width_px
        height = self.layout.warp_height_px
        destination = np.array(
            [[0, 0], [width - 1, 0], [width - 1, height - 1], [0, height - 1]],
            dtype=np.float32,
        )
        h_image_to_canvas = cv2.getPerspectiveTransform(
            result.board_corners_image.astype(np.float32),
            destination,
        )
        return cv2.warpPerspective(image, h_image_to_canvas, (width, height))

    def _collect_correspondences(
        self,
        detections: list[TagDetection],
    ) -> tuple[np.ndarray, np.ndarray]:
        object_points = []
        image_points = []
        for item in detections:
            object_points.extend(self.layout.tag_object_corners(item.tag_id))
            image_points.extend(item.corners)
        return (
            np.asarray(object_points, dtype=np.float64),
            np.asarray(image_points, dtype=np.float64),
        )

    def _validate_board_geometry(
        self,
        board_corners: np.ndarray,
        image_shape: tuple[int, ...],
    ) -> str | None:
        if board_corners.shape != (4, 2) or not np.all(np.isfinite(board_corners)):
            return "BOARD_CORNERS_INVALID"
        if not is_convex_quad(board_corners):
            return "BOARD_NOT_CONVEX"
        signed_area = polygon_signed_area(board_corners)
        if signed_area <= 0:
            return "BOARD_MIRRORED"

        image_height, image_width = image_shape[:2]
        area_ratio = signed_area / float(image_height * image_width)
        if not (
            self.layout.minimum_board_area_ratio
            <= area_ratio
            <= self.layout.maximum_board_area_ratio
        ):
            return "BOARD_AREA_INVALID"

        tl, tr, br, bl = board_corners
        top = np.linalg.norm(tr - tl)
        bottom = np.linalg.norm(br - bl)
        left = np.linalg.norm(bl - tl)
        right = np.linalg.norm(br - tr)
        width = (top + bottom) / 2.0
        height = (left + right) / 2.0
        if min(width, height) <= 1.0:
            return "BOARD_EDGE_TOO_SHORT"
        aspect = width / height
        if not 0.55 <= aspect <= 1.8:
            return "BOARD_ASPECT_INVALID"
        return None

    @staticmethod
    def _rejected(
        detections: list[TagDetection],
        batch: DetectionBatch,
        error_code: str,
        warnings: list[str],
        reprojection_error: float | None = None,
    ) -> LocalizationResult:
        return LocalizationResult(
            status=LocalizationStatus.LOCALIZATION_REJECTED,
            detections=detections,
            reprojection_error=reprojection_error,
            error_code=error_code,
            warnings=warnings,
            rejected_tags=batch.rejected,
        )
