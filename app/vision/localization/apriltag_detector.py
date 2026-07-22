from __future__ import annotations

import time
from collections import Counter

import cv2
import numpy as np

from .layout import AprilTagBoardLayout
from .models import DetectionBatch, RejectedTag, TagDetection

try:
    from pupil_apriltags import Detector as PupilDetector
except ImportError:  # pragma: no cover - exercised by deployment diagnostics
    PupilDetector = None


class AprilTagDetector:
    """Thin, testable wrapper around the AprilTag3 detector."""

    def __init__(
        self,
        layout: AprilTagBoardLayout,
        nthreads: int = 2,
        quad_decimate: float = 1.0,
    ) -> None:
        if PupilDetector is None:
            raise RuntimeError(
                "APRILTAG_BACKEND_UNAVAILABLE: install pupil-apriltags from requirements.txt"
            )
        self.layout = layout
        self._detector = PupilDetector(
            families=layout.tag_family,
            nthreads=max(1, int(nthreads)),
            quad_decimate=float(quad_decimate),
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0,
        )

    def detect(self, image: np.ndarray, timestamp: float | None = None) -> DetectionBatch:
        gray = _to_gray_u8(image)
        detected_at = time.time() if timestamp is None else float(timestamp)
        raw = self._detector.detect(gray, estimate_tag_pose=False)
        batch = DetectionBatch()

        quality_candidates: list[TagDetection] = []
        for item in raw:
            tag_id = int(item.tag_id)
            margin = float(item.decision_margin)
            hamming = int(item.hamming)
            reason = None
            if tag_id not in self.layout.allowed_tag_ids:
                reason = "UNKNOWN_TAG_ID"
            elif hamming > self.layout.maximum_hamming:
                reason = "HAMMING_EXCEEDED"
            elif margin < self.layout.minimum_decision_margin:
                reason = "DECISION_MARGIN_TOO_LOW"

            if reason:
                batch.rejected.append(RejectedTag(tag_id, reason, margin, hamming))
                continue

            quality_candidates.append(
                TagDetection(
                    tag_id=tag_id,
                    center=np.asarray(item.center, dtype=np.float64).reshape(2),
                    corners=np.asarray(item.corners, dtype=np.float64).reshape(4, 2),
                    decision_margin=margin,
                    hamming=hamming,
                    timestamp=detected_at,
                )
            )

        counts = Counter(item.tag_id for item in quality_candidates)
        batch.duplicate_ids = sorted(tag_id for tag_id, count in counts.items() if count > 1)
        for item in quality_candidates:
            if item.tag_id in batch.duplicate_ids:
                batch.rejected.append(
                    RejectedTag(
                        item.tag_id,
                        "DUPLICATE_TAG_ID",
                        item.decision_margin,
                        item.hamming,
                    )
                )
            else:
                batch.accepted.append(item)

        batch.accepted.sort(key=lambda item: item.tag_id)
        return batch


def _to_gray_u8(image: np.ndarray) -> np.ndarray:
    if image is None or image.size == 0:
        raise ValueError("Input image is empty")
    if image.ndim == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    elif image.ndim == 2:
        gray = image
    else:
        raise ValueError(f"Unsupported image shape: {image.shape}")
    if gray.dtype != np.uint8:
        gray = np.clip(gray, 0, 255).astype(np.uint8)
    return np.ascontiguousarray(gray)
