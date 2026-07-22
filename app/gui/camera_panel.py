from __future__ import annotations

import numpy as np
from PySide6.QtCore import Qt
from PySide6.QtGui import QImage, QPixmap, QResizeEvent
from PySide6.QtWidgets import QFrame, QHBoxLayout, QLabel, QVBoxLayout, QWidget


class CameraPanel(QWidget):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._source_pixmap: QPixmap | None = None

        title = QLabel("实时视觉 / P77 固定目标")
        title.setStyleSheet("font-size: 17px; font-weight: 600;")
        self.camera_state = QLabel("Camera: DISCONNECTED")
        self.fps_label = QLabel("FPS: 0.0")
        self.board_state = QLabel("BOARD LOST")
        self.board_state.setStyleSheet("color: #d94c4c; font-weight: 700;")
        self.target_label = QLabel("当前目标坐标：(7,7)")

        status = QHBoxLayout()
        status.addWidget(self.camera_state)
        status.addWidget(self.fps_label)
        status.addStretch(1)
        status.addWidget(self.board_state)
        status.addWidget(self.target_label)

        self.image_label = QLabel("摄像头未连接\n程序启动不会自动打开摄像头")
        self.image_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.image_label.setMinimumSize(640, 480)
        self.image_label.setFrameShape(QFrame.Shape.StyledPanel)
        self.image_label.setStyleSheet("background: #16191f; color: #aab0bc; border: 1px solid #343a46;")

        layout = QVBoxLayout(self)
        layout.addWidget(title)
        layout.addLayout(status)
        layout.addWidget(self.image_label, 1)

    def set_frame(self, frame: np.ndarray, fps: float) -> None:
        if frame is None or frame.size == 0:
            return
        contiguous = np.ascontiguousarray(frame)
        height, width = contiguous.shape[:2]
        if contiguous.ndim == 2:
            image = QImage(
                contiguous.data,
                width,
                height,
                contiguous.strides[0],
                QImage.Format.Format_Grayscale8,
            ).copy()
        else:
            image = QImage(
                contiguous.data,
                width,
                height,
                contiguous.strides[0],
                QImage.Format.Format_BGR888,
            ).copy()
        self._source_pixmap = QPixmap.fromImage(image)
        self.fps_label.setText(f"FPS: {fps:.1f}")
        self._refresh_pixmap()

    def set_camera_status(self, status: str) -> None:
        self.camera_state.setText(f"Camera: {status}")
        if status == "DISCONNECTED":
            self._source_pixmap = None
            self.image_label.setPixmap(QPixmap())
            self.image_label.setText("摄像头已断开")

    def set_board_status(self, status: str, reason: str) -> None:
        self.board_state.setText(status)
        if "FROZEN" in status:
            color = "#e5a934"
        else:
            color = "#46c06f" if status.startswith("BOARD LOCKED") else "#d94c4c"
        self.board_state.setStyleSheet(f"color: {color}; font-weight: 700;")
        self.board_state.setToolTip(reason)

    def resizeEvent(self, event: QResizeEvent) -> None:  # noqa: N802 - Qt API
        super().resizeEvent(event)
        self._refresh_pixmap()

    def _refresh_pixmap(self) -> None:
        if self._source_pixmap is None:
            return
        size = self.image_label.size()
        self.image_label.setPixmap(
            self._source_pixmap.scaled(
                size,
                Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation,
            )
        )
