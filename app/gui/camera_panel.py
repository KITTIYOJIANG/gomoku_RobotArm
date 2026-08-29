from __future__ import annotations

import numpy as np
from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QImage, QMouseEvent, QPixmap, QResizeEvent
from PySide6.QtWidgets import QFrame, QHBoxLayout, QLabel, QVBoxLayout, QWidget


class CameraPanel(QWidget):
    image_clicked = Signal(float, float)  # image-pixel coordinates

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._source_pixmap: QPixmap | None = None
        self._image_width = 0
        self._image_height = 0

        title = QLabel("实时视觉 / 标定目标")
        title.setStyleSheet("font-size: 17px; font-weight: 600;")
        self.camera_state = QLabel("Camera: DISCONNECTED")
        self.fps_label = QLabel("FPS: 0.0")
        self.board_state = QLabel("BOARD LOST")
        self.board_state.setStyleSheet("color: #d94c4c; font-weight: 700;")
        self.target_label = QLabel("当前目标坐标：-")

        status = QHBoxLayout()
        status.addWidget(self.camera_state)
        status.addWidget(self.fps_label)
        status.addStretch(1)
        status.addWidget(self.board_state)
        status.addWidget(self.target_label)

        self.image_label = QLabel("摄像头未连接\n程序启动不会自动打开摄像头\n棋盘锁定后点击交点仅选择目标，不会运动")
        self.image_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.image_label.setMinimumSize(640, 480)
        self.image_label.setFrameShape(QFrame.Shape.StyledPanel)
        self.image_label.setStyleSheet("background: #16191f; color: #aab0bc; border: 1px solid #343a46;")
        self.image_label.setMouseTracking(True)
        self.image_label.mousePressEvent = self._on_image_mouse_press  # type: ignore[method-assign]

        layout = QVBoxLayout(self)
        layout.addWidget(title)
        layout.addLayout(status)
        layout.addWidget(self.image_label, 1)

    def set_frame(self, frame: np.ndarray, fps: float) -> None:
        if frame is None or frame.size == 0:
            return
        contiguous = np.ascontiguousarray(frame)
        height, width = contiguous.shape[:2]
        self._image_width = int(width)
        self._image_height = int(height)
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
            self._image_width = 0
            self._image_height = 0
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

    def set_target_text(self, text: str) -> None:
        self.target_label.setText(text)

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

    def _on_image_mouse_press(self, event: QMouseEvent) -> None:
        if event.button() != Qt.MouseButton.LeftButton:
            return
        mapped = self.map_widget_to_image(event.position().x(), event.position().y())
        if mapped is None:
            return
        self.image_clicked.emit(float(mapped[0]), float(mapped[1]))

    def map_widget_to_image(self, widget_x: float, widget_y: float) -> tuple[float, float] | None:
        if self._source_pixmap is None or self._image_width <= 0 or self._image_height <= 0:
            return None
        label_w = max(1, self.image_label.width())
        label_h = max(1, self.image_label.height())
        scale = min(label_w / self._image_width, label_h / self._image_height)
        drawn_w = self._image_width * scale
        drawn_h = self._image_height * scale
        offset_x = (label_w - drawn_w) / 2.0
        offset_y = (label_h - drawn_h) / 2.0
        x = (widget_x - offset_x) / scale
        y = (widget_y - offset_y) / scale
        if x < 0 or y < 0 or x >= self._image_width or y >= self._image_height:
            return None
        return float(x), float(y)
