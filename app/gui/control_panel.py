from __future__ import annotations

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QScrollArea,
    QVBoxLayout,
    QWidget,
)

from app.arm.controller import available_serial_ports
from app.arm.state import ArmState

from .log_panel import LogPanel
from .stage5_panel import Stage5Panel
from .cross_anchor_panel import CrossAnchorPanel
from .hover_learning_panel import HoverLearningPanel
from .status_panel import StatusPanel


MANUAL_ACTIONS = (
    "HOME_IDLE",
    "OBSERVE_IDLE",
    "OBSERVE_HOLD",
    "SOURCE_TOUCH_IDLE",
    "SOURCE_TOUCH_HOLD",
    "CARRY_HIGH_P77_IDLE",
    "CARRY_HIGH_P77_HOLD",
    "P77_ABOVE_IDLE",
    "P77_ABOVE_HOLD",
    "P77_TOUCH_HOLD",
    "P77_TOUCH_RELEASE",
)

FUTURE_ACTIONS = (
    "选择其他棋盘坐标",
    "自动落子",
    "五子棋 AI",
    "相机机械臂标定",
    "保存新固定点",
    "检查是否吸取成功",
    "自动重试吸取",
    "蜂鸣器测试",
    "多目标动作库",
)


class ControlPanel(QWidget):
    connect_camera_requested = Signal()
    disconnect_camera_requested = Signal()
    connect_serial_requested = Signal(str)
    disconnect_serial_requested = Signal()
    return_observe_requested = Signal()
    pick_requested = Signal()
    place_requested = Signal()
    full_cycle_requested = Signal()
    manual_action_requested = Signal(str)
    estop_requested = Signal()
    pump_off_requested = Signal()
    beep_test_requested = Signal()
    corner_overlay_options_changed = Signal(bool, bool)
    piece_recognition_requested = Signal()

    def __init__(
        self,
        *,
        default_port: str = "COM6",
        dry_run: bool = False,
        default_test_pattern: bool = False,
        parent=None,
    ) -> None:
        super().__init__(parent)
        self._dry_run = bool(dry_run)
        self.status_panel = StatusPanel()
        self.log_panel = LogPanel()
        self.stage5_panel = Stage5Panel(default_dry_run=True)
        self.cross_anchor_panel = CrossAnchorPanel()
        self.hover_learning_panel = HoverLearningPanel()

        content = QWidget()
        content_layout = QVBoxLayout(content)
        content_layout.addWidget(self.status_panel)
        content_layout.addWidget(self._build_connection_group(default_port, default_test_pattern))
        content_layout.addWidget(self._build_vision_debug_group())
        content_layout.addWidget(self._build_core_group())
        content_layout.addWidget(self.stage5_panel)
        self._adv_cross = QGroupBox('高级：中心十字标定（默认收起）')
        self._adv_cross.setCheckable(True)
        self._adv_cross.setChecked(False)
        lay1 = QVBoxLayout(self._adv_cross)
        lay1.addWidget(self.cross_anchor_panel)
        self.cross_anchor_panel.setVisible(False)
        self._adv_cross.toggled.connect(self.cross_anchor_panel.setVisible)
        content_layout.addWidget(self._adv_cross)
        self._adv_learn = QGroupBox('高级：学习层影子预测（默认收起）')
        self._adv_learn.setCheckable(True)
        self._adv_learn.setChecked(False)
        lay2 = QVBoxLayout(self._adv_learn)
        lay2.addWidget(self.hover_learning_panel)
        self.hover_learning_panel.setVisible(False)
        self._adv_learn.toggled.connect(self.hover_learning_panel.setVisible)
        content_layout.addWidget(self._adv_learn)
        content_layout.addWidget(self._build_manual_group())
        content_layout.addWidget(self._build_future_group())
        content_layout.addWidget(self._build_safety_group())
        content_layout.addWidget(self.log_panel, 1)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(content)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(scroll)
        self.set_camera_connected(False)
        self.update_controls(
            connected=False,
            state=ArmState.DISCONNECTED,
            busy=False,
            board_locked=False,
            target_visible=False,
        )

    def _build_connection_group(self, default_port: str, default_test_pattern: bool) -> QGroupBox:
        group = QGroupBox("连接")
        layout = QGridLayout(group)
        self.connect_camera_button = QPushButton("连接摄像头")
        self.disconnect_camera_button = QPushButton("断开摄像头")
        self.test_pattern_checkbox = QCheckBox("dry-run 测试画面")
        self.test_pattern_checkbox.setChecked(default_test_pattern)
        self.test_pattern_checkbox.setVisible(self._dry_run)
        self.port_combo = QComboBox()
        ports = available_serial_ports(default_port)
        self.port_combo.addItems(ports)
        index = self.port_combo.findText(default_port)
        if index >= 0:
            self.port_combo.setCurrentIndex(index)
        self.connect_serial_button = QPushButton(f"连接 {default_port}")
        self.disconnect_serial_button = QPushButton("断开 COM")
        baud = QLabel("115200 baud（固定）")

        layout.addWidget(self.connect_camera_button, 0, 0)
        layout.addWidget(self.disconnect_camera_button, 0, 1)
        layout.addWidget(self.test_pattern_checkbox, 1, 0, 1, 2)
        layout.addWidget(self.port_combo, 2, 0)
        layout.addWidget(baud, 2, 1)
        layout.addWidget(self.connect_serial_button, 3, 0)
        layout.addWidget(self.disconnect_serial_button, 3, 1)

        self.connect_camera_button.clicked.connect(lambda _checked=False: self.connect_camera_requested.emit())
        self.disconnect_camera_button.clicked.connect(lambda _checked=False: self.disconnect_camera_requested.emit())
        self.connect_serial_button.clicked.connect(
            lambda: self.connect_serial_requested.emit(self.port_combo.currentText())
        )
        self.disconnect_serial_button.clicked.connect(lambda _checked=False: self.disconnect_serial_requested.emit())
        self.port_combo.currentTextChanged.connect(
            lambda value: self.connect_serial_button.setText(f"连接 {value}")
        )
        return group

    def _build_vision_debug_group(self) -> QGroupBox:
        group = QGroupBox("视觉调试")
        layout = QVBoxLayout(group)
        self.show_corners_checkbox = QCheckBox("显示棋盘四角红点")
        self.show_corners_checkbox.setChecked(True)
        self.show_corner_coordinates_checkbox = QCheckBox("显示角点像素坐标")
        self.show_corner_coordinates_checkbox.setChecked(False)
        self.recognize_pieces_button = QPushButton("重新识别棋子")
        layout.addWidget(self.show_corners_checkbox)
        layout.addWidget(self.show_corner_coordinates_checkbox)
        layout.addWidget(self.recognize_pieces_button)
        self.show_corners_checkbox.toggled.connect(self._emit_corner_options)
        self.show_corner_coordinates_checkbox.toggled.connect(self._emit_corner_options)
        self.recognize_pieces_button.clicked.connect(
            lambda _checked=False: self.piece_recognition_requested.emit()
        )
        return group

    def _build_core_group(self) -> QGroupBox:
        group = QGroupBox("固定点 P77")
        layout = QGridLayout(group)
        self.return_button = QPushButton("回观察位")
        self.pick_button = QPushButton("取料")
        self.place_button = QPushButton("下棋到 P77")
        self.full_cycle_button = QPushButton("完整固定点流程（实验功能）")
        self.pick_button.setMinimumHeight(42)
        self.place_button.setMinimumHeight(42)
        layout.addWidget(self.return_button, 0, 0, 1, 2)
        layout.addWidget(self.pick_button, 1, 0)
        layout.addWidget(self.place_button, 1, 1)
        layout.addWidget(self.full_cycle_button, 2, 0, 1, 2)
        self.return_button.clicked.connect(lambda _checked=False: self.return_observe_requested.emit())
        self.pick_button.clicked.connect(lambda _checked=False: self.pick_requested.emit())
        self.place_button.clicked.connect(lambda _checked=False: self.place_requested.emit())
        self.full_cycle_button.clicked.connect(lambda _checked=False: self.full_cycle_requested.emit())
        return group

    def _build_manual_group(self) -> QGroupBox:
        group = QGroupBox("手动调试")
        layout = QVBoxLayout(group)
        self.manual_toggle = QPushButton("展开单姿态动作")
        self.manual_toggle.setCheckable(True)
        self.manual_container = QWidget()
        grid = QGridLayout(self.manual_container)
        self.manual_buttons: dict[str, QPushButton] = {}
        for index, name in enumerate(MANUAL_ACTIONS):
            button = QPushButton(name)
            button.clicked.connect(lambda _checked=False, action=name: self.manual_action_requested.emit(action))
            grid.addWidget(button, index // 2, index % 2)
            self.manual_buttons[name] = button
        self.manual_container.setVisible(False)
        self.manual_toggle.toggled.connect(self._toggle_manual)
        layout.addWidget(self.manual_toggle)
        layout.addWidget(self.manual_container)
        return group

    def _build_future_group(self) -> QGroupBox:
        group = QGroupBox("扩展 / 诊断")
        layout = QGridLayout(group)
        self.beep_test_button = QPushButton("蜂鸣器测试")
        self.beep_test_button.clicked.connect(lambda _c=False: self.beep_test_requested.emit())
        layout.addWidget(self.beep_test_button, 0, 0, 1, 2)
        row = 1
        for index, name in enumerate(FUTURE_ACTIONS):
            if name == "蜂鸣器测试":
                continue
            button = QPushButton(f"{name}（后续版本）")
            button.setEnabled(False)
            layout.addWidget(button, row + index // 2, index % 2)
        return group

    def _build_safety_group(self) -> QGroupBox:
        group = QGroupBox("安全")
        layout = QHBoxLayout(group)
        self.estop_button = QPushButton("急停")
        self.estop_button.setMinimumHeight(54)
        self.estop_button.setStyleSheet(
            "QPushButton { background: #b3262e; color: white; font-size: 18px; font-weight: 700; }"
            "QPushButton:hover { background: #d3313b; }"
        )
        self.pump_off_button = QPushButton("气泵关闭")
        self.pump_off_button.setMinimumHeight(54)
        layout.addWidget(self.estop_button, 2)
        layout.addWidget(self.pump_off_button, 1)
        self.estop_button.clicked.connect(lambda _checked=False: self.estop_requested.emit())
        self.pump_off_button.clicked.connect(lambda _checked=False: self.pump_off_requested.emit())
        return group

    def _toggle_manual(self, expanded: bool) -> None:
        self.manual_container.setVisible(expanded)
        self.manual_toggle.setText("收起单姿态动作" if expanded else "展开单姿态动作")

    def _emit_corner_options(self, _checked: bool = False) -> None:
        self.corner_overlay_options_changed.emit(
            self.show_corners_checkbox.isChecked(),
            self.show_corner_coordinates_checkbox.isChecked(),
        )

    def corner_overlay_options(self) -> tuple[bool, bool]:
        return (
            self.show_corners_checkbox.isChecked(),
            self.show_corner_coordinates_checkbox.isChecked(),
        )

    def camera_uses_test_pattern(self) -> bool:
        return self._dry_run and self.test_pattern_checkbox.isChecked()

    def set_camera_connected(self, connected: bool) -> None:
        self.connect_camera_button.setEnabled(not connected)
        self.disconnect_camera_button.setEnabled(connected)
        self.test_pattern_checkbox.setEnabled(not connected)
        self.recognize_pieces_button.setEnabled(connected)

    def update_controls(
        self,
        *,
        connected: bool,
        state: ArmState,
        busy: bool,
        board_locked: bool,
        target_visible: bool,
    ) -> None:
        self.port_combo.setEnabled(not connected and not busy)
        self.connect_serial_button.setEnabled(not connected and not busy)
        self.disconnect_serial_button.setEnabled(connected)
        ordinary = connected and not busy
        # 回观察位：串口一连上就应可点（忙时点击会提示），不必等 OBSERVE/悬停状态。
        self.return_button.setEnabled(connected)
        self.pick_button.setEnabled(ordinary and state == ArmState.OBSERVE_IDLE)
        self.place_button.setEnabled(
            ordinary
            and state == ArmState.OBSERVE_HOLD
            and board_locked
            and target_visible
        )
        self.full_cycle_button.setEnabled(
            ordinary
            and state == ArmState.OBSERVE_IDLE
            and board_locked
            and target_visible
        )
        self.manual_toggle.setEnabled(ordinary)
        for button in self.manual_buttons.values():
            button.setEnabled(ordinary)
        self.estop_button.setEnabled(connected)
        self.pump_off_button.setEnabled(connected)
        if hasattr(self, "beep_test_button"):
            self.beep_test_button.setEnabled(connected)
