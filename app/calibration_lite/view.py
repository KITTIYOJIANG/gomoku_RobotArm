from __future__ import annotations

from typing import Mapping

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QComboBox,
    QCheckBox,
    QFormLayout,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QSpinBox,
    QStackedWidget,
    QVBoxLayout,
    QWidget,
)

from app.arm.controller import available_serial_ports
from app.gui.camera_panel import CameraPanel
from .drop_v1_view import LiteDropV1Panel
from .manual_movel_view import P77ManualMoveLPanel
from .point_movel_view import PointMoveLPanel


class StatusRow(QWidget):
    def __init__(self, name: str, parent=None) -> None:
        super().__init__(parent)
        self.setFixedHeight(30)
        self.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 3, 0, 3)
        self.name_label = QLabel(name)
        self.value_label = QLabel("未初始化")
        self.value_label.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        self.value_label.setProperty("tone", "neutral")
        layout.addWidget(self.name_label)
        layout.addStretch(1)
        layout.addWidget(self.value_label)

    def set_status(self, text: str, tone: str = "neutral") -> None:
        self.value_label.setText(text)
        self.value_label.setProperty("tone", tone)
        self.value_label.style().unpolish(self.value_label)
        self.value_label.style().polish(self.value_label)


class PwmRow(QWidget):
    apply_requested = Signal(int, int)
    target_changed = Signal(int, int, int)

    def __init__(self, joint: int, parent=None) -> None:
        super().__init__(parent)
        self.joint = int(joint)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 2, 0, 2)
        self.joint_label = QLabel(f"J{joint}")
        self.joint_label.setFixedWidth(34)
        self.current_label = QLabel("当前 1500")
        self.current_label.setFixedWidth(88)
        self.target = QSpinBox()
        self.target.setRange(550, 2450)
        self.target.setValue(1500)
        self.target.setKeyboardTracking(False)
        self.target.setFixedWidth(92)
        self.target.lineEdit().returnPressed.connect(self._apply)
        layout.addWidget(self.joint_label)
        layout.addWidget(self.current_label)
        for delta in (-50, -10):
            button = QPushButton(str(delta))
            button.setProperty("compact", True)
            button.clicked.connect(lambda _c=False, value=delta: self.adjust(value))
            layout.addWidget(button)
        layout.addWidget(self.target)
        for delta in (10, 50):
            button = QPushButton(f"+{delta}")
            button.setProperty("compact", True)
            button.clicked.connect(lambda _c=False, value=delta: self.adjust(value))
            layout.addWidget(button)
        self.move_button = QPushButton("移动")
        self.move_button.setProperty("compact", True)
        self.move_button.clicked.connect(self._apply)
        layout.addWidget(self.move_button)

    def adjust(self, delta: int) -> None:
        previous = self.target.value()
        self.target.setValue(previous + int(delta))
        self.target_changed.emit(self.joint, previous, self.target.value())

    def _apply(self) -> None:
        self.apply_requested.emit(self.joint, self.target.value())

    def set_values(self, current: int, target: int | None = None) -> None:
        self.current_label.setText(f"当前 {int(current)}")
        self.target.setValue(int(current if target is None else target))

    def set_current(self, value: int) -> None:
        self.current_label.setText(f"当前 {int(value)}")


class CollapsibleGroup(QGroupBox):
    """A checkable group whose body is truly hidden while collapsed."""

    def __init__(
        self, title: str, parent=None, *, collapse_axis: str = "vertical"
    ) -> None:
        super().__init__(parent)
        self._base_title = title
        self._collapse_axis = collapse_axis
        self.setCheckable(True)
        shell = QVBoxLayout(self)
        shell.setContentsMargins(10, 8, 10, 8)
        self.body = QWidget(self)
        self.body_layout = QVBoxLayout(self.body)
        self.body_layout.setContentsMargins(0, 0, 0, 0)
        shell.addWidget(self.body)
        self.toggled.connect(self._set_expanded)
        self.setChecked(False)
        self._set_expanded(False)

    def _set_expanded(self, expanded: bool) -> None:
        self.body.setVisible(bool(expanded))
        if self._collapse_axis == "horizontal":
            self.setMaximumWidth(16777215 if expanded else 58)
            self.setMaximumHeight(16777215)
        else:
            self.setMaximumHeight(16777215 if expanded else 42)
            self.setMaximumWidth(16777215)
        self.setTitle(f"{self._base_title}  {'<' if expanded else '>'}")


class PumpToggle(QCheckBox):
    """Compact ON/OFF switch with an explicit pump state label."""

    def __init__(self, parent=None) -> None:
        super().__init__("泵嘴 OFF", parent)
        self.setObjectName("pumpToggle")
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self.toggled.connect(self._update_label)

    def _update_label(self, enabled: bool) -> None:
        self.setText("泵嘴 ON" if enabled else "泵嘴 OFF")

    def set_pump_on(self, enabled: bool) -> None:
        blocked = self.blockSignals(True)
        self.setChecked(bool(enabled))
        self._update_label(bool(enabled))
        self.blockSignals(blocked)


class CalibrationLiteView(QWidget):
    connect_serial_requested = Signal(str)
    disconnect_serial_requested = Signal()
    connect_camera_requested = Signal()
    disconnect_camera_requested = Signal()
    relocalize_requested = Signal()
    quick_calibration_requested = Signal()
    continue_requested = Signal()
    return_observe_requested = Signal()
    pick_requested = Signal()
    place_requested = Signal()
    manual_movel_requested = Signal()
    point_movel_requested = Signal()
    p77_full_cycle_requested = Signal()
    estop_requested = Signal()
    pump_toggle_requested = Signal(bool)
    pump_off_requested = Signal()
    recover_requested = Signal()
    move_above_requested = Signal()
    confirm_anchor_requested = Signal()
    generate_requested = Signal()
    begin_test_requested = Signal()
    test_accurate_requested = Signal()
    test_inaccurate_requested = Signal()
    commit_requested = Signal()
    add_anchor_requested = Signal(int, int)
    camera_anchor_pick_requested = Signal()
    observe_calibration_requested = Signal()
    observe_move_requested = Signal()
    observe_save_requested = Signal()
    observe_pwm_apply_requested = Signal(int, int)
    observe_pwm_target_changed = Signal(int, int, int)
    observe_pwm_undo_requested = Signal()
    pick_calibration_requested = Signal()
    pick_calibration_move_requested = Signal()
    pick_calibration_save_requested = Signal()
    pick_calibration_pwm_apply_requested = Signal(int, int)
    pick_calibration_pwm_target_changed = Signal(int, int, int)
    pick_calibration_pwm_undo_requested = Signal()
    place_calibration_requested = Signal()
    place_calibration_move_requested = Signal()
    place_calibration_accurate_requested = Signal()
    place_calibration_inaccurate_requested = Signal()
    place_calibration_pwm_apply_requested = Signal(int, int)
    place_calibration_pwm_target_changed = Signal(int, int, int)
    place_calibration_pwm_undo_requested = Signal()
    review_target_requested = Signal(str)
    back_home_requested = Signal()
    open_legacy_requested = Signal()
    pwm_apply_requested = Signal(int, int)
    pwm_target_changed = Signal(int, int, int)
    pwm_undo_requested = Signal()

    HOME = 0
    ANCHOR = 1
    GENERATE = 2
    TEST = 3
    COMPLETE = 4
    OBSERVE = 5
    PICK_CALIBRATION = 6
    PLACE_CALIBRATION = 7
    DROP_V1 = 8
    MANUAL_MOVEL = 9
    POINT_MOVEL = 10

    def __init__(self, *, default_port: str = "COM6", parent=None) -> None:
        super().__init__(parent)
        self._pwm_rows: dict[int, PwmRow] = {}
        self._observe_pwm_rows: dict[int, PwmRow] = {}
        self._pick_pwm_rows: dict[int, PwmRow] = {}
        self._place_pwm_rows: dict[int, PwmRow] = {}
        self.drop_v1_panel = LiteDropV1Panel()
        self.manual_movel_panel = P77ManualMoveLPanel()
        self.point_movel_panel = PointMoveLPanel()
        self.setObjectName("calibrationLite")
        root = QVBoxLayout(self)
        root.setContentsMargins(26, 20, 26, 20)
        root.setSpacing(14)

        header = QHBoxLayout()
        title_box = QVBoxLayout()
        title = QLabel("Gomoku Robot · Calibration Lite")
        title.setObjectName("appTitle")
        subtitle = QLabel("快速部署标定")
        subtitle.setObjectName("subtitle")
        title_box.addWidget(title)
        title_box.addWidget(subtitle)
        header.addLayout(title_box)
        header.addStretch(1)
        self.global_state_label = QLabel("未连接")
        self.global_state_label.setObjectName("globalState")
        header.addWidget(self.global_state_label)
        root.addLayout(header)

        connection = QHBoxLayout()
        self.port_combo = QComboBox()
        self.port_combo.addItems(available_serial_ports(default_port))
        index = self.port_combo.findText(default_port)
        if index >= 0:
            self.port_combo.setCurrentIndex(index)
        self.serial_button = QPushButton("连接 STM32")
        self.camera_button = QPushButton("连接 Camera")
        self.serial_button.clicked.connect(self._toggle_serial)
        self.camera_button.clicked.connect(self._toggle_camera)
        connection.addWidget(self.port_combo, 1)
        connection.addWidget(self.serial_button, 1)
        connection.addWidget(self.camera_button, 1)
        root.addLayout(connection)

        self.camera_preview_group = CollapsibleGroup(
            "Camera", collapse_axis="horizontal"
        )
        camera_tools = QHBoxLayout()
        camera_hint = QLabel("相机移动并稳定后，点击重新定位棋盘")
        camera_hint.setObjectName("muted")
        self.relocalize_button = QPushButton("快速重新定位")
        self.relocalize_button.setEnabled(False)
        self.relocalize_button.clicked.connect(self.relocalize_requested.emit)
        camera_tools.addWidget(camera_hint, 1)
        camera_tools.addWidget(self.relocalize_button)
        self.camera_preview_group.body_layout.addLayout(camera_tools)
        self.camera_preview = CameraPanel()
        self.camera_preview.image_label.setMinimumSize(320, 220)
        self.camera_preview_group.body_layout.addWidget(self.camera_preview)

        self.pages = QStackedWidget()
        self.pages.addWidget(self._scroll_page(self._build_home()))
        self.pages.addWidget(self._scroll_page(self._build_anchor()))
        self.pages.addWidget(self._scroll_page(self._build_generate()))
        self.pages.addWidget(self._scroll_page(self._build_test()))
        self.pages.addWidget(self._scroll_page(self._build_complete()))
        self.pages.addWidget(self._scroll_page(self._build_observe_calibration()))
        self.pages.addWidget(self._scroll_page(self._build_pick_calibration()))
        self.pages.addWidget(self._scroll_page(self._build_place_calibration()))
        self.pages.addWidget(self._scroll_page(self.drop_v1_panel))
        self.pages.addWidget(self._scroll_page(self.manual_movel_panel))
        self.pages.addWidget(self._scroll_page(self.point_movel_panel))
        workspace = QHBoxLayout()
        workspace.setSpacing(14)
        workspace.addWidget(self.camera_preview_group, 3)
        workspace.addWidget(self.pages, 2)
        root.addLayout(workspace, 1)

        safety = QHBoxLayout()
        self.estop_button = QPushButton("急停")
        self.estop_button.setObjectName("estopButton")
        self.pump_toggle = PumpToggle()
        self.pump_button = self.pump_toggle
        self.recover_button = QPushButton("急停后恢复")
        self.recover_button.setEnabled(False)
        self.estop_button.clicked.connect(self.estop_requested.emit)
        self.pump_toggle.toggled.connect(self.pump_toggle_requested.emit)
        self.recover_button.clicked.connect(self.recover_requested.emit)
        safety.addWidget(self.estop_button, 2)
        safety.addWidget(self.pump_toggle, 1)
        safety.addWidget(self.recover_button, 1)
        root.addLayout(safety)

        self.setStyleSheet(self._stylesheet())

    @staticmethod
    def _scroll_page(page: QWidget) -> QScrollArea:
        scroll = QScrollArea()
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        scroll.setWidget(page)
        return scroll

    def _build_home(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(0, 6, 0, 0)
        layout.setSpacing(14)

        status_group = QGroupBox("设备状态")
        status_group.setMinimumHeight(178)
        status_layout = QVBoxLayout(status_group)
        self.stm32_status = StatusRow("STM32")
        self.camera_status = StatusRow("Camera")
        self.robot_reference_status = StatusRow("Robot Reference")
        self.board_status = StatusRow("Board")
        for row in (
            self.stm32_status,
            self.camera_status,
            self.robot_reference_status,
            self.board_status,
        ):
            status_layout.addWidget(row)
        layout.addWidget(status_group)

        summary_group = QGroupBox("上次标定")
        summary_group.setMinimumHeight(156)
        summary = QFormLayout(summary_group)
        summary.setVerticalSpacing(7)
        self.last_date_label = QLabel("—")
        self.last_anchor_label = QLabel("0")
        self.last_generated_label = QLabel("0")
        self.last_status_label = QLabel("未找到")
        summary.addRow("日期", self.last_date_label)
        summary.addRow("Anchors", self.last_anchor_label)
        summary.addRow("Generated Points", self.last_generated_label)
        summary.addRow("Status", self.last_status_label)
        layout.addWidget(summary_group)

        self.quick_button = QPushButton("快速标定")
        self.quick_button.setObjectName("primaryButton")
        self.quick_button.clicked.connect(self.quick_calibration_requested.emit)
        layout.addWidget(self.quick_button)

        secondary = QHBoxLayout()
        self.continue_button = QPushButton("继续使用")
        self.return_button = QPushButton("回观察位")
        self.continue_button.clicked.connect(self.continue_requested.emit)
        self.return_button.clicked.connect(self.return_observe_requested.emit)
        secondary.addWidget(self.continue_button)
        secondary.addWidget(self.return_button)
        layout.addLayout(secondary)

        tests = QHBoxLayout()
        self.pick_button = QPushButton("测试取料")
        self.place_button = QPushButton("DROP / PLACE V1")
        self.manual_movel_button = QPushButton("P77 Manual MoveL Tuning")
        self.point_movel_button = QPushButton("Manual MoveL Calibration V1")
        self.p77_full_cycle_button = QPushButton("一键取料 → P77 下棋")
        self.p77_full_cycle_button.setObjectName("primaryButton")
        self.p77_full_cycle_button.clicked.connect(
            self.p77_full_cycle_requested.emit
        )
        layout.addWidget(self.p77_full_cycle_button)
        self.pick_button.clicked.connect(self.pick_requested.emit)
        self.place_button.clicked.connect(self.place_requested.emit)
        self.manual_movel_button.clicked.connect(self.manual_movel_requested.emit)
        self.point_movel_button.clicked.connect(self.point_movel_requested.emit)
        tests.addWidget(self.pick_button)
        tests.addWidget(self.place_button)
        layout.addLayout(tests)
        layout.addWidget(self.manual_movel_button)
        layout.addWidget(self.point_movel_button)

        self.observe_calibrate_button = QPushButton("标定观察位 · 取料区上方")
        self.observe_calibrate_button.clicked.connect(
            self.observe_calibration_requested.emit
        )
        layout.addWidget(self.observe_calibrate_button)
        self.pick_calibrate_button = QPushButton("标定取料接触位")
        self.pick_calibrate_button.clicked.connect(
            self.pick_calibration_requested.emit
        )
        layout.addWidget(self.pick_calibrate_button)
        self.place_calibrate_button = QPushButton("标定 DROP / PLACE V1")
        self.place_calibrate_button.clicked.connect(
            self.place_calibration_requested.emit
        )
        layout.addWidget(self.place_calibrate_button)

        self.home_message = QLabel("下一步：连接 STM32 和 Camera，然后开始快速标定。")
        self.home_message.setObjectName("nextAction")
        self.home_message.setWordWrap(True)
        layout.addWidget(self.home_message)

        self.advanced_group = CollapsibleGroup("Advanced / Legacy")
        advanced_layout = QHBoxLayout()
        self.advanced_group.body_layout.addLayout(advanced_layout)
        self.legacy_button = QPushButton("打开 Legacy GUI")
        self.legacy_button.clicked.connect(self.open_legacy_requested.emit)
        self.details_label = QLabel("串口日志、225 点网格、诊断和开发工具位于 Legacy。")
        self.details_label.setWordWrap(True)
        advanced_layout.addWidget(self.details_label, 1)
        advanced_layout.addWidget(self.legacy_button)
        layout.addWidget(self.advanced_group)
        layout.addStretch(1)
        return page

    def _build_anchor(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(0, 6, 0, 0)
        layout.setSpacing(14)

        top = QHBoxLayout()
        self.anchor_step_label = QLabel("Step 1 / 5")
        self.anchor_step_label.setObjectName("stepLabel")
        self.anchor_progress_label = QLabel("0 / 5 已保存")
        top.addWidget(self.anchor_step_label)
        top.addStretch(1)
        top.addWidget(self.anchor_progress_label)
        layout.addLayout(top)

        self.anchor_point_label = QLabel("P00")
        self.anchor_point_label.setObjectName("pointTitle")
        self.anchor_coordinate_label = QLabel("棋盘坐标 (0,0)")
        self.anchor_coordinate_label.setObjectName("subtitle")
        layout.addWidget(self.anchor_point_label, 0, Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self.anchor_coordinate_label, 0, Qt.AlignmentFlag.AlignCenter)

        self.anchor_instruction = QLabel("下一步：先回观察位，然后移动到当前点 ABOVE。")
        self.anchor_instruction.setObjectName("nextAction")
        self.anchor_instruction.setWordWrap(True)
        layout.addWidget(self.anchor_instruction)

        self.anchor_move_button = QPushButton("移动到 ABOVE")
        self.anchor_move_button.setObjectName("primaryButton")
        self.anchor_move_button.clicked.connect(self.move_above_requested.emit)
        layout.addWidget(self.anchor_move_button)

        self.pwm_group = CollapsibleGroup("Advanced PWM")
        pwm_layout = self.pwm_group.body_layout
        pwm_note = QLabel("输入只改变目标值；按 Enter 或“移动”才执行。J5 为气泵，保持锁定。")
        pwm_note.setWordWrap(True)
        pwm_note.setObjectName("muted")
        pwm_layout.addWidget(pwm_note)
        for joint in range(5):
            row = PwmRow(joint)
            row.apply_requested.connect(self.pwm_apply_requested.emit)
            row.target_changed.connect(self.pwm_target_changed.emit)
            self._pwm_rows[joint] = row
            pwm_layout.addWidget(row)
        pump_row = QLabel("J5    气泵通道    已锁定")
        pump_row.setObjectName("lockedRow")
        pwm_layout.addWidget(pump_row)
        self.undo_pwm_button = QPushButton("Undo Last Adjustment")
        self.undo_pwm_button.clicked.connect(self.pwm_undo_requested.emit)
        pwm_layout.addWidget(self.undo_pwm_button)
        layout.addWidget(self.pwm_group)

        self.confirm_anchor_button = QPushButton("这个点准确 · 保存并到下一点")
        self.confirm_anchor_button.setObjectName("confirmButton")
        self.confirm_anchor_button.clicked.connect(self.confirm_anchor_requested.emit)
        layout.addWidget(self.confirm_anchor_button)

        self.anchor_back_button = QPushButton("返回首页")
        self.anchor_back_button.clicked.connect(self.back_home_requested.emit)
        layout.addWidget(self.anchor_back_button)
        layout.addStretch(1)
        return page

    def _build_generate(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(0, 18, 0, 0)
        layout.setSpacing(16)
        title = QLabel("5 个 Anchor 已完成")
        title.setObjectName("pointTitle")
        layout.addWidget(title, 0, Qt.AlignmentFlag.AlignCenter)
        self.generate_message = QLabel("下一步：保存并生成全盘。")
        self.generate_message.setObjectName("nextAction")
        self.generate_message.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self.generate_message)
        self.generate_button = QPushButton("保存并生成全盘")
        self.generate_button.setObjectName("primaryButton")
        self.generate_button.clicked.connect(self.generate_requested.emit)
        layout.addWidget(self.generate_button)
        self.generate_result = QLabel("等待生成")
        self.generate_result.setObjectName("resultPanel")
        self.generate_result.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.generate_result.setWordWrap(True)
        layout.addWidget(self.generate_result)
        self.begin_test_button = QPushButton("测试关键点")
        self.begin_test_button.setObjectName("confirmButton")
        self.begin_test_button.setEnabled(False)
        self.begin_test_button.clicked.connect(self.begin_test_requested.emit)
        layout.addWidget(self.begin_test_button)
        self.generate_add_anchor = self._build_add_anchor_group()
        layout.addWidget(self.generate_add_anchor)
        layout.addStretch(1)
        return page

    def _build_test(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(0, 12, 0, 0)
        layout.setSpacing(16)
        self.test_step_label = QLabel("Test 1 / 5")
        self.test_step_label.setObjectName("stepLabel")
        layout.addWidget(self.test_step_label)
        self.test_point_label = QLabel("Test P00")
        self.test_point_label.setObjectName("pointTitle")
        layout.addWidget(self.test_point_label, 0, Qt.AlignmentFlag.AlignCenter)
        self.test_instruction = QLabel("下一步：移动到 ABOVE 后观察实际位置。")
        self.test_instruction.setObjectName("nextAction")
        self.test_instruction.setWordWrap(True)
        layout.addWidget(self.test_instruction)
        self.test_move_button = QPushButton("移动到 ABOVE")
        self.test_move_button.setObjectName("primaryButton")
        self.test_move_button.clicked.connect(self.move_above_requested.emit)
        layout.addWidget(self.test_move_button)
        result = QHBoxLayout()
        self.test_accurate_button = QPushButton("准确")
        self.test_accurate_button.setObjectName("confirmButton")
        self.test_inaccurate_button = QPushButton("不准确 · 修正当前点")
        self.test_accurate_button.clicked.connect(self.test_accurate_requested.emit)
        self.test_inaccurate_button.clicked.connect(self.test_inaccurate_requested.emit)
        result.addWidget(self.test_accurate_button)
        result.addWidget(self.test_inaccurate_button)
        layout.addLayout(result)
        layout.addWidget(self._build_add_anchor_group())
        self.test_back_button = QPushButton("返回首页")
        self.test_back_button.clicked.connect(self.back_home_requested.emit)
        layout.addWidget(self.test_back_button)
        layout.addStretch(1)
        return page

    def _build_complete(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(0, 24, 0, 0)
        layout.setSpacing(18)
        title = QLabel("Calibration Ready")
        title.setObjectName("pointTitle")
        layout.addWidget(title, 0, Qt.AlignmentFlag.AlignCenter)
        self.complete_summary = QLabel("5 / 5 anchors\n225 / 225 points\n5 / 5 key points")
        self.complete_summary.setObjectName("resultPanel")
        self.complete_summary.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self.complete_summary)
        self.commit_button = QPushButton("保存标定并完成")
        self.commit_button.setObjectName("primaryButton")
        self.commit_button.clicked.connect(self.commit_requested.emit)
        layout.addWidget(self.commit_button)
        review = QGroupBox("回看已标定点位")
        review_layout = QHBoxLayout(review)
        self.review_target_combo = QComboBox()
        self.review_target_combo.addItems(
            [
                "P77", "P00", "P014", "P140", "P1414",
                "观察位 / PICK_ABOVE", "取料接触位", "落子接触位",
            ]
        )
        review_button = QPushButton("移动并回看")
        review_button.clicked.connect(
            lambda: self.review_target_requested.emit(
                self.review_target_combo.currentText()
            )
        )
        review_layout.addWidget(self.review_target_combo, 1)
        review_layout.addWidget(review_button)
        layout.addWidget(review)
        layout.addWidget(self._build_add_anchor_group())
        self.complete_home_button = QPushButton("返回首页")
        self.complete_home_button.clicked.connect(self.back_home_requested.emit)
        layout.addWidget(self.complete_home_button)
        layout.addStretch(1)
        return page

    def _build_add_anchor_group(self) -> QGroupBox:
        group = CollapsibleGroup("+ 添加 Anchor")
        camera_add = QPushButton("从 Camera 画面点击棋盘交点")
        camera_add.setObjectName("confirmButton")
        camera_add.clicked.connect(self.camera_anchor_pick_requested.emit)
        group.body_layout.addWidget(camera_add)
        hint = QLabel("点击后，在左侧实时画面中单击目标交点；系统会自动进入该点标定。")
        hint.setWordWrap(True)
        hint.setObjectName("muted")
        group.body_layout.addWidget(hint)
        layout = QHBoxLayout()
        group.body_layout.addLayout(layout)
        layout.addWidget(QLabel("Row"))
        row = QSpinBox()
        row.setRange(0, 14)
        layout.addWidget(row)
        layout.addWidget(QLabel("Col"))
        col = QSpinBox()
        col.setRange(0, 14)
        layout.addWidget(col)
        add = QPushButton("标定这个点")
        add.clicked.connect(
            lambda _c=False, r=row, c=col: self.add_anchor_requested.emit(
                r.value(), c.value()
            )
        )
        layout.addWidget(add)
        return group

    def _build_observe_calibration(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(0, 12, 0, 0)
        layout.setSpacing(14)
        self.observe_calibration_step = QLabel("独立标定")
        self.observe_calibration_step.setObjectName("stepLabel")
        layout.addWidget(self.observe_calibration_step)
        title = QLabel("观察位 / 取料区上方")
        title.setObjectName("pointTitle")
        layout.addWidget(title, 0, Qt.AlignmentFlag.AlignCenter)
        note = QLabel(
            "这是独立动作点 OBSERVE_IDLE，不参与棋盘 225 点插值。保存后，回观察位和取料流程会使用该校准。"
        )
        note.setObjectName("nextAction")
        note.setWordWrap(True)
        layout.addWidget(note)
        self.observe_move_button = QPushButton("移动到当前观察位")
        self.observe_move_button.setObjectName("primaryButton")
        self.observe_move_button.clicked.connect(self.observe_move_requested.emit)
        layout.addWidget(self.observe_move_button)
        rows = QGroupBox("观察位 PWM · J5 气泵锁定")
        rows_layout = QVBoxLayout(rows)
        for joint in range(5):
            row = PwmRow(joint)
            row.apply_requested.connect(self.observe_pwm_apply_requested.emit)
            row.target_changed.connect(self.observe_pwm_target_changed.emit)
            self._observe_pwm_rows[joint] = row
            rows_layout.addWidget(row)
        locked = QLabel("J5    气泵通道    已锁定关闭")
        locked.setObjectName("lockedRow")
        rows_layout.addWidget(locked)
        self.observe_undo_button = QPushButton("Undo Last Adjustment")
        self.observe_undo_button.clicked.connect(self.observe_pwm_undo_requested.emit)
        rows_layout.addWidget(self.observe_undo_button)
        layout.addWidget(rows)
        self.observe_save_button = QPushButton("保存观察位校准")
        self.observe_save_button.setObjectName("confirmButton")
        self.observe_save_button.clicked.connect(self.observe_save_requested.emit)
        layout.addWidget(self.observe_save_button)
        observe_back = QPushButton("返回首页")
        observe_back.clicked.connect(self.back_home_requested.emit)
        layout.addWidget(observe_back)
        layout.addStretch(1)
        return page

    def _build_pick_calibration(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(0, 12, 0, 0)
        layout.setSpacing(14)
        self.pick_calibration_step = QLabel("取料接触位")
        self.pick_calibration_step.setObjectName("stepLabel")
        layout.addWidget(self.pick_calibration_step)
        title = QLabel("PICK_DOWN · 取料接触位")
        title.setObjectName("pointTitle")
        layout.addWidget(title, 0, Qt.AlignmentFlag.AlignCenter)
        note = QLabel(
            "PICK_DOWN 是取料区的实际接触落点。观察位同时作为 PICK_ABOVE 和吸起后的悬停位；"
            "标定从观察位直接下降，期间 J5 保持关闭，正式取料时才在接触点开启泵嘴。"
        )
        note.setObjectName("nextAction")
        note.setWordWrap(True)
        layout.addWidget(note)
        self.pick_calibration_move_button = QPushButton("移动到当前取料接触位")
        self.pick_calibration_move_button.setObjectName("primaryButton")
        self.pick_calibration_move_button.clicked.connect(
            self.pick_calibration_move_requested.emit
        )
        layout.addWidget(self.pick_calibration_move_button)
        rows = QGroupBox("取料位 PWM · J5 气泵锁定关闭")
        rows_layout = QVBoxLayout(rows)
        for joint in range(5):
            row = PwmRow(joint)
            row.apply_requested.connect(self.pick_calibration_pwm_apply_requested.emit)
            row.target_changed.connect(self.pick_calibration_pwm_target_changed.emit)
            self._pick_pwm_rows[joint] = row
            rows_layout.addWidget(row)
        locked = QLabel("J5    气泵通道    标定时强制关闭")
        locked.setObjectName("lockedRow")
        rows_layout.addWidget(locked)
        pick_undo = QPushButton("Undo Last Adjustment")
        pick_undo.clicked.connect(self.pick_calibration_pwm_undo_requested.emit)
        rows_layout.addWidget(pick_undo)
        layout.addWidget(rows)
        self.pick_calibration_save_button = QPushButton("保存取料接触位")
        self.pick_calibration_save_button.setObjectName("confirmButton")
        self.pick_calibration_save_button.clicked.connect(
            self.pick_calibration_save_requested.emit
        )
        layout.addWidget(self.pick_calibration_save_button)
        pick_back = QPushButton("返回首页")
        pick_back.clicked.connect(self.back_home_requested.emit)
        layout.addWidget(pick_back)
        layout.addStretch(1)
        return page

    def _build_place_calibration(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(0, 12, 0, 0)
        layout.setSpacing(14)
        self.place_calibration_step = QLabel("独立标定")
        self.place_calibration_step.setObjectName("stepLabel")
        layout.addWidget(self.place_calibration_step)
        title = QLabel("P77 落子接触位")
        title.setObjectName("pointTitle")
        layout.addWidget(title, 0, Qt.AlignmentFlag.AlignCenter)
        note = QLabel(
            "以快速标定的 P77 ABOVE 为起点，沿用原动作的相对下降量生成接触位。"
            "这是分层 PWM 下降候选，并非已验证的笛卡尔 MoveL；标定时泵嘴强制关闭。"
        )
        note.setObjectName("nextAction")
        note.setWordWrap(True)
        layout.addWidget(note)
        self.place_calibration_status = QLabel("先移动到候选接触位，再确认是否准确。")
        self.place_calibration_status.setObjectName("resultPanel")
        self.place_calibration_status.setWordWrap(True)
        layout.addWidget(self.place_calibration_status)
        self.place_calibration_move_button = QPushButton("移动到 P77 落子接触候选位")
        self.place_calibration_move_button.setObjectName("primaryButton")
        self.place_calibration_move_button.clicked.connect(
            self.place_calibration_move_requested.emit
        )
        layout.addWidget(self.place_calibration_move_button)
        rows = QGroupBox("落子接触位 PWM · J5 标定时锁定关闭")
        rows_layout = QVBoxLayout(rows)
        for joint in range(5):
            row = PwmRow(joint)
            row.apply_requested.connect(self.place_calibration_pwm_apply_requested.emit)
            row.target_changed.connect(
                self.place_calibration_pwm_target_changed.emit
            )
            self._place_pwm_rows[joint] = row
            rows_layout.addWidget(row)
        locked = QLabel("J5    泵嘴通道    标定时强制关闭")
        locked.setObjectName("lockedRow")
        rows_layout.addWidget(locked)
        place_undo = QPushButton("Undo Last Adjustment")
        place_undo.clicked.connect(self.place_calibration_pwm_undo_requested.emit)
        rows_layout.addWidget(place_undo)
        layout.addWidget(rows)
        result = QHBoxLayout()
        self.place_calibration_accurate_button = QPushButton("位置准确 · 保存")
        self.place_calibration_accurate_button.setObjectName("confirmButton")
        self.place_calibration_inaccurate_button = QPushButton("位置不准确 · 继续微调")
        self.place_calibration_accurate_button.clicked.connect(
            self.place_calibration_accurate_requested.emit
        )
        self.place_calibration_inaccurate_button.clicked.connect(
            self.place_calibration_inaccurate_requested.emit
        )
        result.addWidget(self.place_calibration_accurate_button)
        result.addWidget(self.place_calibration_inaccurate_button)
        layout.addLayout(result)
        place_back = QPushButton("返回首页")
        place_back.clicked.connect(self.back_home_requested.emit)
        layout.addWidget(place_back)
        layout.addStretch(1)
        return page

    def _toggle_serial(self) -> None:
        if self.serial_button.property("connected"):
            self.disconnect_serial_requested.emit()
        else:
            self.connect_serial_requested.emit(self.port_combo.currentText())

    def _toggle_camera(self) -> None:
        if self.camera_button.property("connected"):
            self.disconnect_camera_requested.emit()
        else:
            self.connect_camera_requested.emit()

    def set_serial_connected(self, connected: bool) -> None:
        self.serial_button.setProperty("connected", bool(connected))
        self.serial_button.setText("断开 STM32" if connected else "连接 STM32")
        self.port_combo.setEnabled(not connected)

    def set_camera_connected(self, connected: bool) -> None:
        self.camera_button.setProperty("connected", bool(connected))
        self.camera_button.setText("断开 Camera" if connected else "连接 Camera")

    def set_summary(self, *, date: str, anchors: int, generated: int, status: str) -> None:
        self.last_date_label.setText(date)
        self.last_anchor_label.setText(str(int(anchors)))
        self.last_generated_label.setText(str(int(generated)))
        self.last_status_label.setText(status)
        self.continue_button.setEnabled(bool(date and status != "未找到"))

    def show_home(self) -> None:
        self.pages.setCurrentIndex(self.HOME)

    def show_anchor(
        self,
        *,
        label: str,
        row: int,
        col: int,
        step: int,
        total: int,
        saved: int,
        correction: bool = False,
    ) -> None:
        self.anchor_step_label.setText(
            "补充 Anchor" if correction else f"Step {step} / {total}"
        )
        self.anchor_progress_label.setText(f"{saved} / {total} 已保存")
        self.anchor_point_label.setText(label)
        self.anchor_coordinate_label.setText(f"棋盘坐标 ({row},{col})")
        self.confirm_anchor_button.setText(
            "保存修正并重新计算" if correction else "这个点准确 · 保存并到下一点"
        )
        self.pages.setCurrentIndex(self.ANCHOR)

    def show_generate(self) -> None:
        self.pages.setCurrentIndex(self.GENERATE)

    def show_test(self, *, label: str, step: int, total: int) -> None:
        self.test_accurate_button.setVisible(True)
        self.test_inaccurate_button.setVisible(True)
        self.test_step_label.setText(f"Test {step} / {total}")
        self.test_point_label.setText(f"Test {label}")
        self.pages.setCurrentIndex(self.TEST)

    def show_review(self, *, label: str, row: int, col: int) -> None:
        self.test_step_label.setText("回看已标定点位")
        self.test_point_label.setText(f"Review {label}")
        self.test_instruction.setText(
            f"棋盘坐标 ({row},{col})。点击移动到 ABOVE；查看后点返回首页会自动安全返回观察位。"
        )
        self.test_accurate_button.setVisible(False)
        self.test_inaccurate_button.setVisible(False)
        self.pages.setCurrentIndex(self.TEST)

    def show_complete(self, *, anchors: int, points: int, verified: int) -> None:
        self.complete_summary.setText(
            f"{anchors} anchors saved\n{points} / 225 points generated\n"
            f"{verified} / 5 key points verified"
        )
        self.pages.setCurrentIndex(self.COMPLETE)

    def set_pwm_values(self, pwm: Mapping[int | str, int]) -> None:
        for joint, row in self._pwm_rows.items():
            key = f"{joint:03d}"
            value = int(pwm[key] if key in pwm else pwm[joint])
            row.set_values(value)

    def pwm_values(self) -> dict[str, int]:
        return {
            f"{joint:03d}": row.target.value()
            for joint, row in self._pwm_rows.items()
        }

    def set_pwm_current(self, joint: int, value: int) -> None:
        self._pwm_rows[int(joint)].set_current(int(value))

    def show_observe_calibration(self, pwm: Mapping[int | str, int]) -> None:
        for joint, row in self._observe_pwm_rows.items():
            key = f"{joint:03d}"
            value = int(pwm[key] if key in pwm else pwm[joint])
            row.set_values(value)
        self.pages.setCurrentIndex(self.OBSERVE)

    def observe_pwm_values(self) -> dict[str, int]:
        return {
            f"{joint:03d}": row.target.value()
            for joint, row in self._observe_pwm_rows.items()
        }

    def set_observe_pwm_current(self, joint: int, value: int) -> None:
        self._observe_pwm_rows[int(joint)].set_current(int(value))

    def set_observe_quick_mode(self, enabled: bool) -> None:
        self.observe_calibration_step.setText(
            "Step 6 / 7 · 快速标定" if enabled else "独立标定"
        )
        self.observe_move_button.setText(
            "已在观察位 · 调整后保存" if enabled else "移动到当前观察位"
        )

    def show_pick_calibration(
        self, pwm: Mapping[int | str, int], *, quick_mode: bool = False
    ) -> None:
        for joint, row in self._pick_pwm_rows.items():
            key = f"{joint:03d}"
            value = int(pwm[key] if key in pwm else pwm[joint])
            row.set_values(value)
        self.pick_calibration_step.setText(
            "Step 7 / 8 · 快速标定" if quick_mode else "独立标定"
        )
        self.pages.setCurrentIndex(self.PICK_CALIBRATION)

    def pick_calibration_pwm_values(self) -> dict[str, int]:
        return {
            f"{joint:03d}": row.target.value()
            for joint, row in self._pick_pwm_rows.items()
        }

    def set_pick_calibration_pwm_current(self, joint: int, value: int) -> None:
        self._pick_pwm_rows[int(joint)].set_current(int(value))

    def show_place_calibration(
        self,
        pwm: Mapping[int | str, int],
        *,
        quick_mode: bool = False,
        verified: bool = False,
    ) -> None:
        for joint, row in self._place_pwm_rows.items():
            key = f"{joint:03d}"
            value = int(pwm[key] if key in pwm else pwm[joint])
            row.set_values(value)
        self.place_calibration_step.setText(
            "Step 8 / 8 · 快速标定" if quick_mode else "独立标定"
        )
        self.place_calibration_status.setText(
            "已保存且确认准确；仍可重新移动复核。"
            if verified
            else "由最新 P77 ABOVE + 原相对下降量生成，尚未确认。"
        )
        self.pages.setCurrentIndex(self.PLACE_CALIBRATION)

    def show_drop_v1(self, *, row: int = 7, col: int = 7) -> None:
        self.drop_v1_panel.set_point(row, col)
        self.pages.setCurrentIndex(self.DROP_V1)

    def show_manual_movel(self) -> None:
        self.pages.setCurrentIndex(self.MANUAL_MOVEL)

    def show_point_movel(self) -> None:
        self.pages.setCurrentIndex(self.POINT_MOVEL)

    def place_calibration_pwm_values(self) -> dict[str, int]:
        return {
            f"{joint:03d}": row.target.value()
            for joint, row in self._place_pwm_rows.items()
        }

    def set_place_calibration_pwm_current(self, joint: int, value: int) -> None:
        self._place_pwm_rows[int(joint)].set_current(int(value))

    def set_pump_state(self, enabled: bool) -> None:
        self.pump_toggle.set_pump_on(enabled)

    def set_motion_ready(self, ready: bool) -> None:
        self.anchor_move_button.setEnabled(ready)
        self.test_move_button.setEnabled(ready)

    def _stylesheet(self) -> str:
        return """
        QWidget#calibrationLite {
            background: #f4f5f7;
            color: #1d2733;
            font-family: "Microsoft YaHei UI";
            font-size: 14px;
        }
        QLabel#appTitle { font-size: 24px; font-weight: 700; }
        QLabel#subtitle, QLabel#muted { color: #697582; }
        QLabel#globalState {
            background: #e5e7eb; border-radius: 4px; padding: 8px 14px; font-weight: 700;
        }
        QLabel[tone="ready"] { color: #197044; font-weight: 700; }
        QLabel[tone="warning"] { color: #a05a00; font-weight: 700; }
        QLabel[tone="error"] { color: #b3262e; font-weight: 700; }
        QLabel[tone="neutral"] { color: #697582; font-weight: 600; }
        QLabel#stepLabel { color: #38526b; font-weight: 700; }
        QLabel#pointTitle { font-size: 28px; font-weight: 700; }
        QLabel#nextAction {
            background: #eef3f7; border-left: 4px solid #385f7a; padding: 12px;
        }
        QLabel#resultPanel {
            background: white; border: 1px solid #cfd5db; border-radius: 5px;
            padding: 20px; font-size: 16px; line-height: 1.5;
        }
        QLabel#lockedRow {
            background: #eceff2; color: #687480; padding: 8px; border-radius: 3px;
        }
        QGroupBox {
            background: white; border: 1px solid #cfd5db; border-radius: 5px;
            margin-top: 11px; padding: 13px 12px 10px 12px; font-weight: 600;
        }
        QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; }
        QPushButton {
            min-height: 38px; background: white; border: 1px solid #aeb7c0;
            border-radius: 4px; padding: 5px 14px;
        }
        QPushButton:hover { background: #edf2f5; }
        QPushButton:disabled { color: #9ca3aa; background: #eceff2; }
        QPushButton#primaryButton {
            min-height: 56px; background: #315f7d; color: white; border: 1px solid #284f68;
            font-size: 18px; font-weight: 700;
        }
        QPushButton#primaryButton:hover { background: #284f68; }
        QPushButton#confirmButton {
            min-height: 46px; background: #eaf5ef; border: 1px solid #5e9b78;
            color: #185c39; font-weight: 700;
        }
        QPushButton#estopButton {
            min-height: 52px; background: #b3262e; color: white; border: 1px solid #921d24;
            font-size: 18px; font-weight: 700;
        }
        QPushButton[compact="true"] { min-height: 28px; padding: 2px 8px; }
        QCheckBox#pumpToggle {
            min-height: 42px; spacing: 12px; padding: 5px 12px; font-weight: 700;
        }
        QCheckBox#pumpToggle::indicator {
            width: 52px; height: 26px; border-radius: 13px;
            border: 1px solid #9aa5ae; background: #d9dee3;
        }
        QCheckBox#pumpToggle::indicator:checked {
            background: #2f7d57; border: 1px solid #246344;
        }
        QCheckBox#pumpToggle:disabled { color: #9ca3aa; }
        QComboBox, QSpinBox {
            min-height: 34px; background: white; border: 1px solid #aeb7c0;
            border-radius: 3px; padding: 1px 7px;
        }
        """
