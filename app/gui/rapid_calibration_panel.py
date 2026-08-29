from __future__ import annotations

from typing import Any, Mapping

from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtGui import QKeySequence, QShortcut
from PySide6.QtWidgets import (
    QButtonGroup,
    QComboBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QRadioButton,
    QSizePolicy,
    QSpinBox,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from app.stage7.baseline import SPATIAL_KEYS, point_label
from .drop_calibration_panel import DropCalibrationPanel
from .game_panel import GomokuGamePanel


class RapidCalibrationPanel(QWidget):
    """Calibration-first task UI; it emits intents and never owns serial I/O."""

    load_baseline_requested = Signal()
    new_session_requested = Signal(str)
    point_selected = Signal(int, int)
    move_above_requested = Signal()
    jog_requested = Signal(int, int)
    apply_joint_requested = Signal(int, int, int)
    apply_all_requested = Signal(object, int)
    save_anchor_requested = Signal()
    recalculate_requested = Signal()
    verify_requested = Signal()
    commit_requested = Signal()
    rollback_requested = Signal()
    pick_pose_selected = Signal(str)
    save_pick_pose_requested = Signal(str)
    move_pick_above_requested = Signal()
    return_observe_requested = Signal()
    pick_once_requested = Signal()
    runtime_cycle_requested = Signal()
    runtime_stop_requested = Signal()

    # Normal reuses the stable action-table time. Slow/Fast are explicit first-pass
    # presets and remain labelled NOT HARDWARE TUNED in the UI.
    SPEED_TIMES_MS = {"Slow": 1500, "Normal": 1000, "Fast": 700}

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._row = 0
        self._col = 0
        self._selected_joint = 0
        self._continuous_test = False
        self._flash_point: tuple[int, int] | None = None
        self._flash_visible = False
        self._flash_steps_remaining = 0
        self._grid_buttons: dict[tuple[int, int], QPushButton] = {}
        self._joint_edits: dict[int, QSpinBox] = {}
        self._joint_radios: dict[int, QRadioButton] = {}
        self._joint_apply_buttons: dict[int, QPushButton] = {}
        self._step_buttons: dict[int, QRadioButton] = {}
        self._speed_buttons: dict[str, QRadioButton] = {}

        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(8)
        root.addWidget(self._build_mode_strip())
        root.addWidget(self._build_current_point_strip())

        self.task_tabs = QTabWidget()
        self.task_tabs.setDocumentMode(True)
        self.task_tabs.addTab(self._build_calibration_page(), "1  快速标定")
        self.task_tabs.addTab(self._build_verification_page(), "2  点位验证")
        self.task_tabs.addTab(self._build_runtime_page(), "3  运行")
        self.drop_calibration_panel = DropCalibrationPanel()
        self.game_panel = GomokuGamePanel()
        self.task_tabs.addTab(self.drop_calibration_panel, "Integrated V1 · Calibration")
        self.task_tabs.addTab(self.game_panel, "Game")
        self.game_panel.calibration_requested.connect(self.show_first_setup)
        self.game_panel.quick_check_requested.connect(self.show_quick_check)
        self.game_panel.fast_calibration_requested.connect(self.show_drop_calibration)
        self.game_panel.full_calibration_requested.connect(self.show_drop_calibration)
        self.game_panel.advanced_calibration_requested.connect(self.show_advanced_calibration)
        root.addWidget(self.task_tabs, 1)

        self._install_shortcuts()
        self._flash_timer = QTimer(self)
        self._flash_timer.setInterval(180)
        self._flash_timer.timeout.connect(self._tick_point_flash)
        self.select_point(0, 0, emit=False)

    def show_game(self) -> None:
        self.task_tabs.setCurrentWidget(self.game_panel)

    def show_first_setup(self) -> None:
        self.show_drop_calibration()

    def show_drop_calibration(self) -> None:
        self.task_tabs.setCurrentWidget(self.drop_calibration_panel)

    def show_quick_check(self) -> None:
        self.drop_calibration_panel.select_point("P77")
        self.show_drop_calibration()

    def show_advanced_calibration(self) -> None:
        self.task_tabs.setCurrentIndex(0)

    def _build_mode_strip(self) -> QGroupBox:
        group = QGroupBox("标定状态")
        layout = QGridLayout(group)
        layout.setColumnStretch(0, 1)
        self.baseline_label = QLabel("Baseline: not loaded")
        self.session_label = QLabel("Session: not created")
        self.session_label.setWordWrap(True)
        self.live_status_label = QLabel("SERIAL NOT CONNECTED / NOT SENT")
        self.live_status_label.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        self.live_status_label.setStyleSheet("font-weight: 700; color: #a05a00;")
        layout.addWidget(self.baseline_label, 0, 0)
        layout.addWidget(self.session_label, 1, 0)
        layout.addWidget(self.live_status_label, 1, 1)
        return group

    def _build_current_point_strip(self) -> QGroupBox:
        group = QGroupBox("当前点")
        layout = QHBoxLayout(group)
        self.current_point_label = QLabel(point_label(0, 0))
        self.current_point_label.setStyleSheet("font-size: 18px; font-weight: 700;")
        self.point_source_label = QLabel("UNSET")
        self.point_index = QSpinBox()
        self.point_index.setRange(0, 224)
        self.point_index.setPrefix("P")
        self.point_index.valueChanged.connect(self._on_index_changed)
        layout.addWidget(self.current_point_label)
        layout.addWidget(self.point_source_label, 1)
        layout.addWidget(QLabel("点号"))
        layout.addWidget(self.point_index)
        return group

    def _build_calibration_page(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(4, 8, 4, 4)
        layout.addWidget(self._build_session_group())
        layout.addWidget(self._build_joint_group())
        layout.addWidget(self._build_calibration_actions())
        layout.addWidget(self._build_pick_group())
        layout.addStretch(1)
        return page

    def _build_session_group(self) -> QGroupBox:
        group = QGroupBox("标定方案")
        layout = QGridLayout(group)
        self.mode_combo = QComboBox()
        self.mode_combo.addItem("QUICK 5 · P00 P14 P112 P210 P224", "QUICK_5")
        self.mode_combo.addItem("STANDARD 9 · 推荐", "STANDARD_9")
        self.mode_combo.setCurrentIndex(1)
        self.load_baseline_button = QPushButton("载入稳定 Baseline")
        self.new_session_button = QPushButton("开始新标定")
        layout.addWidget(self.mode_combo, 0, 0, 1, 2)
        layout.addWidget(self.load_baseline_button, 1, 0)
        layout.addWidget(self.new_session_button, 1, 1)
        self.load_baseline_button.clicked.connect(
            lambda _checked=False: self.load_baseline_requested.emit()
        )
        self.new_session_button.clicked.connect(
            lambda _checked=False: self.new_session_requested.emit(str(self.mode_combo.currentData()))
        )
        return group

    def _build_joint_group(self) -> QGroupBox:
        group = QGroupBox("PWM Editor · 仅 J0–J4；J5=气泵，已锁定")
        layout = QGridLayout(group)
        layout.setHorizontalSpacing(8)
        headers = ("选", "关节", "PWM", "Apply", "−", "+")
        for column, text in enumerate(headers):
            layout.addWidget(QLabel(text), 0, column)

        radio_group = QButtonGroup(self)
        for joint, key in enumerate(SPATIAL_KEYS):
            radio = QRadioButton()
            radio.setChecked(joint == 0)
            radio.toggled.connect(
                lambda checked, jid=joint: self._select_joint(jid) if checked else None
            )
            radio_group.addButton(radio, joint)
            edit = QSpinBox()
            edit.setRange(550, 2450)
            edit.setSingleStep(1)
            edit.setKeyboardTracking(False)
            edit.setAccelerated(False)
            edit.lineEdit().setToolTip("输入只修改本地值；按 Enter 或 Apply 才发送")
            edit.lineEdit().returnPressed.connect(
                lambda jid=joint: self._emit_apply_joint(jid)
            )
            apply_button = QPushButton("Apply")
            minus = QPushButton("−")
            plus = QPushButton("+")
            apply_button.clicked.connect(
                lambda _checked=False, jid=joint: self._emit_apply_joint(jid)
            )
            minus.clicked.connect(
                lambda _checked=False, jid=joint: self._emit_jog(jid, -self.step())
            )
            plus.clicked.connect(
                lambda _checked=False, jid=joint: self._emit_jog(jid, self.step())
            )
            layout.addWidget(radio, joint + 1, 0)
            layout.addWidget(QLabel(f"J{joint} / {key}"), joint + 1, 1)
            layout.addWidget(edit, joint + 1, 2)
            layout.addWidget(apply_button, joint + 1, 3)
            layout.addWidget(minus, joint + 1, 4)
            layout.addWidget(plus, joint + 1, 5)
            self._joint_radios[joint] = radio
            self._joint_edits[joint] = edit
            self._joint_apply_buttons[joint] = apply_button

        step_row = QHBoxLayout()
        step_row.addWidget(QLabel("Step"))
        step_group = QButtonGroup(self)
        for value in (1, 5, 10, 20):
            button = QRadioButton(str(value))
            button.setChecked(value == 5)
            step_group.addButton(button, value)
            step_row.addWidget(button)
            self._step_buttons[value] = button
        step_row.addSpacing(16)
        step_row.addWidget(QLabel("速度"))
        speed_group = QButtonGroup(self)
        for index, name in enumerate(("Slow", "Normal", "Fast")):
            button = QRadioButton(name)
            button.setChecked(name == "Normal")
            speed_group.addButton(button, index)
            step_row.addWidget(button)
            self._speed_buttons[name] = button
        step_row.addStretch(1)
        layout.addLayout(step_row, 6, 0, 1, 6)
        note = QLabel("Normal=1000 ms（稳定动作默认）；Slow/Fast 为 NOT HARDWARE TUNED 预设")
        note.setStyleSheet("color: #6b7280;")
        layout.addWidget(note, 7, 0, 1, 6)
        return group

    def _build_calibration_actions(self) -> QGroupBox:
        group = QGroupBox("当前点操作")
        layout = QGridLayout(group)
        self.apply_all_button = QPushButton("Apply All · 整体粗定位")
        self.move_button = QPushButton("Move ABOVE")
        self.save_anchor_button = QPushButton("Save Point · DIRECT")
        self.next_button = QPushButton("Next")
        self.recalculate_button = QPushButton("Generate / Recalculate 225")
        self.commit_button = QPushButton("Commit Calibration")
        self.rollback_button = QPushButton("Rollback Baseline")
        self.workflow_label = QLabel("载入 Baseline → 新建 Session → 标定 5/9 个点")
        self.workflow_label.setWordWrap(True)
        self.workflow_label.setStyleSheet("color: #334155;")
        layout.addWidget(self.apply_all_button, 0, 0)
        layout.addWidget(self.move_button, 0, 1)
        layout.addWidget(self.save_anchor_button, 1, 0)
        layout.addWidget(self.next_button, 1, 1)
        layout.addWidget(self.recalculate_button, 2, 0, 1, 2)
        layout.addWidget(self.commit_button, 3, 0)
        layout.addWidget(self.rollback_button, 3, 1)
        layout.addWidget(self.workflow_label, 4, 0, 1, 2)
        self.apply_all_button.clicked.connect(lambda _c=False: self._emit_apply_all())
        self.move_button.clicked.connect(lambda _c=False: self.move_above_requested.emit())
        self.save_anchor_button.clicked.connect(lambda _c=False: self.save_anchor_requested.emit())
        self.next_button.clicked.connect(lambda _c=False: self.select_next_point())
        self.recalculate_button.clicked.connect(lambda _c=False: self.recalculate_requested.emit())
        self.commit_button.clicked.connect(lambda _c=False: self.commit_requested.emit())
        self.rollback_button.clicked.connect(lambda _c=False: self.rollback_requested.emit())
        return group

    def _build_verification_page(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(4, 8, 4, 4)
        layout.setAlignment(Qt.AlignmentFlag.AlignTop)
        legend = QLabel("AUTO  灰黄   ·   DIRECT  蓝   ·   VERIFIED  绿   ·   ERROR  红")
        legend.setStyleSheet("color: #475569;")
        layout.addWidget(legend)
        layout.addWidget(self._build_grid_group())
        actions = QHBoxLayout()
        self.verify_move_button = QPushButton("Move ABOVE")
        self.verify_button = QPushButton("准确 · Verify")
        self.recalibrate_button = QPushButton("不准确 · 重新标定")
        self.continuous_button = QPushButton("连续测试")
        self.continuous_button.setCheckable(True)
        actions.addWidget(self.verify_move_button)
        actions.addWidget(self.verify_button)
        actions.addWidget(self.recalibrate_button)
        actions.addWidget(self.continuous_button)
        layout.addLayout(actions)
        self.verify_move_button.clicked.connect(lambda _c=False: self.move_above_requested.emit())
        self.verify_button.clicked.connect(lambda _c=False: self._verify_and_maybe_advance())
        self.recalibrate_button.clicked.connect(lambda _c=False: self.task_tabs.setCurrentIndex(0))
        self.continuous_button.toggled.connect(self._set_continuous_test)
        layout.addStretch(1)
        return page

    def _build_grid_group(self) -> QGroupBox:
        group = QGroupBox("15 × 15 点位")
        group.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Maximum)
        layout = QGridLayout(group)
        layout.setSpacing(2)
        for row in range(15):
            for col in range(15):
                index = row * 15 + col
                button = QPushButton(f"{index:03d}")
                button.setMinimumWidth(29)
                button.setFixedHeight(23)
                button.setToolTip(point_label(row, col))
                button.clicked.connect(
                    lambda _checked=False, r=row, c=col: self.select_point(r, c)
                )
                layout.addWidget(button, row, col)
                self._grid_buttons[(row, col)] = button
        return group

    def _build_pick_group(self) -> QGroupBox:
        group = QGroupBox("固定取料 · 候选姿态与稳定流程分离")
        layout = QGridLayout(group)
        self.pick_pose_combo = QComboBox()
        self.pick_pose_combo.addItem("选择取料姿态…", "")
        self.pick_pose_combo.addItem("PICK_ABOVE", "PICK_ABOVE")
        self.pick_pose_combo.addItem("PICK_DOWN", "PICK_DOWN")
        self.pick_pose_status = QLabel("候选姿态：OFFLINE / NOT HARDWARE VERIFIED")
        self.pick_pose_status.setStyleSheet("color: #a05a00; font-weight: 600;")
        self.move_pick_above_button = QPushButton("Move PICK ABOVE · stable")
        self.save_pick_pose_button = QPushButton("保存候选姿态")
        self.pick_once_button = QPushButton("一键取料：观察位 → 取料 → 观察位悬停")
        layout.addWidget(self.pick_pose_combo, 0, 0)
        layout.addWidget(self.pick_pose_status, 0, 1)
        layout.addWidget(self.move_pick_above_button, 1, 0)
        layout.addWidget(self.save_pick_pose_button, 1, 1)
        layout.addWidget(self.pick_once_button, 2, 0, 1, 2)
        self.pick_pose_combo.currentIndexChanged.connect(
            lambda _index: self.pick_pose_selected.emit(
                str(self.pick_pose_combo.currentData())
            )
            if self.pick_pose_combo.currentData()
            else None
        )
        self.move_pick_above_button.clicked.connect(lambda _c=False: self.move_pick_above_requested.emit())
        self.save_pick_pose_button.clicked.connect(
            lambda _c=False: self.save_pick_pose_requested.emit(
                str(self.pick_pose_combo.currentData())
            )
        )
        self.pick_once_button.clicked.connect(lambda _c=False: self.pick_once_requested.emit())
        return group

    def _build_runtime_page(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(16, 18, 16, 16)
        title = QLabel("运行准备")
        title.setStyleSheet("font-size: 18px; font-weight: 700;")
        layout.addWidget(title)
        status_group = QGroupBox("状态")
        status_layout = QGridLayout(status_group)
        self.runtime_robot_label = QLabel("Robot: DISCONNECTED")
        self.runtime_camera_label = QLabel("Camera: NOT READY")
        self.runtime_calibration_label = QLabel("Calibration: BASELINE ONLY")
        self.runtime_pick_label = QLabel("Pick Pose: STABLE FLOW / CANDIDATE NOT VERIFIED")
        for row, widget in enumerate((
            self.runtime_robot_label,
            self.runtime_camera_label,
            self.runtime_calibration_label,
            self.runtime_pick_label,
        )):
            status_layout.addWidget(widget, row, 0)
        layout.addWidget(status_group)
        self.runtime_return_button = QPushButton("回观察位（取料区上方）")
        self.runtime_pick_button = QPushButton("取料：观察位 → 取料 → 观察位悬停")
        self.runtime_cycle_button = QPushButton("开始固定点演示")
        self.runtime_stop_button = QPushButton("停止")
        self.runtime_stop_button.setStyleSheet(
            "QPushButton { background: #b3262e; color: white; font-weight: 700; min-height: 42px; }"
        )
        layout.addWidget(self.runtime_return_button)
        layout.addWidget(self.runtime_pick_button)
        layout.addWidget(self.runtime_cycle_button)
        layout.addWidget(self.runtime_stop_button)
        layout.addStretch(1)
        self.runtime_return_button.clicked.connect(
            lambda _c=False: self.return_observe_requested.emit()
        )
        self.runtime_pick_button.clicked.connect(lambda _c=False: self.pick_once_requested.emit())
        self.runtime_cycle_button.clicked.connect(lambda _c=False: self.runtime_cycle_requested.emit())
        self.runtime_stop_button.clicked.connect(lambda _c=False: self.runtime_stop_requested.emit())
        return page

    def _install_shortcuts(self) -> None:
        shortcuts = (
            ("Up", lambda: self._emit_jog(self._selected_joint, self.step())),
            ("Down", lambda: self._emit_jog(self._selected_joint, -self.step())),
            ("Ctrl+Up", lambda: self._emit_jog(self._selected_joint, 1)),
            ("Ctrl+Down", lambda: self._emit_jog(self._selected_joint, -1)),
            ("Shift+Up", lambda: self._emit_jog(self._selected_joint, 20)),
            ("Shift+Down", lambda: self._emit_jog(self._selected_joint, -20)),
            ("S", lambda: self.save_anchor_requested.emit()),
            ("N", self.select_next_point),
            ("Space", lambda: self.task_tabs.setCurrentIndex(0)),
        )
        self._shortcuts: list[QShortcut] = []
        for sequence, callback in shortcuts:
            shortcut = QShortcut(QKeySequence(sequence), self)
            shortcut.setContext(Qt.ShortcutContext.WidgetWithChildrenShortcut)
            shortcut.activated.connect(callback)
            self._shortcuts.append(shortcut)

    def _on_index_changed(self, index: int) -> None:
        self.select_point(*divmod(int(index), 15))

    def _select_joint(self, joint: int) -> None:
        self._selected_joint = int(joint)

    def _emit_jog(self, joint: int, delta: int) -> None:
        jid = int(joint)
        self._joint_radios[jid].setChecked(True)
        self.jog_requested.emit(jid, int(delta))

    def _emit_apply_joint(self, joint: int) -> None:
        jid = int(joint)
        self._joint_radios[jid].setChecked(True)
        self.apply_joint_requested.emit(jid, self.joint_pwm(jid), self.speed_time_ms())

    def _emit_apply_all(self) -> None:
        self.apply_all_requested.emit(self.pwm_values(), self.speed_time_ms())

    def _verify_and_maybe_advance(self) -> None:
        self.verify_requested.emit()

    def _set_continuous_test(self, enabled: bool) -> None:
        self._continuous_test = bool(enabled)
        self.continuous_button.setText("连续测试 · ON" if enabled else "连续测试")

    def select_next_point(self) -> None:
        next_index = min(224, self._row * 15 + self._col + 1)
        self.select_point(*divmod(next_index, 15))

    def select_point(self, row: int, col: int, *, emit: bool = True) -> None:
        row, col = int(row), int(col)
        if not 0 <= row < 15 or not 0 <= col < 15:
            raise ValueError("point outside 15x15 board")
        self._row, self._col = row, col
        self.current_point_label.setText(point_label(row, col))
        index = row * 15 + col
        self.point_index.blockSignals(True)
        self.point_index.setValue(index)
        self.point_index.blockSignals(False)
        self._refresh_selected_border()
        if emit:
            self.point_selected.emit(row, col)

    def _refresh_selected_border(self) -> None:
        for (row, col), button in self._grid_buttons.items():
            base = button.property("source_color") or "#eeeeee"
            if self._flash_visible and (row, col) == self._flash_point:
                base = "#fb923c"
                border = "3px solid #9a3412"
            else:
                border = "2px solid #111827" if (row, col) == (self._row, self._col) else "1px solid #94a3b8"
            button.setStyleSheet(f"background: {base}; border: {border}; padding: 0;")

    def flash_point(self, row: int, col: int, *, pulses: int = 4) -> None:
        """Pulse the simulated target in the grid; no hardware side effects."""
        self._flash_point = (int(row), int(col))
        self._flash_visible = True
        self._flash_steps_remaining = max(2, int(pulses) * 2)
        self._refresh_selected_border()
        self._flash_timer.start()

    def _tick_point_flash(self) -> None:
        self._flash_steps_remaining -= 1
        self._flash_visible = not self._flash_visible
        if self._flash_steps_remaining <= 0:
            self._flash_timer.stop()
            self._flash_visible = False
        self._refresh_selected_border()

    @property
    def current_point(self) -> tuple[int, int]:
        return self._row, self._col

    def step(self) -> int:
        return next((value for value, button in self._step_buttons.items() if button.isChecked()), 5)

    def speed_time_ms(self) -> int:
        name = next((name for name, button in self._speed_buttons.items() if button.isChecked()), "Normal")
        return int(self.SPEED_TIMES_MS[name])

    def pwm_values(self) -> dict[str, int]:
        return {SPATIAL_KEYS[joint]: edit.value() for joint, edit in self._joint_edits.items()}

    def set_pwm_values(self, values: Mapping[int | str, int]) -> None:
        for joint, key in enumerate(SPATIAL_KEYS):
            raw = values[key] if key in values else values[joint]
            self._joint_edits[joint].setValue(int(raw))

    def set_joint_pwm(self, joint: int, value: int) -> None:
        self._joint_edits[int(joint)].setValue(int(value))

    def joint_pwm(self, joint: int) -> int:
        return self._joint_edits[int(joint)].value()

    def set_baseline_info(self, *, sha256: str, direct_count: int) -> None:
        self.baseline_label.setText(
            f"Baseline: 225 ABOVE · {direct_count} direct · SHA {sha256[:12]}…"
        )

    def set_session_info(
        self,
        *,
        session_id: str | None,
        mode: str | None = None,
        anchor_count: int = 0,
        revision: int = 0,
        stale: bool = True,
        missing: tuple[str, ...] = (),
    ) -> None:
        if not session_id:
            self.session_label.setText("Session: not created")
            self.runtime_calibration_label.setText("Calibration: BASELINE ONLY")
            return
        state = "STALE" if stale else f"candidate r{revision}"
        self.session_label.setText(
            f"Session: {session_id} · {mode} · anchors={anchor_count} · {state}"
        )
        self.workflow_label.setText(
            "还需标定: " + ", ".join(missing) if missing else "Required anchors ready"
        )
        self.runtime_calibration_label.setText(
            f"Calibration: {'NEEDS RECALCULATE' if stale else 'CANDIDATE LOADED'}"
        )

    def set_live_status(self, text: str) -> None:
        self.live_status_label.setText(str(text))
        color = "#167040" if str(text).startswith("LIVE HARDWARE") else "#a05a00"
        self.live_status_label.setStyleSheet(f"font-weight: 700; color: {color};")

    def set_runtime_status(self, *, robot: str | None = None, camera: str | None = None) -> None:
        if robot is not None:
            self.runtime_robot_label.setText(f"Robot: {robot}")
        if camera is not None:
            self.runtime_camera_label.setText(f"Camera: {camera}")

    def set_point_record(self, record: Mapping[str, Any] | None) -> None:
        if not record:
            self.point_source_label.setText("BASELINE preview")
            return
        source = str(record.get("source", "UNSET"))
        verified = bool(record.get("verified", False))
        display_source = "AUTO (INTERPOLATED)" if source == "INTERPOLATED" else source
        self.point_source_label.setText("VERIFIED" if verified else display_source)

    def update_grid(self, records: Mapping[str, Mapping[str, Any]]) -> None:
        colors = {
            "DIRECT": "#93c5fd",
            "INTERPOLATED": "#fde68a",
            "AUTO": "#fde68a",
            "VERIFIED": "#86efac",
            "BASELINE": "#d9dde3",
            "ERROR": "#fca5a5",
            "UNSET": "#eeeeee",
        }
        for (row, col), button in self._grid_buttons.items():
            key = f"P{row * 15 + col:03d}"
            record = records.get(key)
            source = "UNSET" if record is None else str(record.get("source", "UNSET"))
            if record is not None and record.get("verified"):
                source = "VERIFIED"
            display_source = "AUTO (INTERPOLATED)" if source == "INTERPOLATED" else source
            color = colors.get(source, colors["ERROR"])
            button.setProperty("source_color", color)
            button.setToolTip(f"{point_label(row, col)} · {display_source}")
        self._refresh_selected_border()

    def set_busy(self, busy: bool) -> None:
        for button in (
            self.move_button,
            self.verify_move_button,
            self.save_anchor_button,
            self.recalculate_button,
            self.verify_button,
            self.commit_button,
            self.rollback_button,
            self.apply_all_button,
            *self._joint_apply_buttons.values(),
        ):
            button.setEnabled(not busy)
