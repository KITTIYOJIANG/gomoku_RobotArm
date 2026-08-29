from __future__ import annotations

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QComboBox,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from app.stage6.models import DESCENT_LEVELS, SPATIAL_KEYS, DescentLevel


class Stage6Panel(QWidget):
    generate_current_requested = Signal(int, int)
    generate_all_requested = Signal()
    preview_requested = Signal(int, int)
    test_level_requested = Signal(int, int, str)
    test_next_requested = Signal(int, int)
    ascend_one_requested = Signal(int, int)
    return_above_requested = Signal(int, int)
    nudge_requested = Signal(int, int, str, int, int)
    save_delta_requested = Signal(int, int, str)
    verify_level_requested = Signal(int, int, str)
    verify_profile_requested = Signal(int, int)
    reset_delta_requested = Signal(int, int, str)
    undo_requested = Signal(int, int, str)
    estop_requested = Signal()
    pump_off_requested = Signal()
    overheat_requested = Signal()

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        root = QVBoxLayout(self)
        warning = QLabel(
            "候选生成/预览开放；当前版本 FORCE_DRY_RUN，未经实机验证不得带棋子。"
        )
        warning.setWordWrap(True)
        warning.setStyleSheet("color:#9a5a00;font-weight:600;")
        root.addWidget(warning)

        target_layout = QGridLayout()
        self.row_spin = QSpinBox()
        self.col_spin = QSpinBox()
        self.row_spin.setRange(0, 14)
        self.col_spin.setRange(0, 14)
        self.row_spin.setValue(7)
        self.col_spin.setValue(7)
        self.level_combo = QComboBox()
        for level in DESCENT_LEVELS:
            self.level_combo.addItem(level.value.upper(), level.value)
        self.lock_label = QLabel("ABOVE_OR_HIGH_UNLOCKED")
        self.status_label = QLabel("NOT_GENERATED")
        target_layout.addWidget(QLabel("当前目标 row"), 0, 0)
        target_layout.addWidget(self.row_spin, 0, 1)
        target_layout.addWidget(QLabel("col"), 0, 2)
        target_layout.addWidget(self.col_spin, 0, 3)
        target_layout.addWidget(QLabel("当前层"), 1, 0)
        target_layout.addWidget(self.level_combo, 1, 1, 1, 3)
        target_layout.addWidget(self.lock_label, 2, 0, 1, 4)
        target_layout.addWidget(self.status_label, 3, 0, 1, 4)
        root.addLayout(target_layout)

        values = QGridLayout()
        self.value_edits: dict[str, dict[str, QLineEdit]] = {}
        rows = (
            ("above", "已有 ABOVE PWM"),
            ("computed", "模型计算 PWM"),
            ("delta", "人工修正 ΔPWM"),
            ("final", "最终 PWM"),
        )
        values.addWidget(QLabel(""), 0, 0)
        for column, joint in enumerate(SPATIAL_KEYS, start=1):
            values.addWidget(QLabel(joint), 0, column)
        for row_index, (key, label) in enumerate(rows, start=1):
            values.addWidget(QLabel(label), row_index, 0)
            self.value_edits[key] = {}
            for column, joint in enumerate(SPATIAL_KEYS, start=1):
                edit = QLineEdit("-")
                edit.setReadOnly(True)
                edit.setMaximumWidth(64)
                values.addWidget(edit, row_index, column)
                self.value_edits[key][joint] = edit
        root.addLayout(values)

        generation = QGridLayout()
        self.generate_current_button = QPushButton("生成当前点下降轨迹")
        self.generate_all_button = QPushButton("生成全棋盘候选轨迹")
        self.preview_button = QPushButton("DRY RUN 预览")
        self.test_level_button = QPushButton("测试下降一层")
        self.test_next_button = QPushButton("测试到下一层")
        self.ascend_one_button = QPushButton("沿原路径抬升一层")
        self.return_above_button = QPushButton("沿原路径安全返回 ABOVE")
        for index, button in enumerate(
            (
                self.generate_current_button,
                self.generate_all_button,
                self.preview_button,
                self.test_level_button,
                self.test_next_button,
                self.ascend_one_button,
                self.return_above_button,
            )
        ):
            generation.addWidget(button, index // 2, index % 2)
        root.addLayout(generation)

        nudge = QGridLayout()
        for row_index, joint in enumerate(SPATIAL_KEYS):
            nudge.addWidget(QLabel(f"微调 {joint}"), row_index, 0)
            for column, amount in enumerate((-10, -5, -1, 1, 5, 10), start=1):
                button = QPushButton(f"{amount:+d}")
                button.setMaximumWidth(45)
                button.clicked.connect(
                    lambda _checked=False, j=joint, a=amount: self.nudge_requested.emit(
                        self.row_spin.value(),
                        self.col_spin.value(),
                        self.current_level().value,
                        int(j),
                        a,
                    )
                )
                nudge.addWidget(button, row_index, column)
        root.addLayout(nudge)

        actions = QGridLayout()
        self.save_button = QPushButton("保存当前层修正")
        self.verify_level_button = QPushButton("标记当前层通过")
        self.verify_profile_button = QPushButton("标记整个下降轨迹通过")
        self.reset_button = QPushButton("清零当前层修正")
        self.undo_button = QPushButton("恢复上一个版本")
        self.overheat_button = QPushButton("舵机过热（锁定新任务）")
        self.estop_button = QPushButton("急停")
        self.pump_off_button = QPushButton("气泵关闭")
        self.estop_button.setStyleSheet("background:#b3262e;color:white;font-weight:700;")
        for index, button in enumerate(
            (
                self.save_button,
                self.verify_level_button,
                self.verify_profile_button,
                self.reset_button,
                self.undo_button,
                self.overheat_button,
                self.estop_button,
                self.pump_off_button,
            )
        ):
            actions.addWidget(button, index // 2, index % 2)
        root.addLayout(actions)

        self.generate_current_button.clicked.connect(
            lambda _checked=False: self.generate_current_requested.emit(*self.target())
        )
        self.generate_all_button.clicked.connect(
            lambda _checked=False: self.generate_all_requested.emit()
        )
        self.preview_button.clicked.connect(
            lambda _checked=False: self.preview_requested.emit(*self.target())
        )
        self.test_level_button.clicked.connect(
            lambda _checked=False: self.test_level_requested.emit(
                *self.target(), self.current_level().value
            )
        )
        self.test_next_button.clicked.connect(
            lambda _checked=False: self.test_next_requested.emit(*self.target())
        )
        self.ascend_one_button.clicked.connect(
            lambda _checked=False: self.ascend_one_requested.emit(*self.target())
        )
        self.return_above_button.clicked.connect(
            lambda _checked=False: self.return_above_requested.emit(*self.target())
        )
        self.save_button.clicked.connect(
            lambda _checked=False: self.save_delta_requested.emit(
                *self.target(), self.current_level().value
            )
        )
        self.verify_level_button.clicked.connect(
            lambda _checked=False: self.verify_level_requested.emit(
                *self.target(), self.current_level().value
            )
        )
        self.verify_profile_button.clicked.connect(
            lambda _checked=False: self.verify_profile_requested.emit(*self.target())
        )
        self.reset_button.clicked.connect(
            lambda _checked=False: self.reset_delta_requested.emit(
                *self.target(), self.current_level().value
            )
        )
        self.undo_button.clicked.connect(
            lambda _checked=False: self.undo_requested.emit(
                *self.target(), self.current_level().value
            )
        )
        self.estop_button.clicked.connect(
            lambda _checked=False: self.estop_requested.emit()
        )
        self.pump_off_button.clicked.connect(
            lambda _checked=False: self.pump_off_requested.emit()
        )
        self.overheat_button.clicked.connect(
            lambda _checked=False: self.overheat_requested.emit()
        )

    def target(self) -> tuple[int, int]:
        return self.row_spin.value(), self.col_spin.value()

    def current_level(self) -> DescentLevel:
        return DescentLevel(self.level_combo.currentData())

    def set_profile_view(self, profile: dict) -> None:
        level = self.current_level().value
        levels = profile.get("levels") or {}
        item = levels.get(level) or {}
        above = levels.get(DescentLevel.ABOVE.value) or {}
        self._set_values("above", above.get("computed_pwm") or {})
        self._set_values("computed", item.get("computed_pwm") or {})
        self._set_values("delta", item.get("manual_delta_pwm") or {})
        self._set_values("final", item.get("final_pwm") or {})
        self.status_label.setText(
            f"{item.get('status', 'NOT_GENERATED')} | "
            f"{profile.get('verification_stage', 'COMPUTED')}"
        )

    def set_lock_state(self, locked: bool, label: str) -> None:
        self.row_spin.setEnabled(not locked)
        self.col_spin.setEnabled(not locked)
        self.lock_label.setText(label)
        self.lock_label.setStyleSheet(
            "color:#b3262e;font-weight:700;" if locked else ""
        )

    def _set_values(self, group: str, values: dict) -> None:
        for joint in SPATIAL_KEYS:
            self.value_edits[group][joint].setText(
                str(values[joint]) if joint in values else "-"
            )
