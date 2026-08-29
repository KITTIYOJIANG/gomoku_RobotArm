from __future__ import annotations

from typing import Any, Mapping

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from .manual_movel import FINAL_DROP_J1_REFERENCE_PWM, FINAL_DROP_STEP_INDEX


class P77ManualMoveLPanel(QWidget):
    """Absolute-PWM editor for the isolated P77 manual MoveL path."""

    move_requested = Signal(int, object)
    save_requested = Signal(int, object)
    confirm_requested = Signal(int)
    next_requested = Signal(int)
    previous_requested = Signal(int)
    return_previous_requested = Signal()
    return_above_requested = Signal()
    set_drop_requested = Signal(int)
    full_cycle_requested = Signal()
    back_requested = Signal()
    emergency_stop_requested = Signal()

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._step_index = 0
        self._drop_step: int | None = None
        self._fields: dict[str, QSpinBox] = {}
        self._adjust_buttons: list[QPushButton] = []
        self._build_ui()

    @property
    def step_index(self) -> int:
        return self._step_index

    def final_pwm(self) -> dict[str, int]:
        return {joint: field.value() for joint, field in self._fields.items()}

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)
        title = QLabel("P77 MANUAL MOVEL TUNER")
        title.setStyleSheet("font-size:22px;font-weight:900;color:#174a73;")
        root.addWidget(title)
        scope = QLabel(
            "P77 only · 3 manual descent steps · absolute final PWM · "
            "old automatic WP01–WP05 are reference only"
        )
        scope.setWordWrap(True)
        root.addWidget(scope)

        self.step_label = QLabel("Step 0")
        self.step_label.setStyleSheet("font-size:20px;font-weight:800;")
        root.addWidget(self.step_label)

        editor = QGroupBox("Absolute PWM sent to J0–J4")
        grid = QGridLayout(editor)
        grid.addWidget(QLabel("Joint"), 0, 0)
        grid.addWidget(QLabel("-50"), 0, 1)
        grid.addWidget(QLabel("-20"), 0, 2)
        grid.addWidget(QLabel("-10"), 0, 3)
        grid.addWidget(QLabel("Direct input"), 0, 4)
        grid.addWidget(QLabel("+10"), 0, 5)
        grid.addWidget(QLabel("+20"), 0, 6)
        grid.addWidget(QLabel("+50"), 0, 7)
        for row, joint in enumerate(range(5), start=1):
            key = f"J{joint}"
            grid.addWidget(QLabel(key), row, 0)
            field = QSpinBox()
            field.setRange(550, 2450)
            field.setKeyboardTracking(False)
            self._fields[key] = field
            for column, delta in enumerate((-50, -20, -10), start=1):
                button = QPushButton(str(delta))
                button.clicked.connect(
                    lambda _c=False, value=delta, target=field: target.setValue(
                        target.value() + value
                    )
                )
                grid.addWidget(button, row, column)
                self._adjust_buttons.append(button)
            grid.addWidget(field, row, 4)
            for column, delta in enumerate((10, 20, 50), start=5):
                button = QPushButton(f"+{delta}")
                button.clicked.connect(
                    lambda _c=False, value=delta, target=field: target.setValue(
                        target.value() + value
                    )
                )
                grid.addWidget(button, row, column)
                self._adjust_buttons.append(button)
        locked = QLabel("J5 · PUMP · LOCKED (not displayed, not sent, not saved in pose)")
        locked.setStyleSheet("font-weight:800;color:#875000;padding:8px;")
        grid.addWidget(locked, 6, 0, 1, 8)
        root.addWidget(editor)

        self.suggestion_label = QLabel(
            "Initial direction suggestion only: J1/J2 small decrease, J3 small increase. "
            "No fixed ratio is assumed."
        )
        self.suggestion_label.setWordWrap(True)
        root.addWidget(self.suggestion_label)

        primary = QGridLayout()
        self.move_button = QPushButton("Move Current Step")
        self.save_button = QPushButton("Save Current Step")
        self.confirm_button = QPushButton("Confirm Step")
        self.next_button = QPushButton("Next Step")
        self.previous_button = QPushButton("Previous Step")
        self.return_previous_button = QPushButton("Return Previous Step")
        self.return_above_button = QPushButton("Return ABOVE")
        self.set_drop_button = QPushButton("Set As FINAL DROP")
        self.full_cycle_button = QPushButton("一键取料 → P77 下棋")
        self.back_button = QPushButton("Back to Home")
        self.emergency_button = QPushButton("Emergency Stop")
        self.emergency_button.setStyleSheet(
            "background:#b3261e;color:white;font-weight:900;padding:9px;"
        )
        for button in (self.move_button, self.save_button, self.confirm_button):
            button.setStyleSheet("font-weight:800;padding:8px;")
        primary.addWidget(self.move_button, 0, 0)
        primary.addWidget(self.save_button, 0, 1)
        primary.addWidget(self.confirm_button, 0, 2)
        primary.addWidget(self.previous_button, 1, 0)
        primary.addWidget(self.next_button, 1, 1)
        primary.addWidget(self.back_button, 1, 2)
        primary.addWidget(self.return_previous_button, 2, 0)
        primary.addWidget(self.return_above_button, 2, 1)
        primary.addWidget(self.set_drop_button, 3, 0, 1, 2)
        primary.addWidget(self.emergency_button, 3, 2)
        primary.addWidget(self.full_cycle_button, 4, 0, 1, 3)
        root.addLayout(primary)

        self.status_label = QLabel("Step0 is immutable P77 Golden ABOVE.")
        self.status_label.setWordWrap(True)
        self.status_label.setStyleSheet("padding:8px;background:#f1f5f8;border-radius:5px;")
        root.addWidget(self.status_label)
        root.addStretch(1)

        self.move_button.clicked.connect(
            lambda _c=False: self.move_requested.emit(self.step_index, self.final_pwm())
        )
        self.save_button.clicked.connect(
            lambda _c=False: self.save_requested.emit(self.step_index, self.final_pwm())
        )
        self.confirm_button.clicked.connect(
            lambda _c=False: self.confirm_requested.emit(self.step_index)
        )
        self.next_button.clicked.connect(
            lambda _c=False: self.next_requested.emit(self.step_index)
        )
        self.previous_button.clicked.connect(
            lambda _c=False: self.previous_requested.emit(self.step_index)
        )
        self.return_previous_button.clicked.connect(self.return_previous_requested.emit)
        self.return_above_button.clicked.connect(self.return_above_requested.emit)
        self.set_drop_button.clicked.connect(
            lambda _c=False: self.set_drop_requested.emit(self.step_index)
        )
        self.full_cycle_button.clicked.connect(self.full_cycle_requested.emit)
        self.back_button.clicked.connect(self.back_requested.emit)
        self.emergency_button.clicked.connect(self.emergency_stop_requested.emit)

    def set_step(self, record: Mapping[str, Any], *, drop_step: int | None = None) -> None:
        self._step_index = int(record["step_index"])
        self._drop_step = None if drop_step is None else int(drop_step)
        self.step_label.setText(
            f"Step {FINAL_DROP_STEP_INDEX} / FINAL DROP"
            if self._step_index == FINAL_DROP_STEP_INDEX
            else f"Step {self._step_index}"
        )
        values = record["final_pwm"]
        for joint, field in self._fields.items():
            field.setValue(int(values[joint]))
        immutable = self._step_index == 0
        for field in self._fields.values():
            field.setEnabled(not immutable)
        for button in self._adjust_buttons:
            button.setEnabled(not immutable)
        if self._step_index == 0:
            self.suggestion_label.setText("Step0 is immutable P77 Golden ABOVE.")
        elif self._step_index == FINAL_DROP_STEP_INDEX:
            self.suggestion_label.setText(
                "FINAL DROP reference only: P77 J1 ≈ "
                f"{FINAL_DROP_J1_REFERENCE_PWM}. Suggested direction: "
                "J1/J2 decrease, J3 increase. All J0–J4 remain freely editable."
            )
        else:
            self.suggestion_label.setText(
                "Suggestion only: J1/J2 small decrease, J3 small increase. "
                "All J0–J4 remain freely editable; no direction is enforced."
            )
        status = "operator_confirmed=true" if record.get("operator_confirmed") else "operator_confirmed=false"
        drop = f" · P77 DROP=Step{drop_step}" if drop_step is not None else ""
        self.status_label.setText(
            f"Step{self._step_index} · {status} · hardware_verified=false{drop}"
        )

    def set_controls(
        self,
        *,
        connected: bool,
        busy: bool,
        estop: bool,
        pose_index: int | None,
        record: Mapping[str, Any],
        step_count: int,
    ) -> None:
        ready = bool(connected and not busy and not estop)
        index = self.step_index
        adjacent_forward = (
            (index == 0 and pose_index is None)
            or pose_index == index
            or (pose_index is not None and index == pose_index + 1)
        )
        self.move_button.setEnabled(ready and adjacent_forward)
        self.save_button.setEnabled(not busy and not estop)
        self.confirm_button.setEnabled(
            ready and pose_index == index and record.get("final_pwm") == self.final_pwm()
        )
        self.previous_button.setEnabled(not busy and index > 0)
        self.next_button.setEnabled(
            not busy
            and bool(record.get("operator_confirmed"))
            and index < FINAL_DROP_STEP_INDEX
            and index <= int(step_count) - 1
        )
        self.return_previous_button.setEnabled(
            ready and pose_index is not None and pose_index > 0
        )
        self.return_above_button.setEnabled(
            ready and pose_index is not None and pose_index > 0
        )
        self.set_drop_button.setEnabled(
            not busy
            and index == FINAL_DROP_STEP_INDEX
            and pose_index == index
            and bool(record.get("operator_confirmed"))
        )
        self.full_cycle_button.setEnabled(
            ready and pose_index in {None, 0} and self._drop_step is not None
        )
        self.back_button.setEnabled(not busy)
        self.emergency_button.setEnabled(bool(connected and not estop))
