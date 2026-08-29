from __future__ import annotations

from typing import Any, Mapping

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)


class PointMoveLPanel(QWidget):
    """Small absolute-PWM editor for one arbitrary board point."""

    load_requested = Signal(int, int)
    move_above_requested = Signal(str)
    move_drop_requested = Signal(str, object)
    save_requested = Signal(str, object)
    confirm_requested = Signal(str)
    recalibrate_requested = Signal(str)
    return_above_requested = Signal(str)
    back_requested = Signal()
    emergency_stop_requested = Signal()

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._point_id = "P03_03"
        self._selection_dirty = False
        self._above_pwm: dict[str, int] = {}
        self._saved_record: dict[str, Any] | None = None
        self._recalibration_unlocked = False
        self._drop_fields: dict[str, QSpinBox] = {}
        self._adjust_buttons: list[QPushButton] = []
        self._build_ui()

    @property
    def point_id(self) -> str:
        return self._point_id

    @property
    def above_pwm(self) -> dict[str, int]:
        return dict(self._above_pwm)

    @property
    def saved_record(self) -> dict[str, Any] | None:
        return None if self._saved_record is None else dict(self._saved_record)

    @property
    def recalibration_unlocked(self) -> bool:
        return self._recalibration_unlocked

    def drop_pwm(self) -> dict[str, int]:
        return {joint: field.value() for joint, field in self._drop_fields.items()}

    @staticmethod
    def canonical_point_id(row: int, col: int) -> str:
        row_value, col_value = int(row), int(col)
        if not 0 <= row_value < 15 or not 0 <= col_value < 15:
            raise ValueError("board row/col must be within 0..14")
        return f"P{row_value:02d}_{col_value:02d}"

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)
        title = QLabel("MANUAL MOVEL CALIBRATION V1")
        title.setStyleSheet("font-size:22px;font-weight:900;color:#174a73;")
        root.addWidget(title)
        scope = QLabel(
            "One selected point only · ABOVE → FINAL DROP → ABOVE · "
            "no interpolation or 225-point generation"
        )
        scope.setWordWrap(True)
        root.addWidget(scope)

        selector = QGroupBox("Point")
        selector_layout = QHBoxLayout(selector)
        selector_layout.addWidget(QLabel("Row"))
        self.row_field = QSpinBox()
        self.row_field.setRange(0, 14)
        self.row_field.setValue(3)
        selector_layout.addWidget(self.row_field)
        selector_layout.addWidget(QLabel("Col"))
        self.col_field = QSpinBox()
        self.col_field.setRange(0, 14)
        self.col_field.setValue(3)
        selector_layout.addWidget(self.col_field)
        self.resolved_label = QLabel("Resolved: P03_03")
        self.resolved_label.setStyleSheet("font-weight:800;")
        selector_layout.addWidget(self.resolved_label, 1)
        self.load_button = QPushButton("Load Point")
        selector_layout.addWidget(self.load_button)
        root.addWidget(selector)

        above_group = QGroupBox("ABOVE · current project source · read only")
        above_form = QFormLayout(above_group)
        self._above_labels: dict[str, QLabel] = {}
        for joint in range(5):
            key = f"J{joint}"
            value = QLabel("—")
            value.setStyleSheet("font-weight:800;")
            self._above_labels[key] = value
            above_form.addRow(key, value)
        root.addWidget(above_group)

        drop_group = QGroupBox("DROP · absolute PWM sent to J0–J4")
        grid = QGridLayout(drop_group)
        headers = ("Joint", "-50", "-20", "-10", "Direct input", "+10", "+20", "+50")
        for column, text in enumerate(headers):
            grid.addWidget(QLabel(text), 0, column)
        for row, joint in enumerate(range(5), start=1):
            key = f"J{joint}"
            grid.addWidget(QLabel(key), row, 0)
            field = QSpinBox()
            field.setRange(550, 2450)
            field.setKeyboardTracking(False)
            self._drop_fields[key] = field
            for column, delta in enumerate((-50, -20, -10), start=1):
                button = QPushButton(str(delta))
                button.clicked.connect(
                    lambda _checked=False, value=delta, target=field: target.setValue(
                        target.value() + value
                    )
                )
                grid.addWidget(button, row, column)
                self._adjust_buttons.append(button)
            grid.addWidget(field, row, 4)
            for column, delta in enumerate((10, 20, 50), start=5):
                button = QPushButton(f"+{delta}")
                button.clicked.connect(
                    lambda _checked=False, value=delta, target=field: target.setValue(
                        target.value() + value
                    )
                )
                grid.addWidget(button, row, column)
                self._adjust_buttons.append(button)
        j5 = QLabel("J5 · PUMP · excluded from Point MoveL pose calibration")
        j5.setStyleSheet("font-weight:800;color:#875000;padding:8px;")
        grid.addWidget(j5, 6, 0, 1, 8)
        root.addWidget(drop_group)

        self.guess_label = QLabel("UNVERIFIED INITIAL GUESS · source: p77_delta_v1")
        self.guess_label.setWordWrap(True)
        self.guess_label.setStyleSheet("font-weight:800;color:#9a5700;")
        root.addWidget(self.guess_label)

        actions = QGridLayout()
        self.move_above_button = QPushButton("Move ABOVE")
        self.move_drop_button = QPushButton("Move Current DROP")
        self.save_button = QPushButton("Save Current DROP")
        self.confirm_button = QPushButton("Confirm Hardware")
        self.recalibrate_button = QPushButton("Recalibrate This Point")
        self.return_above_button = QPushButton("Return ABOVE")
        self.back_button = QPushButton("Back to Home")
        self.emergency_button = QPushButton("Emergency Stop")
        self.emergency_button.setStyleSheet(
            "background:#b3261e;color:white;font-weight:900;padding:9px;"
        )
        actions.addWidget(self.move_above_button, 0, 0)
        actions.addWidget(self.move_drop_button, 0, 1)
        actions.addWidget(self.save_button, 1, 0)
        actions.addWidget(self.confirm_button, 1, 1)
        actions.addWidget(self.return_above_button, 2, 0)
        actions.addWidget(self.back_button, 2, 1)
        actions.addWidget(self.recalibrate_button, 3, 0, 1, 2)
        actions.addWidget(self.emergency_button, 4, 0, 1, 2)
        root.addLayout(actions)

        self.status_label = QLabel("Load a point to begin.")
        self.status_label.setWordWrap(True)
        self.status_label.setStyleSheet("padding:8px;background:#f1f5f8;border-radius:5px;")
        root.addWidget(self.status_label)
        root.addStretch(1)

        self.row_field.valueChanged.connect(self._selection_changed)
        self.col_field.valueChanged.connect(self._selection_changed)
        self.load_button.clicked.connect(
            lambda _checked=False: self.load_requested.emit(
                self.row_field.value(), self.col_field.value()
            )
        )
        self.move_above_button.clicked.connect(
            lambda _checked=False: self.move_above_requested.emit(self.point_id)
        )
        self.move_drop_button.clicked.connect(
            lambda _checked=False: self.move_drop_requested.emit(
                self.point_id, self.drop_pwm()
            )
        )
        self.save_button.clicked.connect(
            lambda _checked=False: self.save_requested.emit(
                self.point_id, self.drop_pwm()
            )
        )
        self.confirm_button.clicked.connect(
            lambda _checked=False: self.confirm_requested.emit(self.point_id)
        )
        self.recalibrate_button.clicked.connect(
            lambda _checked=False: self.recalibrate_requested.emit(self.point_id)
        )
        self.return_above_button.clicked.connect(
            lambda _checked=False: self.return_above_requested.emit(self.point_id)
        )
        self.back_button.clicked.connect(self.back_requested.emit)
        self.emergency_button.clicked.connect(self.emergency_stop_requested.emit)

    def _selection_changed(self) -> None:
        selected = self.canonical_point_id(
            self.row_field.value(), self.col_field.value()
        )
        self._selection_dirty = selected != self._point_id
        self.resolved_label.setText(f"Resolved: {selected}")
        if self._selection_dirty:
            self.status_label.setText(f"{selected} selected · click Load Point")

    def set_point(
        self,
        *,
        point_id: str,
        board: tuple[int, int],
        above_pwm: Mapping[str, int],
        drop_pwm: Mapping[str, int],
        record: Mapping[str, Any] | None,
        prediction_source: str,
    ) -> None:
        self._point_id = str(point_id)
        self._selection_dirty = False
        self._recalibration_unlocked = False
        self._above_pwm = {f"J{joint}": int(above_pwm[f"J{joint}"]) for joint in range(5)}
        self._saved_record = None if record is None else dict(record)
        blocked = self.row_field.blockSignals(True)
        self.row_field.setValue(int(board[0]))
        self.row_field.blockSignals(blocked)
        blocked = self.col_field.blockSignals(True)
        self.col_field.setValue(int(board[1]))
        self.col_field.blockSignals(blocked)
        self.resolved_label.setText(f"Resolved: {self._point_id}")
        for joint in range(5):
            key = f"J{joint}"
            self._above_labels[key].setText(str(self._above_pwm[key]))
            self._drop_fields[key].setValue(int(drop_pwm[key]))

        if record is None:
            self.guess_label.setText(
                f"UNVERIFIED INITIAL GUESS · source: {prediction_source}"
            )
            self.status_label.setText(
                f"{self._point_id} · not saved · hardware_verified=false"
            )
        else:
            level = "HARDWARE VERIFIED" if record.get("hardware_verified") else "NOT VERIFIED"
            self.guess_label.setText(
                f"Saved DROP · prediction source: {record.get('prediction_source', prediction_source)}"
            )
            self.status_label.setText(
                f"{self._point_id} · operator_confirmed="
                f"{str(bool(record.get('operator_confirmed'))).lower()} · {level}"
            )

        protected = bool(record and record.get("hardware_verified"))
        for field in self._drop_fields.values():
            field.setEnabled(not protected)
        for button in self._adjust_buttons:
            button.setEnabled(not protected)
        if protected:
            self.guess_label.setText("HARDWARE VERIFIED · protected from overwrite")

    def set_recalibration_unlocked(self, enabled: bool) -> None:
        verified = bool(
            self._saved_record and self._saved_record.get("hardware_verified")
        )
        self._recalibration_unlocked = bool(enabled and verified)
        editing_enabled = bool(not verified or self._recalibration_unlocked)
        for field in self._drop_fields.values():
            field.setEnabled(editing_enabled)
        for button in self._adjust_buttons:
            button.setEnabled(editing_enabled)
        if self._recalibration_unlocked:
            self.guess_label.setText(
                "RECALIBRATION DRAFT · saved HARDWARE VERIFIED data unchanged until Save"
            )
            self.status_label.setText(
                f"{self._point_id} · edit and test this draft; "
                "Save will require hardware reconfirmation"
            )

    def set_controls(
        self,
        *,
        connected: bool,
        busy: bool,
        estop: bool,
        dry_run: bool,
        pose_state: str,
        active_point: str | None,
    ) -> None:
        ready = bool(connected and not busy and not estop)
        same_point = active_point == self.point_id
        verified = bool(
            self._saved_record and self._saved_record.get("hardware_verified")
        )
        editing_enabled = bool(not verified or self._recalibration_unlocked)
        self.load_button.setEnabled(not busy and pose_state == "SAFE")
        selection_ready = not self._selection_dirty
        self.row_field.setEnabled(not busy and pose_state == "SAFE")
        self.col_field.setEnabled(not busy and pose_state == "SAFE")
        self.move_above_button.setEnabled(
            ready
            and selection_ready
            and pose_state in {"SAFE", "ABOVE"}
            and (pose_state == "SAFE" or same_point)
        )
        self.move_drop_button.setEnabled(
            ready
            and selection_ready
            and same_point
            and pose_state == "ABOVE"
        )
        self.save_button.setEnabled(
            not busy
            and not estop
            and editing_enabled
            and same_point
            and pose_state == "DROP"
        )
        saved_matches = bool(
            self._saved_record
            and self._saved_record.get("drop_pwm") == self.drop_pwm()
        )
        self.confirm_button.setEnabled(
            ready
            and not dry_run
            and not verified
            and same_point
            and pose_state == "DROP"
            and saved_matches
        )
        self.return_above_button.setEnabled(
            ready and same_point and pose_state == "DROP"
        )
        self.recalibrate_button.setEnabled(
            verified
            and not self._recalibration_unlocked
            and self.point_id != "P07_07"
            and not self._selection_dirty
            and not busy
            and not estop
        )
        self.back_button.setEnabled(not busy)
        self.emergency_button.setEnabled(bool(connected and not estop))
