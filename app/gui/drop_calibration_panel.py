from __future__ import annotations

from typing import Any, Mapping

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QComboBox,
    QPushButton,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from app.integrated_v1.golden import SPATIAL_KEYS
from app.integrated_v1.points import PointRef, format_point_id


STATUS_COLORS = {
    "NOT_GENERATED": "#e5e7eb",
    "GENERATED": "#dbeafe",
    "PENDING_VERIFY": "#fde68a",
    "VERIFIED": "#86efac",
    "MOVE_L_UNREACHABLE": "#fca5a5",
    "MANUAL_CORRECTED": "#c4b5fd",
    "INVALID": "#ef4444",
    "GOLDEN": "#f59e0b",
}


class DropCalibrationPanel(QWidget):
    """15x15 DROP verification UI. It emits intents and owns no controller."""

    point_selected = Signal(str)
    generate_point_requested = Signal(str)
    generate_all_requested = Signal()
    move_above_requested = Signal(str)
    preview_requested = Signal(str)
    move_drop_requested = Signal(str)
    retract_requested = Signal(str)
    full_place_requested = Signal(str)
    save_correction_requested = Signal(str, object)
    reset_correction_requested = Signal(str)
    verify_requested = Signal(str)
    save_profile_requested = Signal()
    promote_profile_requested = Signal()
    export_golden_baseline_requested = Signal()
    fast_anchor_saved = Signal(str, str, object)
    fast_apply_requested = Signal(str)
    direct_anchor_requested = Signal(str, object)
    emergency_stop_requested = Signal()

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._point = PointRef(7, 7)
        self._statuses: dict[str, str] = {}
        self._buttons: dict[str, QPushButton] = {}
        self._corrections: dict[str, QSpinBox] = {}
        self._above_edits: dict[str, QSpinBox] = {}
        self._build_ui()
        self.select_point(self._point.point_id, emit=False)

    @property
    def current_point_id(self) -> str:
        return self._point.point_id

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(4, 6, 4, 4)
        root.setSpacing(7)
        title_row = QHBoxLayout()
        title = QLabel("DROP Calibration · MoveL")
        title.setStyleSheet("font-size:18px;font-weight:700;")
        self.current_label = QLabel(self.current_point_id)
        self.current_label.setStyleSheet("font-size:18px;font-weight:700;color:#0f4c5c;")
        title_row.addWidget(title)
        title_row.addStretch(1)
        title_row.addWidget(self.current_label)
        root.addLayout(title_row)

        self.stats_label = QLabel("Total 225 · Generated 0 · Verified 0 · Pending 0 · Unreachable 0 · Invalid 0 · Manual 0 · Golden 5")
        self.stats_label.setWordWrap(True)
        root.addWidget(self.stats_label)
        legend = QLabel(
            "灰 未生成  ·  蓝 已生成  ·  黄 待验证  ·  绿 已验证  ·  红 不可达/错误  ·  紫 人工修正  ·  金 Golden"
        )
        legend.setWordWrap(True)
        root.addWidget(legend)

        matrix = QGroupBox("15×15 DROP 状态")
        grid = QGridLayout(matrix)
        grid.setHorizontalSpacing(1)
        grid.setVerticalSpacing(2)
        grid.setAlignment(Qt.AlignmentFlag.AlignLeft)
        for row in range(15):
            for col in range(15):
                point_id = format_point_id(row, col)
                button = QPushButton(f"{row * 15 + col:03d}")
                button.setFixedHeight(22)
                button.setFixedWidth(34)
                button.setToolTip(f"{point_id} ({row},{col})")
                button.clicked.connect(
                    lambda _checked=False, pid=point_id: self.select_point(pid)
                )
                grid.addWidget(button, row, col)
                self._buttons[point_id] = button
        root.addWidget(matrix)

        navigation = QHBoxLayout()
        previous = QPushButton("Previous")
        next_button = QPushButton("Next")
        next_pending = QPushButton("Next Pending")
        previous.clicked.connect(lambda _c=False: self._step(-1))
        next_button.clicked.connect(lambda _c=False: self._step(1))
        next_pending.clicked.connect(lambda _c=False: self._next_pending())
        navigation.addWidget(previous)
        navigation.addWidget(next_button)
        navigation.addWidget(next_pending)
        root.addLayout(navigation)

        details = QGroupBox("Selected Point")
        details_layout = QGridLayout(details)
        self.above_source = QLabel("ABOVE source: -")
        self.above_flags = QLabel("ABOVE verification: -")
        self.above_pwm = QLabel("ABOVE PWM: -")
        self.drop_auto = QLabel("Auto DROP: -")
        self.drop_final = QLabel("Final DROP: -")
        self.drop_status = QLabel("MoveL status: NOT_GENERATED")
        self.drop_reason = QLabel("Reason: -")
        self.drop_reason.setWordWrap(True)
        for row, widget in enumerate(
            (
                self.above_source,
                self.above_flags,
                self.above_pwm,
                self.drop_auto,
                self.drop_final,
                self.drop_status,
                self.drop_reason,
            )
        ):
            details_layout.addWidget(widget, row, 0, 1, 6)
        details_layout.addWidget(QLabel("DROP correction"), 7, 0)
        for index, joint in enumerate(SPATIAL_KEYS):
            spin = QSpinBox()
            spin.setRange(-500, 500)
            spin.setPrefix(f"J{index} ")
            spin.setToolTip("drop_final_pwm = drop_auto_pwm + this correction")
            details_layout.addWidget(spin, 7, index + 1)
            self._corrections[joint] = spin
        step_row = QHBoxLayout()
        self.correction_joint_combo = QComboBox()
        for index, joint in enumerate(SPATIAL_KEYS):
            self.correction_joint_combo.addItem(f"J{index}", joint)
        step_row.addWidget(self.correction_joint_combo)
        for delta in (-50, -10, -1, 1, 10, 50):
            button = QPushButton(f"{delta:+d}")
            button.clicked.connect(
                lambda _c=False, amount=delta: self._nudge_selected_correction(amount)
            )
            step_row.addWidget(button)
        details_layout.addLayout(step_row, 8, 0, 1, 6)
        details_layout.addWidget(QLabel("ABOVE PWM editor"), 9, 0)
        for index, joint in enumerate(SPATIAL_KEYS):
            spin = QSpinBox()
            spin.setRange(550, 2450)
            spin.setPrefix(f"J{index} ")
            details_layout.addWidget(spin, 9, index + 1)
            self._above_edits[joint] = spin
        root.addWidget(details)

        fast = QGroupBox("5/9 Anchor Fast Calibration · ABOVE correction field")
        fast_layout = QGridLayout(fast)
        self.fast_mode = QComboBox()
        self.fast_mode.setSizeAdjustPolicy(
            QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon
        )
        self.fast_mode.setMinimumContentsLength(12)
        self.fast_mode.addItem("5 Point · P33 P3_11 P77 P11_3 P11_11", "FAST_5")
        self.fast_mode.addItem("9 Point · P00 P07 P0_14 / P70 P77 P7_14 / P14_0 P14_7 P14_14", "FAST_9")
        self.fast_progress = QLabel("Captured: 0")
        capture = QPushButton("Capture Current ABOVE as Fast Anchor")
        apply_fast = QPushButton("Apply Fast Calibration")
        save_direct = QPushButton("Save as Direct Anchor")
        fast_layout.addWidget(self.fast_mode, 0, 0, 1, 3)
        fast_layout.addWidget(self.fast_progress, 1, 0, 1, 3)
        fast_layout.addWidget(capture, 2, 0)
        fast_layout.addWidget(apply_fast, 2, 1)
        fast_layout.addWidget(save_direct, 2, 2)
        capture.clicked.connect(lambda _c=False: self.fast_anchor_saved.emit(
            str(self.fast_mode.currentData()), self.current_point_id, self.above_pwm_values()
        ))
        apply_fast.clicked.connect(
            lambda _c=False: self.fast_apply_requested.emit(str(self.fast_mode.currentData()))
        )
        save_direct.clicked.connect(
            lambda _c=False: self.direct_anchor_requested.emit(
                self.current_point_id, self.above_pwm_values()
            )
        )
        root.addWidget(fast)

        actions = QGroupBox("Safe Workflow")
        action_grid = QGridLayout(actions)
        labels = (
            ("Generate DROP", lambda: self.generate_point_requested.emit(self.current_point_id)),
            ("Move ABOVE", lambda: self.move_above_requested.emit(self.current_point_id)),
            ("Preview MoveL", lambda: self.preview_requested.emit(self.current_point_id)),
            ("Move DROP", lambda: self.move_drop_requested.emit(self.current_point_id)),
            ("Retract", lambda: self.retract_requested.emit(self.current_point_id)),
            ("Test Full Place", lambda: self.full_place_requested.emit(self.current_point_id)),
            ("Save Correction", self._save_correction),
            ("Reset Correction", lambda: self.reset_correction_requested.emit(self.current_point_id)),
            ("Mark Verified", lambda: self.verify_requested.emit(self.current_point_id)),
        )
        for index, (label, callback) in enumerate(labels):
            button = QPushButton(label)
            button.clicked.connect(lambda _c=False, fn=callback: fn())
            action_grid.addWidget(button, index // 3, index % 3)
        root.addWidget(actions)

        batch = QGridLayout()
        self.generate_all_button = QPushButton("Generate All DROP · OFFLINE ONLY")
        self.save_profile_button = QPushButton("Save Profile")
        self.promote_profile_button = QPushButton("Set Calibration Ready")
        self.golden_baseline_button = QPushButton("Save Golden Baseline")
        stop = QPushButton("Emergency Stop")
        stop.setStyleSheet("background:#b3262e;color:white;font-weight:700;")
        self.generate_all_button.clicked.connect(lambda _c=False: self.generate_all_requested.emit())
        self.save_profile_button.clicked.connect(lambda _c=False: self.save_profile_requested.emit())
        self.promote_profile_button.clicked.connect(lambda _c=False: self.promote_profile_requested.emit())
        self.golden_baseline_button.clicked.connect(
            lambda _c=False: self.export_golden_baseline_requested.emit()
        )
        stop.clicked.connect(lambda _c=False: self.emergency_stop_requested.emit())
        batch.addWidget(self.generate_all_button, 0, 0, 1, 3)
        batch.addWidget(self.save_profile_button, 1, 0)
        batch.addWidget(self.promote_profile_button, 1, 1)
        batch.addWidget(self.golden_baseline_button, 2, 0, 1, 2)
        batch.addWidget(stop, 1, 2, 2, 1)
        root.addLayout(batch)

    def select_point(self, point_id: str, *, emit: bool = True) -> None:
        from app.integrated_v1.points import parse_point_id

        point = parse_point_id(point_id)
        self._point = point
        self.current_label.setText(f"{point.point_id} ({point.row},{point.col})")
        for key, button in self._buttons.items():
            self._apply_button_style(key, button, selected=key == point.point_id)
        if emit:
            self.point_selected.emit(point.point_id)

    def set_point_data(
        self, above: Mapping[str, Any], drop: Mapping[str, Any] | None
    ) -> None:
        source = str(above.get("source", "-"))
        protected = bool(above.get("protected"))
        verified = bool(above.get("verified"))
        verification_level = str(above.get("verification_level", "NOT VERIFIED"))
        flags = []
        if source == "golden_direct_anchor":
            flags.append("✓ Golden Anchor")
        if verified:
            flags.append(f"✓ {verification_level}")
        if protected:
            flags.append("✓ Protected")
        display_source = (
            "User Direct Calibration (golden_direct_anchor)"
            if source == "golden_direct_anchor"
            else source
        )
        self.above_source.setText(f"ABOVE source: {display_source}")
        self.above_flags.setText(" · ".join(flags) if flags else "ABOVE verification: NOT VERIFIED")
        self.above_pwm.setText(f"ABOVE PWM: {self._pwm_text(above.get('final_above_pwm'))}")
        final_above = above.get("final_above_pwm") or {}
        for joint, spin in self._above_edits.items():
            spin.setValue(int(final_above.get(joint, 1500)))
        if drop is None:
            self.drop_auto.setText("Auto DROP: -")
            self.drop_final.setText("Final DROP: -")
            self.drop_status.setText("MoveL status: NOT_GENERATED")
            self.drop_reason.setText("Reason: -")
            for spin in self._corrections.values():
                spin.setValue(0)
            return
        self.drop_auto.setText(f"Auto DROP: {self._pwm_text(drop.get('drop_auto_pwm'))}")
        self.drop_final.setText(f"Final DROP: {self._pwm_text(drop.get('drop_final_pwm'))}")
        self.drop_status.setText(
            f"MoveL status: {drop.get('status')} · max safe {drop.get('max_safe_descent_mm', 0)} mm"
        )
        self.drop_reason.setText(f"Reason: {drop.get('reason') or '-'}")
        correction = drop.get("drop_correction_pwm") or {}
        for joint, spin in self._corrections.items():
            spin.setValue(int(correction.get(joint, 0)))

    def set_statuses(
        self,
        statuses: Mapping[str, str],
        *,
        golden_ids: set[str] | None = None,
    ) -> None:
        self._statuses = {str(key): str(value) for key, value in statuses.items()}
        goldens = golden_ids or set()
        for key, button in self._buttons.items():
            if key in goldens and self._statuses.get(key) in {None, "NOT_GENERATED"}:
                self._statuses[key] = "GOLDEN"
            self._apply_button_style(key, button, selected=key == self.current_point_id)

    def set_statistics(self, stats: Mapping[str, int]) -> None:
        order = ("Total", "Generated", "Verified", "Pending", "Unreachable", "Invalid", "Manual", "Golden")
        self.stats_label.setText(" · ".join(f"{key} {int(stats.get(key, 0))}" for key in order))

    def set_fast_progress(self, captured: int, required: int) -> None:
        self.fast_progress.setText(f"Captured: {int(captured)} / {int(required)}")

    def above_pwm_values(self) -> dict[str, int]:
        return {joint: spin.value() for joint, spin in self._above_edits.items()}

    def _apply_button_style(self, key: str, button: QPushButton, *, selected: bool) -> None:
        status = self._statuses.get(key, "NOT_GENERATED")
        color = STATUS_COLORS.get(status, STATUS_COLORS["INVALID"])
        border = "3px solid #0f4c5c" if selected else "1px solid #94a3b8"
        button.setStyleSheet(f"background:{color};border:{border};padding:0px;font-size:9px;")

    def _step(self, delta: int) -> None:
        index = self._point.row * 15 + self._point.col
        self.select_point(format_point_id(*divmod((index + int(delta)) % 225, 15)))

    def _next_pending(self) -> None:
        start = self._point.row * 15 + self._point.col
        for offset in range(1, 226):
            row, col = divmod((start + offset) % 225, 15)
            key = format_point_id(row, col)
            if self._statuses.get(key, "NOT_GENERATED") in {
                "NOT_GENERATED",
                "GENERATED",
                "PENDING_VERIFY",
                "MANUAL_CORRECTED",
            }:
                self.select_point(key)
                return

    def _nudge_selected_correction(self, amount: int) -> None:
        joint = str(self.correction_joint_combo.currentData())
        spin = self._corrections[joint]
        spin.setValue(spin.value() + int(amount))

    def _save_correction(self) -> None:
        self.save_correction_requested.emit(
            self.current_point_id,
            {joint: spin.value() for joint, spin in self._corrections.items()},
        )

    @staticmethod
    def _pwm_text(values: Mapping[str, int] | None) -> str:
        if not values:
            return "-"
        return "[" + ", ".join(str(int(values[joint])) for joint in SPATIAL_KEYS) + "]"
