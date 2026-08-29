from __future__ import annotations

from typing import Any, Mapping

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPlainTextEdit,
    QPushButton,
    QSpinBox,
    QStackedWidget,
    QVBoxLayout,
    QWidget,
)

from app.integrated_v1.golden import SPATIAL_KEYS
from app.integrated_v1.movel import DropStatus
from app.integrated_v1.points import PointRef


STEP_TITLES = (
    "1 选择点位",
    "2 确认 ABOVE",
    "3 测试 DROP",
    "4 修正 DROP",
    "5 Test PLACE",
)


class _StepStrip(QWidget):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(6)
        self.labels: list[QLabel] = []
        for title in STEP_TITLES:
            label = QLabel(title)
            label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            label.setMinimumHeight(42)
            layout.addWidget(label, 1)
            self.labels.append(label)
        self.set_step(1)

    def set_step(self, current: int, completed: set[int] | None = None) -> None:
        done = set(completed or set())
        for index, label in enumerate(self.labels, start=1):
            if index == int(current):
                style = "background:#1769aa;color:white;border:2px solid #0d4775;"
            elif index in done:
                style = "background:#dff1e6;color:#185c39;border:1px solid #8cc7a5;"
            else:
                style = "background:#eef1f4;color:#69737d;border:1px solid #d6dce2;"
            label.setStyleSheet(
                style + "padding:7px;border-radius:5px;font-weight:700;"
            )


class _FoldSection(QGroupBox):
    def __init__(self, title: str, *, expanded: bool = False, parent=None) -> None:
        super().__init__(title, parent)
        self._base_title = title
        self.setCheckable(True)
        shell = QVBoxLayout(self)
        shell.setContentsMargins(10, 8, 10, 8)
        self.body = QWidget(self)
        self.body_layout = QVBoxLayout(self.body)
        self.body_layout.setContentsMargins(0, 0, 0, 0)
        shell.addWidget(self.body)
        self.toggled.connect(self._set_expanded)
        self.setChecked(bool(expanded))
        self._set_expanded(bool(expanded))

    def _set_expanded(self, expanded: bool) -> None:
        self.body.setVisible(bool(expanded))
        self.setTitle(f"{self._base_title}  {'▾' if expanded else '▸'}")

    def expand(self) -> None:
        self.setChecked(True)


class LiteDropV1Panel(QWidget):
    """State-driven one-point wizard. It owns no controller or serial path."""

    point_requested = Signal(int, int)
    generate_requested = Signal(str)
    generate_all_requested = Signal()
    preview_requested = Signal(str)
    move_above_requested = Signal(str)
    above_confirmed_requested = Signal(str)
    above_incorrect_requested = Signal(str)
    next_waypoint_requested = Signal(str)
    previous_waypoint_requested = Signal(str)
    move_drop_requested = Signal(str)
    retract_requested = Signal(str)
    drop_accurate_requested = Signal(str)
    correction_mode_requested = Signal(str)
    drop_mark_verified_requested = Signal(str)
    return_retest_requested = Signal(str)
    test_place_requested = Signal(str)
    correction_apply_requested = Signal(str, int, int)
    correction_save_requested = Signal(str, object)
    verify_requested = Signal(str)
    safe_return_requested = Signal(str)
    emergency_stop_requested = Signal()
    back_requested = Signal()

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._point = PointRef(7, 7)
        self._corrections: dict[str, QSpinBox] = {}
        self._correction_apply_buttons: list[QPushButton] = []
        self._candidate_executable = False
        self._build_ui()

    @property
    def point_id(self) -> str:
        return self._point.point_id

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 6, 0, 0)
        root.setSpacing(10)

        title_row = QHBoxLayout()
        title = QLabel("Single-Point MoveL / PLACE Wizard")
        title.setObjectName("pointTitle")
        title_row.addWidget(title, 1)
        self.point_label = QLabel("P77  ·  (7, 7)")
        self.point_label.setStyleSheet(
            "font-size:18px;font-weight:800;color:#174a73;padding:4px 10px;"
            "background:#e8f3fb;border-radius:5px;"
        )
        title_row.addWidget(self.point_label)
        root.addLayout(title_row)

        self.step_strip = _StepStrip()
        root.addWidget(self.step_strip)

        instruction = QFrame()
        instruction.setStyleSheet(
            "QFrame{background:#f7fbfe;border:1px solid #b9d7ea;border-radius:6px;}"
        )
        instruction_layout = QVBoxLayout(instruction)
        self.step_title_label = QLabel("Step 1 · 选择点位并载入 ABOVE")
        self.step_title_label.setStyleSheet(
            "font-size:18px;font-weight:800;color:#174a73;"
        )
        self.next_action_label = QLabel("选择棋盘交点，然后载入该点已保存的 ABOVE。")
        self.next_action_label.setWordWrap(True)
        self.next_action_label.setObjectName("nextAction")
        instruction_layout.addWidget(self.step_title_label)
        instruction_layout.addWidget(self.next_action_label)
        root.addWidget(instruction)

        safety = QGroupBox("Safety · 始终显示")
        safety_layout = QHBoxLayout(safety)
        self.emergency_button = QPushButton("EMERGENCY STOP")
        self.emergency_button.setStyleSheet(
            "background:#b3261e;color:white;font-weight:900;padding:9px 14px;"
        )
        self.retract_button = QPushButton("Return to ABOVE")
        self.safe_return_button = QPushButton("Safe Return")
        self.emergency_button.clicked.connect(self.emergency_stop_requested.emit)
        self.retract_button.clicked.connect(
            lambda _c=False: self.retract_requested.emit(self.point_id)
        )
        self.safe_return_button.clicked.connect(
            lambda _c=False: self.safe_return_requested.emit(self.point_id)
        )
        safety_layout.addWidget(self.emergency_button)
        safety_layout.addStretch(1)
        safety_layout.addWidget(self.retract_button)
        safety_layout.addWidget(self.safe_return_button)
        root.addWidget(safety)

        self.workflow_pages = QStackedWidget()
        self.workflow_pages.addWidget(self._build_step_1())
        self.workflow_pages.addWidget(self._build_step_2())
        self.workflow_pages.addWidget(self._build_step_3())
        self.workflow_pages.addWidget(self._build_step_4())
        self.workflow_pages.addWidget(self._build_step_5())
        root.addWidget(self.workflow_pages)

        self.pwm_section = self._build_pwm_section()
        self.advanced_section = self._build_advanced_section()
        root.addWidget(self.pwm_section)
        root.addWidget(self.advanced_section)

        back = QPushButton("返回 Lite 首页")
        back.clicked.connect(self.back_requested.emit)
        root.addWidget(back)
        root.addStretch(1)

    @staticmethod
    def _step_box(title: str, description: str) -> tuple[QGroupBox, QVBoxLayout]:
        box = QGroupBox(title)
        layout = QVBoxLayout(box)
        text = QLabel(description)
        text.setWordWrap(True)
        text.setStyleSheet("color:#46535e;padding:2px 0 6px 0;")
        layout.addWidget(text)
        return box, layout

    def _build_step_1(self) -> QWidget:
        box, layout = self._step_box(
            "Step 1 · 选择点位并载入 ABOVE",
            "选择单个棋盘点。切换点位会重置本页进度，但不会修改保存的数据。",
        )
        row = QHBoxLayout()
        row.addWidget(QLabel("Row"))
        self.row_spin = QSpinBox()
        self.row_spin.setRange(0, 14)
        self.row_spin.setValue(7)
        row.addWidget(self.row_spin)
        row.addWidget(QLabel("Col"))
        self.col_spin = QSpinBox()
        self.col_spin.setRange(0, 14)
        self.col_spin.setValue(7)
        row.addWidget(self.col_spin)
        self.load_point_button = QPushButton("载入已保存 ABOVE")
        self.load_point_button.setObjectName("primaryButton")
        self.load_point_button.clicked.connect(
            lambda _c=False: self.point_requested.emit(
                self.row_spin.value(), self.col_spin.value()
            )
        )
        row.addWidget(self.load_point_button, 1)
        layout.addLayout(row)
        self.step1_summary = QLabel("当前默认：P77。点击载入开始此点流程。")
        self.step1_summary.setWordWrap(True)
        layout.addWidget(self.step1_summary)
        return box

    def _build_step_2(self) -> QWidget:
        box, layout = self._step_box(
            "Step 2 · Move ABOVE 并确认",
            "移动到已保存 ABOVE。机械臂停止后，目视确认位置正确才可继续。",
        )
        self.move_above_button = QPushButton("1. Move ABOVE")
        self.move_above_button.setObjectName("primaryButton")
        self.above_confirm_button = QPushButton("ABOVE 正确 · 继续测试 DROP")
        self.above_incorrect_button = QPushButton("ABOVE 不正确 · Safe Return")
        self.move_above_button.clicked.connect(
            lambda _c=False: self.move_above_requested.emit(self.point_id)
        )
        self.above_confirm_button.clicked.connect(
            lambda _c=False: self.above_confirmed_requested.emit(self.point_id)
        )
        self.above_incorrect_button.clicked.connect(
            lambda _c=False: self.above_incorrect_requested.emit(self.point_id)
        )
        layout.addWidget(self.move_above_button)
        choice = QHBoxLayout()
        choice.addWidget(self.above_confirm_button, 1)
        choice.addWidget(self.above_incorrect_button)
        layout.addLayout(choice)
        return box

    def _build_step_3(self) -> QWidget:
        box, layout = self._step_box(
            "Step 3 · 测试 MoveL / DROP",
            "推荐逐 waypoint 下降。也可从 ABOVE 完整下降；随时可原路回撤。",
        )
        self.waypoint_progress_label = QLabel("Waypoint: ABOVE / WP00")
        self.waypoint_progress_label.setStyleSheet("font-size:16px;font-weight:700;")
        layout.addWidget(self.waypoint_progress_label)
        navigation = QGridLayout()
        self.previous_waypoint_button = QPushButton("Previous Waypoint")
        self.next_waypoint_button = QPushButton("Next Waypoint")
        self.move_drop_button = QPushButton("2. Full DROP")
        self.step_return_above_button = QPushButton("3. Return to ABOVE")
        self.next_waypoint_button.setObjectName("primaryButton")
        self.previous_waypoint_button.clicked.connect(
            lambda _c=False: self.previous_waypoint_requested.emit(self.point_id)
        )
        self.next_waypoint_button.clicked.connect(
            lambda _c=False: self.next_waypoint_requested.emit(self.point_id)
        )
        self.move_drop_button.clicked.connect(
            lambda _c=False: self.move_drop_requested.emit(self.point_id)
        )
        self.step_return_above_button.clicked.connect(
            lambda _c=False: self.retract_requested.emit(self.point_id)
        )
        navigation.addWidget(self.previous_waypoint_button, 0, 0)
        navigation.addWidget(self.next_waypoint_button, 0, 1)
        navigation.addWidget(self.move_drop_button, 1, 0)
        navigation.addWidget(self.step_return_above_button, 1, 1)
        layout.addLayout(navigation)
        outcome = QHBoxLayout()
        self.drop_accurate_button = QPushButton("DROP 准确 · 继续")
        self.open_correction_button = QPushButton("DROP 不准 · 微调")
        self.drop_accurate_button.clicked.connect(
            lambda _c=False: self.drop_accurate_requested.emit(self.point_id)
        )
        self.open_correction_button.clicked.connect(self._request_correction)
        outcome.addWidget(self.drop_accurate_button, 1)
        outcome.addWidget(self.open_correction_button, 1)
        layout.addLayout(outcome)
        return box

    def _build_step_4(self) -> QWidget:
        box, layout = self._step_box(
            "Step 4 · PWM correction 与复测",
            "Advanced PWM 默认折叠。仅调整 J0–J4；保存不会覆盖自动 DROP。",
        )
        self.expand_pwm_button = QPushButton("展开 PWM correction")
        self.expand_pwm_button.clicked.connect(self._request_correction)
        self.drop_mark_verified_button = QPushButton("Mark DROP Verified")
        self.drop_mark_verified_button.setObjectName("primaryButton")
        self.return_retest_button = QPushButton("Return to re-test DROP")
        self.drop_mark_verified_button.clicked.connect(
            lambda _c=False: self.drop_mark_verified_requested.emit(self.point_id)
        )
        self.return_retest_button.clicked.connect(
            lambda _c=False: self.return_retest_requested.emit(self.point_id)
        )
        layout.addWidget(self.expand_pwm_button)
        row = QHBoxLayout()
        row.addWidget(self.drop_mark_verified_button, 1)
        row.addWidget(self.return_retest_button, 1)
        layout.addLayout(row)
        note = QLabel(
            "这里的 Mark 只完成本页 DROP 精度门控，不写入 HARDWARE VERIFIED。"
            "最终硬件确认必须等待 Step 5 Test PLACE 成功。"
        )
        note.setWordWrap(True)
        note.setStyleSheet("color:#875000;")
        layout.addWidget(note)
        return box

    def _build_step_5(self) -> QWidget:
        box, layout = self._step_box(
            "Step 5 · Test PLACE 与最终确认",
            "DROP 已确认并安全返回后，执行完整取棋、落棋、释放和垂直回撤。",
        )
        self.test_place_button = QPushButton("4. Test PLACE")
        self.test_place_button.setObjectName("primaryButton")
        self.verify_button = QPushButton("Confirm HARDWARE VERIFIED")
        self.verify_button.setStyleSheet("font-weight:800;padding:9px;")
        self.test_place_button.clicked.connect(
            lambda _c=False: self.test_place_requested.emit(self.point_id)
        )
        self.verify_button.clicked.connect(
            lambda _c=False: self.verify_requested.emit(self.point_id)
        )
        layout.addWidget(self.test_place_button)
        layout.addWidget(self.verify_button)
        self.step5_status = QLabel("Test PLACE 尚未成功，最终确认保持锁定。")
        self.step5_status.setWordWrap(True)
        layout.addWidget(self.step5_status)
        return box

    def _build_pwm_section(self) -> _FoldSection:
        section = _FoldSection("Advanced PWM correction · 默认折叠")
        layout = QGridLayout()
        layout.addWidget(QLabel("Joint"), 0, 0)
        layout.addWidget(QLabel("Correction PWM"), 0, 1)
        layout.addWidget(QLabel("Live apply"), 0, 2)
        for index, joint in enumerate(SPATIAL_KEYS, start=1):
            layout.addWidget(QLabel(f"J{index - 1}"), index, 0)
            spin = QSpinBox()
            spin.setRange(-500, 500)
            spin.setSingleStep(1)
            layout.addWidget(spin, index, 1)
            apply_button = QPushButton(f"Apply J{index - 1}")
            apply_button.clicked.connect(
                lambda _c=False, jid=index - 1, field=spin: self.correction_apply_requested.emit(
                    self.point_id, jid, field.value()
                )
            )
            layout.addWidget(apply_button, index, 2)
            self._correction_apply_buttons.append(apply_button)
            self._corrections[joint] = spin
        locked = QLabel("J5  PUMP CHANNEL  LOCKED · excluded from FK/IK/correction")
        locked.setObjectName("lockedRow")
        layout.addWidget(locked, 6, 0, 1, 3)
        self.save_correction_button = QPushButton("Save DROP Correction")
        self.save_correction_button.setObjectName("confirmButton")
        self.save_correction_button.clicked.connect(
            lambda _c=False: self.correction_save_requested.emit(
                self.point_id, self.correction_values()
            )
        )
        layout.addWidget(self.save_correction_button, 7, 0, 1, 3)
        section.body_layout.addLayout(layout)
        return section

    def _build_advanced_section(self) -> _FoldSection:
        section = _FoldSection(
            "Advanced · MoveL Candidate / Cartesian / 225 statistics"
        )
        self.above_label = QLabel("ABOVE PWM: -")
        self.above_source_label = QLabel("ABOVE source: -")
        self.drop_auto_label = QLabel("DROP auto: -")
        self.drop_final_label = QLabel("DROP final: -")
        self.drop_status_label = QLabel("DROP status: NOT_GENERATED")
        self.drop_reason_label = QLabel("Reason: -")
        self.drop_reason_label.setWordWrap(True)
        for label in (
            self.above_label,
            self.above_source_label,
            self.drop_auto_label,
            self.drop_final_label,
            self.drop_status_label,
            self.drop_reason_label,
        ):
            section.body_layout.addWidget(label)
        planning = QHBoxLayout()
        self.generate_button = QPushButton("Generate / Refresh Candidate")
        self.preview_button = QPushButton("Preview MoveL")
        self.generate_all_button = QPushButton("Generate 225 OFFLINE ONLY")
        self.generate_button.clicked.connect(
            lambda _c=False: self.generate_requested.emit(self.point_id)
        )
        self.preview_button.clicked.connect(
            lambda _c=False: self.preview_requested.emit(self.point_id)
        )
        self.generate_all_button.clicked.connect(self.generate_all_requested.emit)
        planning.addWidget(self.generate_button)
        planning.addWidget(self.preview_button)
        planning.addWidget(self.generate_all_button)
        section.body_layout.addLayout(planning)
        self.preview_text = QPlainTextEdit()
        self.preview_text.setReadOnly(True)
        self.preview_text.setMaximumBlockCount(100)
        self.preview_text.setPlaceholderText(
            "Preview 显示 Cartesian Z、J0–J4 PWM 和 exact reverse path。"
        )
        self.preview_text.setMinimumHeight(150)
        section.body_layout.addWidget(self.preview_text)
        self.stats_label = QLabel("225 points · 0 generated · 0 verified")
        section.body_layout.addWidget(self.stats_label)
        return section

    def _request_correction(self) -> None:
        self.pwm_section.expand()
        self.correction_mode_requested.emit(self.point_id)

    def set_point(self, row: int, col: int) -> None:
        point = PointRef(int(row), int(col))
        self._point = point
        self.row_spin.setValue(point.row)
        self.col_spin.setValue(point.col)
        self.point_label.setText(f"{point.point_id}  ·  ({point.row}, {point.col})")

    def set_record(
        self,
        above: Mapping[str, Any],
        drop: Mapping[str, Any] | None,
    ) -> None:
        self.above_label.setText(
            "ABOVE PWM: " + self._pwm_text(above.get("final_above_pwm"))
        )
        flags = []
        if above.get("protected"):
            flags.append("PROTECTED GOLDEN")
        flags.append(str(above.get("verification_level", "NOT VERIFIED")))
        self.above_source_label.setText(
            f"ABOVE source: {above.get('source', '-')} · {' · '.join(flags)}"
        )
        if drop is None:
            self._candidate_executable = False
            self.drop_auto_label.setText("DROP auto: -")
            self.drop_final_label.setText("DROP final: -")
            self.drop_status_label.setText("DROP status: NOT_GENERATED")
            self.drop_reason_label.setText("Reason: -")
            self.step1_summary.setText(
                f"{self.point_id} ABOVE 已载入；DROP candidate 尚未生成。请在 Advanced 中生成。"
            )
            self.set_corrections({joint: 0 for joint in SPATIAL_KEYS})
            return
        status = str(drop.get("status"))
        self._candidate_executable = status not in {
            DropStatus.NOT_GENERATED.value,
            DropStatus.MOVE_L_UNREACHABLE.value,
            DropStatus.INVALID.value,
        }
        self.drop_auto_label.setText(
            "DROP auto: " + self._pwm_text(drop.get("drop_auto_pwm"))
        )
        self.drop_final_label.setText(
            "DROP final: " + self._pwm_text(drop.get("drop_final_pwm"))
        )
        self.drop_status_label.setText(
            f"DROP status: {status} · "
            f"{drop.get('verification_level', 'NOT VERIFIED')} · "
            f"max safe {drop.get('max_safe_descent_mm', 0)} mm"
        )
        self.drop_reason_label.setText(f"Reason: {drop.get('reason') or '-'}")
        self.step1_summary.setText(
            f"{self.point_id} ABOVE 已载入 · Candidate {status} · "
            f"{len(drop.get('waypoints') or [])} Cartesian waypoints"
        )
        self.set_corrections(
            drop.get("drop_correction_pwm")
            or {joint: 0 for joint in SPATIAL_KEYS}
        )

    def set_preview(self, preview: Mapping[str, Any]) -> None:
        lines = [
            f"Point: {preview.get('point_id')}",
            f"Status: {preview.get('status')}",
            "Hardware execution: NO",
            "Descent:",
        ]
        for waypoint in preview.get("descent") or []:
            pose = waypoint.get("cartesian_pose") or {}
            lines.append(
                f"  WP{int(waypoint['index']):02d}  dz={waypoint['descent_mm']:.1f} mm  "
                f"XYZ=({pose.get('x', 0):.2f},{pose.get('y', 0):.2f},{pose.get('z', 0):.2f})  "
                f"PWM={self._pwm_text(waypoint.get('pwm'))}"
            )
        lines.append("Exact reverse:")
        for waypoint in preview.get("reverse_ascent") or []:
            lines.append(
                f"  WP{int(waypoint['index']):02d}  dz={waypoint['descent_mm']:.1f} mm"
            )
        if preview.get("reason"):
            lines.append(f"Blocked reason: {preview['reason']}")
        self.preview_text.setPlainText("\n".join(lines))

    def set_workflow_state(
        self,
        *,
        step: int,
        completed_steps: set[int] | None = None,
        message: str = "",
        waypoint_index: int = 0,
        waypoint_count: int = 0,
        test_place_succeeded: bool = False,
    ) -> None:
        current = max(1, min(5, int(step)))
        self.step_strip.set_step(current, completed_steps)
        self.workflow_pages.setCurrentIndex(current - 1)
        self.step_title_label.setText(f"Step {current} · {STEP_TITLES[current - 1][2:]}")
        if message:
            self.next_action_label.setText(message)
        last_index = max(0, int(waypoint_count) - 1)
        self.waypoint_progress_label.setText(
            f"Waypoint: WP{int(waypoint_index):02d} / WP{last_index:02d}"
            if waypoint_count
            else "Waypoint: candidate unavailable"
        )
        self.step5_status.setText(
            "Test PLACE 已成功；现在才允许保存最终验证。"
            if test_place_succeeded
            else "Test PLACE 尚未成功，最终确认保持锁定。"
        )

    def correction_values(self) -> dict[str, int]:
        return {joint: spin.value() for joint, spin in self._corrections.items()}

    def set_corrections(self, values: Mapping[str, int]) -> None:
        for joint, spin in self._corrections.items():
            spin.setValue(int(values.get(joint, 0)))

    def set_statistics(self, stats: Mapping[str, int]) -> None:
        self.stats_label.setText(
            f"{stats.get('Total', 225)} points · {stats.get('Generated', 0)} generated · "
            f"{stats.get('Verified', 0)} verified · "
            f"{stats.get('Unreachable', 0)} unreachable · "
            f"{stats.get('Invalid', 0)} invalid"
        )

    def set_motion_state(
        self,
        *,
        connected: bool,
        busy: bool,
        estop: bool,
        pose_state: str,
        verification_eligible: bool,
        dry_run: bool,
        wizard_step: int = 1,
        above_confirmed: bool = False,
        drop_accuracy_confirmed: bool = False,
        test_place_succeeded: bool = False,
        waypoint_index: int = 0,
        waypoint_count: int = 0,
    ) -> None:
        ready = bool(connected and not busy and not estop)
        offline_ready = bool(not busy and not estop and pose_state == "SAFE")
        last_index = max(0, int(waypoint_count) - 1)
        at_drop = pose_state == "DROP"
        on_path = pose_state in {"WAYPOINT", "DROP"}

        self.load_point_button.setEnabled(not busy and pose_state == "SAFE")
        self.generate_button.setEnabled(offline_ready)
        self.generate_all_button.setEnabled(offline_ready)
        self.preview_button.setEnabled(not busy and not estop)
        self.move_above_button.setEnabled(
            ready
            and int(wizard_step) == 2
            and pose_state == "SAFE"
            and self._candidate_executable
        )
        self.above_confirm_button.setEnabled(
            ready and int(wizard_step) == 2 and pose_state == "ABOVE"
        )
        self.above_incorrect_button.setEnabled(
            ready and int(wizard_step) == 2 and pose_state == "ABOVE"
        )

        step3_ready = ready and int(wizard_step) == 3 and bool(above_confirmed)
        self.next_waypoint_button.setEnabled(
            step3_ready
            and pose_state in {"ABOVE", "WAYPOINT"}
            and int(waypoint_index) < last_index
        )
        self.previous_waypoint_button.setEnabled(
            step3_ready and on_path and int(waypoint_index) > 0
        )
        self.move_drop_button.setEnabled(step3_ready and pose_state == "ABOVE")
        self.step_return_above_button.setEnabled(step3_ready and on_path)
        self.drop_accurate_button.setEnabled(step3_ready and at_drop)
        self.open_correction_button.setEnabled(step3_ready and at_drop)

        correction_ready = ready and int(wizard_step) == 4 and at_drop
        self.expand_pwm_button.setEnabled(correction_ready)
        for button in self._correction_apply_buttons:
            button.setEnabled(correction_ready)
        self.save_correction_button.setEnabled(correction_ready)
        for spin in self._corrections.values():
            spin.setEnabled(correction_ready)
        self.drop_mark_verified_button.setEnabled(correction_ready)
        self.return_retest_button.setEnabled(correction_ready)

        self.retract_button.setEnabled(ready and on_path)
        self.safe_return_button.setEnabled(ready and pose_state == "ABOVE")
        self.emergency_button.setEnabled(bool(connected and not estop))

        self.test_place_button.setEnabled(
            ready
            and int(wizard_step) == 5
            and bool(drop_accuracy_confirmed)
            and pose_state == "SAFE"
        )
        self.verify_button.setEnabled(
            ready
            and int(wizard_step) == 5
            and bool(test_place_succeeded)
            and (bool(dry_run) or bool(verification_eligible))
        )
        self.verify_button.setText(
            "Confirm OFFLINE VERIFIED" if dry_run else "Confirm HARDWARE VERIFIED"
        )

    @staticmethod
    def _pwm_text(values: Mapping[str, int] | None) -> str:
        if not values:
            return "-"
        return " ".join(
            f"J{index}={int(values[joint])}"
            for index, joint in enumerate(SPATIAL_KEYS)
        )
