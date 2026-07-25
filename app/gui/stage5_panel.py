from __future__ import annotations

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QCheckBox,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

from app.stage5.constants import OUTER_RING, STAR_CORNERS


class Stage5Panel(QWidget):
    """Simple test-first Stage-5 UI. Advanced tools stay collapsed."""

    dry_run_toggled = Signal(bool)
    hover_requested = Signal()
    safe_return_requested = Signal()
    set_anchor_requested = Signal()
    load_from_action_requested = Signal()
    save_anchor_requested = Signal()
    test_anchor_hover_requested = Signal()
    confirm_anchor_requested = Signal()
    revoke_anchor_requested = Signal()
    load_calibration_requested = Signal()
    export_calibration_requested = Signal()
    restore_backup_requested = Signal()
    clear_target_requested = Signal()
    recover_requested = Signal()
    estop_requested = Signal()
    board_tour_requested = Signal()
    reload_inference_requested = Signal()
    save_finetune_requested = Signal()
    nudge_joint_requested = Signal(str, int)
    star_select_requested = Signal(int)
    star_next_requested = Signal()
    star_seed_requested = Signal()
    outer_select_requested = Signal(int)
    outer_next_requested = Signal()

    def __init__(self, *, default_dry_run: bool = True, parent=None) -> None:
        super().__init__(parent)

        # ---- status ----
        self.state_label = QLabel("状态: 未连接")
        self.state_label.setStyleSheet("font-weight:700;font-size:16px;")
        self.next_label = QLabel("下一步: 连接串口与相机，回观察位")
        self.next_label.setWordWrap(True)
        self.next_label.setStyleSheet(
            "background:#e8f5e9;border:1px solid #a5d6a7;border-radius:6px;"
            "padding:10px;font-size:14px;font-weight:600;color:#1b5e20;"
        )
        self.target_label = QLabel("目标: 未选择（请点左侧画面交点）")
        self.target_label.setStyleSheet("font-size:13px;")
        self.summary_label = QLabel("来源: -    标定: -")
        self.summary_label.setStyleSheet("color:#555;")

        # keep these for main_window.update_target_view / diagnostics
        self.pwm_label = QLabel("PWM: -")
        self.source_label = QLabel("来源: -")
        self.calib_label = QLabel("标定: -")
        self.region_label = QLabel("覆盖区: -")
        self.pixel_label = QLabel("像素: -")
        self.verified_label = QLabel("verified_runs: 0")
        self.serial_sync_label = QLabel("Serial Sync: FALSE")
        self.board_sync_label = QLabel("Board Sync: FALSE")
        self.arm_sync_label = QLabel("Arm Sync: -")
        self.estop_sync_label = QLabel("E-Stop: FALSE")
        self.controller_shared_label = QLabel("Controller Shared: -")
        self.blocked_label = QLabel("Blocked: -")
        self.guide_label = self.next_label  # alias if anything still references guide

        # dry-run: inverted wording for operators
        self.dry_run_checkbox = QCheckBox("仅演练（勾选=不动臂，只打日志）")
        self.dry_run_checkbox.setChecked(bool(default_dry_run))
        self.dry_run_checkbox.toggled.connect(self.dry_run_toggled.emit)
        self.live_hint = QLabel("真机测试请取消勾选「仅演练」")
        self.live_hint.setStyleSheet("color:#b3262e;font-weight:600;")

        # ---- primary actions ----
        self.hover_button = QPushButton("1. 用当前PWM去悬停（再测）")
        self.hover_button.setMinimumHeight(48)
        self.hover_button.setStyleSheet("font-size:15px;font-weight:700;")
        self.hover_button.setToolTip("需：串口已连、棋盘锁定、臂在观察位、已选目标")
        self.return_button = QPushButton("2. 回观察位")
        self.return_button.setMinimumHeight(48)
        self.return_button.setStyleSheet("font-size:15px;font-weight:700;")
        self.save_finetune_button = QPushButton("3. 确认无误 → 保存当前PWM到标定")
        self.save_finetune_button.setMinimumHeight(44)
        self.save_finetune_button.setStyleSheet("font-size:14px;font-weight:700;")
        self.clear_button = QPushButton("清除目标")
        self.clear_button.setMinimumHeight(36)

        self.hover_button.clicked.connect(lambda: self.hover_requested.emit())
        self.return_button.clicked.connect(lambda: self.safe_return_requested.emit())
        self.save_finetune_button.clicked.connect(lambda: self.save_finetune_requested.emit())
        self.clear_button.clicked.connect(lambda: self.clear_target_requested.emit())

        # ---- star corners: 4 simple buttons ----
        star_box = QGroupBox("快捷：星位四角（盘内插值用）")
        star_l = QVBoxLayout(star_box)
        self.star_status_label = QLabel("星位进度: -")
        self.star_status_label.setWordWrap(True)
        star_grid = QGridLayout()
        self.star_buttons: list[QPushButton] = []
        positions = [(0, 0), (0, 1), (1, 0), (1, 1)]
        for i, ((r, c, _lab, cn), (gr, gc)) in enumerate(zip(STAR_CORNERS, positions)):
            b = QPushButton(f"{cn}\nP({r},{c})")
            b.setMinimumHeight(52)
            b.clicked.connect(lambda _=False, idx=i: self._on_star_clicked(idx))
            star_grid.addWidget(b, gr, gc)
            self.star_buttons.append(b)
        self.star_next_button = QPushButton("下一个未校准星位")
        self.star_next_button.setMinimumHeight(40)
        self.star_next_button.clicked.connect(lambda: self.star_next_requested.emit())
        # keep combo for update_star_status index API (hidden)
        from PySide6.QtWidgets import QComboBox

        self.star_combo = QComboBox()
        self.star_combo.setVisible(False)
        for i, (r, c, lab, cn) in enumerate(STAR_CORNERS):
            self.star_combo.addItem(f"P({r},{c}) {cn}", i)
        self.star_seed_button = QPushButton("载入星位初值")  # kept for API; hidden, auto on select
        self.star_seed_button.setVisible(False)
        star_l.addWidget(self.star_status_label)
        star_l.addLayout(star_grid)
        star_l.addWidget(self.star_next_button)
        star_l.addWidget(QLabel("点星位 → 自动载入初值 → 按上面 1→2→3 测并保存"))

        # ---- tour ----
        self.board_tour_button = QPushButton("巡检（锚点复验 / 全点+冷却）")
        self.board_tour_button.setMinimumHeight(40)
        self.board_tour_button.clicked.connect(lambda: self.board_tour_requested.emit())

        # ---- estop / recover ----
        self.estop_button = QPushButton("急停")
        self.estop_button.setMinimumHeight(52)
        self.estop_button.setStyleSheet(
            "QPushButton { background:#b3262e; color:white; font-size:18px; font-weight:700; }"
            "QPushButton:hover { background:#d3313b; }"
        )
        self.estop_button.clicked.connect(lambda: self.estop_requested.emit())
        self.recover_button = QPushButton("急停后恢复（然后请回观察位）")
        self.recover_button.setMinimumHeight(40)
        self.recover_button.clicked.connect(lambda: self.recover_requested.emit())

        # ---- PWM fine tune (collapsed) ----
        self.tune_group = QGroupBox("微调 PWM 后：必须再点「1再测」，满意再点「3保存」")
        self.tune_group.setCheckable(True)
        self.tune_group.setChecked(False)
        tune_l = QVBoxLayout(self.tune_group)
        self.pwm_edits: dict[str, QLineEdit] = {}
        pwm_row = QHBoxLayout()
        for jid in ("000", "001", "002", "003", "004"):
            edit = QLineEdit()
            edit.setPlaceholderText(jid)
            edit.setMaximumWidth(64)
            self.pwm_edits[jid] = edit
            pwm_row.addWidget(QLabel(jid))
            pwm_row.addWidget(edit)
        nudge_row = QHBoxLayout()
        for jid in ("000", "001", "002", "003"):
            for delta in (-5, 5):
                b = QPushButton(f"{jid[-1]}{delta:+d}")
                b.setMaximumWidth(48)
                b.clicked.connect(
                    lambda _c=False, j=jid, d=delta: self.nudge_joint_requested.emit(j, d)
                )
                nudge_row.addWidget(b)
        self.reload_inference_button = QPushButton("恢复推理值（放弃本次微调）")
        self.reload_inference_button.clicked.connect(lambda: self.reload_inference_requested.emit())
        self.tune_flow_label = QLabel(
            "校准步骤：①改下面数字/±5  →  ②先回观察位  →  ③点「1再测」看位置  →  ④满意再点「3保存」"
        )
        self.tune_flow_label.setWordWrap(True)
        self.tune_flow_label.setStyleSheet(
            "background:#fff3e0;border:1px solid #ffcc80;border-radius:4px;padding:8px;"
            "color:#e65100;font-weight:600;"
        )
        tune_l.addWidget(self.tune_flow_label)
        tune_l.addLayout(pwm_row)
        tune_l.addLayout(nudge_row)
        tune_l.addWidget(self.reload_inference_button)
        self.tune_group.toggled.connect(self._toggle_tune)

        # ---- advanced (collapsed) ----
        self._advanced_group = QGroupBox("高级：标定文件 / 诊断")
        self._advanced_group.setCheckable(True)
        self._advanced_group.setChecked(False)
        adv_l = QVBoxLayout(self._advanced_group)
        info = QFormLayout()
        info.addRow(self.pixel_label)
        info.addRow(self.region_label)
        info.addRow(self.pwm_label)
        info.addRow(self.verified_label)
        info.addRow(self.serial_sync_label)
        info.addRow(self.board_sync_label)
        info.addRow(self.arm_sync_label)
        info.addRow(self.estop_sync_label)
        info.addRow(self.controller_shared_label)
        info.addRow(self.blocked_label)
        adv_l.addLayout(info)

        self.set_anchor_button = QPushButton("设为标定点")
        self.load_action_button = QPushButton("从现有动作载入")
        self.save_anchor_button = QPushButton("保存锚点")
        self.test_anchor_button = QPushButton("测试锚点悬停")
        self.confirm_button = QPushButton("确认该锚点安全")
        self.revoke_button = QPushButton("取消安全确认")
        self.load_file_button = QPushButton("加载标定文件")
        self.export_button = QPushButton("导出标定文件")
        self.restore_button = QPushButton("恢复上一个备份")
        for b, sig in (
            (self.set_anchor_button, self.set_anchor_requested),
            (self.load_action_button, self.load_from_action_requested),
            (self.save_anchor_button, self.save_anchor_requested),
            (self.test_anchor_button, self.test_anchor_hover_requested),
            (self.confirm_button, self.confirm_anchor_requested),
            (self.revoke_button, self.revoke_anchor_requested),
            (self.load_file_button, self.load_calibration_requested),
            (self.export_button, self.export_calibration_requested),
            (self.restore_button, self.restore_backup_requested),
        ):
            b.clicked.connect(lambda _=False, s=sig: s.emit())
        grid = QGridLayout()
        buttons = [
            self.set_anchor_button,
            self.load_action_button,
            self.save_anchor_button,
            self.test_anchor_button,
            self.confirm_button,
            self.revoke_button,
            self.load_file_button,
            self.export_button,
            self.restore_button,
        ]
        for index, button in enumerate(buttons):
            grid.addWidget(button, index // 2, index % 2)
        adv_l.addLayout(grid)
        self._advanced_group.toggled.connect(self._toggle_advanced)

        # ---- layout ----
        root = QVBoxLayout(self)
        title = QLabel("棋盘悬停测试")
        title.setStyleSheet("font-weight:800;font-size:16px;")
        root.addWidget(title)
        root.addWidget(self.state_label)
        root.addWidget(self.next_label)
        root.addWidget(self.target_label)
        root.addWidget(self.summary_label)
        root.addWidget(self.dry_run_checkbox)
        root.addWidget(self.live_hint)

        root.addWidget(self.hover_button)
        root.addWidget(self.return_button)
        root.addWidget(self.save_finetune_button)
        root.addWidget(self.clear_button)
        root.addWidget(star_box)

        outer_box = QGroupBox("扩展：全盘外圈（覆盖到边角）")
        outer_l = QVBoxLayout(outer_box)
        self.outer_status_label = QLabel("外圈: 中心区已可用；教外圈后覆盖全盘")
        self.outer_status_label.setWordWrap(True)
        outer_grid = QGridLayout()
        self.outer_buttons: list[QPushButton] = []
        # 8 points in 2 rows
        for i, (r, c, _lab, cn) in enumerate(OUTER_RING):
            b = QPushButton(f"{cn}\nP({r},{c})")
            b.setMinimumHeight(44)
            b.clicked.connect(lambda _=False, idx=i: self.outer_select_requested.emit(int(idx)))
            outer_grid.addWidget(b, i // 4, i % 4)
            self.outer_buttons.append(b)
        self.outer_next_button = QPushButton("下一个未校准外圈点")
        self.outer_next_button.setMinimumHeight(36)
        self.outer_next_button.clicked.connect(lambda: self.outer_next_requested.emit())
        outer_l.addWidget(self.outer_status_label)
        outer_l.addLayout(outer_grid)
        outer_l.addWidget(self.outer_next_button)
        outer_l.addWidget(QLabel("建议顺序：先上下左右中点，再四角 → 1悬停 → 2回 → 3保存"))
        root.addWidget(outer_box)
        root.addWidget(self.board_tour_button)
        root.addWidget(self.recover_button)
        root.addWidget(self.estop_button)
        root.addWidget(self.tune_group)
        root.addWidget(self._advanced_group)
        root.addStretch(1)

        self._set_tune_visible(False)
        self._set_advanced_visible(False)

        self.set_enabled_state(
            serial_connected=False,
            board_locked=False,
            busy=False,
            can_hover=False,
            can_return=False,
            has_target=False,
            estop=False,
        )

    def _on_star_clicked(self, index: int) -> None:
        # select_star auto-loads taught or seed PWM; no extra dialogs
        self.star_select_requested.emit(int(index))

    def _toggle_tune(self, checked: bool) -> None:
        self._set_tune_visible(checked)

    def _set_tune_visible(self, visible: bool) -> None:
        lay = self.tune_group.layout()
        if lay is None:
            return
        for i in range(lay.count()):
            item = lay.itemAt(i)
            w = item.widget()
            if w is not None:
                w.setVisible(visible)
            elif item.layout() is not None:
                for j in range(item.layout().count()):
                    ww = item.layout().itemAt(j).widget()
                    if ww is not None:
                        ww.setVisible(visible)

    def _toggle_advanced(self, checked: bool) -> None:
        self._set_advanced_visible(checked)

    def _set_advanced_visible(self, visible: bool) -> None:
        lay = self._advanced_group.layout()
        if lay is None:
            return
        for i in range(lay.count()):
            item = lay.itemAt(i)
            w = item.widget()
            if w is not None:
                w.setVisible(visible)
            elif item.layout() is not None:
                for j in range(item.layout().count()):
                    child = item.layout().itemAt(j)
                    ww = child.widget() if child is not None else None
                    if ww is not None:
                        ww.setVisible(visible)
                    elif child is not None and child.layout() is not None:
                        for k in range(child.layout().count()):
                            www = child.layout().itemAt(k).widget()
                            if www is not None:
                                www.setVisible(visible)

    def pwm_values(self) -> dict[str, int]:
        values: dict[str, int] = {}
        for jid, edit in self.pwm_edits.items():
            text = edit.text().strip()
            if not text:
                raise ValueError(f"PWM {jid} is empty")
            values[jid] = int(text)
        return values

    def set_pwm_values(self, values: dict[str, int] | dict[int, int]) -> None:
        for jid, edit in self.pwm_edits.items():
            key_int = int(jid)
            if jid in values:
                edit.setText(str(int(values[jid])))
            elif key_int in values:
                edit.setText(str(int(values[key_int])))

    def update_outer_status(self, lines: list[str], current_index: int = 0) -> None:
        if hasattr(self, "outer_status_label"):
            self.outer_status_label.setText("外圈: " + "  ·  ".join(lines))
        if hasattr(self, "outer_buttons"):
            for i, b in enumerate(self.outer_buttons):
                done = i < len(lines) and "已校准" in lines[i]
                if i == int(current_index):
                    b.setStyleSheet("font-weight:700;border:2px solid #6a1b9a;")
                elif done:
                    b.setStyleSheet("background:#f3e5f5;font-weight:600;")
                else:
                    b.setStyleSheet("")

    def update_star_status(self, lines: list[str], current_index: int = 0) -> None:
        self.star_status_label.setText("星位: " + "  ·  ".join(lines))
        if 0 <= int(current_index) < self.star_combo.count():
            self.star_combo.blockSignals(True)
            self.star_combo.setCurrentIndex(int(current_index))
            self.star_combo.blockSignals(False)
        # highlight buttons
        for i, b in enumerate(self.star_buttons):
            done = i < len(lines) and "已校准" in lines[i]
            if i == int(current_index):
                b.setStyleSheet("font-weight:700;border:2px solid #1565c0;")
            elif done:
                b.setStyleSheet("background:#e8f5e9;font-weight:600;")
            else:
                b.setStyleSheet("")

    def update_target_view(
        self,
        *,
        state: str,
        row: int | None,
        col: int | None,
        pixel_x: float | None,
        pixel_y: float | None,
        calibrated: str,
        region: str,
        source: str,
        pwm_text: str,
        verified_runs: int,
    ) -> None:
        self.state_label.setText(f"状态: {state}")
        if row is None or col is None:
            self.target_label.setText("目标: 未选择（请点左侧画面交点，或点下方星位）")
        else:
            self.target_label.setText(f"目标: P({row},{col})")
        src_cn = {
            "direct_anchor": "已校准手教",
            "bilinear_interpolation": "默认插值",
            "star_parallelogram_seed": "星位初值",
            "user_edited": "手动编辑",
        }.get(source, source)
        cal_cn = "是" if str(calibrated).upper() in {"YES", "TRUE", "1"} else "否"
        self.summary_label.setText(f"来源: {src_cn}    已校准: {cal_cn}    覆盖区: {region}")
        self.calib_label.setText(f"标定: {calibrated}")
        self.source_label.setText(f"来源: {source}")
        self.pwm_label.setText(f"PWM: {pwm_text}")
        self.region_label.setText(f"覆盖区: {region}")
        self.verified_label.setText(f"verified_runs: {verified_runs}")
        if pixel_x is None or pixel_y is None:
            self.pixel_label.setText("像素: -")
        else:
            self.pixel_label.setText(f"像素: ({pixel_x:.1f}, {pixel_y:.1f})")

        # next-step coach
        if state in {"EMERGENCY_STOP", "ESTOP"} or "ESTOP" in state:
            self.next_label.setText("下一步: 点「急停后恢复」，再点主界面「回观察位」")
            self.next_label.setStyleSheet(
                "background:#ffebee;border:1px solid #ef9a9a;border-radius:6px;"
                "padding:10px;font-size:14px;font-weight:600;color:#b71c1c;"
            )
        elif row is None:
            self.next_label.setText("下一步: 在左侧画面点一个交点，或点「星位四角」之一")
            self.next_label.setStyleSheet(
                "background:#e3f2fd;border:1px solid #90caf9;border-radius:6px;"
                "padding:10px;font-size:14px;font-weight:600;color:#0d47a1;"
            )
        elif state in {"HOVERING", "AT_TARGET", "TARGET_REACHED"} or "HOVER" in state:
            self.next_label.setText("下一步: 看位置 → 点「2. 回观察位」→ 若位置好再点「3. 保存此点」")
            self.next_label.setStyleSheet(
                "background:#fff8e1;border:1px solid #ffe082;border-radius:6px;"
                "padding:10px;font-size:14px;font-weight:600;color:#e65100;"
            )
        else:
            self.next_label.setText("下一步: 取消「仅演练」(真机) → 点「1. 去目标点」")
            self.next_label.setStyleSheet(
                "background:#e8f5e9;border:1px solid #a5d6a7;border-radius:6px;"
                "padding:10px;font-size:14px;font-weight:600;color:#1b5e20;"
            )

    def update_sync_diagnostics(
        self,
        *,
        serial_sync: bool,
        board_sync: bool,
        arm_sync: str,
        estop: bool,
        controller_shared: bool,
        blocked_reason: str = "",
    ) -> None:
        self.serial_sync_label.setText(f"Serial Sync: {'TRUE' if serial_sync else 'FALSE'}")
        self.board_sync_label.setText(f"Board Sync: {'TRUE' if board_sync else 'FALSE'}")
        self.arm_sync_label.setText(f"Arm Sync: {arm_sync}")
        self.estop_sync_label.setText(f"E-Stop: {'TRUE' if estop else 'FALSE'}")
        self.controller_shared_label.setText(
            f"Controller Shared: {'TRUE' if controller_shared else 'FALSE'}"
        )
        self.blocked_label.setText(f"Blocked: {blocked_reason or '-'}")
        if blocked_reason and blocked_reason not in {"-", ""}:
            # keep next step informative when blocked
            if "ESTOP" in str(arm_sync).upper() or estop:
                pass
            elif not serial_sync:
                self.next_label.setText("下一步: 先连接串口")
            elif not board_sync:
                self.next_label.setText("下一步: 将棋盘放入视野并锁定（BOARD LOCKED）")

    def set_enabled_state(
        self,
        *,
        serial_connected: bool,
        board_locked: bool,
        busy: bool,
        can_hover: bool,
        can_return: bool,
        has_target: bool,
        estop: bool,
    ) -> None:
        ordinary = serial_connected and not busy and not estop
        self.dry_run_checkbox.setEnabled(True)
        # Hover needs TARGET_READY/DRY_RUN_READY + arm OBSERVE_*
        self.hover_button.setEnabled(ordinary and can_hover)
        # 回观察位：串口已连且不忙即可点（未悬停时走主回观察位）
        self.return_button.setEnabled(serial_connected and not busy and not estop)
        # Allow save whenever a target+PWM is present (taught or seed)
        self.save_finetune_button.setEnabled(ordinary and has_target)
        self.clear_button.setEnabled(ordinary and has_target and not can_return)
        if can_return:
            self.clear_button.setEnabled(False)
        self.board_tour_button.setEnabled(ordinary and serial_connected)
        self.reload_inference_button.setEnabled(ordinary and has_target)
        for b in self.star_buttons:
            b.setEnabled(ordinary)
        self.star_next_button.setEnabled(ordinary)
        if hasattr(self, "outer_buttons"):
            for b in self.outer_buttons:
                b.setEnabled(ordinary)
            self.outer_next_button.setEnabled(ordinary)
        self.recover_button.setEnabled(serial_connected and estop and not busy)
        self.estop_button.setEnabled(serial_connected)
        for button in (
            self.set_anchor_button,
            self.save_anchor_button,
            self.test_anchor_button,
            self.confirm_button,
            self.revoke_button,
        ):
            button.setEnabled(ordinary and has_target)
        self.load_file_button.setEnabled(not busy)
        self.export_button.setEnabled(not busy)
        self.restore_button.setEnabled(not busy)
        self.load_action_button.setEnabled(not busy)
        for edit in self.pwm_edits.values():
            edit.setEnabled(ordinary)

        # Why is hover gray? Coach the operator.
        if estop:
            self.next_label.setText("下一步: 点「急停后恢复」，再主界面「回观察位」")
            self._style_next("red")
        elif not serial_connected:
            self.next_label.setText("下一步: 先连接串口")
            self._style_next("blue")
        elif busy:
            self.next_label.setText("下一步: 等待当前动作完成…")
            self._style_next("blue")
        elif not board_locked:
            self.next_label.setText("下一步: 将棋盘锁定（BOARD LOCKED）")
            self._style_next("blue")
        elif not has_target:
            self.next_label.setText("下一步: 点左侧交点，或点下方星位四角")
            self._style_next("blue")
        elif can_return:
            self.next_label.setText("下一步: 看位置 →「2. 回观察位」→ 好则「3. 保存此点」")
            self._style_next("orange")
        elif can_hover:
            self.next_label.setText("下一步: 点「1. 去目标点（悬停）」；需要可展开下方微调 PWM")
            self._style_next("green")
        else:
            # Typical: arm not OBSERVE_IDLE after previous move
            self.next_label.setText(
                "下一步: 悬停暂不可用 — 请先点「2. 回观察位」或主界面「回观察位」，"
                "等 OBSERVE_IDLE 后再点「1. 用当前PWM去悬停」"
            )
            self._style_next("orange")

    def _style_next(self, kind: str) -> None:
        styles = {
            "green": (
                "background:#e8f5e9;border:1px solid #a5d6a7;border-radius:6px;"
                "padding:10px;font-size:14px;font-weight:600;color:#1b5e20;"
            ),
            "blue": (
                "background:#e3f2fd;border:1px solid #90caf9;border-radius:6px;"
                "padding:10px;font-size:14px;font-weight:600;color:#0d47a1;"
            ),
            "orange": (
                "background:#fff8e1;border:1px solid #ffe082;border-radius:6px;"
                "padding:10px;font-size:14px;font-weight:600;color:#e65100;"
            ),
            "red": (
                "background:#ffebee;border:1px solid #ef9a9a;border-radius:6px;"
                "padding:10px;font-size:14px;font-weight:600;color:#b71c1c;"
            ),
        }
        self.next_label.setStyleSheet(styles.get(kind, styles["blue"]))
