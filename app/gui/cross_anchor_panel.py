from __future__ import annotations

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QComboBox,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from app.stage5.constants import CROSS_ANCHORS, FORCE_STAGE5_DRY_RUN, SPATIAL_JOINTS


class CrossAnchorPanel(QWidget):
    """Center-cross calibration wizard UI. Emits intents only."""

    prev_requested = Signal()
    next_requested = Signal()
    select_index_requested = Signal(int)
    nudge_requested = Signal(str, int)
    set_joint_requested = Signal(str, int)
    reset_p77_requested = Signal()
    undo_requested = Signal()
    save_draft_requested = Signal()
    load_draft_requested = Signal()
    validate_requested = Signal()
    plan_carry_requested = Signal()
    plan_target_requested = Signal()
    plan_return_requested = Signal()
    execute_mock_requested = Signal()
    result_requested = Signal(str)
    complete_requested = Signal()
    cancel_confirm_requested = Signal()
    reverify_tour_requested = Signal()

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.progress_label = QLabel("中心十字标定：1/4  P(3,7)")
        self.state_label = QLabel("Wizard: IDLE")
        self.hint_label = QLabel(
            "点按钮后请看下方「向导日志」。不会自动动臂；先生成计划，再点执行。"
        )
        self.hint_label.setWordWrap(True)
        self.hint_label.setStyleSheet("color:#333;background:#fff8e1;padding:6px;")

        self.status_labels: dict[str, QLabel] = {}
        status_row = QHBoxLayout()
        for row, col, label, cn in CROSS_ANCHORS:
            key = f"{row},{col}"
            lab = QLabel(f"P({row},{col}) 未标定")
            self.status_labels[key] = lab
            status_row.addWidget(lab)

        self.anchor_combo = QComboBox()
        for i, (row, col, label, cn) in enumerate(CROSS_ANCHORS):
            self.anchor_combo.addItem(f"P({row},{col}) {cn}", i)
        # currentIndexChanged(int) matches Signal(int)
        self.anchor_combo.currentIndexChanged.connect(self._on_combo_changed)

        self.ref_label = QLabel("P77参考: -")
        self.delta_label = QLabel("差值: -")
        self.runs_label = QLabel("已验证 0 / 3")
        force_on = bool(FORCE_STAGE5_DRY_RUN)
        self.force_label = QLabel(
            f"FORCE_STAGE5_DRY_RUN={'TRUE（强制只演练）' if force_on else 'FALSE（取消DRY RUN可真机）'}"
        )
        self.force_label.setStyleSheet("color:#b3262e;font-weight:700;")
        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMinimumHeight(160)
        self.log_view.setPlaceholderText("这里会显示每次按钮的结果。若始终空白，说明信号未接通。")

        self.joint_edits: dict[str, QLineEdit] = {}
        joint_grid = QGridLayout()
        joint_grid.addWidget(QLabel("关节"), 0, 0)
        joint_grid.addWidget(QLabel("候选PWM"), 0, 1)
        joint_grid.addWidget(QLabel("微调"), 0, 2)
        for i, jid in enumerate(SPATIAL_JOINTS, start=1):
            joint_grid.addWidget(QLabel(jid), i, 0)
            edit = QLineEdit()
            edit.setMaximumWidth(70)
            self.joint_edits[jid] = edit
            joint_grid.addWidget(edit, i, 1)
            btn_row = QHBoxLayout()
            for delta in (-10, -5, -1, 1, 5, 10):
                b = QPushButton(f"{delta:+d}")
                b.setMaximumWidth(40)
                b.clicked.connect(lambda checked=False, j=jid, d=delta: self.nudge_requested.emit(j, d))
                btn_row.addWidget(b)
            wrap = QWidget()
            wrap.setLayout(btn_row)
            joint_grid.addWidget(wrap, i, 2)

        apply_btn = QPushButton("应用输入框数值")
        apply_btn.clicked.connect(self._emit_set_joints)

        nav = QHBoxLayout()
        prev_b = QPushButton("上一个锚点")
        next_b = QPushButton("下一个锚点")
        prev_b.clicked.connect(lambda checked=False: self.prev_requested.emit())
        next_b.clicked.connect(lambda checked=False: self.next_requested.emit())
        nav.addWidget(prev_b)
        nav.addWidget(next_b)

        # Step-oriented buttons (clearer labels)
        actions = QGridLayout()
        step_buttons = [
            (0, 0, "1.恢复P77参考", lambda c=False: self.reset_p77_requested.emit()),
            (0, 1, "撤销上次修改", lambda c=False: self.undo_requested.emit()),
            (1, 0, "2.保存草稿", lambda c=False: self.save_draft_requested.emit()),
            (1, 1, "载入草稿", lambda c=False: self.load_draft_requested.emit()),
            (2, 0, "3.安全检查", lambda c=False: self.validate_requested.emit()),
            (2, 1, "4a.生成运输高位计划", lambda c=False: self.plan_carry_requested.emit()),
            (3, 0, "4b.生成目标上方计划", lambda c=False: self.plan_target_requested.emit()),
            (3, 1, "4c.生成安全返回计划", lambda c=False: self.plan_return_requested.emit()),
            (4, 0, "5.执行当前计划(DRY/MOCK或真机)", lambda c=False: self.execute_mock_requested.emit()),
            (4, 1, "结果:位置正确安全", lambda c=False: self.result_requested.emit("SAFE_OK")),
            (5, 0, "结果:安全但偏移", lambda c=False: self.result_requested.emit("SAFE_WITH_OFFSET")),
            (5, 1, "结果:不安全", lambda c=False: self.result_requested.emit("UNSAFE")),
            (6, 0, "结果:急停", lambda c=False: self.result_requested.emit("ESTOP")),
            (6, 1, "结果:未完成", lambda c=False: self.result_requested.emit("INCOMPLETE")),
            (7, 0, "6.确认锚点完成(需3次成功)", lambda c=False: self.complete_requested.emit()),
            (7, 1, "取消安全确认", lambda c=False: self.cancel_confirm_requested.emit()),
            (8, 0, "一键复验已完成锚点", lambda c=False: self.reverify_tour_requested.emit()),
        ]
        for r, c, text, slot in step_buttons:
            b = QPushButton(text)
            b.clicked.connect(slot)
            actions.addWidget(b, r, c)

        layout = QVBoxLayout(self)
        title = QLabel("中心十字锚点标定向导")
        title.setStyleSheet("font-weight:700;font-size:14px;")
        layout.addWidget(title)
        layout.addWidget(self.hint_label)
        layout.addWidget(self.force_label)
        layout.addWidget(self.progress_label)
        layout.addWidget(self.state_label)
        layout.addLayout(status_row)
        layout.addWidget(self.anchor_combo)
        layout.addLayout(nav)
        layout.addWidget(self.ref_label)
        layout.addWidget(self.delta_label)
        layout.addWidget(self.runs_label)
        layout.addLayout(joint_grid)
        layout.addWidget(apply_btn)
        layout.addLayout(actions)
        layout.addWidget(QLabel("向导日志（点按钮后这里必须有字）"))
        layout.addWidget(self.log_view)

        self.append_log("向导已就绪。请按 1→2→3→4→5 顺序点；日志应同步更新。")

    def _on_combo_changed(self, index: int) -> None:
        if index < 0:
            return
        self.select_index_requested.emit(int(index))

    def _emit_set_joints(self) -> None:
        try:
            for jid, edit in self.joint_edits.items():
                text = edit.text().strip()
                if not text:
                    continue
                self.set_joint_requested.emit(jid, int(text))
            self.append_log("已请求应用输入框数值")
        except Exception as exc:
            self.append_log(f"ERROR 输入无效: {exc}")

    def read_candidate_pwm(self) -> dict[str, int]:
        """Read current line-edit values (user edits)."""
        values: dict[str, int] = {}
        for jid, edit in self.joint_edits.items():
            text = edit.text().strip()
            if text == "":
                raise ValueError(f"joint {jid} empty")
            values[jid] = int(text)
        return values

    def append_log(self, message: str) -> None:
        self.log_view.append(message)
        # keep latest visible
        bar = self.log_view.verticalScrollBar()
        bar.setValue(bar.maximum())

    def update_view(self, snap: dict, *, push_edits: bool = False) -> None:
        self.progress_label.setText(str(snap.get("progress", "-")))
        self.state_label.setText(f"Wizard: {snap.get('state', '-')}")
        self.runs_label.setText(
            f"已验证 {snap.get('verified_runs', 0)} / {snap.get('required_runs', 3)}"
        )
        ref = snap.get("reference_pwm") or {}
        cand = snap.get("candidate_pwm") or {}
        deltas = snap.get("deltas") or {}
        self.ref_label.setText(
            "P77参考: " + ", ".join(f"{k}={ref.get(k, '-')}" for k in SPATIAL_JOINTS)
        )
        self.delta_label.setText(
            "差值: " + ", ".join(f"{k}:{int(deltas.get(k, 0)):+d}" for k in SPATIAL_JOINTS)
        )
        # Only overwrite input boxes when wizard values intentionally changed
        # (nudge/reset/load). Otherwise user typing would be wiped on every click.
        if push_edits:
            for jid in SPATIAL_JOINTS:
                if jid in cand and cand[jid] is not None:
                    self.joint_edits[jid].setText(str(cand[jid]))
        status_map = snap.get("status_map") or {}
        for row, col, _label, _cn in CROSS_ANCHORS:
            key = f"{row},{col}"
            st = status_map.get(key, "EMPTY")
            if st == "COMPLETED":
                text = f"P({row},{col}) 已确认"
            elif st in {"VERIFIED_ONCE", "DRAFT"}:
                text = f"P({row},{col}) 候选/{st}"
            else:
                text = f"P({row},{col}) 未标定"
            if key in self.status_labels:
                self.status_labels[key].setText(text)
        idx = int(snap.get("index", 0))
        if self.anchor_combo.currentIndex() != idx:
            self.anchor_combo.blockSignals(True)
            self.anchor_combo.setCurrentIndex(idx)
            self.anchor_combo.blockSignals(False)
