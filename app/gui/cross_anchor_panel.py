
from __future__ import annotations

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QComboBox,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from app.stage5.constants import CROSS_ANCHORS, SPATIAL_JOINTS


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

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.progress_label = QLabel("中心十字标定：-")
        self.state_label = QLabel("Wizard: IDLE")
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
        self.anchor_combo.currentIndexChanged.connect(self.select_index_requested.emit)

        self.ref_label = QLabel("P77参考: -")
        self.delta_label = QLabel("差值: -")
        self.runs_label = QLabel("已验证 0 / 3")
        self.force_label = QLabel("FORCE_STAGE5_DRY_RUN=TRUE")
        self.force_label.setStyleSheet("color:#b3262e;font-weight:700;")
        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumHeight(120)

        self.joint_edits: dict[str, QLineEdit] = {}
        joint_grid = QGridLayout()
        joint_grid.addWidget(QLabel("关节"), 0, 0)
        joint_grid.addWidget(QLabel("候选"), 0, 1)
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
                b.clicked.connect(lambda _=False, j=jid, d=delta: self.nudge_requested.emit(j, d))
                btn_row.addWidget(b)
            wrap = QWidget()
            wrap.setLayout(btn_row)
            joint_grid.addWidget(wrap, i, 2)

        apply_btn = QPushButton("应用输入框数值")
        apply_btn.clicked.connect(self._emit_set_joints)

        nav = QHBoxLayout()
        prev_b = QPushButton("上一个锚点")
        next_b = QPushButton("下一个锚点")
        prev_b.clicked.connect(self.prev_requested.emit)
        next_b.clicked.connect(self.next_requested.emit)
        nav.addWidget(prev_b)
        nav.addWidget(next_b)

        actions = QGridLayout()
        buttons = [
            ("恢复P77参考", self.reset_p77_requested),
            ("撤销", self.undo_requested),
            ("保存草稿", self.save_draft_requested),
            ("载入草稿", self.load_draft_requested),
            ("安全检查", self.validate_requested),
            ("生成运输高位计划", self.plan_carry_requested),
            ("生成目标上方计划", self.plan_target_requested),
            ("生成安全返回计划", self.plan_return_requested),
            ("执行DRY RUN/MOCK", self.execute_mock_requested),
            ("结果:位置正确安全", lambda: self.result_requested.emit("SAFE_OK")),
            ("结果:安全但偏移", lambda: self.result_requested.emit("SAFE_WITH_OFFSET")),
            ("结果:不安全", lambda: self.result_requested.emit("UNSAFE")),
            ("结果:急停", lambda: self.result_requested.emit("ESTOP")),
            ("结果:未完成", lambda: self.result_requested.emit("INCOMPLETE")),
            ("确认锚点完成", self.complete_requested),
            ("取消安全确认", self.cancel_confirm_requested),
        ]
        for idx, (text, sig) in enumerate(buttons):
            b = QPushButton(text)
            if hasattr(sig, "emit"):
                b.clicked.connect(sig.emit)
            else:
                b.clicked.connect(sig)
            actions.addWidget(b, idx // 2, idx % 2)

        layout = QVBoxLayout(self)
        title = QLabel("中心十字锚点标定向导")
        title.setStyleSheet("font-weight:700;font-size:14px;")
        layout.addWidget(title)
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
        layout.addWidget(QLabel("向导日志"))
        layout.addWidget(self.log_view)

    def _emit_set_joints(self) -> None:
        for jid, edit in self.joint_edits.items():
            text = edit.text().strip()
            if not text:
                continue
            self.set_joint_requested.emit(jid, int(text))

    def append_log(self, message: str) -> None:
        self.log_view.append(message)

    def update_view(self, snap: dict) -> None:
        self.progress_label.setText(str(snap.get("progress", "-")))
        self.state_label.setText(f"Wizard: {snap.get('state', '-')}")
        self.runs_label.setText(
            f"已验证 {snap.get('verified_runs', 0)} / {snap.get('required_runs', 3)}"
        )
        ref = snap.get("reference_pwm") or {}
        cand = snap.get("candidate_pwm") or {}
        deltas = snap.get("deltas") or {}
        self.ref_label.setText(
            "P77参考: " + ", ".join(f"{k}={ref.get(k,'-')}" for k in SPATIAL_JOINTS)
        )
        self.delta_label.setText(
            "差值: " + ", ".join(f"{k}:{deltas.get(k,0):+d}" for k in SPATIAL_JOINTS)
        )
        for jid in SPATIAL_JOINTS:
            if jid in cand:
                self.joint_edits[jid].setText(str(cand[jid]))
        status_map = snap.get("status_map") or {}
        for row, col, _label, _cn in CROSS_ANCHORS:
            key = f"{row},{col}"
            st = status_map.get(key, "EMPTY")
            if st in {"COMPLETED"}:
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
