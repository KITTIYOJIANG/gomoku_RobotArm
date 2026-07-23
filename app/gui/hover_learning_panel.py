from __future__ import annotations

from pathlib import Path

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QFormLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)


class HoverLearningPanel(QWidget):
    """Shadow learning panel. Emits intents only; never sends arm motion."""

    sync_p77_preview_requested = Signal()
    sync_p77_apply_requested = Signal()
    train_smoke_requested = Signal()
    load_model_requested = Signal()
    predict_current_target_requested = Signal()
    compare_requested = Signal()
    inspect_dataset_requested = Signal()

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.title = QLabel("阶段五学习层（影子预测，不控制机械臂）")
        self.title.setStyleSheet("font-weight:700;font-size:14px;")
        self.live_label = QLabel("MODEL_LIVE_CONTROL_ENABLED=false")
        self.live_label.setStyleSheet("color:#b3262e;font-weight:700;")
        self.dataset_label = QLabel("样本: -")
        self.model_label = QLabel("模型: 未加载")
        self.generalization_label = QLabel("generalization_valid: -")
        self.shadow_label = QLabel("影子PWM: -")
        self.preferred_label = QLabel("优先来源: -")
        self.delta_label = QLabel("与优先来源最大差: -")
        self.log = QTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumHeight(140)

        form = QFormLayout()
        form.addRow(self.dataset_label)
        form.addRow(self.model_label)
        form.addRow(self.generalization_label)
        form.addRow(self.shadow_label)
        form.addRow(self.preferred_label)
        form.addRow(self.delta_label)

        row1 = QHBoxLayout()
        b1 = QPushButton("预览同步P77样本")
        b2 = QPushButton("应用同步P77样本")
        b1.clicked.connect(self.sync_p77_preview_requested.emit)
        b2.clicked.connect(self.sync_p77_apply_requested.emit)
        row1.addWidget(b1)
        row1.addWidget(b2)

        row2 = QHBoxLayout()
        b3 = QPushButton("冒烟训练(--smoke-test)")
        b4 = QPushButton("加载最新模型")
        b3.clicked.connect(self.train_smoke_requested.emit)
        b4.clicked.connect(self.load_model_requested.emit)
        row2.addWidget(b3)
        row2.addWidget(b4)

        row3 = QHBoxLayout()
        b5 = QPushButton("影子预测当前目标")
        b6 = QPushButton("三来源对比")
        b7 = QPushButton("检查数据集")
        b5.clicked.connect(self.predict_current_target_requested.emit)
        b6.clicked.connect(self.compare_requested.emit)
        b7.clicked.connect(self.inspect_dataset_requested.emit)
        row3.addWidget(b5)
        row3.addWidget(b6)
        row3.addWidget(b7)

        layout = QVBoxLayout(self)
        layout.addWidget(self.title)
        layout.addWidget(self.live_label)
        layout.addLayout(form)
        layout.addLayout(row1)
        layout.addLayout(row2)
        layout.addLayout(row3)
        layout.addWidget(self.log)

    def append_log(self, text: str) -> None:
        self.log.append(text)

    def update_status(
        self,
        *,
        n_samples: int,
        n_coords: int,
        model_loaded: bool,
        generalization_valid: bool | None,
        shadow_text: str,
        preferred: str,
        delta_text: str,
    ) -> None:
        self.dataset_label.setText(f"样本: {n_samples}  独立棋位: {n_coords}")
        self.model_label.setText("模型: 已加载" if model_loaded else "模型: 未加载")
        if generalization_valid is None:
            self.generalization_label.setText("generalization_valid: -")
        else:
            self.generalization_label.setText(f"generalization_valid: {str(generalization_valid).lower()}")
        self.shadow_label.setText(f"影子PWM: {shadow_text}")
        self.preferred_label.setText(f"优先来源: {preferred}")
        self.delta_label.setText(f"与优先来源最大差: {delta_text}")
