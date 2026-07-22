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
    QVBoxLayout,
    QWidget,
)


class Stage5Panel(QWidget):
    """Stage-5 calibration and hover controls. Emits intent only; no motion logic."""

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

    def __init__(self, *, default_dry_run: bool = True, parent=None) -> None:
        super().__init__(parent)
        self.target_label = QLabel("目标: -")
        self.pixel_label = QLabel("像素: -")
        self.calib_label = QLabel("标定: -")
        self.region_label = QLabel("覆盖区: -")
        self.pwm_label = QLabel("PWM预览: -")
        self.state_label = QLabel("Stage5: DISCONNECTED")
        self.verified_label = QLabel("verified_runs: 0")
        self.source_label = QLabel("来源: -")
        self.serial_sync_label = QLabel("Serial Sync: FALSE")
        self.board_sync_label = QLabel("Board Sync: FALSE")
        self.arm_sync_label = QLabel("Arm Sync: -")
        self.estop_sync_label = QLabel("E-Stop: FALSE")
        self.controller_shared_label = QLabel("Controller Shared: -")
        self.blocked_label = QLabel("Blocked: -")

        self.dry_run_checkbox = QCheckBox("DRY RUN（默认开启，关闭后才允许实机发送）")
        self.dry_run_checkbox.setChecked(bool(default_dry_run))
        self.dry_run_checkbox.toggled.connect(self.dry_run_toggled.emit)

        self.pwm_edits: dict[str, QLineEdit] = {}
        pwm_row = QHBoxLayout()
        for jid in ("000", "001", "002", "003", "004"):
            edit = QLineEdit()
            edit.setPlaceholderText(jid)
            edit.setMaximumWidth(70)
            self.pwm_edits[jid] = edit
            pwm_row.addWidget(QLabel(jid))
            pwm_row.addWidget(edit)

        self.hover_button = QPushButton("悬停到目标点")
        self.return_button = QPushButton("安全返回观察位")
        self.clear_button = QPushButton("清除目标")
        self.set_anchor_button = QPushButton("设为标定点")
        self.load_action_button = QPushButton("从现有动作载入")
        self.save_anchor_button = QPushButton("保存锚点")
        self.test_anchor_button = QPushButton("测试锚点悬停")
        self.confirm_button = QPushButton("确认该锚点安全")
        self.revoke_button = QPushButton("取消安全确认")
        self.load_file_button = QPushButton("加载标定文件")
        self.export_button = QPushButton("导出标定文件")
        self.restore_button = QPushButton("恢复上一个备份")
        self.recover_button = QPushButton("急停后确认恢复")
        self.estop_button = QPushButton("急停")
        self.estop_button.setStyleSheet(
            "QPushButton { background: #b3262e; color: white; font-weight: 700; }"
        )

        self.hover_button.clicked.connect(lambda: self.hover_requested.emit())
        self.return_button.clicked.connect(lambda: self.safe_return_requested.emit())
        self.clear_button.clicked.connect(lambda: self.clear_target_requested.emit())
        self.set_anchor_button.clicked.connect(lambda: self.set_anchor_requested.emit())
        self.load_action_button.clicked.connect(lambda: self.load_from_action_requested.emit())
        self.save_anchor_button.clicked.connect(lambda: self.save_anchor_requested.emit())
        self.test_anchor_button.clicked.connect(lambda: self.test_anchor_hover_requested.emit())
        self.confirm_button.clicked.connect(lambda: self.confirm_anchor_requested.emit())
        self.revoke_button.clicked.connect(lambda: self.revoke_anchor_requested.emit())
        self.load_file_button.clicked.connect(lambda: self.load_calibration_requested.emit())
        self.export_button.clicked.connect(lambda: self.export_calibration_requested.emit())
        self.restore_button.clicked.connect(lambda: self.restore_backup_requested.emit())
        self.recover_button.clicked.connect(lambda: self.recover_requested.emit())
        self.estop_button.clicked.connect(lambda: self.estop_requested.emit())

        info = QFormLayout()
        info.addRow(self.state_label)
        info.addRow(self.target_label)
        info.addRow(self.pixel_label)
        info.addRow(self.calib_label)
        info.addRow(self.region_label)
        info.addRow(self.source_label)
        info.addRow(self.pwm_label)
        info.addRow(self.verified_label)
        info.addRow(self.serial_sync_label)
        info.addRow(self.board_sync_label)
        info.addRow(self.arm_sync_label)
        info.addRow(self.estop_sync_label)
        info.addRow(self.controller_shared_label)
        info.addRow(self.blocked_label)

        move_row = QHBoxLayout()
        move_row.addWidget(self.hover_button)
        move_row.addWidget(self.return_button)
        move_row.addWidget(self.clear_button)

        calib_grid = QGridLayout()
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
            self.recover_button,
        ]
        for index, button in enumerate(buttons):
            calib_grid.addWidget(button, index // 2, index % 2)

        layout = QVBoxLayout(self)
        title = QLabel("阶段五：任意交点安全悬停")
        title.setStyleSheet("font-weight: 700; font-size: 14px;")
        layout.addWidget(title)
        layout.addLayout(info)
        layout.addWidget(self.dry_run_checkbox)
        layout.addLayout(pwm_row)
        layout.addLayout(move_row)
        layout.addLayout(calib_grid)
        layout.addWidget(self.estop_button)

        self.set_enabled_state(
            serial_connected=False,
            board_locked=False,
            busy=False,
            can_hover=False,
            can_return=False,
            has_target=False,
            estop=False,
        )

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
        self.state_label.setText(f"Stage5: {state}")
        if row is None or col is None:
            self.target_label.setText("目标: -")
        else:
            self.target_label.setText(f"目标: P({row},{col})")
        if pixel_x is None or pixel_y is None:
            self.pixel_label.setText("像素: -")
        else:
            self.pixel_label.setText(f"像素: ({pixel_x:.1f}, {pixel_y:.1f})")
        self.calib_label.setText(f"标定: {calibrated}")
        self.region_label.setText(f"覆盖区: {region}")
        self.source_label.setText(f"来源: {source}")
        self.pwm_label.setText(f"PWM预览: {pwm_text}")
        self.verified_label.setText(f"verified_runs: {verified_runs}")


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
        # DRY RUN toggle must remain operable for inspection even when DISCONNECTED.
        # Always allow inspecting/toggling DRY RUN, including DISCONNECTED.
        self.dry_run_checkbox.setEnabled(True)
        self.hover_button.setEnabled(ordinary and can_hover)
        self.return_button.setEnabled(ordinary and can_return)
        self.clear_button.setEnabled(ordinary and has_target)
        for button in (
            self.set_anchor_button,
            self.load_action_button,
            self.save_anchor_button,
            self.test_anchor_button,
            self.confirm_button,
            self.revoke_button,
            self.load_file_button,
            self.export_button,
            self.restore_button,
        ):
            button.setEnabled(ordinary and has_target)
        self.load_file_button.setEnabled(not busy)
        self.export_button.setEnabled(not busy)
        self.restore_button.setEnabled(not busy)
        self.load_action_button.setEnabled(not busy)
        self.recover_button.setEnabled(serial_connected and estop and not busy)
        self.estop_button.setEnabled(serial_connected)
        for edit in self.pwm_edits.values():
            edit.setEnabled(not busy)
