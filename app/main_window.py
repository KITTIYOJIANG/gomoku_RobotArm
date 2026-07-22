from __future__ import annotations

import logging

from PySide6.QtCore import Qt
from PySide6.QtGui import QCloseEvent
from PySide6.QtWidgets import QHBoxLayout, QMainWindow, QMessageBox, QSplitter, QWidget

from app.arm.actions import ActionLibrary
from app.arm.controller import SerialArmController
from app.arm.sequences import (
    ActionStep,
    SequenceDefinition,
    pick_piece,
    place_to_p77,
    return_to_observe,
    run_full_cycle,
)
from app.arm.state import ArmState, ArmStateMachine, InvalidTransition
from app.arm.worker import ArmSequenceWorker
from app.config import AppConfig
from app.gui.camera_panel import CameraPanel
from app.gui.control_panel import ControlPanel
from app.logging_config import LogEmitter, QtLogHandler
from app.vision.camera_worker import CameraWorker


LOGGER = logging.getLogger(__name__)


class MainWindow(QMainWindow):
    def __init__(
        self,
        config: AppConfig,
        *,
        dry_run: bool = False,
        default_test_pattern: bool = False,
        parent=None,
    ) -> None:
        super().__init__(parent)
        self.config = config
        self.dry_run = bool(dry_run)
        self.default_test_pattern = bool(default_test_pattern)
        self.board_locked = False
        self.target_visible = False
        self.board_display_status = "BOARD LOST"
        self.corner_status = "0/4 LOST"
        self.piece_status = "NOT RUN"
        self.camera_state = "DISCONNECTED"
        self.camera_worker: CameraWorker | None = None

        self.actions = ActionLibrary()
        self.controller = SerialArmController(
            baudrate=config.serial.baudrate,
            write_timeout_seconds=config.serial.write_timeout_seconds,
            dry_run=self.dry_run,
        )
        self.state_machine = ArmStateMachine()
        self.arm_worker = ArmSequenceWorker(
            self.controller,
            self.actions,
            action_wait_margin_ms=config.timing.action_wait_margin_ms,
        )

        self.camera_panel = CameraPanel()
        self.control_panel = ControlPanel(
            default_port=config.serial.default_port,
            dry_run=self.dry_run,
            default_test_pattern=self.default_test_pattern,
        )
        splitter = QSplitter(Qt.Orientation.Horizontal)
        splitter.addWidget(self.camera_panel)
        splitter.addWidget(self.control_panel)
        splitter.setStretchFactor(0, 65)
        splitter.setStretchFactor(1, 35)
        splitter.setSizes([1100, 590])
        root = QWidget()
        layout = QHBoxLayout(root)
        layout.addWidget(splitter)
        self.setCentralWidget(root)
        self.setWindowTitle("J1 五子棋机械臂整合开发版 V0.1" + (" — DRY RUN" if self.dry_run else ""))
        self.resize(1680, 980)

        self.log_emitter = LogEmitter(self)
        self.log_handler = QtLogHandler(self.log_emitter)
        logging.getLogger().addHandler(self.log_handler)
        self.log_emitter.message.connect(self.control_panel.log_panel.append_message)

        self._connect_signals()
        self.arm_worker.start()
        self._refresh_ui()
        LOGGER.info("APPLICATION READY dry_run=%s; no automatic camera/COM connection", self.dry_run)

    def _connect_signals(self) -> None:
        panel = self.control_panel
        panel.connect_camera_requested.connect(self.connect_camera)
        panel.disconnect_camera_requested.connect(self.disconnect_camera)
        panel.connect_serial_requested.connect(self.connect_serial)
        panel.disconnect_serial_requested.connect(self.disconnect_serial)
        panel.return_observe_requested.connect(self.start_return_to_observe)
        panel.pick_requested.connect(self.start_pick)
        panel.place_requested.connect(self.start_place)
        panel.full_cycle_requested.connect(self.start_full_cycle)
        panel.manual_action_requested.connect(self.start_manual)
        panel.estop_requested.connect(self.emergency_stop)
        panel.pump_off_requested.connect(self.pump_off)
        panel.corner_overlay_options_changed.connect(self.set_corner_overlay_options)
        panel.piece_recognition_requested.connect(self.request_piece_recognition)

        self.arm_worker.sequence_started.connect(self._on_sequence_started)
        self.arm_worker.step_started.connect(self._on_step_started)
        self.arm_worker.sequence_finished.connect(self._on_sequence_finished)
        self.arm_worker.log_message.connect(lambda message: LOGGER.info("%s", message))

    def connect_camera(self) -> None:
        if self.camera_worker is not None and self.camera_worker.isRunning():
            return
        worker = CameraWorker(
            self.config,
            test_pattern=self.control_panel.camera_uses_test_pattern(),
            dry_run=self.dry_run,
        )
        worker.frame_ready.connect(self.camera_panel.set_frame)
        worker.camera_status.connect(self._on_camera_status)
        worker.board_status.connect(self._on_board_status)
        worker.piece_status.connect(self._on_piece_status)
        worker.error.connect(lambda message: LOGGER.error("CAMERA %s", message))
        worker.finished.connect(self._on_camera_finished)
        self.camera_worker = worker
        worker.set_corner_overlay_options(*self.control_panel.corner_overlay_options())
        self.control_panel.set_camera_connected(True)
        self.camera_state = "CONNECTING"
        self._refresh_ui()
        worker.start()
        LOGGER.info("CAMERA CONNECT REQUESTED")

    def disconnect_camera(self) -> None:
        worker = self.camera_worker
        if worker is None:
            return
        worker.stop()
        if worker.isRunning() and not worker.wait(2500):
            LOGGER.error("CAMERA worker did not stop within 2500ms")
        self.camera_worker = None
        self.camera_state = "DISCONNECTED"
        self.board_locked = False
        self.target_visible = False
        self.board_display_status = "BOARD LOST"
        self.corner_status = "0/4 LOST"
        self.piece_status = "NOT RUN"
        self.camera_panel.set_camera_status("DISCONNECTED")
        self.camera_panel.set_board_status("BOARD LOST", "BOARD LOST")
        self.control_panel.set_camera_connected(False)
        self._refresh_ui()
        LOGGER.info("CAMERA DISCONNECTED")

    def connect_serial(self, port: str) -> None:
        try:
            self.controller.connect(port)
            transition = self.state_machine.connect()
            self._log_transition(transition)
        except Exception as exc:
            LOGGER.error("SERIAL CONNECT FAILED: %s", exc)
            QMessageBox.critical(self, "串口连接失败", str(exc))
        self._refresh_ui()

    def disconnect_serial(self) -> None:
        if self.arm_worker.busy:
            self.arm_worker.cancel_pending()
        self.controller.disconnect()
        transition = self.state_machine.disconnect()
        self._log_transition(transition)
        self._set_camera_arm_busy(False)
        self._refresh_ui()

    def start_return_to_observe(self) -> None:
        if self.state_machine.state == ArmState.OBSERVE_HOLD:
            answer = QMessageBox.warning(
                self,
                "吸气状态确认",
                "回观察位动作使用 OBSERVE_IDLE，会关闭吸气。请确认棋子不会掉落到危险位置。",
                QMessageBox.StandardButton.Ok | QMessageBox.StandardButton.Cancel,
                QMessageBox.StandardButton.Cancel,
            )
            if answer != QMessageBox.StandardButton.Ok:
                return
        self._start_sequence(return_to_observe(), self.state_machine.begin_return_to_observe)

    def start_pick(self) -> None:
        sequence = pick_piece(self.config.timing.vacuum_build_ms)
        self._start_sequence(sequence, self.state_machine.begin_pick)

    def start_place(self) -> None:
        sequence = place_to_p77(self.config.timing.release_ms)
        self._start_sequence(
            sequence,
            lambda: self.state_machine.begin_place(
                board_locked=self.board_locked,
                target_visible=self.target_visible,
            ),
        )

    def start_full_cycle(self) -> None:
        answer = QMessageBox.warning(
            self,
            "实验功能确认",
            "完整固定点流程将连续执行取料和 P77 放棋。请确认机械臂周围无障碍。",
            QMessageBox.StandardButton.Ok | QMessageBox.StandardButton.Cancel,
            QMessageBox.StandardButton.Cancel,
        )
        if answer != QMessageBox.StandardButton.Ok:
            return
        sequence = run_full_cycle(
            self.config.timing.vacuum_build_ms,
            self.config.timing.release_ms,
        )
        self._start_sequence(
            sequence,
            lambda: self.state_machine.begin_full_cycle(
                board_locked=self.board_locked,
                target_visible=self.target_visible,
            ),
        )

    def start_manual(self, action_name: str) -> None:
        answer = QMessageBox.warning(
            self,
            "单姿态动作确认",
            f"即将执行机械臂单姿态动作 {action_name}，请确认周围无障碍。",
            QMessageBox.StandardButton.Ok | QMessageBox.StandardButton.Cancel,
            QMessageBox.StandardButton.Cancel,
        )
        if answer != QMessageBox.StandardButton.Ok:
            return
        sequence = SequenceDefinition(
            name=f"MANUAL:{action_name}",
            display_name=f"手动 {action_name}",
            steps=(ActionStep(action_name),),
        )
        self._start_sequence(sequence, lambda: self.state_machine.begin_manual(action_name))

    def _start_sequence(self, sequence: SequenceDefinition, begin) -> None:
        if not self.controller.is_connected:
            QMessageBox.warning(self, "未连接", "请先连接 COM（dry-run 中为模拟连接）。")
            return
        if self.arm_worker.busy:
            QMessageBox.warning(self, "动作忙", "已有机械臂动作正在执行。")
            return
        try:
            transition = begin()
            if transition is not None:
                self._log_transition(transition)
            if not self.arm_worker.submit(sequence):
                raise RuntimeError("ArmSequenceWorker rejected a concurrent action")
        except (InvalidTransition, RuntimeError) as exc:
            if self.state_machine.busy and not self.arm_worker.busy:
                self._log_transition(self.state_machine.fail(str(exc)))
            LOGGER.error("ACTION REJECTED %s: %s", sequence.name, exc)
            QMessageBox.warning(self, "动作被拒绝", str(exc))
            self._refresh_ui()
            return
        self._set_camera_arm_busy(True)
        self._refresh_ui()

    def emergency_stop(self) -> None:
        self.arm_worker.cancel_pending()
        try:
            self.controller.emergency_stop()
        except Exception as exc:
            LOGGER.error("ESTOP WRITE FAILED: %s", exc)
        transition = self.state_machine.estop()
        self._log_transition(transition)
        self._set_camera_arm_busy(False)
        LOGGER.critical("EMERGENCY STOP LATCHED")
        self._refresh_ui()

    def pump_off(self) -> None:
        if self.arm_worker.busy:
            self.arm_worker.cancel_pending()
        try:
            self.controller.pump_off()
            LOGGER.warning("PUMP OFF SENT")
            if self.state_machine.state not in (ArmState.ESTOP, ArmState.DISCONNECTED):
                self._log_transition(self.state_machine.fail("Pump was manually turned off; pose/state unknown"))
        except Exception as exc:
            LOGGER.error("PUMP OFF FAILED: %s", exc)
            QMessageBox.critical(self, "气泵关闭失败", str(exc))
        self._set_camera_arm_busy(False)
        self._refresh_ui()

    def set_corner_overlay_options(self, show_corners: bool, show_coordinates: bool) -> None:
        if self.camera_worker is not None:
            self.camera_worker.set_corner_overlay_options(show_corners, show_coordinates)
        LOGGER.info(
            "CORNER OVERLAY show=%s coordinates=%s",
            show_corners,
            show_coordinates,
        )

    def request_piece_recognition(self) -> None:
        if self.camera_worker is None:
            QMessageBox.warning(self, "摄像头未连接", "请先连接摄像头。")
            return
        self.piece_status = "PENDING"
        self.camera_worker.request_piece_recognition()
        LOGGER.info("PIECE RECOGNITION REQUESTED")
        self._refresh_ui()

    def _on_sequence_started(self, name: str, display_name: str) -> None:
        LOGGER.info("SEQUENCE START %s (%s)", name, display_name)
        self._refresh_ui()

    def _on_step_started(self, sequence_name: str, step_name: str) -> None:
        if sequence_name == "FULL_CYCLE" and step_name == "CARRY_HIGH_P77_HOLD":
            try:
                self._log_transition(self.state_machine.mark_full_cycle_placing())
            except InvalidTransition as exc:
                LOGGER.error("FULL CYCLE STATE ERROR: %s", exc)
        LOGGER.info("STEP %s -> %s", sequence_name, step_name)
        self._refresh_ui(current_action=step_name)

    def _on_sequence_finished(self, name: str, success: bool, message: str) -> None:
        if self.state_machine.state == ArmState.ESTOP:
            LOGGER.warning("SEQUENCE %s ended after ESTOP: %s", name, message)
        elif success:
            try:
                if name == "RETURN_TO_OBSERVE":
                    transition = self.state_machine.complete_return_to_observe()
                elif name == "PICK_PIECE":
                    transition = self.state_machine.complete_pick()
                elif name == "PLACE_TO_P77":
                    transition = self.state_machine.complete_place()
                elif name == "FULL_CYCLE":
                    transition = self.state_machine.complete_full_cycle()
                elif name.startswith("MANUAL:"):
                    transition = self.state_machine.complete_manual()
                else:
                    raise InvalidTransition(f"Unknown completed sequence {name}")
                self._log_transition(transition)
            except InvalidTransition as exc:
                self._log_transition(self.state_machine.fail(str(exc)))
        else:
            self._log_transition(self.state_machine.fail(message or f"{name} failed"))

        self._set_camera_arm_busy(False)
        if success and name in {"RETURN_TO_OBSERVE", "PLACE_TO_P77", "FULL_CYCLE"}:
            self.board_locked = False
            if self.camera_worker is not None:
                self.camera_worker.request_relocalize()
                if name in {"PLACE_TO_P77", "FULL_CYCLE"}:
                    self.piece_status = "PENDING"
                    self.camera_worker.request_piece_recognition()
            LOGGER.info("VISION RELOCALIZATION REQUESTED after %s", name)
        self._refresh_ui()

    def _on_camera_status(self, status: str) -> None:
        self.camera_state = status
        self.camera_panel.set_camera_status(status)
        self._refresh_ui()

    def _on_board_status(
        self,
        locked: bool,
        reason: str,
        target_visible: bool,
        display_status: str,
        corner_status: str,
    ) -> None:
        changed = display_status != self.board_display_status
        self.board_locked = bool(locked)
        self.target_visible = bool(target_visible)
        self.board_display_status = display_status
        self.corner_status = corner_status
        self.camera_panel.set_board_status(display_status, reason)
        if changed:
            LOGGER.info("APRILTAG %s corners=%s: %s", display_status, corner_status, reason)
        self._refresh_ui()

    def _on_piece_status(self, summary: str, board_matrix: object) -> None:
        self.piece_status = summary
        LOGGER.info("PIECE MATRIX %s", board_matrix)
        self._refresh_ui()

    def _on_camera_finished(self) -> None:
        self.camera_state = "DISCONNECTED"
        self.board_locked = False
        self.target_visible = False
        self.board_display_status = "BOARD LOST"
        self.corner_status = "0/4 LOST"
        self.piece_status = "NOT RUN"
        self.camera_panel.set_camera_status("DISCONNECTED")
        self.camera_panel.set_board_status("BOARD LOST", "BOARD LOST")
        self.control_panel.set_camera_connected(False)
        self.camera_worker = None
        self._refresh_ui()

    def _set_camera_arm_busy(self, busy: bool) -> None:
        if self.camera_worker is not None:
            self.camera_worker.set_arm_busy(busy)

    def _log_transition(self, transition: tuple[ArmState, ArmState]) -> None:
        previous, target = transition
        LOGGER.info("STATE %s -> %s", previous.value, target.value)

    def _refresh_ui(self, *, current_action: str | None = None) -> None:
        snapshot = self.state_machine.snapshot()
        action = current_action or snapshot.current_action or "-"
        com = (
            f"DRY-RUN {self.controller.port or ''}".strip()
            if self.dry_run and self.controller.is_connected
            else (f"CONNECTED {self.controller.port}" if self.controller.is_connected else "DISCONNECTED")
        )
        self.control_panel.status_panel.update_values(
            camera=self.camera_state,
            com=com,
            board=self.board_display_status,
            corners=self.corner_status,
            pieces=self.piece_status,
            arm=snapshot.state.value,
            action=action,
        )
        self.control_panel.update_controls(
            connected=self.controller.is_connected,
            state=snapshot.state,
            busy=snapshot.busy or self.arm_worker.busy,
            board_locked=self.board_locked,
            target_visible=self.target_visible,
        )

    def closeEvent(self, event: QCloseEvent) -> None:  # noqa: N802 - Qt API
        if self.arm_worker.busy and self.controller.is_connected:
            self.arm_worker.cancel_pending()
            try:
                self.controller.emergency_stop()
                LOGGER.warning("ESTOP sent because the GUI closed during an active action")
            except Exception as exc:
                LOGGER.error("Failed to send close-time ESTOP: %s", exc)
        self.disconnect_camera()
        self.arm_worker.shutdown()
        self.controller.close()
        logging.getLogger().removeHandler(self.log_handler)
        event.accept()
