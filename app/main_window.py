from __future__ import annotations

import logging
from pathlib import Path

from PySide6.QtCore import Qt, QTimer
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
from app.stage5.coordinator import Stage5Coordinator
from app.stage5.cross_anchor_wizard import CrossAnchorWizard, UserTestResult
from app.stage5.candidate_store import CandidateStore
from app.learning.hover_sample_store import HoverSampleStore
from app.learning.hover_dataset import VerifiedHoverPoseDataset
from app.learning.hover_predictor import HoverPosePredictor
from app.learning.hover_comparator import HoverPoseComparator
from app.learning.hover_trainer import TrainConfig, train_hover_pose
from app.learning import MODEL_LIVE_CONTROL_ENABLED
from app.stage5.safety import derive_pwm_safety_limits
from app.stage5.constants import FORCE_STAGE5_DRY_RUN
from app.stage5.state_machine import Stage5Invalid, Stage5State
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
        self.stage5 = Stage5Coordinator(
            config=config.stage5,
            actions=self.actions,
            controller=self.controller,
            arm_state=self.state_machine,
            worker=self.arm_worker,
            action_wait_margin_ms=config.timing.action_wait_margin_ms,
            logs_dir=config.logs_dir,
        )
        same = self.stage5.controller is self.controller
        LOGGER.info(
            "[STAGE5][CONTROLLER_ID] main_controller_id=%s stage5_controller_id=%s same_instance=%s",
            id(self.controller),
            id(self.stage5.controller),
            int(same),
        )
        if not same:
            raise RuntimeError("Stage5 must share MainWindow SerialArmController instance")
        self._selected_target_freeze = False
        draft_path = getattr(config.stage5, "cross_draft_path", None) or (config.logs_dir.parent / "calibration" / "stage5_cross_anchor_drafts.json")
        sample_path = getattr(config.stage5, "hover_samples_path", None) or (config.logs_dir.parent / "datasets" / "hover_pose" / "verified_samples.jsonl")
        # Offline demo must not pollute production calibration: wizard writes anchors only on complete.
        self.cross_wizard = CrossAnchorWizard(
            library=self.actions,
            calibration=self.stage5.store,
            drafts=CandidateStore(draft_path),
            samples=HoverSampleStore(sample_path),
            required_runs=int(getattr(config.stage5, "cross_anchor_required_runs", 3)),
            force_dry_run=bool(getattr(config.stage5, "force_dry_run", FORCE_STAGE5_DRY_RUN)),
        )
        self._cross_last_plan = None
        self._samples_path = Path(sample_path)
        self._model_dir = self.config.logs_dir.parent / "models" / "hover_pose"
        self.hover_predictor = HoverPosePredictor(
            model_path=self._model_dir / "hover_pose_net_latest.pt",
            normalizer_path=self._model_dir / "hover_normalizer_latest.json",
            board_size=self.config.vision.board_size,
            limits=derive_pwm_safety_limits(self.actions),
        )
        self.hover_comparator = HoverPoseComparator(self.stage5.store, limits=derive_pwm_safety_limits(self.actions))
        LOGGER.info("[STAGE5][FORCE_DRY_RUN] enabled=%s", int(FORCE_STAGE5_DRY_RUN))
        LOGGER.info("[LEARNING] MODEL_LIVE_CONTROL_ENABLED=%s", MODEL_LIVE_CONTROL_ENABLED)

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
        # Stage5 is created after controller; immediately pull current main context
        # so late construction never misses an already-connected COM/board/arm state.
        self._sync_stage5_context(reason="init")
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

        self.camera_panel.image_clicked.connect(self.on_image_clicked)
        s5 = panel.stage5_panel
        s5.dry_run_toggled.connect(self.on_stage5_dry_run)
        s5.hover_requested.connect(self.start_stage5_hover)
        s5.safe_return_requested.connect(self.start_stage5_safe_return)
        s5.clear_target_requested.connect(self.clear_stage5_target)
        s5.set_anchor_requested.connect(self.stage5_set_anchor_from_target)
        s5.load_from_action_requested.connect(self.stage5_load_from_action)
        s5.save_anchor_requested.connect(self.stage5_save_anchor)
        s5.test_anchor_hover_requested.connect(self.start_stage5_hover)
        s5.confirm_anchor_requested.connect(self.stage5_confirm_anchor)
        s5.revoke_anchor_requested.connect(self.stage5_revoke_anchor)
        s5.load_calibration_requested.connect(self.stage5_reload_calibration)
        s5.export_calibration_requested.connect(self.stage5_export_calibration)
        s5.restore_backup_requested.connect(self.stage5_restore_backup)
        s5.recover_requested.connect(self.stage5_recover)
        s5.estop_requested.connect(self.emergency_stop)
        self._connect_cross_anchor_signals()
        self._connect_learning_signals()

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
        worker.board_geometry.connect(self._on_board_geometry)
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
            LOGGER.info("[STAGE5][SERIAL_SYNC] connected=1 port=%s", port)
        except Exception as exc:
            LOGGER.error("SERIAL CONNECT FAILED: %s", exc)
            QMessageBox.critical(self, "串口连接失败", str(exc))
            self._sync_stage5_context(reason="serial_connect_failed")
            self._refresh_ui()
            return
        self._sync_stage5_context(reason="serial_connected")
        self._refresh_ui()

    def disconnect_serial(self) -> None:
        if self.arm_worker.busy:
            self.arm_worker.cancel_pending()
        self.controller.disconnect()
        transition = self.state_machine.disconnect()
        self._log_transition(transition)
        self._set_camera_arm_busy(False)
        LOGGER.info("[STAGE5][SERIAL_SYNC] connected=0")
        self._sync_stage5_context(reason="serial_disconnected")
        self._refresh_ui()

    def start_return_to_observe(self) -> None:
        # From TARGET_ABOVE/HOVERING, never jump directly to OBSERVE.
        if self.state_machine.snapshot().state == ArmState.HOVERING:
            self.start_stage5_safe_return()
            return
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
        self.stage5.estop()
        self._sync_stage5_context(reason="estop")
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
        if self.stage5.on_sequence_finished(name, success, message):
            self._set_camera_arm_busy(False)
            if success and name == "SAFE_RETURN_FROM_HOVER":
                self.board_locked = False
                if self.camera_worker is not None:
                    self.camera_worker.request_relocalize()
            self._refresh_stage5_ui()
            self._refresh_ui()
            return
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

        self._sync_stage5_context(reason=f"sequence_finished:{name}:{success}")
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
        LOGGER.info("[STAGE5][BOARD_SYNC] locked=%s", int(bool(locked)))
        self._sync_stage5_context(reason="board_status")
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
        LOGGER.info("[STAGE5][ARM_SYNC] state=%s", target.value)
        # Keep Stage5 arm context aligned with every main arm transition.
        self._sync_stage5_context(reason=f"arm_transition:{previous.value}->{target.value}")

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
        # Always re-pull Stage5 context from the live main system and push to the panel.
        # Without this, Stage5 can be READY internally while the UI stays DISCONNECTED.
        self._sync_stage5_context(reason="refresh_ui")
        self._refresh_stage5_ui()


    def _on_board_geometry(self, payload: object) -> None:
        if isinstance(payload, dict):
            self.stage5.update_geometry(payload)

    def on_image_clicked(self, image_x: float, image_y: float) -> None:
        if self.stage5.stage_state.is_moving() or self.arm_worker.busy:
            LOGGER.info("STAGE5 click ignored while arm busy")
            return
        selection = self.stage5.handle_click(image_x, image_y, self.config.vision.board_size)
        if selection.accepted and selection.row is not None and selection.col is not None:
            if self.camera_worker is not None:
                self.camera_worker.set_selected_target(selection.row, selection.col)
            self.camera_panel.set_target_text(f"目标: P({selection.row},{selection.col})")
        else:
            LOGGER.info("STAGE5 %s", selection.reason)
        self._refresh_stage5_ui()
        self._refresh_ui()

    def on_stage5_dry_run(self, enabled: bool) -> None:
        try:
            self.stage5.set_dry_run(bool(enabled))
        except Stage5Invalid as exc:
            LOGGER.error("%s", exc)
            self.control_panel.stage5_panel.dry_run_checkbox.blockSignals(True)
            self.control_panel.stage5_panel.dry_run_checkbox.setChecked(self.stage5.stage_state.snapshot().dry_run)
            self.control_panel.stage5_panel.dry_run_checkbox.blockSignals(False)
        self._refresh_stage5_ui()

    def clear_stage5_target(self) -> None:
        try:
            self.stage5.clear_target()
        except Stage5Invalid as exc:
            LOGGER.error("%s", exc)
            return
        if self.camera_worker is not None:
            self.camera_worker.set_selected_target(None, None)
        self.camera_panel.set_target_text("当前目标坐标：-")
        self._refresh_stage5_ui()
        self._refresh_ui()

    def start_stage5_hover(self) -> None:
        try:
            holding = self.state_machine.snapshot().state == ArmState.OBSERVE_HOLD
            plan = self.stage5.plan_hover(holding_piece=holding)
            LOGGER.info(
                "STAGE5 HOVER plan target=P(%s,%s) source=%s dry_run=%s duration_ms=%s commands=%s",
                plan.target_row,
                plan.target_col,
                plan.source,
                plan.dry_run,
                plan.estimated_duration_ms,
                plan.serial_commands,
            )
            submitted, mode = self.stage5.begin_hover_execution(plan)
            self._set_camera_arm_busy(True)
            if not submitted:
                # Software dry-run completion after estimated duration (non-blocking).
                QTimer.singleShot(max(50, plan.estimated_duration_ms), self._finish_stage5_hover_dry_run)
            self._refresh_stage5_ui()
            self._refresh_ui(current_action="HOVER_TO_TARGET")
        except Exception as exc:
            LOGGER.error("STAGE5 hover rejected: %s", exc)
            QMessageBox.warning(self, "阶段五悬停", str(exc))
            self._refresh_stage5_ui()
            self._refresh_ui()

    def _finish_stage5_hover_dry_run(self) -> None:
        if self.stage5.stage_state.state not in {
            Stage5State.MOVING_TO_CARRY_HIGH,
            Stage5State.MOVING_TO_TARGET_ABOVE,
            Stage5State.PRE_MOVE_CHECK,
        }:
            return
        self.stage5.complete_hover_dry_run()
        self._set_camera_arm_busy(False)
        self._refresh_stage5_ui()
        self._refresh_ui()
        LOGGER.info("STAGE5 DRY-RUN hover complete (ESTIMATED_MOTION_COMPLETE)")

    def start_stage5_safe_return(self) -> None:
        try:
            sequence, submitted = self.stage5.begin_safe_return()
            self._set_camera_arm_busy(True)
            if not submitted:
                duration = sum(
                    self.actions.get(step.action_name).duration_ms
                    for step in sequence.steps
                    if hasattr(step, "action_name")
                ) + self.config.timing.action_wait_margin_ms * len(sequence.action_names)
                QTimer.singleShot(max(50, duration), self._finish_stage5_return_dry_run)
            self._refresh_stage5_ui()
            self._refresh_ui(current_action=sequence.name)
        except Exception as exc:
            LOGGER.error("STAGE5 safe return rejected: %s", exc)
            QMessageBox.warning(self, "安全返回", str(exc))
            self._refresh_stage5_ui()
            self._refresh_ui()

    def _finish_stage5_return_dry_run(self) -> None:
        if self.stage5.stage_state.state not in {
            Stage5State.RETURNING_TO_CARRY_HIGH,
            Stage5State.RETURNING_TO_OBSERVE,
        }:
            return
        self.stage5.complete_return_dry_run()
        self._set_camera_arm_busy(False)
        self.board_locked = False
        if self.camera_worker is not None:
            self.camera_worker.request_relocalize()
        self._refresh_stage5_ui()
        self._refresh_ui()
        LOGGER.info("STAGE5 DRY-RUN safe return complete (ESTIMATED_MOTION_COMPLETE)")

    def stage5_load_from_action(self) -> None:
        try:
            pwm = self.stage5.store.load_pwm_from_action("P77_ABOVE_IDLE")
            self.control_panel.stage5_panel.set_pwm_values(pwm)
            LOGGER.info("STAGE5 loaded PWM from P77_ABOVE_IDLE: %s", pwm)
        except Exception as exc:
            LOGGER.error("%s", exc)
            QMessageBox.warning(self, "载入动作", str(exc))

    def stage5_set_anchor_from_target(self) -> None:
        target = self.stage5.target
        if target.row is None or target.col is None:
            QMessageBox.warning(self, "设为标定点", "请先选择目标交点")
            return
        if not self.stage5.store.is_anchor_point(target.row, target.col):
            QMessageBox.warning(self, "设为标定点", f"P({target.row},{target.col}) 不在锚点集合中")
            return
        try:
            pwm = self.control_panel.stage5_panel.pwm_values()
        except Exception as exc:
            QMessageBox.warning(self, "设为标定点", str(exc))
            return
        try:
            self.stage5.store.upsert_anchor(target.row, target.col, pwm, calibrated=False)
            LOGGER.info("STAGE5 draft anchor P(%s,%s) %s", target.row, target.col, pwm)
            self.stage5.select_target_programmatically(target.row, target.col, self.config.vision.board_size)
            self._refresh_stage5_ui()
        except Exception as exc:
            QMessageBox.warning(self, "设为标定点", str(exc))

    def stage5_save_anchor(self) -> None:
        target = self.stage5.target
        if target.row is None or target.col is None:
            QMessageBox.warning(self, "保存锚点", "请先选择目标交点")
            return
        try:
            pwm = self.control_panel.stage5_panel.pwm_values()
            self.stage5.store.upsert_anchor(target.row, target.col, pwm, calibrated=False)
            self.stage5.store.save()
            LOGGER.info("STAGE5 saved anchor P(%s,%s)", target.row, target.col)
            self.stage5.select_target_programmatically(target.row, target.col, self.config.vision.board_size)
            self._refresh_stage5_ui()
        except Exception as exc:
            QMessageBox.warning(self, "保存锚点", str(exc))

    def stage5_confirm_anchor(self) -> None:
        target = self.stage5.target
        if target.row is None or target.col is None:
            QMessageBox.warning(self, "确认锚点", "请先选择目标交点")
            return
        reply = QMessageBox.question(
            self,
            "确认锚点安全",
            f"确认 P({target.row},{target.col}) 已完成实机悬停并安全？\n"
            "确认后 calibrated=true 且 verified_runs +1。",
        )
        if reply != QMessageBox.StandardButton.Yes:
            return
        try:
            anchor = self.stage5.store.confirm_anchor_safe(target.row, target.col)
            self.stage5.logger.log("VERIFIED_BY_USER", row=target.row, col=target.col, verified_runs=anchor.verified_runs)
            self.stage5.select_target_programmatically(target.row, target.col, self.config.vision.board_size)
            self._refresh_stage5_ui()
        except Exception as exc:
            QMessageBox.warning(self, "确认锚点", str(exc))

    def stage5_revoke_anchor(self) -> None:
        target = self.stage5.target
        if target.row is None or target.col is None:
            return
        try:
            self.stage5.store.revoke_anchor_safe(target.row, target.col)
            self.stage5.select_target_programmatically(target.row, target.col, self.config.vision.board_size)
            self._refresh_stage5_ui()
        except Exception as exc:
            QMessageBox.warning(self, "取消确认", str(exc))

    def stage5_reload_calibration(self) -> None:
        self.stage5.store.reload()
        if self.stage5.target.row is not None and self.stage5.target.col is not None:
            self.stage5.select_target_programmatically(
                self.stage5.target.row,
                self.stage5.target.col,
                self.config.vision.board_size,
            )
        self._refresh_stage5_ui()
        LOGGER.info("STAGE5 calibration reloaded")

    def stage5_export_calibration(self) -> None:
        try:
            dest = self.config.logs_dir / "stage5" / "exported_calibration.json"
            self.stage5.store.export_to(dest)
            LOGGER.info("STAGE5 calibration exported to %s", dest)
            QMessageBox.information(self, "导出标定", f"已导出到\n{dest}")
        except Exception as exc:
            QMessageBox.warning(self, "导出标定", str(exc))

    def stage5_restore_backup(self) -> None:
        try:
            source = self.stage5.store.restore_latest_backup()
            LOGGER.info("STAGE5 restored backup %s", source)
            self._refresh_stage5_ui()
            QMessageBox.information(self, "恢复备份", f"已从备份恢复\n{source}")
        except Exception as exc:
            QMessageBox.warning(self, "恢复备份", str(exc))

    def stage5_recover(self) -> None:
        try:
            # User must manually re-arm after ESTOP.
            if self.state_machine.snapshot().state == ArmState.ESTOP:
                # Keep arm ESTOP until user uses existing flow (return observe after reconnect).
                pass
            self.stage5.recover()
            self._sync_stage5_context(reason="recover")
            if self.camera_worker is not None:
                self.camera_worker.set_selected_target(None, None)
            self.camera_panel.set_target_text("当前目标坐标：-")
            self._refresh_stage5_ui()
            self._refresh_ui()
        except Exception as exc:
            QMessageBox.warning(self, "恢复", str(exc))



    def _connect_cross_anchor_signals(self) -> None:
        panel = self.control_panel.cross_anchor_panel
        panel.prev_requested.connect(lambda: self._cross_nav(-1))
        panel.next_requested.connect(lambda: self._cross_nav(1))
        panel.select_index_requested.connect(self._cross_select_index)
        panel.nudge_requested.connect(self._cross_nudge)
        panel.set_joint_requested.connect(self._cross_set_joint)
        panel.reset_p77_requested.connect(self._cross_reset_p77)
        panel.undo_requested.connect(self._cross_undo)
        panel.save_draft_requested.connect(self._cross_save_draft)
        panel.load_draft_requested.connect(self._cross_load_draft)
        panel.validate_requested.connect(self._cross_validate)
        panel.plan_carry_requested.connect(lambda: self._cross_plan("carry"))
        panel.plan_target_requested.connect(lambda: self._cross_plan("target"))
        panel.plan_return_requested.connect(lambda: self._cross_plan("return"))
        panel.execute_mock_requested.connect(self._cross_execute_mock)
        panel.result_requested.connect(self._cross_result)
        panel.complete_requested.connect(self._cross_complete)
        panel.cancel_confirm_requested.connect(self._cross_cancel)
        self._refresh_cross_panel()

    def _cross_nav(self, delta: int) -> None:
        if delta < 0:
            self.cross_wizard.prev_anchor()
        else:
            self.cross_wizard.next_anchor()
        self._refresh_cross_panel()

    def _cross_select_index(self, index: int) -> None:
        self.cross_wizard.select_index(int(index))
        self._refresh_cross_panel()

    def _cross_nudge(self, joint_id: str, delta: int) -> None:
        try:
            self.cross_wizard.nudge(joint_id, int(delta))
            self.control_panel.cross_anchor_panel.append_log(f"nudge {joint_id} {delta:+d}")
        except Exception as exc:
            self.control_panel.cross_anchor_panel.append_log(f"ERROR {exc}")
        self._refresh_cross_panel()

    def _cross_set_joint(self, joint_id: str, value: int) -> None:
        try:
            self.cross_wizard.set_joint(joint_id, int(value))
        except Exception as exc:
            self.control_panel.cross_anchor_panel.append_log(f"ERROR {exc}")
        self._refresh_cross_panel()

    def _cross_reset_p77(self) -> None:
        self.cross_wizard.reset_to_p77()
        self.control_panel.cross_anchor_panel.append_log("reset to P77 reference")
        self._refresh_cross_panel()

    def _cross_undo(self) -> None:
        ok = self.cross_wizard.undo()
        self.control_panel.cross_anchor_panel.append_log("undo" if ok else "nothing to undo")
        self._refresh_cross_panel()

    def _cross_save_draft(self) -> None:
        try:
            entry = self.cross_wizard.save_draft()
            self.control_panel.cross_anchor_panel.append_log(f"draft saved status={entry.get('status')}")
        except Exception as exc:
            self.control_panel.cross_anchor_panel.append_log(f"ERROR save draft: {exc}")
        self._refresh_cross_panel()

    def _cross_load_draft(self) -> None:
        try:
            entry = self.cross_wizard.load_draft()
            self.control_panel.cross_anchor_panel.append_log(f"draft loaded runs={entry.get('verified_runs')}")
        except Exception as exc:
            self.control_panel.cross_anchor_panel.append_log(f"ERROR load draft: {exc}")
        self._refresh_cross_panel()

    def _cross_validate(self) -> None:
        issues = self.cross_wizard.validate_candidate()
        for issue in issues:
            self.control_panel.cross_anchor_panel.append_log(f"[{issue.level}] {issue.code}: {issue.message}")
        self._refresh_cross_panel()

    def _cross_plan(self, kind: str) -> None:
        try:
            if kind == "carry":
                plan = self.cross_wizard.plan_carry_high_test()
            elif kind == "target":
                plan = self.cross_wizard.plan_target_above_test()
            else:
                plan = self.cross_wizard.plan_safe_return()
            self._cross_last_plan = plan
            self.control_panel.cross_anchor_panel.append_log(
                f"PLAN {plan.name} duration={plan.estimated_duration_ms}ms pump_off={plan.pump_off}"
            )
            for label, command in plan.serial_commands:
                self.control_panel.cross_anchor_panel.append_log(f"  {label}: {command}")
        except Exception as exc:
            self.control_panel.cross_anchor_panel.append_log(f"ERROR plan: {exc}")
        self._refresh_cross_panel()

    def _cross_execute_mock(self) -> None:
        plan = self._cross_last_plan
        if plan is None:
            self.control_panel.cross_anchor_panel.append_log("ERROR no plan; generate one first")
            return
        try:
            # Always mock under FORCE_STAGE5_DRY_RUN; never touch real serial.
            sent = self.cross_wizard.execute_plan_mock(
                plan,
                gui_dry_run_checked=self.control_panel.stage5_panel.dry_run_checkbox.isChecked(),
            )
            for label, command in sent:
                self.control_panel.cross_anchor_panel.append_log(f"MOCK_TX {label}: {command}")
            self.control_panel.cross_anchor_panel.append_log(
                f"REAL_SERIAL_WRITE_COUNT={self.cross_wizard.real_serial_write_count}"
            )
            if plan.name == "CROSS_SAFE_RETURN":
                self.cross_wizard.mark_safe_return_completed()
        except Exception as exc:
            self.control_panel.cross_anchor_panel.append_log(f"ERROR execute: {exc}")
        self._refresh_cross_panel()

    def _cross_result(self, result: str) -> None:
        try:
            entry = self.cross_wizard.record_user_result(UserTestResult(result))
            self.control_panel.cross_anchor_panel.append_log(
                f"USER_RESULT {result} verified_runs={entry.get('verified_runs')}"
            )
        except Exception as exc:
            self.control_panel.cross_anchor_panel.append_log(f"ERROR result: {exc}")
        self._refresh_cross_panel()

    def _cross_complete(self) -> None:
        try:
            # Use production calibration only if not in pure offline pollution-safe mode.
            # Offline GUI still writes to configured calibration path; tests use temp paths.
            out = self.cross_wizard.complete_anchor(write_calibration=True)
            sample = out.get("sample")
            self.control_panel.cross_anchor_panel.append_log(
                f"ANCHOR_COMPLETED sample={None if sample is None else sample.get('sample_id')}"
            )
        except Exception as exc:
            self.control_panel.cross_anchor_panel.append_log(f"ERROR complete: {exc}")
        self._refresh_cross_panel()

    def _cross_cancel(self) -> None:
        self.cross_wizard.reset_test_session_flags()
        self.control_panel.cross_anchor_panel.append_log("cancelled confirmation flags")
        self._refresh_cross_panel()

    def _refresh_cross_panel(self) -> None:
        snap = self.cross_wizard.status_snapshot()
        self.control_panel.cross_anchor_panel.update_view(snap)


    def _connect_learning_signals(self) -> None:
        panel = self.control_panel.hover_learning_panel
        panel.sync_p77_preview_requested.connect(lambda: self._learning_sync_p77(apply=False))
        panel.sync_p77_apply_requested.connect(lambda: self._learning_sync_p77(apply=True))
        panel.train_smoke_requested.connect(self._learning_train_smoke)
        panel.load_model_requested.connect(self._learning_load_model)
        panel.predict_current_target_requested.connect(self._learning_predict_target)
        panel.compare_requested.connect(self._learning_compare)
        panel.inspect_dataset_requested.connect(self._learning_inspect)
        self._refresh_learning_panel()

    def _learning_dataset(self) -> VerifiedHoverPoseDataset:
        return VerifiedHoverPoseDataset(
            self._samples_path,
            min_verified_runs=1,
            latest_per_coordinate=True,
        )

    def _refresh_learning_panel(self) -> None:
        ds = self._learning_dataset()
        shadow = "-"
        preferred = "-"
        delta = "-"
        gen = ds.manifest().get("generalization_valid")
        model_loaded = self.hover_predictor.model is not None
        self.control_panel.hover_learning_panel.update_status(
            n_samples=len(ds),
            n_coords=len(ds.unique_coordinates()),
            model_loaded=model_loaded,
            generalization_valid=gen,
            shadow_text=shadow,
            preferred=preferred,
            delta_text=delta,
        )

    def _learning_sync_p77(self, *, apply: bool) -> None:
        panel = self.control_panel.hover_learning_panel
        try:
            import json
            from datetime import datetime
            calib = self.stage5.store.to_public_dict()
            anchor = (calib.get("anchors") or {}).get("7,7")
            if not anchor or not anchor.get("calibrated"):
                panel.append_log("P77 not calibrated in formal JSON")
                return
            pwm = {j: int(anchor["pwm"][j]) for j in ["000", "001", "002", "003", "004"]}
            if not apply:
                panel.append_log(f"PREVIEW sync P(7,7) pwm={pwm} (no write)")
                return
            store = HoverSampleStore(self._samples_path)
            rec = store.add_sample(
                row=7,
                col=7,
                pwm=pwm,
                verified_runs=max(int(anchor.get("verified_runs", 1)), 1),
                safe_return_completed=True,
                emergency_stop=False,
                calibration_version=f"gui_sync_{datetime.now():%Y%m%d}",
            )
            if rec is None:
                panel.append_log("P77 already present (duplicate fingerprint)")
            else:
                panel.append_log(f"P77 sample added {rec['sample_id']}")
            self._refresh_learning_panel()
        except Exception as exc:
            panel.append_log(f"ERROR sync: {exc}")

    def _learning_train_smoke(self) -> None:
        panel = self.control_panel.hover_learning_panel
        try:
            result = train_hover_pose(
                self._samples_path,
                self._model_dir,
                TrainConfig(smoke_test=True, epochs=200, latest_per_coordinate=True, min_verified_runs=1),
            )
            panel.append_log(
                f"TRAIN smoke loss={result.final_loss:.6f} samples={result.n_samples} "
                f"coords={result.n_unique_coords} gen_valid={result.generalization_valid} "
                f"params={result.parameter_count}"
            )
            panel.append_log(f"model={result.model_path.name}")
            panel.append_log("MODEL_LIVE_CONTROL_ENABLED=false")
            self._learning_load_model()
        except Exception as exc:
            panel.append_log(f"ERROR train: {exc}")
        self._refresh_learning_panel()

    def _learning_load_model(self) -> None:
        panel = self.control_panel.hover_learning_panel
        try:
            model = self._model_dir / "hover_pose_net_latest.pt"
            norm = self._model_dir / "hover_normalizer_latest.json"
            if not model.exists() or not norm.exists():
                panel.append_log("No latest model; run smoke train first")
                return
            self.hover_predictor.load(model, norm)
            panel.append_log(f"loaded {model.name}")
        except Exception as exc:
            panel.append_log(f"ERROR load model: {exc}")
        self._refresh_learning_panel()

    def _learning_predict_target(self) -> None:
        panel = self.control_panel.hover_learning_panel
        target = self.stage5.target
        row = target.row if target.row is not None else 7
        col = target.col if target.col is not None else 7
        pred = self.hover_predictor.predict(row, col)
        panel.append_log(f"SHADOW P({row},{col}) status={pred.status.value} pwm={pred.pwm} msg={pred.message}")
        panel.append_log("MODEL_LIVE_CONTROL_ENABLED=false (shadow only, no serial)")
        self.control_panel.hover_learning_panel.update_status(
            n_samples=len(self._learning_dataset()),
            n_coords=len(self._learning_dataset().unique_coordinates()),
            model_loaded=self.hover_predictor.model is not None,
            generalization_valid=self._learning_dataset().manifest().get("generalization_valid"),
            shadow_text=str(pred.pwm) if pred.pwm else pred.status.value,
            preferred="pytorch_shadow" if pred.pwm else "-",
            delta_text="-",
        )

    def _learning_compare(self) -> None:
        panel = self.control_panel.hover_learning_panel
        target = self.stage5.target
        row = target.row if target.row is not None else 7
        col = target.col if target.col is not None else 7
        shadow = self.hover_predictor.predict(row, col)
        cmp = self.hover_comparator.compare(row, col, shadow)
        panel.append_log(f"COMPARE P({row},{col}) preferred={cmp.preferred_source}")
        for name, payload in cmp.sources.items():
            panel.append_log(f"  {name}: {payload}")
        panel.append_log(f"deltas={cmp.max_abs_delta_vs_preferred}")
        self.control_panel.hover_learning_panel.update_status(
            n_samples=len(self._learning_dataset()),
            n_coords=len(self._learning_dataset().unique_coordinates()),
            model_loaded=self.hover_predictor.model is not None,
            generalization_valid=self._learning_dataset().manifest().get("generalization_valid"),
            shadow_text=str(shadow.pwm) if shadow.pwm else shadow.status.value,
            preferred=str(cmp.preferred_source),
            delta_text=str(cmp.max_abs_delta_vs_preferred),
        )

    def _learning_inspect(self) -> None:
        panel = self.control_panel.hover_learning_panel
        ds = self._learning_dataset()
        m = ds.manifest()
        panel.append_log(f"dataset samples={m['n_samples']} coords={m['n_unique_coords']} ids={m['sample_ids']}")
        panel.append_log(f"generalization_valid={m['generalization_valid']}")
        self._refresh_learning_panel()

    def _sync_stage5_context(self, *, reason: str = "") -> None:
        """Single source of truth: pull live COM/board/arm/estop into Stage5."""
        arm = self.state_machine.snapshot()
        serial_connected = bool(self.controller is not None and self.controller.is_connected)
        board_locked = bool(self.board_locked)
        estop = arm.state == ArmState.ESTOP or self.stage5.stage_state.snapshot().state == Stage5State.EMERGENCY_STOP
        # If arm is ESTOP, force estop flag; otherwise only keep latched Stage5 ESTOP until recover.
        if arm.state == ArmState.ESTOP:
            estop = True
        elif self.stage5.stage_state.snapshot().state != Stage5State.EMERGENCY_STOP:
            estop = False

        before = self.stage5.stage_state.state
        self.stage5.update_context(
            serial_connected=serial_connected,
            board_locked=board_locked,
            arm_state=arm.state,
            emergency_stopped=estop,
        )
        after = self.stage5.stage_state.state
        # Avoid flooding logs on high-frequency refresh_ui; always log transitions.
        if reason != "refresh_ui" or before != after:
            LOGGER.info(
                "[STAGE5][SYNC] reason=%s serial=%s board=%s arm=%s estop=%s stage5=%s->%s same_controller=%s",
                reason or "-",
                int(serial_connected),
                int(board_locked),
                arm.state.value,
                int(estop),
                before.value,
                after.value,
                int(self.stage5.controller is self.controller),
            )
        if after == Stage5State.DISCONNECTED and serial_connected:
            LOGGER.warning(
                "[STAGE5][STATE_BLOCKED] reason=UNEXPECTED_DISCONNECTED serial=1 board=%s arm=%s",
                int(board_locked),
                arm.state.value,
            )
        if after == Stage5State.DISCONNECTED and not serial_connected:
            LOGGER.info("[STAGE5][STATE_BLOCKED] reason=SERIAL_NOT_CONNECTED")

    def _refresh_stage5_ui(self) -> None:
        snap = self.stage5.stage_state.snapshot()
        target = self.stage5.target
        panel = self.control_panel.stage5_panel
        arm = self.state_machine.snapshot()
        if target.pwm:
            panel.set_pwm_values(target.pwm)
        panel.update_target_view(
            state=snap.state.value,
            row=target.row,
            col=target.col,
            pixel_x=target.pixel_x,
            pixel_y=target.pixel_y,
            calibrated=target.calibrated_text,
            region=target.region_text,
            source=target.source,
            pwm_text=target.pwm_text,
            verified_runs=target.verified_runs,
        )
        panel.update_sync_diagnostics(
            serial_sync=bool(self.controller.is_connected),
            board_sync=bool(self.board_locked),
            arm_sync=arm.state.value,
            estop=arm.state == ArmState.ESTOP or snap.state == Stage5State.EMERGENCY_STOP,
            controller_shared=self.stage5.controller is self.controller,
            blocked_reason=self.stage5.blocked_reason(),
        )
        panel.set_enabled_state(
            serial_connected=self.controller.is_connected,
            board_locked=self.board_locked,
            busy=arm.busy or self.arm_worker.busy or self.stage5.stage_state.is_moving(),
            can_hover=self.stage5.stage_state.can_execute_hover()
            and arm.state in {ArmState.OBSERVE_IDLE, ArmState.OBSERVE_HOLD}
            and not arm.busy
            and not self.arm_worker.busy,
            can_return=self.stage5.stage_state.can_safe_return() and not arm.busy and not self.arm_worker.busy,
            has_target=target.row is not None,
            estop=snap.state == Stage5State.EMERGENCY_STOP or arm.state == ArmState.ESTOP,
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
