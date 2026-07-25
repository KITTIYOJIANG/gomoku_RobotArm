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
from app.stage5.safety import derive_calibration_limits, derive_pwm_safety_limits
from app.stage5.constants import FORCE_STAGE5_DRY_RUN, OUTER_RING, STAR_CORNERS
from app.stage5.pwm_interpolator import estimate_outer_ring_pwm, estimate_star_corner_pwm
from app.stage5.tour_planner import (
    build_board_reachable_tour,
    build_cross_reverify_tour,
    sync_completed_drafts_into_calibration,
)
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
            force_dry_run=bool(getattr(config.stage5, "force_dry_run", False) or FORCE_STAGE5_DRY_RUN),
        )
        self._cross_last_plan = None
        self._active_tour = None
        self._star_index = 0
        self._outer_index = 0
        self._active_tour_dry = False
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
        panel.beep_test_requested.connect(self.test_beep)
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
        s5.board_tour_requested.connect(self.start_board_hover_tour)
        s5.reload_inference_requested.connect(self.stage5_reload_inference)
        s5.save_finetune_requested.connect(self.stage5_save_finetune)
        s5.nudge_joint_requested.connect(self.stage5_nudge_joint)
        s5.star_select_requested.connect(self.stage5_select_star)
        s5.star_next_requested.connect(self.stage5_next_star)
        s5.star_seed_requested.connect(self.stage5_seed_star)
        s5.outer_select_requested.connect(self.stage5_select_outer)
        s5.outer_next_requested.connect(self.stage5_next_outer)
        self._connect_cross_anchor_signals()
        self._connect_learning_signals()

        self.arm_worker.sequence_started.connect(self._on_sequence_started)
        self.arm_worker.step_started.connect(self._on_step_started)
        self.arm_worker.sequence_finished.connect(self._on_sequence_finished)
        self.arm_worker.log_message.connect(lambda message: LOGGER.info("%s", message))
        try:
            self._refresh_star_status()
        except Exception:
            pass

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
        if not self.controller.is_connected:
            QMessageBox.warning(self, "回观察位", "请先连接串口")
            return
        if self.arm_worker.busy or self.state_machine.busy:
            QMessageBox.warning(self, "回观察位", "机械臂忙，请稍候或急停后恢复再试")
            return
        if self.state_machine.snapshot().state == ArmState.ESTOP:
            QMessageBox.warning(self, "回观察位", "当前急停锁存：请先点「急停后恢复」，再回观察位")
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
        if name in {"CROSS_REVERIFY_TOUR", "BOARD_HOVER_TOUR"}:
            self._finish_hover_tour(name, success=success, message=message)
            return
        if name in {"CROSS_CARRY_HIGH", "CROSS_TARGET_ABOVE", "CROSS_SAFE_RETURN"}:
            # CROSS_TARGET_ABOVE ends with the arm parked over the board. Keep
            # vision frozen until its safe-return sequence has actually ended.
            self._set_camera_arm_busy(success and name == "CROSS_TARGET_ABOVE")
            try:
                if (self.state_machine.snapshot().current_action or "").startswith("MANUAL:"):
                    self.state_machine.complete_manual()
            except Exception as exc:
                LOGGER.warning("cross complete_manual: %s", exc)
            self.cross_wizard.mark_live_plan_finished(name, success=success)
            if success and name == "CROSS_SAFE_RETURN":
                self.cross_wizard.mark_safe_return_completed()
                try:
                    self._log_transition(self.state_machine.mark_observe_idle())
                except Exception as exc:
                    LOGGER.warning("mark_observe_idle failed: %s", exc)
            panel = self.control_panel.cross_anchor_panel
            panel.append_log(
                f"LIVE 完成 name={name} success={success} msg={message or '-'}"
            )
            if success:
                # Count real sends as number of action steps roughly
                plan = self._cross_last_plan
                if plan is not None:
                    self.cross_wizard.real_serial_write_count += len(plan.action_names)
                panel.append_log(
                    f"REAL_SERIAL_WRITE_COUNT={self.cross_wizard.real_serial_write_count}"
                )
            self._cross_live_plan_name = None
            # After live cross move, arm state is UNKNOWN from complete_manual; ask user return observe
            if success and name != "CROSS_SAFE_RETURN":
                panel.append_log("提示：请点「生成安全返回计划」并执行，或日常区回观察位")
            self._refresh_cross_panel()
            self._refresh_ui()
            return
        if self.stage5.on_sequence_finished(name, success, message):
            # Worker "finished" only means motion stopped. HOVER_TO_TARGET
            # deliberately leaves the arm parked over the board, where it can
            # continue hiding the top-left AprilTag. Release the frozen board
            # pose only after SAFE_RETURN_FROM_HOVER (or a failed hover).
            keep_frozen = bool(success and name == "HOVER_TO_TARGET")
            self._set_camera_arm_busy(keep_frozen)
            if success and name == "SAFE_RETURN_FROM_HOVER":
                # Keep board lock if camera still has a valid pose so user can
                # immediately click the next intersection. Soft relocalize only.
                if self.camera_worker is not None:
                    self.camera_worker.request_relocalize()
                # Clear previous target so next click selects a fresh point.
                try:
                    self.stage5.clear_target()
                except Exception:
                    pass
                if self.camera_worker is not None:
                    self.camera_worker.set_selected_target(None, None)
                self.camera_panel.set_target_text("目标: 请再点画面交点")
                LOGGER.info("STAGE5 ready for next click; board_locked=%s", self.board_locked)
            self._sync_stage5_context(reason=f"sequence_finished:{name}")
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
        """Freeze vision while the arm moves or remains parked over the board."""
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
            self._refresh_stage5_ui()
            # Auto-fill inferred PWM into editors for fine-tuning.
            if self.stage5.target.pwm:
                self.control_panel.stage5_panel.set_pwm_values(self.stage5.target.pwm)
                LOGGER.info(
                    "[STAGE5][CLICK_INFER] P(%s,%s) source=%s pwm=%s",
                    selection.row,
                    selection.col,
                    self.stage5.target.source,
                    self.stage5.target.pwm,
                )
            else:
                LOGGER.info(
                    "[STAGE5][CLICK_INFER] P(%s,%s) no pwm source=%s",
                    selection.row,
                    selection.col,
                    self.stage5.target.source,
                )
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
            pwm_override = None
            try:
                pwm_override = self.control_panel.stage5_panel.pwm_values()
            except Exception:
                pwm_override = None
            # Remember editor PWM as the live target so later UI refresh won't show stale seed.
            if pwm_override and self.stage5.target.row is not None:
                self.stage5.target.pwm = dict(pwm_override)
                self.stage5.target.source = "user_edited"
                self.stage5.target.pwm_text = ", ".join(
                    f"{k}:{pwm_override[k]}" for k in ("000", "001", "002", "003", "004")
                    if k in pwm_override
                )
                self.stage5.target.calibrated_text = "NO"
                self.stage5.target.calibrated = False
            plan = self.stage5.plan_hover(holding_piece=holding, pwm_override=pwm_override)
            LOGGER.info(
                "STAGE5 HOVER plan target=P(%s,%s) source=%s dry_run=%s duration_ms=%s commands=%s",
                plan.target_row,
                plan.target_col,
                plan.source,
                plan.dry_run,
                plan.estimated_duration_ms,
                plan.serial_commands,
            )
            # Freeze after validation but before the worker can move the arm
            # into the camera FOV. A rejected plan must not latch vision.
            self._set_camera_arm_busy(True)
            submitted, mode = self.stage5.begin_hover_execution(plan)
            self._set_camera_arm_busy(True)
            if not submitted:
                LOGGER.warning(
                    "STAGE5 HOVER DRY-RUN only (mode=%s). Uncheck DRY RUN for live send like manual P77_ABOVE.",
                    mode,
                )
                QMessageBox.information(
                    self,
                    "阶段五悬停 = 仅演练（臂不会动）",
                    "当前不会驱动机械臂。\n\n"
                    "原因：Stage5 勾选了 DRY RUN（或 FORCE_STAGE5_DRY_RUN）。\n"
                    "手动调试里的 P77_ABOVE 会真实发送，所以能动。\n\n"
                    "真机悬停：\n"
                    "1. 取消勾选 Stage5 的 DRY RUN\n"
                    "2. 确认 Arm=OBSERVE_IDLE、BOARD LOCKED、目标 P(7,7)\n"
                    "3. 再点「悬停到目标点」\n\n"
                    f"计划: {plan.sequence.action_names}\n"
                    f"预计: {plan.estimated_duration_ms} ms",
                )
                QTimer.singleShot(max(50, plan.estimated_duration_ms), self._finish_stage5_hover_dry_run)
            else:
                LOGGER.info("STAGE5 HOVER LIVE submitted to arm worker (same path as serial TX)")
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
        """Stage5 button 2: safe return from hover, else same as main 回观察位."""
        if not self.stage5.stage_state.can_safe_return():
            # Allow early/anytime return when not currently hovering a target.
            self.start_return_to_observe()
            return
        try:
            self._set_camera_arm_busy(True)
            sequence, submitted = self.stage5.begin_safe_return()
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
        try:
            self.stage5.clear_target()
        except Exception:
            pass
        if self.camera_worker is not None:
            self.camera_worker.request_relocalize()
            self.camera_worker.set_selected_target(None, None)
        self.camera_panel.set_target_text("目标: 请再点画面交点")
        self._sync_stage5_context(reason="dry_return_done")
        self._refresh_stage5_ui()
        self._refresh_ui()
        LOGGER.info("STAGE5 DRY-RUN safe return complete; ready for next click")

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
        try:
            pwm = self.control_panel.stage5_panel.pwm_values()
        except Exception as exc:
            QMessageBox.warning(self, "设为标定点", str(exc))
            return
        try:
            self.stage5.store.upsert_anchor(
                target.row,
                target.col,
                pwm,
                calibrated=False,
                require_anchor_set=False,
                expand_grid=True,
                skip_envelope_check=True,
                notes="user fine-tune draft",
            )
            LOGGER.info("STAGE5 draft anchor P(%s,%s) %s", target.row, target.col, pwm)
            self.stage5.select_target_programmatically(target.row, target.col, self.config.vision.board_size)
            self._refresh_stage5_ui()
        except Exception as exc:
            QMessageBox.warning(self, "设为标定点", str(exc))

    def stage5_save_anchor(self) -> None:
        """Legacy advanced save — same as fine-tune without confirm dialog."""
        self.stage5_save_finetune(confirm=False)

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





    def stage5_reload_inference(self) -> None:
        """Recompute interpolated PWM for current click and fill editors."""
        target = self.stage5.target
        if target.row is None or target.col is None:
            QMessageBox.warning(self, "载入推理", "请先点击画面交点")
            return
        self.stage5.select_target_programmatically(
            target.row, target.col, self.config.vision.board_size
        )
        self._refresh_stage5_ui()
        pwm = self.stage5.target.pwm
        if not pwm:
            QMessageBox.warning(
                self,
                "载入推理",
                f"P({target.row},{target.col}) 当前无法推理 PWM（{self.stage5.target.source}）",
            )
            return
        self.control_panel.stage5_panel.set_pwm_values(pwm)
        LOGGER.info(
            "[STAGE5][INFER] P(%s,%s) source=%s pwm=%s",
            target.row,
            target.col,
            self.stage5.target.source,
            pwm,
        )

    def stage5_nudge_joint(self, joint_id: str, delta: int) -> None:
        panel = self.control_panel.stage5_panel
        try:
            values = panel.pwm_values()
        except Exception:
            # empty fields: try inference first
            self.stage5_reload_inference()
            try:
                values = panel.pwm_values()
            except Exception as exc:
                QMessageBox.warning(self, "微调", str(exc))
                return
        jid = str(joint_id).zfill(3)
        if jid not in values:
            return
        values[jid] = max(500, min(2500, int(values[jid]) + int(delta)))
        panel.set_pwm_values(values)
        LOGGER.info("[STAGE5][NUDGE] %s %+d -> %s", jid, delta, values[jid])
        # Coach: edited PWM is only used when user clicks hover again.
        panel.next_label.setText(
            f"已改 {jid}={values[jid]}。请：回观察位 → 点「1.用当前PWM去悬停」验证 → 满意再点「3.保存」"
        )
        if hasattr(panel, "_style_next"):
            panel._style_next("orange")

    def stage5_save_finetune(self, confirm: bool = True) -> None:
        """Save current editor PWM as calibrated taught point; expands grid for midpoints."""
        target = self.stage5.target
        if target.row is None or target.col is None:
            QMessageBox.warning(self, "保存微调点", "请先点击画面交点")
            return
        try:
            pwm = self.control_panel.stage5_panel.pwm_values()
        except Exception as exc:
            QMessageBox.warning(self, "保存微调点", str(exc))
            return
        if confirm:
            reply = QMessageBox.question(
                self,
                "保存微调点",
                (
                    f"将 P({target.row},{target.col}) 的当前 PWM 写入正式标定？\n"
                    f"PWM={pwm}\n\n"
                    "会扩展插值网格（该行/列成为锚线），之后巡检/悬停将优先用此手教值。"
                ),
            )
            if reply != QMessageBox.StandardButton.Yes:
                return
        try:
            from app.stage5.safety import derive_calibration_limits

            self.stage5.store.upsert_anchor(
                target.row,
                target.col,
                pwm,
                time_ms=1000,
                notes="user fine-tune taught point",
                calibrated=True,
                verified_runs=max(1, int(getattr(self.stage5.target, "verified_runs", 0) or 0)),
                require_anchor_set=False,
                expand_grid=True,
                safety_limits=derive_calibration_limits(self.actions),
                skip_envelope_check=False,
            )
            path = self.stage5.store.save()
            LOGGER.info(
                "[STAGE5][FINETUNE_SAVED] P(%s,%s) pwm=%s path=%s rows=%s cols=%s",
                target.row,
                target.col,
                pwm,
                path,
                self.stage5.store.anchor_rows,
                self.stage5.store.anchor_cols,
            )
            self.stage5.select_target_programmatically(
                target.row, target.col, self.config.vision.board_size
            )
            self._refresh_stage5_ui(push_pwm=True)
            self._refresh_star_status()
            QMessageBox.information(
                self,
                "已保存",
                f"P({target.row},{target.col}) 已写入标定。\n"
                f"网格 rows={self.stage5.store.anchor_rows}\n"
                f"cols={self.stage5.store.anchor_cols}",
            )
        except Exception as exc:
            LOGGER.exception("finetune save failed")
            QMessageBox.warning(self, "保存微调点失败", str(exc))

    def test_beep(self) -> None:
        """Manual buzzer test for factory firmware + host fallback."""
        if not self.controller.is_connected:
            QMessageBox.warning(self, "蜂鸣测试", "请先连接串口")
            return
        ok = self._beep_once()
        QMessageBox.information(
            self,
            "蜂鸣测试",
            (
                "已发送：\n"
                "1) 本机扬声器/蜂鸣（应能听到）\n"
                "2) 串口候选 $BEEP / beep,n\n\n"
                f"结果 host/arm 见日志 [TOUR][BEEP]。\n"
                f"综合={'有提示' if ok else '失败'}。\n\n"
                "若仅本机响、机械臂不响：当前出厂固件未实现蜂鸣指令，"
                "完成提示以本机声为准（无需改舵机动作）。"
            ),
        )



    def _refresh_star_status(self) -> None:
        lines = []
        for r, c, _lab, cn in STAR_CORNERS:
            a = self.stage5.store.get_anchor(r, c)
            if a is not None and a.calibrated:
                mark = "已校准"
            else:
                mark = "默认/未教"
            lines.append(f"P({r},{c}){cn}={mark}")
        self.control_panel.stage5_panel.update_star_status(lines, self._star_index)

    def stage5_select_star(self, index: int) -> None:
        if not 0 <= int(index) < len(STAR_CORNERS):
            return
        self._star_index = int(index)
        row, col, lab, cn = STAR_CORNERS[self._star_index]
        self.stage5.select_target_programmatically(row, col, self.config.vision.board_size)
        if self.camera_worker is not None:
            self.camera_worker.set_selected_target(row, col)
        self.camera_panel.set_target_text(f"目标: P({row},{col}) {cn}")
        self._refresh_stage5_ui()

        anchor = self.stage5.store.get_anchor(row, col)
        taught = anchor is not None and anchor.calibrated
        if taught and anchor is not None:
            self.control_panel.stage5_panel.set_pwm_values(anchor.pwm)
            LOGGER.info("[STAGE5][STAR] P(%s,%s) taught pwm=%s", row, col, anchor.pwm)
        else:
            # Prefer current resolve, else parallelogram seed — no popup.
            pwm = self.stage5.target.pwm
            if not pwm:
                try:
                    seed = estimate_star_corner_pwm(self.stage5.store, row, col)
                    pwm = seed.pwm_str_keys()
                    LOGGER.info("[STAGE5][STAR] P(%s,%s) seed=%s", row, col, pwm)
                except Exception as exc:
                    LOGGER.warning("[STAGE5][STAR] no pwm: %s", exc)
                    pwm = None
            if pwm:
                self.control_panel.stage5_panel.set_pwm_values(pwm)
        self._refresh_star_status()
        self._refresh_ui()

    def stage5_next_star(self) -> None:
        # Prefer first uncalibrated star; else cycle.
        for offset in range(len(STAR_CORNERS)):
            idx = (self._star_index + offset) % len(STAR_CORNERS)
            r, c, _l, _cn = STAR_CORNERS[idx]
            a = self.stage5.store.get_anchor(r, c)
            if a is None or not a.calibrated:
                self.stage5_select_star(idx)
                return
        self._star_index = (self._star_index + 1) % len(STAR_CORNERS)
        self.stage5_select_star(self._star_index)

    def stage5_seed_star(self) -> None:
        """Load parallelogram seed for current star corner into editors (no popup)."""
        row, col, lab, cn = STAR_CORNERS[self._star_index]
        try:
            seed = estimate_star_corner_pwm(self.stage5.store, row, col)
        except Exception as exc:
            LOGGER.warning("[STAGE5][STAR_SEED] %s", exc)
            return
        self.stage5.select_target_programmatically(row, col, self.config.vision.board_size)
        if self.camera_worker is not None:
            self.camera_worker.set_selected_target(row, col)
        self.camera_panel.set_target_text(f"目标: P({row},{col}) {cn}")
        self.control_panel.stage5_panel.set_pwm_values(seed.pwm_str_keys())
        self._refresh_stage5_ui()
        self._refresh_star_status()
        LOGGER.info(
            "[STAGE5][STAR_SEED] P(%s,%s) from %s pwm=%s",
            row, col, seed.anchors_used, seed.pwm_str_keys(),
        )



    def _refresh_outer_status(self) -> None:
        lines = []
        for r, c, _lab, cn in OUTER_RING:
            a = self.stage5.store.get_anchor(r, c)
            mark = "已校准" if (a is not None and a.calibrated) else "未教"
            lines.append(f"P({r},{c}){cn}={mark}")
        self.control_panel.stage5_panel.update_outer_status(lines, self._outer_index)

    def stage5_select_outer(self, index: int) -> None:
        if not 0 <= int(index) < len(OUTER_RING):
            return
        self._outer_index = int(index)
        row, col, lab, cn = OUTER_RING[self._outer_index]
        self.stage5.select_target_programmatically(row, col, self.config.vision.board_size)
        if self.camera_worker is not None:
            self.camera_worker.set_selected_target(row, col)
        self.camera_panel.set_target_text(f"目标: P({row},{col}) {cn}")
        self._refresh_stage5_ui()
        anchor = self.stage5.store.get_anchor(row, col)
        if anchor is not None and anchor.calibrated:
            self.control_panel.stage5_panel.set_pwm_values(anchor.pwm)
            LOGGER.info("[STAGE5][OUTER] taught P(%s,%s) %s", row, col, anchor.pwm)
        else:
            pwm = self.stage5.target.pwm
            if not pwm:
                try:
                    seed = estimate_outer_ring_pwm(self.stage5.store, row, col)
                    pwm = seed.pwm_str_keys()
                    LOGGER.info("[STAGE5][OUTER] seed P(%s,%s) %s %s", row, col, seed.details, pwm)
                except Exception as exc:
                    LOGGER.warning("[STAGE5][OUTER] no pwm: %s", exc)
                    pwm = None
            if pwm:
                self.control_panel.stage5_panel.set_pwm_values(pwm)
        self._refresh_outer_status()
        self._refresh_ui()

    def stage5_next_outer(self) -> None:
        for offset in range(len(OUTER_RING)):
            idx = (self._outer_index + offset) % len(OUTER_RING)
            r, c, _l, _cn = OUTER_RING[idx]
            a = self.stage5.store.get_anchor(r, c)
            if a is None or not a.calibrated:
                self.stage5_select_outer(idx)
                return
        self._outer_index = (self._outer_index + 1) % len(OUTER_RING)
        self.stage5_select_outer(self._outer_index)


    def start_cross_reverify_tour(self) -> None:
        """One-click re-verify all completed cross anchors; beep once on success."""
        panel = self.control_panel.cross_anchor_panel
        try:
            self.stage5.store.reload()
            self.cross_wizard.drafts.reload()
            promoted = sync_completed_drafts_into_calibration(
                self.stage5.store,
                self.cross_wizard.drafts._data if hasattr(self.cross_wizard.drafts, "_data") else {},
                safety_limits=derive_calibration_limits(self.actions),
            )
            if promoted:
                panel.append_log(f"已补写正式标定: {', '.join(promoted)}")
                LOGGER.info("[TOUR] promoted drafts into calibration: %s", promoted)
            drafts_payload = getattr(self.cross_wizard.drafts, "_data", None)
            plan = build_cross_reverify_tour(
                self.actions,
                self.stage5.store,
                drafts=drafts_payload,
                dwell_ms=450,
                action_wait_margin_ms=self.config.timing.action_wait_margin_ms,
            )
        except Exception as exc:
            panel.append_log(f"ERROR 复验计划失败: {exc}")
            QMessageBox.warning(self, "复验计划失败", str(exc))
            return
        self._start_hover_tour(plan, log_panel=panel)

    def start_board_hover_tour(self) -> None:
        """Board tour with mode choice: anchors-only (accurate) or full+thermal cool."""
        chooser = QMessageBox(self)
        chooser.setWindowTitle("棋盘巡检模式")
        chooser.setIcon(QMessageBox.Icon.Question)
        chooser.setText(
            "一键复验(锚点)准确、全点巡检容易越跑越偏，常见原因：\n"
            "1) 中间点是插值估算，不是手教 PWM\n"
            "2) 连续动作舵机发热，开环漂移\n\n"
            "请选择本轮模式："
        )
        btn_anchors = chooser.addButton("仅锚点（推荐，准）", QMessageBox.ButtonRole.AcceptRole)
        btn_full = chooser.addButton("全可达点+分段冷却", QMessageBox.ButtonRole.ActionRole)
        btn_cancel = chooser.addButton("取消", QMessageBox.ButtonRole.RejectRole)
        chooser.setDefaultButton(btn_anchors)
        chooser.exec()
        clicked = chooser.clickedButton()
        if clicked is None or clicked is btn_cancel:
            return
        direct_only = clicked is btn_anchors
        try:
            self.stage5.store.reload()
            self.cross_wizard.drafts.reload()
            promoted = sync_completed_drafts_into_calibration(
                self.stage5.store,
                getattr(self.cross_wizard.drafts, "_data", {}) or {},
                safety_limits=derive_calibration_limits(self.actions),
            )
            if promoted:
                LOGGER.info("[TOUR] promoted drafts into calibration: %s", promoted)
            # Wide teaching limits so taught extremes (e.g. P11,7) are not rejected.
            plan = build_board_reachable_tour(
                self.actions,
                self.stage5.store,
                limits=derive_calibration_limits(self.actions),
                dwell_ms=400 if direct_only else 350,
                action_wait_margin_ms=self.config.timing.action_wait_margin_ms,
                direct_only=direct_only,
                # Anchors are few: no cool needed. Full tour: cool every 4 points ~4s.
                cool_every_n=0 if direct_only else 3,
                cool_ms=0 if direct_only else 6000,
                path_mode="carry_each" if direct_only else "segment",
            )
        except Exception as exc:
            QMessageBox.warning(self, "巡检计划失败", str(exc))
            LOGGER.error("BOARD TOUR plan failed: %s", exc)
            return
        self._start_hover_tour(plan, log_panel=self.control_panel.cross_anchor_panel)

    def _start_hover_tour(self, plan, *, log_panel) -> None:
        stops_txt = ", ".join(f"P({s.row},{s.col})" for s in plan.stops)
        est_s = plan.estimated_duration_ms / 1000.0
        dry_checked = self.control_panel.stage5_panel.dry_run_checkbox.isChecked()
        mode_txt = "DRY RUN（只日志）" if dry_checked else "真机 LIVE"
        answer = QMessageBox.question(
            self,
            plan.display_name,
            (
                f"模式: {mode_txt}\n"
                f"点数: {len(plan.stops)}\n"
                f"路径: {stops_txt}\n"
                f"预计约 {est_s:.1f}s\n"
                f"备注: {', '.join(plan.notes[:6])}\n"
                f"成功结束后蜂鸣 1 次\n\n"
                "确认开始？"
            ),
            QMessageBox.StandardButton.Ok | QMessageBox.StandardButton.Cancel,
            QMessageBox.StandardButton.Cancel,
        )
        if answer != QMessageBox.StandardButton.Ok:
            log_panel.append_log("巡检已取消")
            return
        if not self.controller.is_connected:
            QMessageBox.warning(self, "未连接", "请先连接串口（或 dry-run 模拟连接）。")
            return
        if self.arm_worker.busy or self.state_machine.busy:
            QMessageBox.warning(self, "动作忙", "已有机械臂动作正在执行。")
            return

        log_panel.append_log(
            f"TOUR start name={plan.name} stops={len(plan.stops)} dry={int(dry_checked)} est_ms={plan.estimated_duration_ms}"
        )
        for stop in plan.stops:
            log_panel.append_log(
                f"  stop P({stop.row},{stop.col}) src={stop.source} pwm={stop.pwm}"
            )
        LOGGER.info(
            "[TOUR][START] name=%s stops=%s dry=%s duration_ms=%s",
            plan.name,
            len(plan.stops),
            int(dry_checked),
            plan.estimated_duration_ms,
        )

        self._active_tour = plan
        self._active_tour_dry = bool(dry_checked)
        try:
            self.state_machine.begin_manual(plan.name)
        except InvalidTransition as exc:
            self._active_tour = None
            QMessageBox.warning(self, "状态不允许", str(exc))
            return

        if dry_checked or self.controller.dry_run:
            # Log each command without submitting to worker queue semantics for multi-min waits.
            for step in plan.sequence.steps:
                if hasattr(step, "action_name"):
                    action = self.actions.get(step.action_name)
                    LOGGER.info("[TOUR][DRY] TX %s %s", action.name, action.command)
                    log_panel.append_log(f"DRY TX {action.name}")
                else:
                    LOGGER.info("[TOUR][DRY] WAIT %s %sms", step.label, step.duration_ms)
                    log_panel.append_log(f"DRY WAIT {step.label} {step.duration_ms}ms")
            QTimer.singleShot(
                max(200, min(plan.estimated_duration_ms, 3000)),
                lambda: self._finish_hover_tour(plan.name, success=True, message="dry_run_complete"),
            )
            self._set_camera_arm_busy(True)
            self._refresh_ui()
            return

        if not self.arm_worker.submit(plan.sequence):
            self._active_tour = None
            try:
                self.state_machine.fail("tour submit rejected")
            except Exception:
                pass
            QMessageBox.warning(self, "提交失败", "ArmSequenceWorker 拒绝了巡检序列")
            return
        self._set_camera_arm_busy(True)
        self._refresh_ui()

    def _finish_hover_tour(self, name: str, *, success: bool, message: str) -> None:
        plan = self._active_tour
        self._active_tour = None
        self._set_camera_arm_busy(False)
        panel = self.control_panel.cross_anchor_panel
        if success:
            try:
                self.state_machine.mark_observe_idle()
            except Exception as exc:
                LOGGER.warning("tour mark_observe_idle failed: %s", exc)
                try:
                    self.state_machine.complete_manual()
                except Exception:
                    pass
            beep_ok = self._beep_once()
            stop_n = 0 if plan is None else len(plan.stops)
            panel.append_log(
                f"TOUR DONE name={name} success=1 stops={stop_n} beep={'ok' if beep_ok else 'fail/skip'} msg={message or '-'}"
            )
            LOGGER.info("[TOUR][DONE] name=%s success=1 beep=%s", name, int(beep_ok))
            QMessageBox.information(
                self,
                "巡检完成",
                f"{name} 完成，共 {stop_n} 点。\n蜂鸣: {'已发送' if beep_ok else '发送失败/未支持（见日志）'}",
            )
        else:
            try:
                self.state_machine.fail(message or f"{name} failed")
            except Exception:
                pass
            panel.append_log(f"TOUR FAIL name={name} msg={message or '-'}")
            LOGGER.error("[TOUR][FAIL] name=%s msg=%s", name, message)
            QMessageBox.warning(self, "巡检失败", message or name)
        self._sync_stage5_context(reason=f"tour_finished:{name}:{success}")
        self._refresh_cross_panel()
        self._refresh_stage5_ui()
        self._refresh_ui()


    def _beep_once(self) -> bool:
        """Completion cue. Host beep is authoritative; arm commands best-effort."""
        host_ok = False
        try:
            import winsound
            # Longer, louder pattern so it is hard to miss on Windows.
            winsound.MessageBeep(winsound.MB_ICONASTERISK)
            winsound.Beep(1500, 220)
            winsound.Beep(1200, 180)
            winsound.Beep(1500, 220)
            host_ok = True
        except Exception:
            try:
                from PySide6.QtWidgets import QApplication
                for _ in range(3):
                    QApplication.beep()
                host_ok = True
            except Exception:
                pass

        arm_ok = False
        if self.controller.is_connected:
            try:
                sent = self.controller.beep(times=2, duration_ms=150)
                arm_ok = bool(sent)
                LOGGER.info("[BEEP] arm candidates sent=%s", [repr(s) for s in sent])
            except Exception as exc:
                LOGGER.warning("[BEEP] arm send failed: %s", exc)
        else:
            LOGGER.warning("[BEEP] serial not connected; host-only")

        LOGGER.info("[BEEP] host=%s arm_tx=%s (arm hardware may ignore unknown cmds)", int(host_ok), int(arm_ok))
        return host_ok or arm_ok


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
        panel.reverify_tour_requested.connect(self.start_cross_reverify_tour)
        self._refresh_cross_panel()


    def _cross_pull_edits(self) -> bool:
        """Sync QLineEdit candidate PWM into wizard before any action."""
        panel = self.control_panel.cross_anchor_panel
        try:
            values = panel.read_candidate_pwm()
            self.cross_wizard.apply_candidate_pwm(values, note="from_gui_edits")
            return True
        except Exception as exc:
            panel.append_log(f"ERROR PWM输入无效: {exc}")
            return False

    def _cross_nav(self, delta: int) -> None:
        self._cross_pull_edits()
        if delta < 0:
            self.cross_wizard.prev_anchor()
        else:
            self.cross_wizard.next_anchor()
        self._refresh_cross_panel(push_edits=True)

    def _cross_select_index(self, index: int) -> None:
        self._cross_pull_edits()
        self.cross_wizard.select_index(int(index))
        self._refresh_cross_panel(push_edits=True)

    def _cross_nudge(self, joint_id: str, delta: int) -> None:
        if not self._cross_pull_edits():
            return
        try:
            self.cross_wizard.nudge(joint_id, int(delta))
            self.control_panel.cross_anchor_panel.append_log(f"nudge {joint_id} {delta:+d} -> {self.cross_wizard.candidate_pwm[joint_id]}")
        except Exception as exc:
            self.control_panel.cross_anchor_panel.append_log(f"ERROR {exc}")
        self._refresh_cross_panel(push_edits=True)

    def _cross_set_joint(self, joint_id: str, value: int) -> None:
        try:
            self.cross_wizard.set_joint(joint_id, int(value))
            self.control_panel.cross_anchor_panel.append_log(f"set {joint_id}={value}")
        except Exception as exc:
            self.control_panel.cross_anchor_panel.append_log(f"ERROR {exc}")
        self._refresh_cross_panel(push_edits=True)

    def _cross_reset_p77(self) -> None:
        self.cross_wizard.reset_to_p77()
        self.control_panel.cross_anchor_panel.append_log("reset to P77 reference")
        self._refresh_cross_panel(push_edits=True)

    def _cross_undo(self) -> None:
        ok = self.cross_wizard.undo()
        self.control_panel.cross_anchor_panel.append_log("undo" if ok else "nothing to undo")
        self._refresh_cross_panel(push_edits=True)

    def _cross_save_draft(self) -> None:
        if not self._cross_pull_edits():
            return
        try:
            entry = self.cross_wizard.save_draft()
            self.control_panel.cross_anchor_panel.append_log(
                f"draft saved status={entry.get('status')} pwm={self.cross_wizard.candidate_pwm}"
            )
        except Exception as exc:
            self.control_panel.cross_anchor_panel.append_log(f"ERROR save draft: {exc}")
        self._refresh_cross_panel(push_edits=True)

    def _cross_load_draft(self) -> None:
        try:
            entry = self.cross_wizard.load_draft()
            self.control_panel.cross_anchor_panel.append_log(f"draft loaded runs={entry.get('verified_runs')}")
        except Exception as exc:
            self.control_panel.cross_anchor_panel.append_log(f"ERROR load draft: {exc}")
        self._refresh_cross_panel(push_edits=True)

    def _cross_validate(self) -> None:
        if not self._cross_pull_edits():
            return
        issues = self.cross_wizard.validate_candidate()
        for issue in issues:
            self.control_panel.cross_anchor_panel.append_log(f"[{issue.level}] {issue.code}: {issue.message}")
        self._refresh_cross_panel()

    def _cross_plan(self, kind: str) -> None:
        if not self._cross_pull_edits():
            return
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
        if not self._cross_pull_edits():
            return
        panel = self.control_panel.cross_anchor_panel
        plan = self._cross_last_plan
        if plan is None:
            panel.append_log("ERROR: generate a plan first, then execute")
            return
        # Rebuild plan so candidate PWM edits are included in TARGET command
        try:
            if plan.name == "CROSS_TARGET_ABOVE":
                plan = self.cross_wizard.plan_target_above_test()
                self._cross_last_plan = plan
                panel.append_log(f"rebuilt plan with pwm={self.cross_wizard.candidate_pwm}")
            elif plan.name == "CROSS_CARRY_HIGH":
                plan = self.cross_wizard.plan_carry_high_test()
                self._cross_last_plan = plan
            elif plan.name == "CROSS_SAFE_RETURN":
                plan = self.cross_wizard.plan_safe_return()
                self._cross_last_plan = plan
        except Exception as exc:
            panel.append_log(f"ERROR rebuild plan: {exc}")
            return
        dry_checked = self.control_panel.stage5_panel.dry_run_checkbox.isChecked()
        try:
            mode, payload = self.cross_wizard.execute_plan(
                plan,
                gui_dry_run_checked=dry_checked,
            )
        except Exception as exc:
            panel.append_log(f"ERROR execute: {exc}")
            self._refresh_cross_panel()
            return

        if mode == "dry_run":
            for label, command in payload:
                panel.append_log(f"MOCK_TX {label}: {command}")
            panel.append_log("MODE=DRY_RUN/MOCK (arm will NOT move)")
            panel.append_log(
                "For LIVE: 1) uncheck Stage5 DRY RUN  2) return to observe  3) Execute plan again"
            )
            panel.append_log(
                f"REAL_SERIAL_WRITE_COUNT={self.cross_wizard.real_serial_write_count}"
            )
            if plan.name == "CROSS_SAFE_RETURN":
                self.cross_wizard.mark_safe_return_completed()
            self._refresh_cross_panel()
            return

        # LIVE path
        sequence = payload
        detail = "\n".join(sequence.action_names)
        reply = QMessageBox.question(
            self,
            "Confirm LIVE motion",
            f"Will send REAL serial actions:\n{sequence.display_name}\n{detail}\n\n"
            f"Target P({self.cross_wizard.current_row},{self.cross_wizard.current_col})\n"
            "Pump OFF. E-stop: $DST!\n\nProceed?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        if reply != QMessageBox.StandardButton.Yes:
            panel.append_log("LIVE cancelled by user")
            self._refresh_cross_panel()
            return

        if self.arm_worker.busy:
            panel.append_log("ERROR arm busy")
            return
        if not self.controller.is_connected:
            panel.append_log("ERROR serial not connected")
            return
        arm = self.state_machine.snapshot()
        if arm.state not in {
            ArmState.OBSERVE_IDLE,
            ArmState.OBSERVE_HOLD,
            ArmState.HOVERING,
            ArmState.UNKNOWN,
        }:
            panel.append_log(f"ERROR Arm={arm.state.value}, return to observe first")
            return
        try:
            if arm.state in {ArmState.OBSERVE_IDLE, ArmState.OBSERVE_HOLD, ArmState.UNKNOWN}:
                self.state_machine.begin_manual(sequence.name)
            ok = self.arm_worker.submit(sequence)
            if not ok:
                try:
                    self.state_machine.complete_manual()
                except Exception:
                    pass
                panel.append_log("ERROR worker rejected sequence")
                return
            self._cross_live_plan_name = sequence.name
            self._set_camera_arm_busy(True)
            panel.append_log(f"LIVE submitted: {sequence.name} {sequence.action_names}")
            panel.append_log("Wait for TX in main log...")
            LOGGER.info("CROSS LIVE submit %s", sequence.name)
        except Exception as exc:
            panel.append_log(f"ERROR live submit: {exc}")
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

    def _refresh_cross_panel(self, *, push_edits: bool = False) -> None:
        snap = self.cross_wizard.status_snapshot()
        self.control_panel.cross_anchor_panel.update_view(snap, push_edits=push_edits)


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

    def _refresh_stage5_ui(self, *, push_pwm: bool = False) -> None:
        """Refresh Stage5 labels. Never clobber user PWM edits unless push_pwm=True."""
        snap = self.stage5.stage_state.snapshot()
        target = self.stage5.target
        panel = self.control_panel.stage5_panel
        arm = self.state_machine.snapshot()
        if push_pwm and target.pwm:
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
