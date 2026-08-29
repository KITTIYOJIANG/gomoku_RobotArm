from __future__ import annotations

import json
import logging
from pathlib import Path
from typing import Any

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QDialog, QMessageBox, QVBoxLayout

from app.arm.sequences import (
    ActionStep,
    SequenceDefinition,
    pick_piece,
    retry_pick_piece,
)
from app.arm.ordered_motion import (
    BOARD_SAFE_RETURN_PHASE_TIME_MS,
    j1_first_sequence,
    j1_last_sequence,
)
from app.arm.state import ArmState
from app.config import AppConfig, PROJECT_ROOT
from app.integrated_v1.golden import SPATIAL_KEYS
from app.integrated_v1.movel import DropStatus, MoveLPlanner
from app.integrated_v1.points import all_points, parse_point_id
from app.main_window import MainWindow
from app.stage6.kinematics import ArmKinematics, KinematicsConfig
from app.stage7 import CalibrationMode
from app.stage7.baseline import point_id

from .context import CalibrationSummary, load_calibration_summary
from .drop_v1 import LiteDropSequenceBuilder, LiteDropStore
from .manual_movel import (
    P77ManualMoveLSequenceBuilder,
    P77ManualMoveLStore,
)
from .point_movel import (
    P77_DELTA_PREDICTION_SOURCE,
    P77_POINT_ID,
    PointMoveLSequenceBuilder,
    PointMoveLStore,
)
from .observe_pose import (
    action_pwm,
    action_times,
    build_action,
    load_observe_override,
    save_observe_override,
)
from .place_pose import (
    derive_place_contact,
    load_place_override,
    save_place_override,
)
from .p77_point import load_p77_point, save_p77_point
from .view import CalibrationLiteView
from .wizard import ANCHOR_SPECS, AnchorSpec, LiteWizardState, WizardPhase


LOGGER = logging.getLogger(__name__)
MAX_UNAPPLIED_POSE_DELTA = 150
PICK_CALIBRATION_RETURN_PHASE_TIME_MS = 2000


class CalibrationLiteWindow(MainWindow):
    """Guided shell that reuses MainWindow's runtime services and safety guards."""

    def __init__(
        self,
        config: AppConfig,
        *,
        dry_run: bool = False,
        default_test_pattern: bool = False,
        parent=None,
    ) -> None:
        self._lite_ready = False
        self._pending_after_return: tuple[str, Any] | None = None
        self._last_pwm_adjustment: tuple[int, int, int, bool] | None = None
        self._last_sent_pwm: dict[int, int] = {}
        self._current_spec: AnchorSpec | None = None
        self._summary: CalibrationSummary | None = None
        self._lite_camera_worker = None
        self._camera_anchor_pick_mode = False
        self._observe_last_adjustment: tuple[int, int, int, bool] | None = None
        self._observe_last_sent_pwm: dict[int, int] = {}
        self._pick_last_adjustment: tuple[int, int, int, bool] | None = None
        self._pick_last_sent_pwm: dict[int, int] = {}
        self._pick_calibration_parked = False
        self._place_last_adjustment: tuple[int, int, int, bool] | None = None
        self._place_last_sent_pwm: dict[int, int] = {}
        self._place_calibration_parked = False
        self._place_record: dict | None = None
        self._p77_point_pwm: dict[str, int] | None = None
        self._pump_is_on = False
        self._quick_special_flow = False
        self._drop_pose_state = "SAFE"
        self._drop_active_point: str | None = None
        self._drop_wizard_step = 1
        self._drop_completed_steps: set[int] = set()
        self._drop_above_confirmed = False
        self._drop_accuracy_confirmed = False
        self._drop_test_place_succeeded = False
        self._drop_waypoint_index = 0
        self._drop_at_final_correction = False
        self._drop_after_retract_step: int | None = None
        self._drop_after_safe_return_step: int | None = None
        self._drop_workflow_message = "选择棋盘点并载入已保存 ABOVE。"
        self._drop_live_pending_retract: set[str] = set()
        self._drop_live_verification_eligible: set[str] = set()
        self._drop_pending_back_home = False
        self._drop_last_applied: dict[int, int] = {}
        self._drop_profile_error: str | None = None
        self.drop_store: LiteDropStore | None = None
        self.drop_planner: MoveLPlanner | None = None
        self.drop_sequences: LiteDropSequenceBuilder | None = None
        self.manual_movel_store: P77ManualMoveLStore | None = None
        self.manual_movel_sequences: P77ManualMoveLSequenceBuilder | None = None
        self.point_movel_store: PointMoveLStore | None = None
        self.point_movel_sequences: PointMoveLSequenceBuilder | None = None
        self._point_movel_error: str | None = None
        self._point_movel_pose_state = "SAFE"
        self._point_movel_active_point: str | None = None
        self._point_movel_back_pending = False
        self._manual_movel_error: str | None = None
        self._manual_movel_view_index = 0
        self._manual_movel_pose_index: int | None = None
        self._manual_movel_back_pending = False
        self.wizard = LiteWizardState()
        super().__init__(
            config,
            dry_run=dry_run,
            default_test_pattern=default_test_pattern,
            parent=parent,
        )
        # Calibration Lite defines CARRY_HIGH_P77 itself as P77 + one J1 lift.
        # Prevent HoverPlanner from adding its legacy lift a second time.
        self.stage5.planner.carry_lift_001 = 0
        self._observe_override_path = (
            PROJECT_ROOT / "calibration" / "calibration_lite_observe_pose.json"
        )
        self._place_override_path = (
            PROJECT_ROOT / "calibration" / "calibration_lite_place_pose.json"
        )
        self._p77_point_path = PROJECT_ROOT / "calibration" / "p77.json"
        self._manual_movel_path = (
            PROJECT_ROOT / "calibration" / "p77_manual_movel.json"
        )
        self._point_movel_path = (
            PROJECT_ROOT / "calibration" / "point_movel.json"
        )
        self._stable_observe_actions = {
            name: self.actions.get(name) for name in ("OBSERVE_IDLE", "OBSERVE_HOLD")
        }
        self._stable_p77_actions = {
            name: self.actions.get(name)
            for name in (
                "P77_ABOVE_IDLE",
                "P77_ABOVE_HOLD",
                "P77_TOUCH_HOLD",
                "P77_TOUCH_RELEASE",
            )
        }
        self._stable_carry_actions = {
            name: self.actions.get(name)
            for name in ("CARRY_HIGH_P77_IDLE", "CARRY_HIGH_P77_HOLD")
        }
        self._stable_pick_actions = {
            name: self.actions.get(name)
            for name in ("SOURCE_TOUCH_IDLE", "SOURCE_TOUCH_HOLD")
        }
        self._stable_pick_action = self._stable_pick_actions["SOURCE_TOUCH_HOLD"]
        try:
            loaded = load_observe_override(self._observe_override_path, self.actions)
            LOGGER.info("[LITE][OBSERVE_OVERRIDE] loaded=%s", int(loaded))
        except Exception as exc:
            LOGGER.warning("[LITE][OBSERVE_OVERRIDE_INVALID] %s", exc)
        self._load_pick_runtime_override()

        legacy_root = self.takeCentralWidget()
        if legacy_root is None:  # pragma: no cover - QMainWindow contract
            raise RuntimeError("legacy central widget is missing")
        self._legacy_dialog = QDialog(self)
        self._legacy_dialog.setWindowTitle("Gomoku Robot · Advanced / Legacy")
        self._legacy_dialog.resize(1600, 920)
        QVBoxLayout(self._legacy_dialog).addWidget(legacy_root)
        self._legacy_root = legacy_root

        self.lite = CalibrationLiteView(default_port=config.serial.default_port)
        self.setCentralWidget(self.lite)
        title = "Gomoku Robot — Calibration Lite V1"
        self.setWindowTitle(title + (" — DRY RUN" if self.dry_run else ""))
        self.resize(1200, 900)
        self.setMinimumSize(900, 720)
        self._connect_lite_signals()
        self._load_lite_settings()

        self.lite_timer = QTimer(self)
        self.lite_timer.setInterval(200)
        self.lite_timer.timeout.connect(self._refresh_lite_status)
        self.lite_timer.start()
        self._lite_ready = True
        self._refresh_summary()
        self._load_committed_calibration_runtime()
        self._load_p77_point_runtime()
        self._initialize_drop_v1()
        self._initialize_manual_movel()
        self._initialize_point_movel()
        self._refresh_lite_status()
        LOGGER.info("CALIBRATION LITE READY; no automatic camera/COM connection")

    def _initialize_point_movel(self) -> None:
        """Initialize arbitrary-point ABOVE -> DROP calibration storage."""
        try:
            self.point_movel_store = PointMoveLStore(
                self._point_movel_path
            )
            self.point_movel_sequences = PointMoveLSequenceBuilder(
                actions=self.actions,
                store=self.point_movel_store,
                move_time_ms=1000,
            )

            # Bootstrap the already hardware-proven P77 calibration.
            if self.point_movel_store.point(P77_POINT_ID) is None:
                self.point_movel_store.save_drop(
                    point_id=P77_POINT_ID,
                    board=(7, 7),
                    above_pwm={
                        "J0": 1500,
                        "J1": 1230,
                        "J2": 870,
                        "J3": 1230,
                        "J4": 1500,
                    },
                    drop_pwm={
                        "J0": 1500,
                        "J1": 1090,
                        "J2": 790,
                        "J3": 1370,
                        "J4": 1500,
                    },
                    operator_confirmed=True,
                    hardware_verified=True,
                )
                self.point_movel_store.save()

            LOGGER.info(
                "[LITE][POINT_MOVEL] READY path=%s points=%d",
                self._point_movel_path,
                len(self.point_movel_store.data["points"]),
            )

        except Exception as exc:
            self.point_movel_store = None
            self.point_movel_sequences = None
            self._point_movel_error = str(exc)
            LOGGER.exception("[LITE][POINT_MOVEL] init failed: %s", exc)

    def _require_point_movel(
        self,
    ) -> tuple[
        PointMoveLStore,
        PointMoveLSequenceBuilder,
        LiteDropStore,
        LiteDropSequenceBuilder,
    ]:
        if self.point_movel_store is None or self.point_movel_sequences is None:
            raise RuntimeError(
                "Point MoveL is unavailable: "
                + (self._point_movel_error or "not initialized")
            )
        drop_store, _planner, drop_sequences = self._require_drop_v1()
        return (
            self.point_movel_store,
            self.point_movel_sequences,
            drop_store,
            drop_sequences,
        )

    @staticmethod
    def _point_movel_id(row: int, col: int) -> str:
        point = parse_point_id((int(row), int(col)))
        return f"P{point.row:02d}_{point.col:02d}"

    @staticmethod
    def _point_movel_pwm(pwm: dict[str, int]) -> dict[str, int]:
        return {
            f"J{joint}": int(
                pwm[f"J{joint}"]
                if f"J{joint}" in pwm
                else pwm[f"{joint:03d}"]
            )
            for joint in range(5)
        }

    def load_point_movel(self, row: int, col: int) -> None:
        try:
            store, _builder, drop_store, _drop_builder = self._require_point_movel()
            point_id_value = self._point_movel_id(row, col)
            if (
                self._point_movel_pose_state != "SAFE"
                and self._point_movel_active_point != point_id_value
            ):
                raise RuntimeError(
                    "Return the current point to Observation before loading another point."
                )
            above = self._point_movel_pwm(drop_store.above_pwm(point_id_value))
            prediction = store.initial_drop_from_p77_delta(above)
            record = store.point(point_id_value)
            if record is not None:
                stored_above = self._point_movel_pwm(record["above_pwm"])
                if stored_above != above:
                    raise RuntimeError(
                        f"{point_id_value} saved ABOVE differs from the current ABOVE source"
                    )
                drop = self._point_movel_pwm(record["drop_pwm"])
            else:
                drop = prediction
            self.lite.point_movel_panel.set_point(
                point_id=point_id_value,
                board=(int(row), int(col)),
                above_pwm=above,
                drop_pwm=drop,
                record=record,
                prediction_source=P77_DELTA_PREDICTION_SOURCE,
            )
            LOGGER.info(
                "[LITE][POINT_MOVEL][LOAD] point=%s saved=%s above=%s drop=%s",
                point_id_value,
                int(record is not None),
                above,
                drop,
            )
            self._refresh_lite_status()
        except Exception as exc:
            self._warn("Load Point MoveL", str(exc))

    def show_point_movel(self) -> None:
        try:
            self._require_point_movel()
            panel = self.lite.point_movel_panel
            self.load_point_movel(panel.row_field.value(), panel.col_field.value())
            self.lite.show_point_movel()
        except Exception as exc:
            self._warn("Manual MoveL Calibration V1", str(exc))

    def _submit_point_movel_sequence(self, sequence: SequenceDefinition) -> None:
        if not self.controller.is_connected:
            raise RuntimeError("Connect STM32 (or the Dry Run controller) first.")
        if self._is_estop_latched() or self.arm_worker.busy or self.state_machine.busy:
            raise RuntimeError("Robot is busy or Emergency Stop is latched.")
        if not self.dry_run:
            answer = QMessageBox.warning(
                self,
                "Live Point MoveL confirmation",
                f"Execute on the real arm?\n\n{sequence.display_name}\n\n"
                "Check clearance and keep Emergency Stop ready.",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                QMessageBox.StandardButton.No,
            )
            if answer != QMessageBox.StandardButton.Yes:
                raise RuntimeError("Live motion cancelled by operator.")
        self._start_sequence(
            sequence,
            lambda: self.state_machine.begin_manual(sequence.name),
        )

    def move_point_movel_above(self, point_id_value: str) -> None:
        try:
            _store, _builder, _drop_store, drop_builder = self._require_point_movel()
            panel = self.lite.point_movel_panel
            if point_id_value != panel.point_id:
                raise RuntimeError("Load the selected point before moving.")
            if self._point_movel_pose_state == "DROP":
                raise RuntimeError("Return DROP to ABOVE before another entry motion.")
            if self._point_movel_pose_state not in {"SAFE", "ABOVE"}:
                raise RuntimeError("Point MoveL pose is unknown; recover before moving.")
            if self._point_movel_pose_state == "ABOVE":
                raise RuntimeError("Already at this point's ABOVE.")
            if (
                not self.dry_run
                and self.state_machine.snapshot().state != ArmState.OBSERVE_IDLE
            ):
                raise RuntimeError("Live Move ABOVE must start from OBSERVE_IDLE.")
            source = drop_builder.build_move_above(point_id_value)
            sequence = SequenceDefinition(
                name=f"MANUAL:POINT_MOVEL:ABOVE:{point_id_value}",
                display_name=f"Point MoveL safe entry to ABOVE {point_id_value}",
                steps=source.steps,
                requires_board=source.requires_board,
            )
            self._submit_point_movel_sequence(sequence)
        except Exception as exc:
            self._warn("Move ABOVE", str(exc))

    def move_point_movel_drop(self, point_id_value: str, drop_pwm: object) -> None:
        try:
            store, builder, _drop_store, _drop_builder = self._require_point_movel()
            if not isinstance(drop_pwm, dict):
                raise RuntimeError("Invalid absolute DROP PWM payload")
            if (
                self._point_movel_active_point != point_id_value
                or self._point_movel_pose_state not in {"ABOVE", "DROP"}
            ):
                raise RuntimeError("Move this point to ABOVE before moving DROP.")
            if point_id_value == P77_POINT_ID:
                raise RuntimeError("Golden P07_07 is protected; use the mature P77 page.")
            store._normalize_pwm(drop_pwm)
            self._submit_point_movel_sequence(
                builder.build_move_drop(point_id_value, drop_pwm)
            )
        except Exception as exc:
            self._warn("Move Current DROP", str(exc))

    def save_point_movel_drop(self, point_id_value: str, drop_pwm: object) -> None:
        try:
            store, _builder, _drop_store, _drop_builder = self._require_point_movel()
            panel = self.lite.point_movel_panel
            if not isinstance(drop_pwm, dict):
                raise RuntimeError("Invalid absolute DROP PWM payload")
            if (
                self._point_movel_active_point != point_id_value
                or self._point_movel_pose_state != "DROP"
            ):
                raise RuntimeError("Move Current DROP before saving this point.")
            if point_id_value == P77_POINT_ID:
                raise RuntimeError("Golden P07_07 is protected from overwrite.")
            saved_record = store.point(point_id_value)
            if (
                saved_record
                and saved_record.get("hardware_verified")
                and not panel.recalibration_unlocked
            ):
                raise RuntimeError(
                    "HARDWARE VERIFIED point is protected. "
                    "Click Recalibrate This Point before replacing it."
                )
            prediction = store.initial_drop_from_p77_delta(panel.above_pwm)
            point = parse_point_id(point_id_value)
            store.save_drop(
                point_id=point_id_value,
                board=(point.row, point.col),
                above_pwm=panel.above_pwm,
                drop_pwm=drop_pwm,
                operator_confirmed=False,
                hardware_verified=False,
                predicted_drop_pwm=prediction,
                prediction_source=P77_DELTA_PREDICTION_SOURCE,
                allow_hardware_verified_overwrite=panel.recalibration_unlocked,
            )
            store.save()
            self.load_point_movel(point.row, point.col)
            self.lite.point_movel_panel.status_label.setText(
                f"{point_id_value} saved · operator_confirmed=false · "
                "hardware_verified=false"
            )
        except Exception as exc:
            self._warn("Save Current DROP", str(exc))

    def begin_point_movel_recalibration(self, point_id_value: str) -> None:
        try:
            store, _builder, _drop_store, _drop_builder = self._require_point_movel()
            panel = self.lite.point_movel_panel
            if point_id_value != panel.point_id:
                raise RuntimeError("Load this point before starting recalibration.")
            if point_id_value == P77_POINT_ID:
                raise RuntimeError("Golden P07_07 remains protected; use the P77 page.")
            if self.arm_worker.busy or self.state_machine.busy:
                raise RuntimeError("Robot is busy; wait before editing calibration data.")
            if self._is_estop_latched():
                raise RuntimeError("Emergency Stop is latched.")
            record = store.point(point_id_value)
            if not record or not record.get("hardware_verified"):
                raise RuntimeError("This point is not a protected HARDWARE VERIFIED point.")
            panel.set_recalibration_unlocked(True)
            self._refresh_lite_status()
            LOGGER.warning(
                "[LITE][POINT_MOVEL][RECALIBRATION_DRAFT] "
                "point=%s persisted_unchanged=1",
                point_id_value,
            )
        except Exception as exc:
            self._warn("Recalibrate Point MoveL", str(exc))

    def confirm_point_movel_drop(self, point_id_value: str) -> None:
        try:
            store, _builder, _drop_store, _drop_builder = self._require_point_movel()
            panel = self.lite.point_movel_panel
            if self.dry_run:
                raise RuntimeError("Dry Run cannot create HARDWARE VERIFIED evidence.")
            if (
                self._point_movel_active_point != point_id_value
                or self._point_movel_pose_state != "DROP"
            ):
                raise RuntimeError("Reach the saved DROP before hardware confirmation.")
            record = store.point(point_id_value)
            if record is None or record.get("drop_pwm") != panel.drop_pwm():
                raise RuntimeError("Save the displayed DROP before confirmation.")
            answer = QMessageBox.warning(
                self,
                "Confirm Point MoveL hardware result",
                f"Confirm {point_id_value} DROP was physically verified on hardware?",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                QMessageBox.StandardButton.No,
            )
            if answer != QMessageBox.StandardButton.Yes:
                raise RuntimeError("Hardware confirmation cancelled by operator.")
            store.confirm_hardware(point_id_value)
            store.save()
            point = parse_point_id(point_id_value)
            self.load_point_movel(point.row, point.col)
        except Exception as exc:
            self._warn("Confirm Hardware", str(exc))

    def return_point_movel_above(self, point_id_value: str) -> None:
        try:
            _store, builder, _drop_store, _drop_builder = self._require_point_movel()
            panel = self.lite.point_movel_panel
            if (
                self._point_movel_active_point != point_id_value
                or self._point_movel_pose_state != "DROP"
            ):
                raise RuntimeError("Return ABOVE is available only from this point's DROP.")
            self._submit_point_movel_sequence(
                builder.build_return_above(point_id_value, panel.above_pwm)
            )
        except Exception as exc:
            self._warn("Return ABOVE", str(exc))

    def back_from_point_movel(self) -> None:
        try:
            _store, _builder, _drop_store, drop_builder = self._require_point_movel()
            if self.arm_worker.busy or self.state_machine.busy:
                raise RuntimeError("Robot is busy; wait before leaving this page.")
            if self._point_movel_pose_state == "SAFE":
                self.lite.show_home()
                return
            if self._point_movel_pose_state == "DROP":
                raise RuntimeError("Return DROP to ABOVE before leaving this page.")
            if self._point_movel_pose_state != "ABOVE" or not self._point_movel_active_point:
                raise RuntimeError("Point MoveL pose is unknown; recover before leaving.")
            source = drop_builder.build_safe_return_from_above(
                self._point_movel_active_point
            )
            self._point_movel_back_pending = True
            self._submit_point_movel_sequence(
                SequenceDefinition(
                    name=(
                        "MANUAL:POINT_MOVEL:RETURN_OBSERVATION:"
                        f"{self._point_movel_active_point}"
                    ),
                    display_name=(
                        "Point MoveL J1-FIRST return to Observation "
                        f"{self._point_movel_active_point}"
                    ),
                    steps=source.steps,
                    requires_board=source.requires_board,
                )
            )
        except Exception as exc:
            self._point_movel_back_pending = False
            self._warn("Back to Home", str(exc))
            
    def _connect_lite_signals(self) -> None:
        view = self.lite
        view.connect_serial_requested.connect(self.connect_serial)
        view.disconnect_serial_requested.connect(self.disconnect_serial)
        view.connect_camera_requested.connect(self.connect_camera)
        view.disconnect_camera_requested.connect(self.disconnect_camera)
        view.relocalize_requested.connect(self.relocalize_camera)
        view.quick_calibration_requested.connect(self.start_quick_calibration)
        view.continue_requested.connect(self.continue_calibration)
        view.return_observe_requested.connect(self.start_return_to_observe)
        view.pick_requested.connect(self.start_pick)
        view.place_requested.connect(self.show_drop_v1)
        view.estop_requested.connect(self.emergency_stop)
        view.pump_toggle_requested.connect(self.set_manual_pump)
        view.recover_requested.connect(self.stage5_recover)
        view.move_above_requested.connect(self._move_or_return)
        view.confirm_anchor_requested.connect(self.confirm_current_anchor)
        view.generate_requested.connect(self.generate_full_board)
        view.begin_test_requested.connect(self.begin_quick_test)
        view.test_accurate_requested.connect(self.confirm_test_accurate)
        view.test_inaccurate_requested.connect(self.correct_test_point)
        view.commit_requested.connect(self.commit_calibration)
        view.add_anchor_requested.connect(self.add_anchor)
        view.camera_anchor_pick_requested.connect(self.begin_camera_anchor_selection)
        view.observe_calibration_requested.connect(self.show_observe_calibration)
        view.observe_move_requested.connect(self.start_return_to_observe)
        view.observe_save_requested.connect(self.save_observe_calibration)
        view.observe_pwm_apply_requested.connect(self.apply_observe_pwm_joint)
        view.observe_pwm_target_changed.connect(self.remember_observe_pwm_change)
        view.observe_pwm_undo_requested.connect(self.undo_observe_adjustment)
        view.pick_calibration_requested.connect(self.show_pick_calibration)
        view.pick_calibration_move_requested.connect(self.move_to_pick_calibration)
        view.pick_calibration_save_requested.connect(self.save_pick_calibration)
        view.pick_calibration_pwm_apply_requested.connect(
            self.apply_pick_calibration_pwm_joint
        )
        view.pick_calibration_pwm_target_changed.connect(
            self.remember_pick_calibration_pwm_change
        )
        view.pick_calibration_pwm_undo_requested.connect(
            self.undo_pick_calibration_adjustment
        )
        view.place_calibration_requested.connect(self.show_drop_v1)
        view.manual_movel_requested.connect(self.show_manual_movel)
        view.point_movel_requested.connect(self.show_point_movel)
        view.p77_full_cycle_requested.connect(
            self.run_manual_movel_full_cycle
        )
        view.place_calibration_move_requested.connect(self.move_to_place_calibration)
        view.place_calibration_accurate_requested.connect(
            self.confirm_place_calibration_accurate
        )
        view.place_calibration_inaccurate_requested.connect(
            self.confirm_place_calibration_inaccurate
        )
        view.place_calibration_pwm_apply_requested.connect(
            self.apply_place_calibration_pwm_joint
        )
        view.place_calibration_pwm_target_changed.connect(
            self.remember_place_calibration_pwm_change
        )
        view.place_calibration_pwm_undo_requested.connect(
            self.undo_place_calibration_adjustment
        )
        view.review_target_requested.connect(self.review_calibrated_target)
        view.back_home_requested.connect(self.back_home)
        view.open_legacy_requested.connect(self.open_legacy)
        view.pwm_apply_requested.connect(self.apply_pwm_joint)
        view.pwm_target_changed.connect(self.remember_pwm_target_change)
        view.pwm_undo_requested.connect(self.undo_last_adjustment)

        view.camera_preview.image_clicked.connect(self.on_lite_camera_clicked)
        drop = view.drop_v1_panel
        drop.point_requested.connect(self.select_drop_v1_point)
        drop.generate_requested.connect(self.generate_drop_v1)
        drop.generate_all_requested.connect(self.generate_all_drop_v1)
        drop.preview_requested.connect(self.preview_drop_v1)
        drop.move_above_requested.connect(self.move_drop_v1_above)
        drop.above_confirmed_requested.connect(self.confirm_drop_v1_above)
        drop.above_incorrect_requested.connect(self.reject_drop_v1_above)
        drop.next_waypoint_requested.connect(self.next_drop_v1_waypoint)
        drop.previous_waypoint_requested.connect(self.previous_drop_v1_waypoint)
        drop.move_drop_requested.connect(self.move_drop_v1)
        drop.retract_requested.connect(self.retract_drop_v1)
        drop.drop_accurate_requested.connect(self.confirm_drop_v1_accuracy)
        drop.correction_mode_requested.connect(self.enter_drop_v1_correction)
        drop.drop_mark_verified_requested.connect(self.mark_drop_v1_workflow_verified)
        drop.return_retest_requested.connect(self.return_to_retest_drop_v1)
        drop.test_place_requested.connect(self.test_place_drop_v1)
        drop.correction_apply_requested.connect(self.apply_drop_v1_joint)
        drop.correction_save_requested.connect(self.save_drop_v1_correction)
        drop.verify_requested.connect(self.verify_drop_v1)
        drop.safe_return_requested.connect(self.safe_return_drop_v1)
        drop.emergency_stop_requested.connect(self.emergency_stop)
        drop.back_requested.connect(self.back_from_drop_v1)
        manual = view.manual_movel_panel
        manual.move_requested.connect(self.move_manual_movel_step)
        manual.save_requested.connect(self.save_manual_movel_step)
        manual.confirm_requested.connect(self.confirm_manual_movel_step)
        manual.next_requested.connect(self.next_manual_movel_step)
        manual.previous_requested.connect(self.previous_manual_movel_step)
        manual.return_previous_requested.connect(
            self.return_previous_manual_movel_step
        )
        manual.return_above_requested.connect(self.return_manual_movel_above)
        manual.set_drop_requested.connect(self.set_manual_movel_drop)
        manual.full_cycle_requested.connect(self.run_manual_movel_full_cycle)
        manual.back_requested.connect(self.back_from_manual_movel)
        manual.emergency_stop_requested.connect(self.emergency_stop)
        point_manual = view.point_movel_panel
        point_manual.load_requested.connect(self.load_point_movel)
        point_manual.move_above_requested.connect(self.move_point_movel_above)
        point_manual.move_drop_requested.connect(self.move_point_movel_drop)
        point_manual.save_requested.connect(self.save_point_movel_drop)
        point_manual.confirm_requested.connect(self.confirm_point_movel_drop)
        point_manual.recalibrate_requested.connect(
            self.begin_point_movel_recalibration
        )
        point_manual.return_above_requested.connect(self.return_point_movel_above)
        point_manual.back_requested.connect(self.back_from_point_movel)
        point_manual.emergency_stop_requested.connect(self.emergency_stop)

    def _build_power_on_observe_sequence(self) -> SequenceDefinition:
        observe = self.actions.get("OBSERVE_IDLE")
        times = {f"{joint:03d}": 5000 for joint in range(8)}
        action_name = "LITE_POWER_ON_TO_OBSERVE_IDLE"
        self.actions.register_runtime(
            build_action(action_name, action_pwm(observe), times)
        )
        return SequenceDefinition(
            name="RETURN_TO_OBSERVE",
            display_name="上电直立位缓慢进入观察位（5 秒）",
            steps=(ActionStep(action_name),),
        )

    def start_return_to_observe(self) -> None:
        if self.state_machine.snapshot().state != ArmState.UNKNOWN:
            super().start_return_to_observe()
            return
        if (
            not self.controller.is_connected
            or self.arm_worker.busy
            or self.state_machine.busy
            or self._is_estop_latched()
        ):
            super().start_return_to_observe()
            return
        sequence = self._build_power_on_observe_sequence()
        self._start_sequence(sequence, self.state_machine.begin_return_to_observe)
        LOGGER.info(
            "[LITE][POWER_ON_TO_OBSERVE] action=%s time_ms=5000",
            sequence.action_names[0],
        )

    def connect_camera(self) -> None:
        super().connect_camera()
        worker = self.camera_worker
        if worker is not None and worker is not self._lite_camera_worker:
            worker.frame_ready.connect(self.lite.camera_preview.set_frame)
            self._lite_camera_worker = worker
        if worker is not None:
            self.lite.camera_preview_group.setChecked(True)

    def disconnect_camera(self) -> None:
        super().disconnect_camera()
        self._lite_camera_worker = None
        if self._lite_ready:
            self.lite.camera_preview.set_camera_status("DISCONNECTED")

    def relocalize_camera(self) -> None:
        worker = self.camera_worker
        if worker is None:
            self._warn("快速重新定位", "请先连接 Camera。")
            return
        self.board_locked = False
        self.target_visible = False
        worker.set_selected_target(None, None)
        worker.request_relocalize()
        self.camera_panel.set_target_text("目标: 等待重新定位")
        self.lite.camera_preview.set_target_text("目标: 等待重新定位")
        self.lite.camera_preview.set_board_status(
            "RELOCALIZING", "Manual relocalization requested"
        )
        self.lite.board_status.set_status("重新定位中…", "warning")
        self.lite.home_message.setText("正在重新定位棋盘，请保持相机稳定并让 Tag 可见…")
        self._sync_stage5_context(reason="lite_manual_relocalize")
        LOGGER.info("[LITE][VISION_RELOCALIZE_REQUESTED]")

    def begin_camera_anchor_selection(self) -> None:
        try:
            self.stage7.require_session()
            if self.camera_worker is None:
                raise RuntimeError("请先连接 Camera。")
            if not self.board_locked:
                raise RuntimeError("棋盘尚未锁定，请先快速重新定位。")
            self._camera_anchor_pick_mode = True
            self.lite.camera_preview_group.setChecked(True)
            self.lite.camera_preview.set_target_text("添加 Anchor：请点击棋盘交点")
            self.lite.home_message.setText("添加 Anchor：请在左侧 Camera 画面点击棋盘交点。")
            LOGGER.info("[LITE][CAMERA_ANCHOR_PICK_ARMED]")
        except Exception as exc:
            self._warn("从 Camera 添加 Anchor", str(exc))

    def on_lite_camera_clicked(self, image_x: float, image_y: float) -> None:
        if not self._camera_anchor_pick_mode:
            self.on_image_clicked(image_x, image_y)
            return
        if self.stage5.stage_state.is_moving() or self.arm_worker.busy:
            self._warn("选择 Anchor", "机械臂运动中，请等待停止后再点击。")
            return
        selection = self.stage5.handle_click(
            image_x, image_y, self.config.vision.board_size
        )
        if not selection.accepted or selection.row is None or selection.col is None:
            self.lite.camera_preview.set_target_text(
                f"未选中交点：{selection.reason or '请重新点击'}"
            )
            return
        row, col = int(selection.row), int(selection.col)
        self._camera_anchor_pick_mode = False
        if self.camera_worker is not None:
            self.camera_worker.set_selected_target(row, col)
        text = f"添加 Anchor：{point_id(row, col)}"
        self.camera_panel.set_target_text(text)
        self.lite.camera_preview.set_target_text(text)
        LOGGER.info("[LITE][CAMERA_ANCHOR_SELECTED] point=%s", point_id(row, col))
        self.add_anchor(row, col)

    def _on_camera_status(self, status: str) -> None:
        super()._on_camera_status(status)
        if self._lite_ready:
            self.lite.camera_preview.set_camera_status(status)

    def _on_board_status(
        self,
        locked: bool,
        reason: str,
        target_visible: bool,
        display_status: str,
        corner_status: str,
    ) -> None:
        super()._on_board_status(
            locked, reason, target_visible, display_status, corner_status
        )
        if self._lite_ready:
            self.lite.camera_preview.set_board_status(display_status, reason)

    def _load_lite_settings(self) -> None:
        path = PROJECT_ROOT / "config" / "calibration_lite.json"
        try:
            self.lite_settings = json.loads(path.read_text(encoding="utf-8"))
        except Exception:
            self.lite_settings = {
                "robot_tag_ids": [10, 11, 12, 13],
                "board_tag_ids": [15, 16, 17, 18],
                "robot_reference_enabled": False,
            }

    def _refresh_summary(self) -> None:
        try:
            self._summary = load_calibration_summary(self.stage7.settings)
            summary = self._summary
            self.lite.set_summary(
                date=summary.date,
                anchors=summary.anchors,
                generated=summary.generated_points,
                status=summary.status,
            )
            self.lite.continue_button.setEnabled(
                summary.session_path is not None and summary.session_path.is_file()
            )
        except Exception as exc:
            LOGGER.warning("CALIBRATION LITE summary unavailable: %s", exc)
            self._summary = None
            self.lite.set_summary(date="—", anchors=0, generated=0, status="未找到")
            self.lite.continue_button.setEnabled(False)

    def _load_committed_calibration_runtime(self) -> bool:
        """Load the committed candidate and apply its P77 correction at runtime."""
        try:
            deployment = json.loads(
                self.stage7.settings.current_deployment_path.read_text(encoding="utf-8")
            )
            if deployment.get("deployment_state") != "CANDIDATE_COMMITTED":
                return False
            raw_path = str(deployment.get("session_path") or "").strip()
            if not raw_path:
                return False
            session_path = Path(raw_path)
            if not session_path.is_absolute():
                session_path = PROJECT_ROOT / session_path
            session = self.stage7.load_session(session_path)
            if session.candidate_stale or len(session.generated_points) != 225:
                raise RuntimeError("committed calibration session is incomplete")
            self._apply_p77_runtime_from_session(session)
            LOGGER.info(
                "[LITE][COMMITTED_RUNTIME_LOADED] session=%s points=%s",
                session_path,
                len(session.generated_points),
            )
            return True
        except FileNotFoundError:
            return False
        except Exception as exc:
            LOGGER.warning("[LITE][COMMITTED_RUNTIME_LOAD_FAILED] %s", exc)
            return False

    def _apply_p77_runtime_from_session(self, session) -> None:
        record = session.generated_points[point_id(7, 7)]
        calibrated = record.get("new_pwm") or {}
        self._apply_p77_runtime_from_pwm(calibrated)

    def _apply_p77_runtime_from_pwm(self, calibrated) -> None:
        stable_above = action_pwm(self._stable_p77_actions["P77_ABOVE_IDLE"])
        delta = {
            f"{joint:03d}": int(calibrated[f"{joint:03d}"])
            - int(stable_above[f"{joint:03d}"])
            for joint in range(5)
        }
        for name, stable_action in self._stable_p77_actions.items():
            pwm = action_pwm(stable_action)
            for joint in range(5):
                key = f"{joint:03d}"
                pwm[key] = max(
                    self.actions.pwm_min,
                    min(self.actions.pwm_max, int(pwm[key]) + delta[key]),
                )
            if name == "P77_TOUCH_RELEASE":
                pwm["005"] = 1500
            self.actions.register_runtime(
                build_action(name, pwm, action_times(stable_action))
            )
        carry_j1 = min(
            int(self.stage7.limits.joint_max[1]),
            int(calibrated["001"]) + 60,
        )
        for name, stable_action in self._stable_carry_actions.items():
            carry_pwm = action_pwm(stable_action)
            for joint in range(5):
                key = f"{joint:03d}"
                carry_pwm[key] = int(calibrated[key])
            carry_pwm["001"] = carry_j1
            self.actions.register_runtime(
                build_action(name, carry_pwm, action_times(stable_action))
            )
        LOGGER.info("[LITE][P77_RUNTIME_DELTA] %s", delta)
        LOGGER.info(
            "[LITE][CARRY_HIGH_FROM_P77] p77=%s j1_lift=60 carry_j1=%s",
            {f"{joint:03d}": int(calibrated[f"{joint:03d}"]) for joint in range(5)},
            carry_j1,
        )

    def _load_p77_point_runtime(self) -> bool:
        try:
            pwm = load_p77_point(self._p77_point_path)
            if pwm is None:
                return False
            self._p77_point_pwm = pwm
            self._apply_p77_runtime_from_pwm(pwm)
            LOGGER.info(
                "[LITE][P77_POINT_LOADED] path=%s new_pwm=%s",
                self._p77_point_path,
                pwm,
            )
            return True
        except Exception as exc:
            self._p77_point_pwm = None
            LOGGER.warning("[LITE][P77_POINT_LOAD_FAILED] %s", exc)
            return False

    def _authoritative_point_pwm(self, row: int, col: int) -> dict[str, int]:
        if (int(row), int(col)) == (7, 7) and self._p77_point_pwm is not None:
            return dict(self._p77_point_pwm)
        return self.stage7.point_pwm(int(row), int(col))

    def _initialize_drop_v1(self) -> None:
        """Load an independent Lite DROP file without mutating saved ABOVE assets."""
        try:
            settings = dict(self.lite_settings.get("drop_v1") or {})
            if settings.get("auto_hardware_batch", False):
                raise RuntimeError("drop_v1.auto_hardware_batch must remain false")
            profile_path = PROJECT_ROOT / settings.get(
                "profile_path", "calibration/calibration_lite_drop_v1.json"
            )
            kinematics_path = PROJECT_ROOT / settings.get(
                "kinematics_path", "config/arm_kinematics.json"
            )
            above = {
                point.as_tuple(): self._authoritative_point_pwm(point.row, point.col)
                for point in all_points()
            }
            store = LiteDropStore(profile_path, library=self.actions)
            store.load_or_initialize(
                above,
                source={
                    "above_policy": "saved_lite_above_with_five_golden_overlays",
                    "golden_above_count": 5,
                    "stage5_asset": "calibration/stage5_board_calibration.json",
                    "arm_actions_asset": "config/arm_actions.json",
                },
            )
            kinematics = ArmKinematics(KinematicsConfig.load(kinematics_path))
            if int(kinematics.config.pump_joint_id) != 5:
                raise RuntimeError("kinematics pump_joint_id must be J5")
            self.drop_store = store
            self.drop_planner = MoveLPlanner(
                store,
                kinematics=kinematics,
                target_descent_mm=float(settings.get("target_descent_mm", 25.0)),
                step_mm=float(settings.get("waypoint_step_mm", 5.0)),
                max_waypoint_joint_delta_pwm=int(
                    settings.get("max_waypoint_joint_delta_pwm", 400)
                ),
            )
            self.drop_sequences = LiteDropSequenceBuilder(
                actions=self.actions,
                store=store,
                move_time_ms=int(settings.get("move_time_ms", 1000)),
                vacuum_build_ms=int(settings.get("vacuum_build_ms", 700)),
                release_ms=int(settings.get("release_ms", 700)),
            )
            self._drop_profile_error = None
            LOGGER.info(
                "[LITE_DROP_V1][READY] path=%s dry_run=%s j5_kinematics=0",
                profile_path,
                int(self.dry_run),
            )
        except Exception as exc:
            self.drop_store = None
            self.drop_planner = None
            self.drop_sequences = None
            self._drop_profile_error = str(exc)
            LOGGER.error("[LITE_DROP_V1][LOAD_FAILED] %s", exc)

    def _require_drop_v1(self) -> tuple[LiteDropStore, MoveLPlanner, LiteDropSequenceBuilder]:
        if self.drop_store is None or self.drop_planner is None or self.drop_sequences is None:
            raise RuntimeError(
                "Lite DROP V1 is unavailable: " + (self._drop_profile_error or "not initialized")
            )
        return self.drop_store, self.drop_planner, self.drop_sequences

    def _initialize_manual_movel(self) -> None:
        """Load the isolated P77 manual path without reading auto waypoints."""
        try:
            joint_limits = (
                self.drop_store.joint_limits
                if self.drop_store is not None
                else {joint: (550, 2450) for joint in range(5)}
            )
            store = P77ManualMoveLStore(
                self._manual_movel_path,
                joint_limits=joint_limits,
            )
            store.load_or_initialize()
            self.manual_movel_store = store
            self.manual_movel_sequences = P77ManualMoveLSequenceBuilder(
                actions=self.actions,
                store=store,
                move_time_ms=1000,
                vacuum_build_ms=self.config.timing.vacuum_build_ms,
                release_ms=self.config.timing.release_ms,
            )
            self._manual_movel_error = None
            self._refresh_manual_movel_panel()
            LOGGER.info(
                "[P77_MANUAL_MOVEL][READY] path=%s steps=%s dry_run=%s",
                self._manual_movel_path,
                store.step_count(),
                int(self.dry_run),
            )
        except Exception as exc:
            self.manual_movel_store = None
            self.manual_movel_sequences = None
            self._manual_movel_error = str(exc)
            LOGGER.error("[P77_MANUAL_MOVEL][LOAD_FAILED] %s", exc)

    def _require_manual_movel(
        self,
    ) -> tuple[P77ManualMoveLStore, P77ManualMoveLSequenceBuilder]:
        if self.manual_movel_store is None or self.manual_movel_sequences is None:
            raise RuntimeError(
                "P77 Manual MoveL is unavailable: "
                + (self._manual_movel_error or "not initialized")
            )
        return self.manual_movel_store, self.manual_movel_sequences

    def _load_pick_runtime_override(self) -> bool:
        try:
            record = self.stage7.pick_poses.get("PICK_DOWN")
            if not record.get("updated_at"):
                return False
            self._apply_pick_runtime(record["new_pwm"])
            LOGGER.info(
                "[LITE][PICK_RUNTIME_LOADED] path=%s updated=%s",
                self.stage7.settings.pick_pose_path,
                record.get("updated_at"),
            )
            return True
        except Exception as exc:
            LOGGER.warning("[LITE][PICK_RUNTIME_LOAD_FAILED] %s", exc)
            return False

    def _apply_pick_runtime(self, spatial_pwm) -> None:
        for name, stable in self._stable_pick_actions.items():
            pwm = action_pwm(stable)
            for joint in range(5):
                key = f"{joint:03d}"
                pwm[key] = int(spatial_pwm[key])
            self.actions.register_runtime(
                build_action(name, pwm, action_times(stable))
            )

    def _derived_place_pwm(self) -> dict[str, int]:
        correction = (
            None
            if self._place_record is None
            else self._place_record.get("correction_delta")
        )
        target = derive_place_contact(
            self._current_p77_above_pwm(),
            action_pwm(self._stable_p77_actions["P77_ABOVE_IDLE"]),
            action_pwm(self._stable_p77_actions["P77_TOUCH_HOLD"]),
            correction,
        )
        return {
            key: max(self.actions.pwm_min, min(self.actions.pwm_max, int(value)))
            for key, value in target.items()
        }

    def _current_p77_above_pwm(self) -> dict[str, int]:
        if self._p77_point_pwm is not None:
            return dict(self._p77_point_pwm)
        session = self.stage7.session
        key = point_id(7, 7)
        if session is not None:
            if key in session.anchors:
                return dict(session.anchors[key]["new_pwm"])
            if key in session.generated_points:
                return dict(session.generated_points[key]["new_pwm"])
        return action_pwm(self.actions.get("P77_ABOVE_IDLE"))

    def _register_current_place_above(self) -> dict[str, int]:
        above_pwm = action_pwm(self.actions.get("P77_ABOVE_IDLE"))
        above_pwm.update(self._current_p77_above_pwm())
        above_pwm["005"] = 1500
        current = self.actions.get("P77_ABOVE_IDLE")
        self.actions.register_runtime(
            build_action("P77_ABOVE_IDLE", above_pwm, action_times(current))
        )
        return above_pwm

    def _load_place_runtime_override(self) -> bool:
        try:
            self._place_record = load_place_override(
                self._place_override_path,
                library=self.actions,
                stable_above=self._stable_p77_actions["P77_ABOVE_IDLE"],
                stable_touch_hold=self._stable_p77_actions["P77_TOUCH_HOLD"],
                stable_touch_release=self._stable_p77_actions["P77_TOUCH_RELEASE"],
            )
            if self._place_record is None:
                return False
            LOGGER.info(
                "[LITE][PLACE_RUNTIME_LOADED] path=%s updated=%s",
                self._place_override_path,
                self._place_record.get("updated_at"),
            )
            return True
        except Exception as exc:
            self._place_record = None
            LOGGER.warning("[LITE][PLACE_RUNTIME_LOAD_FAILED] %s", exc)
            return False

    def set_manual_pump(self, enabled: bool) -> None:
        requested = bool(enabled)
        try:
            if not self.controller.is_connected:
                raise RuntimeError("请先连接 STM32。")
            if self._is_estop_latched():
                raise RuntimeError("急停已锁存。")
            if self.arm_worker.busy or self.state_machine.busy:
                raise RuntimeError("机械臂运动中，泵嘴开关已锁定。")
            if requested:
                self.controller.pump_on()
            else:
                self.controller.pump_off()
            self._pump_is_on = requested
            self.lite.set_pump_state(requested)
            LOGGER.warning("[LITE][MANUAL_PUMP] state=%s", "ON" if requested else "OFF")
        except Exception as exc:
            self.lite.set_pump_state(self._pump_is_on)
            self._warn("泵嘴控制失败", str(exc))

    def start_pick(self) -> None:
        retry = self.state_machine.snapshot().state == ArmState.OBSERVE_HOLD
        sequence = (
            retry_pick_piece(self.config.timing.vacuum_build_ms)
            if retry
            else pick_piece(self.config.timing.vacuum_build_ms)
        )
        sequence = self._prepare_pick_execution_sequence(sequence)
        begin = self.state_machine.begin_pick_retry if retry else self.state_machine.begin_pick
        self._start_sequence(sequence, begin)
        LOGGER.info(
            "[LITE][PICK_J1_LAST] retry=%s actions=%s",
            int(retry),
            sequence.action_names,
        )

    def _prepare_pick_execution_sequence(
        self, sequence: SequenceDefinition
    ) -> SequenceDefinition:
        """Stage only the PICK_DOWN approach; preserve the proven lift sequence."""
        approach = j1_last_sequence(
            self.actions,
            SequenceDefinition(
                name=f"{sequence.name}:J1_LAST_APPROACH",
                display_name=sequence.display_name,
                steps=(ActionStep("SOURCE_TOUCH_IDLE"),),
            ),
            initial_action=self.actions.get("OBSERVE_IDLE"),
            runtime_prefix="LITE_PICK_EXEC",
        )
        staged_steps = []
        replaced = False
        for step in sequence.steps:
            if (
                not replaced
                and isinstance(step, ActionStep)
                and step.action_name == "SOURCE_TOUCH_IDLE"
            ):
                staged_steps.extend(approach.steps)
                replaced = True
            else:
                staged_steps.append(step)
        if not replaced:
            raise RuntimeError("取料序列缺少 SOURCE_TOUCH_IDLE 安全接近动作。")
        return SequenceDefinition(
            name=sequence.name,
            display_name=sequence.display_name,
            steps=tuple(staged_steps),
            requires_board=sequence.requires_board,
        )

    def _build_lite_stage7_return_sequence(
        self,
        *,
        row: int,
        col: int,
        parked_pwm: dict[str, int],
    ) -> SequenceDefinition:
        """Return directly from the current ABOVE with J1 FIRST."""
        del row, col
        sequence = SequenceDefinition(
            name="STAGE7_SAFE_RETURN",
            display_name="Stage 7 ABOVE to OBSERVE (J1 first)",
            steps=(ActionStep("OBSERVE_IDLE"),),
        )
        stable_center = self._stable_p77_actions["P77_ABOVE_IDLE"]
        parked_action = build_action(
            "LITE_RETURN_START",
            {**parked_pwm, "005": 1500, "006": 1500, "007": 1500},
            action_times(stable_center),
        )
        return j1_first_sequence(
            self.actions,
            sequence,
            initial_action=parked_action,
            runtime_prefix="LITE_RETURN",
            phase_time_ms=BOARD_SAFE_RETURN_PHASE_TIME_MS,
        )

    def _prepare_stage7_above_sequence(
        self, sequence: SequenceDefinition
    ) -> SequenceDefinition:
        if not sequence.action_names:
            raise RuntimeError("Stage 7 ABOVE sequence has no target action")
        target_name = sequence.action_names[-1]
        direct_sequence = SequenceDefinition(
            name=sequence.name,
            display_name=sequence.display_name,
            steps=(ActionStep(target_name),),
            requires_board=sequence.requires_board,
        )
        return j1_last_sequence(
            self.actions,
            direct_sequence,
            initial_action=self.actions.get("OBSERVE_IDLE"),
            runtime_prefix="LITE_ABOVE",
        )

    def stage7_safe_return(self) -> None:
        try:
            if self._stage7_parked_above is None:
                raise RuntimeError("Stage 7 is not parked at an ABOVE point")
            self.stage7.clear_pending_jogs()
            row, col = self._stage7_parked_above
            parked_pwm = self._stage7_parked_pwm
            if parked_pwm is None:
                raise RuntimeError("Stage 7 parked PWM snapshot is missing")
            sequence = self._build_lite_stage7_return_sequence(
                row=row,
                col=col,
                parked_pwm=parked_pwm,
            )
            self._start_sequence(sequence, self.state_machine.begin_stage7_return)
            LOGGER.info(
                "[LITE][SAFE_RETURN_VIA_P77] from=%s actions=%s",
                point_id(row, col),
                sequence.action_names,
            )
        except Exception as exc:
            self._stage7_error("Stage 7 safe return blocked", exc)

    def start_quick_calibration(self) -> None:
        if not self.controller.is_connected:
            self._warn("快速标定", "请先连接 STM32。")
            return
        if self.camera_worker is None:
            self._warn("快速标定", "请先连接 Camera。")
            return
        if not self.board_locked:
            self._warn("快速标定", "尚未找到棋盘，请保持 Tag 15–18 可见。")
            return
        try:
            self._quick_special_flow = True
            self.stage7.new_session(CalibrationMode.QUICK_5)
            spec = self.wizard.start()
            self._show_anchor(spec)
            LOGGER.info("[LITE][WIZARD_START] session=%s", self.stage7.session.session_id)
        except Exception as exc:
            self._warn("无法开始快速标定", str(exc))

    def continue_calibration(self) -> None:
        summary = self._summary
        if summary is None or summary.session_path is None or not summary.session_path.exists():
            self._warn("继续使用", "没有可继续的 Calibration Lite 会话。")
            return
        try:
            session = self.stage7.load_session(summary.session_path)
            anchor_ids = set(session.anchors)
            missing_index = next(
                (
                    index
                    for index, spec in enumerate(ANCHOR_SPECS)
                    if point_id(spec.row, spec.col) not in anchor_ids
                ),
                None,
            )
            self.wizard.completed_anchor_labels = [
                spec.label
                for spec in ANCHOR_SPECS
                if point_id(spec.row, spec.col) in anchor_ids
            ]
            if missing_index is not None:
                self.wizard.phase = WizardPhase.ANCHOR
                self.wizard.anchor_index = missing_index
                self._show_anchor(ANCHOR_SPECS[missing_index])
            elif len(session.generated_points) == 225:
                self.wizard.begin_test()
                verified = set(session.verified_points)
                next_test = next(
                    (
                        index
                        for index, spec in enumerate(ANCHOR_SPECS)
                        if point_id(spec.row, spec.col) not in verified
                    ),
                    None,
                )
                if next_test is None:
                    self.wizard.phase = WizardPhase.COMPLETE
                    self.wizard.verified_labels = [spec.label for spec in ANCHOR_SPECS]
                    self._show_complete()
                else:
                    self.wizard.test_index = next_test
                    self.wizard.verified_labels = [
                        spec.label
                        for spec in ANCHOR_SPECS
                        if point_id(spec.row, spec.col) in verified
                    ]
                    self._show_test(ANCHOR_SPECS[next_test])
            else:
                self.wizard.phase = WizardPhase.GENERATE
                self.lite.show_generate()
            LOGGER.info("[LITE][SESSION_RESUMED] path=%s", summary.session_path)
        except Exception as exc:
            self._warn("继续会话失败", str(exc))

    def _show_anchor(self, spec: AnchorSpec) -> None:
        self._current_spec = spec
        self.stage7_select_point(spec.row, spec.col)
        pwm = self._authoritative_point_pwm(spec.row, spec.col)
        self.lite.set_pwm_values(pwm)
        self._last_sent_pwm = {joint: int(pwm[f"{joint:03d}"]) for joint in range(5)}
        step, total = self.wizard.anchor_step
        self.lite.show_anchor(
            label=spec.label,
            row=spec.row,
            col=spec.col,
            step=step,
            total=total,
            saved=len(self.wizard.completed_anchor_labels),
            correction=self.wizard.correction_target is not None,
        )
        self._refresh_lite_status()

    def _show_test(self, spec: AnchorSpec) -> None:
        self._current_spec = spec
        self.stage7_select_point(spec.row, spec.col)
        pwm = self._authoritative_point_pwm(spec.row, spec.col)
        self.lite.set_pwm_values(pwm)
        self._last_sent_pwm = {
            joint: int(pwm[f"{joint:03d}"]) for joint in range(5)
        }
        step, total = self.wizard.test_step
        self.lite.show_test(label=spec.label, step=step, total=total)
        self._refresh_lite_status()

    def _show_complete(self) -> None:
        session = self.stage7.require_session()
        self.lite.show_complete(
            anchors=len(session.anchors),
            points=len(session.generated_points),
            verified=len(self.wizard.verified_labels),
        )

    def _move_or_return(self) -> None:
        if self._is_estop_latched():
            self._warn("急停已锁存", "请先安全复位并点击“急停后恢复”。")
            return
        state = self.state_machine.snapshot().state
        if state != ArmState.OBSERVE_IDLE:
            if state == ArmState.HOVERING and self._current_spec is not None:
                expected = (self._current_spec.row, self._current_spec.col)
                if self._stage7_parked_above == expected:
                    return
            self.start_return_to_observe()
            return
        spec = self._current_spec
        if spec is None:
            return
        try:
            pwm = self._validated_lite_move_pwm(spec)
            hidden = self.control_panel.rapid_calibration_panel
            hidden.select_point(spec.row, spec.col, emit=False)
            hidden.set_pwm_values(pwm)
            self.stage7_move_above()
        except Exception as exc:
            self._warn(f"{spec.label} ABOVE 安全保护已阻止动作", str(exc))

    def _validated_lite_move_pwm(self, spec: AnchorSpec) -> dict[str, int]:
        expected = self._authoritative_point_pwm(spec.row, spec.col)
        if self.lite.pages.currentIndex() == self.lite.ANCHOR:
            proposed = self.lite.pwm_values()
            reference = {
                f"{joint:03d}": int(
                    self._last_sent_pwm.get(joint, expected[f"{joint:03d}"])
                )
                for joint in range(5)
            }
            excessive = {
                key: (int(reference[key]), int(proposed[key]))
                for key in reference
                if abs(int(proposed[key]) - int(reference[key]))
                > MAX_UNAPPLIED_POSE_DELTA
            }
            if excessive:
                LOGGER.warning(
                    "[LITE][ANCHOR_LARGE_MANUAL_CHANGE] point=%s changes=%s",
                    spec.label,
                    excessive,
                )
        else:
            proposed = dict(expected)
            self.lite.set_pwm_values(proposed)
        values = [int(proposed[f"{joint:03d}"]) for joint in range(5)]
        if len(set(values)) == 1:
            raise RuntimeError(
                f"检测到 J0–J4 异常相同目标 {values[0]}，疑似控件未初始化。"
            )
        rails = [
            f"J{joint}={values[joint]}"
            for joint in range(5)
            if values[joint] in {
                int(self.stage7.limits.joint_min[joint]),
                int(self.stage7.limits.joint_max[joint]),
            }
        ]
        if rails:
            raise RuntimeError("检测到关节位于软件极限，已禁止动作：" + ", ".join(rails))
        return {f"{joint:03d}": values[joint] for joint in range(5)}

    def apply_pwm_joint(self, joint: int, target: int) -> None:
        spec = self._current_spec
        if spec is None:
            return
        previous = int(self._last_sent_pwm.get(int(joint), target))
        self._last_pwm_adjustment = (int(joint), previous, int(target), True)
        self.control_panel.rapid_calibration_panel.select_point(
            spec.row, spec.col, emit=False
        )
        self.stage7_apply_joint(int(joint), int(target), 1000)

    def remember_pwm_target_change(self, joint: int, previous: int, target: int) -> None:
        self._last_pwm_adjustment = (int(joint), int(previous), int(target), False)

    def undo_last_adjustment(self) -> None:
        adjustment = self._last_pwm_adjustment
        if adjustment is None:
            self._warn("Undo", "当前没有可撤销的 PWM 调整。")
            return
        joint, previous, _target, was_applied = adjustment
        row = self.lite._pwm_rows[joint]
        row.target.setValue(previous)
        if was_applied:
            self.stage7_apply_joint(joint, previous, 1000)
        self._last_pwm_adjustment = None

    def confirm_current_anchor(self) -> None:
        spec = self._current_spec
        if spec is None:
            return
        if self._stage7_parked_above != (spec.row, spec.col):
            self._warn("尚未验证位置", "请先移动到当前点 ABOVE，再确认这个 Anchor。")
            return
        try:
            saved_pwm = self.lite.pwm_values()
            self.stage7.save_anchor(spec.row, spec.col, saved_pwm)
            self._stage7_parked_pwm = dict(saved_pwm)
            if (spec.row, spec.col) == (7, 7):
                p77_pwm = dict(saved_pwm)
                save_p77_point(self._p77_point_path, p77_pwm)
                self._p77_point_pwm = dict(p77_pwm)
                self._apply_p77_runtime_from_pwm(p77_pwm)
            self._pending_after_return = ("anchor_saved", spec)
            self.lite.anchor_instruction.setText("Anchor 已保存，正在沿安全路径返回观察位…")
            self.stage7_safe_return()
        except Exception as exc:
            self._warn("保存 Anchor 失败", str(exc))

    def _finish_anchor_after_return(self) -> None:
        next_spec = self.wizard.mark_anchor_saved()
        if next_spec is None:
            if (
                self._quick_special_flow
                and self.wizard.correction_target is None
                and len(self.wizard.completed_anchor_labels) >= 5
            ):
                self.show_observe_calibration()
            else:
                self.lite.show_generate()
                if self.wizard.correction_target is None and len(self.wizard.completed_anchor_labels) >= 5:
                    self.lite.generate_message.setText("下一步：保存并生成全盘。")
        else:
            self._show_anchor(next_spec)

    def generate_full_board(self, *, automatic: bool = False) -> None:
        try:
            generated = self.stage7.recalculate()
            valid = 0
            for record in generated.values():
                pwm = record.get("new_pwm") or {}
                if all(
                    self.stage7.limits.joint_min[joint]
                    <= int(pwm[f"{joint:03d}"])
                    <= self.stage7.limits.joint_max[joint]
                    for joint in range(5)
                ):
                    valid += 1
            session = self.stage7.require_session()
            self.lite.generate_result.setText(
                "Calibration Complete\n\n"
                f"{len(session.anchors)} anchors saved\n"
                f"{len(generated)} / 225 points generated\n"
                f"{valid} / 225 valid"
            )
            self.lite.begin_test_button.setEnabled(valid == 225)
            self._refresh_summary()
            if automatic:
                self._show_test(self.wizard.current_test)
        except Exception as exc:
            self._warn("生成全盘失败", str(exc))

    def begin_quick_test(self) -> None:
        try:
            session = self.stage7.require_session()
            if len(session.generated_points) != 225:
                raise RuntimeError("请先生成完整 225 点。")
            spec = self.wizard.begin_test()
            self._show_test(spec)
        except Exception as exc:
            self._warn("无法开始关键点测试", str(exc))

    def confirm_test_accurate(self) -> None:
        spec = self._current_spec
        if spec is None or self._stage7_parked_above != (spec.row, spec.col):
            self._warn("尚未测试", "请先移动到当前点 ABOVE。")
            return
        try:
            self.stage7.verify(spec.row, spec.col)
            self._pending_after_return = ("test_accurate", spec)
            self.lite.test_instruction.setText("已确认准确，正在安全返回观察位…")
            self.stage7_safe_return()
        except Exception as exc:
            self._warn("确认测试失败", str(exc))

    def correct_test_point(self) -> None:
        spec = self._current_spec
        if spec is None:
            return
        if self._stage7_parked_above == (spec.row, spec.col):
            self._pending_after_return = ("test_inaccurate", spec)
            self.lite.test_instruction.setText("正在安全返回，然后只修正当前点…")
            self.stage7_safe_return()
        else:
            self._show_anchor(self.wizard.correct_current_test())

    def add_anchor(self, row: int, col: int) -> None:
        try:
            self.stage7.require_session()
            spec = self.wizard.add_anchor(int(row), int(col))
            if self.state_machine.state == ArmState.HOVERING:
                self._pending_after_return = ("show_add_anchor", spec)
                self.stage7_safe_return()
            else:
                self._show_anchor(spec)
        except Exception as exc:
            self._warn("添加 Anchor 失败", str(exc))

    def show_observe_calibration(self) -> None:
        action = self.actions.get("OBSERVE_IDLE")
        pwm = action_pwm(action)
        self._observe_last_sent_pwm = {
            joint: int(pwm[f"{joint:03d}"]) for joint in range(5)
        }
        self._observe_last_adjustment = None
        self.lite.set_observe_quick_mode(self._quick_special_flow)
        self.lite.show_observe_calibration(pwm)

    def apply_observe_pwm_joint(self, joint: int, target: int) -> None:
        jid = int(joint)
        requested = int(target)
        try:
            if not self.controller.is_connected:
                raise RuntimeError("请先连接 STM32。")
            if self._is_estop_latched():
                raise RuntimeError("急停已锁存。")
            if self.arm_worker.busy or self.state_machine.busy:
                raise RuntimeError("机械臂运动中。")
            if self.state_machine.snapshot().state != ArmState.OBSERVE_IDLE:
                raise RuntimeError("请先点击“移动到当前观察位”。")
            lo = int(self.stage7.limits.joint_min[jid])
            hi = int(self.stage7.limits.joint_max[jid])
            applied = max(lo, min(hi, requested))
            previous = int(self._observe_last_sent_pwm.get(jid, applied))
            self.controller.send_joint_pwm(jid, applied, time_ms=1000)
            self._observe_last_adjustment = (jid, previous, applied, True)
            self._observe_last_sent_pwm[jid] = applied
            self.lite._observe_pwm_rows[jid].target.setValue(applied)
            self.lite.set_observe_pwm_current(jid, applied)
            if applied != requested:
                self.lite.home_message.setText(
                    f"J{jid} 已按安全范围限制：{requested} → {applied}"
                )
        except Exception as exc:
            self._warn("观察位 PWM", str(exc))

    def remember_observe_pwm_change(
        self, joint: int, previous: int, target: int
    ) -> None:
        self._observe_last_adjustment = (
            int(joint), int(previous), int(target), False
        )

    def undo_observe_adjustment(self) -> None:
        adjustment = self._observe_last_adjustment
        if adjustment is None:
            self._warn("Undo", "当前没有可撤销的观察位调整。")
            return
        joint, previous, _target, was_applied = adjustment
        self.lite._observe_pwm_rows[joint].target.setValue(previous)
        if was_applied:
            try:
                if self.state_machine.snapshot().state != ArmState.OBSERVE_IDLE:
                    raise RuntimeError("机械臂不在观察位，不能执行 Undo。")
                self.controller.send_joint_pwm(joint, previous, time_ms=1000)
                self._observe_last_sent_pwm[joint] = previous
                self.lite.set_observe_pwm_current(joint, previous)
            except Exception as exc:
                self._warn("Undo", str(exc))
                return
        self._observe_last_adjustment = None

    def save_observe_calibration(self) -> None:
        try:
            if self.state_machine.snapshot().state != ArmState.OBSERVE_IDLE:
                raise RuntimeError("请先移动并确认机械臂位于观察位。")
            path = save_observe_override(
                self._observe_override_path,
                library=self.actions,
                stable_actions=self._stable_observe_actions,
                observe_idle_pwm=self.lite.observe_pwm_values(),
            )
            QMessageBox.information(
                self,
                "观察位已保存",
                f"观察位已同时保存为 PICK_ABOVE 和取料后悬停位。\n{path}",
            )
            LOGGER.info("[LITE][OBSERVE_SAVED] path=%s", path)
            if self._quick_special_flow:
                self.show_pick_calibration()
        except Exception as exc:
            self._warn("保存观察位失败", str(exc))

    def show_pick_calibration(self) -> None:
        record = self.stage7.pick_poses.get("PICK_DOWN")
        pwm = record["new_pwm"]
        self._pick_last_sent_pwm = {
            joint: int(pwm[f"{joint:03d}"]) for joint in range(5)
        }
        self._pick_last_adjustment = None
        self._pick_calibration_parked = False
        self.lite.show_pick_calibration(pwm, quick_mode=self._quick_special_flow)

    def move_to_pick_calibration(self) -> None:
        try:
            if not self.controller.is_connected:
                raise RuntimeError("请先连接 STM32。")
            if self._is_estop_latched() or self.arm_worker.busy or self.state_machine.busy:
                raise RuntimeError("机械臂尚未准备好。")
            if self.state_machine.snapshot().state != ArmState.OBSERVE_IDLE:
                raise RuntimeError("取料位标定必须从观察位开始。")
            down_pwm = action_pwm(self._stable_pick_action)
            down_pwm.update(self.lite.pick_calibration_pwm_values())
            down_pwm["005"] = 1500
            self.actions.register_runtime(
                build_action(
                    "LITE_PICK_DOWN_IDLE",
                    down_pwm,
                    action_times(self._stable_pick_action),
                )
            )
            sequence = SequenceDefinition(
                name="MANUAL:LITE_PICK_CALIBRATE",
                display_name="移动到取料接触标定位（气泵关闭）",
                steps=(ActionStep("LITE_PICK_DOWN_IDLE"),),
            )
            sequence = self._prepare_pick_calibration_sequence(sequence)
            self._start_sequence(
                sequence,
                lambda: self.state_machine.begin_manual("LITE_PICK_CALIBRATE"),
            )
            self._pick_calibration_parked = False
            LOGGER.info("[LITE][PICK_CALIBRATION_MOVE]")
        except Exception as exc:
            self._warn("移动到取料位失败", str(exc))

    def _prepare_pick_calibration_sequence(
        self, sequence: SequenceDefinition
    ) -> SequenceDefinition:
        return j1_last_sequence(
            self.actions,
            sequence,
            initial_action=self.actions.get("OBSERVE_IDLE"),
            runtime_prefix="LITE_PICK_DOWN",
        )

    def _return_from_pick_calibration(self) -> None:
        current_pwm = action_pwm(self._stable_pick_action)
        current_pwm.update(self.lite.pick_calibration_pwm_values())
        current_pwm["005"] = 1500
        observe = self.actions.get("OBSERVE_IDLE")
        if int(action_pwm(observe)["005"]) != 1500:
            raise RuntimeError("OBSERVE_IDLE must keep J5 pump OFF at PWM 1500.")
        current_action = build_action(
            "LITE_PICK_CALIBRATION_RETURN_START",
            current_pwm,
            {
                f"{joint:03d}": PICK_CALIBRATION_RETURN_PHASE_TIME_MS
                for joint in range(8)
            },
        )
        sequence = j1_first_sequence(
            self.actions,
            SequenceDefinition(
                name="RETURN_TO_OBSERVE",
                display_name="Pick calibration return to Observation (J1 first)",
                steps=(ActionStep("OBSERVE_IDLE"),),
            ),
            initial_action=current_action,
            runtime_prefix="LITE_PICK_CALIBRATION_RETURN",
            phase_time_ms=PICK_CALIBRATION_RETURN_PHASE_TIME_MS,
        )
        self._start_sequence(sequence, self.state_machine.begin_return_to_observe)
        LOGGER.info(
            "[LITE][PICK_CALIBRATION_RETURN] actions=%s phase_time_ms=%d pump_pwm=1500",
            sequence.action_names,
            PICK_CALIBRATION_RETURN_PHASE_TIME_MS,
        )

    def apply_pick_calibration_pwm_joint(self, joint: int, target: int) -> None:
        jid = int(joint)
        requested = int(target)
        try:
            if not self.controller.is_connected:
                raise RuntimeError("请先连接 STM32。")
            if not self._pick_calibration_parked:
                raise RuntimeError("请先移动到当前取料接触位。")
            if self._is_estop_latched() or self.arm_worker.busy:
                raise RuntimeError("机械臂尚未准备好。")
            lo = int(self.stage7.limits.joint_min[jid])
            hi = int(self.stage7.limits.joint_max[jid])
            applied = max(lo, min(hi, requested))
            previous = int(self._pick_last_sent_pwm.get(jid, applied))
            self.controller.send_joint_pwm(jid, applied, time_ms=1000)
            self._pick_last_adjustment = (jid, previous, applied, True)
            self._pick_last_sent_pwm[jid] = applied
            self.lite._pick_pwm_rows[jid].target.setValue(applied)
            self.lite.set_pick_calibration_pwm_current(jid, applied)
        except Exception as exc:
            self._warn("取料位 PWM", str(exc))

    def remember_pick_calibration_pwm_change(
        self, joint: int, previous: int, target: int
    ) -> None:
        self._pick_last_adjustment = (
            int(joint), int(previous), int(target), False
        )

    def undo_pick_calibration_adjustment(self) -> None:
        adjustment = self._pick_last_adjustment
        if adjustment is None:
            self._warn("Undo", "当前没有可撤销的取料位调整。")
            return
        joint, previous, _target, was_applied = adjustment
        self.lite._pick_pwm_rows[joint].target.setValue(previous)
        if was_applied:
            try:
                if not self._pick_calibration_parked:
                    raise RuntimeError("机械臂不在取料接触标定位。")
                self.controller.send_joint_pwm(joint, previous, time_ms=1000)
                self._pick_last_sent_pwm[joint] = previous
                self.lite.set_pick_calibration_pwm_current(joint, previous)
            except Exception as exc:
                self._warn("Undo", str(exc))
                return
        self._pick_last_adjustment = None

    def save_pick_calibration(self) -> None:
        try:
            if not self._pick_calibration_parked:
                raise RuntimeError("请先移动并确认取料接触位。")
            session_id = None if self.stage7.session is None else self.stage7.session.session_id
            record = self.stage7.pick_poses.save_candidate(
                "PICK_DOWN",
                self.lite.pick_calibration_pwm_values(),
                calibration_session=session_id,
            )
            self._apply_pick_runtime(record["new_pwm"])
            self._pending_after_return = ("pick_calibration_saved", self._quick_special_flow)
            self._return_from_pick_calibration()
            LOGGER.info(
                "[LITE][PICK_CALIBRATION_SAVED] path=%s delta=%s",
                self.stage7.settings.pick_pose_path,
                record["delta_pwm"],
            )
        except Exception as exc:
            self._warn("保存取料位失败", str(exc))

    def _refresh_manual_movel_panel(self) -> None:
        if self.manual_movel_store is None or not hasattr(self, "lite"):
            return
        index = max(
            0,
            min(self._manual_movel_view_index, self.manual_movel_store.step_count() - 1),
        )
        self._manual_movel_view_index = index
        drop = self.manual_movel_store.drop_candidate()
        self.lite.manual_movel_panel.set_step(
            self.manual_movel_store.step(index),
            drop_step=None if drop is None else int(drop["step_index"]),
        )

    def show_manual_movel(self) -> None:
        try:
            self._require_manual_movel()
            self._refresh_manual_movel_panel()
            self.lite.show_manual_movel()
        except Exception as exc:
            self._warn("P77 Manual MoveL", str(exc))

    def _submit_manual_movel_sequence(self, sequence: SequenceDefinition) -> None:
        if not self.controller.is_connected:
            raise RuntimeError("Connect STM32 (or the Dry Run controller) first.")
        if self._is_estop_latched() or self.arm_worker.busy or self.state_machine.busy:
            raise RuntimeError("Robot is busy or Emergency Stop is latched.")
        if not self.dry_run:
            answer = QMessageBox.warning(
                self,
                "Live P77 Manual MoveL confirmation",
                f"Execute on the real arm?\n\n{sequence.display_name}\n\n"
                "Check clearance and keep Emergency Stop ready.",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                QMessageBox.StandardButton.No,
            )
            if answer != QMessageBox.StandardButton.Yes:
                raise RuntimeError("Live motion cancelled by operator.")
        self._start_sequence(
            sequence,
            lambda: self.state_machine.begin_manual(sequence.name),
        )

    def move_manual_movel_step(self, index: int, final_pwm: object) -> None:
        try:
            store, builder = self._require_manual_movel()
            step_index = int(index)
            if not isinstance(final_pwm, dict):
                raise RuntimeError("Invalid absolute PWM payload")
            if step_index == 0:
                if self._manual_movel_pose_index not in {None, 0}:
                    raise RuntimeError("Return through saved manual steps to ABOVE first.")
                if (
                    not self.dry_run
                    and self._manual_movel_pose_index is None
                    and self.state_machine.snapshot().state != ArmState.OBSERVE_IDLE
                ):
                    raise RuntimeError("Live Step0 entry must start from OBSERVE_IDLE.")
                if self._manual_movel_pose_index == 0:
                    raise RuntimeError("Already at immutable P77 ABOVE Step0.")
                observation = action_pwm(self.actions.get("OBSERVE_IDLE"))
                sequence = builder.build_enter_above(observation)
            else:
                if self._manual_movel_pose_index not in {step_index - 1, step_index}:
                    raise RuntimeError(
                        "Move Current Step only permits the current or next adjacent manual step."
                    )
                store.validate_candidate(step_index, final_pwm)
                sequence = builder.build_move_candidate(step_index, final_pwm)
            self._submit_manual_movel_sequence(sequence)
        except Exception as exc:
            self._warn("Move Current Step", str(exc))

    def save_manual_movel_step(self, index: int, final_pwm: object) -> None:
        try:
            store, _builder = self._require_manual_movel()
            if not isinstance(final_pwm, dict):
                raise RuntimeError("Invalid absolute PWM payload")
            store.save_step(int(index), final_pwm)
            store.save()
            self._refresh_manual_movel_panel()
        except Exception as exc:
            self._warn("Save Current Step", str(exc))

    def confirm_manual_movel_step(self, index: int) -> None:
        try:
            store, _builder = self._require_manual_movel()
            step_index = int(index)
            if self._manual_movel_pose_index != step_index:
                raise RuntimeError("Move to this saved step before Confirm Step.")
            if (
                store.step(step_index)["final_pwm"]
                != self.lite.manual_movel_panel.final_pwm()
            ):
                raise RuntimeError("Save the displayed absolute PWM before Confirm Step.")
            store.confirm_step(step_index)
            store.save()
            self._refresh_manual_movel_panel()
        except Exception as exc:
            self._warn("Confirm Step", str(exc))

    def next_manual_movel_step(self, index: int) -> None:
        try:
            store, _builder = self._require_manual_movel()
            record = store.create_next_step(int(index))
            self._manual_movel_view_index = int(record["step_index"])
            self._refresh_manual_movel_panel()
        except Exception as exc:
            self._warn("Next Step", str(exc))

    def previous_manual_movel_step(self, index: int) -> None:
        try:
            self._require_manual_movel()
            self._manual_movel_view_index = max(0, int(index) - 1)
            self._refresh_manual_movel_panel()
        except Exception as exc:
            self._warn("Previous Step", str(exc))

    def return_previous_manual_movel_step(self) -> None:
        try:
            _store, builder = self._require_manual_movel()
            if self._manual_movel_pose_index is None:
                raise RuntimeError("Manual MoveL pose is unknown or not entered.")
            self._submit_manual_movel_sequence(
                builder.build_return_previous(self._manual_movel_pose_index)
            )
        except Exception as exc:
            self._warn("Return Previous Step", str(exc))

    def return_manual_movel_above(self) -> None:
        try:
            _store, builder = self._require_manual_movel()
            if self._manual_movel_pose_index is None:
                raise RuntimeError("Manual MoveL pose is unknown or not entered.")
            self._submit_manual_movel_sequence(
                builder.build_return_above(self._manual_movel_pose_index)
            )
        except Exception as exc:
            self._warn("Return ABOVE", str(exc))

    def set_manual_movel_drop(self, index: int) -> None:
        try:
            store, _builder = self._require_manual_movel()
            drop_store, planner, _drop_builder = self._require_drop_v1()
            step_index = int(index)
            if self._manual_movel_pose_index != step_index:
                raise RuntimeError("Move to the confirmed step before Set As P77 DROP.")
            store.set_as_drop(step_index)
            store.save()
            record = drop_store.drop_record("P77")
            if record is None or record.get("drop_auto_pwm") is None:
                planner.generate_point("P77", persist=False)
            drop_store.apply_p77_manual_movel_calibration(
                store.calibration_payload()
            )
            drop_store.save()
            store.mark_movel_applied()
            store.save()
            self._refresh_manual_movel_panel()
            self.lite.manual_movel_panel.status_label.setText(
                f"P77 DROP=Step{step_index} · applied to MoveL · "
                "hardware_verified=false"
            )
        except Exception as exc:
            self._warn("Set As P77 DROP", str(exc))

    def run_manual_movel_full_cycle(self) -> None:
        try:
            store, builder = self._require_manual_movel()
            drop = store.drop_candidate()
            if drop is None:
                raise RuntimeError(
                    "P77 FINAL DROP is not saved. Calibrate and Set As FINAL DROP first."
                )

            # Validate that the saved manual calibration is complete and confirmed.
            store.calibration_payload()
            if self._manual_movel_pose_index not in {None, 0}:
                raise RuntimeError(
                    "Return through saved steps to P77 ABOVE before the full flow."
                )
            start_from_above = self._manual_movel_pose_index == 0
            if (
                not start_from_above
                and self.state_machine.snapshot().state != ArmState.OBSERVE_IDLE
            ):
                raise RuntimeError("P77 full flow must start from OBSERVE_IDLE.")
            if not self.board_locked:
                raise RuntimeError("P77 full flow requires BOARD LOCKED.")
            self._submit_manual_movel_sequence(
                builder.build_full_pick_place(start_from_above=start_from_above)
            )
        except Exception as exc:
            self._warn("P77 Pick & Place Full Flow", str(exc))

    def back_from_manual_movel(self) -> None:
        try:
            _store, builder = self._require_manual_movel()
            if self.arm_worker.busy or self.state_machine.busy:
                raise RuntimeError("Robot is busy; wait before leaving this page.")
            if self._manual_movel_pose_index is None:
                self.lite.show_home()
                return
            if self._manual_movel_pose_index > 0:
                raise RuntimeError(
                    "Return ABOVE through the saved manual steps before leaving."
                )
            self._manual_movel_back_pending = True
            self._submit_manual_movel_sequence(builder.build_return_observation())
        except Exception as exc:
            self._manual_movel_back_pending = False
            self._warn("Back to Home", str(exc))

    def show_drop_v1(self, *_args, row: int = 7, col: int = 7) -> None:
        try:
            self._require_drop_v1()
            point = parse_point_id((int(row), int(col)))
            if self._drop_pose_state != "SAFE" and self._drop_active_point:
                point = parse_point_id(self._drop_active_point)
            self.lite.show_drop_v1(row=point.row, col=point.col)
            self._refresh_drop_v1_panel(point.point_id)
        except Exception as exc:
            self._warn("DROP / PLACE V1", str(exc))

    def show_place_calibration(self) -> None:
        """Compatibility route: the old fixed-PWM page is no longer exposed."""
        self.show_drop_v1()

    def select_drop_v1_point(self, row: int, col: int) -> None:
        try:
            point = parse_point_id((row, col))
            if self._drop_pose_state != "SAFE":
                raise RuntimeError("Retract and Safe Return before loading a point.")
            self.lite.drop_v1_panel.set_point(point.row, point.col)
            self._reset_drop_v1_workflow(point.point_id, loaded=True)
            self._refresh_drop_v1_panel(point.point_id)
        except Exception as exc:
            self._warn("Select board point", str(exc))

    def _refresh_drop_v1_panel(self, point_id_value: str | None = None) -> None:
        if self.drop_store is None:
            return
        point = parse_point_id(point_id_value or self.lite.drop_v1_panel.point_id)
        panel = self.lite.drop_v1_panel
        panel.set_point(point.row, point.col)
        drop = self.drop_store.drop_record(point)
        panel.set_record(self.drop_store.above_record(point), drop)
        panel.set_statistics(self.drop_store.statistics())
        panel.set_workflow_state(
            step=self._drop_wizard_step,
            completed_steps=self._drop_completed_steps,
            message=self._drop_workflow_message,
            waypoint_index=self._drop_waypoint_index,
            waypoint_count=len((drop or {}).get("waypoints") or []),
            test_place_succeeded=self._drop_test_place_succeeded,
        )

    def _reset_drop_v1_workflow(self, point_id_value: str, *, loaded: bool) -> None:
        point = parse_point_id(point_id_value)
        self._drop_active_point = None
        self._drop_pose_state = "SAFE"
        self._drop_wizard_step = 1
        self._drop_completed_steps.clear()
        self._drop_above_confirmed = False
        self._drop_accuracy_confirmed = False
        self._drop_test_place_succeeded = False
        self._drop_waypoint_index = 0
        self._drop_at_final_correction = False
        self._drop_after_retract_step = None
        self._drop_after_safe_return_step = None
        self._drop_last_applied.clear()
        if not loaded:
            self._drop_workflow_message = "选择棋盘点并载入已保存 ABOVE。"
            return
        record = None if self.drop_store is None else self.drop_store.drop_record(point)
        blocked = record is None or record.get("status") in {
            DropStatus.NOT_GENERATED.value,
            DropStatus.MOVE_L_UNREACHABLE.value,
            DropStatus.INVALID.value,
        }
        if blocked:
            self._drop_workflow_message = (
                f"{point.point_id} ABOVE 已载入，但 DROP candidate 不可执行。"
                "请展开 Advanced 查看原因或离线重新生成。"
            )
            return
        self._drop_completed_steps.add(1)
        self._drop_wizard_step = 2
        self._drop_workflow_message = (
            f"{point.point_id} 已载入。连接控制器后点击 1. Move ABOVE，"
            "到位后确认 ABOVE 是否正确。"
        )

    def generate_drop_v1(self, point_id_value: str) -> None:
        try:
            store, planner, _builder = self._require_drop_v1()
            if self._drop_pose_state != "SAFE":
                raise RuntimeError("Generate only while the arm is in a safe/returned state.")
            record = planner.generate_point(point_id_value, persist=True)
            self._drop_live_verification_eligible.discard(parse_point_id(point_id_value).point_id)
            self._reset_drop_v1_workflow(point_id_value, loaded=True)
            self._refresh_drop_v1_panel(point_id_value)
            self.lite.drop_v1_panel.set_preview(planner.preview(point_id_value))
            if record.get("status") in {
                DropStatus.MOVE_L_UNREACHABLE.value,
                DropStatus.INVALID.value,
            }:
                self._warn(
                    "DROP candidate blocked",
                    f"{record.get('status')}: {record.get('reason') or '-'}",
                )
            LOGGER.info(
                "[LITE_DROP_V1][GENERATE] point=%s status=%s execution=OFFLINE_ONLY",
                point_id_value,
                record.get("status"),
            )
        except Exception as exc:
            self._warn("Generate DROP", str(exc))

    def generate_all_drop_v1(self) -> None:
        try:
            store, planner, _builder = self._require_drop_v1()
            if self._drop_pose_state != "SAFE" or self.arm_worker.busy:
                raise RuntimeError("Return the arm before offline batch generation.")
            summary = planner.generate_all(persist=True)
            self._drop_live_verification_eligible.clear()
            self._refresh_drop_v1_panel()
            values = summary.to_dict()
            QMessageBox.information(
                self,
                "225 DROP candidates — OFFLINE ONLY",
                "No controller or serial command was used.\n\n"
                + "\n".join(f"{key}: {value}" for key, value in values.items()),
            )
            LOGGER.info("[LITE_DROP_V1][GENERATE_ALL] %s", values)
        except Exception as exc:
            self._warn("Generate 225 OFFLINE ONLY", str(exc))

    def preview_drop_v1(self, point_id_value: str) -> None:
        try:
            _store, planner, _builder = self._require_drop_v1()
            self.lite.drop_v1_panel.set_preview(planner.preview(point_id_value))
        except Exception as exc:
            self._warn("Preview MoveL", str(exc))

    def _submit_drop_v1_sequence(self, sequence: SequenceDefinition) -> None:
        if not self.controller.is_connected:
            raise RuntimeError("Connect STM32 (or the Dry Run controller) first.")
        if self._is_estop_latched() or self.arm_worker.busy or self.state_machine.busy:
            raise RuntimeError("Robot is busy or Emergency Stop is latched.")
        if not self.dry_run:
            answer = QMessageBox.warning(
                self,
                "Live hardware confirmation",
                f"Execute on the real arm?\n\n{sequence.display_name}\n\n"
                "Check the workspace and keep Emergency Stop ready.",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                QMessageBox.StandardButton.No,
            )
            if answer != QMessageBox.StandardButton.Yes:
                raise RuntimeError("Live motion cancelled by operator.")
        self._start_sequence(
            sequence,
            lambda: self.state_machine.begin_manual(sequence.name),
        )

    def _require_drop_v1_safe_start(self) -> None:
        if self._drop_pose_state != "SAFE":
            raise RuntimeError("Safe Return is required before starting this sequence.")
        if not self.dry_run and self.state_machine.snapshot().state != ArmState.OBSERVE_IDLE:
            raise RuntimeError(
                "Live Lite motion must start from the known OBSERVE_IDLE pose. "
                "Use Return to Observe first."
            )

    def move_drop_v1_above(self, point_id_value: str) -> None:
        try:
            _store, _planner, builder = self._require_drop_v1()
            self._require_drop_v1_safe_start()
            self._submit_drop_v1_sequence(builder.build_move_above(point_id_value))
        except Exception as exc:
            self._warn("Move ABOVE", str(exc))

    def confirm_drop_v1_above(self, point_id_value: str) -> None:
        try:
            point = parse_point_id(point_id_value)
            if self._drop_wizard_step != 2:
                raise RuntimeError("Complete Step 1 before confirming ABOVE.")
            if self._drop_pose_state != "ABOVE" or self._drop_active_point != point.point_id:
                raise RuntimeError("Move to this point's ABOVE before confirming it.")
            self._drop_above_confirmed = True
            self._drop_completed_steps.update({1, 2})
            self._drop_wizard_step = 3
            self._drop_waypoint_index = 0
            self._drop_workflow_message = (
                "ABOVE 已确认。推荐点击 Next Waypoint 逐层下降；"
                "也可从 ABOVE 点击 2. Full DROP。"
            )
            self._refresh_drop_v1_panel(point.point_id)
        except Exception as exc:
            self._warn("Confirm ABOVE", str(exc))

    def reject_drop_v1_above(self, point_id_value: str) -> None:
        try:
            point = parse_point_id(point_id_value)
            if self._drop_pose_state != "ABOVE" or self._drop_active_point != point.point_id:
                raise RuntimeError("The arm is not parked at this point's ABOVE.")
            self._drop_after_safe_return_step = 1
            self._drop_workflow_message = (
                "ABOVE 未通过确认，正在 Safe Return。返回后请检查保存数据。"
            )
            self.safe_return_drop_v1(point.point_id)
        except Exception as exc:
            self._warn("Reject ABOVE", str(exc))

    def next_drop_v1_waypoint(self, point_id_value: str) -> None:
        self._move_drop_v1_waypoint(point_id_value, direction=1)

    def previous_drop_v1_waypoint(self, point_id_value: str) -> None:
        self._move_drop_v1_waypoint(point_id_value, direction=-1)

    def _move_drop_v1_waypoint(self, point_id_value: str, *, direction: int) -> None:
        try:
            store, _planner, builder = self._require_drop_v1()
            point = parse_point_id(point_id_value)
            if self._drop_wizard_step != 3 or not self._drop_above_confirmed:
                raise RuntimeError("Confirm ABOVE before waypoint testing.")
            if self._drop_active_point != point.point_id or self._drop_pose_state not in {
                "ABOVE",
                "WAYPOINT",
                "DROP",
            }:
                raise RuntimeError("Move to this point's confirmed ABOVE first.")
            record = store.drop_record(point) or {}
            count = len(record.get("waypoints") or [])
            target = self._drop_waypoint_index + (1 if int(direction) > 0 else -1)
            if not 0 <= target < count:
                raise RuntimeError(f"Waypoint target {target} is outside WP00..WP{count - 1:02d}.")
            sequence = builder.build_move_waypoint(
                point,
                target,
                from_final_correction=bool(
                    int(direction) < 0 and self._drop_at_final_correction
                ),
            )
            self._submit_drop_v1_sequence(sequence)
        except Exception as exc:
            self._warn("Move waypoint", str(exc))

    def move_drop_v1(self, point_id_value: str) -> None:
        try:
            store, _planner, builder = self._require_drop_v1()
            point = parse_point_id(point_id_value)
            if self._drop_wizard_step != 3 or not self._drop_above_confirmed:
                raise RuntimeError("Confirm ABOVE before Full DROP.")
            if self._drop_pose_state != "ABOVE" or self._drop_active_point != point.point_id:
                raise RuntimeError("Move to this point's saved ABOVE first.")
            record = store.drop_record(point)
            if record is None:
                raise RuntimeError("Generate and preview the DROP candidate first.")
            self._submit_drop_v1_sequence(builder.build_move_drop(point))
        except Exception as exc:
            self._warn("Move DROP", str(exc))

    def retract_drop_v1(self, point_id_value: str) -> None:
        try:
            _store, _planner, builder = self._require_drop_v1()
            point = parse_point_id(point_id_value)
            if self._drop_active_point != point.point_id or self._drop_pose_state not in {
                "WAYPOINT",
                "DROP",
            }:
                raise RuntimeError("Return to ABOVE is available only on this MoveL path.")
            if self._drop_pose_state == "DROP" and self._drop_at_final_correction:
                sequence = builder.build_retract(point)
            else:
                sequence = builder.build_retract_from_waypoint(
                    point, self._drop_waypoint_index
                )
            self._submit_drop_v1_sequence(sequence)
        except Exception as exc:
            self._warn("Retract", str(exc))

    def confirm_drop_v1_accuracy(self, point_id_value: str) -> None:
        self._finish_drop_v1_accuracy(point_id_value, from_correction=False)

    def mark_drop_v1_workflow_verified(self, point_id_value: str) -> None:
        self._finish_drop_v1_accuracy(point_id_value, from_correction=True)

    def _finish_drop_v1_accuracy(
        self, point_id_value: str, *, from_correction: bool
    ) -> None:
        try:
            point = parse_point_id(point_id_value)
            expected_step = 4 if from_correction else 3
            if self._drop_wizard_step != expected_step:
                raise RuntimeError(f"Complete Step {expected_step} before marking DROP.")
            if self._drop_pose_state != "DROP" or self._drop_active_point != point.point_id:
                raise RuntimeError("Reach this point's final DROP before confirming accuracy.")
            self._drop_accuracy_confirmed = True
            self._drop_completed_steps.update({1, 2, 3})
            if from_correction:
                self._drop_completed_steps.add(4)
            self._drop_after_retract_step = 5
            self._drop_workflow_message = (
                "DROP 精度已在本次向导中确认。正在垂直回撤到 ABOVE；"
                "随后 Safe Return 并执行 Test PLACE。"
            )
            self.retract_drop_v1(point.point_id)
        except Exception as exc:
            self._warn("Mark DROP Verified", str(exc))

    def enter_drop_v1_correction(self, point_id_value: str) -> None:
        try:
            point = parse_point_id(point_id_value)
            if self._drop_wizard_step not in {3, 4}:
                raise RuntimeError("Correction is available after testing final DROP.")
            if self._drop_pose_state != "DROP" or self._drop_active_point != point.point_id:
                raise RuntimeError("Reach final DROP before opening PWM correction.")
            self._drop_wizard_step = 4
            self._drop_workflow_message = (
                "PWM correction 已展开。逐个 Apply J0–J4，保存 correction；"
                "然后 Mark DROP Verified 或返回 Step 3 复测。"
            )
            self._refresh_drop_v1_panel(point.point_id)
        except Exception as exc:
            self._warn("PWM correction", str(exc))

    def return_to_retest_drop_v1(self, point_id_value: str) -> None:
        try:
            point = parse_point_id(point_id_value)
            if self._drop_wizard_step != 4:
                raise RuntimeError("Return to re-test is available in Step 4.")
            self._drop_accuracy_confirmed = False
            self._drop_test_place_succeeded = False
            self._drop_after_retract_step = 3
            self._drop_workflow_message = (
                "正在垂直回撤到 ABOVE；完成后重新测试修正后的 DROP。"
            )
            self.retract_drop_v1(point.point_id)
        except Exception as exc:
            self._warn("Return to re-test DROP", str(exc))

    def test_place_drop_v1(self, point_id_value: str) -> None:
        try:
            _store, _planner, builder = self._require_drop_v1()
            if self._drop_wizard_step != 5 or not self._drop_accuracy_confirmed:
                raise RuntimeError("Confirm DROP accuracy before Test PLACE.")
            self._require_drop_v1_safe_start()
            self._submit_drop_v1_sequence(builder.build_test_place(point_id_value))
        except Exception as exc:
            self._warn("Test PLACE", str(exc))

    def apply_drop_v1_joint(self, point_id_value: str, joint: int, correction: int) -> None:
        try:
            _store, _planner, builder = self._require_drop_v1()
            point = parse_point_id(point_id_value)
            if self._drop_pose_state != "DROP" or self._drop_active_point != point.point_id:
                raise RuntimeError("Move DROP before applying J0–J4 correction.")
            if not self.controller.is_connected:
                raise RuntimeError("Connect STM32 (or the Dry Run controller) first.")
            if self._is_estop_latched() or self.arm_worker.busy:
                raise RuntimeError("Robot is busy or Emergency Stop is latched.")
            target = builder.apply_joint_target(point, joint, correction)
            self.controller.send_joint_pwm(int(joint), target, time_ms=builder.move_time_ms)
            self._drop_last_applied[int(joint)] = target
            correction_values = self.lite.drop_v1_panel.correction_values()
            self._drop_at_final_correction = any(
                int(value) != 0 for value in correction_values.values()
            )
            LOGGER.info(
                "[LITE_DROP_V1][JOG] point=%s joint=%s target=%s dry_run=%s",
                point.point_id,
                joint,
                target,
                int(self.dry_run),
            )
        except Exception as exc:
            self._warn("DROP PWM correction", str(exc))

    def save_drop_v1_correction(self, point_id_value: str, correction: object) -> None:
        try:
            store, _planner, builder = self._require_drop_v1()
            point = parse_point_id(point_id_value)
            if self._drop_pose_state != "DROP" or self._drop_active_point != point.point_id:
                raise RuntimeError("Move DROP and apply corrections before saving.")
            if not self.controller.is_connected:
                raise RuntimeError("Connect STM32 (or the Dry Run controller) first.")
            if self._is_estop_latched() or self.arm_worker.busy or self.state_machine.busy:
                raise RuntimeError("Robot is busy or Emergency Stop is latched.")
            if not isinstance(correction, dict):
                raise RuntimeError("Invalid correction payload.")
            expected = {
                joint: builder.apply_joint_target(point, joint, int(correction[key]))
                for joint, key in enumerate(SPATIAL_KEYS)
            }
            missing = [
                f"J{joint}"
                for joint, value in expected.items()
                if self._drop_last_applied.get(joint) != value
            ]
            if missing:
                raise RuntimeError(
                    "Apply every displayed correction target before saving: "
                    + ", ".join(missing)
                )
            saved = store.save_correction(point, correction)
            store.save()
            self._drop_at_final_correction = (
                saved.get("drop_final_pwm") != saved.get("drop_auto_pwm")
            )
            self._drop_accuracy_confirmed = False
            self._drop_test_place_succeeded = False
            self._drop_wizard_step = 4
            self._drop_workflow_message = (
                "Correction 已独立保存，并已清除旧验证。"
                "可 Mark DROP Verified，或 Return to re-test DROP。"
            )
            self._drop_live_verification_eligible.discard(point.point_id)
            self._refresh_drop_v1_panel(point.point_id)
        except Exception as exc:
            self._warn("Save DROP correction", str(exc))

    def safe_return_drop_v1(self, point_id_value: str) -> None:
        try:
            _store, _planner, builder = self._require_drop_v1()
            point = parse_point_id(point_id_value)
            if self._drop_active_point != point.point_id or self._drop_pose_state not in {
                "ABOVE",
                "DROP",
            }:
                raise RuntimeError("Safe Return requires this point's ABOVE or DROP pose.")
            if (
                self._drop_after_safe_return_step is None
                and not (
                    self._drop_wizard_step == 5
                    and self._drop_accuracy_confirmed
                )
            ):
                # Returning to the global safe pose invalidates the local ABOVE
                # confirmation.  Resume at Step 2 so the operator always has a
                # visible, safe next action instead of being stranded in Step 3/4.
                self._drop_after_safe_return_step = 2
            sequence = (
                builder.build_safe_return_from_drop(point)
                if self._drop_pose_state == "DROP"
                else builder.build_safe_return_from_above(point)
            )
            self._submit_drop_v1_sequence(sequence)
        except Exception as exc:
            self._warn("Safe Return", str(exc))

    def verify_drop_v1(self, point_id_value: str) -> None:
        try:
            store, _planner, _builder = self._require_drop_v1()
            point = parse_point_id(point_id_value)
            if not self.controller.is_connected:
                raise RuntimeError("Connect STM32 (or the Dry Run controller) first.")
            if self._drop_wizard_step != 5 or not self._drop_test_place_succeeded:
                raise RuntimeError(
                    "Confirm verification only after Step 5 Test PLACE succeeds."
                )
            if not self.dry_run and point.point_id not in self._drop_live_verification_eligible:
                raise RuntimeError(
                    "Complete a real Move DROP + Retract or Test PLACE before hardware verification."
                )
            if not self.dry_run:
                answer = QMessageBox.question(
                    self,
                    "Confirm real DROP verification",
                    f"Did you physically observe {point.point_id} place and vertical retract correctly?",
                    QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                    QMessageBox.StandardButton.No,
                )
                if answer != QMessageBox.StandardButton.Yes:
                    raise RuntimeError("Hardware verification was not confirmed.")
            store.mark_verified(point, hardware_confirmed=not self.dry_run)
            store.save()
            self._drop_completed_steps.update({1, 2, 3, 4, 5})
            self._drop_workflow_message = (
                "当前点已保存 OFFLINE VERIFIED。"
                if self.dry_run
                else "当前点已由操作员保存 HARDWARE VERIFIED。"
            )
            self._refresh_drop_v1_panel(point.point_id)
        except Exception as exc:
            self._warn("Save DROP verification", str(exc))

    def back_from_drop_v1(self) -> None:
        if self._drop_pose_state in {"WAYPOINT", "DROP"}:
            self._warn("Return required", "Retract vertically before leaving this page.")
            return
        if self._drop_pose_state == "ABOVE" and self._drop_active_point:
            self._drop_pending_back_home = True
            self.safe_return_drop_v1(self._drop_active_point)
            return
        if self._drop_pose_state == "UNKNOWN":
            self._warn(
                "Robot pose unknown",
                "Use Emergency Stop/recovery and return the arm to a known safe pose.",
            )
            return
        self.lite.show_home()

    def _show_legacy_place_calibration(self) -> None:
        self._register_current_place_above()
        pwm = self._derived_place_pwm()
        self._place_last_sent_pwm = {
            joint: int(pwm[f"{joint:03d}"]) for joint in range(5)
        }
        self._place_last_adjustment = None
        self._place_calibration_parked = False
        self.lite.show_place_calibration(
            pwm,
            quick_mode=self._quick_special_flow,
            verified=bool(self._place_record and self._place_record.get("verified")),
        )

    def move_to_place_calibration(self) -> None:
        try:
            if not self.controller.is_connected:
                raise RuntimeError("请先连接 STM32。")
            if self._is_estop_latched() or self.arm_worker.busy or self.state_machine.busy:
                raise RuntimeError("机械臂尚未准备好。")
            if self.state_machine.snapshot().state != ArmState.OBSERVE_IDLE:
                raise RuntimeError("落子位标定必须从观察位开始。")
            above = self._register_current_place_above()
            plan = self.stage5.planner.plan_hover_with_pwm(
                7,
                7,
                above,
                holding_piece=False,
                dry_run=False,
                source="calibration_lite_place_above",
            )
            touch = action_pwm(self._stable_p77_actions["P77_TOUCH_HOLD"])
            touch.update(self.lite.place_calibration_pwm_values())
            touch["005"] = 1500
            self.actions.register_runtime(
                build_action(
                    "LITE_PLACE_TOUCH_IDLE",
                    touch,
                    action_times(self._stable_p77_actions["P77_TOUCH_HOLD"]),
                )
            )
            sequence = SequenceDefinition(
                name="MANUAL:LITE_PLACE_CALIBRATE",
                display_name="P77 分层下降到落子接触候选位（泵嘴关闭）",
                steps=tuple(plan.sequence.steps) + (ActionStep("LITE_PLACE_TOUCH_IDLE"),),
            )
            self._start_sequence(
                sequence,
                lambda: self.state_machine.begin_manual("LITE_PLACE_CALIBRATE"),
            )
            self._place_calibration_parked = False
            LOGGER.info("[LITE][PLACE_CALIBRATION_MOVE] source=P77_ABOVE")
        except Exception as exc:
            self._warn("移动到落子接触位失败", str(exc))

    def apply_place_calibration_pwm_joint(self, joint: int, target: int) -> None:
        jid = int(joint)
        requested = int(target)
        try:
            if not self.controller.is_connected:
                raise RuntimeError("请先连接 STM32。")
            if not self._place_calibration_parked:
                raise RuntimeError("请先移动到当前落子接触候选位。")
            if self._is_estop_latched() or self.arm_worker.busy:
                raise RuntimeError("机械臂尚未准备好。")
            lo = int(self.stage7.limits.joint_min[jid])
            hi = int(self.stage7.limits.joint_max[jid])
            applied = max(lo, min(hi, requested))
            previous = int(self._place_last_sent_pwm.get(jid, applied))
            self.controller.send_joint_pwm(jid, applied, time_ms=1000)
            self._place_last_adjustment = (jid, previous, applied, True)
            self._place_last_sent_pwm[jid] = applied
            self.lite._place_pwm_rows[jid].target.setValue(applied)
            self.lite.set_place_calibration_pwm_current(jid, applied)
        except Exception as exc:
            self._warn("落子位 PWM", str(exc))

    def remember_place_calibration_pwm_change(
        self, joint: int, previous: int, target: int
    ) -> None:
        self._place_last_adjustment = (
            int(joint), int(previous), int(target), False
        )

    def undo_place_calibration_adjustment(self) -> None:
        adjustment = self._place_last_adjustment
        if adjustment is None:
            self._warn("Undo", "当前没有可撤销的落子位调整。")
            return
        joint, previous, _target, was_applied = adjustment
        self.lite._place_pwm_rows[joint].target.setValue(previous)
        if was_applied:
            try:
                if not self._place_calibration_parked:
                    raise RuntimeError("机械臂不在落子接触标定位。")
                self.controller.send_joint_pwm(joint, previous, time_ms=1000)
                self._place_last_sent_pwm[joint] = previous
                self.lite.set_place_calibration_pwm_current(joint, previous)
            except Exception as exc:
                self._warn("Undo", str(exc))
                return
        self._place_last_adjustment = None

    def confirm_place_calibration_inaccurate(self) -> None:
        if not self._place_calibration_parked:
            self._warn("尚未测试", "请先移动到落子接触候选位。")
            return
        self.lite.place_calibration_status.setText(
            "已标记为不准确：请用下方 J0–J4 微调，确认接触位置后点击“位置准确 · 保存”。"
        )

    def _return_from_place_calibration(self) -> None:
        sequence = SequenceDefinition(
            name="RETURN_TO_OBSERVE",
            display_name="落子标定位安全返回观察位",
            steps=(
                ActionStep("P77_ABOVE_IDLE"),
                ActionStep("CARRY_HIGH_P77_IDLE"),
                ActionStep("OBSERVE_IDLE"),
            ),
        )
        self._start_sequence(sequence, self.state_machine.begin_return_to_observe)

    def confirm_place_calibration_accurate(self) -> None:
        try:
            if not self._place_calibration_parked:
                raise RuntimeError("请先移动并确认落子接触位。")
            session_id = None if self.stage7.session is None else self.stage7.session.session_id
            self._place_record = save_place_override(
                self._place_override_path,
                library=self.actions,
                stable_above=self._stable_p77_actions["P77_ABOVE_IDLE"],
                stable_touch_hold=self._stable_p77_actions["P77_TOUCH_HOLD"],
                stable_touch_release=self._stable_p77_actions["P77_TOUCH_RELEASE"],
                requested_pwm=self.lite.place_calibration_pwm_values(),
                calibration_session=session_id,
            )
            self._pending_after_return = (
                "place_calibration_saved",
                self._quick_special_flow,
            )
            self.lite.place_calibration_status.setText(
                "位置已确认并保存，正在先上升到 P77 ABOVE，再返回观察位…"
            )
            self._return_from_place_calibration()
            LOGGER.info(
                "[LITE][PLACE_CALIBRATION_SAVED] path=%s correction=%s",
                self._place_override_path,
                self._place_record["correction_delta"],
            )
        except Exception as exc:
            self._warn("保存落子接触位失败", str(exc))

    def review_calibrated_target(self, label: str) -> None:
        if label == "观察位 / PICK_ABOVE":
            self.show_observe_calibration()
            return
        if label == "取料接触位":
            self.show_pick_calibration()
            return
        if label == "落子接触位":
            self.show_place_calibration()
            return
        spec = next((item for item in ANCHOR_SPECS if item.label == label), None)
        if spec is None:
            self._warn("回看点位", f"未知点位：{label}")
            return
        self._current_spec = spec
        self.stage7_select_point(spec.row, spec.col)
        pwm = self._authoritative_point_pwm(spec.row, spec.col)
        self.lite.set_pwm_values(pwm)
        self._last_sent_pwm = {
            joint: int(pwm[f"{joint:03d}"]) for joint in range(5)
        }
        self.lite.show_review(label=spec.label, row=spec.row, col=spec.col)

    def commit_calibration(self) -> None:
        try:
            session = self.stage7.require_session()
            if len(session.generated_points) != 225:
                raise RuntimeError("生成点不完整，不能保存。")
            path = self.stage7.commit()
            self._apply_p77_runtime_from_session(session)
            if self._p77_point_pwm is not None:
                self._apply_p77_runtime_from_pwm(self._p77_point_pwm)
            self._load_place_runtime_override()
            self._refresh_summary()
            self.lite.global_state_label.setText("READY")
            QMessageBox.information(
                self,
                "Calibration Ready",
                f"标定已保存。\n{path}\n\n新运动路径仍需按点完成实机验证。",
            )
            LOGGER.info("[LITE][COMMITTED] path=%s", path)
        except Exception as exc:
            self._warn("保存标定失败", str(exc))

    def back_home(self) -> None:
        if self.state_machine.state == ArmState.HOVERING:
            self._pending_after_return = ("back_home", None)
            self.stage7_safe_return()
            return
        if self._pick_calibration_parked:
            self._pending_after_return = ("back_home", None)
            self._return_from_pick_calibration()
            return
        if self._place_calibration_parked:
            self._pending_after_return = ("back_home", None)
            self._return_from_place_calibration()
            return
        self._quick_special_flow = False
        self.wizard.reset_home()
        self._refresh_summary()
        self.lite.show_home()

    def open_legacy(self) -> None:
        self._legacy_dialog.show()
        self._legacy_dialog.raise_()
        self._legacy_dialog.activateWindow()

    def _after_safe_return(self) -> None:
        pending = self._pending_after_return
        self._pending_after_return = None
        if pending is None:
            return
        action, payload = pending
        if action == "anchor_saved":
            correction = self.wizard.correction_target is not None
            self._finish_anchor_after_return()
            if correction:
                self.generate_full_board(automatic=True)
        elif action == "test_accurate":
            next_spec = self.wizard.mark_test_accurate()
            if next_spec is None:
                self._show_complete()
            else:
                self._show_test(next_spec)
        elif action == "test_inaccurate":
            self._show_anchor(self.wizard.correct_current_test())
        elif action == "show_add_anchor":
            self._show_anchor(payload)
        elif action == "back_home":
            self.wizard.reset_home()
            self.lite.show_home()

    def emergency_stop(self) -> None:
        self._manual_movel_pose_index = None
        self._manual_movel_back_pending = False
        self._point_movel_pose_state = "UNKNOWN"
        self._point_movel_active_point = None
        self._point_movel_back_pending = False
        self._drop_pose_state = "UNKNOWN"
        self._drop_active_point = None
        self._drop_last_applied.clear()
        self._drop_live_pending_retract.clear()
        self._drop_live_verification_eligible.clear()
        self._drop_pending_back_home = False
        self._drop_waypoint_index = 0
        self._drop_at_final_correction = False
        self._drop_after_retract_step = None
        self._drop_after_safe_return_step = None
        self._drop_workflow_message = (
            "Emergency Stop 已锁定，机械臂位姿未知。人工恢复到安全位后重新开始。"
        )
        super().emergency_stop()

    def _complete_drop_v1_retract_transition(self) -> None:
        next_step = self._drop_after_retract_step
        self._drop_after_retract_step = None
        if next_step == 5:
            self._drop_wizard_step = 5
            self._drop_completed_steps.update({1, 2, 3})
            self._drop_workflow_message = (
                "DROP 已确认并回撤到 ABOVE。点击 Safe Return；"
                "回到安全位后 4. Test PLACE 将解锁。"
            )
        elif next_step == 3:
            self._drop_wizard_step = 3
            self._drop_above_confirmed = True
            self._drop_workflow_message = (
                "已回到 ABOVE。重新逐 waypoint 测试或点击 2. Full DROP。"
            )
        elif self._drop_wizard_step == 3:
            self._drop_workflow_message = (
                "已回到 ABOVE。可继续逐 waypoint 或执行完整 DROP。"
            )

    def _on_sequence_started(self, name: str, display_name: str) -> None:
        super()._on_sequence_started(name, display_name)
        if not self._lite_ready:
            return
        if name == "PICK_PIECE":
            self.lite.home_message.setText("正在取料…")
        elif name == "PLACE_TO_P77":
            self.lite.home_message.setText("正在向固定 P77 落子…")

    def _on_sequence_finished(self, name: str, success: bool, message: str) -> None:
        super()._on_sequence_finished(name, success, message)
        if not self._lite_ready:
            return
        if name.startswith("MANUAL:POINT_MOVEL:"):
            point_id_value = name.rsplit(":", 1)[-1]
            if not success:
                self._point_movel_pose_state = "UNKNOWN"
                self._point_movel_active_point = None
                self._point_movel_back_pending = False
                self.lite.point_movel_panel.status_label.setText(
                    f"Point MoveL failed: {message or 'unknown error'} · pose unknown"
                )
            elif name.startswith("MANUAL:POINT_MOVEL:ABOVE:"):
                self._point_movel_pose_state = "ABOVE"
                self._point_movel_active_point = point_id_value
                self.lite.point_movel_panel.status_label.setText(
                    f"{point_id_value} ABOVE reached · "
                    + ("OFFLINE VERIFIED" if self.dry_run else "NOT VERIFIED")
                )
            elif name.startswith("MANUAL:POINT_MOVEL:DROP:"):
                self._point_movel_pose_state = "DROP"
                self._point_movel_active_point = point_id_value
                self.lite.point_movel_panel.status_label.setText(
                    f"{point_id_value} DROP reached · unsaved tuning draft · "
                    + ("OFFLINE VERIFIED" if self.dry_run else "NOT VERIFIED")
                )
            elif name.startswith("MANUAL:POINT_MOVEL:RETURN_ABOVE:"):
                self._point_movel_pose_state = "ABOVE"
                self._point_movel_active_point = point_id_value
                self.lite.point_movel_panel.status_label.setText(
                    f"{point_id_value} returned to exact ABOVE"
                )
            elif name.startswith("MANUAL:POINT_MOVEL:RETURN_OBSERVATION:"):
                pending_back = self._point_movel_back_pending
                self._point_movel_back_pending = False
                self._point_movel_pose_state = "SAFE"
                self._point_movel_active_point = None
                try:
                    self._log_transition(self.state_machine.mark_observe_idle())
                except Exception as exc:
                    LOGGER.warning(
                        "Point MoveL return mark_observe_idle failed: %s", exc
                    )
                if pending_back:
                    self.lite.show_home()
            LOGGER.info(
                "[LITE][POINT_MOVEL][SEQUENCE_FINISHED] "
                "name=%s success=%s pose=%s evidence=%s",
                name,
                int(success),
                self._point_movel_pose_state,
                "OFFLINE VERIFIED" if self.dry_run and success else "NOT VERIFIED",
            )
            self._refresh_lite_status()
            return
        if name == "MANUAL:P77_MOVEL:RETURN_OBSERVATION":
            pending_back = self._manual_movel_back_pending
            self._manual_movel_back_pending = False
            self._manual_movel_pose_index = None
            if success:
                try:
                    self._log_transition(self.state_machine.mark_observe_idle())
                except Exception as exc:
                    LOGGER.warning(
                        "P77 manual return mark_observe_idle failed: %s", exc
                    )
                if pending_back:
                    self.lite.show_home()
            else:
                self.lite.manual_movel_panel.status_label.setText(
                    f"Return Observation failed: {message or 'unknown error'} · pose unknown"
                )
            self._refresh_lite_status()
            return
        if name == "MANUAL:P77_MOVEL:FULL_CYCLE":
            self._manual_movel_pose_index = None
            if success:
                self._pump_is_on = False
                self.lite.set_pump_state(False)
                try:
                    self._log_transition(self.state_machine.mark_observe_idle())
                except Exception as exc:
                    LOGGER.warning(
                        "P77 full cycle mark_observe_idle failed: %s", exc
                    )
                self.lite.manual_movel_panel.status_label.setText(
                    "P77 full flow completed at Observation · "
                    + ("OFFLINE VERIFIED" if self.dry_run else "NOT VERIFIED")
                )
            else:
                self.lite.manual_movel_panel.status_label.setText(
                    f"P77 full flow failed: {message or 'unknown error'} · pose unknown"
                )
            LOGGER.info(
                "[P77_MANUAL_MOVEL][FULL_CYCLE_FINISHED] success=%s evidence=%s",
                int(success),
                "OFFLINE VERIFIED" if self.dry_run and success else "NOT VERIFIED",
            )
            self._refresh_lite_status()
            return
        if name.startswith("MANUAL:P77_MOVEL:"):
            if not success:
                self._manual_movel_pose_index = None
                self.lite.manual_movel_panel.status_label.setText(
                    f"Manual MoveL failed: {message or 'unknown error'} · pose unknown"
                )
            else:
                target = int(name.rsplit(":", 1)[-1])
                self._manual_movel_pose_index = target
                self._manual_movel_view_index = target

                # Important:
                # Do NOT refresh the panel after Move Current Step.
                # The current UI PWM values are an unsaved tuning draft and must remain visible.
                if not name.startswith("MANUAL:P77_MOVEL:MOVE:"):
                    self._refresh_manual_movel_panel()

                self.lite.manual_movel_panel.status_label.setText(
                    f"Dry Run reached Step{target} · NOT VERIFIED"
                    if self.dry_run
                    else f"Sequence completed at Step{target} · operator confirmation required"
                )

            LOGGER.info(
                "[P77_MANUAL_MOVEL][SEQUENCE_FINISHED] name=%s success=%s pose=%s evidence=%s",
                name,
                int(success),
                self._manual_movel_pose_index,
                "OFFLINE VERIFIED" if self.dry_run and success else "NOT VERIFIED",
            )
            self._refresh_lite_status()
            return
        if name.startswith("MANUAL:LITE_DROP_"):
            point_text = name.rsplit(":", 1)[-1]
            try:
                point = parse_point_id(point_text).point_id
            except Exception:
                point = self._drop_active_point
            if not success:
                if point:
                    self._drop_live_pending_retract.discard(point)
                    self._drop_live_verification_eligible.discard(point)
                self._drop_pose_state = "UNKNOWN"
                self._drop_last_applied.clear()
                self._drop_test_place_succeeded = False
                self._drop_workflow_message = (
                    f"动作失败：{message or 'unknown error'}。机械臂位姿未知，"
                    "请人工恢复后重新开始。"
                )
            elif name.startswith("MANUAL:LITE_DROP_MOVE_ABOVE:"):
                self._drop_pose_state = "ABOVE"
                self._drop_active_point = point
                self._drop_waypoint_index = 0
                self._drop_at_final_correction = False
                self._drop_last_applied.clear()
                self._drop_wizard_step = 2
                self._drop_workflow_message = (
                    "已到达保存的 ABOVE。目视确认正确后再继续。"
                )
            elif name.startswith("MANUAL:LITE_DROP_WAYPOINT_RETURN:"):
                self._drop_pose_state = "ABOVE"
                self._drop_active_point = point
                self._drop_waypoint_index = 0
                self._drop_at_final_correction = False
                if point and not self.dry_run and point in self._drop_live_pending_retract:
                    self._drop_live_pending_retract.discard(point)
                    self._drop_live_verification_eligible.add(point)
                self._complete_drop_v1_retract_transition()
            elif name.startswith("MANUAL:LITE_DROP_WAYPOINT_"):
                target_text = name.split("MANUAL:LITE_DROP_WAYPOINT_", 1)[1].split(":", 1)[0]
                target = int(target_text)
                count = 0
                if point and self.drop_store is not None:
                    record = self.drop_store.drop_record(point) or {}
                    waypoints = record.get("waypoints") or []
                    count = len(waypoints)
                    if target == count - 1:
                        auto = record.get("drop_auto_pwm") or {}
                        self._drop_last_applied = {
                            joint: int(auto[key])
                            for joint, key in enumerate(SPATIAL_KEYS)
                        }
                self._drop_waypoint_index = target
                self._drop_at_final_correction = False
                self._drop_active_point = point
                self._drop_pose_state = (
                    "ABOVE" if target == 0 else ("DROP" if target == count - 1 else "WAYPOINT")
                )
                if point and target == count - 1 and not self.dry_run:
                    self._drop_live_pending_retract.add(point)
                self._drop_workflow_message = (
                    f"已到达 WP{target:02d}。"
                    + (
                        "确认 DROP 是否准确，或进入 PWM correction。"
                        if self._drop_pose_state == "DROP"
                        else "继续 Next Waypoint，或随时 Return to ABOVE。"
                    )
                )
            elif name.startswith("MANUAL:LITE_DROP_MOVE:"):
                self._drop_pose_state = "DROP"
                self._drop_active_point = point
                if point and self.drop_store is not None:
                    record = self.drop_store.drop_record(point) or {}
                    self._drop_waypoint_index = max(
                        0, len(record.get("waypoints") or []) - 1
                    )
                    final = record.get("drop_final_pwm") or {}
                    self._drop_at_final_correction = (
                        final != (record.get("drop_auto_pwm") or {})
                    )
                    self._drop_last_applied = {
                        joint: int(final[key]) for joint, key in enumerate(SPATIAL_KEYS)
                    }
                if point and not self.dry_run:
                    self._drop_live_pending_retract.add(point)
                self._drop_workflow_message = (
                    "完整 DROP 已到位。准确则继续；不准确则点击微调。"
                )
            elif name.startswith("MANUAL:LITE_DROP_RETRACT:"):
                self._drop_pose_state = "ABOVE"
                self._drop_active_point = point
                self._drop_waypoint_index = 0
                self._drop_at_final_correction = False
                if point and not self.dry_run and point in self._drop_live_pending_retract:
                    self._drop_live_pending_retract.discard(point)
                    self._drop_live_verification_eligible.add(point)
                self._complete_drop_v1_retract_transition()
            elif name.startswith("MANUAL:LITE_DROP_TEST_PLACE:"):
                self._drop_pose_state = "ABOVE"
                self._drop_active_point = point
                self._drop_waypoint_index = 0
                self._drop_at_final_correction = False
                self._pump_is_on = False
                self.lite.set_pump_state(False)
                self._drop_test_place_succeeded = True
                self._drop_completed_steps.update({1, 2, 3, 4})
                self._drop_workflow_message = (
                    "Test PLACE 动作已完成。目视确认取棋、落棋、释放和回撤均正确后，"
                    "再执行最终验证。"
                )
                if point and not self.dry_run:
                    self._drop_live_pending_retract.discard(point)
                    self._drop_live_verification_eligible.add(point)
            elif name.startswith("MANUAL:LITE_DROP_SAFE_RETURN:"):
                self._drop_pose_state = "SAFE"
                self._drop_active_point = None
                self._drop_last_applied.clear()
                self._drop_waypoint_index = 0
                self._drop_at_final_correction = False
                try:
                    self._log_transition(self.state_machine.mark_observe_idle())
                except Exception as exc:
                    LOGGER.warning("Lite DROP mark_observe_idle failed: %s", exc)
                next_step = self._drop_after_safe_return_step
                self._drop_after_safe_return_step = None
                if self._drop_pending_back_home:
                    self._drop_pending_back_home = False
                    self.lite.show_home()
                elif next_step == 1:
                    selected = point or self.lite.drop_v1_panel.point_id
                    self._reset_drop_v1_workflow(selected, loaded=False)
                elif next_step == 2:
                    self._drop_wizard_step = 2
                    self._drop_completed_steps = {1}
                    self._drop_above_confirmed = False
                    self._drop_accuracy_confirmed = False
                    self._drop_test_place_succeeded = False
                    self._drop_workflow_message = (
                        "已安全返回。重新执行 1. Move ABOVE 并确认后继续。"
                    )
                elif self._drop_wizard_step == 5 and self._drop_accuracy_confirmed:
                    self._drop_workflow_message = (
                        "已安全返回。现在点击 4. Test PLACE 执行完整动作。"
                    )
            LOGGER.info(
                "[LITE_DROP_V1][SEQUENCE_FINISHED] name=%s success=%s pose=%s evidence=%s",
                name,
                int(success),
                self._drop_pose_state,
                "OFFLINE VERIFIED" if self.dry_run and success else "NOT VERIFIED",
            )
            self._refresh_drop_v1_panel(point)
            self._refresh_lite_status()
            return
        if name == "MANUAL:LITE_PICK_CALIBRATE":
            self._pick_calibration_parked = bool(success)
            if success:
                for joint, value in self.lite.pick_calibration_pwm_values().items():
                    jid = int(joint)
                    self._pick_last_sent_pwm[jid] = int(value)
                    self.lite.set_pick_calibration_pwm_current(jid, int(value))
            else:
                self.lite.home_message.setText(
                    f"取料位标定移动失败：{message or 'unknown error'}"
                )
        elif name == "MANUAL:LITE_PLACE_CALIBRATE":
            self._place_calibration_parked = bool(success)
            if success:
                for joint, value in self.lite.place_calibration_pwm_values().items():
                    jid = int(joint)
                    self._place_last_sent_pwm[jid] = int(value)
                    self.lite.set_place_calibration_pwm_current(jid, int(value))
                self.lite.place_calibration_status.setText(
                    "已到达候选接触位。准确则保存，不准确则用 J0–J4 微调。"
                )
            else:
                self.lite.place_calibration_status.setText(
                    f"落子接触位移动失败：{message or 'unknown error'}"
                )
        elif name == "RETURN_TO_OBSERVE":
            self._pick_calibration_parked = False
            self._place_calibration_parked = False
            if success:
                self._pump_is_on = False
                self.lite.set_pump_state(False)
            pending = self._pending_after_return
            if success and pending is not None and pending[0] in {
                "pick_calibration_saved",
                "place_calibration_saved",
                "back_home",
            }:
                self._pending_after_return = None
                if pending[0] == "pick_calibration_saved" and bool(pending[1]):
                    self.show_place_calibration()
                elif pending[0] == "place_calibration_saved" and bool(pending[1]):
                    self._quick_special_flow = False
                    self.wizard.phase = WizardPhase.GENERATE
                    self.lite.show_generate()
                    self.lite.generate_message.setText(
                        "8 个部署标定步骤已完成。下一步：保存并生成全盘。"
                    )
                else:
                    self._quick_special_flow = False
                    self.wizard.reset_home()
                    self.lite.show_home()
            elif not success and pending is not None:
                self._pending_after_return = None
        elif name == "STAGE7_SAFE_RETURN" and success:
            self._after_safe_return()
        elif name == "PICK_PIECE":
            if success:
                self._pump_is_on = True
                self.lite.set_pump_state(True)
            self.lite.home_message.setText(
                "Pick sequence completed（无传感器确认）"
                if success
                else f"Pick sequence failed：{message or 'unknown error'}"
            )
        elif name == "PLACE_TO_P77":
            if success:
                self._pump_is_on = False
                self.lite.set_pump_state(False)
            self.lite.home_message.setText(
                "P77 place sequence completed（按动作时序估计）"
                if success
                else f"P77 place sequence failed：{message or 'unknown error'}"
            )
        self._refresh_lite_status()

    def _flush_stage7_jog(self) -> None:
        super()._flush_stage7_jog()
        if not self._lite_ready:
            return
        result = self.stage7.last_jog_result
        if result is not None and result.sent:
            self._last_sent_pwm[result.joint_id] = result.applied_pwm
            self.lite.set_pwm_current(result.joint_id, result.applied_pwm)

    def _refresh_lite_status(self) -> None:
        if not self._lite_ready:
            return
        connected = self.controller.is_connected
        camera_ready = self.camera_worker is not None and self.camera_state == "CONNECTED"
        estop = self._is_estop_latched()
        snapshot = self.state_machine.snapshot()
        busy = snapshot.busy or self.arm_worker.busy

        if self.drop_store is not None:
            current = self.lite.drop_v1_panel.point_id
            self._refresh_drop_v1_panel(current)
            drop_record = self.drop_store.drop_record(current) or {}
            self.lite.drop_v1_panel.set_motion_state(
                connected=connected,
                busy=busy,
                estop=estop,
                pose_state=self._drop_pose_state,
                verification_eligible=current in self._drop_live_verification_eligible,
                dry_run=self.dry_run,
                wizard_step=self._drop_wizard_step,
                above_confirmed=self._drop_above_confirmed,
                drop_accuracy_confirmed=self._drop_accuracy_confirmed,
                test_place_succeeded=self._drop_test_place_succeeded,
                waypoint_index=self._drop_waypoint_index,
                waypoint_count=len(drop_record.get("waypoints") or []),
            )

        if self.manual_movel_store is not None:
            record = self.manual_movel_store.step(self._manual_movel_view_index)
            self.lite.manual_movel_panel.set_controls(
                connected=connected,
                busy=busy,
                estop=estop,
                pose_index=self._manual_movel_pose_index,
                record=record,
                step_count=self.manual_movel_store.step_count(),
            )

        if self.point_movel_store is not None:
            self.lite.point_movel_panel.set_controls(
                connected=connected,
                busy=busy,
                estop=estop,
                dry_run=self.dry_run,
                pose_state=self._point_movel_pose_state,
                active_point=self._point_movel_active_point,
            )

        self.lite.set_serial_connected(connected)
        self.lite.set_camera_connected(camera_ready)
        self.lite.relocalize_button.setEnabled(camera_ready and not busy)
        self.lite.observe_calibrate_button.setEnabled(connected and not busy and not estop)
        self.lite.observe_move_button.setEnabled(connected and not busy and not estop)
        self.lite.observe_save_button.setEnabled(connected and not busy and not estop)
        self.lite.pick_calibrate_button.setEnabled(connected and not busy and not estop)
        self.lite.place_calibrate_button.setEnabled(connected and not busy and not estop)
        self.lite.pick_calibration_move_button.setEnabled(
            connected
            and not busy
            and not estop
            and snapshot.state == ArmState.OBSERVE_IDLE
        )
        self.lite.pick_calibration_save_button.setEnabled(
            connected and not busy and not estop and self._pick_calibration_parked
        )
        self.lite.place_calibration_move_button.setEnabled(
            connected
            and not busy
            and not estop
            and snapshot.state == ArmState.OBSERVE_IDLE
        )
        self.lite.place_calibration_accurate_button.setEnabled(
            connected and not busy and not estop and self._place_calibration_parked
        )
        self.lite.place_calibration_inaccurate_button.setEnabled(
            connected and not busy and not estop and self._place_calibration_parked
        )
        self.lite.stm32_status.set_status(
            f"Ready · {self.controller.port}" if connected else "未连接",
            "ready" if connected else "neutral",
        )
        self.lite.camera_status.set_status(
            "Ready" if camera_ready else "未连接",
            "ready" if camera_ready else "neutral",
        )
        reference_enabled = bool(self.lite_settings.get("robot_reference_enabled", False))
        self.lite.robot_reference_status.set_status(
            "Not found" if reference_enabled else "Reserved · NOT VERIFIED",
            "warning" if reference_enabled else "neutral",
        )
        self.lite.board_status.set_status(
            "Found" if self.board_locked else "Not found",
            "ready" if self.board_locked else ("warning" if camera_ready else "neutral"),
        )

        if estop:
            self.lite.global_state_label.setText("ERROR · ESTOP")
            self.lite.global_state_label.setStyleSheet("background:#f8d7da;color:#8f1d24;padding:8px 14px;font-weight:700;")
        elif connected and camera_ready and self.board_locked:
            self.lite.global_state_label.setText("READY")
            self.lite.global_state_label.setStyleSheet("background:#dff1e6;color:#185c39;padding:8px 14px;font-weight:700;")
        elif connected:
            self.lite.global_state_label.setText("CHECK SETUP")
            self.lite.global_state_label.setStyleSheet("background:#fff0cc;color:#875000;padding:8px 14px;font-weight:700;")
        else:
            self.lite.global_state_label.setText("NOT READY")
            self.lite.global_state_label.setStyleSheet("")

        ordinary = connected and not busy and not estop
        self.lite.return_button.setEnabled(ordinary)
        can_pick = snapshot.state in {ArmState.OBSERVE_IDLE, ArmState.OBSERVE_HOLD}
        self.lite.pick_button.setEnabled(ordinary and can_pick)
        self.lite.pick_button.setText(
            "再次取料（确认未吸到）"
            if snapshot.state == ArmState.OBSERVE_HOLD
            else "测试取料"
        )
        self.lite.place_button.setEnabled(
            ordinary
            and snapshot.state == ArmState.OBSERVE_HOLD
            and self.board_locked
            and self.target_visible
        )
        self.lite.estop_button.setEnabled(connected)
        self.lite.pump_toggle.setEnabled(connected and not busy and not estop)
        self.lite.recover_button.setEnabled(connected and estop and not busy)

        spec = self._current_spec
        parked = None if spec is None else (spec.row, spec.col)
        at_current = parked is not None and self._stage7_parked_above == parked
        if not connected:
            primary_text = "先连接 STM32"
            motion_enabled = False
        elif estop:
            primary_text = "急停已锁存"
            motion_enabled = False
        elif busy:
            primary_text = "机械臂运动中…"
            motion_enabled = False
        elif at_current and snapshot.state == ArmState.HOVERING:
            primary_text = "已在当前点 ABOVE"
            motion_enabled = False
        elif snapshot.state != ArmState.OBSERVE_IDLE:
            primary_text = "先回观察位"
            motion_enabled = True
        else:
            primary_text = "移动到 ABOVE"
            motion_enabled = spec is not None
        self.lite.anchor_move_button.setText(primary_text)
        self.lite.test_move_button.setText(primary_text)
        self.lite.set_motion_ready(motion_enabled)
        self.lite.confirm_anchor_button.setEnabled(at_current and not busy and not estop)
        self.lite.test_accurate_button.setEnabled(at_current and not busy and not estop)
        self.lite.test_inaccurate_button.setEnabled(not busy and not estop)

        if self.lite.pages.currentIndex() == self.lite.HOME:
            if not connected:
                self.lite.home_message.setText("下一步：连接 STM32。")
            elif not camera_ready:
                self.lite.home_message.setText("下一步：连接 Camera。")
            elif not self.board_locked:
                self.lite.home_message.setText("下一步：保持 Tag 15–18 可见，等待找到棋盘。")
            elif snapshot.state == ArmState.UNKNOWN:
                self.lite.home_message.setText("下一步：回观察位，建立安全起点。")
            else:
                self.lite.home_message.setText("设备已准备。可以开始快速标定。")

    def _warn(self, title: str, message: str) -> None:
        LOGGER.warning("[LITE][BLOCKED] %s: %s", title, message)
        QMessageBox.warning(self, title, message)

    def closeEvent(self, event) -> None:  # noqa: N802 - Qt API
        self.lite_timer.stop()
        self._legacy_dialog.close()
        super().closeEvent(event)
