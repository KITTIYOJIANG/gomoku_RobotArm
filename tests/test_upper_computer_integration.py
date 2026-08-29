from __future__ import annotations

import os
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

from app.game import GameMode, GameSession
from app.gui.game_panel import BOARD_THEMES, GomokuGamePanel
from app.integrated_v1.points import PointRef
from app.robot_api import (
    INVALID_POINT,
    STOPPED,
    SUCCESS,
    IntegratedRobotInterface,
    point_id_to_row_col,
    row_col_to_point_id,
)


def qt_app() -> QApplication:
    return QApplication.instance() or QApplication([])


def test_canonical_point_id_boundary_and_read_only_aliases() -> None:
    assert row_col_to_point_id(0, 0) == "P00_00"
    assert row_col_to_point_id(7, 7) == "P07_07"
    assert row_col_to_point_id(11, 14) == "P11_14"
    assert point_id_to_row_col("P11_14") == (11, 14)
    assert point_id_to_row_col("P77") == (7, 7)
    assert point_id_to_row_col("P311") == (3, 11)
    assert point_id_to_row_col("P99_99") is None


def test_game_session_records_canonical_history_and_difficulty() -> None:
    session = GameSession(mode=GameMode.HUMAN_AI, seed=3)
    session.set_ai_difficulty("easy")
    assert session.ai.search_radius == 1
    assert session.ai.max_search_candidates == 8
    assert session.human_move(7, 7)
    assert session.move_history[0].point_id == "P07_07"
    pending = session.choose_ai_move()
    assert pending is not None
    assert pending.point_id == row_col_to_point_id(pending.point.row, pending.point.col)
    assert session.complete_robot_move(success=True)
    assert session.move_history[-1] == pending


def test_integrated_robot_interface_delegates_without_pwm_or_second_serial() -> None:
    class FakeController:
        is_connected = False

        def connect(self, _port: str) -> None:
            self.is_connected = True

        def disconnect(self) -> None:
            self.is_connected = False

    class FakeRobot:
        state = SimpleNamespace(value="IDLE")
        is_busy = False
        worker = None
        active_point = None
        last_error = None

        def __init__(self) -> None:
            self.calls: list[tuple[int, int]] = []

        def place_piece(self, point, *, target_available: bool):
            assert target_available
            self.calls.append(tuple(point))
            self.active_point = PointRef(*point)
            return SimpleNamespace(accepted=True, reason="accepted")

    controller = FakeController()
    robot = FakeRobot()
    moves: list[str] = []
    stopped: list[bool] = []
    interface = IntegratedRobotInterface(
        controller=controller,
        robot=robot,
        calibration_ready=lambda: True,
        default_port="DRY_TEST",
        move_above_action=lambda point_id: moves.append(point_id) or True,
        home_action=lambda: True,
        stop_action=lambda: stopped.append(True),
    )

    assert interface.place_piece("bad") == INVALID_POINT
    assert interface.connect()
    assert interface.place_piece("P11_04") == SUCCESS
    assert robot.calls == [(11, 4)]
    assert interface.get_status()["current_point"] == "P11_04"
    assert interface.move_above("P07_07") == SUCCESS
    assert moves == ["P07_07"]
    assert interface.stop() == STOPPED
    assert stopped == [True]
    assert not hasattr(interface, "send_pwm")


def test_player_panel_contains_latest_upper_computer_features() -> None:
    qt_app()
    panel = GomokuGamePanel()
    try:
        assert panel.difficulty_combo.count() == 3
        assert panel.theme_combo.count() == len(BOARD_THEMES)
        assert panel.quick_calibration_button.toolTip()
        assert panel.stop_button.toolTip()
        panel.mode_combo.setCurrentIndex(0)
        panel.start_game()
        panel._on_board_clicked(7, 7)
        assert "P07_07" in panel.record_text()
        panel.open_replay()
        assert panel._replay_active
        panel.replay_step(1)
        panel.exit_replay()
        assert not panel._replay_active
    finally:
        panel.clock_timer.stop()
        panel.flash_timer.stop()
        panel.replay_timer.stop()
        panel.deleteLater()
