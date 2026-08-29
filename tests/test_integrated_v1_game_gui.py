from __future__ import annotations

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

from app.game import GameMode, GameSession, Stone
from app.gui.drop_calibration_panel import DropCalibrationPanel
from app.gui.game_panel import GomokuGamePanel
from app.gui.rapid_calibration_panel import RapidCalibrationPanel
from app.main import build_parser


def qt_app() -> QApplication:
    return QApplication.instance() or QApplication([])


def test_game_session_outputs_only_point_id_for_robot() -> None:
    session = GameSession(mode=GameMode.HUMAN_AI, seed=1)
    assert session.human_move(7, 7)
    move = session.choose_ai_move()
    assert move is not None
    assert move.point_id.startswith("P")
    assert session.board.grid[move.point.row][move.point.col] == Stone.EMPTY
    assert session.complete_robot_move(success=True)
    assert session.board.grid[move.point.row][move.point.col] == Stone.WHITE


def test_human_human_mode_never_creates_robot_move() -> None:
    session = GameSession(mode=GameMode.HUMAN_HUMAN)
    assert session.human_move(0, 0)
    assert session.choose_ai_move() is None
    assert session.pending_robot_move is None


def test_drop_panel_has_225_status_cells_and_v1_actions() -> None:
    qt_app()
    panel = DropCalibrationPanel()
    assert len(panel._buttons) == 225
    assert len(panel._corrections) == 5
    assert len(panel._above_edits) == 5
    assert "OFFLINE ONLY" in panel.generate_all_button.text()
    assert panel.fast_mode.count() == 2


def test_game_panel_emits_point_id_not_pwm_or_serial() -> None:
    qt_app()
    panel = GomokuGamePanel()
    emitted: list[str] = []
    panel.robot_place_requested.connect(emitted.append)
    panel.session.reset(mode=GameMode.HUMAN_AI)
    assert panel.session.human_move(7, 7)
    move = panel.session.choose_ai_move()
    assert move is not None
    panel.session.pending_robot_move = None
    panel._on_ai_ready(move.point.as_tuple())
    assert emitted and emitted[-1].startswith("P")
    assert not hasattr(panel, "serial")


def test_integrated_tabs_route_first_setup_and_game() -> None:
    qt_app()
    panel = RapidCalibrationPanel()
    assert panel.task_tabs.count() == 5
    panel.show_first_setup()
    assert panel.task_tabs.currentWidget() is panel.drop_calibration_panel
    panel.show_drop_calibration()
    assert panel.task_tabs.currentWidget() is panel.drop_calibration_panel
    panel.show_game()
    assert panel.task_tabs.currentWidget() is panel.game_panel
    panel.show_advanced_calibration()
    assert panel.task_tabs.currentIndex() == 0


def test_cli_restores_explicit_dry_run_mode() -> None:
    args = build_parser().parse_args(["--dry-run", "--test-pattern"])
    assert args.dry_run
    assert args.test_pattern
