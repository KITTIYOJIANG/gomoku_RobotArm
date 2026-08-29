from __future__ import annotations

from datetime import datetime
from pathlib import Path
import threading
import time

from PySide6.QtCore import QRectF, Qt, QTimer, Signal
from PySide6.QtGui import QColor, QPainter, QPen, QRadialGradient
from PySide6.QtWidgets import (
    QComboBox,
    QFileDialog,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from app.calibration_lite.launcher import open_quick_calibration
from app.game import GameMode, GameSession, GomokuBoard, Stone
from app.robot_api import row_col_to_point_id


BOARD_THEMES = {
    "木质": {
        "background": "#D9A85F",
        "border": "#7C4B22",
        "grid": "#2D261E",
        "star": "#2D261E",
    },
    "深蓝": {
        "background": "#1E3E60",
        "border": "#10263E",
        "grid": "#D2E6F5",
        "star": "#EBF0F8",
    },
    "大理石": {
        "background": "#EEEAE2",
        "border": "#9E988C",
        "grid": "#5F5F5F",
        "star": "#5F5F5F",
    },
}


def _beep(frequency: int, duration_ms: int) -> None:
    """Play a best-effort Windows beep without blocking the UI thread."""

    def play() -> None:
        try:
            import winsound

            winsound.Beep(int(frequency), int(duration_ms))
        except Exception:
            return

    threading.Thread(target=play, daemon=True).start()


class GomokuBoardWidget(QWidget):
    point_clicked = Signal(int, int)

    def __init__(self, board: GomokuBoard, parent=None) -> None:
        super().__init__(parent)
        self.board = board
        self.pending: tuple[int, int] | None = None
        self.theme_name = "木质"
        self.setMinimumSize(420, 420)
        self.setToolTip("点击最近的棋盘交叉点落子；机械臂目标以 canonical point_id 发送")

    def set_theme(self, name: str) -> None:
        self.theme_name = name if name in BOARD_THEMES else "木质"
        self.update()

    def paintEvent(self, _event: object) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        side = min(self.width(), self.height())
        margin = 32.0
        spacing = (side - margin * 2.0) / (self.board.size - 1)
        theme = BOARD_THEMES[self.theme_name]
        painter.setBrush(QColor(theme["background"]))
        painter.setPen(QPen(QColor(theme["border"]), 3))
        painter.drawRoundedRect(QRectF(6, 6, side - 12, side - 12), 10, 10)
        painter.setPen(QPen(QColor(theme["grid"]), 1))
        for index in range(self.board.size):
            pos = margin + index * spacing
            painter.drawLine(int(margin), int(pos), int(side - margin), int(pos))
            painter.drawLine(int(pos), int(margin), int(pos), int(side - margin))
        painter.setBrush(QColor(theme["star"]))
        painter.setPen(Qt.PenStyle.NoPen)
        for row, col in ((3, 3), (3, 11), (7, 7), (11, 3), (11, 11)):
            x, y = margin + col * spacing, margin + row * spacing
            painter.drawEllipse(QRectF(x - 3, y - 3, 6, 6))
        radius = spacing * 0.42
        for row in range(self.board.size):
            for col in range(self.board.size):
                stone = self.board.grid[row][col]
                if stone == Stone.EMPTY:
                    continue
                x, y = margin + col * spacing, margin + row * spacing
                gradient = QRadialGradient(x - radius * 0.25, y - radius * 0.25, radius)
                if stone == Stone.BLACK:
                    gradient.setColorAt(0, QColor(88, 88, 88))
                    gradient.setColorAt(1, QColor(8, 8, 8))
                else:
                    gradient.setColorAt(0, QColor(255, 255, 255))
                    gradient.setColorAt(1, QColor(170, 170, 170))
                painter.setBrush(gradient)
                painter.setPen(Qt.PenStyle.NoPen)
                painter.drawEllipse(QRectF(x - radius, y - radius, radius * 2, radius * 2))
        if self.board.last_move is not None:
            row, col = self.board.last_move
            x, y = margin + col * spacing, margin + row * spacing
            painter.setBrush(QColor("#E53935"))
            painter.setPen(Qt.PenStyle.NoPen)
            painter.drawEllipse(QRectF(x - 3, y - 3, 6, 6))
        if self.pending is not None:
            row, col = self.pending
            x, y = margin + col * spacing, margin + row * spacing
            painter.setBrush(Qt.BrushStyle.NoBrush)
            painter.setPen(QPen(QColor("#00A8E8"), 3, Qt.PenStyle.DashLine))
            painter.drawEllipse(QRectF(x - radius, y - radius, radius * 2, radius * 2))

    def mousePressEvent(self, event) -> None:
        if event.button() != Qt.MouseButton.LeftButton:
            return
        side = min(self.width(), self.height())
        margin = 32.0
        spacing = (side - margin * 2.0) / (self.board.size - 1)
        col = int(round((event.position().x() - margin) / spacing))
        row = int(round((event.position().y() - margin) / spacing))
        if 0 <= row < self.board.size and 0 <= col < self.board.size:
            self.point_clicked.emit(row, col)


class GomokuGamePanel(QWidget):
    """Player UI; emits canonical point IDs and imports no motion internals."""

    robot_place_requested = Signal(str)
    calibration_requested = Signal()
    quick_check_requested = Signal()
    fast_calibration_requested = Signal()
    full_calibration_requested = Signal()
    advanced_calibration_requested = Signal()
    emergency_stop_requested = Signal()
    ai_ready = Signal(object)
    ai_failed = Signal(str)

    _STATUS_NORMAL = "background:#eef6f7;padding:8px;border-radius:5px;border:2px solid transparent;"
    _STATUS_TURN = "background:#fff7d6;padding:8px;border-radius:5px;border:2px solid #d89b00;font-weight:700;"

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.session = GameSession()
        self._ai_busy = False
        self._game_started_at: float | None = None
        self._last_ai_seconds = 0.0
        self._replay_active = False
        self._replay_index = 0
        self._replay_snapshot: GomokuBoard | None = None
        self._flash_on = False
        self._build_ui()
        self.ai_ready.connect(self._on_ai_ready)
        self.ai_failed.connect(self._on_ai_failed)
        self.clock_timer = QTimer(self)
        self.clock_timer.timeout.connect(self._update_clock)
        self.clock_timer.start(1000)
        self.flash_timer = QTimer(self)
        self.flash_timer.timeout.connect(self._tick_turn_flash)
        self.flash_timer.start(500)
        self.replay_timer = QTimer(self)
        self.replay_timer.timeout.connect(self._replay_auto_next)

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)
        title = QLabel("棋伴 · Gomoku Robot")
        title.setStyleSheet("font-size: 21px; font-weight: 700; color: #0f4c5c;")
        root.addWidget(title)

        status = QGroupBox("设备状态")
        status_layout = QGridLayout(status)
        self.camera_label = QLabel("Camera: Disconnected")
        self.robot_label = QLabel("Robot: Disconnected")
        self.calibration_label = QLabel("Calibration: Required")
        status_layout.addWidget(self.camera_label, 0, 0)
        status_layout.addWidget(self.robot_label, 0, 1)
        status_layout.addWidget(self.calibration_label, 1, 0, 1, 2)
        root.addWidget(status)

        self.time_label = QLabel("⏱ 本局用时 00:00  |  AI 思考 --")
        self.time_label.setToolTip("显示当前对局累计时间和最近一次 AI 计算耗时")
        root.addWidget(self.time_label)

        self.board_widget = GomokuBoardWidget(self.session.board)
        self.board_widget.point_clicked.connect(self._on_board_clicked)
        root.addWidget(self.board_widget, 1, Qt.AlignmentFlag.AlignHCenter)

        display_row = QHBoxLayout()
        display_row.addWidget(QLabel("棋盘皮肤"))
        self.theme_combo = QComboBox()
        self.theme_combo.addItems(BOARD_THEMES)
        self.theme_combo.setToolTip("切换木质、深蓝或大理石棋盘配色")
        self.theme_combo.currentTextChanged.connect(self.board_widget.set_theme)
        display_row.addWidget(self.theme_combo)
        display_row.addStretch()
        root.addLayout(display_row)

        controls = QFrame()
        controls_layout = QGridLayout(controls)
        self.mode_combo = QComboBox()
        self.mode_combo.addItem("人人对战", GameMode.HUMAN_HUMAN.value)
        self.mode_combo.addItem("人机对战", GameMode.HUMAN_AI.value)
        self.mode_combo.setCurrentIndex(1)
        self.mode_combo.setToolTip("选择人人对战或人机对战；机械臂仅处理 AI 的待落子请求")
        self.difficulty_combo = QComboBox()
        self.difficulty_combo.addItem("简单", "easy")
        self.difficulty_combo.addItem("标准", "standard")
        self.difficulty_combo.addItem("困难", "hard")
        self.difficulty_combo.setCurrentIndex(1)
        self.difficulty_combo.setToolTip("简单搜索范围小；困难搜索范围和候选更多")
        self.difficulty_combo.currentIndexChanged.connect(self._on_difficulty_changed)
        self.start_button = QPushButton("开始游戏")
        self.start_button.setToolTip("清空软件棋盘并开始一局；不会自动连接相机或串口")
        self.ai_button = QPushButton("请求 AI 落子")
        self.ai_button.setToolTip("后台计算 AI 落点，然后只发送 canonical point_id")
        self.quick_calibration_button = QPushButton("🎯 快速标定")
        self.quick_calibration_button.setToolTip("通过冻结的 launcher 公共入口打开 Calibration Lite")
        self.settings_button = QPushButton("标定设置")
        self.settings_button.setToolTip("打开集合版标定流程")
        self.quick_check_button = QPushButton("Quick Calibration Check · P77")
        self.quick_check_button.setToolTip("检查中心 P77 标定，不直接读写标定 JSON")
        self.fast_calibration_button = QPushButton("Fast Calibration · 5/9")
        self.fast_calibration_button.setToolTip("进入 5/9 点快速标定界面")
        self.full_calibration_button = QPushButton("Full Calibration · 15×15")
        self.full_calibration_button.setToolTip("进入 15×15 全棋盘标定界面")
        self.advanced_button = QPushButton("Advanced Calibration")
        self.advanced_button.setToolTip("显示高级标定工具；不会自动连接硬件")
        self.stop_button = QPushButton("Emergency Stop")
        self.stop_button.setToolTip("取消待执行动作并发送现有急停命令；停止状态保持锁存")
        self.stop_button.setStyleSheet("background:#b3262e;color:white;font-weight:700;")
        controls_layout.addWidget(QLabel("模式选择"), 0, 0)
        controls_layout.addWidget(self.mode_combo, 0, 1)
        controls_layout.addWidget(QLabel("AI 难度"), 1, 0)
        controls_layout.addWidget(self.difficulty_combo, 1, 1)
        controls_layout.addWidget(self.start_button, 2, 0)
        controls_layout.addWidget(self.ai_button, 2, 1)
        controls_layout.addWidget(self.quick_calibration_button, 3, 0)
        controls_layout.addWidget(self.settings_button, 3, 1)
        controls_layout.addWidget(self.quick_check_button, 4, 0, 1, 2)
        controls_layout.addWidget(self.fast_calibration_button, 5, 0)
        controls_layout.addWidget(self.full_calibration_button, 5, 1)
        controls_layout.addWidget(self.advanced_button, 6, 0, 1, 2)
        controls_layout.addWidget(self.stop_button, 7, 0, 1, 2)
        root.addWidget(controls)

        replay_row = QHBoxLayout()
        self.replay_button = QPushButton("📼 复盘")
        self.replay_prev_button = QPushButton("◀")
        self.replay_next_button = QPushButton("▶")
        self.replay_auto_button = QPushButton("自动播放")
        self.replay_exit_button = QPushButton("退出复盘")
        self.save_button = QPushButton("💾 保存棋谱")
        replay_tips = {
            self.replay_button: "从第 0 手开始复盘当前棋谱",
            self.replay_prev_button: "复盘上一手",
            self.replay_next_button: "复盘下一手",
            self.replay_auto_button: "每 800ms 自动播放一手",
            self.replay_exit_button: "退出复盘并恢复当前棋盘",
            self.save_button: "将棋谱以 UTF-8 文本导出",
        }
        for widget, tooltip in replay_tips.items():
            widget.setToolTip(tooltip)
            replay_row.addWidget(widget)
        root.addLayout(replay_row)

        self.game_status = QLabel("Ready. Click Start Game.")
        self.game_status.setWordWrap(True)
        self.game_status.setStyleSheet(self._STATUS_NORMAL)
        root.addWidget(self.game_status)

        self.start_button.clicked.connect(self.start_game)
        self.ai_button.clicked.connect(self.request_ai_move)
        self.quick_calibration_button.clicked.connect(self._open_quick_calibration)
        self.settings_button.clicked.connect(lambda _c=False: self.calibration_requested.emit())
        self.quick_check_button.clicked.connect(lambda _c=False: self.quick_check_requested.emit())
        self.fast_calibration_button.clicked.connect(lambda _c=False: self.fast_calibration_requested.emit())
        self.full_calibration_button.clicked.connect(lambda _c=False: self.full_calibration_requested.emit())
        self.advanced_button.clicked.connect(lambda _c=False: self.advanced_calibration_requested.emit())
        self.stop_button.clicked.connect(lambda _c=False: self.emergency_stop_requested.emit())
        self.replay_button.clicked.connect(self.open_replay)
        self.replay_prev_button.clicked.connect(lambda _c=False: self.replay_step(-1))
        self.replay_next_button.clicked.connect(lambda _c=False: self.replay_step(1))
        self.replay_auto_button.clicked.connect(self.toggle_replay_auto)
        self.replay_exit_button.clicked.connect(self.exit_replay)
        self.save_button.clicked.connect(self.save_game_record)
        self._set_replay_controls(False)

    def start_game(self) -> None:
        self.exit_replay(restore=False)
        self.session.reset(mode=str(self.mode_combo.currentData()))
        self.session.set_ai_difficulty(str(self.difficulty_combo.currentData()))
        self.board_widget.pending = None
        self.board_widget.update()
        self._ai_busy = False
        self._game_started_at = time.monotonic()
        self._last_ai_seconds = 0.0
        self.ai_button.setEnabled(True)
        self.start_button.setText("重新开始")
        self._set_status("Game started. Black moves first.", flash=True)

    def request_ai_move(self) -> None:
        if self._ai_busy or self.session.pending_robot_move is not None or self._replay_active:
            return
        self._ai_busy = True
        self.ai_button.setEnabled(False)
        self._set_status("AI is calculating…")
        snapshot = self.session.board.copy()

        def calculate() -> None:
            started = time.monotonic()
            try:
                move = self.session.ai.select_move(snapshot)
                self.ai_ready.emit((move, time.monotonic() - started))
            except Exception as exc:  # pragma: no cover - defensive signal bridge
                self.ai_failed.emit(str(exc))

        threading.Thread(target=calculate, daemon=True).start()

    def _on_ai_ready(self, payload: object) -> None:
        self._ai_busy = False
        elapsed = 0.0
        move = payload
        if (
            isinstance(payload, tuple)
            and len(payload) == 2
            and (payload[0] is None or isinstance(payload[0], tuple))
        ):
            move, elapsed = payload
        self._last_ai_seconds = float(elapsed)
        if move is None:
            self.ai_button.setEnabled(True)
            self._set_status("AI has no legal move.")
            return
        row, col = move
        from app.game.session import GameMove
        from app.integrated_v1.points import PointRef

        point = PointRef(int(row), int(col))
        pending = GameMove(
            point,
            self.session.ai_stone,
            row_col_to_point_id(point.row, point.col),
        )
        self.session.pending_robot_move = pending
        self.board_widget.pending = pending.point.as_tuple()
        self.board_widget.update()
        self._set_status(f"AI target: {pending.point_id}. Waiting for RobotInterface.")
        self.robot_place_requested.emit(pending.point_id)

    def _on_ai_failed(self, message: str) -> None:
        self._ai_busy = False
        self.ai_button.setEnabled(True)
        self._set_status(f"AI failed: {message}")

    def _on_board_clicked(self, row: int, col: int) -> None:
        if self._replay_active:
            self._set_status("Replay is active; exit replay before moving.")
            return
        if not self.session.human_move(row, col):
            self._set_status("Move rejected: occupied, wrong turn, or robot move pending.")
            return
        _beep(880, 50)
        self.board_widget.update()
        if self._handle_game_end():
            return
        if self.session.mode == GameMode.HUMAN_AI:
            self.request_ai_move()
        else:
            self._set_status(f"Move accepted. Next: {self.session.board.current_player.name}", flash=True)

    def complete_robot_request(
        self,
        success: bool,
        message: str = "",
        *,
        vision_confirmed: bool = False,
    ) -> None:
        if success and not vision_confirmed:
            self._set_status("Robot motion finished; waiting for vision verification. " + str(message))
            return
        committed = self.session.complete_robot_move(success=bool(success))
        self.board_widget.pending = None
        self.board_widget.update()
        self.ai_button.setEnabled(True)
        if success and committed:
            _beep(880, 50)
            if not self._handle_game_end():
                self._set_status("Robot move confirmed. " + str(message), flash=True)
        else:
            self._set_status("Robot move failed or was blocked: " + str(message))

    def apply_vision_matrix(self, matrix: object) -> None:
        if not isinstance(matrix, (tuple, list)):
            return
        pending_before = self.session.pending_robot_move
        try:
            self.session.apply_vision_matrix(matrix)
        except Exception as exc:
            self._set_status(f"Vision board rejected: {exc}")
            return
        pending_after = self.session.pending_robot_move
        if pending_before is not None and pending_after is None:
            _beep(880, 50)
            self.board_widget.pending = None
            self.ai_button.setEnabled(True)
            if not self._handle_game_end():
                self._set_status("Vision verified the robot move.", flash=True)
        self.board_widget.update()

    def _handle_game_end(self) -> bool:
        winner = self.session.winner
        if winner is None and not self.session.board.is_full():
            return False
        self._game_started_at = None
        self.ai_button.setEnabled(False)
        if winner is None:
            self._set_status("Draw. Replay or save the record.")
        else:
            self._set_status(f"Winner: {winner.name}. Replay or save the record.")
            _beep(1100, 220)
        return True

    def _on_difficulty_changed(self, _index: int) -> None:
        self.session.set_ai_difficulty(str(self.difficulty_combo.currentData()))

    def _open_quick_calibration(self) -> None:
        try:
            open_quick_calibration()
        except Exception as exc:
            QMessageBox.warning(self, "快速标定", str(exc))

    def _update_clock(self) -> None:
        elapsed = 0 if self._game_started_at is None else int(time.monotonic() - self._game_started_at)
        minutes, seconds = divmod(elapsed, 60)
        ai_text = "--" if self._last_ai_seconds <= 0 else f"{self._last_ai_seconds:.2f}s"
        self.time_label.setText(f"⏱ 本局用时 {minutes:02d}:{seconds:02d}  |  AI 思考 {ai_text}")

    def _set_status(self, text: str, *, flash: bool = False) -> None:
        self.game_status.setText(text)
        self._flash_on = bool(flash)
        self.game_status.setStyleSheet(self._STATUS_TURN if flash else self._STATUS_NORMAL)

    def _tick_turn_flash(self) -> None:
        if not self._flash_on:
            return
        current = self.game_status.styleSheet()
        self.game_status.setStyleSheet(
            self._STATUS_NORMAL if current == self._STATUS_TURN else self._STATUS_TURN
        )

    def open_replay(self) -> None:
        if not self.session.move_history:
            QMessageBox.information(self, "复盘", "当前没有可复盘的棋谱。")
            return
        self.replay_timer.stop()
        self._replay_snapshot = self.session.board.copy()
        self._replay_active = True
        self._replay_index = 0
        self._set_replay_controls(True)
        self._render_replay()

    def replay_step(self, delta: int) -> None:
        if not self._replay_active:
            return
        self._replay_index = max(
            0,
            min(len(self.session.move_history), self._replay_index + int(delta)),
        )
        self._render_replay()

    def toggle_replay_auto(self) -> None:
        if not self._replay_active:
            self.open_replay()
            if not self._replay_active:
                return
        if self.replay_timer.isActive():
            self.replay_timer.stop()
            self.replay_auto_button.setText("自动播放")
        else:
            if self._replay_index >= len(self.session.move_history):
                self._replay_index = 0
            self.replay_auto_button.setText("暂停")
            self.replay_timer.start(800)

    def _replay_auto_next(self) -> None:
        if self._replay_index >= len(self.session.move_history):
            self.replay_timer.stop()
            self.replay_auto_button.setText("自动播放")
            return
        self.replay_step(1)

    def _render_replay(self) -> None:
        board = self.session.board
        board.reset()
        for move in self.session.move_history[: self._replay_index]:
            board.place_stone(move.point.row, move.point.col, stone=move.stone)
        self.board_widget.pending = None
        self.board_widget.update()
        self._set_status(
            f"📼 复盘 {self._replay_index}/{len(self.session.move_history)} 手"
        )

    def exit_replay(self, *, restore: bool = True) -> None:
        self.replay_timer.stop()
        self.replay_auto_button.setText("自动播放")
        if restore and self._replay_snapshot is not None:
            snapshot = self._replay_snapshot
            board = self.session.board
            board.grid = [list(row) for row in snapshot.grid]
            board.current_player = snapshot.current_player
            board.last_move = snapshot.last_move
            board.move_count = snapshot.move_count
        self._replay_snapshot = None
        self._replay_active = False
        self._set_replay_controls(False)
        self.board_widget.update()

    def _set_replay_controls(self, active: bool) -> None:
        for widget in (
            self.replay_prev_button,
            self.replay_next_button,
            self.replay_auto_button,
            self.replay_exit_button,
        ):
            widget.setVisible(bool(active))
        self.start_button.setEnabled(not active)
        self.ai_button.setEnabled(not active and not self._ai_busy)

    def record_text(self) -> str:
        difficulty = str(self.difficulty_combo.currentData())
        lines = [
            "五子棋对局记录",
            f"导出时间：{datetime.now().isoformat(timespec='seconds')}",
            f"AI 难度：{difficulty}",
            f"总手数：{len(self.session.move_history)}",
            "",
        ]
        for index, move in enumerate(self.session.move_history, 1):
            color = "黑" if move.stone == Stone.BLACK else "白"
            lines.append(
                f"{index:03d}. {color} {move.point_id} "
                f"(row={move.point.row}, col={move.point.col})"
            )
        return "\n".join(lines) + "\n"

    def save_game_record(self) -> None:
        if not self.session.move_history:
            QMessageBox.information(self, "保存棋谱", "当前没有可保存的棋谱。")
            return
        suggested = f"gomoku_{datetime.now():%Y%m%d_%H%M%S}.txt"
        selected, _filter = QFileDialog.getSaveFileName(
            self,
            "保存棋谱",
            str(Path.home() / "Desktop" / suggested),
            "Text Files (*.txt)",
        )
        if not selected:
            return
        try:
            Path(selected).write_text(self.record_text(), encoding="utf-8")
        except Exception as exc:
            QMessageBox.warning(self, "保存棋谱", f"保存失败：{exc}")
            return
        QMessageBox.information(self, "保存棋谱", f"棋谱已保存：\n{selected}")

    def set_device_status(
        self,
        *,
        camera: str | None = None,
        robot: str | None = None,
        calibration: str | None = None,
    ) -> None:
        if camera is not None:
            self.camera_label.setText(f"Camera: {camera}")
        if robot is not None:
            self.robot_label.setText(f"Robot: {robot}")
        if calibration is not None:
            self.calibration_label.setText(f"Calibration: {calibration}")
