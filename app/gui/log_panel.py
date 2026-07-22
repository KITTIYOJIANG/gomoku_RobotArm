from __future__ import annotations

from PySide6.QtGui import QTextCursor
from PySide6.QtWidgets import QGroupBox, QPlainTextEdit, QVBoxLayout


class LogPanel(QGroupBox):
    def __init__(self, parent=None) -> None:
        super().__init__("运行日志", parent)
        self.editor = QPlainTextEdit()
        self.editor.setReadOnly(True)
        self.editor.setMaximumBlockCount(2000)
        self.editor.setMinimumHeight(150)
        self.editor.setStyleSheet("font-family: Consolas, monospace; font-size: 11px;")
        layout = QVBoxLayout(self)
        layout.addWidget(self.editor)

    def append_message(self, message: str) -> None:
        self.editor.appendPlainText(message)
        self.editor.moveCursor(QTextCursor.MoveOperation.End)
