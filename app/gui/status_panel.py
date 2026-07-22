from __future__ import annotations

from PySide6.QtWidgets import QFormLayout, QGroupBox, QLabel


class StatusPanel(QGroupBox):
    def __init__(self, parent=None) -> None:
        super().__init__("连接与状态", parent)
        self.camera = QLabel("DISCONNECTED")
        self.com = QLabel("DISCONNECTED")
        self.board = QLabel("BOARD LOST")
        self.corners = QLabel("0/4 LOST")
        self.pieces = QLabel("NOT RUN")
        self.arm = QLabel("DISCONNECTED")
        self.action = QLabel("-")
        layout = QFormLayout(self)
        layout.addRow("Camera", self.camera)
        layout.addRow("COM", self.com)
        layout.addRow("Board", self.board)
        layout.addRow("Corner Status", self.corners)
        layout.addRow("Piece Status", self.pieces)
        layout.addRow("Arm State", self.arm)
        layout.addRow("Current Action", self.action)

    def update_values(
        self,
        *,
        camera: str,
        com: str,
        board: str,
        corners: str,
        pieces: str,
        arm: str,
        action: str,
    ) -> None:
        self.camera.setText(camera)
        self.com.setText(com)
        self.board.setText(board)
        self.corners.setText(corners)
        self.pieces.setText(pieces)
        self.arm.setText(arm)
        self.action.setText(action)
