"""Vendored game-domain layer from the existing gomoku_project host UI.

The original source remains at D:/Projects/Embodied/RobotArms/gomoku_project.
Only UI-independent game/AI behavior is carried into the integrated runtime.
"""

from .ai import DIFFICULTY_CONFIG, HeuristicAI, RandomAI
from .core import BOARD_SIZE, Coord, GomokuBoard, Stone
from .session import GameMode, GameSession

__all__ = [
    "BOARD_SIZE",
    "Coord",
    "DIFFICULTY_CONFIG",
    "GameMode",
    "GameSession",
    "GomokuBoard",
    "HeuristicAI",
    "RandomAI",
    "Stone",
]
