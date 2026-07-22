import importlib

import pytest


MODULES = (
    "app.config",
    "app.logging_config",
    "app.arm.actions",
    "app.arm.controller",
    "app.arm.state",
    "app.arm.sequences",
    "app.arm.worker",
    "app.vision.camera_selector",
    "app.vision.localization.apriltag_detector",
    "app.vision.localization.board_localizer",
    "app.vision.localization.pipeline",
    "app.vision.board_locator",
    "app.vision.board_tracker",
    "app.vision.overlay",
    "app.vision.piece_recognizer",
    "app.vision.stone_detector",
    "app.vision.camera_worker",
    "app.gui.camera_panel",
    "app.gui.control_panel",
    "app.gui.status_panel",
    "app.gui.log_panel",
    "app.main_window",
    "app.main",
)


@pytest.mark.parametrize("module_name", MODULES)
def test_core_module_imports(module_name):
    assert importlib.import_module(module_name) is not None
