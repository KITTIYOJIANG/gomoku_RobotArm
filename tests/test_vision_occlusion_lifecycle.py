from __future__ import annotations

from types import SimpleNamespace

from app.main_window import MainWindow


class _Stage5Stub:
    def __init__(self) -> None:
        self.cleared = False

    def on_sequence_finished(self, name: str, success: bool, message: str) -> bool:
        return name in {"HOVER_TO_TARGET", "SAFE_RETURN_FROM_HOVER"}

    def clear_target(self) -> None:
        self.cleared = True


def _window_stub() -> SimpleNamespace:
    busy_updates: list[bool] = []
    return SimpleNamespace(
        stage5=_Stage5Stub(),
        camera_worker=None,
        camera_panel=SimpleNamespace(set_target_text=lambda _text: None),
        board_locked=True,
        _set_camera_arm_busy=lambda busy: busy_updates.append(bool(busy)),
        _sync_stage5_context=lambda **_kwargs: None,
        _refresh_stage5_ui=lambda: None,
        _refresh_ui=lambda: None,
        busy_updates=busy_updates,
    )


def test_successful_hover_keeps_vision_frozen_while_arm_is_parked() -> None:
    window = _window_stub()

    MainWindow._on_sequence_finished(
        window,
        "HOVER_TO_TARGET",
        True,
        "completed",
    )

    assert window.busy_updates == [True]


def test_safe_return_or_failed_hover_releases_vision_freeze() -> None:
    returned = _window_stub()
    MainWindow._on_sequence_finished(
        returned,
        "SAFE_RETURN_FROM_HOVER",
        True,
        "completed",
    )
    assert returned.busy_updates == [False]

    failed = _window_stub()
    MainWindow._on_sequence_finished(
        failed,
        "HOVER_TO_TARGET",
        False,
        "serial error",
    )
    assert failed.busy_updates == [False]
