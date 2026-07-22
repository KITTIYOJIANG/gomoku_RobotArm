import re

from app.arm.actions import ActionLibrary


COMMAND = re.compile(r"^\{(?:#\d{3}P\d{4}T\d{4}!)+\}$")


def test_all_actions_are_complete_valid_ascii_commands():
    library = ActionLibrary()
    assert len(library.names) == 11
    for action in library:
        assert action.command.isascii()
        assert COMMAND.fullmatch(action.command)
        assert action.command.startswith("{") and action.command.endswith("}")
        assert [target.servo_id for target in action.targets] == list(range(8))
        assert all(500 <= target.pwm <= 2500 for target in action.targets)
        assert all(100 <= target.time_ms <= 9999 for target in action.targets)


def test_hold_idle_release_and_reserved_channel_invariants():
    library = ActionLibrary()
    for name in library.names:
        action = library.get(name)
        pump = action.target(5).pwm
        if name.endswith("_HOLD"):
            assert pump == 2500, name
        if name.endswith("_IDLE") or name.endswith("_RELEASE"):
            assert pump == 1500, name
        assert action.target(6).pwm == 1500, name
        assert action.target(7).pwm == 1500, name


def test_all_calibrated_p77_path_actions_use_base_1560_and_never_1580():
    library = ActionLibrary()
    p77_path = (
        "CARRY_HIGH_P77_HOLD",
        "CARRY_HIGH_P77_IDLE",
        "P77_ABOVE_HOLD",
        "P77_ABOVE_IDLE",
        "P77_TOUCH_HOLD",
        "P77_TOUCH_RELEASE",
    )
    for name in p77_path:
        action = library.get(name)
        assert action.target(0).pwm == 1560, name
        assert "#000P1560" in action.command, name
        assert "#000P1580" not in action.command, name
    assert all("#000P1580" not in action.command for action in library)
