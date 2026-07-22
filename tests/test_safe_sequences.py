from app.arm.actions import ActionLibrary
from app.arm.sequences import pick_piece, place_to_p77, run_full_cycle, validate_safe_sequence


def test_pick_sequence_never_drops_vacuum_after_hold():
    library = ActionLibrary()
    names = pick_piece().action_names
    assert names == ("SOURCE_TOUCH_IDLE", "SOURCE_TOUCH_HOLD", "OBSERVE_HOLD")
    hold_index = names.index("SOURCE_TOUCH_HOLD")
    assert all(library.get(name).target(5).pwm == 2500 for name in names[hold_index:])


def test_place_sequence_enforces_safe_p77_approach_and_exit():
    sequence = place_to_p77()
    validate_safe_sequence(sequence)
    names = sequence.action_names
    assert names == (
        "CARRY_HIGH_P77_HOLD",
        "P77_ABOVE_HOLD",
        "P77_TOUCH_HOLD",
        "P77_TOUCH_RELEASE",
        "P77_ABOVE_IDLE",
        "CARRY_HIGH_P77_IDLE",
        "OBSERVE_IDLE",
    )
    assert names.index("CARRY_HIGH_P77_HOLD") < names.index("P77_ABOVE_HOLD")
    assert names.index("P77_ABOVE_HOLD") < names.index("P77_TOUCH_HOLD")
    assert names.index("P77_TOUCH_RELEASE") < names.index("P77_ABOVE_IDLE")
    assert names.index("P77_ABOVE_IDLE") < names.index("OBSERVE_IDLE")


def test_full_cycle_contains_no_direct_observe_touch_transition():
    names = run_full_cycle().action_names
    assert names[0] == "OBSERVE_IDLE"
    pairs = set(zip(names, names[1:]))
    assert ("OBSERVE_IDLE", "P77_TOUCH_HOLD") not in pairs
    assert ("OBSERVE_HOLD", "P77_TOUCH_HOLD") not in pairs
    assert ("P77_TOUCH_HOLD", "OBSERVE_IDLE") not in pairs
    assert ("P77_TOUCH_RELEASE", "OBSERVE_IDLE") not in pairs


def test_full_cycle_p77_commands_share_the_same_calibrated_base_pwm():
    library = ActionLibrary()
    names = run_full_cycle().action_names
    p77_names = [
        name
        for name in names
        if name.startswith("P77_") or name.startswith("CARRY_HIGH_P77_")
    ]
    assert p77_names
    assert all(library.get(name).target(0).pwm == 1560 for name in p77_names)
    assert all("#000P1580" not in library.get(name).command for name in names)
    assert library.get("P77_ABOVE_HOLD").target(0).pwm == library.get("P77_TOUCH_HOLD").target(0).pwm
    assert library.get("P77_TOUCH_RELEASE").target(0).pwm == library.get("P77_ABOVE_IDLE").target(0).pwm
