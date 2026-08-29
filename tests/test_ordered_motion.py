from app.arm.actions import ActionLibrary
from app.arm.ordered_motion import action_pwm, j1_first_sequence, j1_last_sequence
from app.arm.sequences import ActionStep, SequenceDefinition
from app.calibration_lite.observe_pose import build_action


def _action(name: str, values: list[int]):
    pwm = {f"{joint:03d}": value for joint, value in enumerate(values)}
    times = {f"{joint:03d}": 1000 for joint in range(8)}
    return build_action(name, pwm, times)


def test_j1_is_held_until_other_spatial_axes_reach_target() -> None:
    library = ActionLibrary()
    source = _action("TEST_SOURCE", [1500] * 8)
    target = _action(
        "TEST_TARGET",
        [1600, 1400, 1300, 1200, 1500, 1500, 1500, 1500],
    )
    library.register_runtime(target)
    sequence = j1_last_sequence(
        library,
        SequenceDefinition(
            name="TEST_ORDER",
            display_name="test",
            steps=(ActionStep(target.name),),
        ),
        initial_action=source,
        runtime_prefix="TEST_ORDER",
    )

    assert sequence.action_names == (
        "TEST_ORDER_00_J1_HELD",
        "TEST_ORDER_00_J1_LAST",
    )
    held = action_pwm(library.get(sequence.action_names[0]))
    final = action_pwm(library.get(sequence.action_names[1]))
    assert held[1] == 1500
    assert {joint: held[joint] for joint in (0, 2, 3, 4)} == {
        0: 1600,
        2: 1300,
        3: 1200,
        4: 1500,
    }
    assert final == action_pwm(target)


def test_j1_only_transition_is_not_duplicated() -> None:
    library = ActionLibrary()
    source = _action("TEST_SOURCE", [1500] * 8)
    target = _action("TEST_J1_ONLY", [1500, 1400, 1500, 1500, 1500, 1500, 1500, 1500])
    library.register_runtime(target)
    sequence = j1_last_sequence(
        library,
        SequenceDefinition(
            name="TEST_J1_ONLY_SEQUENCE",
            display_name="test",
            steps=(ActionStep(target.name),),
        ),
        initial_action=source,
        runtime_prefix="TEST_J1_ONLY",
    )
    assert sequence.action_names == ("TEST_J1_ONLY",)


def test_j1_first_moves_j1_before_other_spatial_axes() -> None:
    library = ActionLibrary()
    source = _action("TEST_SOURCE", [1500] * 8)
    target = _action(
        "TEST_TARGET",
        [1600, 1400, 1300, 1200, 1500, 1500, 1500, 1500],
    )
    library.register_runtime(target)
    sequence = j1_first_sequence(
        library,
        SequenceDefinition(
            name="TEST_ORDER_RETURN",
            display_name="test",
            steps=(ActionStep(target.name),),
        ),
        initial_action=source,
        runtime_prefix="TEST_ORDER_RETURN",
        phase_time_ms=1500,
    )

    assert sequence.action_names == (
        "TEST_ORDER_RETURN_00_J1_FIRST",
        "TEST_ORDER_RETURN_00_J1_HELD",
    )
    first = action_pwm(library.get(sequence.action_names[0]))
    final = action_pwm(library.get(sequence.action_names[1]))
    assert first[1] == 1400
    assert {joint: first[joint] for joint in (0, 2, 3, 4)} == {
        0: 1500,
        2: 1500,
        3: 1500,
        4: 1500,
    }
    assert final == action_pwm(target)
    assert all(
        target.time_ms == 1500
        for name in sequence.action_names
        for target in library.get(name).targets
    )


def test_j1_first_j1_only_transition_is_not_duplicated() -> None:
    library = ActionLibrary()
    source = _action("TEST_SOURCE", [1500] * 8)
    target = _action("TEST_J1_ONLY", [1500, 1400, 1500, 1500, 1500, 1500, 1500, 1500])
    library.register_runtime(target)
    sequence = j1_first_sequence(
        library,
        SequenceDefinition(
            name="TEST_J1_ONLY_SEQUENCE_RETURN",
            display_name="test",
            steps=(ActionStep(target.name),),
        ),
        initial_action=source,
        runtime_prefix="TEST_J1_ONLY_RETURN",
    )
    assert sequence.action_names == ("TEST_J1_ONLY",)
