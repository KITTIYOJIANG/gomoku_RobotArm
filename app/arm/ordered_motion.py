from __future__ import annotations

from typing import Mapping

from .actions import Action, ActionLibrary, ServoTarget
from .sequences import ActionStep, SequenceDefinition, WaitStep


J1_ID = 1
SPATIAL_IDS = tuple(range(5))
BOARD_SAFE_RETURN_PHASE_TIME_MS = 1500


def action_pwm(action: Action) -> dict[int, int]:
    return {int(target.servo_id): int(target.pwm) for target in action.targets}


def _build_action(
    name: str,
    template: Action,
    pwm: Mapping[int, int],
    *,
    time_ms: int | None = None,
) -> Action:
    targets = tuple(
        ServoTarget(
            servo_id=int(target.servo_id),
            pwm=int(pwm[int(target.servo_id)]),
            time_ms=int(target.time_ms if time_ms is None else time_ms),
        )
        for target in template.targets
    )
    body = "".join(
        f"#{target.servo_id:03d}P{target.pwm:04d}T{target.time_ms:04d}!"
        for target in targets
    )
    return Action(name=name, command="{" + body + "}", targets=targets)


def j1_last_sequence(
    library: ActionLibrary,
    sequence: SequenceDefinition,
    *,
    initial_action: Action,
    runtime_prefix: str,
) -> SequenceDefinition:
    """Split coarse spatial transitions so J1 is commanded after J0/J2/J3/J4.

    Entering the board (OBSERVATION -> ABOVE) uses this policy: J0/J2/J3/J4
    reach the target first while J1 stays at the source, then J1 moves last.
    It is for high/ABOVE MoveJ-style transit only.  It must not be used to
    replace a Cartesian descent/ascent trajectory below ABOVE.
    """

    current = action_pwm(initial_action)
    staged_steps: list[ActionStep | WaitStep] = []
    action_index = 0
    for step in sequence.steps:
        if isinstance(step, WaitStep):
            staged_steps.append(step)
            continue
        target = library.get(step.action_name)
        target_pwm = action_pwm(target)
        non_j1_changed = any(
            int(target_pwm[joint]) != int(current[joint])
            for joint in SPATIAL_IDS
            if joint != J1_ID
        )
        j1_changed = int(target_pwm[J1_ID]) != int(current[J1_ID])
        if non_j1_changed and j1_changed:
            prefix = f"{runtime_prefix}_{action_index:02d}"
            pre_pwm = dict(target_pwm)
            pre_pwm[J1_ID] = int(current[J1_ID])
            pre = _build_action(f"{prefix}_J1_HELD", target, pre_pwm)
            final = _build_action(f"{prefix}_J1_LAST", target, target_pwm)
            library.register_runtime(pre)
            library.register_runtime(final)
            staged_steps.extend((ActionStep(pre.name), ActionStep(final.name)))
        else:
            staged_steps.append(step)
        current = target_pwm
        action_index += 1
    return SequenceDefinition(
        name=sequence.name,
        display_name=sequence.display_name,
        steps=tuple(staged_steps),
        requires_board=sequence.requires_board,
    )


def j1_first_sequence(
    library: ActionLibrary,
    sequence: SequenceDefinition,
    *,
    initial_action: Action,
    runtime_prefix: str,
    phase_time_ms: int | None = None,
) -> SequenceDefinition:
    """Split coarse spatial transitions so J1 is commanded before J0/J2/J3/J4.

    Leaving the board (ABOVE -> OBSERVATION) uses this policy: J1 moves to the
    target first while J0/J2/J3/J4 stay at the source, then J0/J2/J3/J4 follow.
    It is the reverse of :func:`j1_last_sequence` and is for high/ABOVE
    MoveJ-style transit only.  It must not be used to replace a Cartesian
    descent/ascent trajectory below ABOVE.
    """

    current = action_pwm(initial_action)
    staged_steps: list[ActionStep | WaitStep] = []
    action_index = 0
    for step in sequence.steps:
        if isinstance(step, WaitStep):
            staged_steps.append(step)
            continue
        target = library.get(step.action_name)
        target_pwm = action_pwm(target)
        non_j1_changed = any(
            int(target_pwm[joint]) != int(current[joint])
            for joint in SPATIAL_IDS
            if joint != J1_ID
        )
        j1_changed = int(target_pwm[J1_ID]) != int(current[J1_ID])
        if non_j1_changed and j1_changed:
            prefix = f"{runtime_prefix}_{action_index:02d}"
            pre_pwm = dict(current)
            pre_pwm[J1_ID] = int(target_pwm[J1_ID])
            pre = _build_action(
                f"{prefix}_J1_FIRST",
                target,
                pre_pwm,
                time_ms=phase_time_ms,
            )
            final = _build_action(
                f"{prefix}_J1_HELD",
                target,
                target_pwm,
                time_ms=phase_time_ms,
            )
            library.register_runtime(pre)
            library.register_runtime(final)
            staged_steps.extend((ActionStep(pre.name), ActionStep(final.name)))
        else:
            staged_steps.append(step)
        current = target_pwm
        action_index += 1
    return SequenceDefinition(
        name=sequence.name,
        display_name=sequence.display_name,
        steps=tuple(staged_steps),
        requires_board=sequence.requires_board,
    )
