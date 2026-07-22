from __future__ import annotations

from dataclasses import dataclass
from typing import TypeAlias


@dataclass(frozen=True)
class ActionStep:
    action_name: str


@dataclass(frozen=True)
class WaitStep:
    label: str
    duration_ms: int


Step: TypeAlias = ActionStep | WaitStep


@dataclass(frozen=True)
class SequenceDefinition:
    name: str
    display_name: str
    steps: tuple[Step, ...]
    requires_board: bool = False

    @property
    def action_names(self) -> tuple[str, ...]:
        return tuple(step.action_name for step in self.steps if isinstance(step, ActionStep))


def return_to_observe() -> SequenceDefinition:
    return SequenceDefinition(
        name="RETURN_TO_OBSERVE",
        display_name="回观察位",
        steps=(ActionStep("OBSERVE_IDLE"),),
    )


def pick_piece(vacuum_build_ms: int = 700) -> SequenceDefinition:
    sequence = SequenceDefinition(
        name="PICK_PIECE",
        display_name="取料",
        steps=(
            ActionStep("OBSERVE_IDLE"),
            ActionStep("OBSERVE_IDLE"),
            ActionStep("SOURCE_TOUCH_HOLD"),
            WaitStep("VACUUM BUILD", int(vacuum_build_ms)),
            ActionStep("OBSERVE_HOLD"),
        ),
    )
    validate_safe_sequence(sequence)
    return sequence


def place_to_p77(release_ms: int = 700) -> SequenceDefinition:
    sequence = SequenceDefinition(
        name="PLACE_TO_P77",
        display_name="下棋到 P77",
        requires_board=True,
        steps=(
            ActionStep("CARRY_HIGH_P77_HOLD"),
            ActionStep("P77_ABOVE_HOLD"),
            ActionStep("P77_TOUCH_HOLD"),
            ActionStep("P77_TOUCH_RELEASE"),
            WaitStep("VACUUM RELEASE", int(release_ms)),
            ActionStep("P77_ABOVE_IDLE"),
            ActionStep("CARRY_HIGH_P77_IDLE"),
            ActionStep("OBSERVE_IDLE"),
        ),
    )
    validate_safe_sequence(sequence)
    return sequence


def run_full_cycle(vacuum_build_ms: int = 700, release_ms: int = 700) -> SequenceDefinition:
    sequence = SequenceDefinition(
        name="FULL_CYCLE",
        display_name="完整固定点流程（实验功能）",
        requires_board=True,
        steps=(ActionStep("OBSERVE_IDLE"),)
        + pick_piece(vacuum_build_ms).steps
        + place_to_p77(release_ms).steps,
    )
    validate_safe_sequence(sequence)
    return sequence



def hover_to_target(*, holding_piece: bool) -> SequenceDefinition:
    """Carry-high then TARGET_ABOVE. Runtime target action must be registered first."""
    carry = "CARRY_HIGH_P77_HOLD" if holding_piece else "CARRY_HIGH_P77_IDLE"
    target = "TARGET_ABOVE_HOLD" if holding_piece else "TARGET_ABOVE_IDLE"
    sequence = SequenceDefinition(
        name="HOVER_TO_TARGET",
        display_name="悬停到目标上方",
        requires_board=True,
        steps=(ActionStep(carry), ActionStep(target)),
    )
    validate_safe_sequence(sequence)
    return sequence


def safe_return_from_hover(*, holding_piece: bool) -> SequenceDefinition:
    carry = "CARRY_HIGH_P77_HOLD" if holding_piece else "CARRY_HIGH_P77_IDLE"
    observe = "OBSERVE_HOLD" if holding_piece else "OBSERVE_IDLE"
    sequence = SequenceDefinition(
        name="SAFE_RETURN_FROM_HOVER",
        display_name="安全返回观察位",
        steps=(ActionStep(carry), ActionStep(observe)),
    )
    validate_safe_sequence(sequence)
    return sequence

def validate_safe_sequence(sequence: SequenceDefinition) -> None:
    names = sequence.action_names
    if any(step.duration_ms < 0 for step in sequence.steps if isinstance(step, WaitStep)):
        raise ValueError(f"{sequence.name}: wait duration cannot be negative")

    unsafe_pairs = {
        ("OBSERVE_IDLE", "P77_TOUCH_HOLD"),
        ("OBSERVE_HOLD", "P77_TOUCH_HOLD"),
        ("P77_TOUCH_HOLD", "OBSERVE_IDLE"),
        ("P77_TOUCH_RELEASE", "OBSERVE_IDLE"),
    }
    pairs = set(zip(names, names[1:]))
    collision = pairs & unsafe_pairs
    if collision:
        raise ValueError(f"{sequence.name}: unsafe direct transition(s): {sorted(collision)}")

    if "P77_TOUCH_HOLD" in names:
        touch_index = names.index("P77_TOUCH_HOLD")
        prefix = names[:touch_index]
        if "CARRY_HIGH_P77_HOLD" not in prefix or "P77_ABOVE_HOLD" not in prefix:
            raise ValueError(
                f"{sequence.name}: P77 touch requires CARRY_HIGH_P77_HOLD and P77_ABOVE_HOLD"
            )
        if prefix.index("CARRY_HIGH_P77_HOLD") > prefix.index("P77_ABOVE_HOLD"):
            raise ValueError(f"{sequence.name}: carry-high must precede P77-above")

    if "P77_TOUCH_RELEASE" in names:
        release_index = names.index("P77_TOUCH_RELEASE")
        suffix = names[release_index + 1 :]
        if (
            "P77_ABOVE_IDLE" not in suffix
            or "CARRY_HIGH_P77_IDLE" not in suffix
            or "OBSERVE_IDLE" not in suffix
        ):
            raise ValueError(f"{sequence.name}: safe P77 exit is incomplete")
        if not (
            suffix.index("P77_ABOVE_IDLE")
            < suffix.index("CARRY_HIGH_P77_IDLE")
            < suffix.index("OBSERVE_IDLE")
        ):
            raise ValueError(f"{sequence.name}: safe P77 exit order is invalid")

