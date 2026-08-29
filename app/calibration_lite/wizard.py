from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum


@dataclass(frozen=True)
class AnchorSpec:
    label: str
    row: int
    col: int


ANCHOR_SPECS = (
    AnchorSpec("P77", 7, 7),
    AnchorSpec("P00", 0, 0),
    AnchorSpec("P014", 0, 14),
    AnchorSpec("P140", 14, 0),
    AnchorSpec("P1414", 14, 14),
)


class WizardPhase(str, Enum):
    HOME = "HOME"
    ANCHOR = "ANCHOR"
    GENERATE = "GENERATE"
    TEST = "TEST"
    COMPLETE = "COMPLETE"


@dataclass
class LiteWizardState:
    phase: WizardPhase = WizardPhase.HOME
    anchor_index: int = 0
    test_index: int = 0
    correction_target: AnchorSpec | None = None
    completed_anchor_labels: list[str] = field(default_factory=list)
    verified_labels: list[str] = field(default_factory=list)

    def start(self) -> AnchorSpec:
        self.phase = WizardPhase.ANCHOR
        self.anchor_index = 0
        self.test_index = 0
        self.correction_target = None
        self.completed_anchor_labels.clear()
        self.verified_labels.clear()
        return self.current_anchor

    @property
    def current_anchor(self) -> AnchorSpec:
        if self.correction_target is not None:
            return self.correction_target
        return ANCHOR_SPECS[self.anchor_index]

    @property
    def current_test(self) -> AnchorSpec:
        return ANCHOR_SPECS[self.test_index]

    @property
    def anchor_step(self) -> tuple[int, int]:
        return min(self.anchor_index + 1, len(ANCHOR_SPECS)), len(ANCHOR_SPECS)

    @property
    def test_step(self) -> tuple[int, int]:
        return min(self.test_index + 1, len(ANCHOR_SPECS)), len(ANCHOR_SPECS)

    def mark_anchor_saved(self) -> AnchorSpec | None:
        label = self.current_anchor.label
        if label not in self.completed_anchor_labels:
            self.completed_anchor_labels.append(label)
        if self.correction_target is not None:
            self.correction_target = None
            self.phase = WizardPhase.GENERATE
            return None
        if self.anchor_index + 1 >= len(ANCHOR_SPECS):
            self.phase = WizardPhase.GENERATE
            return None
        self.anchor_index += 1
        return self.current_anchor

    def begin_test(self) -> AnchorSpec:
        self.phase = WizardPhase.TEST
        self.test_index = 0
        self.verified_labels.clear()
        return self.current_test

    def mark_test_accurate(self) -> AnchorSpec | None:
        label = self.current_test.label
        if label not in self.verified_labels:
            self.verified_labels.append(label)
        if self.test_index + 1 >= len(ANCHOR_SPECS):
            self.phase = WizardPhase.COMPLETE
            return None
        self.test_index += 1
        return self.current_test

    def correct_current_test(self) -> AnchorSpec:
        self.correction_target = self.current_test
        self.phase = WizardPhase.ANCHOR
        return self.correction_target

    def add_anchor(self, row: int, col: int) -> AnchorSpec:
        self.correction_target = AnchorSpec(f"P{int(row)}{int(col)}", int(row), int(col))
        self.phase = WizardPhase.ANCHOR
        return self.correction_target

    def reset_home(self) -> None:
        self.phase = WizardPhase.HOME
        self.correction_target = None
