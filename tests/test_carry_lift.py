from __future__ import annotations

from app.arm.actions import ActionLibrary
from app.stage5.hover_planner import DEFAULT_CARRY_LIFT_001, build_lifted_carry_action


def test_lifted_carry_raises_001() -> None:
    lib = ActionLibrary()
    base = lib.get("CARRY_HIGH_P77_IDLE")
    base_001 = base.target(1).pwm
    action = build_lifted_carry_action(lib, holding_piece=False, reference_001=None)
    assert action.target(1).pwm >= base_001 + DEFAULT_CARRY_LIFT_001 - 1
    # When target is already high, transit stays above it
    high = build_lifted_carry_action(lib, holding_piece=False, reference_001=1260)
    assert high.target(1).pwm >= 1260 + 20
