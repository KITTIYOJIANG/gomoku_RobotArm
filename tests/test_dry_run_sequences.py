from app.arm.actions import ActionLibrary
from app.arm.controller import SerialArmController
from app.arm.sequences import ActionStep, WaitStep, pick_piece, place_to_p77


def execute_without_waiting(sequence, controller, library):
    waits = []
    for step in sequence.steps:
        if isinstance(step, ActionStep):
            controller.send_action(library.get(step.action_name))
        elif isinstance(step, WaitStep):
            waits.append((step.label, step.duration_ms))
    return waits


def test_dry_run_records_exact_pick_and_place_order_without_hardware():
    library = ActionLibrary()
    controller = SerialArmController(dry_run=True)
    controller.connect("COM6")
    assert controller.is_connected
    assert controller._connection is None

    pick_waits = execute_without_waiting(pick_piece(), controller, library)
    pick_names = [label for label, _command in controller.dry_run_commands]
    assert pick_names == ["SOURCE_TOUCH_IDLE", "SOURCE_TOUCH_HOLD", "OBSERVE_HOLD"]
    assert pick_waits == [("VACUUM BUILD", 700)]

    controller.dry_run_commands.clear()
    place_waits = execute_without_waiting(place_to_p77(), controller, library)
    place_names = [label for label, _command in controller.dry_run_commands]
    assert place_names == [
        "CARRY_HIGH_P77_HOLD",
        "P77_ABOVE_HOLD",
        "P77_TOUCH_HOLD",
        "P77_TOUCH_RELEASE",
        "P77_ABOVE_IDLE",
        "CARRY_HIGH_P77_IDLE",
        "OBSERVE_IDLE",
    ]
    assert place_waits == [("VACUUM RELEASE", 700)]


def test_dry_run_full_cycle_contains_only_calibrated_p77_base_commands():
    from app.arm.sequences import run_full_cycle

    library = ActionLibrary()
    controller = SerialArmController(dry_run=True)
    controller.connect("COM6")
    execute_without_waiting(run_full_cycle(), controller, library)
    labels = [label for label, _command in controller.dry_run_commands]
    assert labels == [
        "OBSERVE_IDLE",
        "SOURCE_TOUCH_IDLE",
        "SOURCE_TOUCH_HOLD",
        "OBSERVE_HOLD",
        "CARRY_HIGH_P77_HOLD",
        "P77_ABOVE_HOLD",
        "P77_TOUCH_HOLD",
        "P77_TOUCH_RELEASE",
        "P77_ABOVE_IDLE",
        "CARRY_HIGH_P77_IDLE",
        "OBSERVE_IDLE",
    ]
    for label, command in controller.dry_run_commands:
        if label.startswith("P77_") or label.startswith("CARRY_HIGH_P77_"):
            assert "#000P1560" in command
        assert "#000P1580" not in command


def test_dry_run_estop_records_immediate_stop_command():
    controller = SerialArmController(dry_run=True)
    controller.connect("COM6")
    controller.emergency_stop()
    assert controller.dry_run_commands == [("ESTOP", "$DST!")]
