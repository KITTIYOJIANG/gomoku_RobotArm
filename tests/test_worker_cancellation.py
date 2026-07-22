import time

from app.arm.actions import ActionLibrary
from app.arm.sequences import ActionStep, SequenceDefinition
from app.arm.worker import ArmSequenceWorker


class CancelOnFirstWriteController:
    def __init__(self):
        self.sent = []
        self.worker = None

    def send_action(self, action):
        self.sent.append(action.name)
        assert self.worker is not None
        self.worker.cancel_pending()


def test_cancel_stops_sequence_instead_of_advancing_to_next_step():
    controller = CancelOnFirstWriteController()
    worker = ArmSequenceWorker(controller, ActionLibrary(), action_wait_margin_ms=0)
    controller.worker = worker
    sequence = SequenceDefinition(
        name="CANCEL_TEST",
        display_name="cancel test",
        steps=(
            ActionStep("OBSERVE_IDLE"),
            ActionStep("CARRY_HIGH_P77_HOLD"),
            ActionStep("P77_ABOVE_HOLD"),
        ),
    )
    worker.start()
    try:
        assert worker.submit(sequence)
        deadline = time.monotonic() + 2.0
        while worker.busy and time.monotonic() < deadline:
            time.sleep(0.01)
        assert not worker.busy
        assert controller.sent == ["OBSERVE_IDLE"]
    finally:
        worker.shutdown()
