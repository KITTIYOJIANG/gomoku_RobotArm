from __future__ import annotations

import logging
from queue import Empty, Queue
import threading
import time

from PySide6.QtCore import QThread, Signal

from .actions import ActionLibrary
from .controller import SerialArmController
from .sequences import ActionStep, SequenceDefinition, WaitStep


LOGGER = logging.getLogger(__name__)


class SequenceCancelled(RuntimeError):
    pass


class ArmSequenceWorker(QThread):
    """One persistent worker and one-slot queue for all ordinary arm movement."""

    sequence_started = Signal(str, str)
    step_started = Signal(str, str)
    sequence_finished = Signal(str, bool, str)
    log_message = Signal(str)

    def __init__(
        self,
        controller: SerialArmController,
        actions: ActionLibrary,
        *,
        action_wait_margin_ms: int = 200,
        parent=None,
    ) -> None:
        super().__init__(parent)
        self.controller = controller
        self.actions = actions
        self.action_wait_margin_ms = max(0, int(action_wait_margin_ms))
        self._queue: Queue[SequenceDefinition | None] = Queue(maxsize=1)
        self._cancel = threading.Event()
        self._shutdown = threading.Event()
        self._state_lock = threading.Lock()
        self._accepted_busy = False

    @property
    def busy(self) -> bool:
        with self._state_lock:
            return self._accepted_busy

    def submit(self, sequence: SequenceDefinition) -> bool:
        with self._state_lock:
            if self._accepted_busy or self._shutdown.is_set():
                return False
            self._accepted_busy = True
            self._cancel.clear()
            self._queue.put_nowait(sequence)
            return True

    def cancel_pending(self) -> None:
        self._cancel.set()

    def shutdown(self, timeout_ms: int = 3000) -> None:
        self._shutdown.set()
        self._cancel.set()
        try:
            self._queue.put_nowait(None)
        except Exception:
            pass
        if self.isRunning():
            self.wait(max(0, int(timeout_ms)))

    def run(self) -> None:
        while not self._shutdown.is_set():
            try:
                sequence = self._queue.get(timeout=0.1)
            except Empty:
                continue
            if sequence is None:
                break
            self._execute(sequence)

    def _execute(self, sequence: SequenceDefinition) -> None:
        success = False
        message = ""
        self.sequence_started.emit(sequence.name, sequence.display_name)
        LOGGER.info("ACTION START %s", sequence.name)
        try:
            if self._cancel.is_set():
                raise SequenceCancelled("cancelled before first step")
            for step in sequence.steps:
                if self._cancel.is_set():
                    raise SequenceCancelled("remaining action steps cancelled")
                if isinstance(step, ActionStep):
                    action = self.actions.get(step.action_name)
                    self.step_started.emit(sequence.name, action.name)
                    self.log_message.emit(f"TX {action.name}: {action.command}")
                    self.controller.send_action(action)
                    self._interruptible_wait(action.duration_ms + self.action_wait_margin_ms)
                elif isinstance(step, WaitStep):
                    self.step_started.emit(sequence.name, step.label)
                    self.log_message.emit(f"{step.label} {step.duration_ms}ms")
                    LOGGER.info("WAIT %s %dms", step.label, step.duration_ms)
                    self._interruptible_wait(step.duration_ms)
                else:  # pragma: no cover - defensive
                    raise TypeError(f"Unsupported sequence step: {step!r}")
            success = True
            message = "completed"
            LOGGER.info("ACTION DONE %s", sequence.name)
        except SequenceCancelled as exc:
            message = str(exc)
            LOGGER.warning("ACTION CANCELLED %s: %s", sequence.name, exc)
        except Exception as exc:
            message = str(exc)
            LOGGER.exception("ACTION FAILED %s", sequence.name)
        finally:
            with self._state_lock:
                self._accepted_busy = False
            self.sequence_finished.emit(sequence.name, success, message)

    def _interruptible_wait(self, duration_ms: int) -> None:
        deadline = time.monotonic() + max(0, duration_ms) / 1000.0
        while True:
            if self._cancel.is_set() or self._shutdown.is_set():
                raise SequenceCancelled("cancelled during wait")
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return
            time.sleep(min(0.02, remaining))
