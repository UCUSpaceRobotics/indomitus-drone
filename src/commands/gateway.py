"""Record-before-submit one-shot command gateway."""

from __future__ import annotations

from src.comm.messages import CommandEnvelope
from src.commands.ledger import OperationLedger, OperationRecord, OperationStatus
from src.commands.types import Command
from src.runtime.clock import Clock


class CommandGateway:
    def __init__(self, command_queue, ledger: OperationLedger, clock: Clock):
        self._queue = command_queue
        self.ledger = ledger
        self._clock = clock
        self.usable = True

    def submit(self, command: Command) -> OperationRecord:
        now = self._clock.now()
        self.ledger.record(command, now)
        envelope = CommandEnvelope(command.operation_id, command, now)
        try:
            self._queue.put_nowait(envelope)
        except Exception as exc:
            return self.ledger.transition(
                command.operation_id,
                OperationStatus.TRANSPORT_FAILED,
                self._clock.now(),
                producer="gateway",
                stage="ipc-submission",
                attempted_sends=0,
                detail=str(exc),
            )
        return self.ledger.transition(
            command.operation_id,
            OperationStatus.QUEUED,
            self._clock.now(),
            producer="gateway",
        )

    def cancel(self, operation_id: str, now: float | None = None) -> OperationRecord:
        return self.ledger.cancel(operation_id, self._clock.now() if now is None else now)
