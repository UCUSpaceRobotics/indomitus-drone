import pytest

from src.commands.gateway import CommandGateway
from src.commands.ledger import DuplicateOperationError, OperationLedger, OperationStatus
from src.commands.types import Arm
from tests.fakes.clock import FakeClock
from tests.fakes.queues import SpyQueue


def test_gateway_records_then_submits_once_and_rejects_duplicate():
    clock = FakeClock(10.0)
    queue = SpyQueue()
    ledger = OperationLedger(16)
    gateway = CommandGateway(queue, ledger, clock)

    record = gateway.submit(Arm("mission/m/arm"))

    assert queue.calls == 1
    assert record.status is OperationStatus.QUEUED
    assert [change.status for change in record.history] == [
        OperationStatus.RECORDED,
        OperationStatus.QUEUED,
    ]
    with pytest.raises(DuplicateOperationError):
        gateway.submit(Arm("mission/m/arm"))
    assert queue.calls == 1


def test_gateway_queue_failure_is_terminal_zero_send():
    queue = SpyQueue(RuntimeError("full"))
    ledger = OperationLedger(16)
    gateway = CommandGateway(queue, ledger, FakeClock())

    record = gateway.submit(Arm("mission/m/arm"))

    assert queue.calls == 1
    assert record.status is OperationStatus.TRANSPORT_FAILED
    assert record.history[-1].metadata["attempted_sends"] == 0
