import pytest

from src.commands.ledger import OperationLedger, OperationStatus
from src.commands.types import Arm, MoveToLocalNed


def test_send_only_dispatch_is_terminal_but_ack_command_is_not():
    ledger = OperationLedger(16)
    ledger.record(MoveToLocalNed("move", 1, 2, -3), 0)
    ledger.transition("move", OperationStatus.QUEUED, 0)
    move = ledger.transition("move", OperationStatus.DISPATCHED, 1)
    assert move.terminal
    with pytest.raises(ValueError):
        ledger.transition("move", OperationStatus.UNKNOWN, 2)

    ledger.record(Arm("arm"), 0)
    ledger.transition("arm", OperationStatus.QUEUED, 0)
    arm = ledger.transition("arm", OperationStatus.DISPATCHED, 1)
    assert not arm.terminal
    assert ledger.transition("arm", OperationStatus.UNKNOWN, 4).terminal


def test_cancellation_does_not_block_late_result_history():
    ledger = OperationLedger(16)
    ledger.record(Arm("arm"), 0)
    ledger.transition("arm", OperationStatus.QUEUED, 0)
    ledger.cancel("arm", 0.5)
    ledger.transition("arm", OperationStatus.DISPATCHED, 1)
    record = ledger.transition("arm", OperationStatus.ACKNOWLEDGED, 2)
    assert record.cancelled
    assert record.status is OperationStatus.ACKNOWLEDGED


def test_eviction_does_not_allow_semantic_id_reuse():
    ledger = OperationLedger(16)
    for index in range(16):
        operation_id = f"move-{index}"
        ledger.record(MoveToLocalNed(operation_id, 1, 0, 0), 0)
        ledger.transition(operation_id, OperationStatus.QUEUED, 0)
        ledger.transition(operation_id, OperationStatus.DISPATCHED, 1)
    ledger.record(MoveToLocalNed("new", 1, 0, 0), 2)
    with pytest.raises(Exception):
        ledger.record(MoveToLocalNed("move-0", 1, 0, 0), 3)


def test_ack_consumed_before_dispatch_result_preserves_both_evidence():
    ledger = OperationLedger(16)
    ledger.record(Arm("arm"), 0)
    ledger.transition("arm", OperationStatus.QUEUED, 0)
    ledger.transition("arm", OperationStatus.ACKNOWLEDGED, 2)
    record = ledger.transition("arm", OperationStatus.DISPATCHED, 1)
    assert record.status is OperationStatus.ACKNOWLEDGED
    assert record.history[-1].status is OperationStatus.DISPATCHED
    assert record.history[-1].metadata["consumed_after_ack"]
