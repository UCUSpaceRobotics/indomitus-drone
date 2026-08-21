"""Shared state evaluation helpers."""

from src.commands.ledger import OperationRecord, OperationStatus
from src.mission.model import MissionSnapshot


FAILED_OPERATION_STATUSES = frozenset(
    {
        OperationStatus.REJECTED,
        OperationStatus.DROPPED,
        OperationStatus.TRANSPORT_FAILED,
        OperationStatus.UNKNOWN,
    }
)


def operation(snapshot: MissionSnapshot, operation_id: str | None) -> OperationRecord | None:
    if operation_id is None:
        return None
    return snapshot.operations.get(operation_id)


def operation_failed(snapshot: MissionSnapshot, operation_id: str | None) -> bool:
    record = operation(snapshot, operation_id)
    return record is not None and record.status in FAILED_OPERATION_STATUSES
