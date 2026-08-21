"""Durable in-process operation history."""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from enum import Enum
from typing import Any

from src.commands.types import Command, CommandKind, command_kind, expects_ack


class DuplicateOperationError(RuntimeError):
    pass


class LedgerCapacityError(RuntimeError):
    pass


class OperationStatus(str, Enum):
    RECORDED = "recorded"
    QUEUED = "queued"
    DISPATCHED = "dispatched"
    ACKNOWLEDGED = "acknowledged"
    REJECTED = "rejected"
    DROPPED = "dropped"
    TRANSPORT_FAILED = "transport-failed"
    UNKNOWN = "unknown"


@dataclass(frozen=True)
class StatusChange:
    status: OperationStatus
    occurred_at: float
    metadata: dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class OperationRecord:
    operation_id: str
    kind: CommandKind
    command: Command
    created_at: float
    status: OperationStatus
    history: tuple[StatusChange, ...]
    cancelled: bool = False
    cancelled_at: float | None = None

    @property
    def terminal(self) -> bool:
        if self.status in {
            OperationStatus.ACKNOWLEDGED,
            OperationStatus.REJECTED,
            OperationStatus.DROPPED,
            OperationStatus.TRANSPORT_FAILED,
            OperationStatus.UNKNOWN,
        }:
            return True
        return self.status is OperationStatus.DISPATCHED and not expects_ack(self.command)


class OperationLedger:
    def __init__(self, capacity: int = 2048):
        if capacity < 16:
            raise ValueError("ledger capacity must be at least 16")
        self.capacity = capacity
        self._records: dict[str, OperationRecord] = {}
        self._seen_ids: set[str] = set()
        self.evictions = 0

    def record(self, command: Command, now: float) -> OperationRecord:
        operation_id = command.operation_id
        if operation_id in self._seen_ids:
            raise DuplicateOperationError(operation_id)
        self._make_room()
        change = StatusChange(OperationStatus.RECORDED, now)
        record = OperationRecord(
            operation_id,
            command_kind(command),
            command,
            now,
            OperationStatus.RECORDED,
            (change,),
        )
        self._records[operation_id] = record
        self._seen_ids.add(operation_id)
        return record

    def transition(
        self,
        operation_id: str,
        status: OperationStatus,
        now: float,
        **metadata: Any,
    ) -> OperationRecord:
        record = self._records[operation_id]
        if (
            record.terminal
            and status is OperationStatus.DISPATCHED
            and record.status in {OperationStatus.ACKNOWLEDGED, OperationStatus.REJECTED}
        ):
            change = StatusChange(
                status,
                now,
                {**metadata, "consumed_after_ack": True},
            )
            updated = replace(record, history=record.history + (change,))
            self._records[operation_id] = updated
            return updated
        if record.terminal and status is not record.status:
            raise ValueError(f"terminal operation {operation_id} cannot transition")
        if status in {OperationStatus.RECORDED, OperationStatus.QUEUED} and status is record.status:
            raise ValueError(f"duplicate status {status.value}")
        allowed = self._allowed(record, status)
        if not allowed:
            raise ValueError(f"illegal transition {record.status.value} -> {status.value}")
        change = StatusChange(status, now, metadata)
        updated = replace(record, status=status, history=record.history + (change,))
        self._records[operation_id] = updated
        return updated

    def cancel(self, operation_id: str, now: float) -> OperationRecord:
        record = self._records[operation_id]
        updated = replace(record, cancelled=True, cancelled_at=now)
        self._records[operation_id] = updated
        return updated

    def get(self, operation_id: str) -> OperationRecord | None:
        return self._records.get(operation_id)

    def snapshot(self) -> dict[str, OperationRecord]:
        return dict(self._records)

    def _allowed(self, record: OperationRecord, status: OperationStatus) -> bool:
        if status in {
            OperationStatus.DROPPED,
            OperationStatus.TRANSPORT_FAILED,
        }:
            return not record.terminal
        if status is OperationStatus.QUEUED:
            return record.status is OperationStatus.RECORDED
        if status is OperationStatus.DISPATCHED:
            return record.status is OperationStatus.QUEUED
        if status in {OperationStatus.ACKNOWLEDGED, OperationStatus.REJECTED}:
            return expects_ack(record.command) and record.status in {
                OperationStatus.QUEUED,
                OperationStatus.DISPATCHED,
            }
        if status is OperationStatus.UNKNOWN:
            return expects_ack(record.command) and record.status is OperationStatus.DISPATCHED
        return False

    def _make_room(self) -> None:
        if len(self._records) < self.capacity:
            return
        for operation_id, record in tuple(self._records.items()):
            if record.terminal:
                del self._records[operation_id]
                self.evictions += 1
                return
        raise LedgerCapacityError("ledger full of unresolved operations")
