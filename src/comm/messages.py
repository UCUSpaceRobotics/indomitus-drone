"""Typed IPC messages shared with comm process."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Any

from src.commands.ledger import OperationStatus
from src.commands.types import Command


@dataclass(frozen=True)
class CommandEnvelope:
    operation_id: str
    command: Command
    created_at: float


@dataclass(frozen=True)
class CommResult:
    operation_id: str
    status: OperationStatus
    occurred_at: float
    detail: str = ""
    attempted_sends: int = 0


@dataclass(frozen=True)
class TelemetryPacket:
    values: dict[str, Any]
    published_at: float


class CommHealthKind(str, Enum):
    STARTING = "starting"
    HEALTHY = "healthy"
    FAILED = "failed"
    STOPPED = "stopped"


@dataclass(frozen=True)
class CommHealth:
    kind: CommHealthKind
    occurred_at: float
    detail: str = ""
