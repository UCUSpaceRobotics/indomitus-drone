"""Effect-free state evaluation results."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Any


class FailureReason(str, Enum):
    CONNECTION_TIMEOUT = "CONNECTION_TIMEOUT"
    EKF_TIMEOUT = "EKF_TIMEOUT"
    MODE_TIMEOUT = "MODE_TIMEOUT"
    ARM_TIMEOUT = "ARM_TIMEOUT"
    TAKEOFF_TIMEOUT = "TAKEOFF_TIMEOUT"
    WAYPOINT_TIMEOUT = "WAYPOINT_TIMEOUT"
    LANDING_TIMEOUT = "LANDING_TIMEOUT"
    OPERATION_FAILED = "OPERATION_FAILED"
    ACTIVITY_FAILED = "ACTIVITY_FAILED"
    COMM_FAILED = "COMM_FAILED"
    INVALID_STATE = "INVALID_STATE"


@dataclass(frozen=True)
class Running:
    reason: str
    progress: Any = None


@dataclass(frozen=True)
class AdvanceStep:
    next_step: Enum


@dataclass(frozen=True)
class StateSucceeded:
    outcome: str = "completed"


@dataclass(frozen=True)
class StateFailed:
    reason: FailureReason
    detail: str = ""


StateResult = Running | AdvanceStep | StateSucceeded | StateFailed
