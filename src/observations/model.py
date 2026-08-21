"""Observation and event values."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Any


class ObservationKey(str, Enum):
    HEARTBEAT = "heartbeat"
    LOCAL_POSITION = "local-position"
    ATTITUDE = "attitude"
    EKF = "ekf"
    LANDED_STATE = "landed-state"
    TARGET_DOWN_DISTANCE = "target-down-distance"
    CAMERA = "camera"
    COMM_HEALTH = "comm-health"


class LandedState(str, Enum):
    UNDEFINED = "undefined"
    ON_GROUND = "on-ground"
    IN_AIR = "in-air"
    TAKEOFF = "takeoff"
    LANDING = "landing"


@dataclass(frozen=True)
class Observation:
    value: Any
    received_at: float
    source_timestamp: float | None = None
    sequence: str | None = None


@dataclass(frozen=True)
class CameraObservation:
    observation_id: str
    marker_id: int
    x_offset_m: float
    y_offset_m: float
    observed_at: float
    received_at: float


@dataclass(frozen=True)
class TargetDownDistance:
    down_m: float
    received_at: float


class EventType(str, Enum):
    OPERATION_RESULT = "operation-result"
    CAMERA = "camera"
    COMM_HEALTH = "comm-health"
    CONTROL = "control"
    ACTIVITY_FAULT = "activity-fault"
    OVERFLOW = "overflow"


@dataclass(frozen=True)
class Event:
    event_id: str
    event_type: EventType
    occurred_at: float
    payload: Any
    owner: str
