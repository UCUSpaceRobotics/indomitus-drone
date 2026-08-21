"""Typed one-shot mission commands."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import TypeAlias


class FlightMode(str, Enum):
    LOITER = "LOITER"
    GUIDED = "GUIDED"
    LAND = "LAND"
    STABILIZE = "STABILIZE"
    ALT_HOLD = "ALT_HOLD"
    POSHOLD = "POSHOLD"
    RTL = "RTL"
    MANUAL = "MANUAL"
    UNKNOWN = "UNKNOWN"


class CommandKind(str, Enum):
    SET_MODE = "set-mode"
    ARM = "arm"
    TAKEOFF = "takeoff"
    MOVE_TO_LOCAL_NED = "move-to-local-ned"
    PRECISION_LAND = "precision-land"
    LAND_HERE = "land-here"
    LANDING_TARGET = "landing-target"


def _operation_id(value: str) -> None:
    if not value or value.strip() != value or " " in value:
        raise ValueError("operation_id must be a non-empty, whitespace-free string")


def _finite(*values: float) -> None:
    if not all(math.isfinite(value) for value in values):
        raise ValueError("command values must be finite")


@dataclass(frozen=True)
class SetMode:
    operation_id: str
    mode: FlightMode

    def __post_init__(self) -> None:
        _operation_id(self.operation_id)
        if self.mode not in {FlightMode.LOITER, FlightMode.GUIDED}:
            raise ValueError("SetMode supports only LOITER or GUIDED")


@dataclass(frozen=True)
class Arm:
    operation_id: str

    def __post_init__(self) -> None:
        _operation_id(self.operation_id)


@dataclass(frozen=True)
class Takeoff:
    operation_id: str
    altitude_m: float

    def __post_init__(self) -> None:
        _operation_id(self.operation_id)
        _finite(self.altitude_m)
        if self.altitude_m <= 0:
            raise ValueError("takeoff altitude must be positive")


@dataclass(frozen=True)
class MoveToLocalNed:
    operation_id: str
    north_m: float
    east_m: float
    down_m: float

    def __post_init__(self) -> None:
        _operation_id(self.operation_id)
        _finite(self.north_m, self.east_m, self.down_m)


@dataclass(frozen=True)
class PrecisionLand:
    operation_id: str

    def __post_init__(self) -> None:
        _operation_id(self.operation_id)


@dataclass(frozen=True)
class LandHere:
    operation_id: str

    def __post_init__(self) -> None:
        _operation_id(self.operation_id)


@dataclass(frozen=True)
class LandingTarget:
    operation_id: str
    forward_m: float
    right_m: float
    down_m: float
    observation_id: str
    observed_at: float

    def __post_init__(self) -> None:
        _operation_id(self.operation_id)
        _finite(self.forward_m, self.right_m, self.down_m, self.observed_at)
        if self.down_m <= 0:
            raise ValueError("BODY_FRD down distance must be positive")
        if not self.observation_id:
            raise ValueError("observation_id is required")


Command: TypeAlias = (
    SetMode | Arm | Takeoff | MoveToLocalNed | PrecisionLand | LandHere | LandingTarget
)


def command_kind(command: Command) -> CommandKind:
    mapping = {
        SetMode: CommandKind.SET_MODE,
        Arm: CommandKind.ARM,
        Takeoff: CommandKind.TAKEOFF,
        MoveToLocalNed: CommandKind.MOVE_TO_LOCAL_NED,
        PrecisionLand: CommandKind.PRECISION_LAND,
        LandHere: CommandKind.LAND_HERE,
        LandingTarget: CommandKind.LANDING_TARGET,
    }
    return mapping[type(command)]


def expects_ack(command: Command) -> bool:
    return isinstance(command, (Arm, Takeoff, PrecisionLand, LandHere))
