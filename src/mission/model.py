"""Mission lifecycle values and immutable state snapshots."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Mapping

from src.commands.ledger import OperationRecord
from src.commands.types import FlightMode
from src.navigation.ned import LocalNed
from src.observations.model import LandedState


class BroadState(str, Enum):
    PREFLIGHT = "Preflight"
    TAKEOFF = "Takeoff"
    SEARCH = "Search"
    PRECISION_LANDING = "PrecisionLanding"
    LAND_HERE = "LandHere"
    COMPLETED = "Completed"
    FAULTED = "Faulted"
    YIELDED = "Yielded"
    AIRBORNE_FAULT = "AirborneFault"


TERMINAL_STATES = frozenset(
    {BroadState.COMPLETED, BroadState.FAULTED, BroadState.YIELDED}
)
PASSIVE_STATES = TERMINAL_STATES | {BroadState.AIRBORNE_FAULT}


@dataclass(frozen=True)
class MissionSnapshot:
    now: float
    comm_healthy: bool
    heartbeat_fresh: bool
    mode: FlightMode | None
    armed: bool | None
    ekf_fresh: bool
    ekf_healthy: bool | None
    position_fresh: bool
    position: LocalNed | None
    yaw_fresh: bool
    yaw_rad: float | None
    landed_fresh: bool
    landed_state: LandedState | None
    operations: Mapping[str, OperationRecord]
    activity_healthy: bool = True
    control_requested: bool = False

    @property
    def grounded_and_disarmed(self) -> bool:
        return (
            self.landed_fresh
            and self.landed_state is LandedState.ON_GROUND
            and self.heartbeat_fresh
            and self.armed is False
        )


@dataclass(frozen=True)
class MissionStatus:
    mission_id: str
    phase: BroadState
    step: str | None
    entered_at: float
    waiting_reason: str = ""
    failure_reason: str | None = None
    active_operation_id: str | None = None
    terminal: bool = False
