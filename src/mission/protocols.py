"""Mission state contracts and configuration."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Protocol

from src.commands.gateway import CommandGateway
from src.commands.types import FlightMode
from src.mission.model import BroadState, MissionSnapshot
from src.mission.results import StateResult
from src.navigation.search_route import SearchRoute


@dataclass(frozen=True)
class MissionParameters:
    startup_mode: FlightMode
    takeoff_altitude_m: float
    takeoff_reached_ratio: float
    route: SearchRoute
    departure_threshold_m: float
    position_tolerance_m: float
    settle_dwell_s: float
    connection_timeout_s: float
    ekf_timeout_s: float
    mode_change_timeout_s: float
    arm_timeout_s: float
    takeoff_timeout_s: float
    waypoint_timeout_s: float
    landing_timeout_s: float


@dataclass(frozen=True)
class EntryContext:
    mission_id: str
    now: float
    deadline: float
    gateway: CommandGateway
    snapshot: MissionSnapshot


class MissionState(Protocol):
    phase: BroadState
    step: Enum | None
    active_operation_id: str | None

    def enter(self, ctx: EntryContext) -> None: ...

    def enter_step(self, step: Enum | None, ctx: EntryContext) -> None: ...

    def update(self, snapshot: MissionSnapshot) -> StateResult: ...

    def exit(self, reason: str) -> None: ...

    def timeout_for(self, step: Enum | None) -> float: ...

    def authorized_modes(self) -> frozenset[FlightMode] | None: ...
