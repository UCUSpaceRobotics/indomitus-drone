"""Precision and controlled fallback landing states."""

from __future__ import annotations

from enum import Enum

from src.commands.types import FlightMode, LandHere, PrecisionLand
from src.mission.model import BroadState, MissionSnapshot
from src.mission.protocols import EntryContext, MissionParameters
from src.mission.results import AdvanceStep, FailureReason, Running, StateFailed, StateSucceeded
from src.mission.states.common import operation_failed


class LandingStep(str, Enum):
    LAND_COMMAND_PENDING = "LAND_COMMAND_PENDING"
    DESCENDING = "DESCENDING"


class _LandingState:
    def __init__(self, parameters: MissionParameters):
        self.parameters = parameters
        self.step = LandingStep.LAND_COMMAND_PENDING
        self.deadline = 0.0
        self.active_operation_id: str | None = None
        self.source_mode: FlightMode | None = None

    def enter(self, ctx: EntryContext) -> None:
        self.step = LandingStep.LAND_COMMAND_PENDING
        self.source_mode = ctx.snapshot.mode

    def enter_step(self, step: LandingStep | None, ctx: EntryContext) -> None:
        self.step = step or LandingStep.LAND_COMMAND_PENDING
        self.deadline = ctx.deadline
        if self.step is LandingStep.LAND_COMMAND_PENDING:
            self._submit(ctx)

    def update(self, snapshot: MissionSnapshot):
        if snapshot.grounded_and_disarmed:
            return StateSucceeded("grounded-and-disarmed")
        if operation_failed(snapshot, self.active_operation_id):
            return StateFailed(
                FailureReason.OPERATION_FAILED, str(self.active_operation_id)
            )
        if self.step is LandingStep.LAND_COMMAND_PENDING and (
            snapshot.heartbeat_fresh and snapshot.mode is FlightMode.LAND
        ):
            return AdvanceStep(LandingStep.DESCENDING)
        if snapshot.now >= self.deadline:
            return StateFailed(FailureReason.LANDING_TIMEOUT, self.step.value)
        return Running(f"waiting in {self.step.value}")

    def exit(self, reason: str) -> None:
        pass

    def timeout_for(self, step: LandingStep | None) -> float:
        return self.parameters.landing_timeout_s

    def authorized_modes(self) -> frozenset[FlightMode] | None:
        if self.step is LandingStep.DESCENDING:
            return frozenset({FlightMode.LAND})
        if self.source_mode is None:
            return None
        return frozenset(
            mode for mode in (self.source_mode, FlightMode.LAND) if mode is not None
        )

    def capture_source_mode(self, mode: FlightMode) -> None:
        if self.step is LandingStep.LAND_COMMAND_PENDING and self.source_mode is None:
            self.source_mode = mode

    def _submit(self, ctx: EntryContext) -> None:
        raise NotImplementedError


class PrecisionLandingState(_LandingState):
    phase = BroadState.PRECISION_LANDING

    def _submit(self, ctx: EntryContext) -> None:
        self.active_operation_id = f"mission/{ctx.mission_id}/precision-land"
        ctx.gateway.submit(PrecisionLand(self.active_operation_id))


class LandHereState(_LandingState):
    phase = BroadState.LAND_HERE

    def _submit(self, ctx: EntryContext) -> None:
        self.active_operation_id = f"mission/{ctx.mission_id}/land-here"
        ctx.gateway.submit(LandHere(self.active_operation_id))
