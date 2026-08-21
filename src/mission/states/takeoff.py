"""Typed staged takeoff lifecycle."""

from __future__ import annotations

from enum import Enum

from src.commands.types import Arm, FlightMode, SetMode, Takeoff
from src.mission.model import BroadState, MissionSnapshot
from src.mission.protocols import EntryContext, MissionParameters
from src.mission.results import AdvanceStep, FailureReason, Running, StateFailed, StateSucceeded
from src.mission.states.common import operation_failed


class TakeoffStep(str, Enum):
    SETTING_LOITER = "SETTING_LOITER"
    ARMING = "ARMING"
    SETTING_GUIDED = "SETTING_GUIDED"
    ASCENDING = "ASCENDING"


class TakeoffState:
    phase = BroadState.TAKEOFF

    def __init__(self, parameters: MissionParameters):
        self.parameters = parameters
        self.step = TakeoffStep.SETTING_LOITER
        self.deadline = 0.0
        self.active_operation_id: str | None = None
        self.source_mode: FlightMode | None = None
        self.start_down_m: float | None = None
        self.ascent_observed = False

    def enter(self, ctx: EntryContext) -> None:
        self.step = TakeoffStep.SETTING_LOITER
        self.source_mode = ctx.snapshot.mode

    def enter_step(self, step: TakeoffStep | None, ctx: EntryContext) -> None:
        self.step = step or TakeoffStep.SETTING_LOITER
        self.deadline = ctx.deadline
        suffix = {
            TakeoffStep.SETTING_LOITER: "set-loiter",
            TakeoffStep.ARMING: "arm",
            TakeoffStep.SETTING_GUIDED: "set-guided",
            TakeoffStep.ASCENDING: "ascend",
        }[self.step]
        self.active_operation_id = f"mission/{ctx.mission_id}/takeoff/{suffix}"
        if self.step is TakeoffStep.SETTING_LOITER:
            ctx.gateway.submit(SetMode(self.active_operation_id, FlightMode.LOITER))
        elif self.step is TakeoffStep.ARMING:
            ctx.gateway.submit(Arm(self.active_operation_id))
        elif self.step is TakeoffStep.SETTING_GUIDED:
            ctx.gateway.submit(SetMode(self.active_operation_id, FlightMode.GUIDED))
        else:
            self.start_down_m = (
                ctx.snapshot.position.down_m if ctx.snapshot.position is not None else None
            )
            self.ascent_observed = False
            ctx.gateway.submit(
                Takeoff(self.active_operation_id, self.parameters.takeoff_altitude_m)
            )

    def update(self, snapshot: MissionSnapshot):
        if operation_failed(snapshot, self.active_operation_id):
            return StateFailed(
                FailureReason.OPERATION_FAILED, str(self.active_operation_id)
            )

        if self.step is TakeoffStep.SETTING_LOITER and (
            snapshot.heartbeat_fresh and snapshot.mode is FlightMode.LOITER
        ):
            return AdvanceStep(TakeoffStep.ARMING)
        if self.step is TakeoffStep.ARMING and (
            snapshot.heartbeat_fresh and snapshot.armed is True
        ):
            return AdvanceStep(TakeoffStep.SETTING_GUIDED)
        if self.step is TakeoffStep.SETTING_GUIDED and (
            snapshot.heartbeat_fresh and snapshot.mode is FlightMode.GUIDED
        ):
            return AdvanceStep(TakeoffStep.ASCENDING)
        if self.step is TakeoffStep.ASCENDING:
            if snapshot.position_fresh and snapshot.position is not None:
                altitude = -snapshot.position.down_m
                if (
                    self.start_down_m is not None
                    and snapshot.position.down_m
                    <= self.start_down_m - self.parameters.departure_threshold_m
                ):
                    self.ascent_observed = True
                if (
                    self.ascent_observed
                    and altitude
                    >= self.parameters.takeoff_altitude_m
                    * self.parameters.takeoff_reached_ratio
                ):
                    return StateSucceeded("takeoff-altitude-reached")

        if snapshot.now >= self.deadline:
            reason = {
                TakeoffStep.SETTING_LOITER: FailureReason.MODE_TIMEOUT,
                TakeoffStep.ARMING: FailureReason.ARM_TIMEOUT,
                TakeoffStep.SETTING_GUIDED: FailureReason.MODE_TIMEOUT,
                TakeoffStep.ASCENDING: FailureReason.TAKEOFF_TIMEOUT,
            }[self.step]
            return StateFailed(reason, self.step.value)
        return Running(f"waiting in {self.step.value}")

    def exit(self, reason: str) -> None:
        pass

    def timeout_for(self, step: TakeoffStep | None) -> float:
        return {
            TakeoffStep.SETTING_LOITER: self.parameters.mode_change_timeout_s,
            TakeoffStep.ARMING: self.parameters.arm_timeout_s,
            TakeoffStep.SETTING_GUIDED: self.parameters.mode_change_timeout_s,
            TakeoffStep.ASCENDING: self.parameters.takeoff_timeout_s,
        }[step or TakeoffStep.SETTING_LOITER]

    def authorized_modes(self) -> frozenset[FlightMode] | None:
        if self.step is TakeoffStep.SETTING_LOITER:
            return frozenset(
                mode for mode in (self.source_mode, FlightMode.LOITER) if mode is not None
            )
        if self.step is TakeoffStep.ARMING:
            return frozenset({FlightMode.LOITER})
        if self.step is TakeoffStep.SETTING_GUIDED:
            return frozenset({FlightMode.LOITER, FlightMode.GUIDED})
        return frozenset({FlightMode.GUIDED})
