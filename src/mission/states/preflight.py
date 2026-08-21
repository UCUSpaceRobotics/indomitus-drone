"""Preflight readiness state."""

from __future__ import annotations

from enum import Enum

from src.commands.types import FlightMode
from src.mission.model import BroadState, MissionSnapshot
from src.mission.protocols import EntryContext, MissionParameters
from src.mission.results import AdvanceStep, FailureReason, Running, StateFailed, StateSucceeded


class PreflightStep(str, Enum):
    WAITING_FOR_CONNECTION = "WAITING_FOR_CONNECTION"
    WAITING_FOR_EKF = "WAITING_FOR_EKF"


class PreflightState:
    phase = BroadState.PREFLIGHT
    active_operation_id = None

    def __init__(self, parameters: MissionParameters):
        self.parameters = parameters
        self.step = PreflightStep.WAITING_FOR_CONNECTION
        self.deadline = 0.0

    def enter(self, ctx: EntryContext) -> None:
        self.step = PreflightStep.WAITING_FOR_CONNECTION

    def enter_step(self, step: PreflightStep | None, ctx: EntryContext) -> None:
        self.step = step or PreflightStep.WAITING_FOR_CONNECTION
        self.deadline = ctx.deadline

    def update(self, snapshot: MissionSnapshot):
        if self.step is PreflightStep.WAITING_FOR_CONNECTION:
            if snapshot.comm_healthy and snapshot.heartbeat_fresh:
                return AdvanceStep(PreflightStep.WAITING_FOR_EKF)
            if snapshot.now >= self.deadline:
                return StateFailed(
                    FailureReason.CONNECTION_TIMEOUT, "fresh heartbeat not observed"
                )
            return Running("waiting for healthy comm and fresh heartbeat")

        ready = (
            snapshot.heartbeat_fresh
            and snapshot.mode is self.parameters.startup_mode
            and snapshot.ekf_fresh
            and snapshot.ekf_healthy is True
            and snapshot.position_fresh
            and snapshot.position is not None
            and snapshot.yaw_fresh
            and snapshot.yaw_rad is not None
        )
        if ready:
            return StateSucceeded("preflight-ready")
        if snapshot.now >= self.deadline:
            return StateFailed(FailureReason.EKF_TIMEOUT, "preflight evidence incomplete")
        return Running("waiting for startup mode, EKF, pose, and yaw")

    def exit(self, reason: str) -> None:
        pass

    def timeout_for(self, step: PreflightStep | None) -> float:
        if step is PreflightStep.WAITING_FOR_CONNECTION:
            return self.parameters.connection_timeout_s
        return self.parameters.ekf_timeout_s

    def authorized_modes(self) -> frozenset[FlightMode] | None:
        if self.step is PreflightStep.WAITING_FOR_CONNECTION:
            return None
        return frozenset({self.parameters.startup_mode})
