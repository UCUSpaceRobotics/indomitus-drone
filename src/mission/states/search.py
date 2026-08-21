"""Fixed-route search with one frozen endpoint command per leg."""

from __future__ import annotations

from enum import Enum

from src.commands.ledger import OperationStatus
from src.commands.types import FlightMode, MoveToLocalNed
from src.mission.model import BroadState, MissionSnapshot
from src.mission.protocols import EntryContext, MissionParameters
from src.mission.results import AdvanceStep, FailureReason, Running, StateFailed, StateSucceeded
from src.mission.states.common import operation
from src.navigation.motion_tracker import MotionStage, MotionState, update_motion
from src.navigation.ned import LocalNed


class SearchStep(str, Enum):
    WAITING_FOR_FRESH_POSE = "WAITING_FOR_FRESH_POSE"
    MOVING_LEG = "MOVING_LEG"


class SearchState:
    phase = BroadState.SEARCH

    def __init__(self, parameters: MissionParameters):
        self.parameters = parameters
        self.step = SearchStep.WAITING_FOR_FRESH_POSE
        self.deadline = 0.0
        self.leg_index = 0
        self.start: LocalNed | None = None
        self.endpoint: LocalNed | None = None
        self.motion: MotionState | None = None
        self.active_operation_id: str | None = None

    def enter(self, ctx: EntryContext) -> None:
        self.step = SearchStep.WAITING_FOR_FRESH_POSE
        self.leg_index = 0
        self.endpoint = None
        self.motion = None
        self.active_operation_id = None

    def enter_step(self, step: SearchStep | None, ctx: EntryContext) -> None:
        self.step = step or SearchStep.WAITING_FOR_FRESH_POSE
        self.deadline = ctx.deadline
        if self.step is SearchStep.WAITING_FOR_FRESH_POSE:
            self.endpoint = None
            self.motion = None
            self.active_operation_id = None
            return
        if self.start is None or self.endpoint is None:
            raise RuntimeError("search endpoint must be resolved before movement entry")
        self.active_operation_id = (
            f"mission/{ctx.mission_id}/search/leg/{self.leg_index}"
        )
        self.motion = MotionState(self.start, self.endpoint, ctx.deadline)
        ctx.gateway.submit(
            MoveToLocalNed(
                self.active_operation_id,
                self.endpoint.north_m,
                self.endpoint.east_m,
                self.endpoint.down_m,
            )
        )

    def update(self, snapshot: MissionSnapshot):
        if self.step is SearchStep.WAITING_FOR_FRESH_POSE:
            if (
                snapshot.position_fresh
                and snapshot.position is not None
                and snapshot.yaw_fresh
                and snapshot.yaw_rad is not None
            ):
                self.start = snapshot.position
                self.endpoint = self.parameters.route.resolve(
                    self.leg_index, snapshot.position, snapshot.yaw_rad
                )
                return AdvanceStep(SearchStep.MOVING_LEG)
            if snapshot.now >= self.deadline:
                return StateFailed(
                    FailureReason.WAYPOINT_TIMEOUT, "WAITING_FOR_FRESH_POSE"
                )
            return Running("waiting for fresh pose and yaw")

        if self.motion is None:
            return StateFailed(FailureReason.INVALID_STATE, "missing motion tracker")
        record = operation(snapshot, self.active_operation_id)
        dispatched = record is not None and record.status in {
            OperationStatus.DISPATCHED,
            OperationStatus.ACKNOWLEDGED,
        }
        dispatch_failed = record is not None and record.status in {
            OperationStatus.REJECTED,
            OperationStatus.DROPPED,
            OperationStatus.TRANSPORT_FAILED,
            OperationStatus.UNKNOWN,
        }
        self.motion = update_motion(
            self.motion,
            now=snapshot.now,
            position=snapshot.position,
            position_fresh=snapshot.position_fresh,
            dispatched=dispatched,
            dispatch_failed=dispatch_failed,
            departure_threshold_m=self.parameters.departure_threshold_m,
            position_tolerance_m=self.parameters.position_tolerance_m,
            settle_dwell_s=self.parameters.settle_dwell_s,
        )
        if self.motion.stage is MotionStage.REACHED:
            if self.leg_index + 1 >= len(self.parameters.route):
                return StateSucceeded("route-exhausted")
            self.leg_index += 1
            return AdvanceStep(SearchStep.WAITING_FOR_FRESH_POSE)
        if self.motion.stage is MotionStage.FAILED:
            return StateFailed(
                FailureReason.WAYPOINT_TIMEOUT
                if self.motion.timed_out
                else FailureReason.OPERATION_FAILED,
                self.motion.stage.value,
            )
        return Running(self.motion.stage.value, self.motion)

    def exit(self, reason: str) -> None:
        pass

    def timeout_for(self, step: SearchStep | None) -> float:
        return self.parameters.waypoint_timeout_s

    def authorized_modes(self) -> frozenset[FlightMode] | None:
        return frozenset({FlightMode.GUIDED})
