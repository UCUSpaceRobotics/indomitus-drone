"""Pure waypoint departure, transit, settling, and dwell tracking."""

from __future__ import annotations

from dataclasses import dataclass, replace
from enum import Enum

from src.navigation.ned import LocalNed


class MotionStage(str, Enum):
    WAITING_FOR_DISPATCH = "WAITING_FOR_DISPATCH"
    WAITING_FOR_DEPARTURE = "WAITING_FOR_DEPARTURE"
    IN_TRANSIT = "IN_TRANSIT"
    SETTLING_AT_TARGET = "SETTLING_AT_TARGET"
    REACHED = "REACHED"
    FAILED = "FAILED"


@dataclass(frozen=True)
class MotionState:
    start: LocalNed
    endpoint: LocalNed
    deadline: float
    stage: MotionStage = MotionStage.WAITING_FOR_DISPATCH
    departed_at: float | None = None
    dwell_started_at: float | None = None
    closest_distance_m: float | None = None
    distance_m: float | None = None
    timed_out: bool = False


def update_motion(
    state: MotionState,
    *,
    now: float,
    position: LocalNed | None,
    position_fresh: bool,
    dispatched: bool,
    dispatch_failed: bool,
    departure_threshold_m: float,
    position_tolerance_m: float,
    settle_dwell_s: float,
) -> MotionState:
    if state.stage in {MotionStage.REACHED, MotionStage.FAILED}:
        return state
    if dispatch_failed:
        return replace(state, stage=MotionStage.FAILED)

    updated = state
    if state.stage is MotionStage.WAITING_FOR_DISPATCH and dispatched:
        updated = replace(state, stage=MotionStage.WAITING_FOR_DEPARTURE)

    if (
        dispatched
        and not position_fresh
        and updated.stage is MotionStage.SETTLING_AT_TARGET
    ):
        updated = replace(
            updated,
            stage=MotionStage.IN_TRANSIT,
            dwell_started_at=None,
        )

    if position_fresh and position is not None and dispatched:
        from_start = state.start.distance_to(position)
        to_target = state.endpoint.distance_to(position)
        closest = (
            to_target
            if updated.closest_distance_m is None
            else min(updated.closest_distance_m, to_target)
        )
        departed_at = updated.departed_at
        stage = updated.stage
        dwell_started_at = updated.dwell_started_at

        if departed_at is None and from_start >= departure_threshold_m:
            departed_at = now
            stage = MotionStage.IN_TRANSIT

        if departed_at is not None:
            if to_target <= position_tolerance_m:
                if stage is not MotionStage.SETTLING_AT_TARGET:
                    dwell_started_at = now
                stage = MotionStage.SETTLING_AT_TARGET
                if now - float(dwell_started_at) >= settle_dwell_s:
                    stage = MotionStage.REACHED
            else:
                stage = MotionStage.IN_TRANSIT
                dwell_started_at = None

        updated = replace(
            updated,
            stage=stage,
            departed_at=departed_at,
            dwell_started_at=dwell_started_at,
            closest_distance_m=closest,
            distance_m=to_target,
        )

    # Completion is evaluated before timeout at the exact boundary.
    if updated.stage is not MotionStage.REACHED and now >= state.deadline:
        return replace(updated, stage=MotionStage.FAILED, timed_out=True)
    return updated
