"""Single authoritative broad-state transition policy."""

from __future__ import annotations

from src.mission.model import BroadState, MissionSnapshot


SUCCESS_DESTINATIONS = {
    BroadState.PREFLIGHT: BroadState.TAKEOFF,
    BroadState.TAKEOFF: BroadState.SEARCH,
    BroadState.SEARCH: BroadState.PRECISION_LANDING,
    BroadState.PRECISION_LANDING: BroadState.COMPLETED,
    BroadState.LAND_HERE: BroadState.FAULTED,
    BroadState.AIRBORNE_FAULT: BroadState.FAULTED,
}


def success_destination(phase: BroadState) -> BroadState:
    return SUCCESS_DESTINATIONS[phase]


def failure_destination(
    phase: BroadState, snapshot: MissionSnapshot, gateway_usable: bool
) -> BroadState:
    if snapshot.grounded_and_disarmed:
        return BroadState.FAULTED
    if phase in {BroadState.PRECISION_LANDING, BroadState.LAND_HERE}:
        return BroadState.AIRBORNE_FAULT
    if phase is BroadState.PREFLIGHT:
        return BroadState.FAULTED
    if not gateway_usable:
        return BroadState.AIRBORNE_FAULT
    return BroadState.LAND_HERE
