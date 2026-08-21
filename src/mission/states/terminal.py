"""Passive terminal and airborne-fault states."""

from __future__ import annotations

from src.mission.model import BroadState, MissionSnapshot
from src.mission.protocols import EntryContext
from src.mission.results import Running, StateSucceeded


class PassiveState:
    step = None
    active_operation_id = None

    def __init__(self, phase: BroadState):
        self.phase = phase

    def enter(self, ctx: EntryContext) -> None:
        pass

    def enter_step(self, step, ctx: EntryContext) -> None:
        pass

    def update(self, snapshot: MissionSnapshot):
        if self.phase is BroadState.AIRBORNE_FAULT and snapshot.grounded_and_disarmed:
            return StateSucceeded("grounded-after-airborne-fault")
        return Running("passive")

    def exit(self, reason: str) -> None:
        pass

    def timeout_for(self, step) -> float:
        return 0.0

    def authorized_modes(self):
        return None
