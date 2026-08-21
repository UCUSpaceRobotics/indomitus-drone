"""Central lifecycle ownership, ordering, and transition budget."""

from __future__ import annotations

from src.activities.manager import ActivityManager
from src.commands.gateway import CommandGateway
from src.commands.types import FlightMode
from src.mission.journal import MissionJournal
from src.mission.model import (
    PASSIVE_STATES,
    TERMINAL_STATES,
    BroadState,
    MissionSnapshot,
    MissionStatus,
)
from src.mission.protocols import EntryContext, MissionParameters, MissionState
from src.mission.results import AdvanceStep, Running, StateFailed, StateSucceeded
from src.mission.states import (
    LandHereState,
    PassiveState,
    PrecisionLandingState,
    PreflightState,
    SearchState,
    TakeoffState,
)
from src.mission.transitions import failure_destination, success_destination
from src.observations.model import CameraObservation
from src.observations.store import ObservationStore


class MissionCoordinator:
    def __init__(
        self,
        mission_id: str,
        parameters: MissionParameters,
        gateway: CommandGateway,
        activities: ActivityManager,
        store: ObservationStore,
        journal_capacity: int,
        initial_snapshot: MissionSnapshot,
    ):
        self.mission_id = mission_id
        self.parameters = parameters
        self.gateway = gateway
        self.activities = activities
        self.store = store
        self.journal = MissionJournal(journal_capacity)
        self.state: MissionState = PreflightState(parameters)
        self.phase_entered_at = initial_snapshot.now
        self.waiting_reason = ""
        self.failure_reason: str | None = None
        self._enter_state(self.state, initial_snapshot, "mission-start")

    @property
    def phase(self) -> BroadState:
        return self.state.phase

    def update(
        self,
        snapshot: MissionSnapshot,
        camera_events: tuple[CameraObservation, ...] = (),
    ) -> MissionStatus:
        # Highest priority: parent control request or unexpected fresh mode.
        if self.phase not in PASSIVE_STATES and (
            snapshot.control_requested or self._unexpected_mode(snapshot)
        ):
            active_id = self.state.active_operation_id
            if active_id and self.gateway.ledger.get(active_id) is not None:
                self.gateway.cancel(active_id, snapshot.now)
            self.activities.stop_all()
            self._transition(BroadState.YIELDED, snapshot, "manual-preemption")
            return self.status(snapshot.now)

        if self.phase not in PASSIVE_STATES and not self.gateway.usable:
            self._apply_failure(snapshot, "gateway-unavailable")
            return self.status(snapshot.now)

        self.activities.deliver(camera_events, self.store, snapshot.now)
        if (
            (not self.activities.healthy or not snapshot.activity_healthy)
            and self.phase not in PASSIVE_STATES
        ):
            self._apply_failure(snapshot, "activity-failed")
            return self.status(snapshot.now)

        result = self.state.update(snapshot)
        if isinstance(result, Running):
            self.waiting_reason = result.reason
        elif isinstance(result, AdvanceStep):
            self._advance_step(result.next_step, snapshot)
        elif isinstance(result, StateSucceeded):
            destination = success_destination(self.phase)
            self._transition(destination, snapshot, result.outcome)
        elif isinstance(result, StateFailed):
            self.failure_reason = result.reason.value
            self._apply_failure(snapshot, f"{result.reason.value}:{result.detail}")
        return self.status(snapshot.now)

    def status(self, now: float) -> MissionStatus:
        return MissionStatus(
            mission_id=self.mission_id,
            phase=self.phase,
            step=self.state.step.value if self.state.step is not None else None,
            entered_at=self.phase_entered_at,
            waiting_reason=self.waiting_reason,
            failure_reason=self.failure_reason,
            active_operation_id=self.state.active_operation_id,
            terminal=self.phase in TERMINAL_STATES,
        )

    def _advance_step(self, step, snapshot: MissionSnapshot) -> None:
        deadline = snapshot.now + self.state.timeout_for(step)
        ctx = EntryContext(
            self.mission_id,
            snapshot.now,
            deadline,
            self.gateway,
            snapshot,
        )
        self.state.enter_step(step, ctx)
        self.waiting_reason = ""
        self.journal.append(
            snapshot.now,
            "step-entered",
            phase=self.phase.value,
            step=step.value,
        )

    def _transition(
        self, destination: BroadState, snapshot: MissionSnapshot, reason: str
    ) -> None:
        old = self.phase
        self.state.exit(reason)
        new_state = self._make_state(destination)
        self._enter_state(new_state, snapshot, reason)
        self.journal.append(
            snapshot.now,
            "state-transition",
            source=old.value,
            destination=destination.value,
            reason=reason,
        )

    def _enter_state(
        self, state: MissionState, snapshot: MissionSnapshot, reason: str
    ) -> None:
        self.state = state
        self.phase_entered_at = snapshot.now
        self.activities.set_phase(state.phase)
        initial_step = state.step
        deadline = snapshot.now + state.timeout_for(initial_step)
        ctx = EntryContext(
            self.mission_id,
            snapshot.now,
            deadline,
            self.gateway,
            snapshot,
        )
        state.enter(ctx)
        state.enter_step(state.step, ctx)
        self.waiting_reason = ""
        self.journal.append(
            snapshot.now,
            "state-entered",
            phase=state.phase.value,
            reason=reason,
        )

    def _apply_failure(self, snapshot: MissionSnapshot, reason: str) -> None:
        destination = failure_destination(self.phase, snapshot, self.gateway.usable)
        self._transition(destination, snapshot, reason)

    def _unexpected_mode(self, snapshot: MissionSnapshot) -> bool:
        if not snapshot.heartbeat_fresh or snapshot.mode is None:
            return False
        authorized = self.state.authorized_modes()
        capture_source_mode = getattr(self.state, "capture_source_mode", None)
        if authorized is None and capture_source_mode is not None:
            capture_source_mode(snapshot.mode)
            return False
        return authorized is not None and snapshot.mode not in authorized

    def _make_state(self, phase: BroadState) -> MissionState:
        if phase is BroadState.PREFLIGHT:
            return PreflightState(self.parameters)
        if phase is BroadState.TAKEOFF:
            return TakeoffState(self.parameters)
        if phase is BroadState.SEARCH:
            return SearchState(self.parameters)
        if phase is BroadState.PRECISION_LANDING:
            return PrecisionLandingState(self.parameters)
        if phase is BroadState.LAND_HERE:
            return LandHereState(self.parameters)
        return PassiveState(phase)
