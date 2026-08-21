"""Activity scope and continuity management."""

from __future__ import annotations

from src.activities.landing_target_relay import LandingTargetRelay
from src.mission.model import BroadState
from src.observations.model import CameraObservation
from src.observations.store import ObservationStore


RELAY_SCOPE = frozenset({BroadState.SEARCH, BroadState.PRECISION_LANDING})


class ActivityManager:
    def __init__(
        self,
        relay: LandingTargetRelay | None,
        *,
        production_relay_enabled: bool,
    ):
        self.relay = relay
        self.production_relay_enabled = production_relay_enabled
        self.phase: BroadState | None = None
        self.healthy = True

    def set_phase(self, phase: BroadState) -> None:
        was_enabled = self.phase in RELAY_SCOPE
        now_enabled = phase in RELAY_SCOPE
        self.phase = phase
        if self.relay is None or not self.production_relay_enabled:
            return
        if now_enabled and not was_enabled:
            self.relay.start()
        elif was_enabled and not now_enabled:
            self.relay.stop()

    def deliver(
        self,
        events: tuple[CameraObservation, ...],
        store: ObservationStore,
        now: float,
    ) -> None:
        if (
            self.relay is None
            or not self.production_relay_enabled
            or self.phase not in RELAY_SCOPE
        ):
            return
        try:
            for event in events:
                self.relay.handle(event, store, now)
        except Exception:
            self.healthy = False

    def stop_all(self) -> None:
        if self.relay is not None:
            self.relay.stop()
