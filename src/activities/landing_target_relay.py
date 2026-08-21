"""Exact-once forwarding of validated marker-102 observations."""

from __future__ import annotations

import math
from collections import deque

from src.commands.gateway import CommandGateway
from src.commands.types import LandingTarget
from src.observations.model import CameraObservation, ObservationKey, TargetDownDistance
from src.observations.store import ObservationStore


class LandingTargetRelay:
    def __init__(
        self,
        gateway: CommandGateway,
        mission_id: str,
        *,
        camera_freshness_s: float,
        distance_freshness_s: float,
        delivered_capacity: int = 2048,
    ):
        if delivered_capacity < 16:
            raise ValueError("delivered ID capacity must be at least 16")
        self.gateway = gateway
        self.mission_id = mission_id
        self.camera_freshness_s = camera_freshness_s
        self.distance_freshness_s = distance_freshness_s
        self._delivered_order: deque[str] = deque()
        self._delivered: set[str] = set()
        self.enabled = False
        self.healthy = True
        self.forwarded_count = 0
        self.duplicate_count = 0
        self.stale_count = 0
        self.rejected_count = 0
        self._capacity = delivered_capacity

    def start(self) -> None:
        self.enabled = True

    def stop(self) -> None:
        self.enabled = False

    def handle(
        self,
        observation: CameraObservation,
        store: ObservationStore,
        now: float,
    ) -> bool:
        if not self.enabled or observation.marker_id != 102:
            return False
        if observation.observation_id in self._delivered:
            self.duplicate_count += 1
            return False
        if now - observation.received_at > self.camera_freshness_s:
            self.stale_count += 1
            return False
        distance_observation = store.fresh(
            ObservationKey.TARGET_DOWN_DISTANCE,
            now,
            self.distance_freshness_s,
        )
        if distance_observation is None:
            self.rejected_count += 1
            return False
        value = distance_observation.value
        down_m = value.down_m if isinstance(value, TargetDownDistance) else float(value)
        values = (observation.x_offset_m, observation.y_offset_m, down_m)
        if not all(math.isfinite(item) for item in values) or down_m <= 0:
            self.rejected_count += 1
            return False

        command = LandingTarget(
            operation_id=(
                f"mission/{self.mission_id}/landing-target/"
                f"{observation.observation_id}"
            ),
            forward_m=observation.y_offset_m,
            right_m=observation.x_offset_m,
            down_m=down_m,
            observation_id=observation.observation_id,
            observed_at=observation.observed_at,
        )
        self.gateway.submit(command)
        self._remember(observation.observation_id)
        self.forwarded_count += 1
        return True

    def _remember(self, observation_id: str) -> None:
        if len(self._delivered_order) >= self._capacity:
            removed = self._delivered_order.popleft()
            self._delivered.remove(removed)
        self._delivered_order.append(observation_id)
        self._delivered.add(observation_id)
