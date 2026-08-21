"""Latest-value observation storage with caller-owned freshness time."""

from __future__ import annotations

from src.observations.model import Observation, ObservationKey


class ObservationStore:
    def __init__(self):
        self._values: dict[ObservationKey, Observation] = {}

    def put(
        self,
        key: ObservationKey,
        value,
        received_at: float,
        *,
        source_timestamp: float | None = None,
        sequence: str | None = None,
    ) -> Observation:
        previous = self._values.get(key)
        if previous is not None and received_at < previous.received_at:
            raise ValueError(f"observation time regressed for {key.value}")
        observation = Observation(value, received_at, source_timestamp, sequence)
        self._values[key] = observation
        return observation

    def latest(self, key: ObservationKey) -> Observation | None:
        return self._values.get(key)

    def fresh(
        self, key: ObservationKey, now: float, max_age_s: float
    ) -> Observation | None:
        observation = self.latest(key)
        if observation is None:
            return None
        age = now - observation.received_at
        if age < 0:
            raise ValueError("now precedes observation receive time")
        return observation if age <= max_age_s else None
