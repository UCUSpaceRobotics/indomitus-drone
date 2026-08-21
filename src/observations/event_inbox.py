"""Bounded owner-addressed event inbox."""

from __future__ import annotations

from collections import deque

from src.observations.model import Event, EventType


class EventInbox:
    def __init__(self, capacity: int = 1024):
        if capacity < 16:
            raise ValueError("event capacity must be at least 16")
        self.capacity = capacity
        self._events: deque[Event] = deque()
        self.overflow_count = 0
        self._reported_overflow_count = 0

    def append(self, event: Event) -> None:
        if len(self._events) >= self.capacity:
            self._events.popleft()
            self.overflow_count += 1
        self._events.append(event)

    def consume(self, owner: str) -> tuple[Event, ...]:
        selected: list[Event] = []
        retained: deque[Event] = deque()
        while self._events:
            event = self._events.popleft()
            if event.owner == owner:
                selected.append(event)
            else:
                retained.append(event)
        self._events = retained
        if (
            owner == "coordinator"
            and self.overflow_count > self._reported_overflow_count
        ):
            selected.append(
                Event(
                    f"overflow/{self.overflow_count}",
                    EventType.OVERFLOW,
                    0.0,
                    self.overflow_count,
                    owner,
                )
            )
            self._reported_overflow_count = self.overflow_count
        return tuple(selected)

    def __len__(self) -> int:
        return len(self._events)
