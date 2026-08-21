"""Bounded mission audit journal."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class JournalEntry:
    occurred_at: float
    kind: str
    detail: dict[str, Any]


class MissionJournal:
    def __init__(self, capacity: int):
        if capacity < 16:
            raise ValueError("journal capacity must be at least 16")
        self._entries: deque[JournalEntry] = deque(maxlen=capacity)
        self.evictions = 0

    def append(self, occurred_at: float, kind: str, **detail: Any) -> None:
        if len(self._entries) == self._entries.maxlen:
            self.evictions += 1
        self._entries.append(JournalEntry(occurred_at, kind, detail))

    def entries(self) -> tuple[JournalEntry, ...]:
        return tuple(self._entries)

    @property
    def capacity(self) -> int:
        return int(self._entries.maxlen or 0)
