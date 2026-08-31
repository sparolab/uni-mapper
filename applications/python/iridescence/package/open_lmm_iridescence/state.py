from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from enum import Enum
import threading
from typing import Any


def enum_name(value: object | None, fallback: str = "-") -> str:
    if value is None:
        return fallback
    return str(getattr(value, "name", value))


class EventBuffer:
    """Bounded callback-to-frame queue with progress coalescing and gap detection."""

    def __init__(self, capacity: int = 1024) -> None:
        if capacity < 1:
            raise ValueError("capacity must be positive")
        self._capacity = capacity
        self._events: deque[Any] = deque()
        self._lock = threading.Lock()
        self._last_sequence = 0
        self._resync_required = False
        self._evictions = 0

    def push(self, event: Any) -> None:
        sequence = int(getattr(event, "sequence", 0))
        name = enum_name(getattr(event, "type", None)).upper()
        with self._lock:
            if self._last_sequence and sequence != self._last_sequence + 1:
                self._resync_required = True
            self._last_sequence = max(self._last_sequence, sequence)
            if name == "PROGRESS_UPDATED" and self._events:
                previous = self._events[-1]
                if (
                    enum_name(getattr(previous, "type", None)).upper()
                    == "PROGRESS_UPDATED"
                    and getattr(previous, "job_id", None)
                    == getattr(event, "job_id", None)
                    and getattr(previous, "agent", None)
                    == getattr(event, "agent", None)
                ):
                    self._events[-1] = event
                    return
            if len(self._events) == self._capacity:
                self._events.popleft()
                self._evictions += 1
                self._resync_required = True
            self._events.append(event)

    def drain(self, limit: int = 128) -> tuple[Any, ...]:
        if limit < 1:
            raise ValueError("limit must be positive")
        with self._lock:
            result = tuple(self._events.popleft() for _ in range(min(limit, len(self._events))))
        return result

    def consume_resync(self) -> bool:
        with self._lock:
            result = self._resync_required
            self._resync_required = False
            return result

    @property
    def diagnostics(self) -> tuple[int, int, bool]:
        with self._lock:
            return len(self._events), self._evictions, self._resync_required


class PresentationPhase(Enum):
    VISIBLE = "visible"
    PENDING = "pending"
    READY = "ready"


@dataclass(frozen=True, slots=True)
class PresentationToken:
    epoch: int
    agent: str
    source_revision: int
    generation: int


@dataclass(slots=True)
class PresentationMetadata:
    token: PresentationToken
    displayed_point_count: int
    source_point_count: int
    min_bound: tuple[float, float, float] | None
    max_bound: tuple[float, float, float] | None
    visible: bool = True


class PresentationState:
    """Metadata-only owner for visible/pending presentation generations."""

    def __init__(self) -> None:
        self._epoch = 0
        self._generation = 0
        self._requested: dict[str, PresentationToken] = {}
        self._visible: dict[str, PresentationMetadata] = {}
        self._visibility: dict[str, bool] = {}

    def reset_epoch(self, epoch: int, agents: tuple[str, ...]) -> tuple[str, ...]:
        authoritative = set(agents)
        removed = tuple(agent for agent in self._visible if agent not in authoritative)
        if epoch != self._epoch:
            self._epoch = epoch
            self._requested.clear()
            removed = tuple(self._visible)
            self._visible.clear()
            self._visibility = {agent: True for agent in agents}
        else:
            for agent in removed:
                self._visible.pop(agent, None)
                self._requested.pop(agent, None)
                self._visibility.pop(agent, None)
            for agent in agents:
                self._visibility.setdefault(agent, True)
        return removed

    def request(self, agent: str, source_revision: int) -> PresentationToken:
        self._generation += 1
        token = PresentationToken(self._epoch, agent, source_revision, self._generation)
        self._requested[agent] = token
        return token

    def accepts(self, token: PresentationToken) -> bool:
        current = self._requested.get(token.agent)
        visible = self._visible.get(token.agent)
        return (
            token.epoch == self._epoch
            and current is not None
            and current.epoch == token.epoch
            and current.generation == token.generation
            and (visible is None or token.source_revision >= visible.token.source_revision)
        )

    def commit(self, metadata: PresentationMetadata) -> bool:
        if not self.accepts(metadata.token):
            return False
        metadata.visible = self._visibility.get(metadata.token.agent, True)
        self._visible[metadata.token.agent] = metadata
        return True

    def fail(self, token: PresentationToken) -> None:
        current = self._requested.get(token.agent)
        if current is not None and current.generation == token.generation:
            self._requested.pop(token.agent, None)

    def set_visible(self, agent: str, visible: bool) -> bool:
        if agent not in self._visibility:
            return False
        self._visibility[agent] = visible
        metadata = self._visible.get(agent)
        if metadata is not None:
            metadata.visible = visible
        return True

    def visible(self, agent: str) -> bool:
        return self._visibility.get(agent, True)

    @property
    def metadata(self) -> tuple[PresentationMetadata, ...]:
        return tuple(self._visible.values())
