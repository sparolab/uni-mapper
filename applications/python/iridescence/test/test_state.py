from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import unittest

from open_lmm_iridescence.state import (
    EventBuffer,
    PresentationMetadata,
    PresentationState,
)


class Kind(Enum):
    PROGRESS_UPDATED = 1
    JOB_COMPLETED = 2


@dataclass
class Event:
    sequence: int
    type: Kind
    job_id: int = 1
    agent: str | None = None


class StateTests(unittest.TestCase):
    def test_event_queue_coalesces_progress_and_detects_gaps_and_eviction(self) -> None:
        events = EventBuffer(2)
        events.push(Event(1, Kind.PROGRESS_UPDATED, agent="a"))
        events.push(Event(2, Kind.PROGRESS_UPDATED, agent="a"))
        self.assertEqual(events.diagnostics[0], 1)
        events.push(Event(4, Kind.JOB_COMPLETED))
        self.assertTrue(events.consume_resync())
        events.push(Event(5, Kind.JOB_COMPLETED))
        self.assertEqual(events.diagnostics[:2], (2, 1))
        self.assertTrue(events.consume_resync())
        self.assertEqual(tuple(event.sequence for event in events.drain()), (4, 5))

    def test_pending_keeps_visible_and_stale_generation_cannot_commit(self) -> None:
        state = PresentationState()
        state.reset_epoch(1, ("a",))
        first = state.request("a", 3)
        self.assertTrue(state.commit(PresentationMetadata(first, 10, 10, None, None)))
        stale = state.request("a", 4)
        newest = state.request("a", 4)
        self.assertEqual(state.metadata[0].token, first)
        self.assertFalse(state.commit(PresentationMetadata(stale, 20, 20, None, None)))
        self.assertTrue(state.commit(PresentationMetadata(newest, 20, 20, None, None)))
        self.assertEqual(state.metadata[0].token, newest)

        older_result = state.request("a", 9)
        older_result = type(older_result)(
            older_result.epoch, older_result.agent, 3, older_result.generation
        )
        self.assertFalse(
            state.commit(PresentationMetadata(older_result, 30, 30, None, None))
        )

    def test_failed_replacement_and_epoch_reset_preserve_invariants(self) -> None:
        state = PresentationState()
        state.reset_epoch(1, ("a",))
        visible = state.request("a", 2)
        state.commit(PresentationMetadata(visible, 1, 1, None, None))
        pending = state.request("a", 3)
        state.fail(pending)
        self.assertEqual(state.metadata[0].token, visible)
        removed = state.reset_epoch(2, ("b",))
        self.assertEqual(removed, ("a",))
        self.assertEqual(state.metadata, ())
        self.assertTrue(state.visible("b"))


if __name__ == "__main__":
    unittest.main()
