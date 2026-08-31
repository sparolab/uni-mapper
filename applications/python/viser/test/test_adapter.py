from __future__ import annotations

import contextlib
import threading
import time
import unittest
from types import SimpleNamespace

import numpy as np

from open_lmm_viser import ViserAdapter


def wait_until(predicate, timeout: float = 2.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.005)
    raise AssertionError("condition was not satisfied")


def visual(agent: str = "agent1", revision: int = 1, *, valid: bool = True):
    points = np.array([[1.0, 2.0, 3.0, 4.0]], dtype=np.float32)
    if not valid:
        points[0, 0] = np.nan
    poses = np.repeat(np.eye(4, dtype=np.float32)[None], 2, axis=0)
    poses[1, 0, 3] = 1.0
    return SimpleNamespace(
        agent=agent,
        revision=revision,
        points_available=True,
        points=points,
        poses=poses,
    )


class FakeSubscription:
    def __init__(self) -> None:
        self.closed = False

    def close(self) -> None:
        self.closed = True


class FakeRuntime:
    def __init__(self) -> None:
        self.agents = ("agent1",)
        self.revision = 1
        self.callback = None
        self.subscription = FakeSubscription()
        self.queries: list[tuple[str, float | None]] = []
        self.block_query = False
        self.query_entered = threading.Event()
        self.query_release = threading.Event()
        self.failure: Exception | None = None
        self.valid = True

    def subscribe_events(self, callback):
        self.callback = callback
        return self.subscription

    def snapshot(self):
        return SimpleNamespace(pipeline=SimpleNamespace(agents=self.agents))

    def visualization(self, agent, *, preview_voxel_size_m=None):
        self.queries.append((agent, preview_voxel_size_m))
        self.query_entered.set()
        if self.block_query:
            self.query_release.wait(2.0)
        if self.failure is not None:
            raise self.failure
        return visual(agent, self.revision, valid=self.valid)

    def emit(self, name: str, *, agent: str | None = "agent1") -> None:
        assert self.callback is not None
        self.callback(SimpleNamespace(type=name, agent=agent, affected_agents=()))


class FakeHandle:
    def __init__(self, name: str) -> None:
        self.name = name
        self.removed = False
        self.visible = True

    def remove(self) -> None:
        self.removed = True


class FakeScene:
    def __init__(self) -> None:
        self.points: list[tuple[str, FakeHandle, dict]] = []
        self.lines: list[tuple[str, FakeHandle, dict]] = []

    def add_point_cloud(self, name, **kwargs):
        handle = FakeHandle(name)
        self.points.append((name, handle, kwargs))
        return handle

    def add_line_segments(self, name, **kwargs):
        handle = FakeHandle(name)
        self.lines.append((name, handle, kwargs))
        return handle


class FakeServer:
    def __init__(self) -> None:
        self.scene = FakeScene()
        self.atomic_count = 0

    @contextlib.contextmanager
    def atomic(self):
        self.atomic_count += 1
        yield


class AdapterTests(unittest.TestCase):
    def setUp(self) -> None:
        self.runtime = FakeRuntime()
        self.server = FakeServer()
        self.adapter = ViserAdapter(
            self.runtime, self.server, preview_voxel_size_m=0.2
        )

    def tearDown(self) -> None:
        self.adapter.close()

    def start_and_wait(self) -> None:
        self.adapter.start()
        wait_until(lambda: len(self.server.scene.points) == 1)

    def test_start_performs_initial_refresh_and_close_is_idempotent(self) -> None:
        self.start_and_wait()
        self.assertEqual(self.runtime.queries, [("agent1", 0.2)])
        with self.assertRaises(RuntimeError):
            self.adapter.start()
        self.adapter.close()
        self.adapter.close()
        self.assertTrue(self.runtime.subscription.closed)
        self.assertFalse(self.adapter._worker and self.adapter._worker.is_alive())

    def test_progress_is_ignored_and_committed_burst_is_coalesced(self) -> None:
        self.start_and_wait()
        self.runtime.emit("PROGRESS_UPDATED")
        time.sleep(0.03)
        self.assertEqual(len(self.runtime.queries), 1)

        self.runtime.block_query = True
        self.runtime.query_entered.clear()
        self.runtime.emit("ARTIFACT_COMMITTED")
        self.assertTrue(self.runtime.query_entered.wait(1.0))
        for _ in range(8):
            self.runtime.emit("STAGE_COMPLETED")
        self.runtime.query_release.set()
        wait_until(lambda: len(self.runtime.queries) == 3)
        time.sleep(0.03)
        self.assertEqual(len(self.runtime.queries), 3)
        self.assertEqual(len(self.server.scene.points), 2)

    def test_stale_and_failed_candidates_preserve_visible_scene(self) -> None:
        self.runtime.revision = 5
        self.start_and_wait()
        visible = self.adapter._handles["agent1"].points

        self.runtime.revision = 4
        self.runtime.emit("JOB_COMPLETED")
        wait_until(lambda: len(self.runtime.queries) == 2)
        time.sleep(0.02)
        self.assertIs(self.adapter._handles["agent1"].points, visible)

        self.runtime.valid = False
        self.runtime.revision = 6
        self.runtime.emit("ARTIFACT_COMMITTED")
        wait_until(lambda: len(self.runtime.queries) == 3)
        time.sleep(0.02)
        self.assertIs(self.adapter._handles["agent1"].points, visible)

        self.runtime.valid = True
        self.runtime.failure = RuntimeError("temporary")
        self.runtime.emit("ARTIFACT_COMMITTED")
        wait_until(lambda: len(self.runtime.queries) == 4)
        time.sleep(0.02)
        self.assertIs(self.adapter._handles["agent1"].points, visible)

    def test_authoritative_catalog_removal_removes_owned_scene(self) -> None:
        self.start_and_wait()
        points = self.adapter._handles["agent1"].points
        trajectory = self.adapter._handles["agent1"].trajectory
        self.runtime.agents = ()
        self.runtime.emit("JOB_COMPLETED", agent=None)
        wait_until(lambda: "agent1" not in self.adapter._handles)
        self.assertTrue(points.removed)
        self.assertTrue(trajectory.removed)

    def test_agent_points_and_trajectory_share_catalog_color(self) -> None:
        self.runtime.agents = ("agent1", "agent2")
        self.adapter.start()
        wait_until(lambda: len(self.server.scene.points) == 2)
        by_name = {name: kwargs for name, _handle, kwargs in self.server.scene.points}
        lines = {name: kwargs for name, _handle, kwargs in self.server.scene.lines}
        first_points = by_name["/open_lmm/agents/agent1/points"]["colors"]
        second_points = by_name["/open_lmm/agents/agent2/points"]["colors"]
        self.assertFalse(np.array_equal(first_points, second_points))
        self.assertEqual(lines["/open_lmm/agents/agent1/trajectory"]["colors"], (230, 51, 51))
        self.assertEqual(lines["/open_lmm/agents/agent2/trajectory"]["colors"], (51, 166, 255))

    def test_committed_presentation_can_be_hidden_without_discarding_it(self) -> None:
        self.start_and_wait()
        original = self.adapter._handles["agent1"]

        self.adapter.set_presentation_visible(False)
        self.assertFalse(original.points.visible)
        self.assertFalse(original.trajectory.visible)
        self.assertFalse(original.points.removed)

        self.runtime.revision = 2
        self.runtime.emit("ARTIFACT_COMMITTED")
        wait_until(lambda: self.adapter._displayed.get("agent1") == 2)
        replacement = self.adapter._handles["agent1"]
        self.assertFalse(replacement.points.visible)
        self.assertFalse(replacement.trajectory.visible)

        self.adapter.set_presentation_visible(True)
        self.assertTrue(replacement.points.visible)
        self.assertTrue(replacement.trajectory.visible)


if __name__ == "__main__":
    unittest.main()
