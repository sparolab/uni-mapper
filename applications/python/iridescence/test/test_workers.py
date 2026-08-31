from __future__ import annotations

import queue
import threading
import unittest

import numpy as np

from open_lmm_iridescence.state import PresentationToken
from open_lmm_iridescence.workers import VisualizationRequest, VisualizationWorker


class Snapshot:
    agent = "a"
    revision = 1
    phase = "DATA_LOAD"
    point_kind = "FILTERED_SCAN_PREVIEW"
    poses = np.empty((0, 4, 4), dtype=np.float32)
    edges = ()
    points = np.empty((0, 4), dtype=np.float32)
    points_available = False
    has_bounds = False
    displayed_point_count = 0
    source_point_count = 0


class Runtime:
    def __init__(self) -> None:
        self.calls: list[bool] = []
        self.active = 0
        self.max_active = 0
        self.lock = threading.Lock()

    def visualization(self, agent, *, include_points, preview_voxel_size_m):
        del preview_voxel_size_m
        with self.lock:
            self.active += 1
            self.max_active = max(self.max_active, self.active)
            self.calls.append(include_points)
        snapshot = Snapshot()
        snapshot.agent = agent
        with self.lock:
            self.active -= 1
        return snapshot


class WorkerTests(unittest.TestCase):
    def test_full_pending_request_cannot_be_downgraded_and_queries_are_serial(self) -> None:
        runtime = Runtime()
        results = queue.SimpleQueue()
        worker = VisualizationWorker(runtime, results, None)
        full = VisualizationRequest(PresentationToken(1, "a", 1, 1), "HEIGHT", True)
        metadata = VisualizationRequest(PresentationToken(1, "a", 1, 2), "HEIGHT", False)
        worker.request(full)
        worker.request(metadata)
        worker.start()
        result = results.get(timeout=2)
        worker.close()
        self.assertIsNone(result.error)
        self.assertEqual(runtime.calls, [True])
        self.assertEqual(runtime.max_active, 1)


if __name__ == "__main__":
    unittest.main()
