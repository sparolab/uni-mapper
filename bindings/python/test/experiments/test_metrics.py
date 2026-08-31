from __future__ import annotations

import types
import unittest

import numpy as np
import open_lmm

from open_lmm.experiments import MetricPolicy
from open_lmm.experiments._metrics import EventAccumulator, summarize, visualization_records
from open_lmm.experiments import MetricRecord, MetricSource, TrialResult


class MetricTests(unittest.TestCase):
    def test_event_history_is_bounded_and_latency_is_explicit(self) -> None:
        collector = EventAccumulator(2)
        for sequence, event_type in enumerate((open_lmm.EventType.STAGE_STARTED, open_lmm.EventType.PROGRESS_UPDATED, open_lmm.EventType.STAGE_COMPLETED)):
            collector(types.SimpleNamespace(type=event_type, stage=open_lmm.Stage.DATA_LOAD, sequence=sequence))
        records = collector.records()
        dropped = next(item for item in records if item.name == "workflow.event_samples_dropped")
        self.assertEqual(dropped.value, 1)
        self.assertTrue(any(item.name == "workflow.stage_callback_wall" and item.value is not None for item in records))

    def test_visualization_reduces_without_retaining_arrays(self) -> None:
        poses = np.repeat(np.eye(4)[None, :, :], 2, axis=0)
        poses[1, 0, 3] = 3.0
        value = types.SimpleNamespace(
            agent="a", poses=poses, source_point_count=10, displayed_point_count=5,
            points_complete=False, has_bounds=True, min_bound=np.array([0.0, 0.0, 0.0]),
            max_bound=np.array([3.0, 2.0, 1.0]), points=np.empty((0, 4)),
        )
        records = visualization_records(value, MetricPolicy())
        self.assertEqual(next(item.value for item in records if item.name == "trajectory.path_length"), 3.0)
        self.assertFalse(any(isinstance(item.value, np.ndarray) for item in records))

    def test_summary_reports_failures_and_nearest_rank(self) -> None:
        digest = "sha256:" + "0" * 64
        succeeded = []
        for index, value in enumerate((1, 2, 100), 1):
            succeeded.append(TrialResult(str(index) * 64, index, index, "succeeded", None, {}, digest, digest, digest, (MetricRecord("x", value, "ms", MetricSource.SNAPSHOT, "trial"),)))
        failed = TrialResult("f" * 64, 4, 4, "failed", None, {}, digest, digest, digest, ())
        summary = next(item for item in summarize((*succeeded, failed)) if item.name == "x")
        self.assertEqual((summary.median, summary.p95, summary.mad), (2.0, 100.0, 1.0))
        self.assertEqual(summary.failure_count, 1)


if __name__ == "__main__":
    unittest.main()
