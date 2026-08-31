from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import unittest

import numpy as np

from open_lmm_iridescence.conversion import intensity_colors, render_candidate
from open_lmm_iridescence.state import PresentationToken


class EdgeType(Enum):
    INTRA_LOOP = 1
    INTER_LOOP = 2


@dataclass
class Edge:
    from_agent: str
    from_index: int
    to_agent: str
    to_index: int
    type: EdgeType


class Snapshot:
    agent = "a"
    revision = 7
    phase = "OPTIMIZATION"
    point_kind = "FINAL_STATIC_MAP"
    poses = np.repeat(np.eye(4, dtype=np.float32)[None, :, :], 3, axis=0)
    poses[:, 0, 3] = np.arange(3)
    edges = (Edge("a", 0, "a", 2, EdgeType.INTRA_LOOP),)
    points = np.asarray([[0, 0, 0, 0], [1, 2, 3, 1]], dtype=np.float32)
    points_available = True
    has_bounds = True
    min_bound = np.asarray([0, 0, 0], dtype=np.float32)
    max_bound = np.asarray([1, 2, 3], dtype=np.float32)
    displayed_point_count = 2
    source_point_count = 2


class ConversionTests(unittest.TestCase):
    def test_candidate_has_contiguous_float32_buffers_and_metadata_only_record(self) -> None:
        token = PresentationToken(1, "a", 7, 2)
        candidate = render_candidate(Snapshot(), token, "INTENSITY")
        self.assertEqual(candidate.points.shape, (2, 3))
        self.assertEqual(candidate.points.dtype, np.float32)
        self.assertTrue(candidate.points.flags.c_contiguous)
        self.assertEqual(candidate.intensity_colors.shape, (2, 4))
        self.assertEqual(candidate.trajectory.shape, (3, 3))
        self.assertEqual(candidate.intra_loops.shape, (2, 3))
        self.assertFalse(hasattr(candidate.metadata, "points"))

    def test_intensity_colors_are_blue_to_red_and_finite(self) -> None:
        points = np.asarray([[0, 0, 0, -2], [0, 0, 0, 2]], dtype=np.float32)
        colors = intensity_colors(points)
        np.testing.assert_allclose(colors[0], [0, 0.15, 1, 1])
        np.testing.assert_allclose(colors[1], [1, 0.15, 0, 1])

    def test_invalid_point_payload_is_rejected_before_rendering(self) -> None:
        snapshot = Snapshot()
        snapshot.points = np.zeros((2, 3), dtype=np.float32)
        with self.assertRaises(ValueError):
            render_candidate(snapshot, PresentationToken(1, "a", 7, 1), "HEIGHT")


if __name__ == "__main__":
    unittest.main()
