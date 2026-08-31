from __future__ import annotations

import unittest

import numpy as np
import open_lmm

from python_test_fixture import RuntimeFixture


class NumpyTests(unittest.TestCase):
    def setUp(self) -> None:
        self.fixture = RuntimeFixture(
            "numpy",
            scan_count=3,
            points_per_scan=20000,
            voxel_size=0.001,
        )

    def tearDown(self) -> None:
        self.fixture.cleanup()

    def test_semantic_arrays_are_read_only_and_own_their_snapshot(self) -> None:
        runtime = open_lmm.Runtime(1)
        runtime.open(self.fixture.config, label="numpy")
        runtime.run_stage(open_lmm.Stage.DATA_LOAD).wait()
        visualization = runtime.visualization(
            "agent1",
            preview_voxel_size_m=0.001,
        )
        points = visualization.points
        poses = visualization.poses
        first_point = points[0].copy()

        self.assertEqual(points.dtype, np.dtype("float32"))
        self.assertEqual(points.ndim, 2)
        self.assertEqual(points.shape[1], 4)
        self.assertFalse(points.flags.writeable)
        self.assertEqual(points.strides, (16, 4))
        self.assertEqual(points.nbytes, visualization.displayed_point_count * 16)
        self.assertGreaterEqual(points.nbytes, 300_000)
        self.assertEqual(poses.dtype, np.dtype("float32"))
        self.assertEqual(poses.shape, (3, 4, 4))
        self.assertFalse(poses.flags.writeable)
        self.assertEqual(visualization.min_bound.shape, (3,))
        np.testing.assert_allclose(poses[:, 0, 3], [0.0, 0.1, 0.2])
        np.testing.assert_array_equal(points[:, 3], np.ones(points.shape[0]))
        np.testing.assert_array_equal(visualization.min_bound, points[:, :3].min(0))
        np.testing.assert_array_equal(visualization.max_bound, points[:, :3].max(0))

        without_points = runtime.visualization("agent1", include_points=False)
        self.assertEqual(without_points.points.dtype, np.dtype("float32"))
        self.assertEqual(without_points.points.shape, (0, 4))
        self.assertFalse(without_points.points.flags.writeable)

        with self.assertRaises(ValueError):
            points[0, 0] = 0.0
        runtime.close()
        np.testing.assert_array_equal(points[0], first_point)


if __name__ == "__main__":
    unittest.main()
