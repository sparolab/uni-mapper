from __future__ import annotations

import unittest
from types import SimpleNamespace

import numpy as np

from open_lmm_viser.conversion import (
    agent_color,
    point_cloud_payload,
    scene_key,
    trajectory_segments,
    visualization_candidate,
)


class ConversionTests(unittest.TestCase):
    def test_points_are_contiguous_float32_with_normalized_rgb(self) -> None:
        source = np.array(
            [[1.0, 2.0, 3.0, 10.0], [4.0, 5.0, 6.0, 20.0]],
            dtype=np.float32,
        )
        payload = point_cloud_payload(source)
        np.testing.assert_array_equal(payload.points, source[:, :3])
        np.testing.assert_array_equal(
            payload.colors,
            np.array([[0, 0, 0], [255, 255, 255]], dtype=np.uint8),
        )
        self.assertTrue(payload.points.flags.c_contiguous)
        self.assertTrue(payload.colors.flags.c_contiguous)
        self.assertEqual(payload.points.dtype, np.float32)
        self.assertEqual(payload.colors.dtype, np.uint8)

    def test_constant_and_empty_intensity_are_valid(self) -> None:
        constant = np.array([[0.0, 0.0, 0.0, 4.0]], dtype=np.float32)
        np.testing.assert_array_equal(
            point_cloud_payload(constant).colors,
            np.array([[192, 192, 192]], dtype=np.uint8),
        )
        empty = point_cloud_payload(np.empty((0, 4), dtype=np.float32))
        self.assertEqual(empty.points.shape, (0, 3))
        self.assertEqual(empty.colors.shape, (0, 3))

    def test_invalid_points_fail_closed(self) -> None:
        invalid = (
            np.zeros((2, 3), dtype=np.float32),
            np.zeros((2, 4), dtype=np.float64),
            np.zeros((2, 2, 4), dtype=np.float32),
        )
        for value in invalid:
            with self.subTest(shape=value.shape, dtype=value.dtype):
                with self.assertRaises(ValueError):
                    point_cloud_payload(value)
        nonfinite = np.zeros((1, 4), dtype=np.float32)
        nonfinite[0, 0] = np.nan
        with self.assertRaises(ValueError):
            point_cloud_payload(nonfinite)

    def test_poses_become_contiguous_line_segments(self) -> None:
        poses = np.repeat(np.eye(4, dtype=np.float32)[None, :, :], 3, axis=0)
        poses[:, 0, 3] = (1.0, 2.0, 4.0)
        segments = trajectory_segments(poses)
        assert segments is not None
        np.testing.assert_array_equal(
            segments[:, :, 0], np.array([[1.0, 2.0], [2.0, 4.0]], np.float32)
        )
        self.assertEqual(segments.shape, (2, 2, 3))
        self.assertTrue(segments.flags.c_contiguous)

    def test_zero_or_one_pose_has_no_trajectory(self) -> None:
        self.assertIsNone(trajectory_segments(np.empty((0, 4, 4), np.float32)))
        self.assertIsNone(trajectory_segments(np.eye(4, dtype=np.float32)[None]))

    def test_scene_key_is_deterministic_and_unambiguous(self) -> None:
        self.assertEqual(scene_key("agent one"), "agent%20one")
        self.assertEqual(scene_key("agent/one"), "agent%2Fone")
        self.assertNotEqual(scene_key("agent one"), scene_key("agent/one"))

    def test_unavailable_points_preserve_point_candidate(self) -> None:
        snapshot = SimpleNamespace(
            agent="agent1",
            revision=3,
            points_available=False,
            points=None,
            poses=np.empty((0, 4, 4), np.float32),
        )
        candidate = visualization_candidate(snapshot)
        self.assertIsNone(candidate.point_cloud)
        self.assertIsNone(candidate.trajectory)

    def test_agent_palette_matches_iridescence_and_is_distinct(self) -> None:
        expected = (
            (230, 51, 51),
            (51, 166, 255),
            (51, 217, 89),
            (255, 166, 26),
            (166, 89, 255),
            (26, 217, 217),
            (255, 89, 191),
        )
        self.assertEqual(tuple(agent_color(index) for index in range(7)), expected)
        self.assertEqual(len(set(agent_color(index) for index in range(255))), 255)
        self.assertEqual(agent_color(3), agent_color(3))

    def test_agent_point_color_preserves_hue_across_intensity(self) -> None:
        source = np.array(
            [[0.0, 0.0, 0.0, 0.0], [1.0, 1.0, 1.0, 10.0]],
            dtype=np.float32,
        )
        payload = point_cloud_payload(source, (100, 200, 50))
        np.testing.assert_array_equal(payload.colors[1], (100, 200, 50))
        self.assertTrue(np.all(payload.colors[0] < payload.colors[1]))


if __name__ == "__main__":
    unittest.main()
