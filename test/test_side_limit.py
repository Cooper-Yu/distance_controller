"""Correction limits keep the hard guard, planned forward axis and yaw controller intact."""

import math
import sys
from pathlib import Path
from unittest import TestCase, main
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from wall_geometry import bounded_follow_velocity, swept_clearance
from wall_runtime import WallRunner


class SideLimit(TestCase):
    def test_cloud_pair_stops_right_correction(self):
        points = [(-0.00676167, -0.18204344, -1.71676), (-0.00517, -0.18227, -1.708)]
        cmd, scale = bounded_follow_velocity(points, (0.05997, -0.007061, 0.0), 0.0)
        self.assertEqual(scale, 0.0)
        self.assertEqual(cmd, (0.05997, 0.0, 0.0))
        self.assertGreaterEqual(swept_clearance(points, *cmd), 0.02)

    def test_symmetric_left_obstacle(self):
        cmd, scale = bounded_follow_velocity([(0.0, 0.182, 1.57)], (0.06, 0.007, 0.0), 0.0)
        self.assertEqual(scale, 0.0)
        self.assertEqual(cmd, (0.06, 0.0, 0.0))

    def test_clear_corridor_unchanged(self):
        planned = (0.06, -0.007, 0.01)
        cmd, scale = bounded_follow_velocity([(1.0, 0, 0)], planned, 0.0)
        self.assertEqual((cmd, scale), (planned, 1.0))

    def test_front_obstacle_and_static_overlap_still_fail(self):
        for point in [(0.20, 0.0, 0.0), (0.0, -0.17, -1.57)]:
            cmd, _ = bounded_follow_velocity([point], (0.06, -0.007, 0.0), 0.0)
            self.assertLess(swept_clearance([point], *cmd), 0.02)

    def test_reduction_is_in_planned_axis(self):
        rotation = 0.04
        c, s = math.cos(rotation), math.sin(rotation)
        cmd, scale = bounded_follow_velocity([(0, -0.182, -1.57)], (0.06, -0.012, 0.01), rotation)
        self.assertAlmostEqual(c * cmd[0] + s * cmd[1], 0.06)
        self.assertAlmostEqual(-s * cmd[0] + c * cmd[1], -0.012 * scale)
        self.assertEqual(cmd[2], 0.01)

    def test_missing_original_side_coverage_is_not_bypassed(self):
        r = object.__new__(WallRunner)
        points = [(1.0, i * 0.001, 0.0) for i in range(10)]
        r.geometry = lambda: (points, {'front': 10, 'right': 80})
        with self.assertRaisesRegex(RuntimeError, 'right'):
            r.follow_velocity((0.06, -0.007, 0), 0, 'front')

    def test_measured_lateral_motion_can_still_stop(self):
        r = object.__new__(WallRunner)
        r.geometry = lambda: ([(0.0, -0.182, -1.57)], {})
        r.translation_coverage = Mock()
        r.twist_frame = 'base_link'
        r.body_velocity = (0.0, -0.02, 0.0)
        r.reject_translation = Mock(side_effect=RuntimeError('measured blocked'))
        with self.assertRaisesRegex(RuntimeError, 'measured blocked'):
            r.follow_velocity((0.06, -0.007, 0), 0, 'front')
        self.assertEqual(r.reject_translation.call_args.kwargs['source'], 'measured')


if __name__ == '__main__':
    main()
