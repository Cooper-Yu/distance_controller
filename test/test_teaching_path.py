"""Pure path geometry, progress and fixed-origin feedback checks."""

import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from action_plan import Pose
from motion_model import Motion, parse_batch, target_pose
from teaching_path import PathTracker, interpolate, path_ready


class TeachingPathTests(unittest.TestCase):
    def test_measured_anchors_and_spacing(self):
        anchors = [(0.0, 0.0), (0.100728401978, 0.000929167347), (0.840238839617, -0.002600028527)]
        points = interpolate(anchors)
        self.assertEqual(len(points), 19)
        self.assertTrue(all(p in points for p in anchors))
        self.assertTrue(all(math.dist(a, b) <= 0.05 + 1e-12 for a, b in zip(points, points[1:])))

    def test_fixed_reference_rotated_and_wrapped_heading(self):
        origin = Pose(8.0, -2.0, math.pi - 0.002)
        points = interpolate([(0.0, 0.0), (0.84, -0.003)])
        end = target_pose(origin, Motion('move', x=0.84, y=-0.003))
        self.assertTrue(path_ready(origin, Pose(end.x, end.y, end.yaw - 2 * math.pi), points))
        tracker = PathTracker(origin, points)
        command, ready = tracker.feedback(origin, 0.04, 0.1)
        self.assertGreater(command[0], 0)
        self.assertFalse(ready)

    def test_passed_points_are_not_stop_targets(self):
        points = interpolate([(0.0, 0.0), (0.84, 0.0)])
        tracker = PathTracker(Pose(0, 0, 0), points)
        command, ready = tracker.feedback(Pose(0.163, 0, 0), 0.04, 0.1)
        self.assertGreater(command[0], 0)
        self.assertFalse(ready)
        progress = tracker.progress
        tracker.feedback(Pose(0.15, 0, 0), 0.04, 0.1)
        self.assertEqual(tracker.progress, progress)
        self.assertGreater(tracker.ahead()[0], 0.163)

    def test_correction_deceleration_and_endpoint(self):
        points = interpolate([(0.0, 0.0), (0.84, 0.0)])
        tracker = PathTracker(Pose(0, 0, 0), points)
        command, ready = tracker.feedback(Pose(0.3, 0.015, 0.02), 0.04, 0.1)
        self.assertLess(command[1], 0)
        self.assertLess(command[2], 0)
        self.assertFalse(ready)
        command, ready = tracker.feedback(Pose(0.83, 0, 0), 0.04, 0.1)
        self.assertAlmostEqual(command[0], 0.015)
        self.assertFalse(ready)
        self.assertTrue(tracker.feedback(Pose(0.839, 0, 0), 0.04, 0.1)[1])
        self.assertFalse(tracker.feedback(Pose(0.84, 0, 0.02), 0.04, 0.1)[1])
        with self.assertRaises(RuntimeError):
            tracker.feedback(Pose(0.84, 0.06, 0), 0.04, 0.1)

    def test_requested_cruise_speed_and_unchanged_endpoint(self):
        points = interpolate([(0.0, 0.0), (0.84, 0.0)])
        for speed in (0.04, 0.06, 0.08, 0.12):
            tracker = PathTracker(Pose(0, 0, 0), points)
            command, _ = tracker.feedback(Pose(0.3, 0, 0), speed, 0.1)
            self.assertAlmostEqual(command[0], speed)
            command, ready = tracker.feedback(Pose(0.83, 0, 0), speed, 0.1)
            self.assertAlmostEqual(command[0], 0.015)
            self.assertFalse(ready)
            self.assertTrue(tracker.feedback(Pose(0.839, 0, 0), speed, 0.1)[1])

    def test_task5_heading_damping_and_saturation(self):
        tracker = PathTracker(Pose(0, 0, 0), interpolate([(0, 0), (0.84, 0)]))
        command, _ = tracker.feedback(Pose(0.3, 0, 0.02), 0.08, 0.1, 0.05, 0.04)
        self.assertAlmostEqual(command[2], -1.8 * 0.02 - 0.03 * 0.001 - 0.35 * 0.04)
        for _ in range(100):
            tracker.feedback(Pose(0.3, 0, 0.5), 0.08, 0.1, 0.05, 0)
        self.assertAlmostEqual(tracker.heading_pid.integral, -0.001)
        with self.assertRaises(RuntimeError):
            tracker.feedback(Pose(0.3, 0, 0), 0.08, 0.1, 0.3, 0)

    def test_reject_unreviewed_paths_and_batch(self):
        for points in [
            [(0, 0), (math.nan, 0)],
            [(0, 0), (-0.1, 0)],
            [(0, 0), (0.1, 0.1)],
            [(0, 0), (3, 0)],
        ]:
            with self.assertRaises(ValueError):
                interpolate(points)
        self.assertEqual(parse_batch('path_plan; path P02'), [('path_plan',), ('path', 'P02')])
        with self.assertRaises(ValueError):
            parse_batch('path P03')


if __name__ == '__main__':
    unittest.main()
