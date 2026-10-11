"""Pure path geometry, progress and fixed-origin feedback checks."""

import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from action_plan import Pose
from motion_model import Motion, parse_batch, target_pose
from teaching_path import PathTracker, fixed_segment, interpolate, path_ready


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

    def test_p03_fixed_segment_interpolation(self):
        a, b = (
            (0.8402388396167952, -0.0026000285269909117),
            (1.773003723416002, -0.0026000285269909117),
        )
        relative = interpolate([(0, 0), (b[0] - a[0], b[1] - a[1])])
        points = tuple((a[0] + x, a[1] + y) for x, y in relative)
        self.assertEqual(len(points), 20)
        self.assertTrue(all(math.dist(x, y) <= 0.05 for x, y in zip(points, points[1:])))
        tracker = PathTracker(Pose(0, 0, 0), points)
        command, ready = tracker.feedback(Pose(a[0] - 0.003, a[1], 0), 0.12, 0.6)
        self.assertGreater(command[0], 0)
        self.assertFalse(ready)
        self.assertTrue(path_ready(Pose(0, 0, 0), Pose(*b, 0), points))
        self.assertEqual(
            parse_batch('path_plan P03_turn; path P03_turn'),
            [('path_plan', 'P03_turn'), ('path', 'P03_turn')],
        )

    def test_forward_and_right_extensions_hold_rotated_heading(self):
        p3 = (1.773003723416002, -0.0026000285269909117)
        p4 = (1.781439293373129, -0.5239300031759923)
        p5 = (1.3921754778962125, -0.5250984301851296)
        origin = Pose(3, 7, 2.1)
        for a, b, count, direction in ((p3, p4, 12, 'forward'), (p4, p5, 9, 'right')):
            motion = fixed_segment(a, b, -math.pi / 2)
            self.assertEqual(len(motion.points), count)
            self.assertEqual(motion.points[0], a)
            self.assertEqual(motion.points[-1], b)
            self.assertTrue(
                all(
                    math.dist(x, y) <= 0.05 + 1e-12
                    for x, y in zip(motion.points, motion.points[1:])
                )
            )
            actual = target_pose(origin, Motion('move', x=a[0], y=a[1], angle=-math.pi / 2))
            tracker = PathTracker(origin, motion.points, motion.angle)
            command, ready = tracker.feedback(actual, 0.12, 0.6)
            self.assertFalse(ready)
            self.assertAlmostEqual(command[2], 0)
            if direction == 'forward':
                self.assertGreater(command[0], 0.11)
            else:
                self.assertLess(command[1], -0.11)
                self.assertLess(abs(command[0]), 0.001)
            end = target_pose(origin, motion)
            self.assertTrue(path_ready(origin, end, motion.points, motion.angle))
            self.assertFalse(
                path_ready(origin, Pose(end.x, end.y, origin.yaw), motion.points, motion.angle)
            )
        self.assertEqual(
            parse_batch('path_plan P05; path P05'), [('path_plan', 'P05'), ('path', 'P05')]
        )

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
