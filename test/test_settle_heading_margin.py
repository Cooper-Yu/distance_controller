"""Regression coverage for tighter settling yaw without changing final acceptance."""

import io
import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from action_plan import Pose
from motion_console import Console
from motion_model import HeadingPid, Motion, feedback
from teaching_path import PathTracker, path_ready

POINTS = ((0.0, 0.0), (0.1, 0.0))


class HeadingMarginTests(unittest.TestCase):
    def test_path_pid_continues_inside_final_but_outside_settling_margin(self):
        tracker = PathTracker(Pose(0, 0, 0), POINTS)
        tracker.heading_pid.integral = -0.1
        command, ready = tracker.feedback(Pose(0.1, 0, 0.009), 0.02, 0.1, yaw_tolerance=0.006)
        self.assertFalse(ready)
        self.assertLess(command[2], 0)
        self.assertLess(tracker.heading_pid.integral, -0.1)

    def test_normal_path_ready_and_pid_reset_stay_unchanged(self):
        tracker = PathTracker(Pose(0, 0, 0), POINTS)
        tracker.heading_pid.integral = -0.1
        self.assertTrue(tracker.feedback(Pose(0.1, 0, 0.009), 0.02, 0.1)[1])
        self.assertEqual(tracker.heading_pid.integral, 0)
        self.assertFalse(path_ready(Pose(0, 0, 0), Pose(0.1, 0, 0.011), POINTS))

    def test_move_and_turn_use_settling_margin(self):
        for motion, actual in [
            (Motion('move', x=0.1), Pose(0.1, 0, 0.009)),
            (Motion('turn', angle=1), Pose(0, 0, 1.009)),
        ]:
            self.assertTrue(feedback(Pose(0, 0, 0), actual, motion, 0.02, 0.1)[1])
            command, ready = feedback(
                Pose(0, 0, 0), actual, motion, 0.02, 0.1, HeadingPid(), yaw_tolerance=0.006
            )
            self.assertFalse(ready)
            self.assertLess(command[2], 0)

    def test_wrap_keeps_shortest_path_heading(self):
        tracker = PathTracker(Pose(0, 0, 0), POINTS, math.pi)
        command, ready = tracker.feedback(
            Pose(0.1, 0, -math.pi + 0.009), 0.02, 0.1, yaw_tolerance=0.006
        )
        self.assertFalse(ready)
        self.assertLess(command[2], 0)
        self.assertLess(abs(command[2]), 0.03)
        self.assertTrue(path_ready(Pose(0, 0, 0), Pose(0.1, 0, -math.pi + 0.009), POINTS, math.pi))

    def test_turn_four_cm_and_heading_off_preserved(self):
        with self.assertRaisesRegex(RuntimeError, '4 cm'):
            feedback(
                Pose(0, 0, 0),
                Pose(0.0401, 0, 0.009),
                Motion('turn'),
                0.02,
                0.1,
                yaw_tolerance=0.006,
            )
        command, ready = feedback(
            Pose(0, 0, 0),
            Pose(0.1, 0, 0.2),
            Motion('move', x=0.1, heading=False),
            0.02,
            0.1,
            yaw_tolerance=0.006,
        )
        self.assertTrue(ready)
        self.assertEqual(command[2], 0)

    def test_final_stopped_acceptance_remains_point_zero_one(self):
        console = Console(SimpleNamespace(max_speed=0.02), None, None, io.StringIO(), 0.1)
        motion = Motion('path', x=0.1, points=POINTS)
        self.assertTrue(console.stopped_check(Pose(0, 0, 0), Pose(0.1, 0, 0.009), motion, 0)[0])
        self.assertFalse(console.stopped_check(Pose(0, 0, 0), Pose(0.1, 0, 0.011), motion, 0)[0])


if __name__ == '__main__':
    unittest.main()
