"""Geometry, control signs, failure boundaries and actual-anchor history tests."""

from dataclasses import asdict
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from action_plan import Pose, Step, load_steps, reverse_steps
from wall_geometry import fit_wall, scan_points, support, swept_clearance
from wall_policy import Policy, command
from wall_session import WallSession


def wall_points(side, gap):
    if side == 'left':
        return [
            (x, gap + 0.16, math.atan2(gap + 0.16, x)) for x in [i * 0.005 for i in range(-18, 19)]
        ]
    if side == 'right':
        return [
            (x, -gap - 0.16, math.atan2(-gap - 0.16, x))
            for x in [i * 0.005 for i in range(-18, 19)]
        ]
    return [(gap + 0.17, y, math.atan2(y, gap + 0.17)) for y in [i * 0.005 for i in range(-18, 19)]]


class Geometry(unittest.TestCase):
    def test_gap_from_offset_backward_laser(self):
        tf = SimpleNamespace(
            rotation=SimpleNamespace(x=0.0, y=0.0, z=1.0, w=0.0),
            translation=SimpleNamespace(x=0.02, y=0.0, z=0.173),
        )
        scan = SimpleNamespace(
            angle_min=math.pi, angle_increment=0.01, range_min=0.15, range_max=40.0, ranges=[0.24]
        )
        points, _ = scan_points(scan, tf)
        self.assertAlmostEqual(points[0][0], 0.26)
        self.assertAlmostEqual(points[0][0] - support(1, 0), 0.09)

    def test_body_wall_gaps(self):
        for side in ('front', 'left', 'right'):
            points = wall_points(side, 0.10)
            wall = fit_wall(points, {side: len(points)}, side)
            self.assertAlmostEqual(wall.gap, 0.10)
            self.assertAlmostEqual(wall.rms, 0.0)

    def test_sparse_and_wrong_surface_rejected(self):
        with self.assertRaises(ValueError):
            fit_wall([], {'left': 40}, 'left')
        points = [
            (x, 0.2 + 0.4 * x, math.atan2(0.2 + 0.4 * x, x))
            for x in [i * 0.004 for i in range(-20, 21)]
        ]
        with self.assertRaises(ValueError):
            fit_wall(points, {'left': len(points)}, 'left')

    def test_rotation_corner_guard(self):
        points = [(0.19, 0.0, 0.0)]
        self.assertGreaterEqual(swept_clearance(points), 0.019)
        self.assertLess(swept_clearance(points, wz=math.pi / 2, duration=1), 0.0)


class Control(unittest.TestCase):
    def test_left_and_right_signs(self):
        step = Step('f', 'forward', 0.9)
        for side, sign in [('left', 1), ('right', -1)]:
            v, ready, _ = command(step, Policy(follow=side), 0.1, 0, 0, 0, {side: 0.12})
            self.assertGreater(v[1] * sign, 0)
            self.assertFalse(ready)

    def test_front_and_strafe_arrival(self):
        for kind, side in [('forward', 'front'), ('left', 'left'), ('right', 'right')]:
            v, ready, _ = command(
                Step('x', kind, 0.5), Policy(stop=side), 0.1, 0.2, 0, 0, {side: 0.1}
            )
            self.assertTrue(ready)
            self.assertEqual(v, (0.0, 0.0, 0.0))

    def test_close_wall_and_opening_reject(self):
        with self.assertRaises(RuntimeError):
            command(Step('f', 'forward', 0.9), Policy(stop='front'), 0.1, 0, 0, 0, {'front': 0.06})
        with self.assertRaises(RuntimeError):
            command(Step('f', 'forward', 0.9), Policy(follow='left'), 0.1, 0, 0, 0, {'left': 0.5})

    def test_large_heading_error_stops_translation(self):
        v, _, _ = command(Step('f', 'forward', 0.9), Policy(), 0.1, 0, 0, 0.15, {})
        self.assertEqual(v[:2], (0.0, 0.0))
        self.assertGreater(v[2], 0)

    def test_speed_cap_and_approach(self):
        step = Step('f', 'forward', 0.9)
        fast, _, _ = command(step, Policy(), 0.1, 0, 0, 0, {}, 0.06)
        near, _, _ = command(step, Policy(), 0.1, 0.88, 0, 0, {}, 0.06)
        self.assertAlmostEqual(fast[0], 0.06)
        self.assertAlmostEqual(near[0], 0.014)
        for bad in (0, -0.1, 0.09, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                command(step, Policy(), 0.1, 0, 0, 0, {}, bad)

    def test_config_complete_and_reverse_rejected(self):
        path = Path(__file__).resolve().parents[1] / 'config/task6_wall_actions.json'
        steps = load_steps(json.loads(path.read_text()))
        self.assertEqual(len(steps), 21)
        self.assertTrue(all(s.wall for s in steps))
        with self.assertRaises(ValueError):
            reverse_steps(steps)


class History(unittest.TestCase):
    def test_actual_rebase_reference_and_back(self):
        class Backend:
            reference = 0.1
            actual = Pose(0, 0, 0)
            last_report = {}

            def pose(self):
                return self.actual

            def execute(self, step, target):
                if step.wall:
                    self.actual = Pose(0.4, 0.01, target.yaw)
                    self.reference = 0.12
                    return self.actual
                self.actual = target
                return None

        backend = Backend()
        policy = asdict(Policy(stop='front'))
        steps = [Step('f', 'forward', 0.7, policy), Step('g', 'forward', 0.5, asdict(Policy()))]
        session = WallSession(backend.actual, steps, backend, lambda *a: None)
        session.next()
        self.assertAlmostEqual(session.goals[1].x, 0.9)
        session.back()
        self.assertEqual(session.cursor, 0)
        self.assertEqual(backend.reference, 0.1)
        self.assertEqual(backend.actual, Pose(0, 0, 0))


if __name__ == '__main__':
    unittest.main()
