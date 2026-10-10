"""Relative odom trial geometry, direction and invalid-input regression tests."""

import io
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from action_plan import Pose
from odom_path import compile_path, track, limited_velocity, waypoint_stop
from odom_trial import Trial


class OdomPath(unittest.TestCase):
    def setUp(self):
        self.data = json.loads(
            (Path(__file__).resolve().parents[1] / 'config/task6_odom_p01_p05.json').read_text()
        )

    def test_rotated_origin_and_downstream_edit(self):
        route = compile_path(Pose(10, 20, math.pi / 2), self.data)
        p04 = route[3]['points'][-1]
        self.assertAlmostEqual(p04.x, 10.48)
        self.assertAlmostEqual(p04.y, 21.69)
        self.assertTrue(all(abs(p.x - p04.x) < 1e-9 for p in route[4]['points']))
        end = route[-1]['points'][-1]
        self.assertAlmostEqual(end.x, 10.48)
        self.assertAlmostEqual(end.y, 21.403)
        self.data['segments'][0]['offsets'][0][0] += 0.1
        changed = compile_path(Pose(10, 20, math.pi / 2), self.data)[-1]['points'][-1]
        self.assertAlmostEqual(changed.y - end.y, 0.1)

    def test_late_offset_and_turn_direction(self):
        route = compile_path(Pose(0, 0, 0), self.data)
        self.assertTrue(all(p.y == 0 for p in route[1]['points'] if p.x <= 1.49 + 1e-9))
        idx, velocity, ready = track(Pose(1.69, -0.04, 0), route[2], 0, 0.06)
        self.assertLess(velocity[2], 0)
        self.assertEqual(velocity[:2], (0, 0))
        self.assertFalse(ready)

    def test_feedback_frame_and_limits(self):
        segment = {'kind': 'translate', 'points': [Pose(0, 1, math.pi / 2)]}
        _, v, _ = track(Pose(0, 0, math.pi / 2), segment, 0, 0.06)
        self.assertGreater(v[0], 0)
        self.assertAlmostEqual(v[1], 0)
        self.assertLessEqual(math.hypot(*v[:2]), 0.06)
        v = limited_velocity((0, 0, 0), (0.06, 0.06, 0.2), 0.05)
        self.assertLessEqual(math.hypot(*v[:2]), 0.0060001)
        self.assertLessEqual(v[2], 0.020001)

    def test_invalid_geometry(self):
        self.data['spacing_m'] = float('nan')
        with self.assertRaises(ValueError):
            compile_path(Pose(0, 0, 0), self.data)

    def test_named_arrival_and_future_extension(self):
        self.data['segments'].append(
            dict(name='P05_P06', kind='translate', offsets=[[0.1, 0]], waypoint='P06')
        )
        route = compile_path(Pose(0, 0, 0), self.data)
        self.assertEqual(waypoint_stop(route, 'P01'), 0)
        self.assertEqual(waypoint_stop(route, 'P03'), 2)
        self.assertEqual(waypoint_stop(route, 'P04'), 4)
        self.assertEqual(waypoint_stop(route, 'P06'), 6)
        with self.assertRaises(ValueError):
            waypoint_stop(route, 'P99')
        self.data['segments'][-1]['waypoint'] = 'P04'
        with self.assertRaises(ValueError):
            compile_path(Pose(0, 0, 0), self.data)

    def test_run_to_boundary_failure_and_partial(self):
        trial = Trial(SimpleNamespace(pump=lambda: None), self.data, Pose(0, 0, 0), io.StringIO())
        calls = []

        def execute():
            calls.append(trial.cursor)
            trial.cursor += 1

        trial.execute = execute
        trial.run_to('P04')
        self.assertEqual(calls, [0, 1, 2, 3])
        trial.run_to('P04')
        self.assertEqual(len(calls), 4)
        with self.assertRaises(RuntimeError):
            trial.run_to('P03')
        trial.partial = True
        with self.assertRaises(RuntimeError):
            trial.run_to('P05')
        trial.partial = False
        trial.cursor = 0

        def failure():
            trial.partial = True
            raise RuntimeError('stale odom')

        trial.execute = failure
        with self.assertRaisesRegex(RuntimeError, 'stale odom'):
            trial.run_to('P04')
        self.assertEqual(trial.cursor, 0)
        self.assertTrue(trial.partial)
