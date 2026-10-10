"""Relative odom trial geometry, direction and invalid-input regression tests."""

import json
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from action_plan import Pose
from odom_path import compile_path, track, limited_velocity


class OdomPath(unittest.TestCase):
    def setUp(self):
        self.data = json.loads(
            (Path(__file__).resolve().parents[1] / 'config/task6_odom_p01_p05.json').read_text()
        )

    def test_rotated_origin_and_downstream_edit(self):
        route = compile_path(Pose(10, 20, math.pi / 2), self.data)
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
