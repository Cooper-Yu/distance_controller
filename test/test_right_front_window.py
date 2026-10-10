"""Independent stopping sector preserves obstacle points and quality gates."""

import math
import sys
from pathlib import Path
from dataclasses import asdict
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from action_plan import Pose, Step
from wall_policy import Policy
from wall_geometry import side_distance
from wall_session import WallSession


class RightFrontWindow(unittest.TestCase):
    def test_sector_excludes_rear_wall_without_mutating_points(self):
        points = []
        for deg in range(-120, -59):
            a = math.radians(deg)
            dist = min(-0.15 / math.cos(a) if math.cos(a) < -1e-6 else 99, -0.58 / math.sin(a))
            points.append((dist * math.cos(a), dist * math.sin(a), a))
        original = list(points)
        counts = {'right': 40, 'right_wide': 61, 'right_front20': 21}
        wall = side_distance(points, counts, 'right', 0, right_front_window=True)
        self.assertAlmostEqual(wall.gap, 0.42)
        self.assertEqual(points, original)
        with self.assertRaises(ValueError):
            side_distance(points, counts, 'right', 0)
        with self.assertRaises(ValueError):
            side_distance(points[:5], counts, 'right', 0, right_front_window=True)

    def test_policy_only_for_right_stop(self):
        Policy(stop='right', right_front_window=True).validate('right')
        for kind in ('forward', 'left', 'turn'):
            with self.assertRaises(ValueError):
                Policy(right_front_window=True).validate(kind)

    def test_partial_edit_preserves_target_and_path(self):
        pose = Pose(0, 0, 0)
        step = Step('right', 'right', 0.27, {'stop': 'right'})
        s = WallSession(pose, [step], SimpleNamespace(pose=lambda: pose), Mock())
        s.partial = {
            'index': 0,
            'step': asdict(step),
            'target': asdict(Pose(0, -0.27, 0)),
            'path_m': 0.012,
        }
        s.set_policy(0, 'right_front_window', 1)
        self.assertTrue(s.partial['step']['wall']['right_front_window'])
        self.assertEqual(s.partial['path_m'], 0.012)
        self.assertEqual(s.partial['target'], asdict(Pose(0, -0.27, 0)))
