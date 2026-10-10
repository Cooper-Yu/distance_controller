"""Escape geometry, fixed-target retries and checkpoint restrictions."""

import math
import sys
from pathlib import Path
from dataclasses import asdict
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from action_plan import Pose, Step
from turn_escape import check_points, escape_command
from wall_session import WallSession
from session_checkpoint import validate_records


class Escape(unittest.TestCase):
    def test_receding_point_allowed_but_approaching_rejected(self):
        pts = [(0, 0.177, math.pi / 2)]
        self.assertAlmostEqual(check_points(pts, (0, -0.005, 0), 0.015), 0.017)
        with self.assertRaisesRegex(RuntimeError, 'DIRECTION'):
            check_points(pts, (0, 0.005, 0), 0.015)
        with self.assertRaisesRegex(RuntimeError, 'CLEARANCE'):
            check_points(pts, (0, -0.005, 0), 0.018)
        with self.assertRaisesRegex(RuntimeError, 'CLEARANCE'):
            check_points(pts, (0, -0.005, 0), 0.015, True)

    def test_new_obstacle_and_wrong_side_not_exempt(self):
        with self.assertRaises(RuntimeError):
            check_points([(0, -0.177, -math.pi / 2)], (0, -0.005, 0), 0.015)
        with self.assertRaisesRegex(RuntimeError, 'OBSTACLE'):
            check_points([(0, -0.181, -math.pi / 2)], (0, -0.005, 0), 0.015)
        with self.assertRaisesRegex(RuntimeError, 'SCAN'):
            check_points([], (0, -0.005, 0), 0.015)

    def test_command_stops_on_heading_and_cross_track(self):
        state = {'origin': asdict(Pose(0, 0, 0)), 'target': asdict(Pose(0, -0.03, 0))}
        cmd, ready = escape_command(Pose(0, 0, 0), state)
        self.assertEqual(cmd, (0, -0.005, 0))
        self.assertFalse(ready)
        self.assertTrue(escape_command(Pose(0, -0.029, 0), state)[1])
        for pose in [Pose(0, 0, 0.02), Pose(0.005, 0, 0), Pose(0, -0.034, 0)]:
            with self.assertRaises(RuntimeError):
                escape_command(pose, state)

    def test_partial_retry_target_and_back_restriction(self):
        actual = [Pose(0, 0, -0.9)]
        backend = SimpleNamespace(pose=lambda: actual[0], check_turn=lambda target: None)
        step = Step('t', 'turn', -1.57, {'capture': 'left'})
        s = WallSession(Pose(0, 0, 0), [step], backend, lambda *args: None)
        s.last_actual = actual[0]
        s.partial = dict(
            index=0,
            revision=0,
            step=asdict(step),
            start=asdict(Pose(0, 0, 0)),
            target=asdict(Pose(0, 0, -1.57)),
        )
        with self.assertRaises(ValueError):
            s.escape_turn(0.02)
        self.assertNotIn('turn_escape', s.partial)

        def fail(r, state):
            actual[0] = Pose(0.001, 0, -0.9)
            raise RuntimeError('blocked')

        with patch('turn_adjustment.run_adjustment', side_effect=fail):
            with self.assertRaises(RuntimeError):
                s.escape_turn(0.03)
        target = s.partial['turn_escape']['target'].copy()
        with self.assertRaises(RuntimeError):
            s.resume()
        with self.assertRaises(RuntimeError):
            s.back()

        def done(r, state):
            self.assertEqual(state['target'], target)
            actual[0] = Pose(**target)
            state['done'] = True

        with patch('turn_adjustment.run_adjustment', side_effect=done) as run:
            s.escape_turn(0.03)
            s.escape_turn(0.03)
            self.assertEqual(run.call_count, 1)
        self.assertEqual(s.partial['target']['yaw'], -1.57)
        validate_records({'active': [], 'partial': s.partial}, [step])
        s.partial['turn_escape']['floor'] = 0.001
        with self.assertRaises(ValueError):
            validate_records({'active': [], 'partial': s.partial}, [step])
