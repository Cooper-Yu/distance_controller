"""Endpoint calibration preserves recovery anchors and separates braking from correction."""

from dataclasses import asdict
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from action_plan import Pose, Step
from wall_policy import Policy, FrontStopVerification, command
from wall_session import WallSession
from session_checkpoint import validate_records


class FrontStop(unittest.TestCase):
    def test_distinct_stopped_scans_then_correction(self):
        gate = FrontStopVerification()
        policy = Policy(stop='front', stop_tolerance=0.01)

        def tick(stamp, stopped=True, error=0):
            return gate.apply(policy, {'along_m': error}, stamp, stopped, (0, 0.01, 0.02), False)

        self.assertEqual(tick(1, False)[0], (0, 0, 0))
        tick(1)
        self.assertEqual(gate.count, 0)
        tick(2)
        tick(2)
        self.assertEqual(gate.count, 1)
        tick(3)
        self.assertEqual(tick(4)[0], (0, 0, 0))
        self.assertTrue(gate.confirmed)
        self.assertEqual(tick(5)[0], (0, 0.01, 0.02))
        tick(6, error=0.011)
        self.assertFalse(gate.confirmed)
        self.assertEqual(tick(7)[0], (0, 0, 0))
        gate.reset()
        self.assertEqual(gate.count, 0)

    def test_observed_gaps_in_band_not_safety_override(self):
        p = Policy(follow='left', stop='front', stop_clearance=0.075, stop_tolerance=0.01)
        step = Step('P03', 'forward', 0.77, asdict(p))
        for gap in (0.07738459099, 0.07100692152, 0.06948482296):
            velocity, ready, _ = command(
                step, p, 0.0975, 0.815, 0, 0, {'front': gap, 'left': 0.0975}
            )
            self.assertTrue(ready)
            self.assertEqual(velocity, (0, 0, 0))
        with self.assertRaises(RuntimeError):
            command(step, p, 0.0975, 0.815, 0, 0, {'front': 0.059, 'left': 0.0975})
        for value in (float('nan'), 0.02, 0.004):
            with self.assertRaises(ValueError):
                Policy(stop_tolerance=value).validate('forward')

    def test_partial_edit_retains_anchor_and_history(self):
        origin = Pose(0, 0, 0)
        backend = SimpleNamespace(pose=lambda: origin)
        events = []
        step = Step('P03', 'forward', 0.77, asdict(Policy(follow='left', stop='front')))
        s = WallSession(origin, [step], backend, lambda *e: events.append(e))
        s.partial = {
            'index': 0,
            'step': asdict(step),
            'revision': 0,
            'start': asdict(origin),
            'target': asdict(Pose(0.77, 0, 0)),
            'path_m': 0.826,
            'reference_before': 0.0975,
        }
        s.set_policy(0, 'stop_clearance', 0.075)
        s.set_policy(0, 'stop_tolerance', 0.01)
        self.assertEqual(s.partial['path_m'], 0.826)
        self.assertEqual(s.partial['reference_before'], 0.0975)
        self.assertEqual(s.partial['start'], asdict(origin))
        self.assertEqual(s.partial['target'], asdict(Pose(0.77, 0, 0)))
        validate_records({'active': [], 'partial': s.partial}, s.steps)
        self.assertEqual(events[-1][1]['previous']['wall']['stop_tolerance'], 0.008)
        for field, value in (
            ('follow_clearance', 0.08),
            ('max_travel', 2),
            ('stop_tolerance', None),
        ):
            with self.assertRaises(ValueError):
                s.set_policy(0, field, value)
        s.partial['return_started'] = True
        with self.assertRaises(ValueError):
            s.set_policy(0, 'stop_clearance', 0.08)
        s.partial.pop('return_started')
        backend.pose = lambda: Pose(1, 0, 0)
        with self.assertRaises(RuntimeError):
            s.set_policy(0, 'stop_clearance', 0.08)
