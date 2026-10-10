"""Controlled adjustment must not drive forward, erase progress, or bypass guards."""

from dataclasses import asdict
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from action_plan import Pose, Step
from wall_policy import Policy, FollowAlignment
from wall_session import WallSession
from session_checkpoint import validate_records


class FollowAdjustment(unittest.TestCase):
    def test_alignment_axes_and_distinct_stopped_hold(self):
        gate = FollowAlignment(True, 0, 0.2)
        policy = Policy(follow='left')

        def tick(t, stamp, side=0.02, heading=0, stopped=True, path=0.2):
            return gate.apply(
                policy, {'side_m': side, 'heading_rad': heading}, stamp, stopped, t, path
            )

        self.assertEqual(tick(0, 1), (0, 0.01, 0))
        self.assertEqual(tick(0.1, 2, heading=0.1), (0, 0, 0.08))
        tick(1, 3, side=0)
        tick(2, 3, side=0)
        self.assertFalse(gate.done)
        tick(2.1, 4, side=0, stopped=False)
        self.assertEqual(gate.count, 0)
        tick(3, 5, side=0)
        tick(3.3, 6, side=0)
        self.assertEqual(tick(3.6, 7, side=0), (0, 0, 0))
        self.assertTrue(gate.done)
        self.assertIsNone(tick(3.7, 8))

    def test_limits_sign_and_lost_measurement(self):
        policy = Policy(follow='right')
        gate = FollowAlignment(True, 0, 0.2)
        residual = {'side_m': 0.02, 'heading_rad': 0}
        self.assertEqual(gate.apply(policy, residual, 1, True, 0, 0.2), (0, -0.01, 0))
        for time, path in ((21, 0.2), (1, 0.261)):
            with self.assertRaises(RuntimeError):
                gate.apply(policy, residual, 2, True, time, path)
        gate.hold, gate.count = 1, 2
        gate.invalidate()
        self.assertEqual(gate.count, 0)
        self.assertIsNone(gate.hold)
        with self.assertRaises(ValueError):
            Policy(align_follow_first=True).validate('forward')

    def test_partial_policy_roundtrip_and_resume_keep_original_anchor(self):
        origin = Pose(0, 0, 0)
        actual = Pose(0.25, 0, 0)

        class Backend:
            reference = 0.1007
            last_report = {}

            def pose(self):
                return actual

            def execute(self, step, target):
                self.received = (self.resume_origin, self.resume_path, step)
                self.last_report = {'path_m': 0.3}
                return actual

        backend = Backend()
        step = Step('P03', 'forward', 0.77, asdict(Policy(follow='left', stop='front')))
        session = WallSession(origin, [step], backend, lambda *a: None)
        session.last_actual = actual
        session.partial = {
            'index': 0,
            'revision': 0,
            'step': asdict(step),
            'start': asdict(origin),
            'target': asdict(Pose(0.77, 0, 0)),
            'path_m': 0.27,
            'reference_before': 0.1007,
        }
        for target in (None, 0.04, float('nan')):
            with self.assertRaises(ValueError):
                session.set_policy(0, 'follow_clearance', target)
        session.set_policy(0, 'follow_clearance', 0.0807)
        validate_records({'active': [], 'partial': session.partial}, session.steps)
        self.assertTrue(session.partial['step']['wall']['align_follow_first'])
        self.assertEqual(session.partial['path_m'], 0.27)
        self.assertEqual(backend.reference, 0.1007)
        session.resume()
        self.assertEqual(backend.received[:2], (origin, 0.27))
        self.assertEqual(backend.received[2].wall['follow_clearance'], 0.0807)
        self.assertEqual(session.cursor, 1)
