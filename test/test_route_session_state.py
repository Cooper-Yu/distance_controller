"""Fixed-origin reverse geometry and fail-closed live-session state contracts."""

from pathlib import Path
import math
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from action_plan import Pose
from motion_model import Motion
from route_session_state import LiveRouteState, near
from task6_route_model import RouteAction


class LiveRouteStateTest(unittest.TestCase):
    def state(self, origin=Pose(0, 0, 0)):
        self.action = RouteAction('P02', 'path', 0, ((0, 0), (0.04, 0.01), (0.08, 0)))
        return LiveRouteState(origin, (self.action,))

    def complete(self, state, end=Pose(0.08, 0, 0)):
        state.begin_forward(self.action.path_motion(), state.origin)
        state.finish_forward(end)

    def test_reverse_keeps_bend_and_actual_endpoints(self):
        state = self.state()
        self.complete(state, Pose(0.078, 0.001, 0))
        plan = state.preview_back(state.last_stop)
        self.assertEqual(plan.motion.points[0], (0.078, 0.001))
        self.assertEqual(plan.motion.points[1:], tuple(reversed(self.action.points)))
        self.assertEqual(plan.motion.angle, 0)
        self.assertEqual(state.cursor, 1)
        state.begin_back(plan.token, state.last_stop)
        state.finish_back(Pose(0.001, 0, 0))
        self.assertEqual(state.cursor, 0)
        self.assertEqual(state.origin, Pose(0, 0, 0))
        with self.assertRaises(RuntimeError):
            state.begin_back(plan.token, state.last_stop)

    def test_absolute_origin_is_not_rebased(self):
        state = self.state(Pose(3, 4, math.pi / 2))
        self.complete(state, Pose(3, 4.08, math.pi / 2))
        plan = state.preview_back(state.last_stop)
        self.assertTrue(near(plan.target, Pose(3, 4, math.pi / 2)))
        self.assertAlmostEqual(plan.motion.points[0][0], 0.08)
        self.assertAlmostEqual(plan.motion.angle, 0)

    def test_turn_boundary_is_not_reversed(self):
        state = self.state()
        state.actions += (RouteAction('turn', 'turn', 0.1),)
        self.complete(state)
        state.begin_forward(Motion('turn', angle=0.1), state.last_stop)
        state.finish_forward(Pose(0.08, 0, 0.1))
        with self.assertRaisesRegex(RuntimeError, 'across a turn'):
            state.preview_back(state.last_stop)
        self.assertEqual(state.cursor, 2)

    def test_partial_and_failed_return_never_advance(self):
        state = self.state()
        state.begin_forward(self.action.path_motion(), state.origin)
        with self.assertRaises(RuntimeError):
            state.preview_back(state.origin)
        state.fail('Odom stale')
        with self.assertRaises(RuntimeError):
            state.finish_forward(Pose(0.08, 0, 0))
        self.assertEqual(state.cursor, 0)
        state = self.state()
        self.complete(state)
        plan = state.preview_back(state.last_stop)
        state.begin_back(plan.token, state.last_stop)
        state.fail('Obstacle')
        with self.assertRaises(RuntimeError):
            state.finish_back(state.origin)
        self.assertEqual(state.cursor, 1)

    def test_changed_pose_or_preview_token_is_rejected(self):
        state = self.state()
        self.complete(state)
        plan = state.preview_back(state.last_stop)
        with self.assertRaises(RuntimeError):
            state.begin_back('old-session-token', state.last_stop)
        with self.assertRaises(RuntimeError):
            state.begin_back(plan.token, Pose(0.083, 0, 0))
        with self.assertRaises(RuntimeError):
            state.preview_back(Pose(0.09, 0, 0))

    def test_bad_endpoint_latches_fault(self):
        state = self.state()
        self.complete(state)
        plan = state.preview_back(state.last_stop)
        state.begin_back(plan.token, state.last_stop)
        with self.assertRaises(RuntimeError):
            state.finish_back(Pose(0.01, 0, 0))
        self.assertEqual(state.cursor, 1)
        with self.assertRaises(RuntimeError):
            state.preview_back(state.last_stop)

    def test_preview_replacement_invalidates_previous(self):
        state = self.state()
        self.complete(state)
        old = state.preview_back(state.last_stop)
        state.preview_back(state.last_stop)
        with self.assertRaises(RuntimeError):
            state.begin_back(old.token, state.last_stop)

    def test_original_bends_retained_with_subdivided_connections(self):
        state = self.state()
        self.complete(state)
        plan = state.preview_back(state.last_stop)
        self.assertTrue(
            all(
                math.dist(a, b) <= 0.05 + 1e-9
                for a, b in zip(plan.motion.points, plan.motion.points[1:])
            )
        )
        self.assertIn((0.04, 0.01), plan.motion.points)

    def test_nonfinite_and_changed_target_rejected(self):
        with self.assertRaises(ValueError):
            self.state(Pose(float('nan'), 0, 0))
        state = self.state()
        with self.assertRaises(ValueError):
            state.begin_forward(Motion('path', points=((0, 0), (1, 1))), state.origin)


if __name__ == '__main__':
    unittest.main()
