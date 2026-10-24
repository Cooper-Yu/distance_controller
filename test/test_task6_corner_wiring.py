"""Exercise the actual opt-in route dispatch and placement loops with deterministic local I/O."""

import io
import math
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from action_plan import Pose
from motion_console import Console, TeachingBook
from motion_model import Motion, feedback
from task6_route_model import RouteAction
from task6_route_runner import run_actions
import task6_corner_runtime as runtime
from test_task6_corner_runtime import FakeRunner


class CornerWiring(unittest.TestCase):
    def test_actual_front_route_revises_endpoint_and_rejoins_fixed_p11(self):
        r = FakeRunner(front=0.30)
        r.moving_pose = r.pose
        actions = (
            RouteAction('P09', 'path', 0.0, ((0.0, 0.0), (0.0, 0.02))),
            RouteAction('P10', 'path', 0.0, ((0.0, 0.02), (0.04, 0.02), (0.08, 0.02))),
            RouteAction('P10_after_turn', 'turn', math.pi / 2),
            RouteAction(
                'P11',
                'path',
                math.pi / 2,
                ((0.08, 0.02), (0.08, 0.065), (0.08, 0.11), (0.08, 0.155), (0.08, 0.20)),
            ),
        )
        with TemporaryDirectory() as d:
            origin = Pose(0, 0, 0)
            book = TeachingBook(Path(d) / 'points.json', origin, 'odom')
            audit = io.StringIO()
            c = Console(r, book, SimpleNamespace(poll=lambda: (False, None)), audit, 0.20)
            c.measure = lambda: {}
            with (
                patch.object(runtime, '_read_frame', lambda _: r.read_frame()),
                patch.object(runtime.time, 'monotonic', lambda: r.now),
                patch('builtins.print'),
            ):
                run_actions(c, actions, SimpleNamespace(check=lambda: None), 'front')
            points = book.data['points']
            self.assertLess(points['P10']['pose']['x'], 0.06)
            self.assertAlmostEqual(points['P11']['relative_to_p01']['x_m'], 0.08, delta=0.005)
            self.assertAlmostEqual(points['P11']['relative_to_p01']['y_m'], 0.20, delta=0.005)
            self.assertIn('corner_endpoint_accepted', audit.getvalue())
            self.assertIn('corner_connector_plan', audit.getvalue())
            self.assertTrue(any(abs(v[0]) + abs(v[1]) > 0 for v in r.sent))
            self.assertIsNone(r.velocity_pub)
            self.assertEqual(r.sent[-1], (0.0, 0.0, 0.0))

    def test_original_turn_four_cm_guard_still_rejects(self):
        with self.assertRaisesRegex(RuntimeError, '4 cm'):
            feedback(Pose(0, 0, 0), Pose(0.041, 0, 0.1), Motion('turn', angle=1), 0.06, 0.20)


if __name__ == '__main__':
    unittest.main()
