"""Bounded post-stop verification, unchanged acceptance and cancellation contracts."""

import io
import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from action_plan import Pose
from motion_model import Motion
from motion_console import Console


class SettlingTests(unittest.TestCase):
    def make_console(self, poses):
        runner = SimpleNamespace(
            max_speed=0.12,
            exclusive=Mock(),
            pose=Mock(side_effect=poses),
            node=SimpleNamespace(create_publisher=Mock()),
            twist=object(),
            velocity_pub=None,
        )
        c = Console(runner, None, None, io.StringIO(), 0.6)
        c.cancel_input = Mock()
        c.follow = Mock()
        c.stop_motion = Mock()
        return c

    def test_small_heading_error_corrects_without_relaxing_acceptance(self):
        c = self.make_console([Pose(0.1, 0, 0.0107), Pose(0.1, 0, 0.005)])
        result = c.finish_motion(Pose(0, 0, 0), Motion('move', x=0.1), 30)
        self.assertEqual(result.yaw, 0.005)
        self.assertEqual(c.follow.call_count, 2)
        self.assertTrue(c.follow.call_args.kwargs['settling'])
        self.assertLessEqual(c.follow.call_args.args[2], 8)
        self.assertIn('"ready": false', c.audit.getvalue())
        self.assertIn('"ready": true', c.audit.getvalue())

    def test_p03_path_snapshot_requires_correction(self):
        c = self.make_console([])
        motion = Motion(
            'path',
            x=1.773003723416002,
            y=-0.012600028526990912,
            points=((0.0, 0.0), (1.773003723416002, -0.012600028526990912)),
        )
        ready, small = c.stopped_check(
            Pose(0, 0, 0), Pose(1.776157506, -0.011624268, math.radians(-0.613841063)), motion, 0
        )
        self.assertFalse(ready)
        self.assertTrue(small)
        ready, small = c.stopped_check(
            Pose(0, 0, 0), Pose(1.776157506, -0.011624268, 0.005), motion, 1
        )
        self.assertTrue(ready and small)

    def test_large_error_never_retries(self):
        for pose in (Pose(0.111, 0, 0), Pose(0.1, 0, 0.02)):
            c = self.make_console([pose])
            with self.assertRaises(RuntimeError):
                c.finish_motion(Pose(0, 0, 0), Motion('move', x=0.1), 30)
            self.assertEqual(c.follow.call_count, 1)

    def test_retries_are_bounded(self):
        c = self.make_console([Pose(0.1, 0, 0.012)] * 3)
        with self.assertRaises(RuntimeError):
            c.finish_motion(Pose(0, 0, 0), Motion('move', x=0.1), 30)
        self.assertEqual(c.follow.call_count, 3)
        self.assertEqual(c.audit.getvalue().count('"event": "settle_retry"'), 2)

    def test_no_correction_after_fault_or_cancel(self):
        for error in (RuntimeError('Odom stale'), KeyboardInterrupt()):
            c = self.make_console([])
            c.follow.side_effect = error
            with self.assertRaises(type(error)):
                c.finish_motion(Pose(0, 0, 0), Motion('move', x=0.1), 30)
            self.assertEqual(c.follow.call_count, 1)
            c.stop_motion.assert_called_once()

    def test_original_budget_not_reset(self):
        c = self.make_console([Pose(0.1, 0, 0.012)])
        with patch('motion_console.time.monotonic', side_effect=[0, 0, 31]):
            with self.assertRaisesRegex(RuntimeError, 'timeout'):
                c.finish_motion(Pose(0, 0, 0), Motion('move', x=0.1), 30)
        self.assertEqual(c.follow.call_count, 1)

    def test_turn_drift_and_disabled_axes_preserve_contract(self):
        c = self.make_console([])
        ready, small = c.stopped_check(
            Pose(0, 0, 0),
            Pose(0.02, 0, -math.pi / 2 + 0.012),
            Motion('turn', angle=-math.pi / 2),
            0,
        )
        self.assertFalse(ready)
        self.assertTrue(small)
        with self.assertRaisesRegex(RuntimeError, 'center drift'):
            c.stopped_check(
                Pose(0, 0, 0), Pose(0.041, 0, -math.pi / 2), Motion('turn', angle=-math.pi / 2), 0
            )
        ready, small = c.stopped_check(
            Pose(0, 0, 0),
            Pose(0.1, 0.1, 0.2),
            Motion('move', x=0.1, heading=False, lateral=False),
            0,
        )
        self.assertTrue(ready and small)


if __name__ == '__main__':
    unittest.main()
