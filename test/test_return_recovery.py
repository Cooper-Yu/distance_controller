"""Stopped return recovery checks fresh feedback and the intended restart, not zero alone."""

import sys
from pathlib import Path
import unittest
from unittest.mock import Mock, patch
from contextlib import redirect_stdout
from io import StringIO

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from action_plan import Pose
from wall_runtime import WallRunner, ReturnClearanceError


class ReturnRecovery(unittest.TestCase):
    def fixture(self):
        r = object.__new__(WallRunner)
        r.node = Mock()
        r.twist = Mock()
        r.send = Mock()
        r.latest = [Pose(0, 0, 0), 0, 0, 0]
        r.body_velocity = (0.0, 0.0, 0.0)
        r.twist_frame = 'base_link'
        r.guarded_command = None
        r.moving_pose = lambda: r.latest[0]
        r.cached_stamp = r.stamp = 0
        r.observations = []
        r.clock = 0.0

        def pump():
            r.clock += 0.1
            r.cached_stamp += 1
            r.stamp += 1

        r.pump = pump
        r.geometry = lambda: ([(-1.0, i * 0.001, -3.14) for i in range(10)], {'rear': 10})
        return r

    def recover(self, r):
        with (
            patch('wall_runtime.time.monotonic', side_effect=lambda: r.clock),
            redirect_stdout(StringIO()),
        ):
            r.recover_return(Pose(-0.1, 0, 0), [(-0.03, 0, 0)])

    def test_clear_stopped_scans_pass_and_release_publisher(self):
        r = self.fixture()
        self.recover(r)
        self.assertGreaterEqual(r.clock, 0.3)
        self.assertIsNone(r.velocity_pub)
        self.assertTrue(all(c.args == ((0.0, 0.0, 0.0),) for c in r.send.call_args_list))

    def test_same_scan_cannot_count_twice(self):
        r = self.fixture()

        def pump():
            r.clock += 0.1
            r.stamp += 1

        r.pump = pump
        with self.assertRaisesRegex(RuntimeError, 'RETURN_RECOVERY_TIMEOUT'):
            self.recover(r)
        self.assertIsNone(r.velocity_pub)

    def test_moving_feedback_cannot_resume(self):
        r = self.fixture()
        r.latest[2] = 0.02
        with self.assertRaisesRegex(RuntimeError, 'TIMEOUT'):
            self.recover(r)

    def test_static_safe_but_restart_unsafe_is_rejected(self):
        r = self.fixture()
        r.geometry = lambda: ([(-0.20, i * 0.001, -3.14) for i in range(10)], {'rear': 10})
        with self.assertRaisesRegex(RuntimeError, 'TIMEOUT'):
            self.recover(r)

    def test_coverage_loss_does_not_mean_clear(self):
        r = self.fixture()
        r.geometry = lambda: ([(-1.0, 0, -3.14)], {'rear': 40})
        with self.assertRaisesRegex(RuntimeError, 'TIMEOUT'):
            self.recover(r)

    def test_new_obstacle_resets_consecutive_confirmation(self):
        r = self.fixture()
        r.geometry = lambda: (
            [(-(0.18 if r.cached_stamp == 3 else 1.0), i * 0.001, -3.14) for i in range(10)],
            {'rear': 10},
        )
        self.recover(r)
        self.assertGreaterEqual(r.clock, 0.6)

    def test_stale_or_cancel_is_not_retried(self):
        for error in (RuntimeError('Odom stale'), KeyboardInterrupt()):
            r = self.fixture()
            r.pump = Mock(side_effect=error)
            with self.assertRaises(type(error)):
                self.recover(r)
            self.assertIsNone(r.velocity_pub)

    def test_retry_stops_child_and_keeps_target(self):
        r = self.fixture()
        r.translate_target = Mock(side_effect=[ReturnClearanceError(), None])
        r.return_scenarios = Mock(return_value=[(-0.03, 0, 0)])
        r.stop_owned = Mock()
        r.recover_return = Mock()
        target = Pose(-0.1, 0, 0)
        r.return_position(None, target)
        self.assertEqual(
            [c.args for c in r.translate_target.call_args_list], [(target,), (target,)]
        )
        r.stop_owned.assert_called_once()
        self.assertEqual(r.last_report['recoveries'], 1)
        self.assertFalse(r.turn_guard)

    def test_repeated_obstacles_exhaust_budget(self):
        r = self.fixture()
        r.translate_target = Mock(side_effect=ReturnClearanceError())
        r.return_scenarios = Mock(return_value=[(-0.03, 0, 0)])
        r.stop_owned = Mock()
        r.recover_return = Mock()
        with self.assertRaisesRegex(RuntimeError, 'RETURN_RECOVERY_BUDGET'):
            r.return_position(None, Pose(-0.1, 0, 0))
        self.assertEqual(r.recover_return.call_count, 2)
        self.assertEqual(r.stop_owned.call_count, 3)
        self.assertFalse(r.turn_guard)


if __name__ == '__main__':
    unittest.main()
