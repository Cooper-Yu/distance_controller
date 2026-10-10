"""Turn recovery retries fixed targets only after a stopped, fresh multi-scan check."""

import sys
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from action_plan import Pose
from turn_recovery import supervised_turn, recover_turn, remaining_clearance
from wall_runtime import TurnClearanceError


class TurnRecovery(unittest.TestCase):
    def runner(self):
        r = SimpleNamespace(
            pose=Mock(return_value=Pose(0, 0, 0)),
            stop_owned=Mock(),
            child=object(),
            rotate_target=Mock(),
            turn_guard=False,
        )
        return r

    def test_retry_retains_target_and_reaps_before_recovery(self):
        r = self.runner()
        target = Pose(0, 0, -1.57)
        speeds = []

        def rotate(t):
            self.assertIs(t, target)
            speeds.append(r.turn_speed)
            if len(speeds) == 1:
                raise TurnClearanceError('near')

        r.rotate_target.side_effect = rotate

        def verify(runner, t):
            self.assertFalse(runner.turn_guard)
            self.assertIsNone(runner.child)
            runner.stop_owned.assert_called_once()

        with patch('turn_recovery.recover_turn', side_effect=verify):
            supervised_turn(r, target)
        self.assertEqual(speeds, [0.2, 0.08])
        self.assertFalse(r.turn_guard)

    def test_repeated_failures_are_bounded(self):
        r = self.runner()
        r.rotate_target.side_effect = TurnClearanceError('blocked')
        with patch('turn_recovery.recover_turn') as recover:
            with self.assertRaisesRegex(RuntimeError, 'BUDGET'):
                supervised_turn(r, Pose(0, 0, 1))
        self.assertEqual(recover.call_count, 2)
        self.assertEqual(r.stop_owned.call_count, 3)
        self.assertFalse(r.turn_guard)

    def test_feedback_fault_is_not_retried(self):
        r = self.runner()
        r.rotate_target.side_effect = RuntimeError('stale')
        with patch('turn_recovery.recover_turn') as recover:
            with self.assertRaisesRegex(RuntimeError, 'stale'):
                supervised_turn(r, Pose(0, 0, 1))
        recover.assert_not_called()
        self.assertFalse(r.turn_guard)

    def test_coverage_missing_is_rejected(self):
        with self.assertRaisesRegex(RuntimeError, 'coverage'):
            remaining_clearance([], dict(front=20, rear=20, left=20, right=20), 1)

    def recovery_runner(self, advancing=True):
        r = SimpleNamespace(
            cached_stamp=1,
            stamp=1,
            latest=(None, 0, 0, 0),
            node=SimpleNamespace(create_publisher=Mock(), destroy_publisher=Mock()),
            twist=object,
            send=Mock(),
            moving_pose=lambda: Pose(0, 0, 0),
            geometry=lambda: ([], {}),
        )

        def pump():
            if advancing:
                r.cached_stamp += 1
                r.stamp += 1

        r.pump = pump
        return r

    def test_distinct_stopped_scans_required(self):
        r = self.recovery_runner()
        with (
            patch('turn_recovery.remaining_clearance', return_value=0.03),
            patch('turn_recovery.time.monotonic', side_effect=[i * 0.05 for i in range(100)]),
        ):
            recover_turn(r, Pose(0, 0, 1))
        self.assertGreaterEqual(r.cached_stamp, 4)
        r.node.destroy_publisher.assert_called_once()
        self.assertIsNone(r.velocity_pub)
        self.assertTrue(all(call.args == ((0.0, 0.0, 0.0),) for call in r.send.call_args_list))

    def test_duplicate_scans_do_not_resume(self):
        r = self.recovery_runner(False)
        with (
            patch('turn_recovery.remaining_clearance', return_value=0.03),
            patch('turn_recovery.time.monotonic', side_effect=[i * 0.05 for i in range(100)]),
        ):
            with self.assertRaisesRegex(RuntimeError, 'TIMEOUT'):
                recover_turn(r, Pose(0, 0, 1))
        self.assertIsNone(r.velocity_pub)

    def test_insufficient_margin_does_not_resume(self):
        r = self.recovery_runner()
        with (
            patch('turn_recovery.remaining_clearance', return_value=0.024),
            patch('turn_recovery.time.monotonic', side_effect=[i * 0.05 for i in range(100)]),
        ):
            with self.assertRaisesRegex(RuntimeError, 'TIMEOUT'):
                recover_turn(r, Pose(0, 0, 1))
        self.assertIsNone(r.velocity_pub)
