"""READY-only pairing wait never tolerates active, stale, identity or pose faults."""

from dataclasses import asdict
import unittest
from unittest.mock import Mock, patch

import test_current_frame_session as fixtures
from action_plan import Pose
from current_frame_session import command_loop
from current_frame_session_runtime import ReadyPairingError, ReadyPairingWait, StepPermit
from route_prefix_return_plan import ODOM
from wall_runtime import WallFitError


def gap(reason='Odom bracket too wide for scan: 0.165 s', side='time_pairing'):
    return WallFitError('WALL_REFERENCE: ' + reason, dict(side=side, reason=reason))


class PairingGuardTest(unittest.TestCase):
    def setUp(self):
        fixture = fixtures.GuardTest()
        fixture.setUp()
        self.addCleanup(fixture.doCleanups)
        self.guard, self.runner, self.gids = fixture.guard, fixture.runner, fixture.gids

    def test_ready_typed_gap_can_wait_without_latching_or_publishing(self):
        self.runner.scan_pose.side_effect = gap()
        with self.assertRaises(ReadyPairingError):
            self.guard.check(allow_pair_wait=True)
        self.assertIsNone(self.guard.failure)
        self.runner.scan_pose.side_effect = None
        self.guard.check(allow_pair_wait=True)
        self.runner.node.create_publisher.assert_not_called()

    def test_default_and_active_gap_remain_latched(self):
        for active in (False, True):
            self.guard.failure = None
            self.runner.scan_pose.side_effect = None
            if active:
                self.guard.begin_step(0.03)
            self.runner.scan_pose.side_effect = gap()
            with self.assertRaises(WallFitError):
                self.guard.check(allow_pair_wait=active)
            self.assertIsNotNone(self.guard.failure)
            self.runner.scan_pose.side_effect = None
            with self.assertRaises(RuntimeError):
                self.guard.check(allow_pair_wait=True)

    def test_only_exact_typed_bracket_error_is_recoverable(self):
        for error in (
            RuntimeError('WALL_REFERENCE: Odom bracket too wide for scan: 0.165 s'),
            gap('No nearby odom for scan: 0.165 s'),
            gap('No odom history for scan'),
            gap(side='wall_fit'),
            gap('Odom bracket too wide for scan: nan s'),
            gap('Odom bracket too wide for scan: 0.100 s'),
        ):
            self.guard.failure = None
            self.runner.scan_pose.side_effect = error
            with self.assertRaises(RuntimeError) as caught:
                self.guard.check(allow_pair_wait=True)
            self.assertNotIsInstance(caught.exception, ReadyPairingError)
            self.assertIsNotNone(self.guard.failure)

    def test_stale_scan_unknown_tf_geometry_are_not_waited(self):
        for reason in (
            'Required laser scan stale',
            'Required laser geometry/TF unavailable: unknown',
        ):
            self.guard.failure = None
            self.runner.geometry.side_effect = RuntimeError(reason)
            self.runner.scan_pose.side_effect = gap()
            with self.assertRaisesRegex(RuntimeError, reason):
                self.guard.check(allow_pair_wait=True)
            self.assertIsNotNone(self.guard.failure)

    def test_gid_and_publisher_fault_precede_pair_wait(self):
        self.runner.scan_pose.side_effect = gap()
        self.gids[ODOM[1]] = 'cc' * 24
        with self.assertRaisesRegex(RuntimeError, 'identity'):
            self.guard.check(allow_pair_wait=True)
        self.gids[ODOM[1]] = 'bb' * 24
        self.guard.failure = None
        self.runner.node.count_publishers.return_value = 1
        with self.assertRaisesRegex(RuntimeError, 'no cmd_vel'):
            self.guard.check(allow_pair_wait=True)

    def test_pose_and_raw_source_expiry_precede_pair_wait(self):
        self.runner.scan_pose.side_effect = gap()
        self.runner.moving_pose.return_value = Pose(0.006, 0, 0)
        with self.assertRaisesRegex(RuntimeError, 'pose/velocity'):
            self.guard.check(allow_pair_wait=True)
        self.guard.failure = None
        self.runner.moving_pose.return_value = Pose(0, 0, 0)
        self.guard.raw['stamp_ns'] = 499_000_000
        with self.assertRaisesRegex(RuntimeError, 'Raw odometry stale'):
            self.guard.check(allow_pair_wait=True)

    def test_scan_backward_stamp_or_frame_change_latches(self):
        self.guard.check(allow_pair_wait=True)
        self.runner.cached_stamp -= 1
        with self.assertRaisesRegex(RuntimeError, 'chronology/frame'):
            self.guard.check(allow_pair_wait=True)
        self.guard.failure = None
        self.runner.cached_stamp += 1
        self.runner.observer.scan.header.frame_id = 'other'
        with self.assertRaisesRegex(RuntimeError, 'chronology/frame'):
            self.guard.check(allow_pair_wait=True)


class WaitingStateTest(unittest.TestCase):
    def test_requires_three_distinct_new_valid_pairs(self):
        wait = ReadyPairingWait()
        self.assertTrue(wait.failed(100, 10))
        self.assertFalse(wait.paired(100, 10.1))
        self.assertFalse(wait.paired(101, 10.2))
        self.assertFalse(wait.paired(101, 10.3))
        self.assertFalse(wait.paired(102, 10.4))
        self.assertTrue(wait.paired(103, 10.5))
        self.assertEqual(wait.paired_stamps, [101, 102, 103])

    def test_repeated_bad_pair_resets_count_but_not_deadline(self):
        wait = ReadyPairingWait()
        wait.failed(100, 10)
        wait.paired(101, 10.1)
        self.assertFalse(wait.failed(102, 11))
        self.assertEqual(wait.deadline, 12)
        self.assertEqual(wait.paired_stamps, [])
        self.assertFalse(wait.paired(103, 11.1))
        with self.assertRaisesRegex(RuntimeError, '2 seconds'):
            wait.paired(104, 12)

    def test_deadline_before_recovery_and_new_failures_is_hard(self):
        for operation in ('paired', 'failed'):
            wait = ReadyPairingWait()
            wait.failed(1, 1)
            wait.paired(2, 1.1)
            wait.paired(3, 1.2)
            with self.assertRaisesRegex(RuntimeError, '2 seconds'):
                getattr(wait, operation)(4, 3)

    def test_wait_revokes_old_token_discards_step_and_recovers_without_motion(self):
        console, guard, cancellation, permit = Mock(), Mock(), Mock(), StepPermit()
        console.book.data = {'session_id': 'local'}
        guard.anchor = Pose(0, 0, 0)
        state = dict(tick=0, sent=False)
        old_token = []

        def pump():
            state['tick'] += 1
            console.runner.cached_stamp = [10, 11, 12, 12, 13, 14][state['tick'] - 1]

        def check(**kwargs):
            self.assertTrue(kwargs['allow_pair_wait'])
            if state['tick'] == 2:
                raise ReadyPairingError(str(gap()), 11)
            return Pose(0, 0, 0)

        def emit(name, **data):
            if name == 'session_ready':
                old_token.append(data['ready_token'])
                self.assertEqual(data['anchor'], asdict(guard.anchor))

        def poll():
            if state['tick'] == 2 and not state['sent']:
                state['sent'] = True
                return True, 'step ' + old_token[0]
            if state['tick'] == 6 and len(old_token) == 2:
                return True, 'quit'
            return False, None

        console.runner.pump.side_effect = pump
        guard.check.side_effect = check
        console.emit.side_effect = emit
        console.inputs.idle_poll.side_effect = poll
        with patch('current_frame_session.time.monotonic', return_value=10):
            with patch('current_frame_session.run_step') as step:
                command_loop(console, guard, permit, cancellation, 0.03)
                step.assert_not_called()
        self.assertEqual(len(old_token), 2)
        self.assertNotEqual(*old_token)
        events = [(c.args[0], c.kwargs) for c in console.emit.call_args_list]
        waiting = [v for k, v in events if k == 'session_waiting']
        self.assertEqual(len(waiting), 1)
        self.assertEqual(waiting[0]['revoked_ready_token'], old_token[0])
        ready = [v for k, v in events if k == 'session_ready']
        self.assertEqual(ready[1]['recovered_scan_stamps'], [12, 13, 14])
        self.assertTrue(any(k == 'command_rejected' for k, _ in events))
        self.assertEqual(permit.index, 1)
        console.inputs.poll.assert_not_called()


if __name__ == '__main__':
    unittest.main()
