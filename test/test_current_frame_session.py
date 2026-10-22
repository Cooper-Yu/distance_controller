"""Pure fixed-session reference, fail-closed feedback and explicit-permit contracts."""

import io
import json
import math
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from action_plan import Pose
from current_frame_session import (
    SessionBook,
    SessionCancellation,
    arguments,
    command_loop,
    execute,
    run_step,
)
from current_frame_session_runtime import (
    SessionGuard,
    SessionRunner,
    StepPermit,
    backward_path,
    continuous,
)
from current_frame_step_runtime import MANUAL_STEP_BUDGET_S
from route_prefix_return_plan import ODOM
from teaching_path import PathTracker, path_ready


class PathContractTest(unittest.TestCase):
    def test_second_target_uses_current_projection_not_previous_arrival_error(self):
        anchor = Pose(2, 1, 0)
        first, _ = backward_path(anchor, anchor, 0.03)
        actual = Pose(1.973, 1.002, 0.008)
        second, chord = backward_path(anchor, actual, 0.03)
        self.assertAlmostEqual(first.x, -0.03)
        self.assertAlmostEqual(second.x, -0.057)
        self.assertAlmostEqual(second.points[0][0], -0.027)
        self.assertTrue(all(y == 0 for _, y in second.points))
        self.assertEqual(second.angle, 0)
        self.assertAlmostEqual(chord, math.hypot(0.03, 0.002))

    def test_backwards_heading_and_lateral_feedback_for_both_error_signs(self):
        for sign in (-1, 1):
            anchor, actual = Pose(0, 0, 0), Pose(-0.027, sign * 0.002, sign * 0.008)
            motion, _ = backward_path(anchor, actual, 0.03)
            command, ready = PathTracker(anchor, motion.points).feedback(actual, 0.01, 0.1)
            self.assertLess(command[0], 0)
            self.assertLess(sign * command[1], 0)
            self.assertLess(sign * command[2], 0)
            self.assertFalse(ready)

    def test_wrap_boundary_does_not_request_full_turn(self):
        anchor = Pose(1, 2, math.pi - 0.003)
        actual = Pose(1, 2, -math.pi + 0.003)
        motion, _ = backward_path(anchor, actual, 0.03)
        command, _ = PathTracker(anchor, motion.points).feedback(actual, 0.01, 0.1)
        self.assertLess(command[0], 0)
        self.assertLess(command[2], 0)
        self.assertLess(abs(command[2]), 0.02)

    def test_actual_chord_limit_not_just_requested_distance(self):
        with self.assertRaisesRegex(RuntimeError, 'chord'):
            backward_path(Pose(0, 0, 0), Pose(0, 0.001, 0), 0.05)
        self.assertEqual(backward_path(Pose(0, 0, 0), Pose(0, 0, 0), 0.05)[1], 0.05)

    def test_entry_rejects_heading_lateral_nonfinite_and_out_of_range(self):
        for pose in (Pose(0, 0.0101, 0), Pose(0, 0, math.radians(1.01))):
            with self.assertRaises(RuntimeError):
                backward_path(Pose(0, 0, 0), pose, 0.03)
        for value in (0, -0.01, 0.051, math.inf, math.nan):
            with self.assertRaises(ValueError):
                backward_path(Pose(0, 0, 0), Pose(0, 0, 0), value)

    def test_endpoint_is_two_dimensional_norm_and_fixed_yaw(self):
        anchor = Pose(0, 0, 0)
        motion, _ = backward_path(anchor, anchor, 0.03)
        self.assertTrue(path_ready(anchor, Pose(-0.028, 0.002, 0.009), motion.points))
        self.assertFalse(path_ready(anchor, Pose(-0.026, 0.004, 0), motion.points))
        self.assertFalse(path_ready(anchor, Pose(-0.03, 0, 0.011), motion.points))


class GuardTest(unittest.TestCase):
    def setUp(self):
        self.clock = patch('current_frame_step_runtime.time.monotonic', return_value=10.0)
        self.clock.start()
        self.addCleanup(self.clock.stop)
        self.runner = Mock()
        self.gids = {ODOM[0]: 'aa' * 24, ODOM[1]: 'bb' * 24}
        self.runner.node.get_publishers_info_by_topic.side_effect = lambda topic: [
            SimpleNamespace(endpoint_gid=bytes.fromhex(self.gids[topic]))
        ]
        self.runner.node.count_publishers.return_value = 0
        self.runner.node.get_clock().now.return_value.nanoseconds = 1_000_000_000
        self.runner.stamp = self.runner.cached_stamp = 1_000_000_000
        self.runner.observer.scan.header.frame_id = 'laser'
        self.runner.latest = (Pose(0, 0, 0), 10.0, 0.0, 0.0)
        self.runner.moving_pose.return_value = Pose(0, 0, 0)
        self.runner.body_velocity = (0.0, 0.0, 0.0)
        self.runner.frame, self.runner.twist_frame = 'odom', 'base_link'
        self.guard = SessionGuard(self.runner, dict(self.gids), 0.01, Mock())
        self.guard.raw = dict(
            pose=dict(x=0, y=0, yaw=0),
            frames=('odom', 'base_link'),
            velocity=(0, 0, 0),
            stamp_ns=1_000_000_000,
            receipt=10.0,
        )
        self.guard.anchor = self.guard.raw_anchor = Pose(0, 0, 0)
        self.guard.origin = self.guard.raw_origin = Pose(0, 0, 0)
        self.guard.idle_pose = self.guard.idle_raw = Pose(0, 0, 0)

    def test_ready_no_publisher_and_scan_pair_checked_every_time(self):
        self.guard.check()
        self.guard.check()
        self.assertEqual(self.runner.geometry.call_count, 2)
        self.assertEqual(self.runner.scan_pose.call_count, 2)
        self.runner.node.create_publisher.assert_not_called()
        self.runner.node.count_publishers.return_value = 1
        with self.assertRaisesRegex(RuntimeError, 'no cmd_vel'):
            self.guard.check()

    def test_gid_failure_latches_even_after_identity_recovers(self):
        self.gids[ODOM[1]] = 'cc' * 24
        with self.assertRaisesRegex(RuntimeError, 'identity'):
            self.guard.check()
        self.gids[ODOM[1]] = 'bb' * 24
        with self.assertRaisesRegex(RuntimeError, 'identity'):
            self.guard.check()

    def test_stale_scan_failure_latches(self):
        self.runner.geometry.side_effect = RuntimeError('scan stale')
        with self.assertRaisesRegex(RuntimeError, 'scan stale'):
            self.guard.check()
        self.runner.geometry.side_effect = None
        with self.assertRaisesRegex(RuntimeError, 'scan stale'):
            self.guard.check()

    def test_ready_pose_shift_cannot_become_next_step_origin(self):
        self.runner.moving_pose.return_value = Pose(0.006, 0, 0)
        with self.assertRaisesRegex(RuntimeError, 'Stopped local'):
            self.guard.begin_step(0.03)
        self.assertFalse(self.guard.active)

    def test_step_origin_changes_session_anchor_does_not(self):
        actual = Pose(-0.027, 0.002, 0.008)
        self.runner.moving_pose.return_value = actual
        self.guard.idle_pose = actual
        motion, start = self.guard.begin_step(0.03)
        self.assertEqual(start, actual)
        self.assertEqual(self.guard.origin, actual)
        self.assertEqual(self.guard.anchor, Pose(0, 0, 0))
        self.assertAlmostEqual(motion.x, -0.057)
        self.assertFalse(self.guard.motion_started)
        self.assertEqual(self.guard.path_m, 0)

    def test_fixed_heading_bound_is_not_rebased_on_step_start(self):
        self.guard.begin_step(0.03)
        self.guard.motion_started = True
        self.guard.origin = Pose(0, 0, 0.009)
        with self.assertRaisesRegex(RuntimeError, 'Fixed session heading'):
            self.guard.check_bound(Pose(-0.01, 0, 0.036))

    def test_first_command_chord_rechecked_after_small_precommand_shift(self):
        self.guard.begin_step(0.05)
        self.runner.stamp += 50_000_000
        self.runner.moving_pose.return_value = Pose(0.001, 0, 0)
        with self.assertRaisesRegex(RuntimeError, 'first-command.*chord'):
            self.guard.check()
        self.assertFalse(self.guard.motion_started)

    def test_active_preserves_prefirst_stopped_and_travel_bounds(self):
        self.guard.begin_step(0.03)
        self.runner.stamp += 50_000_000
        self.runner.body_velocity = (0.02, 0, 0)
        with self.assertRaisesRegex(RuntimeError, 'before first'):
            self.guard.check()
        self.guard.failure = None
        self.guard.motion_started = True
        self.guard.path_m = 0.051
        self.runner.stamp += 50_000_000
        self.runner.body_velocity = (0, 0, 0)
        with self.assertRaisesRegex(RuntimeError, 'total travel'):
            self.guard.check()

    def test_source_gap_and_yaw_jump_not_reclassified_as_wrap(self):
        for pose, dt in ((Pose(0, 0, 0), 0.501), (Pose(0, 0, 0.02), 0.05)):
            with self.assertRaises(RuntimeError):
                continuous(Pose(0, 0, 0), pose, dt, 0.01)
        continuous(Pose(0, 0, math.pi - 0.001), Pose(0, 0, -math.pi + 0.001), 0.05, 0.01)

    def test_raw_fixed_heading_and_source_staleness_rejected(self):
        self.guard.raw['pose']['yaw'] = 0.036
        with self.assertRaisesRegex(RuntimeError, 'raw session heading'):
            self.guard.check()
        self.guard.failure = None
        self.guard.raw['pose']['yaw'] = 0
        self.guard.raw['stamp_ns'] = 499_000_000
        with self.assertRaisesRegex(RuntimeError, 'Raw odometry stale'):
            self.guard.check()


class ProtocolTest(unittest.TestCase):
    def test_tokens_single_use_and_never_reused_after_completion(self):
        permit = StepPermit()
        token = permit.ready()
        self.assertFalse(permit.take('step'))
        self.assertFalse(permit.take('step other'))
        self.assertTrue(permit.take('step ' + token))
        self.assertFalse(permit.take('step ' + token))
        with self.assertRaises(RuntimeError):
            permit.ready()
        permit.finish()
        next_token = permit.ready()
        self.assertNotEqual(token, next_token)
        self.assertFalse(permit.take('step ' + token))
        self.assertEqual(permit.index, 2)

    def test_total_deadline_not_reset_when_step_deadline_cleared(self):
        with patch('current_frame_session.time.monotonic', return_value=100):
            cancel = SessionCancellation(Mock(), 20)
            cancel.step_deadline = 100 + MANUAL_STEP_BUDGET_S
        with patch('current_frame_session.time.monotonic', return_value=114.999):
            cancel.check()
        with patch('current_frame_session.time.monotonic', return_value=115):
            with self.assertRaisesRegex(RuntimeError, 'step timeout'):
                cancel.check()
            cancel.step_deadline = None
            cancel.check()
        with patch('current_frame_session.time.monotonic', return_value=120):
            with self.assertRaisesRegex(RuntimeError, 'session deadline'):
                cancel.check()

    def test_queued_duplicate_does_not_execute_next_step(self):
        permit, console, guard, cancel = StepPermit(), Mock(), Mock(), Mock()
        console.book.data = {'session_id': 'local'}
        guard.anchor = guard.check.return_value = Pose(0, 0, 0)
        console.inputs.idle_poll.side_effect = [
            (False, None),
            (True, 'step fixed'),
            (True, 'step fixed'),
            (True, 'quit'),
        ]
        with patch(
            'current_frame_session_runtime.uuid.uuid4', return_value=SimpleNamespace(hex='fixed')
        ):
            with patch('current_frame_session.run_step') as step:
                step.side_effect = lambda *args: permit.finish()
                command_loop(console, guard, permit, cancel, 0.03)
                self.assertEqual(step.call_count, 1)
        self.assertEqual(permit.index, 2)

    def test_run_step_keeps_fixed_reference_options_and_records_before_ready(self):
        console, guard, cancel, permit = Mock(), Mock(), Mock(), StepPermit()
        permit.take('step ' + permit.ready())
        console.book.data = {'session_id': 'local'}
        guard.anchor = Pose(0, 0, 0)
        motion, _ = backward_path(guard.anchor, guard.anchor, 0.03)
        guard.begin_step.return_value = motion, guard.anchor
        guard.finish_step.return_value = Pose(-0.03, 0, 0)
        with patch('current_frame_session.record_stop', return_value=Pose(-0.03, 0, 0)) as record:
            run_step(console, guard, permit, cancel, 0.03)
        console.move.assert_called_once_with(motion, reconnect=False, use_path_length=True)
        record.assert_called_once()
        self.assertEqual(permit.index, 2)
        self.assertFalse(permit.active)
        self.assertIsNone(cancel.step_deadline)
        self.assertNotIn('session_ready', [call.args[0] for call in console.emit.call_args_list])

    def test_failure_never_finishes_permit_or_clears_step_deadline(self):
        console, guard, cancel, permit = Mock(), Mock(), Mock(), StepPermit()
        permit.take('step ' + permit.ready())
        guard.begin_step.side_effect = RuntimeError('epoch')
        with patch('current_frame_session.time.monotonic', return_value=100):
            with self.assertRaisesRegex(RuntimeError, 'epoch'):
                run_step(console, guard, permit, cancel, 0.03)
        self.assertTrue(permit.active)
        self.assertEqual(permit.index, 1)
        self.assertEqual(cancel.step_deadline, 115)
        console.move.assert_not_called()

    def test_book_never_names_or_reloads_p01(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'local.json'
            book = SessionBook(path, Pose(1, 2, 3), 'odom')
            book.record('LOCAL_SESSION_START', book.origin, 1, {})
            data = json.loads(path.read_text())
            self.assertFalse(data['historical_p01'])
            self.assertFalse(data['resumable'])
            self.assertIn('relative_to_session', data['points']['LOCAL_SESSION_START'])
            self.assertNotIn('relative_to_p01', data['points']['LOCAL_SESSION_START'])
            with self.assertRaises(FileExistsError):
                SessionBook(path, Pose(0, 0, 0), 'odom')

    def test_first_fault_preserved_if_cleanup_also_fails(self):
        args = SimpleNamespace(
            speed=0.01,
            max_seconds=10,
            odom_publisher_gid='aa' * 24,
            raw_odom_publisher_gid='bb' * 24,
        )
        audit = io.StringIO()
        with (
            patch('current_frame_session.SessionRunner'),
            patch('current_frame_session.signal.signal'),
        ):
            with patch('current_frame_session.SessionGuard') as guard:
                guard.return_value.bind_session.side_effect = RuntimeError('first epoch fault')
                with patch(
                    'current_frame_session.close_runner', side_effect=RuntimeError('cleanup fault')
                ):
                    with self.assertRaisesRegex(RuntimeError, 'first epoch fault'):
                        execute(args, audit)
        events = [json.loads(line) for line in audit.getvalue().splitlines()]
        self.assertEqual(events[-1]['event'], 'session_failed')
        self.assertEqual(events[-1]['reason'], 'first epoch fault')

    def test_cli_requires_explicit_manual_and_finite_bounded_parameters(self):
        argv = [
            '--step-distance',
            '.03',
            '--output',
            '/tmp/new.json',
            '--odom-publisher-gid',
            'aa' * 24,
            '--raw-odom-publisher-gid',
            'bb' * 24,
        ]
        with self.assertRaises(SystemExit):
            arguments(argv)
        self.assertEqual(arguments([*argv, '--supervised-manual']).speed, 0.01)
        for key, value in (
            ('--step-distance', '.051'),
            ('--step-distance', '0'),
            ('--speed', '.031'),
            ('--speed', 'nan'),
            ('--max-seconds', '0'),
            ('--max-seconds', '1801'),
        ):
            with self.assertRaises(SystemExit):
                arguments([*argv, '--supervised-manual', key, value])


class FilteredCallbackTest(unittest.TestCase):
    def make_runner(self):
        from collections import deque

        runner = SessionRunner.__new__(SessionRunner)
        runner.session_sample = None
        runner.session_speed = 0.01
        runner.invalid = runner.stamp = runner.frame = runner.latest = runner.last_raw = None
        runner.continuous_yaw = 0.0
        runner.odom_history = deque(maxlen=200)
        runner.node = Mock()
        return runner

    def deliver(
        self, runner, stamp, pose, *, velocity=0.0, frame='odom', child='base_link', age=0.0
    ):
        from nav_msgs.msg import Odometry

        message = Odometry()
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
        message.header.frame_id, message.child_frame_id = frame, child
        message.pose.pose.position.x, message.pose.pose.position.y = float(pose.x), float(pose.y)
        message.pose.pose.orientation.z = math.sin(pose.yaw / 2)
        message.pose.pose.orientation.w = math.cos(pose.yaw / 2)
        message.twist.twist.linear.x = float(velocity)
        runner.node.get_clock().now.return_value.nanoseconds = stamp + int(age * 1e9)
        runner.receive_pose(message)  # Actual SessionRunner -> WallRunner -> PlannedRunner chain.

    def test_two_seconds_exclusive_with_continuous_actual_callbacks_is_valid(self):
        runner = self.make_runner()
        # No guard.check occurs here, exactly as during exclusive()'s continuous spin.
        for index in range(41):
            self.deliver(runner, 1_000_000_000 + index * 50_000_000, Pose(0, 0, 0))
        self.assertIsNone(runner.invalid)
        self.assertEqual(runner.session_sample[0], 3_000_000_000)

    def test_real_source_gap_in_actual_callbacks_still_invalidates(self):
        runner = self.make_runner()
        self.deliver(runner, 1_000_000_000, Pose(0, 0, 0))
        self.deliver(runner, 1_501_000_000, Pose(0, 0, 0))
        self.assertIn('gap/pose jump', runner.invalid)
        frozen = runner.session_sample
        self.deliver(runner, 1_551_000_000, Pose(0, 0, 0))
        self.assertEqual(runner.session_sample, frozen)
        self.assertIsNotNone(runner.invalid)

    def test_mid_exclusive_pose_jump_cannot_hide_behind_later_valid_pose(self):
        runner = self.make_runner()
        self.deliver(runner, 1_000_000_000, Pose(0, 0, 0))
        self.deliver(runner, 1_050_000_000, Pose(0.006, 0, 0))
        self.assertIn('pose jump', runner.invalid)
        self.deliver(runner, 1_100_000_000, Pose(0, 0, 0))
        self.assertIsNotNone(runner.invalid)

    def test_same_stamp_pose_or_velocity_change_then_restore_stays_invalid(self):
        for pose, velocity in ((Pose(0.001, 0, 0), 0.0), (Pose(0, 0, 0), 0.001)):
            runner = self.make_runner()
            self.deliver(runner, 1_000_000_000, Pose(0, 0, 0))
            self.deliver(runner, 1_000_000_000, pose, velocity=velocity)
            self.assertIn('same-stamp', runner.invalid)
            self.deliver(runner, 1_050_000_000, Pose(0, 0, 0))
            self.assertIsNotNone(runner.invalid)

    def test_duplicate_preserves_first_receipt(self):
        runner = self.make_runner()
        with patch('action_runtime.time.monotonic', return_value=10.0):
            self.deliver(runner, 1_000_000_000, Pose(0, 0, 0))
        with patch('action_runtime.time.monotonic', return_value=10.2):
            self.deliver(runner, 1_000_000_000, Pose(0, 0, 0))
        self.assertEqual(runner.latest[1], 10.0)
        self.assertEqual(runner.session_sample[5], 10.0)

    def test_callback_frame_and_child_change_cannot_hide_before_next_guard_check(self):
        for fields in ({'frame': 'map'}, {'child': 'other_base'}):
            runner = self.make_runner()
            self.deliver(runner, 1_000_000_000, Pose(0, 0, 0))
            self.deliver(runner, 1_050_000_000, Pose(0, 0, 0), **fields)
            self.assertIn('frame', runner.invalid.lower())
            self.deliver(runner, 1_100_000_000, Pose(0, 0, 0))
            self.assertIsNotNone(runner.invalid)

    def test_source_age_nonpositive_and_backward_stamp_rejected_in_callback(self):
        for stamp, age in ((0, 0.0), (1_000_000_000, 0.501), (1_000_000_000, -0.101)):
            runner = self.make_runner()
            self.deliver(runner, stamp, Pose(0, 0, 0), age=age)
            self.assertIn('timestamp', runner.invalid)
        runner = self.make_runner()
        self.deliver(runner, 1_000_000_000, Pose(0, 0, 0))
        self.deliver(runner, 999_000_000, Pose(0, 0, 0))
        self.assertIn('backward', runner.invalid)

    def test_real_callback_wrap_is_small_continuous_yaw_change(self):
        runner = self.make_runner()
        self.deliver(runner, 1_000_000_000, Pose(0, 0, math.pi - 0.001))
        self.deliver(runner, 1_050_000_000, Pose(0, 0, -math.pi + 0.001))
        self.assertIsNone(runner.invalid)
        self.assertAlmostEqual(runner.latest[0].yaw, math.pi + 0.001)
