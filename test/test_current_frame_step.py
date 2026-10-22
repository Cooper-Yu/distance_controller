"""Manual step bounds and stale-input checks; manual observation is never free-space proof."""

import io
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from action_plan import Pose
from current_frame_step import arguments, StepCancellation, move_once
from current_frame_step_runtime import CurrentEpochGuard
from route_laser_guard import GuardDecision


class StepTest(unittest.TestCase):
    def argv(self):
        return [
            '--distance',
            '.05',
            '--odom-publisher-gid',
            'aa' * 24,
            '--raw-odom-publisher-gid',
            'bb' * 24,
            '--output',
            '/tmp/uncreated_step.json',
        ]

    def test_explicit_mode_and_positive_bounded_distance_speed(self):
        with self.assertRaises(SystemExit):
            arguments(self.argv())
        args = arguments([*self.argv(), '--supervised-manual'])
        self.assertEqual(args.distance, 0.05)
        self.assertFalse(args.strict_dry_run)
        for name, value in (
            ('--distance', '.051'),
            ('--distance', '-.01'),
            ('--speed', '.031'),
            ('--speed', 'nan'),
        ):
            with self.assertRaises(SystemExit):
                arguments([*self.argv(), '--supervised-manual', name, value])

    def guard(self):
        guard = CurrentEpochGuard.__new__(CurrentEpochGuard)
        guard.runner = Mock()
        guard.check = Mock()
        guard.laser = Mock(side_effect=lambda value: value)
        guard.motion_started = False
        guard.speed = 0.03
        return guard

    def test_manual_unknown_can_observe_but_stale_geometry_cannot_move(self):
        guard = self.guard()
        command = guard((-0.03, 0.01, 0.001))
        self.assertLessEqual(math.hypot(*command[:2]), 0.0300000001)
        self.assertTrue(guard.motion_started)
        guard = self.guard()
        guard.runner.geometry.side_effect = RuntimeError('Required laser scan stale')
        with self.assertRaisesRegex(RuntimeError, 'scan stale'):
            guard((-0.01, 0, 0))
        guard.laser.assert_not_called()
        self.assertFalse(guard.motion_started)

    def test_pairing_failure_cannot_move_and_zero_does_not_start(self):
        guard = self.guard()
        self.assertEqual(guard((0, 0, 0)), (0, 0, 0))
        self.assertFalse(guard.motion_started)
        guard.runner.scan_pose.side_effect = RuntimeError('scan pairing unavailable')
        with self.assertRaises(RuntimeError):
            guard((-0.01, 0, 0))
        self.assertFalse(guard.motion_started)

    def test_timeout_latches_without_waiting_input(self):
        cancellation = StepCancellation(Mock())
        cancellation.deadline = 10
        with patch('current_frame_step.time.monotonic', return_value=10):
            with self.assertRaisesRegex(RuntimeError, 'timeout'):
                cancellation.poll()

    def test_current_origin_displacement_yaw_and_total_travel_bounds(self):
        guard = self.guard()
        guard.origin = Pose(0, 0, 0)
        guard.motion_started = True
        guard.distance = 0.05
        guard.path_m = 0
        guard.path_pose = Pose(0, 0, 0)
        guard.check_bound(Pose(-0.04, 0, 0))
        with self.assertRaises(RuntimeError):
            guard.check_bound(Pose(-0.061, 0, 0))
        guard.path_m = 0
        with self.assertRaises(RuntimeError):
            guard.check_bound(Pose(-0.04, 0, 0.011))
        guard.path_m = 0.071
        with self.assertRaises(RuntimeError):
            guard.check_bound(Pose(-0.04, 0, 0))

    def test_filtered_duplicate_cannot_refresh_receipt_or_change_content(self):
        guard = self.guard()
        runner = guard.runner
        runner.stamp = 100
        runner.latest = (Pose(0, 0, 0), 1, 0, 0)
        runner.moving_pose.return_value = Pose(0, 0, 0)
        runner.body_velocity = (0, 0, 0)
        runner.frame, runner.twist_frame = 'odom', 'base_link'
        guard.filtered = None
        with patch('current_frame_step_runtime.time.monotonic', return_value=1.1):
            guard.check_filtered()
        runner.latest = (Pose(0, 0, 0), 1.6, 0, 0)
        with patch('current_frame_step_runtime.time.monotonic', return_value=1.6):
            with self.assertRaises(RuntimeError):
                guard.check_filtered()
        runner.moving_pose.return_value = Pose(0.001, 0, 0)
        with self.assertRaises(RuntimeError):
            guard.check_filtered()

    def test_strict_dryrun_unknown_never_calls_move_or_names_p01(self):
        args = SimpleNamespace(
            output=Path('/tmp/not_created.json'),
            distance=0.05,
            speed=0.03,
            strict_dry_run=True,
            supervised_manual=False,
        )
        guard = Mock()
        guard.bind_stopped.return_value = Pose(0, 0, 0)
        guard.laser.assess.return_value = (GuardDecision((0, 0, 0), 'unknown', 'missing ray'), None)
        with (
            patch('current_frame_step.TeachingBook'),
            patch('current_frame_step.Console') as console,
        ):
            output = io.StringIO()
            move_once(args, Mock(), guard, Mock(), output)
            console.return_value.move.assert_not_called()
            console.return_value.record.assert_called_once_with('LOCAL_START')
            self.assertIn('strict_dry_run', output.getvalue())
            self.assertNotIn('local_step_completed', output.getvalue())
