"""Pure epoch and bounded recovery tests; no ROS installation or hardware needed."""

import math
from pathlib import Path
import sys
from types import SimpleNamespace as Namespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from route_feedback_pause import OdomEpoch, RouteFeedbackPause, PauseDisplacement, ScanWindow


def odom(stamp, x=0.0, speed=0.0, frame='odom'):
    return Namespace(
        header=Namespace(
            stamp=Namespace(sec=int(stamp), nanosec=round(stamp % 1 * 1e9)), frame_id=frame
        ),
        child_frame_id='base_link',
        pose=Namespace(
            pose=Namespace(
                position=Namespace(x=x, y=0.0, z=0.0),
                orientation=Namespace(x=0.0, y=0.0, z=0.0, w=1.0),
            )
        ),
        twist=Namespace(
            twist=Namespace(
                linear=Namespace(x=speed, y=0.0, z=0.0), angular=Namespace(x=0.0, y=0.0, z=0.0)
            )
        ),
    )


class EpochTests(unittest.TestCase):
    def test_cached_frame_expires_and_repeated_stamp_does_not_refresh(self):
        source = OdomEpoch()
        source.receive(odom(10), 10)
        self.assertTrue(source.fresh(10.3, int(10.3e9)))
        source.receive(odom(10), 10.34)
        self.assertFalse(source.fresh(10.36, int(10.36e9)))
        self.assertEqual(len(source.history), 1)

    def test_invalid_epoch_is_latched(self):
        for bad in (
            odom(9),
            odom(10.1, x=0.21),
            odom(10.1, x=float('nan')),
            odom(10.1, frame='changed'),
        ):
            with self.subTest(bad=bad):
                source = OdomEpoch()
                source.receive(odom(10), 10)
                source.receive(bad, 10.1)
                self.assertIsNotNone(source.error)
                source.receive(odom(10.2), 10.2)
                self.assertIsNotNone(source.error)

    def test_unchanged_stamp_changed_pose_or_twist_is_fatal(self):
        for bad in (odom(10, x=0.001), odom(10, speed=0.001)):
            source = OdomEpoch()
            source.receive(odom(10), 10)
            source.receive(bad, 10.1)
            self.assertIn('unchanged timestamp', source.error)
            self.assertEqual(source.sample['x'], 0.0)

    def test_recovery_requires_distinct_continuous_stopped_window(self):
        source = OdomEpoch()
        for stamp in (10, 10.1, 10.2):
            source.receive(odom(stamp), stamp)
        self.assertFalse(source.recovered(10, 10.2, int(10.2e9)))
        source.receive(odom(10.31), 10.31)
        self.assertTrue(source.recovered(10, 10.31, int(10.31e9)))
        source.receive(odom(10.4, speed=0.02), 10.4)
        source.receive(odom(10.5), 10.5)
        self.assertFalse(source.recovered(10, 10.5, int(10.5e9)))
        for stamp in (11.0, 11.1, 11.2):
            source.receive(odom(stamp), stamp)
        self.assertFalse(source.recovered(10, 11.2, int(11.2e9)))


class PauseDisplacementTests(unittest.TestCase):
    def field_source(self, stamp, x, y, speed):
        source = OdomEpoch()
        msg = odom(stamp, x=x, speed=speed)
        msg.pose.pose.position.y = y
        source.receive(msg, 10)
        return source.sample

    def test_real_raw_and_filtered_delayed_positions_fit_bounded_pre_zero_travel(self):
        zero_ns = 10_000_000_000
        anchors = dict(
            raw=self.field_source(9.728, -1.6769, 0.8112, 0.1200),
            filtered=self.field_source(9.648, -1.9341, 0.2955, 0.1220),
        )
        guard = PauseDisplacement(anchors, zero_ns, 0.12, 0.12, True)
        raw = self.field_source(10.368, -1.7113, 0.7646, 0.0088)
        filtered = self.field_source(10.408, -1.8631, 0.2969, 0.0019)
        self.assertGreater(
            math.hypot(raw['x'] - anchors['raw']['x'], raw['y'] - anchors['raw']['y']), 0.04
        )
        guard.check('raw', raw)
        guard.check('filtered', filtered)
        self.assertAlmostEqual(guard.latest['raw']['pre_zero_allowance_m'], 0.03264)
        self.assertAlmostEqual(guard.latest['filtered']['pre_zero_allowance_m'], 0.04224)
        frozen = dict(guard.bounds['raw'])
        raw['stamp'] += 4_000_000_000
        guard.check('raw', raw)
        self.assertEqual(guard.bounds['raw'], frozen)
        self.assertAlmostEqual(guard.latest['raw']['pre_zero_allowance_m'], 0.03264)

    def test_queued_pre_zero_frames_only_get_their_source_interval(self):
        source = self.field_source(9.7, 0, 0, 0.12)
        guard = PauseDisplacement({'raw': source}, 10_000_000_000, 0.12, 0.12, True)
        frame = self.field_source(9.8, 0.05, 0, 0.12)
        guard.check('raw', frame)
        self.assertAlmostEqual(guard.latest['raw']['pre_zero_allowance_m'], 0.012)
        frame['x'] = 0.053
        with self.assertRaisesRegex(RuntimeError, 'drift exceeded 4 cm'):
            guard.check('raw', frame)

    def test_post_zero_excess_and_fresh_anchor_still_fail(self):
        for stamp, x in ((9.7, 0.077), (10.0, 0.041)):
            source = self.field_source(stamp, 0, 0, 0.12)
            guard = PauseDisplacement({'raw': source}, 10_000_000_000, 0.12, 0.12, True)
            with self.assertRaisesRegex(RuntimeError, 'drift exceeded 4 cm'):
                guard.check('raw', self.field_source(10.4, x, 0, 0))

    def test_known_stationary_or_released_owner_gets_no_extra_allowance(self):
        for owned, speed in ((True, 0), (False, 0.12)):
            source = self.field_source(9.7, 0, 0, speed)
            guard = PauseDisplacement({'raw': source}, 10_000_000_000, 0.12, 0, owned)
            with self.assertRaisesRegex(RuntimeError, 'drift exceeded 4 cm'):
                guard.check('raw', self.field_source(10.4, 0.041, 0, 0))

    def test_invalid_anchor_or_config_cannot_expand_bound(self):
        source = self.field_source(9.49, 0, 0, 0.12)
        with self.assertRaisesRegex(RuntimeError, 'outside active feedback age'):
            PauseDisplacement({'raw': source}, 10_000_000_000, 0.12, 0.12, True)
        for speed in (0.13, -1, float('nan')):
            with self.assertRaisesRegex(RuntimeError, 'speed bound'):
                PauseDisplacement({}, 10_000_000_000, speed, 0.12, True)

    def test_original_turn_center_guard_is_unchanged(self):
        from action_plan import Pose
        from motion_model import Motion, feedback

        with self.assertRaisesRegex(RuntimeError, 'Turn center drift'):
            feedback(Pose(0, 0, 0), Pose(0.041, 0, 0), Motion('turn', angle=1.57), 0.12, 0.2)


class RecoveryTests(unittest.TestCase):
    def make_pause(self, *, raw_returns=True, owned=True):
        clock = [10.0]
        pause = RouteFeedbackPause.__new__(RouteFeedbackPause)
        pause.sources = {'raw': OdomEpoch(), 'filtered': OdomEpoch()}
        pause.used, pause.pause_id, pause.generation, pause.active = 0.0, 0, 0, False
        commands, events = [], []
        pause.runner = Namespace(
            velocity_pub=object() if owned else None,
            max_speed=0.04,
            guarded_command=((0.0, 0.0, 0.0), 0.0),
            send=lambda command: commands.append((clock[0], command)),
        )
        pause.emit = lambda name, **fields: events.append((name, fields, len(commands)))
        pause.cancel = Mock()
        pause.check_faults = Mock()
        pause.ros_ns = lambda: round(clock[0] * 1e9)
        pause.scan_ready = lambda: True
        pause.scan_recovered = lambda *_: True

        def pump():
            clock[0] += 0.05
            for name, source in pause.sources.items():
                if raw_returns or name != 'raw':
                    source.receive(odom(clock[0]), clock[0])

        pause.original_pump = pump
        return pause, clock, commands, events

    def test_zero_precedes_event_and_dual_source_recovery_resumes(self):
        pause, clock, commands, events = self.make_pause()
        with patch('route_feedback_pause.time.monotonic', side_effect=lambda: clock[0]):
            self.assertTrue(pause.recover())
        self.assertEqual(
            [event[0] for event in events], ['feedback_pause_started', 'feedback_pause_completed']
        )
        self.assertGreater(events[0][2], 0)
        self.assertTrue(all(command == (0.0, 0.0, 0.0) for _, command in commands))
        self.assertGreaterEqual(pause.used, 0.3)
        self.assertEqual(pause.generation, 1)
        self.assertFalse(pause.active)

    def test_one_source_never_returning_cancels_within_five_seconds(self):
        pause, clock, commands, events = self.make_pause(raw_returns=False)
        with patch('route_feedback_pause.time.monotonic', side_effect=lambda: clock[0]):
            with self.assertRaisesRegex(RuntimeError, 'timeout'):
                pause.recover()
        self.assertLess(clock[0] - 10, 5.06)
        self.assertEqual(len(events), 1)
        self.assertEqual(pause.generation, 0)
        self.assertEqual(commands[-1][1], (0.0, 0.0, 0.0))

    def test_cumulative_budget_is_not_reset_between_outages(self):
        pause, clock, _, events = self.make_pause(raw_returns=False)
        pause.used = 9.8
        with patch('route_feedback_pause.time.monotonic', side_effect=lambda: clock[0]):
            with self.assertRaisesRegex(RuntimeError, 'timeout'):
                pause.recover()
        self.assertAlmostEqual(events[0][1]['budget_s'], 0.2)
        self.assertLess(clock[0] - 10, 0.26)

    def test_stopped_boundary_does_not_create_or_publish(self):
        pause, clock, commands, events = self.make_pause(owned=False)
        with patch('route_feedback_pause.time.monotonic', side_effect=lambda: clock[0]):
            pause.recover()
        self.assertFalse(events[0][1]['owns_publisher'])
        self.assertEqual(commands, [])

    def test_cancel_and_identity_fault_abort_without_completion(self):
        for failure in (KeyboardInterrupt, RuntimeError('changed identity')):
            pause, clock, commands, events = self.make_pause()
            pause.check_faults.side_effect = failure
            with patch('route_feedback_pause.time.monotonic', side_effect=lambda: clock[0]):
                with self.assertRaises(
                    type(failure) if isinstance(failure, Exception) else failure
                ):
                    pause.recover()
            self.assertEqual(len(events), 1)
            self.assertEqual(commands[-1][1], (0.0, 0.0, 0.0))

    def test_pause_drift_cannot_accumulate_in_small_steps(self):
        pause, clock, commands, events = self.make_pause()
        for source in pause.sources.values():
            source.receive(odom(clock[0]), clock[0])

        def drifting_pump():
            clock[0] += 0.05
            for source in pause.sources.values():
                source.receive(odom(clock[0], x=(clock[0] - 10) * 0.2), clock[0])

        pause.original_pump = drifting_pump
        with patch('route_feedback_pause.time.monotonic', side_effect=lambda: clock[0]):
            with self.assertRaisesRegex(RuntimeError, 'drift exceeded 4 cm'):
                pause.recover()
        self.assertEqual(len(events), 1)
        self.assertEqual(commands[-1][1], (0.0, 0.0, 0.0))

    def test_future_odom_is_fatal_before_recovery(self):
        pause, clock, _, _ = self.make_pause()
        pause.runner.invalid = None
        pause.runner.child = None
        pause.runner.client = Mock(service_is_ready=lambda: False)
        pause.sources['raw'].receive(odom(clock[0] + 1), clock[0])
        with self.assertRaisesRegex(RuntimeError, 'future odometry'):
            RouteFeedbackPause.check_faults(pause)


class FollowResumeTests(unittest.TestCase):
    def test_pause_resets_controller_state_but_never_extends_deadline(self):
        from action_plan import Pose
        from motion_console import Console
        from motion_model import Motion

        clock = [10.0]
        pause = Namespace(generation=0)
        runner = Namespace(feedback_pause=pause, max_speed=0.04, send=Mock(), moving_pose=Mock())

        def pump():
            pause.generation += 1
            clock[0] += 2

        runner.pump = pump
        console = Console(runner, None, None, None, 0.1)
        console.cancel_input = Mock()
        start = Pose(1.0, 2.0, 0.3)
        points = ((0.0, 0.0), (0.1, 0.0))
        motion = Motion('path', x=0.1, points=points)
        with (
            patch('motion_console.time.monotonic', side_effect=lambda: clock[0]),
            patch('motion_console.PathTracker') as tracker,
            patch('motion_console.HeadingPid') as pid,
        ):
            with self.assertRaisesRegex(RuntimeError, 'Motion timeout'):
                console.follow(start, motion, budget=1.0)
        self.assertEqual(tracker.call_count, 2)
        for call in tracker.call_args_list:
            self.assertEqual(call.args, (start, points, motion.angle))
        pid.return_value.reset.assert_called_once()
        runner.send.assert_not_called()
        runner.moving_pose.assert_not_called()


class ScanWindowTests(unittest.TestCase):
    def test_cached_or_queued_pre_pause_scan_cannot_resume(self):
        scans = ScanWindow()
        scans.observe(int(9.9e9), 10.1, True)
        scans.observe(int(9.95e9), 10.2, True)
        scans.observe(int(9.99e9), 10.41, True)
        self.assertFalse(scans.recovered(10.0, int(10e9), 10.41, int(10.41e9)))

    def test_duplicate_stamp_never_counts_as_three_scans(self):
        scans = ScanWindow()
        for receipt in (10.0, 10.15, 10.31):
            scans.observe(int(10e9), receipt, True)
        self.assertEqual(len(scans.history), 1)
        self.assertFalse(scans.recovered(10.0, int(10e9), 10.31, int(10.31e9)))

    def test_three_valid_distinct_scans_need_continuous_span_and_current_freshness(self):
        scans = ScanWindow()
        for stamp in (10.0, 10.15, 10.31):
            scans.observe(round(stamp * 1e9), stamp, True)
        self.assertTrue(scans.recovered(10.0, int(10e9), 10.31, int(10.31e9)))
        self.assertFalse(scans.recovered(10.0, int(10e9), 10.7, int(10.7e9)))
        scans.observe(int(10.8e9), 10.8, True)
        self.assertFalse(scans.recovered(10.0, int(10e9), 10.8, int(10.8e9)))

    def test_invalid_ranges_reset_scan_confirmation(self):
        scans = ScanWindow()
        for stamp, valid in ((10.0, True), (10.15, True), (10.2, False), (10.31, True)):
            scans.observe(round(stamp * 1e9), stamp, valid)
        self.assertFalse(scans.recovered(10.0, int(10e9), 10.31, int(10.31e9)))


if __name__ == '__main__':
    unittest.main()
