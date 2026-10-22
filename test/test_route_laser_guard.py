"""Directional geometry, unknown coverage and actual Console injection contracts."""

from dataclasses import replace
import io
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from action_plan import Pose
from motion_console import Console
from motion_model import Motion
from route_laser_guard import (
    GuardPolicy,
    ScanSnapshot,
    ZERO,
    body_displacement,
    evaluate,
    predicted_gap,
    receding_translation,
    sweep_samples,
)
from route_laser_runtime import (
    LaserGuardError,
    RouteLaserGuard,
    ShadowRouteLaserGuard,
    current_points,
    scheduled_rays,
)
from wall_geometry import point_clearance, scan_points


RAYS = tuple((i * math.pi / 360, True) for i in range(720))


def scene(*points, rays=RAYS):
    return ScanSnapshot(tuple(points), rays)


class GeometryTests(unittest.TestCase):
    def test_exact_mixed_body_motion(self):
        x, y, angle = body_displacement((0.12, 0.03, 0.6), 0.5)
        self.assertAlmostEqual(x, 0.12 * math.sin(0.3) / 0.6 - 0.03 * (1 - math.cos(0.3)) / 0.6)
        self.assertAlmostEqual(y, 0.12 * (1 - math.cos(0.3)) / 0.6 + 0.03 * math.sin(0.3) / 0.6)
        self.assertAlmostEqual(angle, 0.3)
        self.assertEqual(body_displacement((0.1, -0.2, 0), 0.5), (0.05, -0.1, 0))

    def test_sample_error_bound_covers_dense_reference(self):
        policy, velocity, point = GuardPolicy(), (0.12, 0.03, 0.6), (0.21, 0.16)
        samples, error, bound = sweep_samples(velocity, policy)
        self.assertLessEqual(error, 0.001 + 1e-12)
        predicted = predicted_gap(point, point_clearance(*point), samples, error, bound, policy)
        dense = math.inf
        for i in range(2001):
            x, y, yaw = body_displacement(velocity, 0.5 * i / 2000)
            c, s = math.cos(yaw), math.sin(yaw)
            px, py = point[0] - x, point[1] - y
            dense = min(dense, point_clearance(c * px + s * py, -s * px + c * py))
        self.assertLessEqual(predicted, dense + 1e-12)
        self.assertLessEqual(dense - predicted, 0.0011)

    def test_back_away_from_close_point_without_deleting_it(self):
        snapshot = scene((0.18, 0.0))
        decision = evaluate(snapshot, (-0.02, 0, 0), ZERO)
        self.assertEqual(decision.state, 'clear')
        self.assertEqual(decision.command, (-0.02, 0, 0))
        self.assertEqual(snapshot.points, ((0.18, 0.0),))
        self.assertEqual(evaluate(snapshot, (0.02, 0, 0), ZERO).state, 'blocked')

    def test_back_away_still_checks_rear_obstacle(self):
        result = evaluate(scene((0.18, 0.0), (-0.18, 0.0)), (-0.02, 0, 0), ZERO)
        self.assertEqual(result.state, 'blocked')
        self.assertEqual(result.limiting_point, (-0.18, 0.0))

    def test_negative_model_gap_never_auto_escapes(self):
        decision = evaluate(scene((0.16, 0.0)), (-0.02, 0, 0), ZERO)
        self.assertEqual(decision.state, 'blocked')
        self.assertLess(decision.predicted_gap_m, 0)

    def test_measured_approach_is_not_excused_by_reverse_command(self):
        decision = evaluate(scene((0.20, 0.0)), (-0.02, 0, 0), (0.04, 0, 0))
        self.assertEqual(decision.state, 'blocked')
        self.assertIn('Measured', decision.reason)

    def test_soft_margin_scales_direction_and_rechecks_hard_margin(self):
        decision = evaluate(scene((0.20, 0.0)), (0.04, 0.004, 0), ZERO)
        self.assertEqual(decision.state, 'slowed')
        self.assertLessEqual(decision.scale, 0.5)
        self.assertEqual(decision.command, tuple(v * decision.scale for v in (0.04, 0.004, 0)))
        self.assertGreaterEqual(decision.predicted_gap_m, 0.02)

    def test_far_irregular_single_return_does_not_require_a_wall_fit(self):
        self.assertEqual(evaluate(scene((0.5, 0.17)), (0.04, 0, 0), ZERO).state, 'clear')

    def test_lateral_diagonal_and_rotation_check_body_corners(self):
        for point, velocity in (
            ((0.0, 0.17), (0, 0.02, 0)),
            ((0.18, 0.14), (0.02, 0.02, 0)),
            ((0.17, 0.145), (0, 0, 0.1)),
            ((0.17, -0.145), (0, 0, -0.1)),
        ):
            with self.subTest(point=point, velocity=velocity):
                self.assertEqual(evaluate(scene(point), velocity, ZERO).state, 'blocked')

    def test_close_rotation_has_no_translation_exemption(self):
        self.assertTrue(receding_translation((0.18, 0.0), (-0.02, 0, 0)))
        self.assertFalse(receding_translation((0.18, 0.0), (-0.02, 0, 0.001)))

    def test_zero_command_is_allowed_even_with_no_feedback(self):
        self.assertEqual(evaluate(scene(), ZERO, (math.nan, 0, 0)).command, ZERO)


class CoverageTests(unittest.TestCase):
    def test_unknown_ahead_blocks_but_unknown_behind_does_not_imply_free_ahead(self):
        front = scene((0.6, 0.0), rays=((0.0, False), *RAYS[1:]))
        self.assertEqual(evaluate(front, (0.03, 0, 0), ZERO).state, 'unknown')
        behind = list(RAYS)
        behind[360] = (math.pi, False)
        self.assertEqual(
            evaluate(scene((0.6, 0.0), rays=tuple(behind)), (0.03, 0, 0), ZERO).state, 'clear'
        )
        self.assertEqual(
            evaluate(scene((0.6, 0.0), rays=tuple(behind)), (-0.03, 0, 0), ZERO).state, 'unknown'
        )

    def test_any_rotation_requires_all_direction_coverage(self):
        rays = list(RAYS)
        rays[90] = (math.pi / 4, False)
        self.assertEqual(
            evaluate(scene((0.6, 0.0), rays=tuple(rays)), (0, 0, 0.1), ZERO).state, 'unknown'
        )

    def test_partial_scan_and_diagonal_holes_are_unknown(self):
        for rays, velocity in (
            (RAYS[180:541], (0.03, 0, 0)),
            (RAYS[:85] + RAYS[96:], (0.02, 0.02, 0)),
        ):
            with self.subTest(velocity=velocity):
                self.assertEqual(
                    evaluate(scene((0.6, 0.0), rays=rays), velocity, ZERO).state, 'unknown'
                )

    def test_missing_invalid_geometry_and_nonfinite_velocity_are_unknown(self):
        for snapshot, candidate, measured in (
            (scene(), (0.03, 0, 0), ZERO),
            (scene((0.6, math.nan)), (0.03, 0, 0), ZERO),
            (scene((0.6, 0.0)), (math.inf, 0, 0), ZERO),
            (scene((0.6, 0.0)), (0.03, 0, 0), (math.nan, 0, 0)),
        ):
            self.assertEqual(evaluate(snapshot, candidate, measured).state, 'unknown')

    def test_invalid_policy_rejected(self):
        for policy in (
            replace(GuardPolicy(), hard_m=-1),
            replace(GuardPolicy(), max_ray_gap_rad=0.1),
        ):
            with self.assertRaises(ValueError):
                evaluate(scene((0.6, 0.0)), (0.03, 0, 0), ZERO, policy)


def scan_fixture():
    scan = SimpleNamespace(
        angle_min=-math.pi,
        angle_increment=math.pi / 360,
        range_min=0.15,
        range_max=10.0,
        ranges=[1.0] * 720,
    )
    transform = SimpleNamespace(
        rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        translation=SimpleNamespace(x=0.02, y=0.0, z=0.173),
    )
    return scan, transform


class AdapterTests(unittest.TestCase):
    def runner(self):
        scan, transform = scan_fixture()
        runner = SimpleNamespace(
            moving_pose=Mock(return_value=Pose(0.02, 0, 0)),
            geometry=Mock(return_value=scan_points(scan, transform)),
            scan_pose=Mock(return_value=Pose(0, 0, 0)),
            frame='odom',
            twist_frame='base_link',
            observer=SimpleNamespace(scan=scan),
            body_velocity=ZERO,
            cached_stamp=1,
            stamp=2,
        )
        return runner, transform

    def test_motion_compensation_preserves_every_point(self):
        points = ((1.0, 0.0, 0.0), (0.0, 1.0, math.pi / 2))
        result = current_points(points, Pose(0, 0, math.pi / 2), Pose(0.1, 0.2, 0))
        self.assertEqual(len(result), len(points))
        self.assertAlmostEqual(result[0][0], -0.1)
        self.assertAlmostEqual(result[0][1], 0.8)
        self.assertAlmostEqual(result[1][0], -1.1)
        self.assertAlmostEqual(result[1][1], -0.2)

    def test_invalid_returns_remain_unknown_rays(self):
        scan, transform = scan_fixture()
        scan.ranges[:4] = [math.nan, math.inf, 0.1, 11.0]
        rays = scheduled_rays(scan, transform, 0.1)
        self.assertEqual(len(rays), 720)
        self.assertFalse(any(valid for _, valid in rays[:4]))
        self.assertTrue(all(valid for _, valid in rays[4:]))

    def test_adapter_uses_real_scan_geometry_without_wall_fit(self):
        runner, transform = self.runner()
        emit = Mock()
        guard = RouteLaserGuard(runner, emit)
        with patch('route_laser_runtime.scan_transform', return_value=transform):
            snapshot = guard.snapshot()
            self.assertEqual(len(snapshot.points), 720)
            self.assertAlmostEqual(snapshot.points[360][0], 1.0)
            self.assertEqual(guard((0.03, 0, 0)), (0.03, 0, 0))
        emit.assert_called_once()

    def test_scan_pairing_tf_stale_or_frame_failure_stops(self):
        for source in ('geometry', 'scan_pose', 'tf', 'frame'):
            runner, transform = self.runner()
            emit = Mock()
            if source in ('geometry', 'scan_pose'):
                getattr(runner, source).side_effect = RuntimeError('feedback unavailable')
            if source == 'frame':
                runner.twist_frame = 'other'
            with patch(
                'route_laser_runtime.scan_transform',
                return_value=transform,
                side_effect=Exception('TF unavailable') if source == 'tf' else None,
            ):
                with self.assertRaisesRegex(LaserGuardError, 'UNKNOWN'):
                    RouteLaserGuard(runner, emit)((0.03, 0, 0))
            self.assertEqual(emit.call_args.kwargs['decision']['state'], 'unknown')

    def test_zero_bypasses_all_sensor_and_audit_operations(self):
        runner, _ = self.runner()
        guard = RouteLaserGuard(runner)
        self.assertEqual(guard(ZERO), ZERO)
        runner.moving_pose.assert_not_called()

    def test_shadow_unknown_keeps_command_and_labels_non_enforcement(self):
        runner, _ = self.runner()
        runner.geometry.side_effect = RuntimeError('scan stale')
        emit = Mock()
        guard = ShadowRouteLaserGuard(runner, emit)
        command = (0.03, 0.002, 0.01)
        self.assertEqual(guard(command), command)
        report = emit.call_args.kwargs
        self.assertFalse(report['enforced'])
        self.assertEqual(report['mode'], 'shadow')
        self.assertEqual(report['decision']['state'], 'unknown')
        self.assertEqual(report['decision']['command'], ZERO)
        self.assertEqual(report['applied_command'], command)

    def test_shadow_blocked_and_slowed_are_diagnostic_only(self):
        runner, _ = self.runner()
        emit = Mock()
        guard = ShadowRouteLaserGuard(runner, emit)
        for snapshot in (scene((0.16, 0.0)), scene((0.20, 0.0))):
            with (
                self.subTest(snapshot=snapshot),
                patch.object(guard, 'snapshot', return_value=snapshot),
            ):
                self.assertEqual(guard((0.04, 0, 0)), (0.04, 0, 0))
                report = emit.call_args.kwargs
                self.assertIn(report['decision']['state'], ('blocked', 'slowed'))
                self.assertFalse(report['enforced'])

    def test_shadow_does_not_swallow_cancellation(self):
        runner, _ = self.runner()
        runner.geometry.side_effect = KeyboardInterrupt()
        with self.assertRaises(KeyboardInterrupt):
            ShadowRouteLaserGuard(runner)((0.03, 0, 0))


class ConsoleInjectionTests(unittest.TestCase):
    def test_guarded_command_is_the_next_slew_state(self):
        runner = SimpleNamespace(
            max_speed=0.04,
            pump=Mock(side_effect=[None, None, RuntimeError('done')]),
            moving_pose=Mock(return_value=Pose(0, 0, 0)),
            twist_frame='base_link',
            body_velocity=ZERO,
            send=Mock(),
            node=SimpleNamespace(get_publishers_info_by_topic=Mock(return_value=[])),
        )
        guard = Mock(side_effect=lambda v: tuple(x / 2 for x in v))
        console = Console(runner, None, None, io.StringIO(), 0.1, command_guard=guard)
        console.cancel_input = Mock()
        with (
            patch(
                'motion_console.time.monotonic', side_effect=[0, 0, 0.01, 0.01, 0.02, 0.02, 0.03]
            ),
            patch('motion_console.feedback', return_value=((0.04, 0, 0), False)),
        ):
            with self.assertRaisesRegex(RuntimeError, 'done'):
                console.follow(Pose(0, 0, 0), Motion('move', x=0.1), 30)
        sent = [call.args[0][0] for call in runner.send.call_args_list]
        self.assertAlmostEqual(sent[0], 0.002)
        self.assertAlmostEqual(sent[1], 0.003)

    def test_default_has_no_guard_and_failure_uses_existing_stop_finally(self):
        runner = SimpleNamespace(
            max_speed=0.04,
            exclusive=Mock(),
            assert_exclusive=Mock(),
            node=SimpleNamespace(create_publisher=Mock()),
            twist=object(),
            velocity_pub=None,
        )
        console = Console(runner, None, None, io.StringIO(), 0.1)
        self.assertIsNone(console.command_guard)
        console.cancel_input, console.stop_motion = Mock(), Mock()
        console.follow = Mock(side_effect=LaserGuardError('blocked'))
        with self.assertRaises(LaserGuardError):
            console.finish_motion(Pose(0, 0, 0), Motion('move', x=0.1), 30)
        console.stop_motion.assert_called_once()


if __name__ == '__main__':
    unittest.main()
