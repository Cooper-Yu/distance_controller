"""Deterministic experimental guard tests; no ROS nodes or sensor qualification."""

from dataclasses import asdict, replace
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace as N
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from route_laser_guard import GuardPolicy, ZERO, velocity_clearance
from route_observed_guard import (
    ObservedFeedback,
    ObservedGuard,
    ObservedPolicy,
    ObservedScan,
    bound_obstacle_points,
)
from route_scan_binding import bind_scan_pair


def policy():
    return ObservedPolicy(
        obstacle=GuardPolicy(0.02, 0.035, 0.5, 0.002, math.radians(1)),
        sector_deg=30,
        stop_fraction=0.5,
        normal_fraction=0.95,
        stop_gap_deg=12,
        normal_gap_deg=2,
        degraded_scale=0.25,
        scan_age_s=0.5,
        receipt_age_s=0.5,
        odom_age_s=0.5,
        pair_age_s=0.15,
        future_s=0.05,
    )


def pose(x=0, y=0, yaw=0):
    return N(x=x, y=y, yaw=yaw)


def scan(points=((2.0, 0.0),), holes=()):
    return ObservedScan(
        points=points,
        rays=tuple((math.radians(i), i not in holes) for i in range(360)),
        step_rad=math.radians(1),
        pose=pose(),
        stamp_ns=10_000_000_000,
        paired_odom_stamp_ns=10_000_000_000,
        received_s=100.0,
        epoch='fixture',
    )


def feedback(offset=0.1, **kwargs):
    return ObservedFeedback(
        pose=pose(**kwargs),
        stamp_ns=10_000_000_000 + round(offset * 1e9),
        received_s=100 + offset,
        now_ns=10_000_000_000 + round(offset * 1e9),
        now_s=100 + offset,
        epoch='fixture',
        frames=('odom', 'base_link'),
        tf_valid=True,
    )


class ObservedGuardTests(unittest.TestCase):
    def check(self, source=None, candidate=(0.04, 0, 0), measured=ZERO):
        return ObservedGuard(policy(), 'fixture').decide(
            source or scan(),
            feedback(),
            candidate,
            measured,
        )

    def test_explicit_policy_and_non_executable_result(self):
        with self.assertRaises(TypeError):
            ObservedPolicy()
        result = self.check()
        self.assertEqual(result.state, 'observed_clear')
        self.assertFalse(result.executable)
        self.assertFalse(result.certified_free)
        json.dumps(asdict(result), allow_nan=False)
        for change in (
            {'stop_fraction': 0},
            {'degraded_scale': 1},
            {'scan_age_s': 0.6},
            {'sector_deg': 31},
            {'normal_fraction': math.nan},
        ):
            with self.assertRaises(ValueError):
                ObservedGuard(replace(policy(), **change), 'fixture')

    def test_one_close_point_overrides_bad_quality_and_stops(self):
        result = self.check(scan(points=((0.18, 0),), holes=range(360)))
        self.assertEqual(result.command, ZERO)
        self.assertEqual(result.state, 'stop')
        self.assertIsNotNone(result.obstacle)
        self.assertIn(result.obstacle.state, ('blocked',))

    def test_rotation_middle_corner_not_only_endpoints(self):
        p = policy().obstacle
        # At start/end of a 90 degree turn the point is safely beyond either edge.
        self.assertTrue(velocity_clearance(((0.23, 0),), ZERO, p)[0])
        self.assertTrue(velocity_clearance(((0, 0.23),), ZERO, p)[0])
        result = self.check(scan(points=((0.23, 0),)), (0, 0, math.pi), (0, 0, math.pi))
        self.assertEqual(result.command, ZERO)
        self.assertEqual(result.obstacle.state, 'blocked')
        self.assertIn('Measured', result.reason)

    def test_raw_masked_exterior_point_is_not_deleted(self):
        def msg(value):
            return N(
                header=N(stamp=N(sec=10, nanosec=0), frame_id='laser'),
                angle_min=0.0,
                angle_max=0.0,
                angle_increment=0.1,
                time_increment=0.0,
                scan_time=0.1,
                range_min=0.15,
                range_max=10.0,
                ranges=[value],
            )

        tf = N(
            header=N(stamp=N(sec=0, nanosec=0), frame_id='base_link'),
            child_frame_id='laser',
            transform=N(
                translation=N(x=0.02, y=0.0, z=0.173), rotation=N(x=0.0, y=0.0, z=1.0, w=0.0)
            ),
        )
        mask = dict(
            schema=1,
            type='laser_filters/LaserScanBoxFilter',
            params=dict(
                box_frame='base_link',
                invert=False,
                min_x=-0.195,
                max_x=0.165,
                min_y=-0.145,
                max_y=0.145,
                min_z=-0.5,
                max_z=0.5,
            ),
        )
        bound = bind_scan_pair(msg(0.2), msg(math.nan), tf, mask, 'fixture')
        self.assertTrue(bound['rays'][0]['supplied_box_would_remove'])
        points = bound_obstacle_points(bound)
        self.assertAlmostEqual(points[0][0], -0.18)
        result = self.check(scan(points=points), (-0.04, 0, 0))
        self.assertEqual(result.state, 'stop')

    def test_worst_sector_not_global_fraction(self):
        source = scan(holes=range(1, 25))  # 93 percent globally, deficient front sector.
        result = self.check(source)
        self.assertEqual(result.command, ZERO)
        self.assertLess(min(row[1] for row in result.sectors), 0.5)

    def test_relevant_blind_sector_and_irrelevant_rear_hole(self):
        self.assertEqual(self.check(scan(holes=range(30, 60))).state, 'stop')
        self.assertEqual(self.check(scan(holes=range(150, 210))).state, 'observed_clear')

    def test_gap_crossing_sector_seam_is_not_split_for_acceptance(self):
        p = replace(policy(), stop_fraction=0.1, stop_gap_deg=10)
        result = ObservedGuard(p, 'fixture').decide(
            scan(holes=range(24, 37)), feedback(), (0.04, 0, 0), ZERO
        )
        self.assertEqual(result.state, 'stop')

    def test_slow_band_is_not_applied_twice(self):
        result = self.check(scan(points=((0.213, 0),)))
        self.assertEqual(result.state, 'degraded')
        self.assertAlmostEqual(result.command[0], 0.02)

    def test_local_missing_rays_degrade_without_filling(self):
        result = self.check(scan(holes=range(10, 13)))
        self.assertEqual(result.state, 'degraded')
        self.assertAlmostEqual(result.command[0], 0.01)
        self.assertFalse(result.certified_free)

    def test_escape_allowed_but_overlap_and_measured_approach_stop(self):
        self.assertGreater(self.check(scan(points=((-0.18, 0),))).command[0], 0)
        self.assertEqual(self.check(scan(points=((0.16, 0),))).command, ZERO)
        result = self.check(scan(points=((0.24, 0),)), (0.01, 0, 0), (0.12, 0, 0))
        self.assertEqual(result.command, ZERO)
        self.assertIn('Measured', result.reason)

    def test_nonzero_tiny_translation_or_rotation_never_idle_bypass(self):
        source = scan(holes=range(30, 60))
        for v in ((1e-14, 0, 0), (0, 0, 1e-14)):
            self.assertEqual(self.check(source, v).state, 'stop')

    def test_same_scan_different_command_and_pose_reuses_original_receipt(self):
        guard = ObservedGuard(policy(), 'fixture')
        source = scan(points=((0.24, 0),))
        self.assertNotEqual(guard.decide(source, feedback(), (-0.04, 0, 0), ZERO).command, ZERO)
        self.assertEqual(
            guard.decide(source, feedback(0.2, x=0.06), (0.04, 0, 0), ZERO).command, ZERO
        )
        self.assertIsNone(guard.identity_fault)

    def test_duplicate_callback_does_not_refresh_source_or_receipt_age(self):
        guard = ObservedGuard(policy(), 'fixture')
        source = scan()
        guard.decide(source, feedback(), (0.04, 0, 0), ZERO)
        duplicate = replace(source, received_s=100.49)
        self.assertNotEqual(
            guard.decide(duplicate, feedback(0.49), (0, 0.04, 0), ZERO).command, ZERO
        )
        self.assertEqual(guard.scan.received_s, 100.0)
        self.assertEqual(guard.decide(duplicate, feedback(0.51), (0.04, 0, 0), ZERO).command, ZERO)
        # Fresh wall/source clock cannot refresh a stale monotonic first receipt either.
        f = replace(feedback(0.49), now_s=100.51, received_s=100.51)
        self.assertIn('Receipt', guard.decide(duplicate, f, (0.04, 0, 0), ZERO).reason)

    def test_same_stamp_changed_content_or_pose_latches_stop(self):
        for changed in (replace(scan(), points=((3.0, 0),)), replace(scan(), pose=pose(yaw=0.01))):
            guard = ObservedGuard(policy(), 'fixture')
            guard.decide(scan(), feedback(), (0.04, 0, 0), ZERO)
            self.assertEqual(guard.decide(changed, feedback(0.2), (0.04, 0, 0), ZERO).state, 'stop')
            self.assertEqual(guard.decide(scan(), feedback(0.3), (0.04, 0, 0), ZERO).state, 'stop')

    def test_epoch_backward_scan_and_clock_stop(self):
        for s, f in (
            (replace(scan(), epoch='other'), feedback()),
            (replace(scan(), stamp_ns=9_999_999_999), feedback(0.2)),
            (scan(), feedback(0.09)),
            (scan(), replace(feedback(0.2), epoch='other')),
        ):
            guard = ObservedGuard(policy(), 'fixture')
            guard.decide(scan(), feedback(), (0.04, 0, 0), ZERO)
            self.assertEqual(guard.decide(s, f, (0.04, 0, 0), ZERO).state, 'stop')

    def test_continuous_direction_changes_never_create_grace(self):
        guard = ObservedGuard(policy(), 'fixture')
        source = scan(holes=range(360))
        for i in range(20):
            f = feedback(0.1 + i * 0.01)
            result = guard.decide(source, f, (0.04 * math.cos(i), 0.04 * math.sin(i), 0), ZERO)
            self.assertEqual(result.command, ZERO)
            self.assertEqual(result.state, 'stop')

    def test_feedback_pair_tf_and_geometry_fail_closed(self):
        cases = [
            (scan(), feedback(0.51)),
            (scan(), replace(feedback(), received_s=99.0)),
            (scan(), replace(feedback(), tf_valid=False)),
            (scan(), replace(feedback(), frames=('map', 'base_link'))),
            (replace(scan(), paired_odom_stamp_ns=9_000_000_000), feedback()),
            (replace(scan(), points=((math.nan, 0),)), feedback()),
            (replace(scan(), rays=((0.0, True), (0.0, False))), feedback()),
            (replace(scan(), received_s=101.0), feedback()),
        ]
        for s, f in cases:
            self.assertEqual(
                ObservedGuard(policy(), 'fixture').decide(s, f, (0.04, 0, 0), ZERO).state, 'stop'
            )

    def test_odom_stamp_backward_stops_even_inside_freshness_window(self):
        guard = ObservedGuard(policy(), 'fixture')
        guard.decide(scan(), feedback(0.2), (0.04, 0, 0), ZERO)
        older = replace(feedback(0.3), stamp_ns=10_100_000_000)
        self.assertIn('Odometry stamp', guard.decide(scan(), older, (0.04, 0, 0), ZERO).reason)

    def test_same_odom_stamp_pose_or_measured_change_latches_stop(self):
        for changed, measured in (
            (replace(feedback(), pose=pose(x=0.01)), ZERO),
            (feedback(), (0.001, 0, 0)),
        ):
            guard = ObservedGuard(policy(), 'fixture')
            guard.decide(scan(), feedback(), (0.04, 0, 0), ZERO)
            changed = replace(changed, now_ns=10_200_000_000, now_s=100.2)
            self.assertIn(
                'Same odometry', guard.decide(scan(), changed, (0.04, 0, 0), measured).reason
            )
            self.assertEqual(guard.decide(scan(), feedback(0.3), (0.04, 0, 0), ZERO).state, 'stop')

    def test_same_odom_can_reassess_command_but_not_refresh_receipt(self):
        guard = ObservedGuard(policy(), 'fixture')
        first = replace(feedback(), received_s=99.9)
        guard.decide(scan(), first, (0.04, 0, 0), ZERO)
        duplicate = replace(first, now_ns=10_300_000_000, now_s=100.3, received_s=100.3)
        self.assertGreater(guard.decide(scan(), duplicate, (0, 0.04, 0), ZERO).command[1], 0)
        self.assertEqual(guard.feedback.received_s, 99.9)
        duplicate = replace(duplicate, now_ns=10_450_000_000, now_s=100.45, received_s=100.45)
        self.assertIn('Receipt', guard.decide(scan(), duplicate, (0.04, 0, 0), ZERO).reason)

    def test_zero_is_always_zero_and_does_not_refresh_observation(self):
        guard = ObservedGuard(policy(), 'fixture')
        result = guard.decide(scan(), feedback(5), ZERO, ZERO)
        self.assertEqual(result.command, ZERO)
        self.assertIsNone(guard.scan)


if __name__ == '__main__':
    unittest.main()
