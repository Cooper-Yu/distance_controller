"""Meaningful coverage diagnostics without converting incomplete observations to free space."""

from copy import deepcopy
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from action_plan import Pose
from route_laser_coverage import angular_evidence, aperture, diagnose, paired_kinds, range_kind
from route_laser_coverage_runtime import CoverageShadowRouteLaserGuard
from route_laser_guard import ScanSnapshot, ZERO, evaluate
from wall_geometry import scan_points


def rays_with_holes(indices):
    return tuple(
        (i * math.pi / 360, None if i in indices else 1.0, 'nan' if i in indices else 'finite')
        for i in range(720)
    )


class CoverageEvidenceTests(unittest.TestCase):
    def test_distinct_invalid_encodings_are_never_finite(self):
        self.assertEqual(
            [range_kind(v, 0.15, 10) for v in (math.nan, math.inf, -math.inf, 0.1, 11, 0.15, 10)],
            ['nan', 'positive_inf', 'negative_inf', 'below_min', 'above_max', 'finite', 'finite'],
        )

    def test_raw_pair_requires_exact_stamp_and_geometry(self):
        filtered = SimpleNamespace(
            header=1,
            angle_min=0.0,
            angle_increment=0.01,
            range_min=0.15,
            range_max=10.0,
            ranges=[math.nan, math.inf, 0.1],
        )
        raw = deepcopy(filtered)
        raw.ranges[0] = 0.16
        kinds, paired = paired_kinds(filtered, raw)
        self.assertTrue(paired)
        self.assertEqual(kinds, ('filtered_finite', 'raw_positive_inf', 'raw_below_min'))
        raw.header = 2
        self.assertEqual(paired_kinds(filtered, raw), (('nan', 'positive_inf', 'below_min'), False))

    def test_one_nan_rotation_retains_obstacle_constraint_and_quantifies_hole(self):
        rays = rays_with_holes({90})
        snapshot = ScanSnapshot(((1.0, 0.0),), tuple((a, k == 'finite') for a, _, k in rays))
        candidate = (0.0, 0.0, 0.1)
        self.assertEqual(evaluate(snapshot, candidate, ZERO).state, 'unknown')
        result = diagnose(snapshot, rays, candidate, ZERO)
        self.assertEqual(result['observed_point_constraint']['state'], 'clear')
        self.assertFalse(result['executable'])
        gap = result['coverage']['gaps'][0]
        self.assertAlmostEqual(gap['width_deg'], 1.0)
        self.assertEqual(gap['invalid_count'], 1)
        self.assertTrue(gap['motion_relevant'])
        self.assertIsNone(gap['hidden_obstacle_distance_lower_bound_m'])
        self.assertLess(gap['aperture_at_audit_radius_m'], 0.005)

    def test_nan_does_not_hide_a_measured_collision(self):
        rays = rays_with_holes({90})
        snapshot = ScanSnapshot(((0.16, 0.0),), ())
        result = diagnose(snapshot, rays, (0.03, 0.0, 0.1), ZERO)
        self.assertEqual(result['observed_point_constraint']['state'], 'blocked')
        self.assertEqual(result['observed_point_count'], 1)
        self.assertLess(result['nearest_observed_clearance_m'], 0)

    def test_circular_gap_merges_across_zero_and_keeps_flanks(self):
        result = angular_evidence(rays_with_holes({719, 0, 1}), ((0.04, 0.0, 0.0),), 0.3)
        self.assertEqual(len(result['gaps']), 1)
        gap = result['gaps'][0]
        self.assertAlmostEqual(gap['width_deg'], 2.0)
        self.assertEqual(gap['flank_ranges_m'], [1.0, 1.0])
        self.assertEqual(gap['invalid_count'], 3)
        self.assertTrue(gap['motion_relevant'])

    def test_rear_gap_is_irrelevant_only_for_pure_forward_scenario(self):
        rays = rays_with_holes({360})
        front = angular_evidence(rays, ((0.04, 0.0, 0.0),), 0.3)
        self.assertFalse(front['gaps'][0]['motion_relevant'])
        reverse_momentum = angular_evidence(rays, ((0.04, 0.0, 0.0), (-0.01, 0.0, 0.0)), 0.3)
        self.assertTrue(reverse_momentum['gaps'][0]['motion_relevant'])

    def test_all_invalid_and_no_rays_are_uncertified_not_near_obstacles(self):
        result = angular_evidence(rays_with_holes(set(range(720))), ((0.0, 0.0, 0.1),), 0.3)
        self.assertEqual(result['finite_count'], 0)
        self.assertEqual(result['gaps'][0]['width_deg'], 360)
        self.assertAlmostEqual(result['largest_aperture_at_audit_radius_m'], 0.6)
        self.assertFalse(angular_evidence((), (), 0.3)['certified_free'])

    def test_missing_scheduled_sector_and_usual_finite_spacing_are_distinct(self):
        rays = rays_with_holes(set())
        complete = angular_evidence(rays, ((0.0, 0.0, 0.1),), 0.3)
        self.assertEqual(complete['gaps'], [])
        self.assertGreater(complete['largest_aperture_at_audit_radius_m'], 0)
        self.assertFalse(complete['certified_free'])
        result = angular_evidence(rays[20:], ((0.0, 0.0, 0.1),), 0.3)
        self.assertTrue(result['gaps'][0]['missing_scheduled_sector'])
        self.assertEqual(result['gaps'][0]['invalid_count'], 0)

    def test_wide_holes_cannot_shrink_at_wrap_and_near_flank_is_not_free(self):
        self.assertAlmostEqual(aperture(0.3, math.radians(350)), 0.6)
        rays = ((0.0, 0.18, 'finite'), (math.pi, None, 'positive_inf'))
        result = angular_evidence(rays, ((0.0, 0.0, 0.1),), 0.3)
        self.assertEqual(result['finite_rays_reaching_audit_radius'], 0)
        self.assertEqual(result['gaps'][0]['flank_ranges_m'], [0.18, 0.18])
        self.assertIsNone(result['gaps'][0]['hidden_obstacle_distance_lower_bound_m'])


class ShadowAdapterTests(unittest.TestCase):
    def fixture(self):
        scan = SimpleNamespace(
            header=1,
            angle_min=-math.pi,
            angle_increment=math.pi / 360,
            range_min=0.15,
            range_max=10.0,
            ranges=[1.0] * 720,
        )
        scan.ranges[400] = math.nan
        tf = SimpleNamespace(
            rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            translation=SimpleNamespace(x=0.02, y=0.0, z=0.173),
        )
        runner = SimpleNamespace(
            moving_pose=Mock(return_value=Pose(0.01, 0.0, 0.0)),
            geometry=Mock(return_value=scan_points(scan, tf)),
            scan_pose=Mock(return_value=Pose(0.0, 0.0, 0.0)),
            frame='odom',
            twist_frame='base_link',
            observer=SimpleNamespace(scan=scan),
            body_velocity=ZERO,
            cached_stamp=1,
            stamp=2,
        )
        return runner, tf

    def test_actual_adapter_keeps_turn_and_logs_finite_constraints_with_unknown_gap(self):
        runner, tf = self.fixture()
        emit = Mock()
        guard = CoverageShadowRouteLaserGuard(runner, emit)
        with (
            patch('route_laser_runtime.scan_transform', return_value=tf),
            patch('route_laser_coverage_runtime.scan_transform', return_value=tf),
        ):
            command = (0.0, 0.0, 0.1)
            self.assertEqual(guard(command), command)
        self.assertEqual(emit.call_args_list[0].kwargs['decision']['state'], 'unknown')
        record = emit.call_args_list[1]
        self.assertEqual(record.args, ('laser_coverage_shadow',))
        self.assertFalse(record.kwargs['enforced'])
        self.assertEqual(record.kwargs['observed_point_constraint']['state'], 'clear')
        self.assertAlmostEqual(record.kwargs['sensor_origin_base_xy'][0], 0.01)

    def test_stale_remains_unavailable_and_cancel_propagates(self):
        runner, _ = self.fixture()
        emit = Mock()
        guard = CoverageShadowRouteLaserGuard(runner, emit)
        runner.geometry.side_effect = RuntimeError('scan stale')
        self.assertEqual(guard((0.03, 0.0, 0.0)), (0.03, 0.0, 0.0))
        self.assertEqual(emit.call_args.kwargs['state'], 'unavailable')
        runner.geometry.side_effect = KeyboardInterrupt()
        with self.assertRaises(KeyboardInterrupt):
            guard((0.03, 0.0, 0.0))


if __name__ == '__main__':
    unittest.main()
