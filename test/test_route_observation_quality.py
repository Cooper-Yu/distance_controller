"""Observation-quality regression tests: shadow diagnostics, not motion permission."""

import json
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from route_observation_quality import QualityPolicy, QualityTracker, metrics

STEP = math.pi / 180
FORWARD, BACK, TURN, ZERO = (0.04, 0, 0), (-0.04, 0, 0), (0, 0, 0.1), (0, 0, 0)


def rays(holes=()):
    return tuple(
        (i * STEP, None if i in holes else 1.0, 'positive_inf' if i in holes else 'finite')
        for i in range(360)
    )


class QualityTests(unittest.TestCase):
    def get(self, source, candidate=FORWARD, measured=ZERO):
        return metrics(source, STEP, candidate, measured, QualityPolicy())[0]

    def test_single_invalid_rotation_does_not_fail_all_rays_condition(self):
        result = QualityTracker().observe(0, rays({0}), STEP, TURN, ZERO, 'one')
        self.assertEqual(result['state'], 'within_example_limits')
        self.assertAlmostEqual(result['maximum_unknown_deg'], 1)
        self.assertFalse(result['executable'])
        self.assertFalse(result['unknown_is_free'])
        self.assertEqual(result['applied_command'], TURN)
        json.dumps(result, allow_nan=False)

    def test_rear_hole_and_measured_reverse_union(self):
        source = rays(set(range(160, 201)))
        self.assertAlmostEqual(self.get(source)['angular_finite_fraction'], 1)
        both = self.get(source, FORWARD, BACK)
        self.assertAlmostEqual(both['relevant_deg'], 360)
        self.assertAlmostEqual(both['maximum_unknown_deg'], 41)

    def test_all_finite_is_not_certified_free(self):
        report = QualityTracker().observe(0, rays(), STEP, TURN, ZERO, 'one')
        self.assertAlmostEqual(report['angular_finite_fraction'], 1)
        self.assertFalse(report['certified_free'])
        self.assertFalse(report['calibrated'])

    def test_missing_scheduled_angles_cannot_claim_100_percent(self):
        result = self.get(rays()[20:340], TURN)
        self.assertLess(result['angular_finite_fraction'], 0.9)
        self.assertAlmostEqual(result['maximum_unknown_deg'], 40)

    def test_wraparound_unknown_and_direction_boundaries(self):
        result = self.get(rays(set(range(350, 360)) | set(range(11))))
        self.assertAlmostEqual(result['maximum_unknown_deg'], 21)
        half = self.get(rays(set(range(80, 101))))
        self.assertAlmostEqual(half['maximum_unknown_deg'], 10.5)

    def test_persistent_degradation_recovery_and_all_invalid(self):
        tracker = QualityTracker()
        for time in (0, 0.1, 0.2):
            result = tracker.observe(time, rays(range(30)), STEP, FORWARD, ZERO, 'one')
            self.assertEqual(result['state'], 'transient_degradation')
        result = tracker.observe(0.3, rays(range(30)), STEP, FORWARD, ZERO, 'one')
        self.assertEqual(result['state'], 'persistent_degradation')
        result = tracker.observe(0.4, rays(), STEP, FORWARD, ZERO, 'one')
        self.assertEqual(result['bad_duration_s'], 0)
        result = tracker.observe(0.5, rays(range(360)), STEP, FORWARD, ZERO, 'one')
        self.assertEqual(result['state'], 'unavailable')

    def test_epoch_timegap_backwards_and_direction_reset(self):
        for time, velocity, epoch in (
            (0.6, FORWARD, 'one'),
            (0.1, FORWARD, 'one'),
            (0.3, BACK, 'one'),
            (0.3, FORWARD, 'two'),
        ):
            tracker = QualityTracker()
            for t in (0, 0.1, 0.2):
                tracker.observe(t, rays(range(360)), STEP, FORWARD, ZERO, 'one')
            result = tracker.observe(time, rays(range(360)), STEP, velocity, ZERO, epoch)
            self.assertTrue(result['continuity_reset'])
            self.assertEqual(result['bad_duration_s'], 0)

    def test_deadband_is_quality_only_and_explicit(self):
        rear_holes = rays(range(160, 201))
        jitter = self.get(rear_holes, FORWARD, (0, 0, 0.001))
        rotating = self.get(rear_holes, FORWARD, (0, 0, 0.02))
        self.assertAlmostEqual(jitter['relevant_deg'], 180)
        self.assertAlmostEqual(rotating['relevant_deg'], 360)

    def test_near_points_are_not_removed_or_given_quality_priority(self):
        source = list(rays({0}))
        source[1] = (STEP, 0.151, 'finite')
        result = QualityTracker().observe(0, source, STEP, FORWARD, ZERO, 'one')
        self.assertEqual(result['finite_rays'], 359)
        self.assertIn('all finite points', result['obstacle_constraints'])
        self.assertEqual(source[1][1], 0.151)

    def test_invalid_input_and_policy(self):
        for policy in (QualityPolicy(minimum_fraction=0), QualityPolicy(maximum_gap_deg=math.nan)):
            with self.assertRaises(ValueError):
                policy.validate()
        for source in (
            ((0, math.inf, 'finite'),),
            ((0, None, 'finite'),),
            ((0, 1, 'finite'), (0, 1, 'finite')),
        ):
            with self.assertRaises(ValueError):
                self.get(source)
        with self.assertRaises(ValueError):
            self.get(rays(), (math.nan, 0, 0))
        with self.assertRaises(ValueError):
            metrics(rays(), 0, FORWARD, ZERO, QualityPolicy())

    def test_observation_quality_never_overrides_a_measured_obstacle(self):
        from route_laser_guard import GuardPolicy, select_command

        quality = QualityTracker().observe(0, rays({0}), STEP, FORWARD, ZERO, 'one')
        constraint = select_command(((0.171, 0.0),), FORWARD, ZERO, GuardPolicy())
        self.assertEqual(quality['state'], 'within_example_limits')
        self.assertEqual(constraint.state, 'blocked')
        self.assertEqual(constraint.command, ZERO)

    def test_bad_frame_clears_persistence(self):
        tracker = QualityTracker()
        for t in (0, 0.1, 0.2):
            tracker.observe(t, rays(range(30)), STEP, FORWARD, ZERO, 'one')
        with self.assertRaises(ValueError):
            tracker.observe(0.25, ((0, None, 'finite'),), STEP, FORWARD, ZERO, 'one')
        report = tracker.observe(0.3, rays(range(30)), STEP, FORWARD, ZERO, 'one')
        self.assertEqual(report['bad_duration_s'], 0)
        self.assertEqual(report['state'], 'transient_degradation')

    def test_no_rays_and_idle(self):
        self.assertEqual(
            QualityTracker().observe(0, (), STEP, FORWARD, ZERO, 'one')['state'], 'unavailable'
        )
        self.assertEqual(
            QualityTracker().observe(0, rays(range(360)), STEP, ZERO, ZERO, 'one')['state'], 'idle'
        )


if __name__ == '__main__':
    unittest.main()
