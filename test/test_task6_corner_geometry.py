"""Behavioral scenes for local corner placement, without ROS or runtime wiring."""

import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))

from task6_corner_geometry import (  # noqa: E402
    CornerPolicy,
    StoppedPlacementGate,
    check_front_motion,
    front_command,
    select_lane,
)
from wall_geometry import fit_wall  # noqa: E402


def policy():
    return CornerPolicy(0.09, 0.005, 0.02, 0.08, 0.02, 0.20, 0.005, 3, 0.3)


def room(front=1.0, rough_side=False):
    points, rays = [], []
    for i in range(720):
        a = -math.pi + i * math.pi / 360
        c, s = math.cos(a), math.sin(a)
        distances = []
        if abs(c) > 1e-12:
            distances.append((front if c > 0 else -1.0) / c)
        if abs(s) > 1e-12:
            side = 0.5 if s > 0 else -0.5
            if rough_side and abs(c / s) < 0.35:
                side += math.copysign(0.04 * math.sin(12 * a), side)
            distances.append(side / s)
        distance = min(d for d in distances if d > 0)
        points.append((c * distance, s * distance))
        rays.append((a, True, distance))
    return points, rays


def choose(points, rays, nominal=0.0):
    return select_lane(
        points,
        rays,
        nominal_y=nominal,
        laser_origin=(0.0, 0.0),
        policy=policy(),
        evidence_id='synthetic-current-source',
    )


class CornerGeometryTests(unittest.TestCase):
    def test_front_local_volume_ignores_unmoved_rear_but_not_front_unknown_or_obstacle(self):
        points, rays = room()
        rear_invalid = [(a, False, None) if abs(a) > math.pi / 2 else (a, v, r) for a, v, r in rays]
        kwargs = dict(
            command=(0.02, 0.0, 0.0),
            measured=(0.0, 0.0, 0.0),
            laser_origin=(0.0, 0.0),
            policy=policy(),
            evidence_id='fresh-current-source',
            horizon_s=0.5,
        )
        self.assertEqual(check_front_motion(points, rear_invalid, **kwargs)['state'], 'SUPPORTED')
        front_invalid = [(a, False, None) if abs(a) < 0.2 else (a, v, r) for a, v, r in rays]
        self.assertEqual(check_front_motion(points, front_invalid, **kwargs)['state'], 'UNKNOWN')
        self.assertEqual(
            check_front_motion([*points, (0.19, 0.0)], rays, **kwargs)['state'], 'BLOCKED'
        )
        mixed = check_front_motion(points, rays, **{**kwargs, 'command': (0.01, 0.001, 0.02)})
        self.assertEqual(mixed['state'], 'SUPPORTED', mixed)
        self.assertEqual(mixed['command'][2], 0.02)

    def test_center_comes_from_observed_rails_not_nearest_right_point(self):
        points, rays = room()
        result = choose(points, rays, nominal=0.04)
        self.assertEqual(result['state'], 'TARGET_SELECTED', result)
        self.assertAlmostEqual(result['target']['y'], 0.0, places=2)
        self.assertFalse(result['endpoint_accepted'])
        self.assertLessEqual(abs(result['target']['y'] - 0.04), 0.08)

    def test_local_rails_work_when_global_side_wall_fit_fails(self):
        points, rays = room(rough_side=True)
        triples = [(x, y, a) for (x, y), (a, _, _) in zip(points, rays)]
        counts = {'right': sum(abs(a + math.pi / 2) <= math.radians(20) for a, _, _ in rays)}
        with self.assertRaises(ValueError):
            fit_wall(triples, counts, 'right')
        self.assertEqual(choose(points, rays)['state'], 'TARGET_SELECTED')

    def test_front_target_wall_limits_strip_instead_of_being_false_obstacle(self):
        points, rays = room(front=0.30)
        result = choose(points, rays)
        self.assertEqual(result['state'], 'TARGET_SELECTED', result)
        self.assertLess(result['forward_strip_m'], 0.05)
        self.assertGreater(result['forward_strip_m'], 0.03)

    def test_partial_approach_is_not_endpoint_acceptance(self):
        points, rays = room()
        result = choose(points, rays, nominal=-0.466)
        self.assertEqual(result['state'], 'APPROACH_SELECTED', result)
        self.assertEqual(result['target']['y'], -0.02)
        self.assertFalse(result['endpoint_accepted'])

    def test_near_obstacle_blocks_wrong_direction_and_no_lane_is_not_success(self):
        points, rays = room()
        result = choose([*points, (0.1, -0.18)], rays, nominal=-0.466)
        self.assertNotIn(result['state'], ('TARGET_SELECTED', 'APPROACH_SELECTED'), result)
        self.assertFalse(result['endpoint_accepted'])

    def test_invalid_motion_rays_are_unknown_not_free(self):
        points, rays = room()
        rays = [(a, False, None) if a < 0 else (a, valid, r) for a, valid, r in rays]
        result = choose(points, rays, nominal=-0.466)
        self.assertEqual(result['state'], 'UNKNOWN', result)

    def test_ambiguous_overlap_can_only_propose_small_monotone_escape(self):
        points, rays = room()
        point = (0.165, 0.13)
        right = choose([*points, point], rays, nominal=-0.466)
        self.assertEqual(right['state'], 'APPROACH_SELECTED', right)
        self.assertTrue(right['escape_model_overlap_ambiguous'])
        left = choose([*points, point], rays, nominal=0.466)
        self.assertNotEqual(left['state'], 'APPROACH_SELECTED', left)

    def test_front_control_stops_at_nine_cm_independent_of_old_odom(self):
        self.assertGreater(front_command(0.3344, 0.0, 0.0, policy=policy())['command'][0], 0)
        for gap in (0.094, 0.09, 0.086):
            result = front_command(gap, 0.002, 0.005, policy=policy())
            self.assertEqual(result['state'], 'VERIFY_STOP')
            self.assertEqual(result['command'], (0.0, 0.0, 0.0))
        self.assertEqual(front_command(0.0585, 0.0, 0.0, policy=policy())['state'], 'BLOCKED')
        self.assertEqual(front_command(0.0163, 0.0, 0.0, policy=policy())['state'], 'BLOCKED')
        self.assertEqual(front_command(float('nan'), 0.0, 0.0, policy=policy())['state'], 'UNKNOWN')

    def test_stopped_gate_needs_distinct_stable_frames_and_rewind_latches(self):
        gate = StoppedPlacementGate(policy())
        kwargs = dict(stopped=True, heading_error_rad=0.0, cross_error_m=0.0)
        self.assertFalse(gate.add(1_000_000_000, 0.09, **kwargs)['accepted'])
        self.assertFalse(gate.add(1_000_000_000, 0.09, **kwargs)['accepted'])
        self.assertFalse(gate.add(1_100_000_000, 0.09, **kwargs)['accepted'])
        self.assertTrue(gate.add(1_300_000_000, 0.09, **kwargs)['accepted'])
        self.assertEqual(gate.add(1_200_000_000, 0.09, **kwargs)['state'], 'UNKNOWN')


if __name__ == '__main__':
    unittest.main()
