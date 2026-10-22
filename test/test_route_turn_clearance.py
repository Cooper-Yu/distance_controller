"""Whole-turn diagnostic regressions, including clear endpoints and blocked interior."""

import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from route_turn_clearance import inspect_turn, load_event
from wall_geometry import point_clearance


class TurnClearanceTests(unittest.TestCase):
    def test_endpoints_clear_but_corner_sweep_blocks(self):
        # Axial start/end clearances exceed 2 cm; an intermediate corner reaches it.
        point = (0.205, 0.0)
        self.assertGreater(point_clearance(*point), 0.02)
        self.assertGreater(point_clearance(0, -0.205), 0.02)
        result = inspect_turn([point], -90)
        self.assertFalse(result['finite_points_preserve_margin'])
        self.assertLess(result['minimum_point_gap_lower_bound_m'], 0)
        self.assertLess(abs(result['first_sample_below_margin_deg']), 90)

    def test_lower_bound_covers_dense_sweep(self):
        points = [(0.25, 0.02), (-0.03, -0.24)]
        result = inspect_turn(points, 90)
        dense = min(
            point_clearance(math.cos(a) * x + math.sin(a) * y, -math.sin(a) * x + math.cos(a) * y)
            for a in (math.pi / 2 * i / 10000 for i in range(10001))
            for x, y in points
        )
        lower = result['minimum_point_gap_lower_bound_m']
        self.assertLessEqual(lower, dense + 1e-12)
        self.assertLess(dense - lower, 0.0011)

    def test_far_points_do_not_certify_unknown_space(self):
        result = inspect_turn([(1.0, 0.0)], 180)
        self.assertTrue(result['finite_points_preserve_margin'])
        self.assertFalse(result['motion_authorized'])
        self.assertFalse(result['certified_free'])

    def test_overlap_is_reported(self):
        result = inspect_turn([(0.1, 0)], -90)
        self.assertLess(result['initial_point_gap_m'], 0)
        self.assertEqual(result['first_sample_below_margin_deg'], 0)

    def test_invalid_inputs_rejected(self):
        for points, angle in (
            ([], 90),
            ([(math.nan, 0)], 90),
            ([(1, 0)], 0),
            ([(1, 0)], 181),
            ([(1, 0)], math.inf),
        ):
            with self.assertRaises(ValueError):
                inspect_turn(points, angle)

    def test_explicit_audit_selection_rejects_registration_cloud(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'audit.jsonl'
            path.write_text(
                json.dumps({'points': [[1, 0]]})
                + '\n'
                + json.dumps({'points_base_xy': [[2, 0]]})
                + '\n'
            )
            with self.assertRaises(ValueError):
                load_event(path, 1)
            self.assertEqual(load_event(path, 2), [[2, 0]])
            with self.assertRaises(ValueError):
                load_event(path, 3)


if __name__ == '__main__':
    unittest.main()
