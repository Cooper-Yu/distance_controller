"""Regression checks for reverse order, frame geometry and conservative completion."""

import importlib.util
import math
from pathlib import Path
import sys
import unittest
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from survey_distances import summarize_scan

PATH = Path(__file__).resolve().parents[1] / 'tools/survey_return.py'
SPEC = importlib.util.spec_from_file_location('survey_return', PATH)
m = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = m
SPEC.loader.exec_module(m)


def integrate(actions, pose=(0.0, 0.0, 0.0)):
    x, y, yaw = pose
    for action in actions:
        if action.kind == 'turn':
            yaw += action.value
        else:
            dx, dy = {
                'forward': (action.value, 0),
                'backward': (-action.value, 0),
                'left': (0, action.value),
                'right': (0, -action.value),
            }[action.kind]
            x += math.cos(yaw) * dx - math.sin(yaw) * dy
            y += math.sin(yaw) * dx + math.cos(yaw) * dy
    return x, y, yaw


class ReturnTest(unittest.TestCase):
    def test_reverse_restores_arbitrary_start(self):
        start = (3.2, -4.1, 0.73)
        end = integrate(m.OUTWARD, start)
        actual = integrate(m.reverse_route(m.OUTWARD), end)
        for a, b in zip(actual, start):
            self.assertAlmostEqual(a, b)

    def test_first_actions_retrace_final_turn_and_translation(self):
        route = m.reverse_route(m.OUTWARD)
        self.assertEqual(route[0].kind, 'turn')
        self.assertAlmostEqual(route[0].value, -math.pi)
        self.assertEqual(route[1].place, 'P15->P14')
        self.assertEqual(route[1].kind, 'backward')
        self.assertEqual(route[1].value, 0.40)
        self.assertAlmostEqual(route[2].value, -math.pi / 2)

    def test_waiting_without_matching_endpoint_is_not_success(self):
        self.assertFalse(
            m.idle('state=WAITING waypoint=A pending=0 return_remaining=0', 'return_02')
        )
        self.assertFalse(m.idle('state=RUNNING waypoint=return_02 pending=0 return_remaining=0'))
        self.assertFalse(m.idle('state=WAITING pending=10 return_remaining=0'))
        self.assertFalse(m.idle('state=FAULT pending=0 return_remaining=0'))
        self.assertTrue(
            m.idle('state=WAITING waypoint=return_02 pending=0 return_remaining=0', 'return_02')
        )

    def test_corrections_remain_provisional(self):
        self.assertEqual(sum(a.provisional for a in m.reverse_route(m.OUTWARD)), 4)
        self.assertIn('NOT REPLAY-VERIFIED', m.describe(1, m.OUTWARD[5]))


class DistanceTest(unittest.TestCase):
    def scan(self):
        # laser front points backward: index0 rear,90 right,180 front,270 left.
        ranges = [1.0] * 360
        for center, distance in [(0, 4.0), (90, 3.0), (180, 1.0), (270, 2.0)]:
            for offset in range(-6, 7):
                ranges[(center + offset) % 360] = distance
        return SimpleNamespace(
            angle_min=0.0,
            angle_increment=math.pi / 180,
            range_min=0.15,
            range_max=40.0,
            ranges=ranges,
        )

    def test_reversed_mount_and_angle_wrap(self):
        data = summarize_scan(self.scan(), (0, 0, 1, 0))
        for name, value in [('front', 1), ('left', 2), ('right', 3), ('rear', 4)]:
            self.assertAlmostEqual(data[name]['median_m'], value)
            self.assertEqual(data[name]['total'], 11)

    def test_bad_returns_are_not_clear_space(self):
        scan = self.scan()
        scan.ranges = [float('inf')] * 360
        scan.ranges[90] = 0.01
        data = summarize_scan(scan, (0, 0, 1, 0))
        self.assertIsNone(data['right']['median_m'])
        self.assertEqual(data['right']['valid'], 0)

    def test_tilted_scan_is_rejected(self):
        with self.assertRaises(ValueError):
            summarize_scan(self.scan(), (math.sqrt(0.5), 0, 0, math.sqrt(0.5)))


if __name__ == '__main__':
    unittest.main()
