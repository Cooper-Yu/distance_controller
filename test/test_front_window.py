"""Front stopping window coverage, near targets and retained peripheral obstacles."""

import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from wall_geometry import scan_points, front_distance, fit_wall, support, swept_clearance


def scan(gap, angle=0):
    nx, ny = math.cos(angle), math.sin(angle)
    values = []
    for i in range(720):
        a = -math.pi + i * math.pi / 360 + math.pi
        denom = nx * math.cos(a) + ny * math.sin(a)
        values.append((support(nx, ny) + gap - 0.02 * nx) / denom if denom > 0.1 else float('inf'))
    message = SimpleNamespace(
        angle_min=-math.pi,
        angle_increment=math.pi / 360,
        range_min=0.15,
        range_max=40.0,
        ranges=values,
    )
    transform = SimpleNamespace(
        rotation=SimpleNamespace(x=0, y=0, z=1, w=0),
        translation=SimpleNamespace(x=0.02, y=0, z=0.173),
    )
    return scan_points(message, transform)


class FrontWindow(unittest.TestCase):
    def test_real_p02_wide_fails_narrow_passes(self):
        f = json.loads((Path(__file__).parent / 'front_p02_returns.json').read_text())
        with self.assertRaises(ValueError):
            fit_wall(f['points'], f['counts'], 'front')
        wall = front_distance(f['points'], f['counts'])
        self.assertGreaterEqual(wall.count / wall.raw_count, 0.8)
        self.assertAlmostEqual(wall.gap, 0.9804663, delta=0.005)
        self.assertEqual(f['counts']['front_narrow'], 48)

    def test_approach_to_current_target_and_supported_close_target(self):
        for gap in [0.98, 0.5, 0.2, 0.1075, 0.0975, 0.06]:
            for angle in [-0.08, 0, 0.08]:
                p, c = scan(gap, angle)
                self.assertAlmostEqual(front_distance(p, c).gap, gap, places=6)

    def test_blind_zone_and_sparse_returns_rejected(self):
        p, c = scan(0.01)
        with self.assertRaises(ValueError):
            front_distance(p, c)
        p, c = scan(0.10)
        with self.assertRaises(ValueError):
            front_distance(p[::3], c)

    def test_opening_and_two_depths_rejected(self):
        p, c = scan(0.5)
        altered = [(x if y < 0 else x + 0.25, y, a) for x, y, a in p]
        with self.assertRaises(ValueError):
            front_distance(altered, c)

    def test_peripheral_obstacle_retained(self):
        p, c = scan(0.1)
        p.append((0.14, 0.17, math.atan2(0.17, 0.12)))
        self.assertAlmostEqual(front_distance(p, c).gap, 0.1)
        self.assertLess(swept_clearance(p), 0.02)


if __name__ == '__main__':
    unittest.main()
