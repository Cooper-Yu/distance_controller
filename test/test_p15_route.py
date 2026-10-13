"""Teaching-data geometry, turn direction and console route scheduling contracts."""

import io
import json
import math
from pathlib import Path
import sys
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from action_plan import Pose
from motion_model import LATE_DESTINATIONS, parse_batch
from motion_console import Console
from teaching_path import late_route, turn_to_heading

DATA = json.loads(
    (Path(__file__).resolve().parents[1] / 'config/task6_p01_p02_path.json').read_text()
)


class P15RouteTests(unittest.TestCase):
    def test_prefixes_stop_before_destination_turn(self):
        for label in LATE_DESTINATIONS:
            self.assertEqual(parse_batch('path_plan ' + label), [('path_plan', label)])
            self.assertEqual(parse_batch('path ' + label), [('path', label)])
            actions = late_route(DATA, label)
            self.assertEqual(actions[-1][0], label)
            self.assertEqual(actions[-1][1].kind, 'path')
        with self.assertRaises(ValueError):
            parse_batch('path P15_turn')

    def test_full_order_spacing_and_absolute_endpoints(self):
        actions = late_route(DATA, 'P15')
        self.assertEqual(
            [label for label, _ in actions],
            [
                'P09',
                'P10',
                'P10_after_turn',
                'P11',
                'P11_after_turn',
                'P12',
                'P12_after_turn',
                'P13',
                'P13_after_turn',
                'P14',
                'P14_after_turn',
                'P15',
            ],
        )
        last = (DATA['p08_candidate']['x'], DATA['p08_candidate']['y'])
        for label, motion in actions:
            if motion.kind == 'turn':
                continue
            self.assertEqual(motion.points[0], last)
            row = DATA[label.lower() + '_candidate']
            self.assertEqual(motion.points[-1], (row['x'], row['y']))
            self.assertTrue(
                all(
                    math.dist(a, b) <= 0.05 + 1e-12
                    for a, b in zip(motion.points, motion.points[1:])
                )
            )
            last = motion.points[-1]

    def test_turn_directions_with_wrapped_pose(self):
        for origin_yaw in (0.0, 3.13, -2.4):
            origin = Pose(5, -4, origin_yaw)
            heading, turns = -math.pi / 2, []
            for _, motion in late_route(DATA, 'P15'):
                if motion.kind == 'turn':
                    wrapped = math.atan2(
                        math.sin(origin_yaw + heading), math.cos(origin_yaw + heading)
                    )
                    turn = turn_to_heading(origin, Pose(5, -4, wrapped), motion.angle)
                    turns.append(round(math.degrees(turn.angle)))
                    heading = motion.angle
            self.assertEqual(turns, [-90, -90, 90, -90, 90])

    def console(self):
        c = Console(
            SimpleNamespace(pose=lambda: Pose(0, 0, 0)),
            SimpleNamespace(origin=Pose(0, 0, 0), data={'points': {}}),
            None,
            io.StringIO(),
            0.6,
        )
        c.extended_route = Mock()
        c.move = Mock()
        c.record = Mock()
        return c

    def test_preview_and_post_turn_reconnect_scope(self):
        actions = late_route(DATA, 'P15')
        c = self.console()
        with patch('motion_console.load_late_route', return_value=actions):
            c.late_route('path_plan', 'P15')
            c.move.assert_not_called()
            c.record.assert_not_called()
            c.late_route('path', 'P15')
        path_calls = [call for call in c.move.call_args_list if call.args[0].kind == 'path']
        self.assertEqual(
            [call.kwargs['reconnect'] for call in path_calls],
            [False, False, True, True, True, True, True],
        )
        self.assertEqual(
            [call.args[0] for call in c.record.call_args_list], [label for label, _ in actions]
        )

    def test_turn_failure_does_not_run_or_record_next_leg(self):
        actions = late_route(DATA, 'P15')
        c = self.console()
        c.move.side_effect = [
            None,
            None,
            RuntimeError('Final stopped pose outside enabled tolerances'),
        ]
        with patch('motion_console.load_late_route', return_value=actions):
            with self.assertRaises(RuntimeError):
                c.late_route('path', 'P15')
        self.assertEqual([call.args[0] for call in c.record.call_args_list], ['P09', 'P10'])


if __name__ == '__main__':
    unittest.main()
