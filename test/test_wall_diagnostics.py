"""Failure diagnostics preserve source wall, exact geometry and stop-before-log ordering."""

import sys
from pathlib import Path
from unittest import TestCase, main
from unittest.mock import Mock, patch
from contextlib import redirect_stdout
from io import StringIO

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from action_plan import Pose
from wall_runtime import WallRunner, WallFitError


class WallDiagnostics(TestCase):
    def fixture(self):
        r = object.__new__(WallRunner)
        r.latest = [Pose(1, 2, 0.1), 0, 0.02, 0.03]
        r.stamp = 120
        r.cached_stamp = 123
        r.cached_geometry = ([(1.0, 0.1, 0.1)], {'front_narrow': 40, 'left_wide': 120})
        r.geometry = lambda: r.cached_geometry
        r.moving_pose = lambda: r.latest[0]
        r.observations = []
        return r

    def test_failed_front_preserves_replay_data_without_printing_before_stop(self):
        r = self.fixture()
        stream = StringIO()
        with (
            patch('wall_runtime.front_distance', side_effect=ValueError('consensus')),
            redirect_stdout(stream),
        ):
            with self.assertRaises(WallFitError) as caught:
                r.motion_walls({'front'}, 0.2)
        self.assertEqual(stream.getvalue(), '')
        detail = caught.exception.detail
        self.assertEqual(detail['side'], 'front')
        self.assertEqual(detail['scan_stamp_ns'], 123)
        self.assertAlmostEqual(detail['heading_error_rad'], 0.1)
        self.assertEqual(detail['points_base_xy_bearing'], r.cached_geometry[0])
        with redirect_stdout(stream):
            r.record_wall_failure(caught.exception, 'motion')
            r.record_wall_failure(caught.exception, 'motion')
        self.assertEqual(len(r.observations), 1)
        self.assertNotIn('points_base_xy_bearing', stream.getvalue())

    def test_side_failure_identified(self):
        r = self.fixture()
        with patch('wall_runtime.side_distance', side_effect=ValueError('rough')):
            with self.assertRaises(WallFitError) as caught:
                r.motion_walls({'left'}, 0.1)
        self.assertEqual(caught.exception.detail['side'], 'left')
        self.assertEqual(caught.exception.detail['estimator'], 'held_heading_30deg')

    def test_zero_precedes_diagnostic_and_recovery(self):
        r = self.fixture()
        events = []
        r.motion_walls = Mock(side_effect=WallFitError('fit'))
        r.send = lambda v: events.append('zero')
        r.record_wall_failure = lambda *args: events.append('diagnostic')
        r.recover_walls = lambda *args: events.append('recovery')
        self.assertIsNone(r.checked_motion_gaps({'left'}, 0, 'front', {}))
        self.assertEqual(events, ['zero', 'diagnostic', 'recovery'])

    def test_feedback_error_does_not_become_fit_failure(self):
        r = self.fixture()
        r.geometry = Mock(side_effect=RuntimeError('scan stale'))
        with self.assertRaisesRegex(RuntimeError, 'scan stale') as caught:
            r.motion_walls({'front'}, 0)
        self.assertNotIsInstance(caught.exception, WallFitError)
        self.assertFalse(r.observations)

    def test_jump_and_stopped_check_are_visible(self):
        r = self.fixture()
        r.recovery_seconds = 1.0
        with redirect_stdout(StringIO()):
            r.record_wall_failure(WallFitError('jump prior=.1 current=.2'), 'motion')
            r.record_recovery_check({'left': 0.2}, {'left': 0.1}, [])
        detail = r.observations[-1]['wall_recovery_check']
        self.assertFalse(detail['stopped'])
        self.assertFalse(detail['distance_continuous'])
        self.assertEqual(r.observations[0]['wall_fit_failure']['side'], 'distance_continuity')


if __name__ == '__main__':
    main()
