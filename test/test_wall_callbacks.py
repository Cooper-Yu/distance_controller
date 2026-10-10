"""Queue ordering and fit caching must not weaken stop, freshness or scan boundaries."""

from collections import deque
from types import SimpleNamespace
from unittest import TestCase, main
from unittest.mock import Mock, patch
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from action_plan import Pose
from wall_runtime import WallRunner, WallFitError


class CallbackTests(TestCase):
    def runner(self):
        r = object.__new__(WallRunner)
        r.cancel_requested = False
        r.invalid = None
        r.node = None
        r.stamp = 0
        r.cached_stamp = 180_000_000
        r.latest = [Pose(0, 0, 0), 0, 0, 0]
        r.odom_history = deque([(0, r.latest[0])], maxlen=200)
        r.moving_pose = lambda: r.latest[0]
        r.ros = SimpleNamespace(spin_once=Mock())
        r.geometry = Mock(return_value=([], {}))
        return r

    def test_scan_arrives_before_queued_odom(self):
        r = self.runner()
        queue = ['scan', 'command', 'odom']

        def spin(*args, **kwargs):
            if queue and queue.pop(0) == 'odom':
                r.stamp = 170_000_000
                r.odom_history.append((r.stamp, Pose(0.01, 0, 0)))

        r.ros.spin_once = Mock(side_effect=spin)
        r.ros.spin_once(r.node, timeout_sec=0.05)
        with self.assertRaises(WallFitError):
            r.scan_pose()
        with patch('wall_runtime.time.monotonic', return_value=0):
            r.refresh_callbacks()
        self.assertEqual(r.scan_pose().x, 0.01)
        self.assertLessEqual(r.ros.spin_once.call_count, 33)
        self.assertTrue(
            all(c.kwargs['timeout_sec'] == 0 for c in r.ros.spin_once.call_args_list[1:])
        )

    def test_budget_and_cancel_and_invalid(self):
        r = self.runner()
        with patch('wall_runtime.time.monotonic', side_effect=[0, 0.006]):
            r.refresh_callbacks()
        r.ros.spin_once.assert_not_called()
        r.cancel_requested = True
        with self.assertRaises(KeyboardInterrupt):
            r.refresh_callbacks()
        r.cancel_requested = False

        def invalid(*args, **kwargs):
            r.invalid = 'Odom timestamp moved backward'

        r.ros.spin_once.side_effect = invalid
        with self.assertRaisesRegex(RuntimeError, 'backward'):
            r.refresh_callbacks()

    def cache_runner(self):
        r = self.runner()
        r.cached_stamp = 0
        r.fit_motion_walls = Mock(return_value={'front': 'wall'})
        return r

    def test_repeated_scan_fit_once_new_scan_and_heading_refit(self):
        r = self.cache_runner()
        for _ in range(10):
            self.assertEqual(r.motion_walls({'front'}, 0), {'front': 'wall'})
        self.assertEqual(r.fit_motion_walls.call_count, 1)
        r.cached_stamp = 1
        r.motion_walls({'front'}, 0)
        r.motion_walls({'front'}, 0.01)
        self.assertEqual(r.fit_motion_walls.call_count, 3)

    def test_cached_failure_retries_only_new_scan(self):
        r = self.cache_runner()
        r.fit_motion_walls.side_effect = WallFitError('bad fit')
        for _ in range(2):
            with self.assertRaises(WallFitError):
                r.motion_walls({'front'}, 0)
        self.assertEqual(r.fit_motion_walls.call_count, 1)
        r.cached_stamp = 1
        with self.assertRaises(WallFitError):
            r.motion_walls({'front'}, 0)
        self.assertEqual(r.fit_motion_walls.call_count, 2)

    def test_cached_success_cannot_mask_stale_geometry(self):
        r = self.cache_runner()
        r.motion_walls({'front'}, 0)
        r.geometry.side_effect = RuntimeError('scan stale')
        with self.assertRaisesRegex(RuntimeError, 'stale'):
            r.motion_walls({'front'}, 0)


if __name__ == '__main__':
    main()
