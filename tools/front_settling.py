"""Stopped multi-scan acceptance for explicit endpoint corrections only."""

from collections import deque
from statistics import median
import time
from action_plan import errors
from wall_geometry import swept_clearance


class StoppedWindow:
    """Use seven distinct scans, at least five valid fits, over a bounded time span."""

    def __init__(self):
        self.samples = deque(maxlen=7)
        self.stamp = None

    def add(self, stamp, now, gaps, stopped):
        if stamp == self.stamp:
            return
        self.stamp = stamp
        if not stopped:
            self.samples.clear()
            return
        self.samples.append((now, gaps))

    def result(self, target, follow, follow_target):
        if len(self.samples) < 7:
            return None
        duration = self.samples[-1][0] - self.samples[0][0]
        valid = [g for _, g in self.samples if g is not None]
        if not 0.5 <= duration <= 1.2 or len(valid) < 5:
            return None
        front = [g['front'] for g in valid]
        center, spread = median(front), max(front) - min(front)
        if min(front) < target - 0.015 or abs(center - target) > 0.01 or spread > 0.02:
            return None
        if follow != 'none':
            side = [g[follow] for g in valid]
            if abs(median(side) - follow_target) > 0.008 or max(side) - min(side) > 0.02:
                return None
        return {
            'front_median_m': center,
            'front_spread_m': spread,
            'valid_scans': len(valid),
            'total_scans': 7,
            'duration_s': duration,
        }


def verify_stopped(runner, target, policy):
    """Publish zero throughout; raw scan and measured-motion guards remain unfiltered."""
    from wall_runtime import WallFitError

    follow_target, stop_target = policy.clearances(runner.reference)
    sides = ['front'] + ([policy.follow] if policy.follow != 'none' else [])
    window = StoppedWindow()
    deadline = time.monotonic() + 2
    last = None
    while time.monotonic() < deadline:
        runner.send((0.0, 0.0, 0.0))
        runner.pump()
        actual = runner.moving_pose()
        runner.guard_translation((0.0, 0.0, 0.0), 'front')
        points, _ = runner.geometry()
        if runner.twist_frame != 'base_link':
            raise RuntimeError('Front settling requires body-frame velocity')
        gap = swept_clearance(points, *runner.body_velocity)
        if gap < 0.02:
            runner.reject_translation(points, runner.body_velocity, 'front', gap, 'measured')
        try:
            gaps = {k: v.gap for k, v in runner.motion_walls(sides, target.yaw).items()}
        except WallFitError:
            gaps = None
        stopped = runner.latest[2] < 0.01 and runner.latest[3] < 0.02
        window.add(runner.cached_stamp, time.monotonic(), gaps, stopped)
        last = window.result(stop_target, policy.follow, follow_target)
        if last is not None and abs(errors(actual, target)[1]) <= 0.02:
            print('FRONT_SETTLED: ' + str(last), flush=True)
            return last
    raise RuntimeError(
        'FRONT_SETTLING: stopped scan window not acceptable; inspect measure; '
        'adjust_front target retained'
    )
