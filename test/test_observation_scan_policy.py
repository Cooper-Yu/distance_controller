"""Observation scan grace does not change odometry, strict modes, or initialization."""

from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import patch

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from task6_route_runner import configure_route_scan_policy  # noqa: E402
from wall_runtime import WallRunner  # noqa: E402


def fixture_runner(age=0.7, receipt_age=0.7):
    stamp_ns = int((100.0 - age) * 1e9)
    stamp = SimpleNamespace(sec=stamp_ns // 10**9, nanosec=stamp_ns % 10**9)
    runner = WallRunner.__new__(WallRunner)
    runner.observer = SimpleNamespace(
        scan=SimpleNamespace(header=SimpleNamespace(stamp=stamp, frame_id='laser')),
        received=10.0 - receipt_age,
    )
    runner.node = SimpleNamespace(
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=100 * 10**9))
    )
    runner.cached_stamp = stamp_ns
    runner.cached_geometry = ('cached geometry', {})
    return runner


@pytest.mark.parametrize('age,receipt_age', [(0.7, 0.1), (0.1, 0.7), (0.99, 0.99)])
def test_only_post_preparation_observation_mode_accepts_short_gap(age, receipt_age):
    runner = fixture_runner(age, receipt_age)
    # No mode configured during preparation / standalone / ordinary wall control.
    with patch('wall_runtime.time.monotonic', return_value=10.0):
        with pytest.raises(RuntimeError, match='scan stale'):
            runner.geometry()
        configure_route_scan_policy(runner, False)
        assert runner.geometry() == runner.cached_geometry
        configure_route_scan_policy(runner, True)
        with pytest.raises(RuntimeError, match='scan stale'):
            runner.geometry()


@pytest.mark.parametrize('age,receipt_age', [(1.001, 0.1), (0.1, 1.001), (-0.2, 0.1)])
def test_unknown_future_or_long_gap_remains_rejected(age, receipt_age):
    runner = fixture_runner(age, receipt_age)
    configure_route_scan_policy(runner, False)
    with patch('wall_runtime.time.monotonic', return_value=10.0):
        with pytest.raises(RuntimeError, match='scan stale'):
            runner.geometry()


@pytest.mark.parametrize('limit', [float('nan'), float('inf'), 0.49, 1.01])
def test_timeout_cannot_expand_beyond_bounded_policy(limit):
    runner = fixture_runner()
    runner.observation_scan_timeout = limit
    with patch('wall_runtime.time.monotonic', return_value=10.0):
        with pytest.raises(RuntimeError, match='Invalid observation'):
            runner.geometry()
