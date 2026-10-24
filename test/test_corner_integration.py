"""Independent multi-step production-control integration with synthetic I/O only."""

import math
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import patch

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from action_plan import Pose  # noqa: E402
from motion_model import Motion, feedback  # noqa: E402
import task6_corner_runtime as runtime  # noqa: E402
from test_task6_corner_runtime import FakeRunner, console, policy  # noqa: E402


def configured_console(runner):
    c = console(runner)
    c.book = SimpleNamespace(origin=Pose(0, 0, 0))
    c.measure = lambda: None  # Only the unrelated observer/logging hook is replaced.
    runner.moving_pose = runner.pose
    return c


def test_p09_p10_p11_nonzero_geometry_convergence_and_connectors():
    r = FakeRunner(Pose(0, -0.025, 0), front=0.35)
    c = configured_console(r)
    calls = []
    r.feedback_pause.check = lambda: calls.append(r.now)
    with (
        patch.object(runtime, '_read_frame', lambda runner: runner.read_frame()),
        patch.object(runtime.time, 'monotonic', lambda: r.now),
    ):
        placement = runtime.CornerPlacement(c, policy(), enabled=True)
        p09 = placement.place('P09', Pose(0, 0.02, 0), budget_s=12)
        assert -0.005 <= p09.actual.y <= 0.005
        assert any(command[1] > 0 for command in r.sent)
        after_lane = len(r.sent)
        p10_path = Motion(
            'path', x=0.08, y=0.02, angle=0, points=((0, 0.02), (0.04, 0.02), (0.08, 0.02))
        )
        connected_p10 = runtime.connector(p10_path, c.book.origin, p09)
        assert connected_p10.points[0] == pytest.approx((p09.actual.x, p09.actual.y))
        assert connected_p10.points[-2:] == p10_path.points[-2:]
        p10 = placement.place('P10', Pose(0.08, 0.02, 0), budget_s=15)
        gap = r.motion_walls(['front'], 0)['front'].gap
        assert abs(gap - policy().front_target_m) <= policy().front_tolerance_m
        assert any(command[0] > 0 for command in r.sent[after_lane:])
        assert p10.actual.x != pytest.approx(p10.original_nominal.x, abs=0.001)
        p11_path = Motion(
            'path', x=0.15, y=0.02, angle=0, points=((0.08, 0.02), (0.10, 0.02), (0.15, 0.02))
        )
        connected_p11 = runtime.connector(p11_path, c.book.origin, p10)
        assert connected_p11.points[0] == pytest.approx((p10.actual.x, p10.actual.y))
        assert connected_p11.points[-2:] == p11_path.points[-2:]
        after_front = len(r.sent)
        c.move(connected_p11)
        assert math.hypot(r.actual.x - 0.15, r.actual.y - 0.02) <= 0.005
        assert any(math.hypot(*command[:2]) > 0 for command in r.sent[after_front:])
        for endpoint in (p09, p10):
            stamps = [scan['scan_stamp_ns'] for scan in endpoint.scans]
            assert len(stamps) >= 3 and len(stamps) == len(set(stamps))
            assert stamps[-1] - stamps[0] >= policy().stable_span_s * 1e9
        assert r.velocity_pub is None and r.sent[-1] == (0.0, 0.0, 0.0)
        assert calls
        assert c.book.origin == Pose(0, 0, 0)


def test_feedback_pause_failure_never_accepts_and_releases_publisher():
    r = FakeRunner()
    c = configured_console(r)
    checks = []

    def fail():
        checks.append(r.now)
        if r.now >= 1.3:
            raise RuntimeError('synthetic feedback-pause abort')

    r.feedback_pause.check = fail
    with (
        patch.object(runtime, '_read_frame', lambda runner: runner.read_frame()),
        patch.object(runtime.time, 'monotonic', lambda: r.now),
    ):
        with pytest.raises(RuntimeError, match='feedback-pause'):
            runtime.CornerPlacement(c, policy(), enabled=True).place('P10', Pose(0.08, 0, 0))
    assert any(command[0] > 0 for command in r.sent)
    assert r.sent[-1] == (0.0, 0.0, 0.0) and r.velocity_pub is None
    assert checks


def test_original_turn_center_guard_still_rejects_over_four_cm():
    motion = Motion('turn', angle=math.pi / 2)
    with pytest.raises(RuntimeError, match='4 cm'):
        feedback(Pose(0, 0, 0), Pose(0.0401, 0, 0.1), motion, 0.04, 0.1)
