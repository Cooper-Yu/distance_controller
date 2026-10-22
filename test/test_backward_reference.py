"""Synthetic reference contracts only: no ROS plant, hardware or epoch certification."""

import math
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from action_plan import Pose
from motion_console import Console
from motion_model import Motion, feedback, local_delta, target_pose
from teaching_path import PathTracker, fixed_segment, path_ready


def console_fixture(origin, current):
    """Keep real move/stop-check logic; replace transport and physical execution only."""
    runner = SimpleNamespace(
        exclusive=Mock(),
        pose=Mock(return_value=current),
        frame='odom',
        twist_frame='base_link',
        max_speed=0.01,
    )
    console = Console(runner, SimpleNamespace(origin=origin), None, None, 0.10)
    console.cancel_input = Mock()
    console.emit = Mock()
    console.stop_motion = Mock()
    console.measure = Mock()
    console.finish_motion = Mock()
    return console


def in_reference(origin, x, y, yaw):
    return target_pose(origin, Motion('move', x=x, y=y, angle=yaw))


@pytest.mark.parametrize('residual', [-0.008, 0.008])
@pytest.mark.parametrize('initial_yaw', [0.0, 0.7])
def test_two_actual_move_dispatches_inherit_previous_residual(residual, initial_yaw):
    origin = Pose(2.0, -1.0, initial_yaw)
    motion = Motion('move', x=-0.03)
    first = console_fixture(origin, origin)
    end1 = in_reference(origin, -0.03, 0.0, residual)
    first.finish_motion.return_value = end1
    first.move(motion)
    start1 = first.finish_motion.call_args.args[0]
    assert start1 == origin
    assert first.stopped_check(start1, end1, motion, 0)[0]

    # The next invocation creates a new book and re-anchors at the stopped pose.
    second = console_fixture(end1, end1)
    end2 = in_reference(end1, -0.03, 0.0, residual)
    second.finish_motion.return_value = end2
    second.move(motion)
    start2 = second.finish_motion.call_args.args[0]
    assert start2 == end1
    assert second.stopped_check(start2, end2, motion, 0)[0]
    assert end2.yaw - origin.yaw == pytest.approx(2 * residual)
    assert abs(end2.yaw - origin.yaw) > 0.01
    assert local_delta(origin, end2)['y_m'] == pytest.approx(-0.03 * math.sin(residual))
    # At the second start the existing controller sees no inherited heading error.
    assert feedback(start2, end1, motion, 0.01, 0.10)[0][2] == 0.0


@pytest.mark.parametrize('residual', [-0.008, 0.008])
def test_fixed_reference_path_dispatch_keeps_original_yaw(residual):
    origin = Pose(2.0, -1.0, 0.7)  # Synthetic newly bound epoch, never historical P01.
    current = in_reference(origin, -0.03, 0.002, residual)
    motion = fixed_segment((-0.03, 0.0), (-0.06, 0.0), 0.0)
    console = console_fixture(origin, current)
    console.finish_motion.return_value = in_reference(origin, -0.06, 0.0, 0.0)
    console.move(motion)
    assert console.finish_motion.call_args.args[0] == origin
    tracker = PathTracker(origin, motion.points, motion.angle)
    command, ready = tracker.feedback(current, 0.01, 0.10)
    assert not ready
    assert command[2] * residual < 0
    # Rotate actual-body output back into fixed reference axes to check signs.
    c, s = math.cos(residual), math.sin(residual)
    vx, vy = c * command[0] - s * command[1], s * command[0] + c * command[1]
    assert vx < 0 and vy < 0
    assert math.hypot(*command[:2]) <= 0.01 + 1e-12
    assert abs(command[2]) <= 0.10


@pytest.mark.parametrize('lateral', [-0.003, 0.003])
@pytest.mark.parametrize('yaw', [-0.015, 0.015])
def test_backward_fixed_path_restores_both_error_signs(lateral, yaw):
    origin = Pose(4.0, 5.0, -0.8)
    points = ((0.0, 0.0), (-0.03, 0.0))
    actual = in_reference(origin, -0.01, lateral, yaw)
    command, ready = PathTracker(origin, points).feedback(actual, 0.01, 0.10)
    c, s = math.cos(yaw), math.sin(yaw)
    fixed_vx = c * command[0] - s * command[1]
    fixed_vy = s * command[0] + c * command[1]
    assert fixed_vx < 0
    assert fixed_vy * lateral < 0
    assert command[2] * yaw < 0
    assert not ready


@pytest.mark.parametrize(
    'dx,dy,yaw,expected',
    [
        (0.003, 0.003, 0.009, True),
        (0.004, 0.004, 0.0, False),
        (0.0, 0.0, 0.0101, False),
        (0.0051, 0.0, 0.0, False),
    ],
)
def test_fixed_path_arrival_uses_endpoint_norm_and_original_heading(dx, dy, yaw, expected):
    origin = Pose(1.0, 2.0, 0.7)
    motion = fixed_segment((-0.03, 0.0), (-0.06, 0.0), 0.0)
    actual = in_reference(origin, -0.06 + dx, dy, yaw)
    assert path_ready(origin, actual, motion.points) is expected
    assert PathTracker(origin, motion.points).feedback(actual, 0.01, 0.10)[1] is expected
    assert console_fixture(origin, actual).stopped_check(origin, actual, motion, 0)[0] is expected


def test_fixed_path_rejects_start_outside_existing_heading_gate():
    origin = Pose(0.0, 0.0, 0.0)
    console = console_fixture(origin, Pose(-0.03, 0.0, math.radians(1.1)))
    with pytest.raises(RuntimeError, match='Path requires current pose'):
        console.move(fixed_segment((-0.03, 0.0), (-0.06, 0.0), 0.0))
    console.finish_motion.assert_not_called()
