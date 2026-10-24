"""Independent runner contracts and actual isolated C++ initialization behavior."""

import io
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from unittest.mock import Mock, patch

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
import task6_route_runner as route
from action_plan import Pose
from verify_precise_correction import Room, rclpy


@pytest.mark.parametrize('value', ['nan', 'inf', '-inf', '.00499', '.01001'])
def test_runner_rejects_invalid_tolerance(value):
    with pytest.raises(ValueError):
        route.validated_start_position_tolerance(value)


@pytest.mark.parametrize('value', [0.005, 0.010])
def test_runner_forwards_exact_ros_parameter(value):
    runner = Mock(prepare_options=['-p', 'alignment_wall:=right'])
    runner.pose.return_value = Pose(0, 0, 0)
    runner.frame, runner.twist_frame = 'odom', 'base_link'
    audit = io.StringIO()
    with (
        patch.object(route, 'TeachingBook'),
        patch.object(route, 'Console'),
        patch.object(route, 'run_actions'),
        patch('route_feedback_pause.RouteFeedbackPause'),
    ):
        route.prepare_and_run(
            runner, Path('unused'), audit, (), 0.1, Mock(), start_position_tolerance=value
        )
    assert f'precise_start_position_tolerance:={value}' in runner.prepare_options
    assert 'precise_start:=true' in runner.prepare_options
    assert '"precise_start_position_tolerance": ' + str(value) in audit.getvalue()


def command(tolerance=None):
    args = ['ros2', 'run', 'distance_controller', 'distance_controller', '2', '--ros-args']
    values = [
        'manual_mode:=true',
        'start_paused:=true',
        'alignment_wall:=right',
        'robust_wall_heading:=true',
        'positioning_wall:=right',
        'right_target_distance:=0.26',
        'precise_start:=true',
    ]
    if tolerance is not None:
        values.append('precise_start_position_tolerance:=' + str(tolerance))
    for value in values:
        args.extend(['-p', value])
    return args


class ToleranceRoom(Room):
    def __init__(self, output, residual, heading):
        super().__init__('tolerance', output)
        self.residual, self.heading = residual, heading

    def distances(self, now):
        if self.verify_start is None:
            return 0.26, 0.28
        return 0.26 + self.residual, 0.28 + self.residual

    def feedback_pose(self, now):
        super().feedback_pose(now)
        if self.verify_start is not None:
            self.pose[2] = self.heading


@pytest.mark.parametrize(
    'tolerance,residual,heading,accepted',
    [
        (None, 0.008, 0.0, False),
        (0.010, 0.008, 0.0, True),
        (0.010, 0.014, 0.0, False),
        (0.010, 0.008, 0.020, False),
    ],
)
def test_actual_cpp_gate(tmp_path, tolerance, residual, heading, accepted):
    assert os.environ['ROS_DOMAIN_ID'] == '194'
    assert os.environ['ROS_LOCALHOST_ONLY'] == '1'
    rclpy.init()
    room = ToleranceRoom(tmp_path, residual, heading)
    process = None
    try:
        with (tmp_path / 'runtime.log').open('w') as log:
            process = subprocess.Popen(
                command(tolerance), stdout=log, stderr=subprocess.STDOUT, start_new_session=True
            )
            deadline = time.monotonic() + 20
            while time.monotonic() < deadline:
                room.tick()
                text = (tmp_path / 'runtime.log').read_text()
                if 'START_VERIFIED:' in text or 'START_REFINE:' in text:
                    break
            else:
                pytest.fail('no completed first verification batch: ' + text[-3000:])
        assert ('START_VERIFIED:' in text) == accepted, text
        if accepted:
            assert 'within 0.0100 m' in text and '0.008727 rad' in text, text
        else:
            assert 'START_REFINE:' in text, text
        assert room.samples
    finally:
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=8)
        for _ in range(15):
            room.tick()
        assert not room.node.get_publishers_info_by_topic('/cmd_vel')
        room.node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('value', ['0.004', '0.011', '.nan', '.inf', '-.inf'])
def test_actual_cpp_rejects_invalid_parameter(value):
    completed = subprocess.run(command(value), capture_output=True, text=True, timeout=8)
    assert completed.returncode != 0
    assert 'precise_start_position_tolerance' in completed.stdout + completed.stderr
