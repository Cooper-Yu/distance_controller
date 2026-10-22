"""First-command lifetime checks stay active through preparation delays; no ROS node."""

from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from action_plan import Pose
from motion_model import Motion
import route_return_calibration as calibration
from route_return_runner import Continuity, run_one


@pytest.fixture
def boundary(monkeypatch):
    state = dict(now=1.0, deadline=120.0, watcher=True, moved=False, checks=0, watch_checks=0)
    expected = Pose(0.0, 0.0, 0.0)
    runner = Mock(frame='odom', twist_frame='base_link')
    runner.node.get_publishers_info_by_topic.return_value = [
        SimpleNamespace(endpoint_gid=bytes([1]) * 24)
    ]
    runner.pose.return_value = expected
    runner.moving_pose.side_effect = lambda: Pose(0.004, 0.0, 0.0) if state['moved'] else expected

    def validate(report, binding):
        state['checks'] += 1
        if state['now'] > state['deadline']:
            raise ValueError('Scan calibration expired')
        return expected

    def watch(report):
        state['watch_checks'] += 1
        if not state['watcher']:
            raise ValueError('Original watcher process is no longer running')

    monkeypatch.setattr(calibration, 'validate_report', validate)
    monkeypatch.setattr(calibration, 'verify_watch', watch)
    continuity = Continuity(runner, '01' * 24, calibration=dict(source_binding={}))
    return state, runner, continuity


def test_preparation_delay_cannot_skip_first_command_expiry(boundary):
    state, runner, continuity = boundary
    state['now'] = 119.0
    runner.exclusive.side_effect = lambda: state.update(now=121.0)
    pose = Pose(0.0, 0.0, 0.0)
    action = SimpleNamespace(name='P02', kind='path', start=pose)
    sequence = SimpleNamespace(
        begin=Mock(return_value=Motion('move', x=0.01)),
        pending=(action, pose),
        cursor=1,
        finish=Mock(),
    )
    console = Mock(runner=runner)
    emitted = []
    console.move.side_effect = lambda *a, **k: emitted.append(continuity((0.01, 0.0, 0.0)))
    with pytest.raises(ValueError, match='expired'):
        run_one(console, sequence, Mock(), continuity, dict(done=True))
    assert emitted == []
    assert not continuity.motion_started
    sequence.finish.assert_not_called()


def test_zero_command_keeps_calibration_active(boundary):
    state, _, continuity = boundary
    assert continuity((0.0, 0.0, 0.0)) == (0.0, 0.0, 0.0)
    assert not continuity.motion_started
    state['watcher'] = False
    with pytest.raises(ValueError, match='watcher'):
        continuity((0.01, 0.0, 0.0))
    assert not continuity.motion_started


@pytest.mark.parametrize('fault', ['expiry', 'watcher', 'pose'])
def test_guard_delay_rechecked_before_first_nonzero_release(boundary, fault):
    state, _, continuity = boundary

    def delay(command):
        if fault == 'expiry':
            state['now'] = 121.0
        elif fault == 'watcher':
            state['watcher'] = False
        else:
            state['moved'] = True
        return command

    continuity.guard = delay
    with pytest.raises((RuntimeError, ValueError)):
        continuity((0.01, 0.0, 0.0))
    assert not continuity.motion_started


def test_guard_replacing_motion_with_zero_does_not_start_epoch(boundary):
    _, _, continuity = boundary
    continuity.guard = lambda command: (0.0, 0.0, 0.0)
    assert continuity((0.01, 0.0, 0.0)) == (0.0, 0.0, 0.0)
    assert not continuity.motion_started


def test_stopped_pose_wait_and_watch_io_cannot_extend_lifetime(boundary, monkeypatch):
    state, runner, continuity = boundary
    runner.pose.side_effect = lambda: state.update(now=121.0) or Pose(0.0, 0.0, 0.0)
    with pytest.raises(ValueError, match='expired'):
        continuity((0.01, 0.0, 0.0))
    runner.pose.side_effect = None
    state['now'] = 1.0
    monkeypatch.setattr(calibration, 'verify_watch', lambda report: state.update(now=121.0))
    with pytest.raises(ValueError, match='expired'):
        continuity((0.01, 0.0, 0.0))
    assert not continuity.motion_started


def test_first_release_then_continuous_gid_checks_remain(boundary):
    state, runner, continuity = boundary
    assert continuity((0.01, 0.0, 0.0)) == (0.01, 0.0, 0.0)
    assert continuity.motion_started
    checks = state['watch_checks']
    state.update(now=121.0, watcher=False, moved=True)
    assert continuity((0.01, 0.0, 0.0)) == (0.01, 0.0, 0.0)
    assert state['watch_checks'] == checks
    runner.node.get_publishers_info_by_topic.return_value = []
    with pytest.raises(RuntimeError, match='identity/count'):
        continuity((0.01, 0.0, 0.0))
