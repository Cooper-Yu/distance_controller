"""Pure replay adapter contracts; synthetic messages, no ROS node or bag writer."""

from copy import deepcopy
from dataclasses import asdict
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace as N

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from route_laser_guard import GuardPolicy
from route_observed_guard import ObservedPolicy
from route_observed_guard_replay import (
    TOPICS,
    check_static_transform,
    candidate_at,
    evaluate_streams,
    file_evidence,
    paired_input,
    read_policy,
    unique_records,
)


def policy():
    return ObservedPolicy(
        GuardPolicy(0.02, 0.035, 0.5, 0.002, math.radians(1)),
        30,
        0.5,
        0.95,
        12,
        2,
        0.25,
        0.5,
        0.5,
        0.5,
        0.15,
        0.05,
    )


def stamp(seconds=10):
    return N(sec=seconds, nanosec=0)


def scan():
    return N(
        header=N(stamp=stamp(), frame_id='laser'),
        ranges=[2.0] * 360,
        angle_min=-math.pi,
        angle_max=math.pi - math.pi / 180,
        angle_increment=math.pi / 180,
        time_increment=0.0,
        scan_time=0.1,
        range_min=0.15,
        range_max=10.0,
    )


def transform():
    return N(translation=N(x=0.02, y=0.0, z=0.1), rotation=N(x=0.0, y=0.0, z=0.0, w=1.0))


def twist(x=0.0):
    return N(linear=N(x=x, y=0.0), angular=N(z=0.0))


def odom():
    return N(
        header=N(stamp=stamp(), frame_id='odom'),
        child_frame_id='base_link',
        pose=N(pose=N(position=N(x=0.0, y=0.0), orientation=N(x=0.0, y=0.0, z=0.0, w=1.0))),
        twist=N(twist=twist()),
    )


def streams():
    return {
        '/scan': [(10_010_000_000, 10_000_000_000, scan())],
        '/scan_filtered': [(10_020_000_000, 10_000_000_000, scan())],
        '/odometry/filtered': [(10_005_000_000, 10_000_000_000, odom())],
        '/cmd_vel': [(10_000_000_000 + i, None, twist(0.03)) for i in range(1000)],
    }


def evaluate(source=None, candidate=(0.03, 0, 0), mount=None):
    return list(
        evaluate_streams(source or streams(), mount or (lambda _: transform()), policy(), candidate)
    )


def test_missing_policy_fields_cannot_get_defaults(tmp_path):
    path = tmp_path / 'policy.json'
    data = asdict(policy())
    path.write_text(json.dumps(data))
    assert read_policy(path) == policy()
    del data['obstacle']['hard_m']
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError, match='explicit'):
        read_policy(path)


def test_absent_provenance_is_none_and_actual_bytes_hashed(tmp_path):
    assert file_evidence(None) is None
    path = tmp_path / 'filter.yaml'
    path.write_text('actual input')
    before = file_evidence(path)
    path.write_text('changed input')
    assert before['sha256'] != file_evidence(path)['sha256']


def test_duplicate_keeps_original_receipt_and_does_not_repeat_evaluation():
    records = [('/scan_filtered', 100, 90, b'a', scan()), ('/scan_filtered', 110, 90, b'a', scan())]
    rows, repeated = unique_records(records)
    assert rows['/scan_filtered'][0][0] == 100
    assert len(rows['/scan_filtered']) == 1
    assert repeated['/scan_filtered'] == 1


@pytest.mark.parametrize('topic', TOPICS[:-1])
def test_changed_duplicate_and_backward_stamp_rejected(topic):
    with pytest.raises(ValueError, match='changed payload'):
        unique_records([(topic, 100, 90, b'a', None), (topic, 110, 90, b'b', None)])
    with pytest.raises(ValueError, match='backward'):
        unique_records([(topic, 100, 90, b'a', None), (topic, 110, 80, b'a', None)])


def test_thousand_recorded_commands_evaluated_once_with_exact_original_time():
    result = evaluate(candidate=None)
    assert len(result) == 1
    assert result[0]['recorded_cmd_bag_receipt_ns'] == 10_000_000_999
    assert result[0]['candidate'] == (0.03, 0.0, 0.0)
    assert result[0]['decision']['state'] == 'observed_clear'
    assert result[0]['numerical_only'] and not result[0]['executable']


def test_explicit_candidate_does_not_require_command_or_configuration():
    source = streams()
    source['/cmd_vel'] = []
    result = evaluate(source)[0]
    assert result['decision']['state'] == 'observed_clear'
    assert result['recorded_cmd_bag_receipt_ns'] is None


def test_recorded_command_future_not_selected_and_stale_rejected():
    commands = [(9_000_000_000, None, twist(0.01)), (11_000_000_000, None, twist(0.02))]
    with pytest.raises(ValueError, match='older'):
        candidate_at(commands, 10_000_000_000, None, policy())


def test_raw_masked_close_point_retained_even_without_filter_manifest():
    source = streams()
    source['/scan'][0][2].ranges[180] = 0.16
    source['/scan_filtered'][0][2].ranges[180] = float('nan')
    result = evaluate(source)[0]
    assert result['point_count'] == 719
    assert result['decision']['state'] == 'stop'
    assert result['decision']['obstacle']['state'] == 'blocked'


@pytest.mark.parametrize(
    'problem', ['raw_missing', 'identity', 'odom_missing', 'frame', 'quaternion', 'tf']
)
def test_incomplete_inputs_report_unavailable_or_stop(problem):
    source = streams()
    mount = None
    if problem == 'raw_missing':
        source['/scan'] = []
    elif problem == 'identity':
        source['/scan'][0][2].scan_time = 0.2
    elif problem == 'odom_missing':
        source['/odometry/filtered'] = []
    elif problem == 'frame':
        source['/odometry/filtered'][0][2].child_frame_id = 'wrong'
    elif problem == 'quaternion':
        source['/odometry/filtered'][0][2].pose.pose.orientation.w = 0.0
    else:

        def mount(_):
            raise ValueError('missing TF')

    assert evaluate(source, mount=mount)[0]['decision']['state'] in ('unavailable', 'stop')


def test_current_measured_velocity_uses_current_odom_not_scan_pair():
    source = streams()
    newer = deepcopy(odom())
    newer.header.stamp.nanosec = 15_000_000
    newer.twist.twist.linear.x = 0.2
    source['/odometry/filtered'].append((10_019_000_000, 10_015_000_000, newer))
    snapshot, feedback, measured, now = paired_input(
        source['/scan_filtered'][0],
        source['/scan'][0],
        source['/odometry/filtered'],
        transform(),
        policy(),
    )
    assert snapshot.paired_odom_stamp_ns == 10_000_000_000
    assert feedback.stamp_ns == 10_015_000_000
    assert measured == (0.2, 0.0, 0.0)
    assert now == 10_020_000_000


def test_changed_static_mount_is_not_silently_reapplied_to_old_scan():
    mount = N(header=N(frame_id='base_link'), child_frame_id='laser', transform=transform())
    known = {}
    check_static_transform(known, mount)
    check_static_transform(known, deepcopy(mount))
    mount.transform.translation.x += 0.001
    with pytest.raises(ValueError, match='static TF changed'):
        check_static_transform(known, mount)
