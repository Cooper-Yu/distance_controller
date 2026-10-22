"""Closed real ROS message bag selection and provenance regressions; no ROS graph."""

from copy import deepcopy
import importlib.util
import json
from pathlib import Path
import sqlite3
import sys

from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
import pytest
from rclpy.serialization import serialize_message
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

from test_route_return_calibration import report_fixture, reference_fixture

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
from route_return_calibration import validate_reference, verify_reference_files  # noqa: E402

spec = importlib.util.spec_from_file_location(
    'reference_scan', ROOT / 'tools/task6_ops/scan_compare.py'
)
scan_compare = importlib.util.module_from_spec(spec)
spec.loader.exec_module(scan_compare)


def make_bag(folder, offset=0, odom_offset=0):
    """Selected medoid is first scan while last odom is 4 mm away."""
    bag = folder / 'bag'
    bag.mkdir()
    (folder / 'result.json').write_text('{"closed_metadata":true}')
    (folder / 'supervisor.pid').write_text('99999999')
    (bag / 'metadata.yaml').write_text(
        'rosbag2_bagfile_information:\n  relative_file_paths: [test.db3]\n'
    )
    con = sqlite3.connect(bag / 'test.db3')
    con.execute('CREATE TABLE topics(id INTEGER,name TEXT)')
    con.execute('CREATE TABLE messages(topic_id INTEGER,timestamp INTEGER,data BLOB)')
    for i, name in enumerate(('/scan_filtered', '/odometry/filtered', '/tf_static'), 1):
        con.execute('INSERT INTO topics VALUES(?,?)', (i, name))
    tf = TransformStamped()
    tf.header.frame_id, tf.child_frame_id = 'base_link', 'laser'
    tf.transform.rotation.w = 1.0
    con.execute(
        'INSERT INTO messages VALUES(?,?,?)',
        (3, 89_000_000_000, serialize_message(TFMessage(transforms=[tf]))),
    )
    for i in range(12):
        stamp = 90_000_000_000 + i * 100_000_000 + offset
        scan = LaserScan()
        scan.header.frame_id = 'laser'
        scan.header.stamp.sec, scan.header.stamp.nanosec = divmod(stamp, 10**9)
        scan.range_min, scan.range_max = 0.15, 10.0
        scan.angle_min, scan.angle_increment = -3.14, 0.02
        scan.ranges = [1.0] * 315
        odom = Odometry()
        odom.header.frame_id, odom.child_frame_id = 'odom', 'base_link'
        odom.header.stamp.sec, odom.header.stamp.nanosec = divmod(stamp + odom_offset, 10**9)
        odom.pose.pose.orientation.w = 1.0
        odom.pose.pose.position.x = i * 0.004 / 11
        con.execute('INSERT INTO messages VALUES(?,?,?)', (1, stamp, serialize_message(scan)))
        con.execute('INSERT INTO messages VALUES(?,?,?)', (2, stamp, serialize_message(odom)))
    con.commit()
    con.close()
    return dict(
        source_hash='h',
        session_id='s',
        completed_time=90.0,
        last_stop=dict(x=0.0, y=0.0, yaw=0.0),
        original_stamp_ns=90_000_000_000,
    )


def test_selected_scan_uses_paired_odom_and_bag_bytes(tmp_path):
    binding = make_bag(tmp_path)
    _, _, reference = scan_compare.closed_reference(tmp_path, binding)
    assert reference['odom_pose']['x'] == 0.0  # Not the final odom x=.004.
    assert reference['scan_stamp_ns'] == reference['odom_stamp_ns'] == 90_000_000_000
    source = tmp_path / 'points.json'
    source.write_text('{}')
    verify_reference_files(reference, source)
    with (tmp_path / 'bag/test.db3').open('ab') as stream:
        stream.write(b'changed')
    with pytest.raises(ValueError, match='bytes changed'):
        verify_reference_files(reference, source)


def test_same_numeric_pose_in_later_epoch_is_not_reference(tmp_path):
    binding = make_bag(tmp_path, offset=10_000_000_000)
    with pytest.raises(RuntimeError, match='completion boundary'):
        scan_compare.closed_reference(tmp_path, binding)


def test_reference_scan_requires_nearby_odom(tmp_path):
    binding = make_bag(tmp_path, odom_offset=60_000_000)
    with pytest.raises(ValueError, match='50 ms'):
        scan_compare.closed_reference(tmp_path, binding)


def test_current_scan_uses_its_paired_pose_not_final_callback():
    samples = [
        dict(stamp_ns=1_000_000_000, pose='selected'),
        dict(stamp_ns=1_100_000_000, pose='last'),
    ]
    assert scan_compare.paired_current(samples, 1_010_000_000)['pose'] == 'selected'
    with pytest.raises(RuntimeError, match='50 ms'):
        scan_compare.paired_current(samples, 1_200_000_000)


@pytest.mark.parametrize('value', [float('nan'), 0.01])
def test_reference_invalid_or_nonzero_command_rejected(tmp_path, value):
    binding = make_bag(tmp_path)
    command = Twist()
    command.linear.z = value
    with sqlite3.connect(tmp_path / 'bag/test.db3') as con:
        con.execute("INSERT INTO topics VALUES(4,'/cmd_vel')")
        con.execute(
            'INSERT INTO messages VALUES(?,?,?)', (4, 90_500_000_000, serialize_message(command))
        )
    with pytest.raises(RuntimeError, match='nonzero cmd_vel'):
        scan_compare.closed_reference(tmp_path, binding)


@pytest.mark.parametrize('fault', ['time', 'epoch', 'pair', 'binding'])
def test_report_rejects_reference_provenance_mismatch(fault):
    _, binding, _ = report_fixture()
    reference = reference_fixture(binding)
    if fault == 'time':
        reference['window_end_bag_ns'] += 10**10
    elif fault == 'epoch':
        reference['scan_stamp_ns'] += 10**10
    elif fault == 'pair':
        reference['odom_stamp_ns'] += 51_000_000
    else:
        reference['source_binding'] = deepcopy(binding)
        reference['source_binding']['session_id'] = 'other'
    with pytest.raises(ValueError):
        validate_reference(reference, binding)


def test_source_directory_cannot_be_swapped(tmp_path):
    _, binding, _ = report_fixture()
    reference = reference_fixture(binding, tmp_path)
    other = tmp_path / 'other'
    other.mkdir()
    source = other / 'points.json'
    source.write_text(json.dumps({}))
    with pytest.raises(ValueError, match='session directory'):
        verify_reference_files(reference, source)
