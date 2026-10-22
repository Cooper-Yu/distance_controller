"""Shadow swept-demand evidence: geometry, history invalidation and occupied precedence."""

from dataclasses import replace
import math
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from route_swept_coverage import CoverageHistory, CurrentState, LaserMount, Pose2, ScanEvidence


def scan(stamp=10.0, holes=(), mount=None, pose=None, epoch='epoch-A'):
    rays = tuple((math.radians(i), None if i in holes else 2.0) for i in range(360))
    return ScanEvidence(
        stamp_ns=int(stamp * 1e9),
        received_s=stamp,
        odom_stamp_ns=int(stamp * 1e9),
        odom_pose=pose or Pose2(0, 0, 0),
        mount=mount or LaserMount(0, 0, 0.173, 0, 0, 0),
        epoch=epoch,
        odom_frame='odom',
        base_frame='base_link',
        laser_frame='laser',
        range_min=0.05,
        range_max=10.0,
        angular_step=math.radians(1),
        rays=rays,
    )


def state(stamp=10.0, pose=None, epoch='epoch-A'):
    return CurrentState(
        pose=pose or Pose2(0, 0, 0),
        stamp_ns=int(stamp * 1e9),
        received_s=stamp,
        now_ns=int(stamp * 1e9),
        now_s=stamp,
        epoch=epoch,
        odom_frame='odom',
        base_frame='base_link',
    )


def assessment(frame, command, measured=(0, 0, 0)):
    history = CoverageHistory()
    history.add(frame)
    return history.assess(state(), command, measured)


def test_forward_demand_does_not_require_receding_rear_sector():
    report = assessment(scan(holes=range(90, 271)), (0.03, 0, 0))
    assert report['state'] == 'sampled_supported'
    assert report['demand_count'] > 0 and report['unknown_count'] == 0
    assert not report['executable'] and not report['certified_free']
    assert report['applied_command'] == (0.03, 0, 0)


def test_reverse_measured_motion_requires_rear_even_for_forward_command():
    report = assessment(scan(holes=range(90, 271)), (0.03, 0, 0), (-0.02, 0, 0))
    assert report['state'] == 'unknown' and report['unknown_count'] > 0
    assert report['unknown_bounds'][0][0] < -0.17


def test_rotation_demands_outward_corner_sweep_and_keeps_unknown():
    report = assessment(scan(holes=range(360)), (0, 0, 0.2))
    assert report['state'] == 'unknown' and report['demand_count'] > 0
    assert report['unknown_count'] == report['demand_count']
    assert report['demand_bounds'][0][0] < -0.17
    assert report['demand_bounds'][1][0] > 0.17


def test_real_sensor_yaw_and_offset_are_used_for_coverage():
    # Only laser-forward rays are valid. A pi-mounted lidar observes robot rear.
    frame = scan(holes=range(45, 316), mount=LaserMount(0.02, 0, 0.173, 0, 0, math.pi))
    assert assessment(frame, (0.03, 0, 0))['state'] == 'unknown'
    assert assessment(frame, (-0.03, 0, 0))['observed_point_constraint']['state'] == 'clear'
    # Offset affects the exact beam needed for this known query point.
    history = CoverageHistory()
    frame = replace(frame, mount=LaserMount(0.1, 0.1, 0.173, 0, 0, 0), rays=((0.0, 1.0),))
    history.add(frame)
    assert history.supported_points(state(), ((0.3, 0.1), (0.3, 0.0))) == [True, False]


def test_scan_time_odom_pose_is_not_replaced_by_current_pose():
    history = CoverageHistory()
    history.add(replace(scan(pose=Pose2(1, 2, math.pi / 2)), rays=((0.0, 1.0),)))
    current = state(10.2, pose=Pose2(1, 2.1, math.pi / 2))
    assert history.supported_points(current, ((0.2, 0), (0, 0.2))) == [True, False]


def test_recent_finite_history_can_support_temporary_missing_ray_only_in_shadow():
    history = CoverageHistory()
    history.add(scan(9.8))
    history.add(scan(10.0, holes=range(360)))
    report = history.assess(state(), (0.03, 0, 0), (0, 0, 0))
    assert report['state'] == 'sampled_supported' and report['frames_used'] == 2
    assert not report['executable']
    expired = replace(state(10.4), received_s=10.4)
    report = history.assess(expired, (0.03, 0, 0), (0, 0, 0))
    assert report['state'] == 'unknown' and report['unknown_count'] > 0


def test_latest_occupied_cannot_be_erased_by_older_far_return_or_majority():
    history = CoverageHistory()
    history.add(scan(9.8))
    history.add(scan(9.9))
    history.add(replace(scan(10.0), rays=((0.0, 0.18),)))
    report = history.assess(state(), (0.03, 0, 0), (0, 0, 0))
    assert report['state'] == 'occupied'
    assert report['observed_point_constraint']['state'] == 'blocked'
    assert report['latest_observed_point_count'] == 1


@pytest.mark.parametrize('change', ['epoch', 'mount', 'frame', 'stamp', 'receipt', 'jump'])
def test_history_resets_on_discontinuity(change):
    history = CoverageHistory()
    history.add(scan(9.9))
    second = scan(10.0, holes=range(360))
    if change == 'epoch':
        second = replace(second, epoch='epoch-B')
    elif change == 'mount':
        second = replace(second, mount=LaserMount(0.02, 0, 0.173, 0, 0, 0))
    elif change == 'frame':
        second = replace(second, laser_frame='other_laser')
    elif change == 'stamp':
        second = replace(second, stamp_ns=int(9.8e9), odom_stamp_ns=int(9.8e9))
    elif change == 'receipt':
        second = replace(second, received_s=9.8)
    else:
        second = replace(second, odom_pose=Pose2(1, 0, 0))
    history.add(second)
    assert len(history.frames) == 1 and history.discard_reasons


@pytest.mark.parametrize('change', ['missing_mount', 'tilt', 'nan_pose', 'pairing', 'invalid_ray'])
def test_missing_or_invalid_geometric_inputs_are_rejected(change):
    frame = scan()
    if change == 'missing_mount':
        frame = replace(frame, mount=None)
    elif change == 'tilt':
        frame = replace(frame, mount=LaserMount(0, 0, 0.173, 0.1, 0, 0))
    elif change == 'nan_pose':
        frame = replace(frame, odom_pose=Pose2(math.nan, 0, 0))
    elif change == 'pairing':
        frame = replace(frame, odom_stamp_ns=frame.stamp_ns - 200_000_000)
    else:
        frame = replace(frame, rays=((0.0, math.inf),))
    with pytest.raises(ValueError):
        CoverageHistory().add(frame)


def test_stale_current_feedback_never_produces_supported_result():
    history = CoverageHistory()
    history.add(scan())
    report = history.assess(replace(state(), now_s=11.0), (0.03, 0, 0), (0, 0, 0))
    assert report['state'] == 'unknown' and 'feedback' in report['reason']


def test_epoch_changed_at_query_drops_all_history():
    history = CoverageHistory()
    history.add(scan())
    report = history.assess(state(epoch='epoch-B'), (0.03, 0, 0), (0, 0, 0))
    assert report['state'] == 'unknown' and report['frames_used'] == 0


def test_query_pose_jump_without_gid_change_invalidates_history():
    history = CoverageHistory()
    history.add(scan())
    report = history.assess(state(pose=Pose2(1, 0, 0)), (0.03, 0, 0), (0, 0, 0))
    assert report['state'] == 'unknown' and report['frames_used'] == 0
    assert 'current_odom_discontinuity' in report['discard_reasons']


def test_extreme_velocity_is_bounded_before_grid_allocation():
    report = assessment(scan(), (1e6, 0, 0))
    assert report['state'] == 'unknown' and 'budget' in report['reason']


def test_reports_serialize_with_near_obstacle_and_invalid_candidate():
    import json

    frame = replace(scan(), rays=((0.0, 0.18),))
    for command in ((0.03, 0, 0), (math.nan, 0, 0)):
        report = assessment(frame, command)
        json.dumps(report, allow_nan=False)


def test_invalid_scheduled_ray_cannot_borrow_adjacent_finite_support():
    history = CoverageHistory()
    history.add(replace(scan(), rays=((0.0, 2.0), (math.radians(1), None), (math.radians(2), 2.0))))
    point = (0.3 * math.cos(math.radians(1)), 0.3 * math.sin(math.radians(1)))
    assert history.supported_points(state(), (point,)) == [False]


def test_feedback_gap_drops_old_finite_evidence():
    history = CoverageHistory()
    history.add(scan(9.4))
    history.add(scan(10.0, holes=range(360)))
    report = history.assess(state(), (0.03, 0, 0), (0, 0, 0))
    assert report['state'] == 'unknown' and report['frames_used'] == 1
    assert 'feedback_gap' in report['discard_reasons']


def test_invalid_input_discards_previously_good_history():
    history = CoverageHistory()
    history.add(scan())
    with pytest.raises(ValueError):
        history.add(replace(scan(10.1), mount=None))
    assert history.frames == []


def test_offline_reader_requires_closure_without_cross_host_pid_guess(tmp_path):
    import json
    import os

    from route_laser_shadow_replay import check_closed

    bag = tmp_path / 'bag'
    bag.mkdir()
    with pytest.raises(ValueError, match='confirm-closed'):
        check_closed(bag)
    with pytest.raises(ValueError, match='metadata'):
        check_closed(bag, True)
    (bag / 'metadata.yaml').write_text('fixture')
    unverified = check_closed(bag, True)
    assert not unverified['recorder_closed_metadata_verified']
    for value in (False, 'true', 1):
        (tmp_path / 'result.json').write_text(json.dumps(dict(closed_metadata=value)))
        with pytest.raises(ValueError, match='result'):
            check_closed(bag, True)
    (tmp_path / 'result.json').write_text(json.dumps(dict(closed_metadata=True)))
    # An archived cloud PID can equal a live local PID: never probe/reject it.
    (tmp_path / 'supervisor.pid').write_text(str(os.getpid()))
    assert check_closed(bag, True)['recorder_closed_metadata_verified']
    (bag / 'recording.db3-wal').touch()
    with pytest.raises(ValueError, match='WAL'):
        check_closed(bag, True)


@pytest.mark.parametrize('invalid', [math.nan, 10.0, 0])
def test_invalid_source_stamps_are_not_history_evidence(invalid):
    with pytest.raises(ValueError, match='stamps'):
        CoverageHistory().add(replace(scan(), stamp_ns=invalid))


@pytest.mark.parametrize('command', [(0, 0, 0.1), (0.03, 0, 0)])
def test_real_min_range_front_seam_remains_unknown_even_with_finite_far_returns(command):
    # The recorded sensor origin x=.02 and range_min=.15 meet the body front x=.17.
    # Ideal finite returns isolate the model seam; they do not fill real missing rays.
    frame = replace(scan(mount=LaserMount(0.02, 0, 0.173, 0, 0, math.pi)), range_min=0.15)
    report = assessment(frame, command)
    assert report['state'] == 'unknown' and report['unknown_count'] > 0
    assert report['observed_point_constraint']['state'] == 'clear'
    assert all(x > 0.17 and abs(y) < 0.02 for x, y in report['unknown_examples'])
    assert report['applied_command'] == command
    assert not report['executable'] and not report['certified_free']


def test_real_min_range_does_not_make_every_direction_unknown():
    frame = replace(scan(mount=LaserMount(0.02, 0, 0.173, 0, 0, math.pi)), range_min=0.15)
    report = assessment(frame, (-0.03, 0, 0))
    assert report['state'] == 'sampled_supported' and report['unknown_count'] == 0
    assert not report['executable'] and not report['certified_free']


def test_finite_front_ray_does_not_certify_grid_margin_inside_min_range():
    from route_swept_coverage import frame_support

    # This exterior center is observable under the point model, but its grid margin
    # reaches the minimum-range disk. Keep that distinction visible without relaxing it.
    point = (0.1729, 0.0)
    frame = replace(
        scan(mount=LaserMount(0.02, 0, 0.173, 0, 0, math.pi)),
        range_min=0.15,
        rays=((math.pi, 1.0),),
    )
    assert frame_support(frame, (point,), uncertainty=0.0).tolist() == [True]
    assert frame_support(frame, (point,), uncertainty=0.005 / math.sqrt(2)).tolist() == [False]


@pytest.mark.parametrize('command', [(0, 0, 0.1), (0.03, 0, 0)])
def test_strictly_internal_blind_disk_excludes_only_internal_lower_bound_gap(command):
    frame = replace(scan(mount=LaserMount(0.019, 0, 0.173, 0, 0, math.pi)), range_min=0.15)
    report = assessment(frame, command)
    assert report['state'] == 'sampled_supported' and report['unknown_count'] == 0
    proof = report['minimum_range_body_proofs'][0]
    assert proof['strictly_contained']
    assert proof['containment_margin_m'] == pytest.approx(0.001)
    assert not report['executable'] and report['applied_command'] == command


@pytest.mark.parametrize('radius', [0.15, 0.15000000596046448, 0.150001])
def test_tangent_or_microscopic_external_blind_disk_is_not_exempt(radius):
    frame = replace(scan(mount=LaserMount(0.02, 0, 0.173, 0, 0, math.pi)), range_min=radius)
    report = assessment(frame, (0, 0, 0.1))
    assert report['state'] == 'unknown' and report['unknown_count'] > 0
    assert not report['minimum_range_body_proofs'][0]['strictly_contained']


def test_containment_uses_scan_time_mount_in_current_body_coordinates():
    from route_swept_coverage import minimum_range_body_proof

    frame = replace(
        scan(mount=LaserMount(0.019, 0, 0.173, 0, 0, math.pi), pose=Pose2(0.005, 0, 0)),
        range_min=0.15,
    )
    old_body = minimum_range_body_proof(frame, Pose2(0.005, 0, 0))
    current_body = minimum_range_body_proof(frame, Pose2(0, 0, 0))
    assert old_body['strictly_contained']
    assert not current_body['strictly_contained']
    assert current_body['center_current_base'] == pytest.approx((0.024, 0))
    frame = replace(frame, odom_pose=Pose2(0, 0, math.pi / 2))
    rotated = minimum_range_body_proof(frame, Pose2(0, 0, 0))
    assert rotated['center_current_base'] == pytest.approx((0, 0.019))
    assert not rotated['strictly_contained']


def test_rectangle_union_corner_is_not_filled_by_bounding_box_containment():
    from route_swept_coverage import minimum_range_body_proof

    frame = replace(scan(mount=LaserMount(0.13, 0.13, 0.173, 0, 0, 0)), range_min=0.03)
    proof = minimum_range_body_proof(frame, Pose2(0, 0, 0))
    assert proof['distance_to_body_exterior_m'] == pytest.approx(math.sqrt(2) * 0.005)
    assert not proof['strictly_contained']


@pytest.mark.parametrize('obstacle_range', [0.159, 0.2])
def test_internal_blind_disk_neither_fills_missing_rays_nor_overrides_occupied(obstacle_range):
    frame = replace(scan(mount=LaserMount(0.019, 0, 0.173, 0, 0, math.pi)), range_min=0.15)
    missing = replace(frame, rays=tuple((a, None) for a, _ in frame.rays))
    assert assessment(missing, (0, 0, 0.1))['state'] == 'unknown'
    # Both a rear overlap and an exterior point within the hard margin stay authoritative.
    obstacle = replace(frame, rays=((0.0, obstacle_range), *frame.rays[1:]))
    occupied = assessment(obstacle, (-0.03, 0, 0))
    assert occupied['minimum_range_body_proofs'][0]['strictly_contained']
    assert occupied['state'] == 'occupied'


def test_internal_queries_do_not_receive_current_exterior_exemption():
    from route_swept_coverage import frame_support

    frame = replace(scan(mount=LaserMount(0.019, 0, 0.173, 0, 0, 0)), range_min=0.15)
    assert frame_support(frame, ((0.03, 0),), 0.005 / math.sqrt(2), Pose2(0, 0, 0)).tolist() == [
        False
    ]


def test_internal_blind_disk_does_not_remove_return_endpoint_margin():
    from route_swept_coverage import frame_support

    frame = replace(
        scan(mount=LaserMount(0.019, 0, 0.173, 0, 0, 0)), range_min=0.15, rays=((0.0, 0.25),)
    )
    assert frame_support(frame, ((0.268, 0),), 0.005 / math.sqrt(2), Pose2(0, 0, 0)).tolist() == [
        False
    ]
