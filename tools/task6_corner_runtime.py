"""Opt-in Task6 corner placement support; no ROS initialization on import.

Main owns activation and route integration. An accepted endpoint is evidence for
an explicit local route revision, never a reset of P01 or odometry.
"""

from dataclasses import asdict, dataclass, replace
import math
import time

from action_plan import Pose
from odom_path import limited_velocity
from motion_model import HeadingPid, local_delta
from route_laser_guard import ScanSnapshot
from route_laser_runtime import current_points, scan_transform, scheduled_rays
from wall_runtime import WallFitError


class CornerUnknownError(RuntimeError):
    """Unproved local geometry: zero and await a later frame within the same deadline."""


@dataclass(frozen=True)
class AcceptedEndpoint:
    name: str
    original_nominal: Pose
    actual: Pose
    scans: tuple
    evidence: tuple


def connector(motion, origin, accepted, actual=None, max_revision_m=0.15):
    """Explicitly revise a path start; retain all later supplied points and final endpoint."""
    if not isinstance(accepted, AcceptedEndpoint) or accepted.name not in ('P09', 'P10'):
        raise ValueError('A verified corner endpoint is required')
    if not 0.15 <= max_revision_m <= 0.20 or motion.kind != 'path' or len(motion.points) < 2:
        raise ValueError('Invalid bounded corner connector')
    if (
        math.hypot(
            accepted.actual.x - accepted.original_nominal.x,
            accepted.actual.y - accepted.original_nominal.y,
        )
        > max_revision_m
    ):
        raise RuntimeError('Accepted endpoint revision exceeds connector bound')
    nominal = local_delta(origin, accepted.original_nominal)
    if math.dist((nominal['x_m'], nominal['y_m']), motion.points[0]) > 1e-6:
        raise RuntimeError('Connector is not bound to the original nominal start')
    actual = accepted.actual if actual is None else actual
    if math.hypot(actual.x - accepted.actual.x, actual.y - accepted.actual.y) > 0.02:
        raise RuntimeError('Post-turn actual pose exceeds accepted start by 2 cm')
    heading_error = math.remainder(actual.yaw - origin.yaw - motion.angle, 2 * math.pi)
    if abs(heading_error) > math.radians(1):
        raise RuntimeError('Connector heading exceeds 1 degree')
    delta = local_delta(origin, actual)
    start, end = (delta['x_m'], delta['y_m']), motion.points[1]
    distance = math.dist(start, end)
    if distance <= 1e-9:
        raise RuntimeError('Connector would duplicate the next fixed point')
    count = max(1, math.ceil(distance / 0.05))
    prefix = tuple(
        (start[0] + (end[0] - start[0]) * i / count, start[1] + (end[1] - start[1]) * i / count)
        for i in range(count)
    )
    return replace(motion, points=(*prefix, *motion.points[1:]))


@dataclass(frozen=True)
class _Frame:
    snapshot: ScanSnapshot
    rays: tuple
    pose: Pose
    stamp: int
    odom_stamp: int
    identity: tuple
    laser_origin: tuple
    raw_signature: str


def _read_frame(runner):
    """Reuse original freshness/TF/pairing checks, transporting points without deletion."""
    points, _counts = runner.geometry()
    at_scan = runner.scan_pose()
    current = runner.moving_pose()
    if runner.frame != 'odom' or runner.twist_frame != 'base_link':
        raise RuntimeError('Corner feedback frame changed')
    scan = runner.observer.scan
    transform = scan_transform(runner, scan)
    scheduled = scheduled_rays(scan, transform, at_scan.yaw - current.yaw)
    q, t = transform.rotation, transform.translation
    xyzq = (t.x, t.y, t.z, q.x, q.y, q.z, q.w)
    identity = (
        scan.header.frame_id,
        len(scan.ranges),
        scan.angle_min,
        scan.angle_increment,
        scan.range_min,
        scan.range_max,
        *xyzq,
    )
    if not all(math.isfinite(v) for v in identity[1:]):
        raise RuntimeError('Nonfinite corner scan identity')
    planar = []
    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    x, y, z, w = (v / norm for v in (q.x, q.y, q.z, q.w))
    for i, (bearing, valid) in enumerate(scheduled):
        angle = scan.angle_min + i * scan.angle_increment
        dx = (1 - 2 * (y * y + z * z)) * math.cos(angle) + 2 * (x * y - z * w) * math.sin(angle)
        dy = 2 * (x * y + z * w) * math.cos(angle) + (1 - 2 * (x * x + z * z)) * math.sin(angle)
        planar.append((bearing, valid, scan.ranges[i] * math.hypot(dx, dy) if valid else None))
    origin = current_points(((t.x, t.y, 0.0),), at_scan, current)[0]
    return _Frame(
        ScanSnapshot(current_points(points, at_scan, current), scheduled),
        tuple(planar),
        current,
        runner.cached_stamp,
        runner.stamp,
        identity,
        origin,
        repr(tuple(scan.ranges)),
    )


class CornerPlacement:
    """Opt-in single-publisher placement; Main explicitly chooses only P09/P10."""

    def __init__(
        self, console, policy, *, enabled=False, max_revision_m=0.20, front_sweep_required=True
    ):
        if enabled is not True or not 0.15 <= max_revision_m <= 0.20:
            raise ValueError('Explicit bounded corner opt-in required')
        policy.validate()
        if type(front_sweep_required) is not bool:
            raise ValueError('Explicit front sweep mode required')
        self.front_sweep_required = front_sweep_required
        self.console, self.runner, self.policy = console, console.runner, policy
        self.max_revision_m = max_revision_m
        self.identity, self.last_stamp, self.raw_signature = None, None, None

    def _frame(self):
        frame = _read_frame(self.runner)
        if self.identity is not None and frame.identity != self.identity:
            raise RuntimeError('Corner scan identity changed')
        if self.last_stamp is not None and frame.stamp < self.last_stamp:
            raise RuntimeError('Corner scan timestamp moved backward')
        if frame.stamp == self.last_stamp and frame.raw_signature != self.raw_signature:
            raise RuntimeError('Corner scan changed at unchanged source timestamp')
        self.identity, self.last_stamp = frame.identity, frame.stamp
        self.raw_signature = frame.raw_signature
        return frame

    def place(self, name, nominal, *, budget_s=45.0):  # noqa: PLR0912, PLR0915 -- one bounded owner/cleanup lifetime
        """Track selected geometry, then accept only distinct fresh post-zero observations."""
        from task6_corner_geometry import StoppedPlacementGate

        if name not in ('P09', 'P10') or not math.isfinite(budget_s) or not 0 < budget_s <= 90:
            raise ValueError('Unsupported corner action or budget')
        if not all(math.isfinite(v) for v in (nominal.x, nominal.y, nominal.yaw)):
            raise ValueError('Nonfinite nominal corner')
        r, c = self.runner, self.console
        if r.velocity_pub is not None:
            raise RuntimeError('Corner requires previous publisher released')
        deadline = time.monotonic() + budget_s
        c.cancel_input()
        r.exclusive()
        start = r.pose()
        if abs(math.remainder(nominal.yaw - start.yaw, 2 * math.pi)) > 0.08:
            raise RuntimeError('Corner start heading outside original line guard')
        c.cancel_input()
        r.assert_exclusive()
        r.velocity_pub = r.node.create_publisher(r.twist, '/cmd_vel', 10)
        target, selected, command = None, None, (0.0, 0.0, 0.0)
        pid, previous = HeadingPid(), time.monotonic()
        generation = getattr(getattr(r, 'feedback_pause', None), 'generation', None)
        gate = StoppedPlacementGate(self.policy)
        evidence, scans, stable = [], [], []
        try:
            while time.monotonic() < deadline:
                r.pump()
                c.cancel_input()
                if time.monotonic() >= deadline:
                    break
                pause = getattr(r, 'feedback_pause', None)
                if pause is not None:
                    pause.check()
                    if pause.generation != generation:
                        generation = pause.generation
                        pid.reset()
                        previous = time.monotonic() - 0.05
                        command = (0.0, 0.0, 0.0)
                        stable.clear()
                        gate = StoppedPlacementGate(self.policy)
                frame = self._frame()
                if len(r.node.get_publishers_info_by_topic('/cmd_vel')) > 1:
                    raise RuntimeError('Competing corner velocity publisher')
                if math.hypot(frame.pose.x - start.x, frame.pose.y - start.y) > (
                    math.hypot(nominal.x - start.x, nominal.y - start.y) + self.max_revision_m
                ):
                    raise RuntimeError('Corner exceeded total travel bound')
                now = time.monotonic()
                dt = now - previous
                previous = now
                try:
                    requested, ready, detail, target, selected = self._control(
                        name, frame, start, nominal, target, selected, pid, dt
                    )
                except (CornerUnknownError, WallFitError) as failure:
                    r.send((0.0, 0.0, 0.0))
                    command, stable = (0.0, 0.0, 0.0), []
                    pid.reset()
                    gate = StoppedPlacementGate(self.policy)
                    c.emit('corner_geometry_wait', name=name, reason=str(failure))
                    continue
                command = limited_velocity(command, requested, dt, linear=0.4, angular=0.6)
                if ready:
                    command = (0.0, 0.0, 0.0)
                    pid.reset()
                if c.command_guard is not None:
                    command = c.command_guard(command)
                if name == 'P10':
                    try:
                        detail['slewed_geometry'] = self._front_guard(frame, command)
                    except CornerUnknownError as failure:
                        r.send((0.0, 0.0, 0.0))
                        command, stable = (0.0, 0.0, 0.0), []
                        pid.reset()
                        gate = StoppedPlacementGate(self.policy)
                        c.emit('corner_geometry_wait', name=name, reason=str(failure))
                        continue
                r.send(command)
                c.emit(
                    'corner_control',
                    name=name,
                    scan_stamp_ns=frame.stamp,
                    actual=asdict(frame.pose),
                    command=command,
                    geometry=detail,
                )
                stopped = r.latest[2] < 0.005 and r.latest[3] < 0.015
                if ready and stopped:
                    if name == 'P10':
                        accepted = gate.add(
                            frame.stamp,
                            detail['front_gap_m'],
                            stopped=True,
                            heading_error_rad=detail['heading_error_rad'],
                            cross_error_m=detail['cross_error_m'],
                        )['accepted']
                    else:
                        stable = self._stable(stable, frame)
                        accepted = self._enough(stable)
                    if accepted:
                        evidence.append(detail)
                        break
                else:
                    stable.clear()
                    gate = StoppedPlacementGate(self.policy)
            else:
                raise RuntimeError('Corner placement deadline exhausted')
            if time.monotonic() >= deadline:
                raise RuntimeError('Corner placement deadline exhausted')
        finally:
            c.stop_motion()  # Original one-second zero/release and feedback-pause checks.
        # A moving-phase gate alone never grants an endpoint. Verify again after release.
        stable = []
        gate = StoppedPlacementGate(self.policy)
        while time.monotonic() < deadline:
            r.pump()
            c.cancel_input()
            frame = self._frame()
            pause = getattr(r, 'feedback_pause', None)
            if pause is not None:
                pause.check()
                if pause.generation != generation:
                    generation = pause.generation
                    stable = []
                    gate = StoppedPlacementGate(self.policy)
            if math.hypot(frame.pose.x - nominal.x, frame.pose.y - nominal.y) > self.max_revision_m:
                raise RuntimeError('Stopped corner outside revision bound')
            try:
                _, ready, detail, _, _ = self._control(
                    name, frame, start, nominal, target, selected, HeadingPid(), 0.05
                )
            except (CornerUnknownError, WallFitError):
                stable = []
                gate = StoppedPlacementGate(self.policy)
                continue
            stopped = r.latest[2] < 0.005 and r.latest[3] < 0.015
            if not ready or not stopped:
                raise RuntimeError('Stopped corner target not verified')
            stable = self._stable(stable, frame)
            if name == 'P10':
                accepted = gate.add(
                    frame.stamp,
                    detail['front_gap_m'],
                    stopped=True,
                    heading_error_rad=detail['heading_error_rad'],
                    cross_error_m=detail['cross_error_m'],
                )['accepted']
            else:
                accepted = self._enough(stable)
            if accepted:
                scans = tuple(
                    dict(
                        scan_stamp_ns=f.stamp, odom_stamp_ns=f.odom_stamp, scan_identity=f.identity
                    )
                    for f in stable
                )
                endpoint = AcceptedEndpoint(
                    name, nominal, frame.pose, scans, tuple(evidence + [detail])
                )
                c.emit('corner_endpoint_accepted', endpoint=asdict(endpoint))
                return endpoint
        raise RuntimeError('Stopped corner verification deadline exhausted')

    def _stable(self, samples, frame):
        if not samples or frame.stamp != samples[-1].stamp:
            samples = (samples + [frame])[-32:]
        return samples

    def _enough(self, samples):
        return len(samples) >= self.policy.stable_scans and (
            samples[-1].stamp - samples[0].stamp >= self.policy.stable_span_s * 1e9
        )

    def _control(self, name, frame, start, nominal, target, selected, pid, dt):
        from task6_corner_geometry import front_command, select_lane

        yaw_error = math.remainder(nominal.yaw - frame.pose.yaw, 2 * math.pi)
        if abs(yaw_error) > 0.08:
            raise RuntimeError('Corner heading outside original line guard')
        if name == 'P10':
            delta = local_delta(Pose(start.x, start.y, nominal.yaw), frame.pose)
            gap = self.runner.motion_walls(['front'], nominal.yaw)['front'].gap
            result = front_command(gap, delta['y_m'], yaw_error, policy=self.policy)
            if result['state'] in ('BLOCKED', 'UNKNOWN'):
                raise RuntimeError('Corner front geometry ' + result['state'])
            vx, vy, wz = result['command']
            c, s = math.cos(yaw_error), math.sin(yaw_error)
            requested = (c * vx - s * vy, s * vx + c * vy, wz)
            if self.runner.cached_stamp != frame.stamp:
                raise RuntimeError('Front wall source changed during corner control')
            check = self._front_guard(frame, requested)
            detail = dict(
                result, front_gap_m=gap, cross_error_m=delta['y_m'], heading_error_rad=yaw_error
            )
            return (
                requested,
                result['state'] == 'VERIFY_STOP',
                dict(detail, front_geometry=check),
                target,
                selected,
            )
        along = local_delta(Pose(start.x, start.y, nominal.yaw), frame.pose)['x_m']
        if abs(along) > 0.04:
            raise RuntimeError('Corner lane longitudinal drift exceeds 4 cm')
        relative = local_delta(frame.pose, nominal)
        lane = select_lane(
            frame.snapshot.points,
            frame.rays,
            nominal_y=relative['y_m'],
            laser_origin=frame.laser_origin,
            policy=self.policy,
            evidence_id=str(frame.stamp),
        )
        if lane['state'] == 'UNKNOWN':
            raise CornerUnknownError('Corner lane geometry UNKNOWN')
        if lane['state'] not in ('TARGET_SELECTED', 'APPROACH_SELECTED'):
            raise RuntimeError('Corner lane geometry ' + lane['state'])
        desired = lane['target']['y']
        if target is None:
            target = Pose(
                frame.pose.x - math.sin(frame.pose.yaw) * desired,
                frame.pose.y + math.cos(frame.pose.yaw) * desired,
                nominal.yaw,
            )
            selected = lane['state']
            if math.hypot(target.x - nominal.x, target.y - nominal.y) > self.max_revision_m:
                raise RuntimeError('Selected lane outside nominal revision bound')
        error = local_delta(frame.pose, target)
        distance = math.hypot(error['x_m'], error['y_m'])
        arrived = distance <= 0.005 and abs(yaw_error) <= 0.01
        if arrived and selected == 'APPROACH_SELECTED':
            # This bounded step is not acceptance; zero, then select from a later frame.
            return (0.0, 0.0, 0.0), False, lane, None, None
        ready = arrived and lane['state'] == 'TARGET_SELECTED' and abs(desired) <= 0.005
        vx, vy = 1.0 * error['x_m'], 1.0 * error['y_m']
        scale = min(1.0, self.policy.final_speed_cap_mps / max(math.hypot(vx, vy), 1e-12))
        wz = pid.update(
            yaw_error, self.runner.body_velocity[2], dt, min(self.console.turn_speed, 0.08)
        )
        return (vx * scale, vy * scale, wz), ready, lane, target, selected

    def _front_guard(self, frame, command):
        """Check the exact candidate after slew as well as measured motion."""
        from task6_corner_geometry import check_front_motion

        if not self.front_sweep_required:
            return dict(
                state='NOT_ENABLED',
                mode='front_wall_feedback_only',
                note='Raw obstacle guard remains observation-only; original feedback protections apply',
            )
        check = check_front_motion(
            frame.snapshot.points,
            frame.rays,
            command=command,
            measured=tuple(self.runner.body_velocity),
            laser_origin=frame.laser_origin,
            policy=self.policy,
            evidence_id=str(frame.stamp),
            horizon_s=0.5,
        )
        if check['state'] == 'UNKNOWN':
            raise CornerUnknownError('Corner front directional geometry UNKNOWN')
        if check['state'] != 'SUPPORTED':
            raise RuntimeError('Corner front directional geometry ' + check['state'])
        return check
