"""Optional scan/odom adapter for Console's command guard; never enabled implicitly."""

from dataclasses import asdict
import math
import time

from route_laser_guard import GuardDecision, GuardPolicy, ScanSnapshot, ZERO, evaluate


class LaserGuardError(RuntimeError):
    """Unknown feedback or a blocked directional prediction requires route termination."""


def current_points(points, scan_pose, current):
    """Rigidly transport every scan-time base point into current base axes."""
    rotation = scan_pose.yaw - current.yaw
    c, s = math.cos(rotation), math.sin(rotation)
    dx, dy = scan_pose.x - current.x, scan_pose.y - current.y
    cx, cy = math.cos(current.yaw), math.sin(current.yaw)
    tx, ty = cx * dx + cy * dy, -cy * dx + cx * dy
    return tuple((tx + c * x - s * y, ty + s * x + c * y) for x, y, _ in points)


def scheduled_rays(scan, transform, rotation):
    """Retain invalid-ray identity; geometry() has already validated mounting and scan bounds."""
    q = transform.rotation
    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    x, y, z, w = (v / norm for v in (q.x, q.y, q.z, q.w))
    rays = []
    for i, distance in enumerate(scan.ranges):
        angle = scan.angle_min + i * scan.angle_increment
        c, s = math.cos(angle), math.sin(angle)
        dx = (1 - 2 * (y * y + z * z)) * c + 2 * (x * y - z * w) * s
        dy = 2 * (x * y + z * w) * c + (1 - 2 * (x * x + z * z)) * s
        rays.append(
            (
                math.atan2(dy, dx) + rotation,
                math.isfinite(distance) and scan.range_min <= distance <= scan.range_max,
            )
        )
    return tuple(rays)


def scan_transform(runner, scan):
    from rclpy.time import Time

    return runner.observer.buffer.lookup_transform(
        'base_link', scan.header.frame_id, Time.from_msg(scan.header.stamp)
    ).transform


class RouteLaserGuard:
    """Callable optional injection: guard(command) returns command or raises to stop.

    WallRunner.geometry supplies existing 0.5 s scan/receipt validity and mounting
    checks; scan_pose supplies bounded 150 ms odom pairing. No wall fit is needed.
    """

    enforced = True

    def __init__(self, runner, emit=None, policy=GuardPolicy()):
        policy.validate()
        self.runner, self.emit, self.policy = runner, emit, policy
        self.last_report, self.last_reported = None, 0.0

    def snapshot(self):
        runner = self.runner
        current = runner.moving_pose()
        if runner.frame != 'odom' or runner.twist_frame != 'base_link':
            raise RuntimeError('Guard requires odom frame and base_link twist')
        points, _counts = runner.geometry()
        at_scan = runner.scan_pose()
        scan = runner.observer.scan
        transform = scan_transform(runner, scan)
        return ScanSnapshot(
            current_points(points, at_scan, current),
            scheduled_rays(scan, transform, at_scan.yaw - current.yaw),
        )

    def report(self, decision, candidate, snapshot=None):
        key = decision.state, decision.reason, decision.scale
        now = time.monotonic()
        stopped = decision.state in ('unknown', 'blocked')
        if self.emit and (
            (stopped and self.enforced) or key != self.last_report or now - self.last_reported >= 1
        ):
            detail = dict(
                mode='enforced' if self.enforced else 'shadow',
                enforced=self.enforced,
                decision=asdict(decision),
                proposed_command=candidate,
                applied_command=decision.command if self.enforced else candidate,
                measured_velocity=self.runner.body_velocity,
                scan_stamp_ns=self.runner.cached_stamp,
                odom_stamp_ns=self.runner.stamp,
                policy=asdict(self.policy),
            )
            if stopped and snapshot is not None:
                detail['points_base_xy'] = snapshot.points
                detail['rays_bearing_valid'] = snapshot.rays
            self.emit('laser_guard', **detail)
            self.last_report, self.last_reported = key, now

    def assess(self, candidate):
        """Treat unavailable evaluation input as unknown; preserve cancellation exceptions."""
        try:
            snapshot = self.snapshot()
        except Exception as error:
            return GuardDecision(ZERO, 'unknown', str(error)), None
        decision = evaluate(snapshot, candidate, self.runner.body_velocity, self.policy)
        return decision, snapshot

    def __call__(self, candidate):
        if candidate == ZERO:
            return ZERO
        decision, snapshot = self.assess(candidate)
        self.report(decision, candidate, snapshot)
        if not self.enforced:
            return candidate
        if decision.state in ('blocked', 'unknown'):
            raise LaserGuardError('LASER_GUARD_' + decision.state.upper() + ': ' + decision.reason)
        return decision.command


class ShadowRouteLaserGuard(RouteLaserGuard):
    """Diagnostic evaluation only: never apply a laser stop or scale to the command.

    Existing Console/odom guards still apply. This mode cannot count as obstacle
    protection acceptance; audit records both hypothetical and applied commands.
    """

    enforced = False

    def __init__(self, runner, emit=None, policy=GuardPolicy()):
        super().__init__(runner, emit, policy)
        print('SHADOW ONLY: laser decisions are logged; commands are unchanged.', flush=True)
