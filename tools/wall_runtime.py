"""Supervised wall-guided motion using stamped TF scans and same-epoch odometry.

Translations publish directly at low speed; turns delegate to the installed C++
turn controller. These publishers never overlap. Missing required feedback stops
an action and preserves its incomplete record. Geometry is the local model.
"""

from collections import deque
from dataclasses import asdict
import json
import math
import time

from action_plan import Pose, errors
from action_runtime import PlannedRunner
from wall_geometry import (
    DIRECTIONS,
    transported_gaps,
    pose_at_scan,
    bounded_follow_velocity,
    fit_wall,
    scan_points,
    swept_clearance,
    side_distance,
    front_distance,
)
from wall_policy import Policy, command, FrontStopVerification, FollowAlignment


class WallFitError(RuntimeError):
    """A fresh scan has no acceptable wall fit; distinct from missing/stale feedback or TF."""

    def __init__(self, message, detail=None):
        super().__init__(message)
        self.detail = detail


class ReturnClearanceError(RuntimeError):
    """Recoverable return obstacle; feedback and turn failures remain immediate failures."""


class WallRunner(PlannedRunner):
    """Own one wall-action lifetime and carry a verified scalar body-gap reference."""

    def __init__(self, max_speed=0.06, alignment_wall='right'):
        if alignment_wall not in ('left', 'right'):
            raise ValueError('Alignment wall must be left or right')
        self.odom_history = deque(maxlen=200)
        self.scan_pair = None
        super().__init__()
        self.prepare_options = [
            '-p',
            f'alignment_wall:={alignment_wall}',
            '-p',
            'robust_wall_heading:=true',
        ]
        self.max_speed = max_speed
        self.reference = None
        self.velocity_pub = None
        self.turn_guard = False
        self.last_report = {}
        self.cached_stamp = None
        self.cached_geometry = None
        self.guarded_command = None
        self.guard_command_sub = self.node.create_subscription(
            self.twist, '/cmd_vel', self.receive_guard_command, 10
        )

    def receive_pose(self, msg):
        """Retain validated timestamped poses for matching scans, not just newest odom."""
        super().receive_pose(msg)
        if not self.invalid and self.latest is not None:
            if self.odom_history and self.odom_history[-1][0] == self.stamp:
                self.odom_history.pop()
            self.odom_history.append((self.stamp, self.latest[0]))

    def scan_pose(self):
        """Freeze one bounded odom pairing per scan; recover while stopped if missing."""
        self.moving_pose()
        pair = getattr(self, 'scan_pair', None)
        if pair is None or pair[0] != self.cached_stamp:
            history = getattr(self, 'odom_history', [(self.stamp, self.latest[0])])
            try:
                pose, offset = pose_at_scan(history, self.cached_stamp)
            except ValueError as error:
                raise WallFitError(
                    f'WALL_REFERENCE: {error}',
                    {
                        'side': 'time_pairing',
                        'reason': str(error),
                        'scan_stamp_ns': self.cached_stamp,
                        'odom_stamp_ns': self.stamp,
                        'pose': asdict(self.latest[0]),
                    },
                ) from error
            self.scan_pair = (self.cached_stamp, pose, offset)
        return self.scan_pair[1]

    def observe(self, phase):
        """Record raw ray ranges and separately labelled model-based fitted body gaps."""
        super().observe(phase)
        result = {}
        for side in DIRECTIONS:
            try:
                result[side] = asdict(self.walls([side])[side])
            except RuntimeError as error:
                result[side] = {'unavailable': str(error)}
        self.observations[-1]['body_wall_clearance'] = result
        self.observations[-1]['body_model'] = 'chassis +/-.170,.135; wheels +/-.135,.160 m'
        print(f'BODY_CLEARANCE {phase}: {result}', flush=True)

    def geometry(self):
        """Read fresh scan and stamped mounting TF; cache geometry only for this exact stamp."""
        from rclpy.time import Time

        scan = self.observer.scan
        if scan is None:
            raise RuntimeError('Required laser scan unavailable')
        stamp = scan.header.stamp.sec * 1_000_000_000 + scan.header.stamp.nanosec
        age = (self.node.get_clock().now().nanoseconds - stamp) / 1e9
        if not -0.1 <= age <= 0.5 or time.monotonic() - self.observer.received > 0.5:
            raise RuntimeError('Required laser scan stale')
        if stamp != self.cached_stamp:
            try:
                tf = self.observer.buffer.lookup_transform(
                    'base_link', scan.header.frame_id, Time.from_msg(scan.header.stamp)
                )
                self.cached_geometry = scan_points(scan, tf.transform)
            except Exception as error:
                raise RuntimeError(f'Required laser geometry/TF unavailable: {error}') from error
            self.cached_stamp = stamp
        return self.cached_geometry

    def walls(self, sides):
        points, counts = self.geometry()
        try:
            return {
                side: front_distance(points, counts)
                if side == 'front'
                else fit_wall(points, counts, side)
                for side in sides
            }
        except ValueError as error:
            raise WallFitError(f'WALL_LOST: {error}') from error

    def motion_walls(self, sides, heading):
        """Fit each scan/heading/side set once, including failures; freshness is never cached."""
        self.geometry()
        self.scan_pose()
        key = (self.cached_stamp, heading, tuple(sorted(sides)))
        cached = getattr(self, 'motion_fit_cache', None)
        if cached is not None and cached[0] == key:
            if isinstance(cached[1], WallFitError):
                raise cached[1]
            return dict(cached[1])
        try:
            result = self.fit_motion_walls(sides, heading)
        except WallFitError as error:
            self.motion_fit_cache = (key, error)
            raise
        self.motion_fit_cache = (key, result)
        return dict(result)

    def fit_motion_walls(self, sides, heading):
        """Use held-heading side distances; front arrival uses its independent +/-12 degree TLS window."""
        points, counts = self.geometry()
        error = math.atan2(
            math.sin(heading - self.scan_pose().yaw), math.cos(heading - self.scan_pose().yaw)
        )
        walls = {}
        for side in sorted(sides):
            try:
                walls[side] = (
                    side_distance(points, counts, side, error)
                    if side in ('left', 'right')
                    else front_distance(points, counts)
                )
            except ValueError as failure:
                detail = {
                    'side': side,
                    'reason': str(failure),
                    'scan_stamp_ns': self.cached_stamp,
                    'odom_stamp_ns': self.stamp,
                    'pose': asdict(self.latest[0]),
                    'heading_reference_rad': heading,
                    'heading_error_rad': error,
                    'measured_speed_m_s': self.latest[2],
                    'measured_abs_wz_rad_s': self.latest[3],
                    'scheduled_ray_counts': counts,
                    'points_base_xy_bearing': points,
                    'estimator': 'held_heading_30deg' if side in ('left', 'right') else 'tls_12deg',
                }
                raise WallFitError(f'WALL_LOST {side}: {failure}', detail) from failure
        return walls

    def record_wall_failure(self, failure, stage, heading=None):
        """Log after zero command; keep replayable geometry in the audit, concise text on screen."""
        detail = failure.detail or {
            'side': 'distance_continuity',
            'heading_reference_rad': heading,
            'reason': str(failure),
            'scan_stamp_ns': self.cached_stamp,
            'pose': asdict(self.latest[0]),
            'points_base_xy_bearing': self.cached_geometry[0],
            'scheduled_ray_counts': self.cached_geometry[1],
        }
        detail = {**detail, 'stage': stage}
        key = (stage, detail['scan_stamp_ns'], detail['side'])
        if key == getattr(self, 'last_wall_failure_key', None):
            return
        self.last_wall_failure_key = key
        self.observations.append({'wall_fit_failure': detail})
        summary = {k: v for k, v in detail.items() if k != 'points_base_xy_bearing'}
        print('WALL_FIT_DIAGNOSTIC ' + json.dumps(summary), flush=True)

    def expected_gaps(self, prior):
        """Transport the last trusted wall instead of treating braking travel as a jump."""
        snapshot = getattr(self, 'wall_snapshot', None)
        if not prior or snapshot is None:
            return prior
        walls, pose = snapshot
        return transported_gaps(walls, pose, self.scan_pose())

    def remember_walls(self, walls):
        """Pair a trusted scan with fresh odometry; reject excessive timestamp skew."""
        paired_pose = self.scan_pose()
        if getattr(self, 'wall_snapshot_stamp', None) != self.cached_stamp:
            self.wall_snapshot = (walls, paired_pose)
            self.wall_snapshot_stamp = self.cached_stamp

    def record_recovery_check(self, gaps, prior, samples):
        """Expose non-fit reasons for waiting without changing the recovery decision."""
        detail = {
            'scan_stamp_ns': self.cached_stamp,
            'gaps': gaps,
            'prior_gaps': prior,
            'expected_gaps': self.expected_gaps(prior),
            'scan_pose': asdict(self.scan_pose()),
            'pair_offset_s': self.scan_pair[2] / 1e9,
            'measured_speed_m_s': self.latest[2],
            'measured_abs_wz_rad_s': self.latest[3],
            'stopped': self.latest[2] < 0.01 and self.latest[3] < 0.02,
            'distance_continuous': all(
                abs(gaps[k] - v) <= 0.03 for k, v in self.expected_gaps(prior).items()
            ),
            'previous_stable_samples': len(samples),
            'used_recovery_seconds': self.recovery_seconds,
        }
        self.observations.append({'wall_recovery_check': detail})
        print('WALL_RECOVERY_CHECK ' + json.dumps(detail), flush=True)

    def recover_walls(self, sides, heading, movement, prior):
        """Hold zero for at most two seconds; require three distinct stable stopped scans.

        Feedback/TF/obstacle failures propagate immediately. Braking motion is compensated from the last trusted fitted wall/pose. A previous wall cannot
        be silently replaced by a surface over 3 cm away during this stationary retry.
        """
        started = time.monotonic()
        stamp, samples = None, []
        print('WALL_RECOVERING: zero commands; 2 s budget, three stopped scans', flush=True)
        while time.monotonic() - started < min(2.0, 4.0 - self.recovery_seconds):
            self.send((0.0, 0.0, 0.0))
            self.pump()
            self.moving_pose()
            self.guard_translation((0.0, 0.0, 0.0), movement)
            try:
                walls = self.motion_walls(sides, heading)
            except WallFitError as failure:
                self.record_wall_failure(failure, 'recovery')
                samples = []
                stamp = self.cached_stamp
                continue
            if self.cached_stamp == stamp:
                continue
            stamp = self.cached_stamp
            gaps = {side: wall.gap for side, wall in walls.items()}
            self.record_recovery_check(gaps, prior, samples)
            if (
                self.latest[2] >= 0.01
                or self.latest[3] >= 0.02
                or any(abs(gaps[k] - v) > 0.03 for k, v in self.expected_gaps(prior).items())
            ):
                samples = []
                continue
            samples = (samples + [gaps])[-3:]
            if len(samples) == 3 and all(
                max(row[k] for row in samples) - min(row[k] for row in samples) < 0.015
                for k in gaps
            ):
                self.recovery_seconds += time.monotonic() - started
                if self.recovery_seconds > 4.0:
                    raise RuntimeError('WALL_RECOVERY_BUDGET: 4 s cumulative limit')
                self.remember_walls(walls)
                print('WALL_RECOVERED: fresh stable wall distances; target retained', flush=True)
                return
        if self.recovery_seconds + time.monotonic() - started >= 4.0:
            raise RuntimeError('WALL_RECOVERY_BUDGET: 4 s cumulative limit')
        raise RuntimeError('WALL_RECOVERY_TIMEOUT: wall did not stabilize within 2 s')

    def moving_pose(self):
        """Read fresh odometry without the stopped-speed qualification used at endpoints."""
        if self.invalid or self.latest is None:
            raise RuntimeError(self.invalid or 'Odom unavailable')
        age = (self.node.get_clock().now().nanoseconds - self.stamp) / 1e9
        if not -0.1 <= age <= 0.5 or time.monotonic() - self.latest[1] > 0.5:
            raise RuntimeError('Odom stale during wall action')
        return self.latest[0]

    def receive_guard_command(self, msg):
        """Observe the delegated controller command without creating another publisher."""
        velocity = (msg.linear.x, msg.linear.y, msg.angular.z)
        if all(math.isfinite(v) for v in velocity):
            self.guarded_command = (velocity, time.monotonic())

    def return_scenarios(self):
        """Protect measured body motion and the latest command, including acceleration onset."""
        if getattr(self, 'twist_frame', None) != 'base_link':
            raise RuntimeError('Return guard requires odom twist in base_link')
        scenarios = [self.body_velocity]
        if self.guarded_command is not None:
            velocity, receipt = self.guarded_command
            if time.monotonic() - receipt <= 0.5:
                scenarios.append(velocity)
        return scenarios

    def refresh_callbacks(self):
        """Process ready callbacks without waiting, bounded by 32 calls and 5 ms.

        One spin_once may consume a scan/command callback while odom is queued.
        Refresh before matching sensors; never extend pairing/freshness limits.
        """
        deadline = time.monotonic() + 0.005
        for _ in range(32):
            if self.cancel_requested:
                raise KeyboardInterrupt
            if self.invalid:
                raise RuntimeError(
                    self.invalid + '; stop and establish a new session after inspection'
                )
            if time.monotonic() >= deadline:
                break
            self.ros.spin_once(self.node, timeout_sec=0.0)
        if self.cancel_requested:
            raise KeyboardInterrupt
        if self.invalid:
            raise RuntimeError(self.invalid + '; stop and establish a new session after inspection')

    def pump(self):
        super().pump()
        self.refresh_callbacks()
        if self.turn_guard:
            self.moving_pose()
            pts, _ = self.geometry()
            scenarios = (
                self.return_scenarios()
                if self.turn_sign == 0
                else [(0.0, 0.0, self.turn_sign * 0.20)]
            )
            gap = min(swept_clearance(pts, *velocity) for velocity in scenarios)
            if gap < 0.02:
                self.reject_sweep(pts, gap, scenarios)

    def reject_sweep(self, points, protected_gap, scenarios):
        """Record the limiting directional prediction, retaining all raw obstacle returns."""
        returning = self.turn_sign == 0
        velocity = min(scenarios, key=lambda v: swept_clearance(points, *v))
        limiting = min(points, key=lambda p: swept_clearance([p], *velocity))
        nearest = min(points, key=lambda p: swept_clearance([p]))
        speed, measured_wz = self.latest[2], self.latest[3]
        detail = {
            'mode': 'return' if returning else 'turn',
            'prediction': 'directional_sweep',
            'velocity_scenarios_vx_vy_wz': scenarios,
            'limiting_velocity_vx_vy_wz': velocity,
            'prediction_duration_s': 0.5,
            'scan_stamp_ns': self.cached_stamp,
            'pose': asdict(self.latest[0]),
            'measured_speed_m_s': speed,
            'measured_abs_wz_rad_s': measured_wz,
            'static_gap_m': swept_clearance(points),
            'sweep_gap_m': swept_clearance(points, *velocity),
            'linear_allowance_m': 0.0,
            'angular_allowance_m': 0.0,
            'protected_gap_m': protected_gap,
            'threshold_m': 0.02,
            'nearest_base_point': nearest,
            'limiting_base_point': limiting,
            'limiting_bearing_deg': math.degrees(limiting[2]),
            'points_base_xy_bearing': points,
        }
        self.observations.append({'guard_failure': detail})
        summary = {k: v for k, v in detail.items() if k != 'points_base_xy_bearing'}
        print('CLEARANCE_DIAGNOSTIC ' + json.dumps(summary), flush=True)
        code = 'RETURN_CLEARANCE' if returning else 'TURN_CLEARANCE'
        error = ReturnClearanceError if returning else RuntimeError
        raise error(f'{code}: protected clearance below 2 cm; see diagnostic')

    def capture_reference(self, side, heading=None):
        """Capture side offset at a fixed heading from five distinct stable stopped scans.

        This reuses the motion estimator; it never requires a second free-angle
        fit after preparation. Commit the carried gap only after all checks pass.
        """
        heading = self.pose().yaw if heading is None else heading
        deadline = time.monotonic() + 4
        values = []
        stamp = None
        last_rejection = None
        while time.monotonic() < deadline:
            self.pump()
            self.moving_pose()
            if self.latest[2] > 0.01 or self.latest[3] > 0.02:
                values = []
                continue
            try:
                wall = self.motion_walls([side], heading)[side]
            except WallFitError as error:
                # Only stopped reference acquisition retries fit failures, within the original budget.
                # Do not swallow stale scan, invalid odom, TF failure or cancellation.
                values = []
                stamp = self.cached_stamp
                last_rejection = str(error)
                continue
            if self.cached_stamp == stamp:
                continue
            stamp = self.cached_stamp
            values = (values + [wall.gap])[-5:]
            if len(values) == 5 and max(values) - min(values) < 0.015:
                value = sum(values) / 5
                if not 0.05 <= value <= 0.30:
                    raise RuntimeError(
                        'Measured reference gap outside [5,30] cm; inspect placement'
                    )
                self.reference = value
                print(
                    f'REFERENCE {side}: body_clearance={value:.4f} m (5 stable scans; held_heading_30deg)',
                    flush=True,
                )
                return value
        raise RuntimeError(
            f'Reference wall did not stabilize within 4 s; last_fit_error={last_rejection}'
        )

    def send(self, velocity):
        msg = self.twist()
        msg.linear.x, msg.linear.y, msg.angular.z = velocity
        self.velocity_pub.publish(msg)

    def stop_translation(self):
        """Keep zero commands for one second, then release the owned publisher even on failure."""
        if self.velocity_pub is None:
            return
        try:
            deadline = time.monotonic() + 1
            while time.monotonic() < deadline:
                self.send((0.0, 0.0, 0.0))
                self.ros.spin_once(self.node, timeout_sec=0.05)
        finally:
            self.node.destroy_publisher(self.velocity_pub)
            self.velocity_pub = None
            self.cancel_requested = False

    @staticmethod
    def translation_coverage(points, counts, velocity, movement):
        """Check original requested directions even if correction will later be reduced."""
        required = {movement}
        if abs(velocity[0]) > 0.001:
            required.add('front' if velocity[0] > 0 else 'rear')
        if abs(velocity[1]) > 0.001:
            required.add('left' if velocity[1] > 0 else 'right')
        for direction in required:
            center = DIRECTIONS[direction]
            rays = [
                p
                for p in points
                if abs(math.atan2(math.sin(p[2] - center), math.cos(p[2] - center)))
                < math.radians(20)
            ]
            if len(rays) < 8 or len(rays) < 0.5 * counts[direction]:
                raise RuntimeError(f'Travel-direction scan unavailable: {direction}')

    def guard_translation(self, velocity, movement):
        """Check coverage in commanded directions, then test predicted footprint clearance."""
        points, counts = self.geometry()
        self.translation_coverage(points, counts, velocity, movement)
        gap = swept_clearance(points, *velocity)
        if gap < 0.02:
            self.reject_translation(points, velocity, movement, gap)

    def reject_translation(self, points, velocity, movement, gap, source='command'):
        """Record the exact translation/recovery guard input without weakening rejection.

        A zero command identifies the stationary check used during wall recovery
        or settling. Measured speeds remain separate from the commanded sweep.
        """
        nearest = min(points, key=lambda p: swept_clearance([p]))
        limiting = min(points, key=lambda p: swept_clearance([p], *velocity))
        detail = {
            'mode': 'translation',
            'velocity_source': source,
            'movement': movement,
            'zero_command': all(v == 0.0 for v in velocity),
            'command_vx_vy_wz': velocity,
            'prediction_duration_s': 0.5,
            'scan_stamp_ns': self.cached_stamp,
            'pose': asdict(self.latest[0]),
            'measured_speed_m_s': self.latest[2],
            'measured_abs_wz_rad_s': self.latest[3],
            'static_gap_m': swept_clearance(points),
            'protected_gap_m': gap,
            'threshold_m': 0.02,
            'nearest_base_point': nearest,
            'limiting_base_point': limiting,
            'limiting_bearing_deg': math.degrees(limiting[2]),
            'points_base_xy_bearing': points,
        }
        self.observations.append({'guard_failure': detail})
        summary = {k: v for k, v in detail.items() if k != 'points_base_xy_bearing'}
        print('CLEARANCE_DIAGNOSTIC ' + json.dumps(summary), flush=True)
        raise RuntimeError('OBSTACLE: predicted body clearance below 2 cm; see diagnostic')

    def checked_motion_gaps(self, sides, heading, movement, prior):
        """Recover only fit loss/jumps; critical feedback errors leave through the stop handler."""
        try:
            walls = self.motion_walls(sides, heading)
            gaps = {name: wall.gap for name, wall in walls.items()}
            if any(abs(gaps[k] - v) > 0.03 for k, v in self.expected_gaps(prior).items()):
                raise WallFitError(
                    f'Wall distance jumped by more than 3 cm: prior={prior}, current={gaps}'
                )
            self.remember_walls(walls)
            return gaps
        except WallFitError as failure:
            self.send((0.0, 0.0, 0.0))
            self.record_wall_failure(failure, 'motion', heading)
            self.recover_walls(sides, heading, movement, prior)
            return None

    def follow_velocity(self, planned, rotation, movement):
        """Constrain side correction without masking measured motion or the final guard."""
        points, counts = self.geometry()
        vx, vy, wz = planned
        c, s = math.cos(rotation), math.sin(rotation)
        requested = (c * vx - s * vy, s * vx + c * vy, wz)
        self.translation_coverage(points, counts, requested, movement)
        velocity, scale = bounded_follow_velocity(points, planned, rotation)
        if self.twist_frame != 'base_link':
            raise RuntimeError('Follow guard requires odom twist in base_link')
        measured_gap = swept_clearance(points, *self.body_velocity)
        if measured_gap < 0.02:
            self.reject_translation(
                points, self.body_velocity, movement, measured_gap, source='measured'
            )
        if scale < 1.0 and time.monotonic() - getattr(self, 'last_side_limit_log', 0) >= 1.0:
            self.last_side_limit_log = time.monotonic()
            detail = {
                'scan_stamp_ns': self.cached_stamp,
                'planned_velocity': planned,
                'rotation_rad': rotation,
                'lateral_scale': scale,
                'body_velocity': velocity,
                'protected_gap_m': swept_clearance(points, *velocity),
            }
            self.observations.append({'side_correction_limited': detail})
            print('SIDE_CORRECTION_LIMITED ' + json.dumps(detail), flush=True)
        return velocity

    def translation_velocity(self, planned, rotation, policy, movement):
        """Rotate planned axes, constraining lateral motion only for wall-following actions."""
        if policy.follow != 'none':
            return self.follow_velocity(planned, rotation, movement)
        vx, vy, wz = planned
        return (
            math.cos(rotation) * vx - math.sin(rotation) * vy,
            math.sin(rotation) * vx + math.cos(rotation) * vy,
            wz,
        )

    def endpoint_stages(self, policy, residual, velocity, ready, path, front, follow):
        """Apply side realignment before approach, then stopped front verification."""
        stopped = self.latest[2] < 0.01 and self.latest[3] < 0.02
        adjusted = follow.apply(
            policy, residual, self.cached_stamp, stopped, time.monotonic(), path
        )
        if adjusted is not None:
            front.reset()
            return adjusted, False
        return front.apply(policy, residual, self.cached_stamp, stopped, velocity, ready)

    def translation(self, step, target, policy):  # noqa: PLR0915 - one ordered safety/control loop
        """Closed-loop distance/wall arrival with heading hold, travel bound and stable endpoint."""
        observed_start = self.pose()
        start = getattr(self, 'resume_origin', None) or observed_start
        follow_goal, stop_goal = policy.clearances(self.reference)
        print(
            f'WALL_ACTION {step.name}: follow={policy.follow} reference={self.reference:.4f} m follow_target={follow_goal}; '
            f'stop={policy.stop} target_clearance={stop_goal} m; '
            f'nominal_distance={step.value:.3f} m max_travel={policy.max_travel:.3f} m',
            flush=True,
        )
        heading = target.yaw
        c, s = math.cos(heading), math.sin(heading)
        sides = {side for side in (policy.follow, policy.stop) if side in DIRECTIONS}
        movement = {'forward': 'front', 'backward': 'rear', 'left': 'left', 'right': 'right'}[
            step.kind
        ]
        self.velocity_pub = self.node.create_publisher(self.twist, '/cmd_vel', 10)
        deadline = time.monotonic() + policy.max_travel / 0.03 * 3 + 20
        self.path_last_pose = observed_start
        prior_gaps = {}
        self.wall_snapshot = None
        self.wall_snapshot_stamp = None
        self.recovery_seconds = 0.0
        path = getattr(self, 'resume_path', 0.0)
        hold = None
        front_verify = FrontStopVerification()
        follow_align = FollowAlignment(policy.align_follow_first, time.monotonic(), path)
        last_log = 0.0
        while time.monotonic() < deadline:
            self.pump()
            actual = self.moving_pose()
            path += errors(actual, self.path_last_pose)[0]
            self.path_last_pose = actual
            self.last_report = {'path_m': path}
            if path > policy.max_travel:
                raise RuntimeError(
                    'MAX_TRAVEL: wall target not reached within configured path limit'
                )
            dx, dy = actual.x - start.x, actual.y - start.y
            forward, left = c * dx + s * dy, -s * dx + c * dy
            progress, cross = {
                'forward': (forward, left),
                'backward': (-forward, left),
                'left': (left, forward),
                'right': (-left, forward),
            }[step.kind]
            gaps = self.checked_motion_gaps(sides, heading, movement, prior_gaps)
            if gaps is None:
                front_verify.reset()
                follow_align.invalidate()
                hold = None
                continue
            prior_gaps = gaps
            velocity, ready, residual = command(
                step,
                policy,
                self.reference,
                progress,
                cross,
                errors(actual, target)[1],
                gaps,
                self.max_speed,
            )
            velocity, ready = self.endpoint_stages(
                policy, residual, velocity, ready, path, front_verify, follow_align
            )
            # Convert the nominal heading axes to the current body axes.
            rotation = heading - actual.yaw
            velocity = self.translation_velocity(velocity, rotation, policy, movement)
            self.guard_translation(velocity, movement)
            self.send((0.0, 0.0, 0.0) if ready else velocity)
            if ready and self.latest[2] < 0.01 and self.latest[3] < 0.02:
                hold = hold or time.monotonic()
                if time.monotonic() - hold >= 0.5:
                    self.last_report = {
                        'completion': policy.stop,
                        'reference_m': self.reference,
                        'follow_target_m': follow_goal,
                        'stop_target_m': stop_goal,
                        'residual': residual,
                        'wall_clearances_m': gaps,
                        'path_m': path,
                        'forward_m': forward,
                        'left_m': left,
                        'policy': asdict(policy),
                    }
                    return Pose(actual.x, actual.y, heading)
            else:
                hold = None
            if time.monotonic() - last_log >= 1:
                print(
                    f'WALL_PROGRESS {step.name} progress={progress:.3f} path={path:.3f} '
                    f'gaps={gaps} residual={residual} front_verified={front_verify.confirmed} follow_aligned={follow_align.done} side_estimator=held_heading_30deg front_estimator=tls_12deg',
                    flush=True,
                )
                last_log = time.monotonic()
        raise RuntimeError('WALL_ACTION_TIMEOUT')

    def check_turn(self, target):
        """Check the complete observed turn sweep without issuing motion commands."""
        actual = self.pose()
        delta = target.yaw - actual.yaw
        points, counts = self.geometry()
        # Require scan coverage in every quadrant for a rotation; missing returns are not free space.
        for side, center in DIRECTIONS.items():
            n = sum(
                abs(math.atan2(math.sin(p[2] - center), math.cos(p[2] - center))) < math.radians(20)
                for p in points
            )
            if n < 8 or n < 0.5 * counts[side]:
                raise RuntimeError(f'Turn scan coverage unavailable: {side}')
        for i in range(1, max(2, int(abs(delta) / 0.04) + 1)):
            angle = delta * i / max(2, int(abs(delta) / 0.04))
            if swept_clearance(points, wz=angle, duration=1) < 0.02:
                raise RuntimeError('Turn would sweep within 2 cm of observed obstacle')
        print(f'TURN_CHECK: full observed sweep passed, delta={delta:.6f} rad', flush=True)

    def turn(self, step, target, policy):
        """Recheck at execution time even if a previous stopped check passed."""
        self.check_turn(target)
        delta = target.yaw - self.pose().yaw
        self.turn_sign = 1 if delta > 0 else -1
        self.turn_guard = True
        try:
            self.rotate_target(target)
        finally:
            self.turn_guard = False
        if policy.capture != 'none':
            self.capture_reference(policy.capture, target.yaw)
        self.last_report = {
            'completion': 'turn',
            'reference_m': self.reference,
            'policy': asdict(policy),
        }
        return None

    def execute(self, step, target):
        """Run exactly one controller; backward recovery disables wall policies but retains scans."""
        self.child = None
        self.path_last_pose = None
        try:
            self.exclusive()
            self.pose()
            self.observe('before')
            policy = Policy(**step.wall) if step.wall else Policy(max_travel=3.0)
            if step.kind == 'turn':
                resolved = self.turn(step, target, policy)
                if step.wall is None and errors(self.pose(), target)[0] > 0.003:
                    # An adjusted historical turn also has a small translation to undo.
                    from turn_adjustment import run_adjustment

                    origin = self.pose()
                    if errors(origin, target)[0] > 0.06:
                        raise RuntimeError('Adjusted turn return exceeds 6 cm')
                    run_adjustment(
                        self,
                        {
                            'origin': asdict(origin),
                            'target': asdict(target),
                            'path_m': 0.0,
                            'done': False,
                        },
                    )
            elif step.wall:
                resolved = self.translation(step, target, policy)
            else:
                # History return goes to the recorded actual start, not a reversed wall threshold.
                resolved = self.return_position(step, target)
            self.stop_translation()
            self.pose()
            self.observe('after')
            return resolved
        except (Exception, KeyboardInterrupt):
            self.stop_translation()
            self.stop_owned()
            raise
        finally:
            self.child = None
            self.cancel_requested = False

    def return_probe(self, target, previous):
        """Test retained motion plus a conservative restart toward the unchanged target."""
        pose = self.moving_pose()
        dx, dy = target.x - pose.x, target.y - pose.y
        length = math.hypot(dx, dy)
        scale = 0.03 / max(length, 0.001)
        c, s = math.cos(pose.yaw), math.sin(pose.yaw)
        vx, vy = scale * (c * dx + s * dy), scale * (-s * dx + c * dy)
        # The child can correct heading before translating. Check both separately and together.
        error = errors(pose, target)[1]
        wz = math.copysign(0.25, error) if abs(error) > 0.01 else 0.0
        return [
            (0.0, 0.0, 0.0),
            *previous,
            *self.return_scenarios(),
            (vx, vy, 0.0),
            (0.0, 0.0, wz),
            (vx, vy, wz),
        ]

    @staticmethod
    def return_coverage(points, counts, required):
        """Missing rays are unknown space, never evidence that an obstacle disappeared."""
        for side in required:
            center = DIRECTIONS[side]
            valid = sum(
                abs(math.remainder(p[2] - center, 2 * math.pi)) < math.radians(20) for p in points
            )
            if valid < 8 or valid < 0.5 * counts.get(side, 0):
                return False
        return True

    def recover_return(self, target, previous):
        """After child shutdown, hold zero for <=2 s and require three new stopped scans.

        All raw points and the 2 cm boundary are retained. Repeated stamps cannot
        count as confirmation; stale feedback, TF failure and cancellation abort.
        Check the intended restart, not merely the zero command seen while stopped.
        """
        started = time.monotonic()
        stamp, odom_stamp, count = self.cached_stamp, self.stamp, 0
        first = None
        self.velocity_pub = self.node.create_publisher(self.twist, '/cmd_vel', 10)
        print('RETURN_RECOVERING: stopped child; 2 s budget, three fresh stopped scans', flush=True)
        try:
            while time.monotonic() - started < 2.0:
                self.send((0.0, 0.0, 0.0))
                self.pump()
                scenarios = self.return_probe(target, previous)
                points, coverage = self.geometry()
                if self.cached_stamp == stamp or self.stamp == odom_stamp:
                    continue
                stamp, odom_stamp = self.cached_stamp, self.stamp
                required = set()
                for vx, vy, _ in scenarios:
                    if abs(vx) > 0.001:
                        required.add('front' if vx > 0 else 'rear')
                    if abs(vy) > 0.001:
                        required.add('left' if vy > 0 else 'right')
                clear = self.return_coverage(points, coverage, required) and all(
                    swept_clearance(points, *velocity) >= 0.02 for velocity in scenarios
                )
                stopped = self.latest[2] < 0.01 and self.latest[3] < 0.02
                if not clear or not stopped:
                    count, first = 0, None
                    continue
                count += 1
                first = time.monotonic() if first is None else first
                if count >= 3 and time.monotonic() - first >= 0.20:
                    self.observations.append(
                        {'return_recovery': {'result': 'clear', 'scans': count}}
                    )
                    print(
                        'RETURN_RECOVERED: clearance confirmed; original target retained',
                        flush=True,
                    )
                    return
            raise RuntimeError('RETURN_RECOVERY_TIMEOUT: clearance not confirmed within 2 s')
        finally:
            self.send((0.0, 0.0, 0.0))
            self.node.destroy_publisher(self.velocity_pub)
            self.velocity_pub = None

    def return_position(self, step, target):
        """Retry the same absolute history target at most twice after bounded stopped checks."""
        self.turn_sign = 0
        for attempt in range(3):
            self.guarded_command = None
            self.turn_guard = True
            try:
                self.translate_target(target)
                self.last_report = {'completion': 'recorded_pose_return', 'recoveries': attempt}
                return None
            except ReturnClearanceError:
                previous = self.return_scenarios()
                self.turn_guard = False
                self.stop_owned()
                self.child = None
                if attempt == 2:
                    raise RuntimeError('RETURN_RECOVERY_BUDGET: two retries exhausted') from None
                self.recover_return(target, previous)
            finally:
                self.turn_guard = False
