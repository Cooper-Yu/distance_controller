"""Supervised wall-guided motion using stamped TF scans and same-epoch odometry.

Translations publish directly at low speed; turns delegate to the installed C++
turn controller. These publishers never overlap. Missing required feedback stops
an action and preserves its incomplete record. Geometry is the local model.
"""

from dataclasses import asdict
import json
import math
import time

from action_plan import Pose, errors
from action_runtime import PlannedRunner
from wall_geometry import (
    DIRECTIONS,
    fit_wall,
    scan_points,
    swept_clearance,
    side_distance,
    front_distance,
)
from wall_policy import Policy, command


class WallFitError(RuntimeError):
    """A fresh scan has no acceptable wall fit; distinct from missing/stale feedback or TF."""


class WallRunner(PlannedRunner):
    """Own one wall-action lifetime and carry a verified scalar body-gap reference."""

    def __init__(self, max_speed=0.06, alignment_wall='right'):
        if alignment_wall not in ('left', 'right'):
            raise ValueError('Alignment wall must be left or right')
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
        """Use held-heading side distances; front arrival uses its independent +/-12 degree TLS window."""
        points, counts = self.geometry()
        error = math.atan2(
            math.sin(heading - self.moving_pose().yaw), math.cos(heading - self.moving_pose().yaw)
        )
        try:
            return {
                side: side_distance(points, counts, side, error)
                if side in ('left', 'right')
                else front_distance(points, counts)
                for side in sides
            }
        except ValueError as error:
            raise WallFitError(f'WALL_LOST: {error}') from error

    def recover_walls(self, sides, heading, movement, prior):
        """Hold zero for at most two seconds; require three distinct stable stopped scans.

        Feedback/TF/obstacle failures propagate immediately. A previous wall cannot
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
            except WallFitError:
                samples = []
                stamp = self.cached_stamp
                continue
            if self.cached_stamp == stamp:
                continue
            stamp = self.cached_stamp
            gaps = {side: wall.gap for side, wall in walls.items()}
            if (
                self.latest[2] >= 0.01
                or self.latest[3] >= 0.02
                or any(abs(gaps[k] - v) > 0.03 for k, v in prior.items())
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

    def pump(self):
        super().pump()
        if self.turn_guard:
            self.moving_pose()
            pts, _ = self.geometry()
            if self.turn_sign == 0:
                speed, wz = self.latest[2], self.latest[3]
                gap = swept_clearance(pts) - speed * 0.5 - wz * 0.22 * 0.5
            else:
                gap = swept_clearance(pts, wz=self.turn_sign * 0.20)
            if gap < 0.02:
                self.reject_sweep(pts, gap)

    def reject_sweep(self, points, protected_gap):
        """Capture the triggering frame before unwinding; never alter the guard decision.

        Return protection subtracts scalar speed allowances from static clearance.
        Turn protection samples a directional sweep. Report both independently so
        a swept residual is not mistaken for a directly measured wall distance.
        """
        returning = self.turn_sign == 0
        wz = 0.0 if returning else self.turn_sign * 0.20
        limiting = min(points, key=lambda p: swept_clearance([p], wz=wz))
        nearest = min(points, key=lambda p: swept_clearance([p]))
        speed, measured_wz = self.latest[2], self.latest[3]
        detail = {
            'mode': 'return' if returning else 'turn',
            'scan_stamp_ns': self.cached_stamp,
            'pose': asdict(self.latest[0]),
            'measured_speed_m_s': speed,
            'measured_abs_wz_rad_s': measured_wz,
            'static_gap_m': swept_clearance(points),
            'sweep_gap_m': swept_clearance(points, wz=wz),
            'linear_allowance_m': speed * 0.5 if returning else 0.0,
            'angular_allowance_m': measured_wz * 0.22 * 0.5 if returning else 0.0,
            'protected_gap_m': protected_gap,
            'threshold_m': 0.02,
            'nearest_base_point': nearest,
            'limiting_base_point': limiting,
            'limiting_bearing_deg': math.degrees(limiting[2]),
            'points_base_xy_bearing': points,
        }
        # Preserve all transformed returns in the existing failure audit, not just
        # a later manual scan. Console output stays compact for cloud diagnosis.
        self.observations.append({'guard_failure': detail})
        summary = {k: v for k, v in detail.items() if k != 'points_base_xy_bearing'}
        print('CLEARANCE_DIAGNOSTIC ' + json.dumps(summary), flush=True)
        code = 'RETURN_CLEARANCE' if returning else 'TURN_CLEARANCE'
        raise RuntimeError(f'{code}: protected clearance below 2 cm; see diagnostic')

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

    def guard_translation(self, velocity, movement):
        """Check coverage in commanded directions, then test predicted footprint clearance."""
        points, counts = self.geometry()
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
        if swept_clearance(points, *velocity) < 0.02:
            raise RuntimeError('OBSTACLE: predicted body clearance below 2 cm')

    def checked_motion_gaps(self, sides, heading, movement, prior):
        """Recover only fit loss/jumps; critical feedback errors leave through the stop handler."""
        try:
            walls = self.motion_walls(sides, heading)
            gaps = {name: wall.gap for name, wall in walls.items()}
            if any(abs(gaps[k] - v) > 0.03 for k, v in prior.items()):
                raise WallFitError('Wall distance jumped by more than 3 cm')
            return gaps
        except WallFitError:
            self.send((0.0, 0.0, 0.0))
            self.recover_walls(sides, heading, movement, prior)
            return None

    def translation(self, step, target, policy):
        """Closed-loop distance/wall arrival with heading hold, travel bound and stable endpoint."""
        start = self.pose()
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
        previous = start
        prior_gaps = {}
        self.recovery_seconds = 0.0
        path = 0.0
        hold = None
        last_log = 0.0
        while time.monotonic() < deadline:
            self.pump()
            actual = self.moving_pose()
            path += math.hypot(actual.x - previous.x, actual.y - previous.y)
            previous = actual
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
            # Convert the nominal heading axes to the current body axes.
            rotation = heading - actual.yaw
            vx, vy, wz = velocity
            velocity = (
                math.cos(rotation) * vx - math.sin(rotation) * vy,
                math.sin(rotation) * vx + math.cos(rotation) * vy,
                wz,
            )
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
                    f'gaps={gaps} residual={residual} side_estimator=held_heading_30deg front_estimator=tls_12deg',
                    flush=True,
                )
                last_log = time.monotonic()
        raise RuntimeError('WALL_ACTION_TIMEOUT')

    def turn(self, step, target, policy):
        """Check observed full turn sweep, then monitor near-term sweep during C++ turn execution."""
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
        try:
            self.exclusive()
            self.pose()
            self.observe('before')
            policy = Policy(**step.wall) if step.wall else Policy(max_travel=3.0)
            if step.kind == 'turn':
                resolved = self.turn(step, target, policy)
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

    def return_position(self, step, target):
        """Return to an actual history pose under short-horizon laser protection."""
        # Retain existing absolute-target controller and guard its motion from fresh scans.
        self.turn_sign = 0
        self.turn_guard = True
        try:
            self.translate_target(target)
        finally:
            self.turn_guard = False
        self.last_report = {'completion': 'recorded_pose_return'}
        return None
