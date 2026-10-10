"""Supervised wall-guided motion using stamped TF scans and same-epoch odometry.

Translations publish directly at low speed; turns delegate to the installed C++
turn controller. These publishers never overlap. Missing required feedback stops
an action and preserves its incomplete record. Geometry is the local model.
"""

from dataclasses import asdict
import math
import time

from action_plan import Pose, errors
from action_runtime import PlannedRunner
from wall_geometry import DIRECTIONS, fit_wall, scan_points, swept_clearance
from wall_policy import Policy, command


class WallRunner(PlannedRunner):
    """Own one wall-action lifetime and carry a verified scalar body-gap reference."""

    def __init__(self, max_speed=0.06):
        super().__init__()
        self.prepare_options = ['-p', 'alignment_wall:=left']
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
            return {side: fit_wall(points, counts, side) for side in sides}
        except ValueError as error:
            raise RuntimeError(f'WALL_LOST: {error}') from error

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
                raise RuntimeError('TURN_CLEARANCE: observed body sweep below 2 cm')

    def capture_reference(self, side):
        """Require five distinct stable stopped scans; write carried body clearance only on success."""
        deadline = time.monotonic() + 4
        values = []
        stamp = None
        while time.monotonic() < deadline:
            self.pump()
            self.moving_pose()
            if self.latest[2] > 0.01 or self.latest[3] > 0.02:
                values = []
                continue
            wall = self.walls([side])[side]
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
                    f'REFERENCE {side}: body_clearance={value:.4f} m (5 stable scans)', flush=True
                )
                return value
        raise RuntimeError('Reference wall did not stabilize')

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

    def translation(self, step, target, policy):
        """Closed-loop distance/wall arrival with heading hold, travel bound and stable endpoint."""
        start = self.pose()
        print(
            f'WALL_ACTION {step.name}: follow={policy.follow} reference={self.reference:.4f} m; '
            f'stop={policy.stop} target_clearance={self.reference + policy.offset:.4f} m; '
            f'nominal_distance={step.value:.3f} m max_travel={policy.max_travel:.3f} m',
            flush=True,
        )
        heading = target.yaw
        c, s = math.cos(heading), math.sin(heading)
        sides = set()
        if policy.follow != 'none':
            sides.add(policy.follow)
        if policy.stop != 'distance':
            sides.add(policy.stop)
        movement = {'forward': 'front', 'backward': 'rear', 'left': 'left', 'right': 'right'}[
            step.kind
        ]
        self.walls(sides)
        self.velocity_pub = self.node.create_publisher(self.twist, '/cmd_vel', 10)
        deadline = time.monotonic() + policy.max_travel / 0.03 * 3 + 20
        previous = start
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
            walls = self.walls(sides)
            gaps = {name: w.gap for name, w in walls.items()}
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
                    f'gaps={gaps} residual={residual}',
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
            self.capture_reference(policy.capture)
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
