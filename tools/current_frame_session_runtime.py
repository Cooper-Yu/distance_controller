"""Fixed new-epoch reference with independently bounded, explicitly requested steps."""

import math
import re
import uuid

from action_plan import Pose
from current_frame_step_runtime import CurrentEpochGuard
from motion_model import local_delta, target_pose
from route_prefix_return_plan import stopped
from route_session_state import near
from teaching_path import fixed_segment
from wall_runtime import WallFitError, WallRunner


def backward_path(anchor, actual, distance):
    """Keep the original lateral line/yaw; never append previous endpoint error to a step."""
    if not math.isfinite(distance) or not 0 < distance <= 0.05:
        raise ValueError('Backward distance must be positive and <= .05 m')
    values = (*vars(anchor).values(), *vars(actual).values())
    if not all(math.isfinite(value) for value in values):
        raise ValueError('Nonfinite session/step pose')
    local = local_delta(anchor, actual)
    x, y = local['x_m'], local['y_m']
    if abs(y) > 0.01 or abs(local['yaw_deg']) > 1.0:
        raise RuntimeError('Fixed-reference entry requires lateral <=1 cm / heading <=1 deg')
    chord = math.hypot(distance, y)
    if chord > 0.05 + 1e-12:
        raise RuntimeError('Actual start-to-target chord exceeds 5 cm; request a smaller step')
    return fixed_segment((x, 0.0), (x - distance, 0.0), 0.0), chord


def continuous(previous, current, dt, speed):
    """Reject observable pose jumps beyond command travel plus existing start tolerances."""
    if dt <= 0:
        return
    if dt > 0.5 or not near(previous, current, speed * dt + 0.005, 0.10 * dt + 0.01):
        raise RuntimeError('Local session odometry gap/pose jump; reference invalidated')


class SessionRunner(WallRunner):
    """Check actual adjacent filtered callbacks, including inside exclusive()/stop loops."""

    def __init__(self, speed):
        self.session_sample = None
        self.session_speed = speed
        super().__init__(speed)

    def receive_pose(self, message):
        super().receive_pose(message)
        if self.invalid or self.latest is None:
            return
        age = (self.node.get_clock().now().nanoseconds - self.stamp) / 1e9
        if self.stamp <= 0 or not -0.1 <= age <= 0.5:
            self.invalid = 'Local session filtered source timestamp stale/invalid'
            return
        if (self.frame, self.twist_frame) != ('odom', 'base_link'):
            self.invalid = 'Local session filtered frame changed/invalid'
            return
        current = (
            self.stamp,
            self.latest[0],
            tuple(self.body_velocity),
            self.frame,
            self.twist_frame,
            self.latest[1],
        )
        if self.session_sample is not None:
            previous = self.session_sample
            if current[0] == previous[0]:
                if current[1:5] != previous[1:5]:
                    self.invalid = 'Local session same-stamp filtered content changed'
                    return
                self.latest = (self.latest[0], previous[5], *self.latest[2:])
                current = previous
            try:
                continuous(
                    previous[1], current[1], (current[0] - previous[0]) / 1e9, self.session_speed
                )
            except RuntimeError as error:
                self.invalid = str(error)
                return
        self.session_sample = current


class SessionGuard(CurrentEpochGuard):
    """Keep session axes immutable; only reset each step's physical travel budget."""

    def __init__(self, runner, gids, speed, emit):
        self.anchor = self.raw_anchor = None
        self.idle_pose = self.idle_raw = None
        self.active, self.failure = False, None
        self.scan_identity = None
        super().__init__(runner, gids, 0.05, speed, emit, supervised_manual=True)

    def receive_raw(self, message):
        previous = self.raw
        super().receive_raw(message)
        if previous is not None and self.raw_invalid is None:
            try:
                continuous(
                    Pose(**previous['pose']),
                    Pose(**self.raw['pose']),
                    (self.raw['stamp_ns'] - previous['stamp_ns']) / 1e9,
                    self.speed,
                )
            except RuntimeError as error:
                self.raw_invalid = str(error)

    def check_bound(self, actual):
        if self.anchor is None:
            return super().check_bound(actual)
        if not near(actual, Pose(actual.x, actual.y, self.anchor.yaw), 0, 0.035):
            raise RuntimeError('Fixed session heading bound exceeded')
        raw = Pose(**self.raw['pose'])
        if not near(raw, Pose(raw.x, raw.y, self.raw_anchor.yaw), 0, 0.035):
            raise RuntimeError('Fixed raw session heading bound exceeded')
        if self.active:
            if (
                not self.motion_started
                and math.hypot(actual.x - self.target.x, actual.y - self.target.y) > 0.05 + 1e-12
            ):
                raise RuntimeError('Actual first-command start-to-target chord exceeds 5 cm')
            return super().check_bound(actual)
        if (
            not stopped(self.runner.body_velocity)
            or not stopped(self.raw['velocity'])
            or not near(actual, self.idle_pose, 0.005, 0.01)
            or not near(raw, self.idle_raw, 0.005, 0.01)
        ):
            raise RuntimeError('Stopped local session pose/velocity changed')
        if self.runner.node.count_publishers('/cmd_vel') != 0:
            raise RuntimeError('READY requires no cmd_vel publisher')

    def check_scan_identity(self):
        current = self.runner.cached_stamp, self.runner.observer.scan.header.frame_id
        if self.scan_identity is not None:
            stamp, frame = self.scan_identity
            if current[0] < stamp or current[1] != frame:
                raise RuntimeError('Local session scan chronology/frame changed')
        self.scan_identity = current

    def check(self, *, allow_pair_wait=False):
        if self.failure is not None:
            raise RuntimeError(self.failure)
        try:
            actual = super().check()
            self.runner.geometry()
            self.check_scan_identity()
            self.runner.scan_pose()
            return actual
        except WallFitError as error:
            if (
                allow_pair_wait
                and not self.active
                and self.anchor is not None
                and pairing_gap(error)
            ):
                raise ReadyPairingError(str(error), self.runner.cached_stamp) from error
            self.failure = str(error)
            raise
        except Exception as error:
            self.failure = str(error)
            raise

    def bind_session(self, cancellation):
        actual = super().bind_stopped(cancellation)
        self.anchor, self.raw_anchor = actual, Pose(**self.raw['pose'])
        self.idle_pose, self.idle_raw = actual, self.raw_anchor
        self.check()
        return actual

    def begin_step(self, distance):
        if self.active or self.anchor is None:
            raise RuntimeError('Step requires an attached READY session')
        actual = self.check()
        motion, chord = backward_path(self.anchor, actual, distance)
        self.origin, self.raw_origin = actual, Pose(**self.raw['pose'])
        self.distance, self.path_m, self.path_pose = chord, 0.0, actual
        self.target = target_pose(self.anchor, motion)
        self.motion_started, self.active = False, True
        return motion, actual

    def finish_step(self, cancellation):
        if not self.active:
            raise RuntimeError('No active local step')
        actual = super().bind_stopped(cancellation)
        self.idle_pose, self.idle_raw = actual, Pose(**self.raw['pose'])
        self.active = False
        self.check()
        return actual


class StepPermit:
    """A new unpredictable token is issued only after release; old requests never queue."""

    def __init__(self):
        self.index, self.token, self.active = 1, None, False

    def ready(self):
        if self.active:
            raise RuntimeError('Cannot advertise READY during an active step')
        self.token = uuid.uuid4().hex
        return self.token

    def take(self, line):
        if self.active or self.token is None or line != 'step ' + self.token:
            return False
        self.token, self.active = None, True
        return True

    def finish(self):
        if not self.active:
            raise RuntimeError('Cannot finish an unrequested step')
        self.index += 1
        self.active = False


class ReadyPairingError(RuntimeError):
    """Only a fresh, released READY bracket failure may enter bounded waiting."""

    def __init__(self, reason, stamp):
        super().__init__(reason)
        self.stamp = stamp


def pairing_gap(error):
    detail = error.detail
    if not isinstance(detail, dict) or detail.get('side') != 'time_pairing':
        return False
    reason = detail.get('reason', '')
    match = re.fullmatch(r'Odom bracket too wide for scan: (\d+\.\d{3}) s', reason)
    return bool(match and float(match[1]) >= 0.150 and str(error) == 'WALL_REFERENCE: ' + reason)


class ReadyPairingWait:
    """One absolute two-second window; three distinct valid pairs after the last bad scan."""

    def __init__(self):
        self.started = self.deadline = None
        self.bad_stamp = None
        self.paired_stamps = []

    def check_deadline(self, now):
        if self.deadline is not None and now >= self.deadline:
            raise RuntimeError('READY scan pairing wait exceeded 2 seconds')

    def failed(self, stamp, now):
        self.check_deadline(now)
        first = self.started is None
        if first:
            self.started, self.deadline = now, now + 2.0
        self.bad_stamp = stamp
        self.paired_stamps.clear()
        return first

    def paired(self, stamp, now):
        self.check_deadline(now)
        if self.started is None:
            return True
        if (
            len(self.paired_stamps) < 3
            and stamp > self.bad_stamp
            and (not self.paired_stamps or stamp > self.paired_stamps[-1])
        ):
            self.paired_stamps.append(stamp)
        return len(self.paired_stamps) >= 3
