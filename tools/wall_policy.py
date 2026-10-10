"""Reviewable per-action wall policies and a pure low-speed translation control law."""

from dataclasses import dataclass, asdict
import math


def clamp(value, limit):
    return max(-limit, min(limit, value))


@dataclass(frozen=True)
class Policy:
    """Side reference, stop surface, offset from carried clearance and maximum path (meters)."""

    follow: str = 'none'
    stop: str = 'distance'
    offset: float = 0.0
    max_travel: float = 1.0
    capture: str = 'none'
    follow_clearance: float | None = None
    stop_clearance: float | None = None
    follow_offset: float = 0.0
    stop_tolerance: float = 0.008
    align_follow_first: bool = False

    def validate(self, kind):
        if self.follow not in ('none', 'left', 'right') or self.capture not in (
            'none',
            'left',
            'right',
        ):
            raise ValueError('Unknown reference wall')
        if self.stop not in ('distance', 'front', 'left', 'right'):
            raise ValueError('Unknown stopping wall')
        if not math.isfinite(self.offset) or not -0.03 <= self.offset <= 0.10:
            raise ValueError('Clearance offset outside [-.03,.10] m')
        if not math.isfinite(self.max_travel) or not 0 < self.max_travel <= 3:
            raise ValueError('Maximum travel outside (0,3] m')
        if kind == 'turn' and (self.follow != 'none' or self.stop != 'distance'):
            raise ValueError('Turns cannot run translation wall control')
        expected = {'front': 'forward', 'left': 'left', 'right': 'right'}
        if self.stop != 'distance' and expected[self.stop] != kind:
            raise ValueError('Stopping wall must be in the movement direction')
        if self.follow != 'none' and kind != 'forward':
            raise ValueError('Wall-follow correction is for forward actions only')
        if self.capture != 'none' and kind != 'turn':
            raise ValueError('Recapture is a stopped post-turn operation')
        if not math.isfinite(self.stop_tolerance) or not 0.005 <= self.stop_tolerance <= 0.010:
            raise ValueError('Stop tolerance must be within [.005,.010] m')
        if type(self.align_follow_first) is not bool or (
            self.align_follow_first and (kind != 'forward' or self.follow == 'none')
        ):
            raise ValueError('align_follow_first requires a forward wall-follow action')
        self.validate_clearances()
        return self

    def validate_clearances(self):
        """Reject unused or unsafe explicit targets; None retains carried-reference behavior."""
        for name, enabled in (
            ('follow_clearance', self.follow != 'none'),
            ('stop_clearance', self.stop != 'distance'),
        ):
            value = getattr(self, name)
            if value is not None:
                if not enabled or not math.isfinite(value) or not 0.04 <= value <= 0.35:
                    raise ValueError(f'{name} requires an active wall and a finite [.04,.35] m gap')
        if not math.isfinite(self.follow_offset) or not -0.03 <= self.follow_offset <= 0.10:
            raise ValueError('Follow offset outside [-.03,.10] m')
        if self.follow == 'none' and self.follow_offset != 0:
            raise ValueError('Follow offset requires a follow wall')

    def clearances(self, reference):
        """Resolve targets independently. Absolute values override offsets without changing d."""
        if not math.isfinite(reference) or not 0.04 <= reference <= 0.30:
            raise ValueError('Reference clearance outside [.04,.30] m')
        self.validate_clearances()
        follow = (
            (
                self.follow_clearance
                if self.follow_clearance is not None
                else reference + self.follow_offset
            )
            if self.follow != 'none'
            else None
        )
        stop = (
            (self.stop_clearance if self.stop_clearance is not None else reference + self.offset)
            if self.stop != 'distance'
            else None
        )
        for value in (follow, stop):
            if value is not None and not 0.04 <= value <= 0.35:
                raise ValueError('Resolved clearance outside [.04,.35] m')
        return follow, stop


def command(step, policy, reference, progress, cross, yaw_error, gaps, max_speed=0.03):  # noqa: PLR0912
    # Keep mutually exclusive axis/arrival rules together for sign review.
    """Return body-aligned desired vx/vy/wz, ready and residuals; runtime rotates XY to live body.

    Read planned-heading progress/cross error and fitted body clearances. Writes no state.
    Side corrections use positive-left convention. Required missing gaps are errors.
    """
    if not math.isfinite(max_speed) or not 0.01 <= max_speed <= 0.08:
        raise ValueError('Wall speed must be finite and within [.01,.08] m/s')
    axis = {'forward': (1, 0), 'backward': (-1, 0), 'left': (0, 1), 'right': (0, -1)}[step.kind]
    follow_goal, stop_goal = policy.clearances(reference)
    residual = step.value - progress
    if policy.stop != 'distance':
        residual = gaps[policy.stop] - stop_goal
        if residual < -0.015:
            raise RuntimeError('Stopping wall already too close; inspect and back')
    tolerance = policy.stop_tolerance if policy.stop != 'distance' else 0.008
    ready = abs(residual) <= tolerance
    speed = 0 if ready else clamp(0.7 * residual, max_speed)
    vx, vy = axis[0] * speed, axis[1] * speed
    side_error = cross
    if policy.follow != 'none':
        side_error = gaps[policy.follow] - follow_goal
        if abs(side_error) > 0.08:
            raise RuntimeError('Reference wall changed by more than 8 cm; possible opening')
        vy = clamp(0.7 * side_error, 0.012) * (1 if policy.follow == 'left' else -1)
    else:
        if axis[0]:
            vy = clamp(-0.7 * cross, 0.012)
        else:
            vx = clamp(-0.7 * cross, 0.012)
    if abs(side_error) <= 0.008:
        if axis[0]:
            vy = 0.0
        else:
            vx = 0.0
    ready = ready and abs(side_error) <= 0.008 and abs(yaw_error) <= 0.01
    wz = 0.0 if abs(yaw_error) <= 0.01 else clamp(1.2 * yaw_error, 0.15)
    # Do not translate through a large heading deviation.
    if abs(yaw_error) > 0.08:
        vx = vy = 0.0
    return (
        (vx, vy, wz),
        ready,
        {'along_m': residual, 'side_m': side_error, 'heading_rad': yaw_error},
    )


def policy_dict(policy):
    return asdict(policy)


class FrontStopVerification:
    """Freeze all axes on entering the front band; require three distinct stopped scans.

    A lost fit or out-of-band sample invalidates verification. After confirmation,
    lateral/heading correction may run, while the front distance remains monitored.
    This is separate from the final all-axis endpoint hold.
    """

    def __init__(self):
        self.reset()

    def reset(self):
        self.count = 0
        self.stamp = None
        self.confirmed = False

    def apply(self, policy, residual, stamp, stopped, velocity, ready):
        if policy.stop != 'front':
            return velocity, ready
        if abs(residual['along_m']) > policy.stop_tolerance:
            self.reset()
            return velocity, ready
        if self.confirmed:
            return velocity, ready
        if not stopped:
            self.count = 0
            self.stamp = stamp
        elif stamp != self.stamp:
            self.stamp = stamp
            self.count += 1
        if self.count >= 3:
            self.confirmed = True
        # Even the confirming tick stays zero; correction starts on the next tick.
        return (0.0, 0.0, 0.0), False


class FollowAlignment:
    """Bounded pre-translation side correction, rechecked on every resumed attempt.

    No commanded motion along the planned heading until three stopped scans and
    a 0.5 s hold satisfy side/heading tolerances. All actual travel still counts
    against the parent action budget; this phase also has a 6 cm / 20 s bound.
    """

    def __init__(self, enabled, now, path):
        self.done = not enabled
        self.started, self.path_start = now, path
        self.hold, self.stamp, self.count = None, None, 0

    def invalidate(self):
        self.hold, self.stamp, self.count = None, None, 0

    def apply(self, policy, residual, stamp, stopped, now, path):
        if self.done:
            return None
        if now - self.started > 20 or path - self.path_start > 0.06:
            raise RuntimeError('FOLLOW_ALIGNMENT_LIMIT: stopped; inspect side correction')
        side, heading = residual['side_m'], residual['heading_rad']
        ready = abs(side) <= 0.005 and abs(heading) <= 0.01
        if ready and stopped:
            if self.hold is None:
                self.hold = now
            if stamp != self.stamp:
                self.stamp = stamp
                self.count += 1
            if self.count >= 3 and now - self.hold >= 0.5:
                self.done = True
        else:
            self.invalidate()
        vy = 0.0 if abs(side) <= 0.005 else clamp(0.7 * side, 0.01)
        vy *= 1 if policy.follow == 'left' else -1
        wz = 0.0 if abs(heading) <= 0.01 else clamp(1.2 * heading, 0.08)
        if abs(heading) > 0.08:
            vy = 0.0
        return (0.0, vy, wz)
