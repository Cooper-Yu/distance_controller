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
        return self


def command(step, policy, reference, progress, cross, yaw_error, gaps, max_speed=0.03):  # noqa: PLR0912
    # Keep mutually exclusive axis/arrival rules together for sign review.
    """Return body-aligned desired vx/vy/wz, ready and residuals; runtime rotates XY to live body.

    Read planned-heading progress/cross error and fitted body clearances. Writes no state.
    Side corrections use positive-left convention. Required missing gaps are errors.
    """
    if not math.isfinite(max_speed) or not 0.01 <= max_speed <= 0.08:
        raise ValueError('Wall speed must be finite and within [.01,.08] m/s')
    axis = {'forward': (1, 0), 'backward': (-1, 0), 'left': (0, 1), 'right': (0, -1)}[step.kind]
    if not 0.04 <= reference <= 0.30:
        raise ValueError('Reference clearance outside [.04,.30] m')
    residual = step.value - progress
    if policy.stop != 'distance':
        goal = reference + policy.offset
        if not 0.04 <= goal <= 0.35:
            raise ValueError('Stopping clearance outside [.04,.35] m')
        residual = gaps[policy.stop] - goal
        if residual < -0.015:
            raise RuntimeError('Stopping wall already too close; inspect and back')
    ready = abs(residual) <= 0.008
    speed = 0 if ready else clamp(0.7 * residual, max_speed)
    vx, vy = axis[0] * speed, axis[1] * speed
    side_error = cross
    if policy.follow != 'none':
        side_error = gaps[policy.follow] - reference
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
