"""Explicit close-point escape, confined to a fixed 3 cm rightward translation.

Only this operator-invoked recovery permits an existing gap below 2 cm. Every
close point must recede along the command and measured-motion prediction. Normal
translation and rotation thresholds are unchanged; no point is removed.
"""

import math
import time
from action_plan import Pose, errors
from wall_geometry import point_clearance, swept_clearance
from turn_recovery import remaining_clearance


def escape_command(actual, state):
    """Translate along the initial right axis; stop on yaw drift rather than turning near a wall."""
    origin, target = Pose(**state['origin']), Pose(**state['target'])
    if abs(errors(actual, origin)[1]) > 0.015:
        raise RuntimeError('ESCAPE_HEADING: drift exceeds 0.015 rad; stopped')
    dx, dy = target.x - actual.x, target.y - actual.y
    forward = math.cos(origin.yaw) * dx + math.sin(origin.yaw) * dy
    right = math.sin(origin.yaw) * dx - math.cos(origin.yaw) * dy
    if abs(forward) > 0.004 or right < -0.003:
        raise RuntimeError('ESCAPE_PATH: cross-track or overshoot exceeds bound')
    speed = min(0.005, max(0.0, right * 0.7)) if right > 0.003 else 0.0
    relative = origin.yaw - actual.yaw
    return (math.sin(relative) * speed, -math.cos(relative) * speed, 0.0), errors(actual, target)[
        0
    ] <= 0.003


def check_points(points, velocity, floor, cleared=False, duration=0.5):
    """Never approach an existing close point or create a new sub-2 cm prediction."""
    if not points:
        raise RuntimeError('ESCAPE_SCAN: no points')
    static = min(point_clearance(x, y) for x, y, _ in points)
    if static < floor or (cleared and static < 0.02):
        raise RuntimeError('ESCAPE_CLEARANCE: static gap worsened or below floor')
    for point in points:
        baseline = point_clearance(*point[:2])
        predicted = swept_clearance([point], *velocity, duration=duration)
        if baseline < 0.02 and not cleared:
            if point[1] <= 0 or predicted < baseline - 0.0003:
                raise RuntimeError('ESCAPE_DIRECTION: close point does not recede')
        elif predicted < 0.02:
            raise RuntimeError('ESCAPE_OBSTACLE: new predicted gap below 2 cm')
    return static


def preflight(runner, state):
    """Three fresh stopped scans must support the full translation and subsequent rotation."""
    deadline, stamp, samples = time.monotonic() + 3.0, None, []
    target = Pose(**state['target'])
    while time.monotonic() < deadline:
        runner.pump()
        actual = runner.moving_pose()
        points, counts = runner.geometry()
        if runner.latest[2] >= 0.01 or runner.latest[3] >= 0.02:
            raise RuntimeError('ESCAPE_START: robot must be stopped')
        if stamp == runner.cached_stamp:
            continue
        stamp = runner.cached_stamp
        velocity, _ = escape_command(actual, state)
        runner.translation_coverage(points, counts, velocity, 'right')
        # Full quadrants required even while translating, because the following action is a turn.
        remaining_clearance(points, counts, 0)
        gap = min(point_clearance(x, y) for x, y, _ in points)
        state.setdefault('floor', max(0.012, min(0.02, gap - 0.002)))
        dx, dy = target.x - actual.x, target.y - actual.y
        c, s = math.cos(actual.yaw), math.sin(actual.yaw)
        bx, by = c * dx + s * dy, -s * dx + c * dy
        check_points(points, (bx, by, 0), state['floor'], state['cleared'], duration=1)
        shifted = [(x - bx, y - by, math.atan2(y - by, x - bx)) for x, y, _ in points]
        if remaining_clearance(shifted, counts, state['final_yaw'] - actual.yaw) < 0.025:
            raise RuntimeError('ESCAPE_TARGET: insufficient remaining rotation clearance')
        samples.append(gap)
        if len(samples) >= 3:
            if max(samples) - min(samples) > 0.005:
                raise RuntimeError('ESCAPE_SCAN: unstable initial clearance')
            print(
                f'ESCAPE_READY: floor={state["floor"]:.4f} m; fixed right target; speed<=0.005 m/s',
                flush=True,
            )
            return
    raise RuntimeError('ESCAPE_START_TIMEOUT: three fresh scans unavailable')


def guard_escape(runner, state, velocity):
    """Check each fresh cloud against commanded and measured motion, including drift."""
    points, counts = runner.geometry()
    runner.translation_coverage(points, counts, velocity, 'right')
    remaining_clearance(points, counts, 0)
    if runner.twist_frame != 'base_link':
        raise RuntimeError('ESCAPE_FRAME: requires base_link velocity')
    for motion in (velocity, runner.body_velocity):
        gap = check_points(points, motion, state['floor'], state['cleared'])
    if gap >= 0.022:
        state['cleared'] = True
