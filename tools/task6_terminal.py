"""Explicit directed P15 half-turn and conservative observed swept-radius precheck."""

from contextlib import contextmanager
import math

from motion_model import Motion
from route_laser_guard import RADIUS
from route_laser_runtime import RouteLaserGuard
from teaching_path import turn_to_heading

TERMINAL_NAME = 'P15_after_turn'


def turn_motion(action, origin, actual):
    """Keep the final half-turn positive even when odom is either side of pi."""
    if action.name == TERMINAL_NAME:
        return Motion('turn', angle=math.pi)
    return turn_to_heading(origin, actual, action.heading_rad)


def observed_turn_gap(points):
    """The centrally symmetric footprint sweeps its circumscribed disk over pi radians."""
    if not points or any(not math.isfinite(v) for point in points for v in point):
        raise RuntimeError('Terminal turn requires observed laser points')
    return min(math.hypot(x, y) for x, y in points) - RADIUS


@contextmanager
def terminal_guard(console, action):
    """Check observed full sweep before ownership; guard each later nonzero command."""
    if action.name != TERMINAL_NAME:
        yield
        return
    previous = console.command_guard
    guard = RouteLaserGuard(console.runner, console.emit)
    snapshot = guard.snapshot()
    gap = observed_turn_gap(snapshot.points)
    console.emit(
        'terminal_turn_preflight',
        name=action.name,
        angle_rad=math.pi,
        observed_sweep_gap_m=gap,
        required_gap_m=guard.policy.hard_m,
        scan_stamp_ns=console.runner.cached_stamp,
    )
    if gap < guard.policy.hard_m:
        raise RuntimeError('Terminal half-turn observed sweep lacks 2 cm clearance')
    console.command_guard = guard
    try:
        yield
    finally:
        console.command_guard = previous
