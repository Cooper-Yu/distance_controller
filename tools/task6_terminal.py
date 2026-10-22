"""Directed P15 half-turn; observation-only geometry with strict feedback freshness."""

from contextlib import contextmanager
import math

from motion_model import Motion
from route_laser_guard import RADIUS
from teaching_path import turn_to_heading

TERMINAL_NAME = 'P15_after_turn'


def turn_motion(action, origin, actual):
    """Keep the final half-turn positive even when odom is either side of pi."""
    if action.name == TERMINAL_NAME:
        return Motion('turn', angle=math.pi)
    return turn_to_heading(origin, actual, action.heading_rad)


def observed_turn_gap(points):
    """Optional recorded-data diagnostic, never an observation-only motion gate."""
    if not points or any(not math.isfinite(v) for point in points for v in point):
        raise RuntimeError('Terminal turn requires observed laser points')
    return min(math.hypot(x, y) for x, y in points) - RADIUS


@contextmanager
def terminal_guard(console, action):
    """Preserve feedback/TF stops without imposing a swept-envelope obstacle veto."""
    if action.name != TERMINAL_NAME:
        yield
        return
    previous = console.command_guard

    def checked(command):
        console.runner.moving_pose()
        console.runner.geometry()
        return previous(command) if previous is not None else command

    checked((0.0, 0.0, 0.0))
    console.emit(
        'terminal_turn_policy',
        name=action.name,
        angle_rad=math.pi,
        swept_envelope_gate=False,
        laser_policy='enforced' if getattr(previous, 'enforced', False) else 'observation_only',
    )
    console.command_guard = checked
    try:
        yield
    finally:
        console.command_guard = previous
