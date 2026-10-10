"""Bounded stopped verification before restarting a turn at reduced speed.

The 2 cm hard margin and 0.5 s prediction horizon remain unchanged. A 3.5 cm
approach margin requests a stop/restart at 0.08 rad/s, not a competing publisher.
"""

import math
import time
from wall_geometry import DIRECTIONS, swept_clearance


def remaining_clearance(points, counts, delta):
    """Include the current footprint and entire remaining rotation, with quadrant coverage."""
    for side, center in DIRECTIONS.items():
        n = sum(
            abs(math.atan2(math.sin(p[2] - center), math.cos(p[2] - center))) < math.radians(20)
            for p in points
        )
        if n < 8 or n < 0.5 * counts[side]:
            raise RuntimeError(f'Turn scan coverage unavailable: {side}')
    steps = max(1, math.ceil(abs(delta) / 0.04))
    return min(swept_clearance(points, wz=delta * i / steps, duration=1) for i in range(steps + 1))


def recover_turn(runner, target):
    """Keep zero output; require three fresh stopped scans over 0.2 s within 2 s.

    Restart needs 2.5 cm throughout the remaining sweep (5 mm hysteresis).
    Missing/stale feedback is an immediate failure, never evidence of free space.
    """
    stamp, odom_stamp = runner.cached_stamp, runner.stamp
    count, first = 0, None
    deadline = time.monotonic() + 2.0
    runner.velocity_pub = runner.node.create_publisher(runner.twist, '/cmd_vel', 10)
    print('TURN_RECOVERING: zero commands; 2 s budget, three stopped scans', flush=True)
    try:
        while time.monotonic() < deadline:
            runner.send((0.0, 0.0, 0.0))
            runner.pump()
            actual = runner.moving_pose()
            points, counts = runner.geometry()
            if stamp == runner.cached_stamp or odom_stamp == runner.stamp:
                continue
            stamp, odom_stamp = runner.cached_stamp, runner.stamp
            gap = remaining_clearance(points, counts, target.yaw - actual.yaw)
            stopped = runner.latest[2] < 0.01 and runner.latest[3] < 0.02
            if gap < 0.025 or not stopped:
                count, first = 0, None
                continue
            count += 1
            first = time.monotonic() if first is None else first
            if count >= 3 and time.monotonic() - first >= 0.2:
                print(
                    f'TURN_RECOVERED: remaining sweep={gap:.4f} m; speed=0.08 rad/s; target retained',
                    flush=True,
                )
                return
        raise RuntimeError('TURN_RECOVERY_TIMEOUT: remaining clearance not confirmed')
    finally:
        runner.send((0.0, 0.0, 0.0))
        runner.node.destroy_publisher(runner.velocity_pub)
        runner.velocity_pub = None


def supervised_turn(runner, target, initial_speed=0.20):
    """Stop and reap the child before recovery; at most two retries to the same yaw."""
    from wall_runtime import TurnClearanceError

    if initial_speed not in (0.08, 0.20):
        raise ValueError('Unsupported supervised turn speed')
    runner.turn_speed = initial_speed
    for attempt in range(3):
        runner.turn_sign = 1 if target.yaw - runner.pose().yaw > 0 else -1
        runner.guarded_command = None
        runner.turn_guard = True
        try:
            runner.rotate_target(target)
            return
        except TurnClearanceError as error:
            runner.turn_guard = False
            runner.stop_owned()
            runner.child = None
            print(str(error), flush=True)
            if attempt == 2:
                raise RuntimeError('TURN_RECOVERY_BUDGET: two retries exhausted') from error
            recover_turn(runner, target)
            runner.turn_speed = 0.08
        finally:
            runner.turn_guard = False
