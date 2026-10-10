#!/usr/bin/env python3
"""Interactive one-origin route editor/executor with verified stopped checkpoints."""

import argparse
import json
import math
import os
from pathlib import Path
import shlex
import signal
import time

from action_plan import Pose, Session, Step, load_steps, reverse_steps, targets
from session_checkpoint import read_checkpoint, write_checkpoint, restore_checkpoint


def read_route(path):
    return load_steps(json.loads(Path(path).read_text()))


def export_route(path, steps):
    """Save action definitions only; no odom origin or resumable execution state."""
    rows = [
        {
            'name': s.name,
            'kind': s.kind,
            'value': math.degrees(s.value) if s.kind == 'turn' else s.value,
            'unit': 'deg' if s.kind == 'turn' else 'm',
            **({'wall': s.wall} if s.wall else {}),
        }
        for s in steps
    ]
    Path(path).write_text(json.dumps({'actions': rows}, indent=2) + '\n')


def show(steps, goals, cursor=0):
    for i, (step, pose) in enumerate(zip(steps, goals), 1):
        value = math.degrees(step.value) if step.kind == 'turn' else step.value
        unit = 'deg' if step.kind == 'turn' else 'm'
        print(
            f'{">" if i == cursor + 1 else " "} {i:02} {step.name}: {step.kind} {value:.4f} {unit}'
            f' -> ({pose.x:.4f}, {pose.y:.4f}, yaw={math.degrees(pose.yaw):.3f} deg)'
        )
        if step.wall:
            print(
                f'     WALL policy={step.wall}; XY is a preview until preceding stops are measured'
            )


class Journal:
    """Append-only audit with flush/fsync; never reused to resume a reset odom epoch."""

    def __init__(self, backend):
        self.backend = backend
        self.path = Path(f'action_session_{time.time_ns()}.jsonl')
        self.file = self.path.open('x')

    def emit(self, event, data):
        row = {
            'time': time.time(),
            'event': event,
            'data': data,
            'laser': self.backend.observations.copy(),
        }
        self.file.write(json.dumps(row, allow_nan=False) + '\n')
        self.file.flush()
        os.fsync(self.file.fileno())
        self.backend.observations.clear()


def edit_command(session, words):
    """Edit definitions without starting motion; recompute future targets for review."""
    verb = words[0]
    if verb == 'set' and len(words) == 3:
        index, value = int(words[1]) - 1, float(words[2])
        if not 0 <= index < len(session.steps):
            raise ValueError('Action index out of range')
        if session.steps[index].kind == 'turn':
            value = math.radians(value)
        session.edit(index, value)
        if (
            session.steps[index].wall
            and session.steps[index].wall.get('stop', 'distance') != 'distance'
        ):
            print(
                'Wall-stop action: set changes preview distance only; use policy INDEX offset VALUE for clearance.'
            )
        show(session.steps, session.goals, session.cursor)
    elif verb == 'load' and len(words) == 2:
        session.reload(read_route(words[1]))
        show(session.steps, session.goals, session.cursor)
    elif verb == 'save' and len(words) == 2:
        export_route(words[1], session.steps)
    else:
        raise ValueError('Usage: set INDEX VALUE | load FILE | save FILE')


def command(session, words):  # noqa: PLR0912 - one explicit branch per operator command
    """Handle one operator command; degrees at the CLI, radians in the plan."""
    verb = words[0]
    if verb == 'next' and len(words) == 1:
        print('Reached:', session.next())
    elif verb == 'resume' and len(words) == 1:
        if input('Inspect stopped pose and path. Type RESUME: ').strip() == 'RESUME':
            print('Reached:', session.resume())
    elif verb == 'manual_turn' and len(words) == 1 and hasattr(session, 'manual_turn'):
        print('Reached:', session.manual_turn())
    elif verb == 'rewind_turn' and len(words) == 1 and hasattr(session, 'rewind_turn'):
        print('Rewound:', session.rewind_turn())
    elif verb == 'adjust_turn' and len(words) == 2 and hasattr(session, 'adjust_turn'):
        print('Adjusted:', session.adjust_turn(float(words[1])))
    elif verb == 'escape_turn' and len(words) == 2 and hasattr(session, 'escape_turn'):
        print('Escaped:', session.escape_turn(float(words[1])))
    elif verb == 'checkpoint' and len(words) == 2:
        session.check_location()
        write_checkpoint(words[1], session)
        print('Stopped checkpoint saved:', words[1])
    elif verb == 'run' and len(words) == 1:
        if any(s.wall for s in session.steps):
            raise ValueError('Wall route is under commissioning: use next one action at a time')
        print('Continuous execution of remaining actions; Ctrl+C stops and marks partial.')
        if input('Type RUN to continue: ').strip() == 'RUN':
            while session.cursor < len(session.steps):
                print('Reached:', session.next(), flush=True)
    elif verb == 'back' and len(words) == 1:
        if input('Check return path is clear. Type BACK: ').strip() == 'BACK':
            print('Returned:', session.back())
    elif verb in ('set', 'load', 'save'):
        edit_command(session, words)
    elif verb == 'policy' and len(words) == 4 and hasattr(session, 'set_policy'):
        session.set_policy(
            int(words[1]) - 1, words[2], None if words[3] == 'auto' else float(words[3])
        )
        show(session.steps, session.goals, session.cursor)
    elif verb == 'plan':
        if any(s.wall for s in session.steps):
            print(
                f'Current carried body clearance: {session.backend.reference:.4f} m; absolute gaps override inherited reference + offsets'
            )
        show(session.steps, session.goals, session.cursor)
    elif verb == 'history':
        print(json.dumps(session.active, indent=2))
        print('Partial:', session.partial)
    elif verb == 'status':
        print(
            f'Next action={session.cursor + 1}, partial={session.partial is not None}, '
            f'actual={session.backend.pose()}'
        )
    elif verb == 'measure':
        session.backend.observe('manual')
        session.emit('measurement', {})
    else:
        print(
            'next | resume | manual_turn | rewind_turn | escape_turn 0.03 | adjust_turn RIGHT_METERS | checkpoint FILE | run | back | set INDEX METERS_OR_DEGREES | load FILE | save FILE | '
            'policy INDEX follow_clearance|stop_clearance METERS_OR_auto | policy INDEX follow_offset|offset|max_travel|stop_tolerance|right_front_window VALUE | plan | status | history | measure | quit'
        )


def session_loop(session, backend, checkpoint_path):
    """Persist stopped boundaries; invalidate autosave before each operator action."""
    while True:
        try:
            words = shlex.split(input('route> '))
            if not words:
                continue
            if words == ['quit']:
                break
            write_checkpoint(checkpoint_path, session, usable=False)
            command(session, words)
        except (ValueError, RuntimeError, OSError, KeyError) as error:
            print(f'STOPPED/REJECTED: {error}', flush=True)
        except KeyboardInterrupt:
            backend.cancel_requested = False
            print(
                'Stopped. Inspect status/history; back recovers an incomplete action.',
                flush=True,
            )
        except EOFError:
            break
        finally:
            write_checkpoint(checkpoint_path, session)


def run_session(steps, prepare, wall_speed=0.06, alignment_wall='right', recovery=None):
    """Capture one origin after optional wall preparation and retain it until exit."""
    import fcntl
    from action_runtime import PlannedRunner

    with open('/tmp/distance_controller_survey_return.lock', 'w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        wall_mode = any(s.wall for s in steps)
        if wall_mode:
            from wall_runtime import WallRunner
            from wall_session import WallSession

            backend = WallRunner(wall_speed, alignment_wall)
        else:
            backend = PlannedRunner()
        journal = Journal(backend)
        previous_handler = signal.signal(signal.SIGINT, lambda *_: backend.request_cancel())
        try:
            backend.exclusive()
            if recovery is not None:
                if recovery.get('legacy_audit'):
                    print('Legacy audit: no saved odom epoch; partial actions allow back only.')
                print('Recovery requires unchanged odom and no manual relocation; no motion yet.')
                if input('Confirm continuity: type SAME_ODOM: ').strip() != 'SAME_ODOM':
                    raise RuntimeError('Recovery continuity not confirmed')
            if prepare:
                backend.prepare_start()
            origin = backend.pose()
            if recovery is not None:
                session = restore_checkpoint(recovery, backend, journal.emit)
                origin = session.origin
            elif wall_mode:
                backend.capture_reference('left', origin.yaw)
                session = WallSession(origin, steps, backend, journal.emit)
                journal.emit(
                    'initial_reference',
                    {'body_clearance_m': backend.reference, 'speed_cap_m_s': wall_speed},
                )
            else:
                session = Session(origin, steps, backend, journal.emit)
            print(f'Origin fixed: {origin}; audit: {journal.path.resolve()}', flush=True)
            show(steps, session.goals, session.cursor)
            checkpoint_path = journal.path.with_suffix('.checkpoint.json')
            write_checkpoint(checkpoint_path, session)
            print(f'Auto-checkpoint: {checkpoint_path.resolve()}', flush=True)
            print('WAITING. next executes one action; help lists commands.', flush=True)
            session_loop(session, backend, checkpoint_path)
        finally:
            backend.stop_owned()
            backend.close(False)
            journal.file.close()
            signal.signal(signal.SIGINT, previous_handler)
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--wall-guided', action='store_true', help='Use provisional Task6 wall-clearance policies'
    )
    parser.add_argument(
        '--wall-speed',
        type=float,
        default=None,
        help='Wall translation cap in m/s: .01-.08, default .06',
    )
    parser.add_argument(
        '--alignment-wall',
        choices=['left', 'right'],
        default=None,
        help='Heading wall for wall-guided preparation only; default right. Following walls stay per-action.',
    )
    parser.add_argument(
        '--resume', type=Path, help='Restore a stopped checkpoint without preparation'
    )
    parser.add_argument('--route', type=Path, help='JSON action definitions')
    parser.add_argument(
        '--start',
        choices=['current', 'prepare'],
        help='Enter session; prepare runs wall alignment/centering once at A',
    )
    parser.add_argument(
        '--reverse', action='store_true', help='Inverse actions from the final pose'
    )
    args = parser.parse_args()
    if args.resume is not None:
        if (
            args.start
            or args.route
            or args.reverse
            or args.alignment_wall
            or args.wall_speed is not None
        ):
            parser.error(
                '--resume uses saved route/settings; do not combine with start/route overrides'
            )
        data = read_checkpoint(args.resume)
        return run_session(
            [Step(**row) for row in data['steps']], False, data['wall_speed'], recovery=data
        )
    if args.reverse and args.start == 'prepare':
        parser.error('Reverse starts at the final pose; do not run A preparation')
    if args.route is None:
        from ament_index_python.packages import get_package_share_directory

        args.route = Path(get_package_share_directory('distance_controller')) / (
            'config/task6_wall_actions.json' if args.wall_guided else 'config/task6_actions.json'
        )
    steps = read_route(args.route)
    if any(s.wall for s in steps) and not all(s.wall for s in steps):
        parser.error('Mixed wall and legacy actions are not supported')
    if args.wall_speed is not None:
        if not any(s.wall for s in steps):
            parser.error('--wall-speed requires a wall-guided route')
        if not math.isfinite(args.wall_speed) or not 0.01 <= args.wall_speed <= 0.08:
            parser.error('--wall-speed must be within [.01,.08] m/s')
    if args.alignment_wall is not None and (
        args.start != 'prepare' or not any(s.wall for s in steps)
    ):
        parser.error('--alignment-wall requires a wall-guided route with --start prepare')
    if args.reverse:
        steps = reverse_steps(steps)
    if args.start is None:
        show(steps, targets(Pose(0, 0, 0), steps))
        print('Preview only. --start current adopts the stopped pose; --start prepare prepares A.')
        return 0
    return run_session(
        steps, args.start == 'prepare', args.wall_speed or 0.06, args.alignment_wall or 'right'
    )


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (ValueError, RuntimeError, OSError, KeyboardInterrupt) as error:
        print(f'STOPPED: {error}')
        raise SystemExit(2) from error
