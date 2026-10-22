"""Opt-in live-session commands; one action per request, never replay old audit files."""

from dataclasses import asdict

from route_session_state import LiveRouteState
from task6_route_model import reconnect_route
from task6_terminal import terminal_guard, turn_motion


class CheckedInput:
    """Use the same cancellation source while waiting and inside Console.move."""

    def __init__(self, source, cancellation):
        self.source, self.cancellation = source, cancellation

    def poll(self):
        self.cancellation.check()
        return self.source.poll()


def forward_once(console, state, cancellation):
    runner = console.runner
    actual = runner.pose()
    state.check_waiting(actual)
    if state.cursor >= len(state.actions):
        raise ValueError('Approved prefix complete; next cannot cross its destination gate')
    action = state.actions[state.cursor]
    motion = (
        turn_motion(action, state.origin, actual) if action.kind == 'turn' else action.path_motion()
    )
    if action.kind == 'path' and state.history and state.history[-1].motion.kind == 'turn':
        motion = reconnect_route(motion, state.origin, actual)
    state.begin_forward(motion, actual)
    console.emit(
        'supervised_action_started',
        index=state.cursor,
        name=action.name,
        actual_start=asdict(actual),
        motion=asdict(motion),
    )
    with terminal_guard(console, action):
        console.move(motion, use_path_length=True)
    cancellation.check()
    # Repeating an approved action after a return must not overwrite its first point record.
    visit = (
        sum(name.startswith(action.name + '_visit_') for name in console.book.data['points']) + 1
    )
    console.record(f'{action.name}_visit_{visit}')
    state.finish_forward(runner.pose())
    console.emit(
        'supervised_action_completed',
        index=state.cursor - 1,
        name=action.name,
        actual=asdict(state.last_stop),
    )


def return_once(console, state, token, cancellation):
    motion = state.begin_back(token, console.runner.pose())
    console.emit('supervised_return_started', plan=asdict(state.pending[1]))
    console.move(motion, use_path_length=True)
    cancellation.check()
    index = state.finish_back(console.runner.pose())
    console.emit('supervised_return_completed', index=index, actual=asdict(state.last_stop))


def dispatch(console, state, line, cancellation):
    words = line.split()
    if words == ['status']:
        console.emit(
            'supervised_status',
            cursor=state.cursor,
            origin=asdict(state.origin),
            last_stop=asdict(state.last_stop),
            route_hash=state.route_hash,
        )
    elif words == ['next']:
        forward_once(console, state, cancellation)
    elif words == ['back_plan']:
        plan = state.preview_back(console.runner.pose())
        console.emit('supervised_return_plan', plan=asdict(plan))
        print(f'RETURN_PLAN {plan.token} {plan.name} -> prior actual start', flush=True)
        print(f'back {plan.token}', flush=True)
    elif len(words) == 2 and words[0] == 'back':
        return_once(console, state, words[1], cancellation)
    else:
        raise ValueError('Expected next | status | back_plan | back TOKEN | quit')


def run_supervised(console, actions, cancellation, inputs):
    """Run only the supplied approved prefix, leaving the original P01 and process alive.

    No turn undo, partial-action recovery, route replacement, or cross-process restore exists.
    A command error before motion is rejected; every fault after begin latches and exits.
    """
    if not getattr(console.command_guard, 'enforced', False):
        raise RuntimeError('Supervised motion requires an enforced laser guard')
    state = LiveRouteState(console.book.origin, actions, console.book.data['session_id'])
    console.inputs = CheckedInput(inputs, cancellation)
    console.emit(
        'supervised_session_started',
        session_id=state.session_id,
        route_hash=state.route_hash,
        origin=asdict(state.origin),
    )
    print('SUPERVISED: next | status | back_plan | back TOKEN | quit', flush=True)
    while True:
        console.runner.pump()
        cancellation.check()
        present, line = console.inputs.poll()
        if not present:
            continue
        if line is None or line == 'quit':
            console.emit(
                'supervised_session_closed',
                cursor=state.cursor,
                prefix_complete=state.cursor == len(actions),
            )
            return state
        try:
            dispatch(console, state, line, cancellation)
        except (ValueError, RuntimeError) as error:
            if state.pending or state.failure or console.runner.invalid:
                state.fail(error)
                console.emit('supervised_session_failed', cursor=state.cursor, reason=str(error))
                raise
            console.emit('supervised_command_rejected', reason=str(error))
            print(f'REJECTED: {error}', flush=True)
        except (Exception, KeyboardInterrupt) as error:
            state.fail(error)
            console.emit(
                'supervised_session_failed', cursor=state.cursor, reason=str(error) or 'Cancelled'
            )
            raise
        print(f'WAITING {state.cursor}/{len(actions)}', flush=True)
