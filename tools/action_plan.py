"""Pure action planning and same-process execution history for supervised surveys."""

from dataclasses import asdict, dataclass, replace
import math


@dataclass(frozen=True)
class Pose:
    """Odom position (meters), continuous planned yaw (radians)."""

    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class Step:
    """Named independent motion; value is meters or signed radians for turn."""

    name: str
    kind: str
    value: float

    def validate(self):
        if not self.name or not isinstance(self.name, str) or len(self.name) > 40:
            raise ValueError('Each action needs a nonempty name of at most 40 characters')
        if not math.isfinite(self.value):
            raise ValueError('Action value must be finite')
        if self.kind == 'turn':
            if not 0 < abs(self.value) <= 2 * math.pi:
                raise ValueError('Turn must be nonzero and at most one full revolution')
        elif self.kind not in ('forward', 'backward', 'left', 'right') or not 0 < self.value <= 3:
            raise ValueError('Translation needs a supported direction and 0 < distance <= 3 m')
        return self


def load_steps(data):
    """Convert reviewable JSON actions with explicit units into validated immutable steps."""
    steps = []
    for row in data['actions']:
        unit, kind = row['unit'], row['kind']
        if unit not in (('deg', 'rad') if kind == 'turn' else ('m',)):
            raise ValueError('Translation unit must be m; turn unit must be deg or rad')
        value = float(row['value'])
        steps.append(
            Step(row['name'], kind, math.radians(value) if unit == 'deg' else value).validate()
        )
    if not steps or len({s.name for s in steps}) != len(steps):
        raise ValueError('Route must be nonempty with unique action names')
    return steps


def targets(origin, steps):
    """Read prior PLANNED pose, never actual stopping pose; compile all downstream targets."""
    result = []
    p = origin
    for step in steps:
        step.validate()
        if step.kind == 'turn':
            p = Pose(p.x, p.y, p.yaw + step.value)
        else:
            dx, dy = {
                'forward': (step.value, 0),
                'backward': (-step.value, 0),
                'left': (0, step.value),
                'right': (0, -step.value),
            }[step.kind]
            p = Pose(
                p.x + math.cos(p.yaw) * dx - math.sin(p.yaw) * dy,
                p.y + math.sin(p.yaw) * dx + math.cos(p.yaw) * dy,
                p.yaw,
            )
        result.append(p)
    return result


def reverse_steps(steps):
    """Build inverse actions for a new endpoint session; use back() for actual current history."""
    inverse = {'forward': 'backward', 'backward': 'forward', 'left': 'right', 'right': 'left'}
    return [
        Step(
            f'R{i:02d}',
            'turn' if s.kind == 'turn' else inverse[s.kind],
            -s.value if s.kind == 'turn' else s.value,
        ).validate()
        for i, s in enumerate(reversed(steps), 1)
    ]


def errors(actual, expected):
    """Return position norm in meters and signed wrapped heading error in radians."""
    return (
        math.hypot(actual.x - expected.x, actual.y - expected.y),
        math.atan2(math.sin(expected.yaw - actual.yaw), math.cos(expected.yaw - actual.yaw)),
    )


class Session:
    """Separate editable future plan from append-only history and actual return anchors.

    Backend supplies fresh stopped pose and execute(step, target). emit writes a
    durable event before/after work. Failed/partial steps cannot advance the cursor.
    Histories cannot be restored across controller sessions or odom resets.
    """

    def __init__(self, origin, steps, backend, emit):
        targets(origin, steps)
        self.origin, self.steps = origin, list(steps)
        self.backend, self.emit = backend, emit
        self.cursor, self.revision = 0, 0
        self.active, self.partial = [], None
        self.last_actual = origin
        self.emit('origin', {'pose': asdict(origin), 'actions': [asdict(s) for s in steps]})

    @property
    def goals(self):
        return targets(self.origin, self.steps)

    def check_location(self):
        actual = self.backend.pose()
        if self.last_actual is None:
            raise RuntimeError(
                'Last stop is unknown; recover feedback and inspect before a new session'
            )
        distance, angle = errors(actual, self.last_actual)
        if distance > 0.03 or abs(angle) > 0.05:
            raise RuntimeError(
                'Pose changed while waiting: possible relocation/odom reset; no motion'
            )
        return actual

    def edit(self, index, value):
        """Edit a future action by zero-based index; recompile without rewriting past events."""
        if self.partial is not None or not self.cursor <= index < len(self.steps):
            raise ValueError(
                'Return a partial/executed step before editing it; only future actions editable'
            )
        candidate = replace(self.steps[index], value=value).validate()
        self.steps[index] = candidate
        self.revision += 1
        self.emit(
            'edit',
            {
                'index': index,
                'revision': self.revision,
                'step': asdict(candidate),
                'targets': [asdict(p) for p in self.goals],
            },
        )

    def reload(self, steps):
        """Allow appending/replacing future actions; an executed prefix must remain identical."""
        if self.partial is not None or steps[: self.cursor] != self.steps[: self.cursor]:
            raise ValueError('Cannot replace active/partial or already executed actions')
        if len(steps) < self.cursor:
            raise ValueError('New route removes executed actions')
        targets(self.origin, steps)
        self.steps, self.revision = list(steps), self.revision + 1
        self.emit('reload', {'revision': self.revision, 'actions': [asdict(s) for s in steps]})

    def perform(self, step, target, record):
        """Attempt motion with a saved start; retain incomplete records and best stopped pose."""
        self.partial = record
        self.emit('started', record)
        try:
            self.backend.execute(step, target)
            actual = self.backend.pose()
            distance, angle = errors(actual, target)
            if distance > 0.03 or abs(angle) > 0.02:
                raise RuntimeError(
                    f'Endpoint outside acceptance: {distance:.4f} m, {angle:.4f} rad'
                )
            self.last_actual = actual
            return actual
        except (Exception, KeyboardInterrupt):
            try:
                self.last_actual = self.backend.pose()
            except (Exception, KeyboardInterrupt):
                self.last_actual = None
            self.emit(
                'incomplete',
                {**record, 'last_stop': asdict(self.last_actual) if self.last_actual else None},
            )
            raise

    def next(self):
        if self.partial is not None:
            raise RuntimeError('Incomplete motion: inspect and back before retrying')
        if self.cursor >= len(self.steps):
            raise ValueError('All actions completed')
        start = self.check_location()
        step, goal = self.steps[self.cursor], self.goals[self.cursor]
        record = {
            'index': self.cursor,
            'revision': self.revision,
            'step': asdict(step),
            'start': asdict(start),
            'target': asdict(goal),
        }
        actual = self.perform(step, goal, record)
        record = {**record, 'end': asdict(actual), 'error': errors(actual, goal)}
        self.emit('completed', record)
        self.active.append(record)
        self.cursor += 1
        self.partial = None
        return actual

    def back(self):
        """Return one actual start, including a partial action; caller confirms clear return path."""
        self.check_location()
        record = self.partial or (self.active[-1] if self.active else None)
        if record is None:
            raise ValueError('Already at session origin')
        returning_partial = self.partial is not None
        goal = Pose(**record['start'])
        # Retain original record even if a return itself fails, for subsequent recovery.
        step = Step(
            'return_' + str(record['index']), record['step']['kind'], record['step']['value']
        )
        self.emit('return_requested', {'source': record, 'target': asdict(goal)})
        actual = self.perform(step, goal, record)
        self.emit('returned', {'index': record['index'], 'actual': asdict(actual)})
        # A failed return of a completed action also has an active entry at this index.
        if self.active and self.active[-1]['index'] == record['index']:
            self.active.pop()
        if not returning_partial or self.cursor > record['index']:
            self.cursor = record['index']
        self.partial = None
        return actual
