"""Wall-route session state: actual endpoints advance future plans, planned yaw stays continuous."""

from dataclasses import asdict, replace
from action_plan import Session, Pose, targets
from wall_policy import Policy


class WallSession(Session):
    """Keep observed endpoint anchors and rollback the carried wall reference on back()."""

    @property
    def goals(self):
        result = []
        anchor = self.origin
        for i, step in enumerate(self.steps):
            goal = targets(anchor, [step])[0]
            result.append(goal)
            record = next((r for r in self.active if r['index'] == i), None)
            if record:
                actual = Pose(**record['end'])
                anchor = Pose(actual.x, actual.y, goal.yaw)
            else:
                anchor = goal
        return result

    def next(self):
        before = self.backend.reference
        try:
            actual = super().next()
        except (Exception, KeyboardInterrupt):
            if self.partial is not None:
                self.partial.setdefault('reference_before', before)
            raise
        self.active[-1]['reference_before'] = before
        self.active[-1]['reference_after'] = self.backend.reference
        self.active[-1]['wall_result'] = self.backend.last_report
        self.emit('wall_result', self.active[-1])
        return actual

    def adjust_turn(self, distance):
        """Explicit rightward preparation; never starts the pending rotation."""
        from turn_adjustment import adjust_session

        return adjust_session(self, distance)

    def resume(self):
        adjustment = (self.partial or {}).get('turn_adjustment')
        if adjustment and not adjustment['done']:
            raise RuntimeError('Finish the fixed adjust_turn target or back before turning')
        actual = super().resume()
        self.active[-1]['reference_after'] = self.backend.reference
        self.active[-1]['wall_result'] = self.backend.last_report
        self.emit('wall_result', self.active[-1])
        return actual

    def back(self):
        record = self.partial or (self.active[-1] if self.active else None)
        before = (
            record.get('reference_before', self.backend.reference)
            if record
            else self.backend.reference
        )
        actual = super().back()
        self.backend.reference = before
        self.emit('reference_restored', {'body_clearance_m': before})
        return actual

    def set_policy(self, index, field, value):
        """Edit independent future gaps or bounds; None restores inherited absolute targets."""
        if not self.cursor <= index < len(self.steps):
            raise ValueError('Only future policies may be edited')
        if (
            field
            not in (
                'offset',
                'max_travel',
                'follow_offset',
                'follow_clearance',
                'stop_clearance',
                'stop_tolerance',
            )
            or not self.steps[index].wall
        ):
            raise ValueError('Unknown wall policy field')
        if value is None and field not in ('follow_clearance', 'stop_clearance'):
            raise ValueError('auto is only valid for absolute clearance fields')
        if self.partial is not None:
            self.check_partial_policy_edit(index, field)
        data = dict(self.steps[index].wall)
        data[field] = value
        if self.partial is not None and field == 'follow_clearance':
            old_goal = Policy(**self.steps[index].wall).clearances(self.backend.reference)[0]
            if value is None or abs(value - old_goal) > 0.03:
                raise ValueError(
                    'Partial follow adjustment requires an absolute target within 3 cm'
                )
            data['align_follow_first'] = True
        candidate = replace(self.steps[index], wall=data).validate()
        previous = asdict(self.steps[index])
        self.steps[index] = candidate
        self.revision += 1
        if self.partial is not None:
            self.partial['step'] = asdict(candidate)
            self.partial['revision'] = self.revision
        self.emit(
            'policy_edit',
            {
                'index': index,
                'revision': self.revision,
                'step': asdict(candidate),
                'previous': previous,
            },
        )

    def check_partial_policy_edit(self, index, field):
        """Recalibrate a stopped outward front action; preserve start/path/history."""
        record = self.partial
        if (
            index != self.cursor
            or record['index'] != index
            or record.get('return_started')
            or record.get('resume_unavailable')
            or self.steps[index].wall.get('stop') != 'front'
            or field not in ('stop_clearance', 'stop_tolerance', 'follow_clearance')
            or (
                field == 'follow_clearance'
                and self.steps[index].wall.get('follow', 'none') == 'none'
            )
        ):
            raise ValueError(
                'Partial forward/front action permits bounded follow_clearance or stop target edits'
            )
        self.check_location()  # Fresh stopped feedback and unchanged odom are required.

    def reload(self, steps):
        if not all(s.wall for s in steps):
            raise ValueError('Wall session requires explicit policies on every action')
        super().reload(steps)
