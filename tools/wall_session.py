"""Wall-route session state: actual endpoints advance future plans, planned yaw stays continuous."""

from dataclasses import asdict, replace
from action_plan import Session, Pose, targets


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

    def resume(self):
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
        if self.partial is not None or not self.cursor <= index < len(self.steps):
            raise ValueError('Only future policies may be edited')
        if (
            field
            not in ('offset', 'max_travel', 'follow_offset', 'follow_clearance', 'stop_clearance')
            or not self.steps[index].wall
        ):
            raise ValueError('Unknown wall policy field')
        if value is None and field not in ('follow_clearance', 'stop_clearance'):
            raise ValueError('auto is only valid for absolute clearance fields')
        data = dict(self.steps[index].wall)
        data[field] = value
        candidate = replace(self.steps[index], wall=data).validate()
        self.steps[index] = candidate
        self.revision += 1
        self.emit(
            'policy_edit', {'index': index, 'revision': self.revision, 'step': asdict(candidate)}
        )

    def reload(self, steps):
        if not all(s.wall for s in steps):
            raise ValueError('Wall session requires explicit policies on every action')
        super().reload(steps)
