"""Explicit shadow injection with quantitative holes; no production entry changes."""

import time

from route_laser_coverage import diagnose, paired_kinds
from route_laser_runtime import (
    ShadowRouteLaserGuard,
    current_points,
    scan_transform,
    scheduled_rays,
)


class CoverageShadowRouteLaserGuard(ShadowRouteLaserGuard):
    """Log strict hypothetical guard plus independently useful finite-point evidence.

    Optional raw_supplier returns a /scan message; only exact matching headers
    and geometry can distinguish filter-created holes. No raw subscription is
    created here. Commands and the existing odom/drift guards are unchanged.
    """

    def __init__(self, runner, emit=None, policy=None, raw_supplier=None):
        if policy is None:
            super().__init__(runner, emit)
        else:
            super().__init__(runner, emit, policy)
        self.raw_supplier = raw_supplier
        self.evidence = None

    def snapshot(self):
        self.evidence = None
        snapshot = super().snapshot()
        runner = self.runner
        scan = runner.observer.scan
        at_scan, current = runner.scan_pose(), runner.moving_pose()
        transform = scan_transform(runner, scan)
        origin = current_points(
            ((transform.translation.x, transform.translation.y, 0),), at_scan, current
        )[0]
        raw = self.raw_supplier() if self.raw_supplier else None
        kinds, paired = paired_kinds(scan, raw)
        directions = scheduled_rays(scan, transform, at_scan.yaw - current.yaw)
        rays = tuple(
            (bearing, float(distance) if valid else None, kind)
            for (bearing, valid), distance, kind in zip(directions, scan.ranges, kinds)
        )
        self.evidence = rays, origin, paired
        return snapshot

    def report(self, decision, candidate, snapshot=None):
        key = decision.state, decision.reason, decision.scale
        if not self.emit or (key == self.last_report and time.monotonic() - self.last_reported < 1):
            return
        super().report(decision, candidate, snapshot)
        detail = dict(
            mode='coverage_shadow',
            enforced=False,
            executable=False,
            certified_free=False,
            proposed_command=candidate,
            applied_command=candidate,
        )
        if snapshot is None or self.evidence is None:
            detail.update(state='unavailable', reason=decision.reason)
        else:
            try:
                rays, origin, paired = self.evidence
                detail = diagnose(
                    snapshot, rays, candidate, self.runner.body_velocity, origin, self.policy
                )
                detail['raw_pair_matched'] = paired
                detail['scan_stamp_ns'] = self.runner.cached_stamp
                detail['odom_stamp_ns'] = self.runner.stamp
            except (ValueError, ArithmeticError) as error:
                detail.update(state='unavailable', reason=str(error))
        self.emit('laser_coverage_shadow', **detail)
