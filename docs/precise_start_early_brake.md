# Precise-start early braking

A real preparation run (`repeat_20261022_150223`, base release `6c33462`)
commanded at most 4 mm/s during a 5 mm odom correction. At its positional arrival,
raw wheel odom reported approximately 39 mm/s; both raw and filtered positions
continued changing after zero was commanded. A new stopped laser batch then
rejected the changed pose. This supports a real base/feedback stopping transient,
not a filter-only jump; the underlying drivetrain/delivery cause remains unproven.

The microstep now releases motion when projected progress reaches half its fixed
plan, capped at 2.5 mm. This is an early brake for remeasurement, not target arrival.
It returns a distinct `remeasure` result, sends zero and runs the existing fresh
stopped 15-scan verification. Only that batch may accept P01 or plan a remaining
correction. The nominal target remains fixed while a step is active.

Nonfinite/frame/freshness, 4 s, 12 mm travel and 3 mm cross-track guards run before
this decision. The microstep heading limit is the configured final heading tolerance,
capped at 0.5 degrees (tighter configuration is preserved). Speed caps, three refinements, 30 s
positioning / 60 s total limits, final 5 mm / 0.5 degree acceptance and route
4 cm turn drift protection are unchanged. No old-run resume or target rebasing.

Tests cover both signs, diagonal/short plans, guard priority, the recorded field
pose and repeated residual correction. The isolated ROS fixture additionally
injects 7 mm motion after zero and requires stopped remeasurement before success.
It verifies control/acceptance sequencing, not the robot's actual braking distance.
Hardware reproduction and repeatability are pending; no Task6 completion claim.
In `p10_forward_20261022_154339`, a step drift of 0.005168rad (0.296deg)
triggered the previous extra 0.005rad (0.286deg) microstep bound before early
braking, although it remained inside the final half-degree gate. The microstep
bound now matches that final tolerance, capped at 0.5deg, with the original
heading-reference error also checked. This changes an intermediate guard, not
P01 final acceptance; a fresh stopped laser batch is still required. The actual
field pose is a regression case, plus over-limit and tighter-configuration checks.