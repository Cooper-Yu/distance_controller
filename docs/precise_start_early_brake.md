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

Original nonfinite/frame/freshness, 4 s, 12 mm travel, 3 mm cross-track and 0.005 rad
microstep guards run before this decision. Speed caps, three refinements, 30 s
positioning / 60 s total limits, final 5 mm / 0.5 degree acceptance and route
4 cm turn drift protection are unchanged. No old-run resume or target rebasing.

Tests cover both signs, diagonal/short plans, guard priority, the recorded field
pose and repeated residual correction. The isolated ROS fixture additionally
injects 7 mm motion after zero and requires stopped remeasurement before success.
It verifies control/acceptance sequencing, not the robot's actual braking distance.
Hardware reproduction and repeatability are pending; no Task6 completion claim.