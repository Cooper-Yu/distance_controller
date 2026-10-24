# Task6 corner placement candidate — 2026-10-25

This is an engineering candidate, not course PASS or validated obstacle avoidance.

- `--corner-placement off` preserves existing motion.
- `front` replaces only P10 travel completion with fresh fitted front-body clearance 0.090 +/- 0.005 m, capped at 0.020 m/s. It keeps other obstacle geometry observation-only, the original feedback/owner/pause protections and 4cm turn guard. P09 follows the existing route. A stopped multi-scan endpoint is explicit evidence for a bounded next connector; P01 is never reset.
- `lidar` additionally enables experimental P09 lane selection and raw swept-geometry checks. Local synthetic tests pass, but the real closed P09 sample is BLOCKED. It is NOT the next hardware candidate.
- The reviewed P11 endpoint is extended 5cm forward. The P10→P11 connector is regenerated from the accepted P10 endpoint to this fixed P11 endpoint. The original 2cm/1deg post-turn check still applies relative to that accepted point. P12–P15 fixed targets remain.

Targeted local tests cover actual route dispatch P09→P10→turn→P11 with simulated I/O, nonzero velocity and convergence, distinct stopped scans, stale/identity/cancel cleanup, feedback recovery, short fit-loss zero/wait, connector integrity and unchanged 4cm turn rejection. These do not prove real stopping distance or full maze clearance.

Observed data: the preceding real run initialized successfully with 1cm position tolerance, reached P14_after_turn, then failed the unchanged 2cm connector check. Earlier P10 front gap decreased to1.63cm while odom tracking continued. Next hardware config is FRONT only, speed .06, turn .20, start tolerance .010, full P15 destination and deadline03:50 Beijing. Same-version complete route count remains0/3 until verified runtime evidence.
