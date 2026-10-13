# P09-P15 teaching evidence and replay boundary

Source: `motion_points_1791868992949070655.json` and its `.jsonl` audit.
User supplied the archive on 2026-10-13. All 21 point snapshots match their audit records.
P01-relative positions are odometry estimates, not surveyed ground truth. Original evidence is retained in training notes.

| Point | P01 x (m) | P01 y (m) | Measured yaw (deg) |
|---|---:|---:|---:|
| P09_candidate | 1.406550278 | -1.587888598 | -90.252880 |
| P10_near_wall | 1.404901348 | -1.885592599 | -89.932141 |
| P10_after_turn | 1.395508779 | -1.899381861 | -179.549146 |
| P11_candidate | 0.941867495 | -1.905847412 | -179.890099 |
| P11_after_turn | 0.937720413 | -1.896458033 | 90.773590 |
| P12_candidate | 0.922137173 | -1.091586873 | 90.542664 |
| P12_after_turn | 0.924428235 | -1.081301790 | 179.996913 |
| P13_candidate | 0.425801955 | -1.081909901 | -179.983699 |
| P13_after_turn | 0.419932699 | -1.071645807 | 89.628548 |
| P14_candidate | 0.422487778 | -0.573262696 | 89.513160 |
| P14_after_turn | 0.423013987 | -0.565709349 | 178.923940 |
| P15_candidate | 0.028999609 | -0.559657567 | 179.306525 |

## Acceptance limits

The user reported no visible wall contact during the manually sampled later translations and turns. This does not establish swept clearance or repeated autonomous success.
P10 was intentionally retained as `P10_near_wall`: fitted left gaps were negative despite no reported visible contact.
The audit contains one `Final stopped pose outside enabled tolerances` event at P11 rotation. The user then measured/recorded and manually continued; this is not a clean uninterrupted automated P01-P15 run.
P15 has no after-turn record. The automatic trial stops at P15, with no implicit 180-degree turn.

## Replay choices

Keep existing P01-P08 targets. Copy P09-P15 candidate xy positions exactly; do not accumulate relative segment deltas or use old global odom poses.
Hold fixed P01-relative cardinal headings rather than copying the small sampled yaw errors. Sequence after P08: right translation to P09; forward to P10; R90; forward P11; R90; forward P12; L90; forward P13; R90; forward P14; L90; forward P15.
Each new post-turn translation uses the actual stopped start only within 2 cm / 1 degree of the nominal start/heading. Endpoints stay fixed. Other starts retain 1 cm / 1 degree checks. No implicit recovery from a failed turn.
Interpolation is at most 5 cm. These chords are not a recorded collision-checked trajectory. Laser remains observation-only.

Source JSON SHA256: `8a2b7c1b37016cf6541761143499426a54b3962c60713aad7184d9b227624812`


2026-10-13 replay correction: user reported left-wall contact on P09-P10.
P10 target x changes from 1.4049013479064716 to 1.3849013479064716 m;
y stays -1.8855925989094273 m. At P01 yaw-90, decreasing x is body-right.
P09-P10 gradually shifts right; P10-P11 starts at the revised nominal P10
and retains its fixed P11 endpoint. All other waypoint targets and headings
are unchanged. The original teaching table remains historical evidence.
This endpoint adjustment does not guarantee 2 cm physical clearance along
the whole segment; collision-free replay is pending. Test `path P10` from
initialized P01 first; it stops before the P10 turn. No mid-route resume.
