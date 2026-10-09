# Independent segments and history return

## Configure geometry once; select order separately

Use the installed independent profile instead of passing shared forward/lateral lengths:

```bash
ros2 run distance_controller distance_controller 2 --ros-args \
  --params-file "$(ros2 pkg prefix distance_controller)/share/distance_controller/config/segments.yaml" \
  -p 'route:=[AB]'
```

The profile contains AB/BC/CB/BA and a short example BD. Each selected edge has its own
`dx`, `dy`, speed, dwell, completion, side-centering, timeout and travel limit. AB is
0.93 m and BC is 0.516780 m rightward, preserving the last user trial. Route axes are
fixed at initialized A. These are relative segment displacements, not absolute map
coordinates. Manual mode pauses after every selected segment; `resume` executes the
next configured edge. Set manual_mode:=false only for automatic sequential execution.

`segment_configuration=independent` requires explicit dx/dy for every selected edge
and rejects forward_distance/lateral_distance overrides. This prevents two competing
sources of geometry. `legacy` remains the default when no profile is loaded: existing
commands keep their shared-length defaults and explicit per-edge overrides still win.
Changing AB in independent mode does not change BA; edit both when desired.

Edit a copied YAML file for experiments or override `segments.AB.dx:=...`. Only selected
edges are instantiated/validated. Switching to a previously unused edge validates it
at the next startup. The installed YAML is a deployment copy of the repository file.

## Explore and return within one running node

From another sourced terminal, wait for WAITING, then execute one command at a time:

```bash
ros2 run distance_controller step right 0.10 --label C
# Wait for completion, then:
ros2 run distance_controller step forward 0.10 --label D
ros2 run distance_controller step history
```

The history command is read-only and works while running or faulted. It lists initial
visit 0 at A and every completed traversal with an edge ID, from/to visit IDs, measured
start/end pose, held heading and executed policy. Failed, canceled and incomplete
steps never become completed history. Actual starts are captured before motion;
ends are committed only after position/heading, standstill and dwell acceptance.

Choose one return command:

```bash
ros2 run distance_controller step backtrack 1
ros2 run distance_controller step backtrack 2
ros2 run distance_controller step return_to B
ros2 run distance_controller step return_to A
```

For A->B->C->D, return_to B executes D->C->B, not a direct D->B chord. The accepted
batch proceeds through each leg's arrival, standstill and dwell automatically and
enters WAITING only at its destination. Busy requests are rejected. `cancel` stops
and latches the existing fault state; uncompleted return legs remain unconsumed.
Any already completed return legs remain recorded. A fault requires restart.

**An accepted return replaces any pending named route.** After returning, define the
next exploration step; the old pending BC/BD route will not resume from the wrong point.
Rejection leaves both the pending route and completed history unchanged.

This follows recorded waypoints, not a dense record of the exact wheel path. Each
return leg is an absolute-odom target at that outward leg's measured start, using
closed-loop position control and the persistent heading. It does not replay velocity
commands. Original speed/dwell/time/travel limits are retained; side-centering and
front-wall arrival are disabled for return and replaced by position arrival. Going
back from a front-wall stop therefore does not try to satisfy the same front gap.
There is no obstacle detection on these return legs and no guarantee of exact physical
path retracing. Side-corrected outward paths may not be straight; use history return
only where the waypoint-to-waypoint return corridor is suitable.

## Repeated names and branching

A->B->C->B contains two visits named B. A named return is rejected as ambiguous rather
than guessing which one you mean. Read history, then select the exact active visit:

```bash
ros2 run distance_controller step return_to --visit-id 1
```

Visit 0 is the initialized A. Outward completion creates a new unique visit ID;
reverse completion returns to an earlier visit and gets its own audit edge ID.
The active_visits line lists currently reachable ancestors. Returned branches remain
in the audit log but cannot be selected as ancestors of a new branch. Returning to the
current visit is rejected. A count of zero or beyond the active path is also rejected.

## Check placement before continuation

While WAITING and before accepting motion, compare current odom position with the
last verified stopped pose. Displacement greater than resume_position_tolerance
(default 0.02 m) latches WAIT_POSITION_CHANGED and publishes zero. Heading drift alone
continues to use the existing heading-hold correction. Fresh odom, stopped feedback
and heading tolerance are required before a new motion request is accepted.

This does not automatically re-run A's side/rear centering at B or D. Investigate
placement/localization before restarting; a new startup will perform A preparation,
so it must be placed in the intended initialization region. Automatic sequential
routes retain normal segment arrival checks; each return leg also checks its departure
pose before target initialization. No mid-route A initialization is added.

History is **current-process/current-odom-reference only**. Restarting clears it;
there is no saved-state restoration or cross-session coordinate alignment. A small
odom discontinuity within the tolerance or physical motion invisible to odom cannot
be reliably detected by this position check. Hardware return paths still need validation.

## Implementation boundaries

`route_history.hpp/.cpp` contain the completed audit, active ancestry and pure return
planning. `history_control.cpp` connects them to accepted poses and continuation
checks. `manual_steps.cpp` handles requests; `route_execution.cpp` executes all legs
through the existing PID and heading helpers. I and D remain zero by default; turn
control is reserved for Task3/4. No competing velocity publisher is added.
