#!/bin/bash
source /opt/ros/humble/setup.bash
source ${HOME}/checkpoint16_sim_ws/install/setup.bash
source ${HOME}/checkpoint18_assets/local_support/install/setup.bash
source ${HOME}/ros2_ws/install/setup.bash
set -eo pipefail
export ROS_DOMAIN_ID=184 ROS_LOCALHOST_ONLY=1 IGN_PARTITION=cp18_turn_audit GZ_PARTITION=cp18_turn_audit LIBGL_ALWAYS_SOFTWARE=1
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
logroot=/tmp/turn_physics_audit_$(date +%s)
mkdir -p "$logroot"
printf '%s\n' "$logroot" > /tmp/turn_physics_latest.txt
sim_pid=''
cleanup() {
 if [ -n "$sim_pid" ]; then
  kill -INT -- -"$sim_pid" 2>/dev/null || true
  sleep 2
  kill -TERM -- -"$sim_pid" 2>/dev/null || true
  wait "$sim_pid" 2>/dev/null || true
 fi
}
trap cleanup EXIT
for wall in 0.30 0.23 0.185; do
 export TURN_AUDIT_OUT="$logroot/wall_$wall" TURN_AUDIT_WALL_Y="$wall"
 mkdir -p "$TURN_AUDIT_OUT"
 setsid ros2 launch "$root/turn_audit.launch.py" > "$TURN_AUDIT_OUT/sim.log" 2>&1 &
 sim_pid=$!
 sleep 18
 timeout --signal=INT --kill-after=5s 135 python3 "$root/verify_turn_audit.py" > "$TURN_AUDIT_OUT/result.log" 2>&1
 cat "$TURN_AUDIT_OUT/result.log"
 cleanup
 sim_pid=''
done
python3 - "$logroot" <<'PY'
import json
import sys
from pathlib import Path
root = Path(sys.argv[1])
for wall, expected in [('0.30', 'completed'), ('0.23', 'completed'), ('0.185', 'contact_stop')]:
    summary = json.loads((root / f'wall_{wall}/summary.json').read_text())
    assert summary['result'] == expected, (wall, summary)
    assert summary['contact_packets'] >= 3, (wall, summary)
    if expected == 'completed':
        assert summary['wall_contacts'] == 0 and abs(summary['final_yaw'] + 1.5708) < .02
    else:
        assert summary['wall_contacts'] > 0
print('Three-case simulation comparison verified:', root)
PY