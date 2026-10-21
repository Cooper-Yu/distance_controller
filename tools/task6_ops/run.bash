#!/usr/bin/env bash
# Repository entry point: sources ROS, then execs a bounded read-only helper.
set -eo pipefail
source /opt/ros/humble/setup.bash
if [[ -n "${TASK6_OVERLAY:-}" ]]; then
  source "$TASK6_OVERLAY"
fi
set -u
folder="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
mode="${1:-help}"
if [[ $# -gt 0 ]]; then shift; fi
case "$mode" in
  record) exec python3 "$folder/evidence_recorder.py" "$@" ;;
  watch) exec python3 "$folder/odom_reset_watch.py" "$@" ;;
  *) printf '%s\n' 'Usage: bash tools/task6_ops/run.bash record|watch [options]'; exit 2 ;;
esac