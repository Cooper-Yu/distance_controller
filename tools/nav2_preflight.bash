#!/usr/bin/env bash
# Read-only ROS interface audit. Never publishes velocity or starts navigation.
set -eo pipefail
source /opt/ros/humble/setup.bash
if [[ -f "$HOME/ros2_ws/install/setup.bash" ]]; then
  source "$HOME/ros2_ws/install/setup.bash"
fi
out="$HOME/ros2_ws/nav2_preflight_$(date +%Y%m%d_%H%M%S)_$$"
mkdir -p "$out"
probe() {
  local name="$1"
  shift
  local rc=0
  timeout --signal=INT --kill-after=2s 8s "$@" > "$out/$name.txt" 2>&1 || rc=$?
  printf '%s: exit=%s\n' "$name" "$rc" | tee -a "$out/summary.txt"
}
printf 'ROS_DISTRO=%s\nROS_DOMAIN_ID=%s\nROS_LOCALHOST_ONLY=%s\n' \
  "${ROS_DISTRO:-unset}" "${ROS_DOMAIN_ID:-unset}" "${ROS_LOCALHOST_ONLY:-unset}" > "$out/environment.txt"
probe topics ros2 topic list -t
probe nodes ros2 node list
probe packages ros2 pkg list
for topic in scan scan_filtered odometry/filtered cmd_vel tf tf_static map clock; do
  probe "topic_${topic//\//_}" ros2 topic info "/$topic" --verbose
done
probe scan_header ros2 topic echo /scan_filtered --once --field header
probe odom_header ros2 topic echo /odometry/filtered --once --field header
probe odom_child ros2 topic echo /odometry/filtered --once --field child_frame_id
probe odom_pose ros2 topic echo /odometry/filtered --once --field pose.pose
probe tf_odom_base ros2 run tf2_ros tf2_echo odom base_link
probe tf_base_laser ros2 run tf2_ros tf2_echo base_link laser
probe tf_map_base ros2 run tf2_ros tf2_echo map base_link
probe robot_description ros2 param get /robot_state_publisher robot_description
probe robot_use_sim_time ros2 param get /robot_state_publisher use_sim_time
probe map_metadata ros2 topic echo /map --once --field info
for pkg in nav2_bringup nav2_controller nav2_amcl nav2_mppi_controller slam_toolbox rosbag2_transport; do
  probe "package_$pkg" ros2 pkg prefix "$pkg"
done
printf '\nAudit finished. Timeouts for continuous tf2_echo are expected; inspect file contents.\nMissing map before SLAM/localization is not itself a fault.\n' | tee -a "$out/summary.txt"
tar -czf "$out.tar.gz" -C "$(dirname "$out")" "$(basename "$out")"
printf '\nSend this archive: %s.tar.gz\n' "$out"