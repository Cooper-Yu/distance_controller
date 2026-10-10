/** @file
 * @brief Bounded initialization stages with explicit sensor, duration and travel failure reasons.
 */
#include <cmath>

#include "distance_controller/distance_controller.hpp"

const char * DistanceController::preparation_stage_name() const
{
  switch (preparation_stage_) {
    case PreparationStage::WaitingForWall:
      return "wall_measurement";
    case PreparationStage::Aligning:
      return "heading_alignment";
    case PreparationStage::Positioning:
      return "positioning";
  }
  return "unknown";
}

double DistanceController::preparation_stage_timeout() const
{
  switch (preparation_stage_) {
    case PreparationStage::WaitingForWall:
      return wall_measurement_timeout_;
    case PreparationStage::Aligning:
      return alignment_timeout_;
    case PreparationStage::Positioning:
      return positioning_timeout_;
  }
  return 0.0;
}

void DistanceController::update_preparation_stage(const std::chrono::steady_clock::time_point & now)
{
  auto next = preparation_stage_;
  if (initial_alignment_complete_) {
    next = PreparationStage::Positioning;
  } else if (
    preparation_stage_ == PreparationStage::WaitingForWall && scan_valid_ && accepted_scan_ &&
    right_heading_valid_ && right_heading_stable_) {
    next = PreparationStage::Aligning;
  }
  if (next == preparation_stage_) return;
  preparation_stage_ = next;
  preparation_stage_start_ = now;
  RCLCPP_INFO(
    get_logger(), "Preparation stage=%s deadline=%.1f s (total limit=%.1f s)",
    preparation_stage_name(), preparation_stage_timeout(), preparation_timeout_);
}

void DistanceController::fail_preparation(
  const char * reason, const std::chrono::steady_clock::time_point & now)
{
  fault_latched_ = true;
  centering_settling_ = false;
  alignment_settling_ = false;
  reset_pid();
  publish_stop();
  RCLCPP_ERROR(
    get_logger(),
    "Initial preparation fault: reason=%s stage=%s stage_elapsed=%.3f s total_elapsed=%.3f s "
    "scan_reason=%s wall_valid=%s wall_stable=%s angle=%.3f deg rms=%.4f m span=%.3f m",
    reason, preparation_stage_name(),
    std::chrono::duration<double>(now - preparation_stage_start_).count(),
    std::chrono::duration<double>(now - preparation_start_).count(), scan_rejection_reason_.c_str(),
    right_heading_valid_ ? "true" : "false", right_heading_stable_ ? "true" : "false",
    right_wall_angle_ * 180.0 / 3.141592653589793, right_wall_rms_, right_wall_span_);
}

bool DistanceController::handle_centering_guard(
  const std::chrono::steady_clock::time_point & now, bool should_log)
{
  if (!heading_control_enabled_ || centering_complete_) return false;
  if (!preparation_started_) {
    preparation_start_ = preparation_stage_start_ = now;
    preparation_x_ = last_odom_.pose.pose.position.x;
    preparation_y_ = last_odom_.pose.pose.position.y;
    preparation_started_ = true;
    RCLCPP_INFO(
      get_logger(), "Preparation stage=%s deadline=%.1f s (total limit=%.1f s)",
      preparation_stage_name(), preparation_stage_timeout(), preparation_timeout_);
  }
  const double elapsed = std::chrono::duration<double>(now - preparation_start_).count();
  const double stage_elapsed =
    std::chrono::duration<double>(now - preparation_stage_start_).count();
  const double travel = std::hypot(
    last_odom_.pose.pose.position.x - preparation_x_,
    last_odom_.pose.pose.position.y - preparation_y_);
  const double age = std::chrono::duration<double>(now - last_scan_time_).count();
  const char * reason = nullptr;
  if (accepted_scan_ && !scan_valid_)
    reason = "SCAN_INVALID";
  else if (accepted_scan_ && age > scan_timeout_)
    reason = "SCAN_STALE";
  else if (travel > preparation_max_travel_)
    reason = "PREPARATION_TRAVEL_LIMIT";
  else if (elapsed > preparation_timeout_)
    reason = "PREPARATION_TOTAL_TIMEOUT";
  else if (stage_elapsed > preparation_stage_timeout()) {
    switch (preparation_stage_) {
      case PreparationStage::WaitingForWall:
        reason = "WALL_MEASUREMENT_TIMEOUT";
        break;
      case PreparationStage::Aligning:
        reason = "HEADING_ALIGNMENT_TIMEOUT";
        break;
      case PreparationStage::Positioning:
        reason = "POSITIONING_TIMEOUT";
        break;
    }
  }
  if (reason) {
    fail_preparation(reason, now);
    return true;
  }
  if (!scan_valid_ || !accepted_scan_) {
    publish_stop();
    if (should_log)
      RCLCPP_INFO(
        get_logger(), "Waiting for initial scan: stage=%s reason=%s", preparation_stage_name(),
        scan_rejection_reason_.c_str());
    return true;
  }
  update_preparation_stage(now);
  if (should_log) log_preparation_progress();
  return false;
}

void DistanceController::log_preparation_progress()
{
  const double yaw = quaternion_to_yaw(last_odom_.pose.pose.orientation);
  const double lateral_error = (left_wall_ - right_wall_) * 0.5;
  const double rear_error = rear_target_distance_ - rear_wall_;
  const double yaw_error = initial_alignment_complete_ ? heading_error(yaw) : -right_wall_angle_;
  RCLCPP_INFO(
    get_logger(),
    "Initialization progress: stage=%s | pose: x=%.6f y=%.6f yaw=%.6f rad | "
    "left=%.3f right=%.3f rear=%.3f rear_target=%.3f m | "
    "center_error=%.4f rear_error=%.4f tolerance=%.4f m | "
    "heading_error=%.4f tolerance=%.4f rad heading_basis=%s | "
    "position_ready=%s heading_ready=%s settling=%s required_hold=%.2f s",
    preparation_stage_name(), last_odom_.pose.pose.position.x, last_odom_.pose.pose.position.y, yaw,
    left_wall_, right_wall_, rear_wall_, rear_target_distance_, lateral_error, rear_error,
    centering_tolerance_, yaw_error, heading_tolerance_,
    initial_alignment_complete_ ? "held_odom_heading"
                                : (alignment_wall_ == "left" ? "left_wall" : "right_wall"),
    std::abs(lateral_error) <= centering_tolerance_ && std::abs(rear_error) <= centering_tolerance_
      ? "true"
      : "false",
    right_heading_valid_ && right_heading_stable_ && !initial_alignment_complete_
      ? (std::abs(yaw_error) <= heading_tolerance_ ? "true" : "false")
      : (initial_alignment_complete_ && std::abs(yaw_error) <= heading_tolerance_ ? "true"
                                                                                  : "false"),
    (initial_alignment_complete_ ? centering_settling_ : alignment_settling_) ? "true" : "false",
    alignment_settle_duration_);
}

void DistanceController::begin_odom_recovery(const std::chrono::steady_clock::time_point & receipt)
{
  if (odom_recovering_ || fault_latched_) return;
  odom_recovering_ = true;
  odom_recovery_stable_ = false;
  recovery_start_ = receipt;
  recovery_anchor_ = last_odom_;
  settling_ = segment_completed_ = alignment_settling_ = centering_settling_ = false;
  reset_pid();
  publish_stop();
  RCLCPP_WARN(
    get_logger(),
    "ODOM_RECOVERING: zero commands; 2 s budget, require 0.3 s fresh stopped "
    "feedback; current target retained");
}

bool DistanceController::validate_odom_recovery(
  const nav_msgs::msg::Odometry & msg, const std::chrono::steady_clock::time_point & receipt)
{
  if (fault_latched_) return false;
  const double gap = std::chrono::duration<double>(receipt - last_odom_time_).count();
  if (received_odom_ && gap > 0.5) begin_odom_recovery(receipt);
  if (odom_recovering_ && std::chrono::duration<double>(receipt - recovery_start_).count() >= 2.0) {
    fail_odom_recovery("ODOM_RECOVERY_TIMEOUT");
    return false;
  }
  const rclcpp::Time stamp(msg.header.stamp, get_clock()->get_clock_type());
  const rclcpp::Time previous(last_odom_.header.stamp, get_clock()->get_clock_type());
  const double age = (now() - stamp).seconds();
  if (
    stamp.nanoseconds() <= 0 || age < -0.1 || age > (odom_recovering_ ? 0.15 : 0.5) ||
    (received_odom_ && stamp <= previous)) {
    odom_recovery_stable_ = false;
    return false;
  }
  if (!odom_recovering_) return true;
  const auto & a = recovery_anchor_.pose.pose;
  const auto & b = msg.pose.pose;
  const double yaw_delta = quaternion_to_yaw(b.orientation) - quaternion_to_yaw(a.orientation);
  if (
    msg.header.frame_id != recovery_anchor_.header.frame_id ||
    msg.child_frame_id != recovery_anchor_.child_frame_id ||
    std::hypot(b.position.x - a.position.x, b.position.y - a.position.y) > 0.25 ||
    std::abs(std::atan2(std::sin(yaw_delta), std::cos(yaw_delta))) > 0.35) {
    fail_odom_recovery("ODOM_RECOVERY_POSE_OR_FRAME_JUMP");
    return false;
  }
  const auto & v = msg.twist.twist;
  const bool stopped = std::hypot(v.linear.x, v.linear.y) < 0.01 && std::abs(v.angular.z) < 0.02;
  if (!stopped)
    odom_recovery_stable_ = false;
  else if (!odom_recovery_stable_ || gap > 0.15) {
    odom_recovery_stable_ = true;
    recovery_since_ = receipt;
  }
  return true;
}

bool DistanceController::handle_odom_recovery(const std::chrono::steady_clock::time_point & receipt)
{
  publish_stop();
  if (std::chrono::duration<double>(receipt - recovery_start_).count() >= 2.0) {
    fail_odom_recovery("ODOM_RECOVERY_TIMEOUT");
    return true;
  }
  if (std::chrono::duration<double>(receipt - last_odom_time_).count() > 0.15) {
    odom_recovery_stable_ = false;
    return true;
  }
  if (
    !odom_recovery_stable_ ||
    std::chrono::duration<double>(receipt - recovery_since_).count() < 0.3)
    return true;
  odom_recovering_ = false;
  reset_pid();
  RCLCPP_INFO(
    get_logger(), "ODOM_RECOVERED: fresh stopped feedback stable; resuming original target");
  return true;
}

void DistanceController::fail_odom_recovery(const char * reason)
{
  fault_latched_ = true;
  odom_recovering_ = false;
  process_exit_code_ = 2;
  reset_pid();
  publish_stop();
  RCLCPP_ERROR(
    get_logger(), "%s: %s", reason,
    base_command_watchdog_verified_
      ? "base timeout policy enabled; exiting with status 2"
      : "automatic exit disabled; latched zero commands until restart");
  if (base_command_watchdog_verified_) rclcpp::shutdown();
}
