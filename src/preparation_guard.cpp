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
  return false;
}
