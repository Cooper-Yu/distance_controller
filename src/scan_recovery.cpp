/** @file
 * @brief Stop on a precise-preparation feedback gap; reacquire walls and odometry within a fixed budget.
 */
#include <cmath>

#include "distance_controller/distance_controller.hpp"

void DistanceController::begin_scan_recovery(const std::chrono::steady_clock::time_point & receipt)
{
  if (scan_recovering_ || fault_latched_) return;
  cancel_precise_correction("scan_recovery");
  scan_recovering_ = true;
  scan_recovery_start_ = receipt;
  scan_recovery_after_ = now().nanoseconds();
  scan_recovery_stamp_ = observation_stamp_ns_;
  scan_recovery_count_ = scan_recovery_odom_count_ = 0;
  scan_recovery_odom_stamp_ = rclcpp::Time(last_odom_.header.stamp).nanoseconds();
  scan_recovery_anchor_valid_ = received_odom_;
  scan_recovery_anchor_ = last_odom_;
  const bool precision_reset = precision_collecting_;
  if (precision_reset) {
    ++precision_diagnostics_.scan_resets;
    precision_diagnostics_.discarded += precision_samples_.size();
  }
  precision_collecting_ = precision_stationary_ = false;
  precision_samples_.clear();
  initial_alignment_complete_ = false;
  wall_target_frozen_ = wall_verifying_ = alignment_settling_ = centering_settling_ = false;
  wall_direction_window_.clear();
  right_heading_stable_ = false;
  reset_pid();
  publish_stop();
  if (precision_reset) log_precise_diagnostics("scan_recovery_reset", receipt);
  RCLCPP_WARN(
    get_logger(),
    "FEEDBACK_RECOVERING: zero commands; 5 s budget; fresh stopped scan and odom required");
}

bool DistanceController::handle_scan_recovery(const std::chrono::steady_clock::time_point & receipt)
{
  publish_stop();
  if (now().nanoseconds() < scan_recovery_after_) {
    fail_preparation("FEEDBACK_CLOCK_RESET", receipt);
    return true;
  }
  if (preparation_started_) {
    const double travel = std::hypot(
      last_odom_.pose.pose.position.x - preparation_x_,
      last_odom_.pose.pose.position.y - preparation_y_);
    if (
      travel > preparation_max_travel_ ||
      std::chrono::duration<double>(receipt - preparation_start_).count() > preparation_timeout_ ||
      std::chrono::duration<double>(receipt - preparation_stage_start_).count() >
        preparation_stage_timeout()) {
      fail_preparation(
        travel > preparation_max_travel_ ? "PREPARATION_TRAVEL_LIMIT"
                                         : "PREPARATION_DEADLINE_DURING_RECOVERY",
        receipt);
      return true;
    }
  }
  if (std::chrono::duration<double>(receipt - scan_recovery_start_).count() >= 5.0) {
    fail_preparation("FEEDBACK_RECOVERY_TIMEOUT", receipt);
    return true;
  }
  const auto & v = last_odom_.twist.twist;
  const rclcpp::Time stamp(last_odom_.header.stamp, get_clock()->get_clock_type());
  const double odom_age = (now() - stamp).seconds();
  const double scan_age = (now().nanoseconds() - last_scan_stamp_ns_) * 1e-9;
  const bool ready =
    received_odom_ && stamp.nanoseconds() > scan_recovery_after_ && scan_valid_ &&
    right_heading_valid_ && !odom_recovering_ && odom_age >= -0.1 && odom_age <= 0.15 &&
    std::chrono::duration<double>(receipt - last_odom_time_).count() <= 0.15 && scan_age >= -0.1 &&
    scan_age <= 0.2 && std::chrono::duration<double>(receipt - last_scan_time_).count() <= 0.2 &&
    last_scan_stamp_ns_ > scan_recovery_after_ && std::hypot(v.linear.x, v.linear.y) < 0.003 &&
    std::abs(v.angular.z) < 0.01;
  if (!ready) {
    scan_recovery_count_ = scan_recovery_odom_count_ = 0;
    scan_recovery_stamp_ = observation_stamp_ns_;
    scan_recovery_odom_stamp_ = stamp.nanoseconds();
    return true;
  }
  if (stamp.nanoseconds() > scan_recovery_odom_stamp_) {
    scan_recovery_odom_stamp_ = stamp.nanoseconds();
    if (scan_recovery_odom_count_++ == 0) scan_recovery_odom_since_ = receipt;
  }
  if (last_scan_stamp_ns_ <= scan_recovery_stamp_) return true;
  scan_recovery_stamp_ = last_scan_stamp_ns_;
  if (scan_recovery_count_++ == 0) scan_recovery_since_ = receipt;
  if (
    scan_recovery_count_ < 3 || scan_recovery_odom_count_ < 3 ||
    std::chrono::duration<double>(receipt - scan_recovery_odom_since_).count() < 0.3 ||
    std::chrono::duration<double>(receipt - scan_recovery_since_).count() < 0.3)
    return true;
  scan_recovering_ = false;
  // Rebuild heading evidence after the stopped recovery; retain original stage/total deadlines.
  wall_direction_window_.clear();
  right_heading_stable_ = false;
  reset_pid();
  RCLCPP_INFO(
    get_logger(),
    "FEEDBACK_RECOVERED: fresh stopped scan and odom; reacquiring wall heading and position");
  return true;
}
