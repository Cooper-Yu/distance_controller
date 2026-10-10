/** @file
 * @brief Multi-scan acquisition, fixed odom rotation, then independent wall verification. */
#include <cmath>

#include "distance_controller/distance_controller.hpp"

bool DistanceController::handle_frozen_alignment(const rclcpp::Time & current_time)
{
  const double yaw = quaternion_to_yaw(last_odom_.pose.pose.orientation);
  const double now =
    std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  if (!wall_target_frozen_) {
    publish_stop();
    if (!wall_direction_window_.estimate(now, frozen_wall_yaw_)) return true;
    wall_target_frozen_ = true;
    wall_direction_window_.clear();
    RCLCPP_INFO(
      get_logger(), "Wall heading target frozen: yaw=%.6f rad; odom alignment", frozen_wall_yaw_);
    return true;
  }
  const double error =
    std::atan2(std::sin(yaw - frozen_wall_yaw_), std::cos(yaw - frozen_wall_yaw_));
  if (std::abs(error) > heading_tolerance_) {
    wall_verifying_ = alignment_settling_ = false;
    wall_direction_window_.clear();
    publish_heading_correction(error);
    return true;
  }
  publish_stop();
  const auto & v = last_odom_.twist.twist;
  if (std::hypot(v.linear.x, v.linear.y) >= .01 || std::abs(v.angular.z) >= .02) {
    wall_verifying_ = alignment_settling_ = false;
    wall_direction_window_.clear();
    return true;
  }
  if (!alignment_settling_) {
    alignment_settling_ = true;
    alignment_settle_start_ = current_time;
  }
  if ((current_time - alignment_settle_start_).seconds() < alignment_settle_duration_) return true;
  if (!wall_verifying_) {
    wall_verifying_ = true;
    wall_direction_window_.clear();
    RCLCPP_INFO(get_logger(), "Wall heading verification: collecting new stopped scans");
    return true;
  }
  return verify_frozen_alignment(yaw);
}

bool DistanceController::verify_frozen_alignment(double yaw)
{
  const double now =
    std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  double direction = 0;
  if (!wall_direction_window_.estimate(now, direction)) return true;
  const double residual = std::atan2(std::sin(direction - yaw), std::cos(direction - yaw));
  RCLCPP_INFO(
    get_logger(), "Wall heading verification: residual=%.6f rad refinement=%u", residual,
    wall_refinements_);
  if (std::abs(residual) > heading_tolerance_) {
    if (wall_refinements_ >= 2) {
      fail_preparation("WALL_HEADING_VERIFICATION_FAILED", std::chrono::steady_clock::now());
      return true;
    }
    ++wall_refinements_;
    frozen_wall_yaw_ = direction;
    alignment_settling_ = wall_verifying_ = false;
    wall_direction_window_.clear();
    RCLCPP_INFO(get_logger(), "Wall heading target refined: yaw=%.6f rad", frozen_wall_yaw_);
    return true;
  }
  heading_reference_ = yaw;
  initial_alignment_complete_ = true;
  alignment_settling_ = false;
  reset_pid();
  RCLCPP_INFO(
    get_logger(),
    "Initial alignment complete: yaw=%.6f; multi-scan wall verified; starting laser centering next "
    "tick",
    yaw);
  return true;
}
