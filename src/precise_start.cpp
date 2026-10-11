/** @file
 * @brief Stopped multi-frame verification for optional precise right/rear preparation. */
#include <cmath>
#include <limits>

#include "distance_controller/distance_controller.hpp"

bool DistanceController::verify_precise_start()
{
  publish_stop();
  const auto now = std::chrono::steady_clock::now();
  if (!precision_collecting_) {
    precision_collecting_ = true;
    precision_stationary_ = false;
    precision_begin_ = now;
    precision_samples_.clear();
    RCLCPP_INFO(get_logger(), "START_VERIFY: stopped hold then 15 fresh scans; 5 s budget");
  }
  if (std::chrono::duration<double>(now - precision_begin_).count() > 5.0) {
    fail_preparation("START_VERIFY_TIMEOUT", now);
    return true;
  }
  const auto & p = last_odom_.pose.pose.position;
  const auto & v = last_odom_.twist.twist;
  const double yaw = quaternion_to_yaw(last_odom_.pose.pose.orientation);
  const bool moved =
    std::hypot(p.x - precision_x_, p.y - precision_y_) > .003 ||
    std::abs(std::atan2(std::sin(yaw - precision_yaw_), std::cos(yaw - precision_yaw_))) > .005;
  if (
    std::hypot(v.linear.x, v.linear.y) >= .003 || std::abs(v.angular.z) >= .01 ||
    (precision_stationary_ && moved)) {
    precision_stationary_ = false;
    precision_samples_.clear();
    return true;
  }
  if (!precision_stationary_) {
    precision_stationary_ = true;
    precision_hold_ = now;
    precision_x_ = p.x;
    precision_y_ = p.y;
    precision_yaw_ = yaw;
  }
  // Exclude pre-stop and pre-hold scans even if the timer runs faster than the sensor.
  if (std::chrono::duration<double>(now - precision_hold_).count() < .5) {
    precision_stamp_ = this->now().nanoseconds();
    return true;
  }
  if (observation_stamp_ns_ <= precision_stamp_) return true;
  precision_stamp_ = observation_stamp_ns_;
  precision_samples_.push_back(
    {right_wall_, rear_wall_,
     right_heading_valid_ ? right_wall_angle_ : std::numeric_limits<double>::quiet_NaN()});
  if (precision_samples_.size() == 15) {
    const auto summary = distance_controller::summarize_start(precision_samples_);
    precision_samples_.clear();
    finish_precise_batch(summary);
  }
  return true;
}

void DistanceController::finish_precise_batch(const distance_controller::StartSummary & summary)
{
  RCLCPP_INFO(
    get_logger(),
    "START_VERIFY: retained=%u/15 stable=%s right=%.6f rear=%.6f angle=%.6f "
    "spread_right=%.6f spread_rear=%.6f spread_angle=%.6f",
    summary.retained, summary.stable ? "true" : "false", summary.median[0], summary.median[1],
    summary.median[2], summary.spread[0], summary.spread[1], summary.spread[2]);
  if (!summary.stable) return;  // Remain stopped; never move based on an unstable batch.
  const bool distances_ok =
    std::abs(summary.median[0] - right_target_distance_) <= centering_tolerance_ &&
    std::abs(summary.median[1] - rear_target_distance_) <= centering_tolerance_;
  const bool heading_ok = std::abs(summary.median[2]) <= heading_tolerance_;
  if (distances_ok && heading_ok) {
    RCLCPP_INFO(
      get_logger(), "START_VERIFIED: median distances within %.4f m; wall angle within %.6f rad",
      centering_tolerance_, heading_tolerance_);
    record_route_origin();
    return;
  }
  if (++precision_refinements_ > 3) {
    fail_preparation("START_REFINEMENT_LIMIT", std::chrono::steady_clock::now());
    return;
  }
  precision_collecting_ = false;
  if (!heading_ok) {
    initial_alignment_complete_ = false;
    wall_target_frozen_ = wall_verifying_ = alignment_settling_ = false;
    wall_direction_window_.clear();
  }
  RCLCPP_INFO(
    get_logger(), "START_REFINE: attempt=%u; recheck after adjustment", precision_refinements_);
}
