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
    precision_batch_log_ = {};
    precision_samples_.clear();
    precision_diagnostics_ = {};
    ++precision_diagnostic_cycle_;
    RCLCPP_INFO(get_logger(), "START_VERIFY: stopped hold then 15 fresh scans; 5 s budget");
  }
  if (std::chrono::duration<double>(now - precision_begin_).count() > 5.0) {
    log_precise_diagnostics("timeout", now);
    fail_preparation("START_VERIFY_TIMEOUT", now);
    return true;
  }
  const auto & p = last_odom_.pose.pose.position;
  const auto & v = last_odom_.twist.twist;
  const double yaw = quaternion_to_yaw(last_odom_.pose.pose.orientation);
  const bool moved =
    std::hypot(p.x - precision_x_, p.y - precision_y_) > .003 ||
    std::abs(std::atan2(std::sin(yaw - precision_yaw_), std::cos(yaw - precision_yaw_))) > .005;
  const unsigned stop_mask =
    (std::hypot(v.linear.x, v.linear.y) >= .003 ? distance_controller::StartDiagnostics::LinearSpeed
                                                : 0u) |
    (std::abs(v.angular.z) >= .01 ? distance_controller::StartDiagnostics::AngularSpeed : 0u) |
    (precision_stationary_ && moved ? distance_controller::StartDiagnostics::AnchorMotion : 0u);
  precision_diagnostics_.observe_stop(stop_mask, precision_samples_.size());
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
    ++precision_diagnostics_.hold_ticks;
    precision_stamp_ = this->now().nanoseconds();
    return true;
  }
  if (observation_stamp_ns_ <= precision_stamp_) {
    ++precision_diagnostics_.no_new_scan_ticks;
    return true;
  }
  precision_stamp_ = observation_stamp_ns_;
  distance_controller::append_start_sample(
    precision_samples_,
    {right_wall_, rear_wall_,
     right_heading_valid_ ? right_wall_angle_ : std::numeric_limits<double>::quiet_NaN()});
  ++precision_diagnostics_.accepted;
  if (precision_samples_.size() == 15) {
    const auto summary = distance_controller::summarize_start(precision_samples_);
    finish_precise_batch(summary);
  }
  return true;
}

void DistanceController::finish_precise_batch(const distance_controller::StartSummary & summary)
{
  precision_diagnostics_.observe_batch(summary);
  const auto receipt = std::chrono::steady_clock::now();
  if (
    summary.stable || std::chrono::duration<double>(receipt - precision_batch_log_).count() >= .5) {
    precision_batch_log_ = receipt;
    RCLCPP_INFO(
      get_logger(),
      "START_VERIFY: retained=%u/15 stable=%s right=%.6f rear=%.6f angle=%.6f "
      "spread_right=%.6f spread_rear=%.6f spread_angle=%.6f",
      summary.retained, summary.stable ? "true" : "false", summary.median[0], summary.median[1],
      summary.median[2], summary.spread[0], summary.spread[1], summary.spread[2]);
  }
  if (!summary.stable) return;  // Remain stopped; never move based on an unstable batch.
  const bool distances_ok =
    std::abs(summary.median[0] - right_target_distance_) <= centering_tolerance_ &&
    std::abs(summary.median[1] - rear_target_distance_) <= centering_tolerance_;
  const bool heading_ok = std::abs(summary.median[2]) <= heading_tolerance_;
  if (distances_ok && heading_ok) {
    log_precise_diagnostics("verified", std::chrono::steady_clock::now());
    RCLCPP_INFO(
      get_logger(), "START_VERIFIED: median distances within %.4f m; wall angle within %.6f rad",
      centering_tolerance_, heading_tolerance_);
    record_route_origin();
    return;
  }
  if (++precision_refinements_ > 3) {
    log_precise_diagnostics("refinement_limit", std::chrono::steady_clock::now());
    fail_preparation("START_REFINEMENT_LIMIT", std::chrono::steady_clock::now());
    return;
  }
  precision_collecting_ = false;
  log_precise_diagnostics("refine", std::chrono::steady_clock::now());
  precision_samples_.clear();
  precision_stationary_ = false;
  precision_reverify_required_ = true;
  if (!heading_ok) {
    // A bounded precise-start retry begins a new alignment cycle. Scan recovery
    // alone must not refresh this budget; the total preparation deadline remains.
    wall_refinements_ = 0;
    initial_alignment_complete_ = false;
    wall_target_frozen_ = wall_verifying_ = alignment_settling_ = false;
    wall_direction_window_.clear();
  } else {
    begin_precise_correction(summary);
  }
  RCLCPP_INFO(
    get_logger(), "START_REFINE: attempt=%u; recheck after adjustment", precision_refinements_);
}

void DistanceController::log_precise_diagnostics(
  const char * phase, const std::chrono::steady_clock::time_point & receipt)
{
  const auto & d = precision_diagnostics_;
  const double ros_now = now().seconds();
  const auto & v = last_odom_.twist.twist;
  const rclcpp::Time odom_stamp(last_odom_.header.stamp, get_clock()->get_clock_type());
  RCLCPP_INFO(
    get_logger(),
    "START_DIAGNOSTIC: phase=%s cycle=%u elapsed=%.3f sample_count=%zu accepted=%u batches=%u "
    "stationary=%s reset_events=%u linear_resets=%u angular_resets=%u anchor_resets=%u "
    "last_stop_mask=%u discarded=%zu blocked_ticks=%u hold_ticks=%u no_new_scan_ticks=%u "
    "scan_resets=%u odom_resets=%u vx=%.6f vy=%.6f wz=%.6f "
    "odom_source_age=%.6f odom_receipt_age=%.6f scan_source_age=%.6f scan_receipt_age=%.6f",
    phase, precision_diagnostic_cycle_,
    std::chrono::duration<double>(receipt - precision_begin_).count(), precision_samples_.size(),
    d.accepted, d.batches, precision_stationary_ ? "true" : "false", d.reset_events,
    d.linear_resets, d.angular_resets, d.anchor_resets, d.last_stop_mask, d.discarded,
    d.blocked_ticks, d.hold_ticks, d.no_new_scan_ticks, d.scan_resets, d.odom_resets, v.linear.x,
    v.linear.y, v.angular.z, ros_now - odom_stamp.seconds(),
    std::chrono::duration<double>(receipt - last_odom_time_).count(),
    ros_now - observation_stamp_ns_ * 1e-9,
    std::chrono::duration<double>(receipt - last_scan_time_).count());
  if (!d.batches) return;
  const auto & b = d.last_batch;
  RCLCPP_INFO(
    get_logger(),
    "START_DIAGNOSTIC_LAST_BATCH: phase=%s cycle=%u retained=%u/15 stable=%s "
    "right=%.6f rear=%.6f angle=%.6f spread_right=%.6f spread_rear=%.6f spread_angle=%.6f",
    phase, precision_diagnostic_cycle_, b.retained, b.stable ? "true" : "false", b.median[0],
    b.median[1], b.median[2], b.spread[0], b.spread[1], b.spread[2]);
}
