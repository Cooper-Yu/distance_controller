/** @file @brief Small stable-median translations with fixed odom targets and fresh revalidation. */
#include <algorithm>
#include <cmath>

#include "distance_controller/distance_controller.hpp"

void DistanceController::begin_precise_correction(const distance_controller::StartSummary & summary)
{
  const auto delta = distance_controller::start_adjustment(
    summary, right_target_distance_, rear_target_distance_, centering_tolerance_);
  if (!delta || std::hypot((*delta)[0], (*delta)[1]) == 0.0) {
    fail_preparation("START_CORRECTION_RANGE", std::chrono::steady_clock::now());
    return;
  }
  if (last_odom_.header.frame_id.empty() || last_odom_.child_frame_id.empty()) {
    fail_preparation("START_CORRECTION_ODOM_FRAME", std::chrono::steady_clock::now());
    return;
  }
  const auto & p = last_odom_.pose.pose.position;
  const double yaw = quaternion_to_yaw(last_odom_.pose.pose.orientation);
  precision_correction_ = distance_controller::StartCorrection{
    p.x,
    p.y,
    yaw,
    std::cos(yaw) * (*delta)[0] - std::sin(yaw) * (*delta)[1],
    std::sin(yaw) * (*delta)[0] + std::cos(yaw) * (*delta)[1],
    last_odom_.header.frame_id,
    last_odom_.child_frame_id};
  precision_correction_begin_ = std::chrono::steady_clock::now();
  reset_pid();
  publish_stop();
  RCLCPP_INFO(
    get_logger(),
    "START_CORRECTION: attempt=%u body_dx=%.6f body_dy=%.6f odom_target=(%.6f,%.6f) "
    "yaw=%.6f cap<=0.008 m/s heading_kp=1.8 wz_cap<=0.02 rad/s step<=0.010 m budget=4 s; require "
    "fresh stopped verification",
    precision_refinements_, (*delta)[0], (*delta)[1], p.x + precision_correction_->dx,
    p.y + precision_correction_->dy, yaw);
}

bool DistanceController::handle_precise_correction()
{
  const auto receipt = std::chrono::steady_clock::now();
  if (fault_latched_) {
    cancel_precise_correction("fault_latched");
    publish_stop();
    return true;
  }
  if (!scan_valid_ || !right_heading_valid_ || odom_recovering_ || scan_recovering_) {
    fail_preparation("START_CORRECTION_FEEDBACK", receipt);
    return true;
  }
  if (
    last_odom_.header.frame_id != precision_correction_->frame ||
    last_odom_.child_frame_id != precision_correction_->child_frame) {
    fail_preparation("START_CORRECTION_ODOM_FRAME", receipt);
    return true;
  }
  const auto & p = last_odom_.pose.pose.position;
  const double yaw = quaternion_to_yaw(last_odom_.pose.pose.orientation);
  const auto feedback = distance_controller::correction_feedback(
    *precision_correction_, p.x, p.y, yaw, heading_error(yaw), heading_tolerance_,
    std::chrono::duration<double>(receipt - precision_correction_begin_).count(),
    std::min(max_speed_, centering_max_speed_));
  if (feedback.fault) {
    fail_preparation(feedback.fault, receipt);
  } else if (feedback.arrived) {
    publish_stop();
    precision_correction_.reset();
    precision_samples_.clear();
    precision_stationary_ = precision_collecting_ = false;
    RCLCPP_INFO(get_logger(), "START_CORRECTION_STOP: odom residual<=0.0005 m; not P01 acceptance");
    // Start the unchanged 5-second stopped-hold/measurement budget after commanding zero.
    verify_precise_start();
  } else {
    geometry_msgs::msg::Twist command;
    command.linear.x = feedback.vx;
    command.linear.y = feedback.vy;
    command.angular.z = feedback.wz;
    cmd_pub_->publish(command);  // Fixed step-entry heading; deviation fault bounds stay unchanged.
  }
  return true;
}

void DistanceController::cancel_precise_correction(const char * reason)
{
  if (!precision_correction_) return;
  precision_correction_.reset();
  precision_reverify_required_ = true;
  precision_collecting_ = precision_stationary_ = false;
  precision_samples_.clear();
  publish_stop();
  RCLCPP_WARN(
    get_logger(),
    "START_CORRECTION_CANCELLED: reason=%s; discarded target, fresh verification required", reason);
}
