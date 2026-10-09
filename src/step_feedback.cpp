/** @file
 * @brief Per-step feedback selection with fail-closed laser goals and bounded execution.
 */
#include <algorithm>
#include <cmath>

#include "distance_controller/distance_controller.hpp"

void DistanceController::fail_step(const std::string & reason)
{
  fault_latched_ = true;
  settling_ = false;
  segment_completed_ = false;
  reset_pid();
  publish_stop();
  RCLCPP_ERROR(get_logger(), "Step fault: %s; route will not advance", reason.c_str());
}

bool DistanceController::apply_step_feedback(ControlDiagnostics & data)
{
  using namespace distance_controller;
  if (!heading_control_enabled_) return true;
  const auto & step = segments_[current_segment_index_];
  const auto clock_now = std::chrono::steady_clock::now();
  step_travel_ += std::hypot(data.x - step_last_x_, data.y - step_last_y_);
  step_last_x_ = data.x;
  step_last_y_ = data.y;
  if (
    std::chrono::duration<double>(clock_now - step_started_).count() > step.timeout ||
    step_travel_ > step.max_travel) {
    fail_step("TIME_OR_TRAVEL_LIMIT");
    return false;
  }
  const bool front = step.completion == CompletionKind::FrontWall;
  if (!front && !step.side_centering) return true;
  const double age = std::chrono::duration<double>(clock_now - wall_observation_time_).count();
  if (
    age > scan_timeout_ || (front && !front_wall_valid_) ||
    (step.side_centering && (!left_wall_valid_ || !right_wall_valid_))) {
    fail_step("REQUIRED_LASER_UNAVAILABLE");
    return false;
  }
  const double axes = step.frame == MotionFrame::Heading ? heading_reference_ : route_yaw_;
  double along = std::cos(axes) * data.ex + std::sin(axes) * data.ey;
  double lateral = -std::sin(axes) * data.ex + std::cos(axes) * data.ey;
  if (front) {
    const double clearance = front_wall_ - front_body_extent_;
    along = clearance - step.front_clearance;
    if (along < -0.01) {
      fail_step("FRONT_CLEARANCE_TOO_SMALL");
      return false;
    }
  }
  if (step.side_centering) lateral = (left_wall_ - right_wall_) * 0.5;
  data.ex = std::cos(axes) * along - std::sin(axes) * lateral;
  data.ey = std::sin(axes) * along + std::cos(axes) * lateral;
  return true;
}

void DistanceController::apply_front_speed_bound(geometry_msgs::msg::Twist & cmd)
{
  if (
    segments_[current_segment_index_].completion != distance_controller::CompletionKind::FrontWall)
    return;
  const double remaining =
    front_wall_ - front_body_extent_ - segments_[current_segment_index_].front_clearance;
  const double cap =
    std::min(centering_max_speed_, std::sqrt(2 * max_acceleration_ * std::max(0.0, remaining)));
  cmd.linear.x = std::clamp(cmd.linear.x, 0.0, cap);
}
