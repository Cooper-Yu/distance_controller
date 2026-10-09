/** @file
 * @brief Nonblocking segment execution and route advancement using shared control helpers.
 */
#include <cmath>
#include <stdexcept>

#include "distance_controller/distance_controller.hpp"

void DistanceController::select_waypoints(int scene_number)
{
  if (scene_number == 1) {
    const std::vector<std::pair<double, double>> offsets = {
      {0.0, 1.0},   {0.0, -1.0}, {0.0, -1.0}, {0.0, 1.0}, {1.0, 1.0},
      {-1.0, -1.0}, {1.0, -1.0}, {-1.0, 1.0}, {1.0, 0.0}, {-1.0, 0.0}};
    for (std::size_t i = 0; i < offsets.size(); ++i) {
      segments_.push_back(
        {"P" + std::to_string(i), "P" + std::to_string(i + 1),
         distance_controller::PlanarMotion::move_relative(
           offsets[i].first, offsets[i].second, max_speed_, dwell_duration_)});
    }
    return;
  }
  const double forward_distance = declare_parameter<double>("forward_distance", 0.90);
  const double lateral_distance = declare_parameter<double>("lateral_distance", 0.516780);
  configure_route_steps(forward_distance, lateral_distance);
}

void DistanceController::log_route_waypoints() const
{
  double x = route_x_;
  double y = route_y_;
  RCLCPP_INFO(
    get_logger(), "Waypoints: frame=%s, position=m, yaw=rad; fixed position goals",
    last_odom_.header.frame_id.c_str());
  RCLCPP_INFO(
    get_logger(), "Waypoint 0 %s: x=%.6f y=%.6f yaw=%.6f (held heading)",
    segments_.front().from.c_str(), x, y, heading_reference_);
  for (std::size_t i = 0; i < segments_.size(); ++i) {
    const auto & step = segments_[i];
    if (
      step.relative_to_start || step.side_centering ||
      step.completion != distance_controller::CompletionKind::Position) {
      RCLCPP_INFO(
        get_logger(),
        "Waypoint preview stops before %s->%s: feedback-dependent endpoint; "
        "see execution target and reached pose",
        step.from.c_str(), step.to.c_str());
      break;
    }
    const double axes =
      step.frame == distance_controller::MotionFrame::Heading ? heading_reference_ : route_yaw_;
    if (step.frame == distance_controller::MotionFrame::Odom) {
      x = step.motion.dx;
      y = step.motion.dy;
    } else {
      x += std::cos(axes) * step.motion.dx - std::sin(axes) * step.motion.dy;
      y += std::sin(axes) * step.motion.dx + std::cos(axes) * step.motion.dy;
    }
    RCLCPP_INFO(
      get_logger(), "Waypoint %zu %s: x=%.6f y=%.6f yaw=%.6f (held heading)", i + 1,
      step.to.c_str(), x, y, heading_reference_);
  }
}

void DistanceController::initialize_segment_target()
{
  if (current_segment_index_ >= segments_.size()) {
    RCLCPP_WARN(get_logger(), "Segment index out of range");
    return;
  }

  if (!route_initialized_) {
    route_x_ = last_odom_.pose.pose.position.x;
    route_y_ = last_odom_.pose.pose.position.y;
    route_yaw_ = heading_control_enabled_ ? heading_reference_
                                          : quaternion_to_yaw(last_odom_.pose.pose.orientation);
    route_initialized_ = true;
    planned_x_ = route_x_;
    planned_y_ = route_y_;
  }

  const auto & step = segments_[current_segment_index_];
  if (step.reverse_of != 0 && !check_continuation_position()) return;
  step_start_pose_ = current_recorded_pose();
  const double anchor_x = step.relative_to_start ? last_odom_.pose.pose.position.x : planned_x_;
  const double anchor_y = step.relative_to_start ? last_odom_.pose.pose.position.y : planned_y_;
  const double axes =
    step.frame == distance_controller::MotionFrame::Heading ? heading_reference_ : route_yaw_;
  if (step.frame == distance_controller::MotionFrame::Odom) {
    target_x_ = step.motion.dx;
    target_y_ = step.motion.dy;
  } else {
    target_x_ = anchor_x + std::cos(axes) * step.motion.dx - std::sin(axes) * step.motion.dy;
    target_y_ = anchor_y + std::sin(axes) * step.motion.dx + std::cos(axes) * step.motion.dy;
  }
  step_started_ = std::chrono::steady_clock::now();
  step_travel_ = 0;
  step_last_x_ = last_odom_.pose.pose.position.x;
  step_last_y_ = last_odom_.pose.pose.position.y;

  RCLCPP_INFO(
    get_logger(), "Segment %zu/%zu target=(%.6f, %.6f), yaw=%.6f rad, heading_control=%s",
    current_segment_index_ + 1, segments_.size(), target_x_, target_y_, heading_reference_,
    heading_control_enabled_ ? "enabled" : "disabled");
  RCLCPP_INFO(
    get_logger(), "Route segment %s->%s: speed=%.3f m/s dwell=%.3f s",
    segments_[current_segment_index_].from.c_str(), segments_[current_segment_index_].to.c_str(),
    segments_[current_segment_index_].motion.max_speed,
    segments_[current_segment_index_].motion.dwell);
  target_initialized_ = true;
}

DistanceController::SegmentResult DistanceController::execute_current_segment(
  const rclcpp::Time & current_time, bool should_log)
{
  if (fault_latched_ || current_segment_index_ >= segments_.size()) {
    publish_stop();
    return SegmentResult::Failed;
  }
  if (!target_initialized_) initialize_segment_target();
  if (!target_initialized_) return SegmentResult::Failed;
  ControlDiagnostics data{};
  data.x = last_odom_.pose.pose.position.x;
  data.y = last_odom_.pose.pose.position.y;
  data.yaw = quaternion_to_yaw(last_odom_.pose.pose.orientation);
  data.ex = target_x_ - data.x;
  data.ey = target_y_ - data.y;
  if (!apply_step_feedback(data)) return SegmentResult::Failed;
  if (should_log) log_route_wall_observation();
  const double position_error = std::hypot(data.ex, data.ey);
  if (handle_heading_recovery(data.yaw, position_error)) return SegmentResult::Running;
  if (segment_completed_) {
    return handle_completed_segment(current_time) ? SegmentResult::Completed
                                                  : SegmentResult::Running;
  }
  if (position_error < 0.01) {
    publish_stop();
    reset_pid();
    check_completion(data.ex, data.ey, current_time);
    return SegmentResult::Running;
  }
  settling_ = false;
  if (handle_pid_timing(data.ex, data.ey, current_time, data.pid_dt)) return SegmentResult::Running;
  compute_and_publish_command(data);
  if (should_log) log_control_state(data);
  return SegmentResult::Running;
}

bool DistanceController::handle_completed_segment(const rclcpp::Time & current_time)
{
  if (!segment_completed_) return false;
  publish_stop();
  return (current_time - dwell_start_time_).seconds() >=
         segments_[current_segment_index_].motion.dwell;
}

void DistanceController::advance_route()
{
  publish_stop();
  if (!record_completed_history()) return;
  record_step_endpoint();
  const auto actual = current_recorded_pose();
  RCLCPP_INFO(
    get_logger(),
    "Reached %s via %s->%s | actual: x=%.6f y=%.6f yaw=%.6f rad | "
    "target: x=%.6f y=%.6f yaw=%.6f rad | stopped and dwell complete",
    segments_[current_segment_index_].to.c_str(), segments_[current_segment_index_].from.c_str(),
    segments_[current_segment_index_].to.c_str(), actual.x, actual.y, actual.yaw, target_x_,
    target_y_, heading_reference_);
  ++current_segment_index_;
  reset_pid();
  settling_ = false;
  segment_completed_ = false;
  target_initialized_ = false;
  if (manual_mode_) {
    if (return_remaining_ > 0) --return_remaining_;
    manual_waiting_ = return_remaining_ == 0;
    if (!manual_waiting_) return;
    RCLCPP_INFO(get_logger(), "Manual WAITING: %s", step_status().c_str());
  } else if (current_segment_index_ >= segments_.size()) {
    RCLCPP_INFO(get_logger(), "Route completed.");
    rclcpp::shutdown();
  }
}
