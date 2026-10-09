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
  const auto names = declare_parameter<std::vector<std::string>>("route", {"AB", "BC", "CB", "BA"});
  segments_ = distance_controller::compose_route(
    names, forward_distance, lateral_distance, max_speed_, dwell_duration_);
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
  }

  double total_dx = 0.0;
  double total_dy = 0.0;
  for (std::size_t i = 0; i <= current_segment_index_; ++i) {
    total_dx += segments_[i].motion.dx;
    total_dy += segments_[i].motion.dy;
  }
  target_x_ = route_x_ + std::cos(route_yaw_) * total_dx - std::sin(route_yaw_) * total_dy;

  target_y_ = route_y_ + std::sin(route_yaw_) * total_dx + std::cos(route_yaw_) * total_dy;

  RCLCPP_INFO(
    get_logger(), "Segment %zu/%zu target=(%.6f, %.6f)", current_segment_index_ + 1,
    segments_.size(), target_x_, target_y_);
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
  ++current_segment_index_;
  if (current_segment_index_ >= segments_.size()) {
    RCLCPP_INFO(get_logger(), "Route completed.");
    rclcpp::shutdown();
    return;
  }
  reset_pid();
  settling_ = false;
  segment_completed_ = false;
  target_initialized_ = false;
}
