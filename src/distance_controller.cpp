/** @file
 * @brief DistanceController implementation; interface documentation is in the header.
 */
#include "distance_controller/distance_controller.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <string>

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

DistanceController::DistanceController(int scene_number) : Node{"distance_controller"}
{
  select_waypoints(scene_number);
  heading_control_enabled_ = (scene_number == 2);
  configure_heading_control();

  const auto & overrides = get_node_parameters_interface()->get_parameter_overrides();
  if (overrides.find("use_sim_time") == overrides.end()) {
    set_parameter(rclcpp::Parameter("use_sim_time", scene_number == 1));
  }

  kp_ = declare_parameter<double>("kp", 1.5);
  ki_ = declare_parameter<double>("ki", 0.0);
  kd_ = declare_parameter<double>("kd", 0.0);
  max_speed_ = declare_parameter<double>("max_speed", scene_number == 1 ? 0.40 : 0.10);
  max_acceleration_ =
    declare_parameter<double>("max_acceleration", scene_number == 1 ? 0.60 : 0.20);
  dwell_duration_ = declare_parameter<double>("dwell_duration", 1.0);
  if (
    !std::isfinite(kp_) || !std::isfinite(ki_) || !std::isfinite(kd_) ||
    !std::isfinite(max_speed_) || max_speed_ <= 0.0 || !std::isfinite(max_acceleration_) ||
    max_acceleration_ <= 0.0 || !std::isfinite(dwell_duration_) || dwell_duration_ < 0.0) {
    throw std::invalid_argument("Invalid controller parameters");
  }
  const auto odom_topic = declare_parameter<std::string>("odom_topic", "/odometry/filtered");
  const auto cmd_topic = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  RCLCPP_INFO(
    get_logger(), "Scene %d: %zu segments, use_sim_time=%s, max_speed=%.3f", scene_number,
    segments_.size(), get_parameter("use_sim_time").as_bool() ? "true" : "false", max_speed_);

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, 10, std::bind(&DistanceController::on_odom, this, std::placeholders::_1));

  timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() { on_timer(); });

  cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_topic, 10);
  configure_centering();
}

void DistanceController::configure_heading_control()
{
  if (!heading_control_enabled_) return;
  heading_gain_ = declare_parameter<double>("heading_gain", 1.0);
  max_yaw_rate_ = declare_parameter<double>("max_yaw_rate", 0.25);
  heading_tolerance_ = declare_parameter<double>("heading_tolerance", 0.02);
  translation_pause_angle_ = declare_parameter<double>("translation_pause_angle", 0.15);
  alignment_settle_duration_ = declare_parameter<double>("alignment_settle_duration", 0.5);
  if (
    !std::isfinite(heading_gain_) || heading_gain_ <= 0.0 || !std::isfinite(max_yaw_rate_) ||
    max_yaw_rate_ <= 0.0 || !std::isfinite(heading_tolerance_) || heading_tolerance_ <= 0.0 ||
    !std::isfinite(translation_pause_angle_) || translation_pause_angle_ <= heading_tolerance_ ||
    translation_pause_angle_ >= 3.141592653589793 || !std::isfinite(alignment_settle_duration_) ||
    alignment_settle_duration_ <= 0.0) {
    throw std::invalid_argument(
      "Invalid heading parameters: positive finite values and tolerance < pause angle < pi "
      "required");
  }
}

bool DistanceController::handle_initial_alignment(const rclcpp::Time & current_time)
{
  if (!heading_control_enabled_ || initial_alignment_complete_) return false;
  const double yaw = quaternion_to_yaw(last_odom_.pose.pose.orientation);
  if (std::abs(yaw) > heading_tolerance_) {
    alignment_settling_ = false;
    publish_heading_correction(yaw);
    return true;
  }
  publish_stop();
  const auto & velocity = last_odom_.twist.twist;
  if (
    std::hypot(velocity.linear.x, velocity.linear.y) >= 0.01 ||
    std::abs(velocity.angular.z) >= 0.02) {
    alignment_settling_ = false;
    return true;
  }
  if (!alignment_settling_) {
    alignment_settle_start_ = current_time;
    alignment_settling_ = true;
  } else if ((current_time - alignment_settle_start_).seconds() >= alignment_settle_duration_) {
    initial_alignment_complete_ = true;
    alignment_settling_ = false;
    reset_pid();
    RCLCPP_INFO(
      get_logger(), "Initial alignment complete: yaw=%.6f; starting laser centering next tick",
      yaw);
  }
  return true;
}

double DistanceController::compute_heading_command(double yaw, double gain, double max_yaw_rate)
{
  const double error = std::atan2(std::sin(-yaw), std::cos(-yaw));
  return std::clamp(gain * error, -max_yaw_rate, max_yaw_rate);
}

void DistanceController::publish_heading_correction(double yaw)
{
  reset_pid();
  previous_vx_odom_ = 0.0;
  previous_vy_odom_ = 0.0;
  geometry_msgs::msg::Twist cmd;
  cmd.angular.z = compute_heading_command(yaw, heading_gain_, max_yaw_rate_);
  cmd_pub_->publish(cmd);
}

bool DistanceController::handle_heading_recovery(double yaw, double position_error)
{
  if (!heading_control_enabled_) return false;
  const bool heading_outside = std::abs(yaw) > heading_tolerance_;
  if (segment_completed_ && (heading_outside || position_error >= 0.01)) {
    segment_completed_ = false;
    settling_ = false;
    reset_pid();
    RCLCPP_INFO(get_logger(), "Dwell interrupted: reacquiring position and heading");
  }
  if (heading_outside && (position_error < 0.01 || std::abs(yaw) > translation_pause_angle_)) {
    settling_ = false;
    publish_heading_correction(yaw);
    return true;
  }
  return false;
}

void DistanceController::log_control_state(const ControlDiagnostics & data)
{
  RCLCPP_INFO(
    get_logger(),
    "Pose: x=%.3f, y=%.3f, yaw=%.3f | "
    "Velocity: vx=%.3f, vy=%.3f, wz=%.3f | "
    "Target: x=%.3f, y=%.3f | "
    "Current segment index:%zu | "
    "Error: ex=%.3f, ey=%.3f | "
    "pid_dt=%.3f | "
    "integral=(%.3f, %.3f) | "
    "odom_raw=(%.3f, %.3f) | "
    "odom_limited=(%.3f, %.3f) | "
    "robot_cmd=(%.3f, %.3f), wz=%.3f",
    data.x, data.y, data.yaw, last_odom_.twist.twist.linear.x, last_odom_.twist.twist.linear.y,
    last_odom_.twist.twist.angular.z, target_x_, target_y_, current_segment_index_, data.ex,
    data.ey, data.pid_dt, integral_x_, integral_y_, data.vx_odom_raw, data.vy_odom_raw,
    data.vx_odom, data.vy_odom, data.vx_robot, data.vy_robot, data.wz_robot);
}

void DistanceController::on_odom(nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (heading_control_enabled_) {
    const auto & pose = msg->pose.pose;
    const auto & q = pose.orientation;
    const auto & v = msg->twist.twist;
    const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if (
      !std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) || !std::isfinite(norm) ||
      std::abs(norm - 1.0) > 0.01 || !std::isfinite(v.linear.x) || !std::isfinite(v.linear.y) ||
      !std::isfinite(v.angular.z)) {
      fault_latched_ = true;
      reset_pid();
      publish_stop();
      RCLCPP_ERROR(get_logger(), "Invalid odometry: heading control fault latched");
      return;
    }
  }
  last_odom_ = *msg;

  last_odom_time_ = std::chrono::steady_clock::now();
  received_odom_ = true;
}

bool DistanceController::handle_odom_wait_or_timeout(
  const std::chrono::steady_clock::time_point & now, bool should_log)
{
  if (!received_odom_) {
    if (should_log) {
      RCLCPP_INFO(get_logger(), "Waiting for odom...");
    }
    publish_stop();
    return true;
  }

  double dt = std::chrono::duration<double>(now - last_odom_time_).count();
  if (dt > 0.5) {
    RCLCPP_WARN(get_logger(), "Odom timeout: %.2f seconds since last update", dt);
    fault_latched_ = true;
    reset_pid();
    publish_stop();
    return true;
  }

  return false;
}

bool DistanceController::handle_ros_time_jump(const rclcpp::Time & current_time)
{
  if (!ros_time_initialized_) {
    last_ros_time_ = current_time;
    ros_time_initialized_ = true;
  } else {
    const double ros_dt = (current_time - last_ros_time_).seconds();

    last_ros_time_ = current_time;

    if (heading_control_enabled_ && ros_dt == 0.0) {
      publish_stop();
      return true;
    }
    if (ros_dt < 0) {
      fault_latched_ = true;
      settling_ = false;
      reset_pid();

      RCLCPP_ERROR(get_logger(), "ROS time moved backwards: ros_dt=%.6f", ros_dt);

      publish_stop();
      return true;
    }

    if (ros_dt > 0.2) {
      alignment_settling_ = false;
      centering_settling_ = false;
      if (segment_completed_) {
        dwell_start_time_ = current_time;
      }
      settling_ = false;
      reset_pid();

      publish_stop();
      return true;
    }
  }

  return false;
}

bool DistanceController::handle_pid_timing(
  double ex, double ey, const rclcpp::Time & current_time, double & pid_dt)
{
  if (!pid_initialized_) {
    initialize_pid(ex, ey, current_time);
    publish_stop();
    return true;
  }

  pid_dt = (current_time - last_pid_time_).seconds();

  if (pid_dt == 0.0) {
    publish_stop();
    return true;
  }

  if (pid_dt < 0.0) {
    reset_pid();
    publish_stop();
    return true;
  }

  if (pid_dt > 0.2) {
    reset_pid();
    publish_stop();
    return true;
  }

  last_pid_time_ = current_time;
  return false;
}

void DistanceController::compute_and_publish_command(ControlDiagnostics & data)
{
  compute_pid(data.ex, data.ey, data.pid_dt, data.vx_odom, data.vy_odom);

  data.vx_odom_raw = data.vx_odom;
  data.vy_odom_raw = data.vy_odom;

  limit_velocity(data.vx_odom, data.vy_odom);
  limit_acceleration(data.vx_odom, data.vy_odom, data.pid_dt);

  double vx_robot = 0.0;
  double vy_robot = 0.0;

  odom_to_robot_velocity(data.vx_odom, data.vy_odom, data.yaw, vx_robot, vy_robot);
  data.vx_robot = vx_robot;
  data.vy_robot = vy_robot;

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = vx_robot;
  cmd.linear.y = vy_robot;
  if (heading_control_enabled_ && std::abs(data.yaw) > heading_tolerance_) {
    cmd.angular.z = compute_heading_command(data.yaw, heading_gain_, max_yaw_rate_);
  }
  data.wz_robot = cmd.angular.z;

  cmd_pub_->publish(cmd);
}

void DistanceController::on_timer()
{
  if (fault_latched_) {
    publish_stop();
    return;
  }

  auto now = std::chrono::steady_clock::now();

  bool should_log = std::chrono::duration<double>(now - last_log_time_).count() >= 1.0;

  if (should_log) {
    last_log_time_ = now;
  }

  if (handle_odom_wait_or_timeout(now, should_log)) {
    return;
  }

  auto current_time = this->now();

  if (handle_ros_time_jump(current_time)) {
    return;
  }

  if (handle_centering_guard(now, should_log)) {
    return;
  }

  if (handle_initial_alignment(current_time)) {
    return;
  }

  if (handle_initial_centering(current_time)) {
    return;
  }

  if (!target_initialized_) {
    initialize_segment_target();
  }

  if (!target_initialized_) {
    publish_stop();
    return;
  }

  double x = last_odom_.pose.pose.position.x;
  double y = last_odom_.pose.pose.position.y;

  double yaw = quaternion_to_yaw(last_odom_.pose.pose.orientation);

  double ex = target_x_ - x;
  double ey = target_y_ - y;

  const double position_error = std::hypot(ex, ey);
  if (handle_heading_recovery(yaw, position_error)) {
    return;
  }

  if (handle_completed_segment(current_time)) {
    return;
  }

  if (position_error < 0.01) {
    publish_stop();
    reset_pid();
    check_completion(ex, ey, current_time);

    return;
  }

  settling_ = false;

  double pid_dt = 0.0;
  if (handle_pid_timing(ex, ey, current_time, pid_dt)) {
    return;
  }

  ControlDiagnostics data{};
  data.x = x;
  data.y = y;
  data.yaw = yaw;
  data.ex = ex;
  data.ey = ey;
  data.pid_dt = pid_dt;

  compute_and_publish_command(data);

  if (should_log) {
    log_control_state(data);
  }
}

bool DistanceController::handle_completed_segment(const rclcpp::Time & current_time)
{
  if (!segment_completed_) {
    return false;
  }

  publish_stop();
  const double dwell_elapsed = (current_time - dwell_start_time_).seconds();

  if (dwell_elapsed < dwell_duration_) {
    return true;
  }
  ++current_segment_index_;
  if (current_segment_index_ >= segments_.size()) {
    RCLCPP_INFO(get_logger(), "Route completed.");
    rclcpp::shutdown();
    return true;
  }

  reset_pid();
  settling_ = false;
  segment_completed_ = false;
  target_initialized_ = false;
  return true;
}

void DistanceController::publish_stop()
{
  previous_vx_odom_ = 0.0;
  previous_vy_odom_ = 0.0;
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 0.0;
  cmd.linear.y = 0.0;
  cmd.linear.z = 0.0;

  cmd.angular.x = 0.0;
  cmd.angular.y = 0.0;
  cmd.angular.z = 0.0;

  cmd_pub_->publish(cmd);
}

void DistanceController::select_waypoints(int scene_number)
{
  if (scene_number == 1) {
    segments_ = {{0.0, 1.0},   {0.0, -1.0}, {0.0, -1.0}, {0.0, 1.0}, {1.0, 1.0},
                 {-1.0, -1.0}, {1.0, -1.0}, {-1.0, 1.0}, {1.0, 0.0}, {-1.0, 0.0}};
  } else if (scene_number == 2) {
    const double forward_distance = declare_parameter<double>("forward_distance", 0.90);
    const double lateral_distance = declare_parameter<double>("lateral_distance", 0.516780);

    if (!std::isfinite(forward_distance)) {
      throw std::invalid_argument("forward_distance must be finite");
    }
    if (forward_distance <= 0.0) {
      throw std::invalid_argument("forward_distance must be positive");
    }

    if (!std::isfinite(lateral_distance)) {
      throw std::invalid_argument("lateral_distance must be finite");
    }
    if (lateral_distance <= 0.0) {
      throw std::invalid_argument("lateral_distance must be positive");
    }

    segments_ = {
      {forward_distance, 0.0},
      {0.0, -lateral_distance},
      {0.0, lateral_distance},
      {-forward_distance, 0.0}};
  } else {
    throw std::invalid_argument("Scene must be 1 (simulation) or 2 (CyberWorld)");
  }
  for (const auto & segment : segments_) {
    if (!std::isfinite(segment.dx) || !std::isfinite(segment.dy)) {
      throw std::invalid_argument(
        "CyberWorld waypoints are not configured; fill all four {dx, dy} rows before running "
        "scene 2");
    }
  }
}

double DistanceController::quaternion_to_yaw(const geometry_msgs::msg::Quaternion & q)
{
  tf2::Quaternion quat(q.x, q.y, q.z, q.w);

  double roll, pitch, yaw;
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  return yaw;
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
    route_yaw_ =
      heading_control_enabled_ ? 0.0 : quaternion_to_yaw(last_odom_.pose.pose.orientation);
    route_initialized_ = true;
  }

  double total_dx = 0.0;
  double total_dy = 0.0;
  for (std::size_t i = 0; i <= current_segment_index_; ++i) {
    total_dx += segments_[i].dx;
    total_dy += segments_[i].dy;
  }
  target_x_ = route_x_ + std::cos(route_yaw_) * total_dx - std::sin(route_yaw_) * total_dy;

  target_y_ = route_y_ + std::sin(route_yaw_) * total_dx + std::cos(route_yaw_) * total_dy;

  RCLCPP_INFO(
    get_logger(), "Segment %zu/%zu target=(%.6f, %.6f)", current_segment_index_ + 1,
    segments_.size(), target_x_, target_y_);
  target_initialized_ = true;
}

void DistanceController::reset_pid()
{
  integral_x_ = 0.0;
  integral_y_ = 0.0;
  prev_error_x_ = 0.0;
  prev_error_y_ = 0.0;

  pid_initialized_ = false;
}

void DistanceController::compute_pid(
  double ex, double ey, double dt, double & vx_odom, double & vy_odom)
{
  integral_x_ += ex * dt;
  integral_y_ += ey * dt;

  integral_x_ = std::clamp(integral_x_, -integral_limit_, integral_limit_);
  integral_y_ = std::clamp(integral_y_, -integral_limit_, integral_limit_);

  double derivative_x = (ex - prev_error_x_) / dt;
  double derivative_y = (ey - prev_error_y_) / dt;

  vx_odom = kp_ * ex + ki_ * integral_x_ + kd_ * derivative_x;

  vy_odom = kp_ * ey + ki_ * integral_y_ + kd_ * derivative_y;

  prev_error_x_ = ex;
  prev_error_y_ = ey;
}

void DistanceController::limit_velocity(double & vx, double & vy)
{
  double speed = std::sqrt(vx * vx + vy * vy);
  if (speed > max_speed_) {
    double scale = max_speed_ / speed;

    vx *= scale;
    vy *= scale;
  }
}

void DistanceController::limit_acceleration(double & vx, double & vy, double dt)
{
  const double dx = vx - previous_vx_odom_;
  const double dy = vy - previous_vy_odom_;
  const double change = std::hypot(dx, dy);
  const double allowed = max_acceleration_ * dt;
  if (change > allowed) {
    const double scale = allowed / change;
    vx = previous_vx_odom_ + dx * scale;
    vy = previous_vy_odom_ + dy * scale;
  }
  previous_vx_odom_ = vx;
  previous_vy_odom_ = vy;
}

void DistanceController::odom_to_robot_velocity(
  double vx_odom, double vy_odom, double yaw, double & vx_robot, double & vy_robot)
{
  vx_robot = std::cos(yaw) * vx_odom + std::sin(yaw) * vy_odom;

  vy_robot = -std::sin(yaw) * vx_odom + std::cos(yaw) * vy_odom;
}

void DistanceController::initialize_pid(double ex, double ey, const rclcpp::Time & current_time)
{
  prev_error_x_ = ex;
  prev_error_y_ = ey;

  last_pid_time_ = current_time;

  integral_x_ = 0.0;
  integral_y_ = 0.0;

  pid_initialized_ = true;
}

void DistanceController::check_completion(double ex, double ey, const rclcpp::Time & current_time)
{
  double position_error = std::sqrt(ex * ex + ey * ey);

  if (
    position_error >= 0.01 ||
    (heading_control_enabled_ &&
     std::abs(quaternion_to_yaw(last_odom_.pose.pose.orientation)) > heading_tolerance_)) {
    settling_ = false;
    return;
  }

  double vx = last_odom_.twist.twist.linear.x;
  double vy = last_odom_.twist.twist.linear.y;

  double linear_speed = std::sqrt(vx * vx + vy * vy);

  double angular_speed = std::abs(last_odom_.twist.twist.angular.z);

  if (linear_speed < 0.01 && angular_speed < 0.02) {
    if (!settling_) {
      settle_start_time_ = current_time;

      settling_ = true;
      RCLCPP_INFO(get_logger(), "Entering SETTLING state");
    } else if ((current_time - settle_start_time_).seconds() >= 0.5) {
      segment_completed_ = true;
      dwell_start_time_ = current_time;
      RCLCPP_INFO(
        get_logger(), "Entering DONE state | segment=%zu error=%.6f speed=%.6f wz=%.6f sim=%.3f",
        current_segment_index_, position_error, linear_speed, angular_speed,
        current_time.seconds());
    }
  } else {
    settling_ = false;
  }
}
