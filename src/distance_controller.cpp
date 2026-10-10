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
  if (scene_number != 1 && scene_number != 2) throw std::invalid_argument("Scene must be 1 or 2");
  adopt_current_pose_ = declare_parameter<bool>("adopt_current_pose", false);
  if (adopt_current_pose_ && scene_number != 2)
    throw std::invalid_argument("adopt_current_pose requires scene 2");
  heading_control_enabled_ = (scene_number == 2);
  configure_heading_control();
  base_command_watchdog_verified_ =
    declare_parameter<bool>("base_command_watchdog_verified", base_command_watchdog_verified_);

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
  if (
    scene_number == 1 &&
    ((overrides.count("manual_mode") && overrides.at("manual_mode").get<bool>()) ||
     (overrides.count("start_paused") && overrides.at("start_paused").get<bool>()))) {
    throw std::invalid_argument("Manual continuation is available only in scene 2");
  }
  select_waypoints(scene_number);
  const auto odom_topic = declare_parameter<std::string>("odom_topic", "/odometry/filtered");
  const auto cmd_topic = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  RCLCPP_INFO(
    get_logger(), "Scene %d: %zu configured segments, use_sim_time=%s, max_speed=%.3f",
    scene_number, segments_.size(), get_parameter("use_sim_time").as_bool() ? "true" : "false",
    max_speed_);

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, 10, std::bind(&DistanceController::on_odom, this, std::placeholders::_1));

  timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() { on_timer(); });

  cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_topic, 10);
  configure_centering();
  configure_step_interface();
}

void DistanceController::configure_heading_control()
{
  if (!heading_control_enabled_) return;
  heading_gain_ = declare_parameter<double>("heading_gain", 1.0);
  max_yaw_rate_ = declare_parameter<double>("max_yaw_rate", 0.25);
  heading_tolerance_ = declare_parameter<double>("heading_tolerance", 0.01);
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
  const auto steady_now = std::chrono::steady_clock::now();
  if (std::chrono::duration<double>(steady_now - heading_log_time_).count() >= 1.0) {
    heading_log_time_ = steady_now;
    RCLCPP_INFO(
      get_logger(), "%s-wall alignment: valid=%s stable=%s angle=%.3f deg rms=%.4f m span=%.3f m",
      alignment_wall_.c_str(), right_heading_valid_ ? "true" : "false",
      right_heading_stable_ ? "true" : "false", right_wall_angle_ * 180.0 / 3.141592653589793,
      right_wall_rms_, right_wall_span_);
  }
  if (robust_wall_heading_) return handle_frozen_alignment(current_time);
  if (!right_heading_valid_ || !right_heading_stable_) {
    alignment_settling_ = false;
    publish_stop();
    return true;
  }
  if (std::abs(right_wall_angle_) > heading_tolerance_) {
    alignment_settling_ = false;
    publish_heading_correction(-right_wall_angle_);
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
    heading_reference_ = yaw;
    initial_alignment_complete_ = true;
    alignment_settling_ = false;
    reset_pid();
    RCLCPP_INFO(
      get_logger(),
      "Initial alignment complete: yaw=%.6f; wall reference captured; starting laser "
      "centering next tick",
      yaw);
  }
  return true;
}

double DistanceController::heading_error(double yaw) const
{
  return std::atan2(std::sin(yaw - heading_reference_), std::cos(yaw - heading_reference_));
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
  yaw = heading_error(yaw);
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
    "Target: x=%.3f, y=%.3f, yaw=%.3f rad | "
    "Current segment index:%zu | "
    "Error: ex=%.3f, ey=%.3f | "
    "pid_dt=%.3f | "
    "integral=(%.3f, %.3f) | "
    "odom_raw=(%.3f, %.3f) | "
    "odom_limited=(%.3f, %.3f) | "
    "robot_cmd=(%.3f, %.3f), wz=%.3f",
    data.x, data.y, data.yaw, last_odom_.twist.twist.linear.x, last_odom_.twist.twist.linear.y,
    last_odom_.twist.twist.angular.z, target_x_, target_y_, heading_reference_,
    current_segment_index_, data.ex, data.ey, data.pid_dt, integral_x_, integral_y_,
    data.vx_odom_raw, data.vy_odom_raw, data.vx_odom, data.vy_odom, data.vx_robot, data.vy_robot,
    data.wz_robot);
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
  const auto receipt = std::chrono::steady_clock::now();
  if (heading_control_enabled_ && !validate_odom_recovery(*msg, receipt)) return;
  last_odom_ = *msg;

  last_odom_time_ = receipt;
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

  if (odom_recovering_) return handle_odom_recovery(now);
  double dt = std::chrono::duration<double>(now - last_odom_time_).count();
  if (dt > 0.5) {
    RCLCPP_WARN(get_logger(), "Odom timeout: %.2f seconds since last update", dt);
    if (heading_control_enabled_) {
      begin_odom_recovery(now);
      return true;
    }
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
  if (heading_control_enabled_) cmd.angular.z = compute_heading_hold_velocity(data.yaw);
  apply_front_speed_bound(cmd);
  if (
    segments_[current_segment_index_].completion ==
    distance_controller::CompletionKind::FrontWall) {
    data.vx_robot = cmd.linear.x;
    data.vy_robot = cmd.linear.y;
    data.vx_odom = std::cos(data.yaw) * cmd.linear.x - std::sin(data.yaw) * cmd.linear.y;
    data.vy_odom = std::sin(data.yaw) * cmd.linear.x + std::cos(data.yaw) * cmd.linear.y;
    previous_vx_odom_ = data.vx_odom;
    previous_vy_odom_ = data.vy_odom;
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

  if (handle_current_pose_start(current_time)) return;

  if (handle_centering_guard(now, should_log)) {
    return;
  }

  if (handle_initial_alignment(current_time)) {
    return;
  }

  if (handle_initial_positioning(current_time)) {
    return;
  }

  if (handle_manual_wait()) return;

  const auto result = execute_current_segment(current_time, should_log);
  if (result == SegmentResult::Completed) advance_route();
  if (result == SegmentResult::Failed) {
    fault_latched_ = true;
    publish_stop();
    RCLCPP_ERROR(get_logger(), "Route execution failed; no segment advancement");
  }
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

double DistanceController::quaternion_to_yaw(const geometry_msgs::msg::Quaternion & q)
{
  tf2::Quaternion quat(q.x, q.y, q.z, q.w);

  double roll, pitch, yaw;
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  return yaw;
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
  const double cap = std::min(max_speed_, segments_[current_segment_index_].motion.max_speed);
  if (speed > cap) {
    double scale = cap / speed;

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
    (heading_control_enabled_ && std::abs(heading_error(quaternion_to_yaw(
                                   last_odom_.pose.pose.orientation))) > heading_tolerance_)) {
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
      log_route_wall_observation();
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
