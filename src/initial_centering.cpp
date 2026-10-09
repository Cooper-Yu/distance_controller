/** @file
 * @brief Initial side/rear-wall positioning; the route uses odometry after centered A is captured.
 */
#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>

#include "distance_controller/distance_controller.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/exceptions.h"

void DistanceController::configure_centering()
{
  if (!heading_control_enabled_) return;
  const auto & overrides = get_node_parameters_interface()->get_parameter_overrides();
  if (overrides.count("start_x") || overrides.count("start_y")) {
    throw std::invalid_argument(
      "start_x/start_y are obsolete: place near A; laser positions the start");
  }
  const auto positive = [this](const std::string & name, double fallback) {
    const double value = declare_parameter<double>(name, fallback);
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument(name + " must be finite and positive");
    }
    return value;
  };
  wall_heading_half_angle_ = positive("wall_heading_half_angle", wall_heading_half_angle_);
  wall_heading_min_span_ = positive("wall_heading_min_span", wall_heading_min_span_);
  wall_heading_max_rms_ = positive("wall_heading_max_rms", wall_heading_max_rms_);
  if (wall_heading_half_angle_ >= 0.7853981633974483) {
    throw std::invalid_argument("wall_heading_half_angle must be less than pi/4");
  }
  centering_gain_ = positive("centering_gain", centering_gain_);
  centering_max_speed_ = positive("centering_max_speed", centering_max_speed_);
  centering_tolerance_ = positive("centering_tolerance", centering_tolerance_);
  rear_target_distance_ = positive("rear_target_distance", rear_target_distance_);
  rear_min_distance_ = positive("rear_min_distance", rear_min_distance_);
  rear_window_half_angle_ = positive("rear_window_half_angle", rear_window_half_angle_);
  side_window_half_angle_ = positive("side_window_half_angle", side_window_half_angle_);
  side_min_distance_ = positive("side_min_distance", side_min_distance_);
  side_max_distance_ = positive("side_max_distance", side_max_distance_);
  side_max_mad_ = positive("side_max_mad", side_max_mad_);
  scan_timeout_ = positive("scan_timeout", scan_timeout_);
  preparation_timeout_ = positive("preparation_timeout", preparation_timeout_);
  preparation_max_travel_ = positive("preparation_max_travel", preparation_max_travel_);
  base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
  const auto topic = declare_parameter<std::string>("scan_topic", "/scan_filtered");
  if (
    side_window_half_angle_ >= 0.7853981633974483 || side_min_distance_ >= side_max_distance_ ||
    rear_window_half_angle_ >= 0.7853981633974483 ||
    rear_target_distance_ <= rear_min_distance_ + centering_tolerance_ ||
    rear_target_distance_ >= side_max_distance_ - centering_tolerance_ || base_frame_.empty() ||
    topic.empty()) {
    throw std::invalid_argument("Invalid side window, wall distance interval or scan/body frame");
  }
  scan_tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  scan_tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*scan_tf_buffer_);
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    topic, rclcpp::SensorDataQoS(),
    std::bind(&DistanceController::on_scan, this, std::placeholders::_1));
}

bool DistanceController::estimate_side(
  std::vector<double> & points, std::size_t samples, double & distance) const
{
  if (points.size() < 6 || samples == 0 || points.size() * 2 < samples) return false;
  std::sort(points.begin(), points.end());
  distance = points[points.size() / 2];
  std::vector<double> deviations;
  for (const double point : points) deviations.push_back(std::abs(point - distance));
  std::sort(deviations.begin(), deviations.end());
  return distance >= side_min_distance_ && distance <= side_max_distance_ &&
         deviations[deviations.size() / 2] <= side_max_mad_;
}

void DistanceController::on_scan(sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
{
  if (fault_latched_) return;
  left_wall_valid_ = false;
  right_wall_valid_ = false;
  scan_valid_ = false;
  right_heading_valid_ = false;
  const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
  const double age = (now() - stamp).seconds();
  if (
    msg->header.frame_id.empty() || stamp.nanoseconds() <= last_scan_stamp_ns_ || age < -0.1 ||
    age > scan_timeout_ || !std::isfinite(msg->angle_min) || !std::isfinite(msg->angle_increment) ||
    msg->angle_increment <= 0.0 || !std::isfinite(msg->range_min) ||
    !std::isfinite(msg->range_max) || msg->range_min < 0.0 || msg->range_max <= msg->range_min)
    return;
  geometry_msgs::msg::TransformStamped mounting;
  try {
    mounting =
      scan_tf_buffer_->lookupTransform(base_frame_, msg->header.frame_id, tf2::TimePointZero);
  } catch (const tf2::TransformException &) {
    return;
  }
  const auto & q = mounting.transform.rotation;
  const auto & t = mounting.transform.translation;
  const tf2::Quaternion rotation(q.x, q.y, q.z, q.w);
  const auto up = tf2::quatRotate(rotation, tf2::Vector3(0.0, 0.0, 1.0));
  if (
    !std::isfinite(rotation.length2()) || std::abs(rotation.length2() - 1.0) > 0.01 ||
    !std::isfinite(t.x) || !std::isfinite(t.y) || up.z() < 0.99)
    return;
  std::vector<std::pair<double, double>> wall_points;
  std::size_t wall_samples = 0;
  std::vector<double> left, right, rear;
  std::size_t left_samples = 0, right_samples = 0, rear_samples = 0;
  for (std::size_t i = 0; i < msg->ranges.size(); ++i) {
    const double angle = msg->angle_min + static_cast<double>(i) * msg->angle_increment;
    const auto direction =
      tf2::quatRotate(rotation, tf2::Vector3(std::cos(angle), std::sin(angle), 0.0));
    const double body_angle = std::atan2(direction.y(), direction.x());
    const double range = msg->ranges[i];
    const bool valid_range =
      std::isfinite(range) && range >= msg->range_min && range <= msg->range_max;
    if (
      !initial_alignment_complete_ &&
      std::abs(body_angle + 1.5707963267948966) <= wall_heading_half_angle_) {
      ++wall_samples;
      const double x = t.x + range * direction.x();
      const double y = t.y + range * direction.y();
      if (valid_range && -y >= side_min_distance_ && -y <= side_max_distance_) {
        wall_points.emplace_back(x, y);
      }
    }
    if (std::abs(std::abs(body_angle) - 3.141592653589793) <= rear_window_half_angle_) {
      ++rear_samples;
      const double rear_distance = -(t.x + range * direction.x());
      if (valid_range && rear_distance > 0.0) rear.push_back(rear_distance);
    }
    if (std::abs(std::abs(body_angle) - 1.5707963267948966) > side_window_half_angle_) continue;
    const bool is_left = body_angle > 0.0;
    if (is_left)
      ++left_samples;
    else
      ++right_samples;
    if (!valid_range) continue;
    const double y = t.y + range * direction.y();
    if ((is_left && y > 0.0) || (!is_left && y < 0.0)) {
      (is_left ? left : right).push_back(std::abs(y));
    }
  }
  if (!initial_alignment_complete_) estimate_right_heading(wall_points, wall_samples);
  wall_observation_time_ = std::chrono::steady_clock::now();
  left_wall_valid_ = estimate_side(left, left_samples, left_wall_);
  right_wall_valid_ = estimate_side(right, right_samples, right_wall_);
  const bool rear_valid =
    estimate_side(rear, rear_samples, rear_wall_) && rear_wall_ >= rear_min_distance_;
  if (!left_wall_valid_ || !right_wall_valid_ || !rear_valid) return;
  last_scan_stamp_ns_ = stamp.nanoseconds();
  last_scan_time_ = std::chrono::steady_clock::now();
  scan_valid_ = true;
  accepted_scan_ = true;
}

bool DistanceController::handle_centering_guard(
  const std::chrono::steady_clock::time_point & now, bool should_log)
{
  if (!heading_control_enabled_ || centering_complete_) return false;
  if (!preparation_started_) {
    preparation_start_ = now;
    preparation_x_ = last_odom_.pose.pose.position.x;
    preparation_y_ = last_odom_.pose.pose.position.y;
    preparation_started_ = true;
  }
  const double elapsed = std::chrono::duration<double>(now - preparation_start_).count();
  const double travel = std::hypot(
    last_odom_.pose.pose.position.x - preparation_x_,
    last_odom_.pose.pose.position.y - preparation_y_);
  const double age = std::chrono::duration<double>(now - last_scan_time_).count();
  const bool bounds = elapsed > preparation_timeout_ || travel > preparation_max_travel_;
  const bool unavailable = !scan_valid_ || !accepted_scan_ || age > scan_timeout_;
  if (!bounds && !unavailable) return false;
  centering_settling_ = false;
  alignment_settling_ = false;
  reset_pid();
  publish_stop();
  if (bounds || accepted_scan_) {
    fault_latched_ = true;
    RCLCPP_ERROR(
      get_logger(), "Initial preparation fault: scan invalid/stale or time/travel limit");
  } else if (should_log) {
    RCLCPP_INFO(get_logger(), "Waiting for fresh side/rear-wall scan and fixed laser-to-body TF");
  }
  return true;
}

bool DistanceController::handle_initial_centering(const rclcpp::Time & current_time)
{
  if (!heading_control_enabled_ || centering_complete_) return false;
  const double yaw = quaternion_to_yaw(last_odom_.pose.pose.orientation);
  const double yaw_error = heading_error(yaw);
  const double error = (left_wall_ - right_wall_) * 0.5;
  const double rear_error = rear_target_distance_ - rear_wall_;
  const auto & velocity = last_odom_.twist.twist;
  if (std::abs(yaw_error) > translation_pause_angle_) {
    centering_settling_ = false;
    publish_heading_correction(yaw_error);
    return true;
  }
  if (
    std::abs(error) > centering_tolerance_ || std::abs(rear_error) > centering_tolerance_ ||
    std::abs(yaw_error) > heading_tolerance_) {
    centering_settling_ = false;
    geometry_msgs::msg::Twist cmd;
    const double cap = std::min(centering_max_speed_, max_speed_);
    if (std::abs(rear_error) > centering_tolerance_) {
      cmd.linear.x = centering_gain_ * rear_error;
    }
    if (std::abs(error) > centering_tolerance_) {
      cmd.linear.y = centering_gain_ * error;
    }
    const double speed = std::hypot(cmd.linear.x, cmd.linear.y);
    if (speed > cap) {
      cmd.linear.x *= cap / speed;
      cmd.linear.y *= cap / speed;
    }
    if (std::abs(yaw_error) > heading_tolerance_) {
      cmd.angular.z = compute_heading_command(yaw_error, heading_gain_, max_yaw_rate_);
    }
    cmd_pub_->publish(cmd);
    return true;
  }
  publish_stop();
  if (
    std::hypot(velocity.linear.x, velocity.linear.y) >= 0.01 ||
    std::abs(velocity.angular.z) >= 0.02) {
    centering_settling_ = false;
    return true;
  }
  if (!centering_settling_) {
    centering_settle_start_ = current_time;
    centering_settling_ = true;
  } else if ((current_time - centering_settle_start_).seconds() >= alignment_settle_duration_) {
    route_x_ = last_odom_.pose.pose.position.x;
    route_y_ = last_odom_.pose.pose.position.y;
    route_yaw_ = heading_reference_;
    route_initialized_ = true;
    centering_complete_ = true;
    centering_settling_ = false;
    reset_pid();
    RCLCPP_INFO(
      get_logger(),
      "Centered A recorded=(%.6f, %.6f), yaw=%.6f rad (%.3f deg), "
      "left=%.3f right=%.3f rear=%.3f; starting route; heading_reference=%.6f rad",
      route_x_, route_y_, yaw, yaw * 180.0 / 3.141592653589793, left_wall_, right_wall_, rear_wall_,
      heading_reference_);
  }
  return true;
}

void DistanceController::log_route_wall_observation()
{
  if (!heading_control_enabled_ || !centering_complete_ || current_segment_index_ != 0) return;
  const double age =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - wall_observation_time_)
      .count();
  const bool fresh = age <= scan_timeout_;
  const auto left = fresh && left_wall_valid_ ? std::to_string(left_wall_) : "unavailable";
  const auto right = fresh && right_wall_valid_ ? std::to_string(right_wall_) : "unavailable";
  const auto & pose = last_odom_.pose.pose;
  const double yaw = quaternion_to_yaw(pose.orientation);
  RCLCPP_INFO(
    get_logger(),
    "A->B observation: x=%.6f y=%.6f dy_from_A=%.6f m yaw=%.6f rad (%.3f deg) | "
    "left=%s right=%s m scan_age=%.3f s | cross_track=%.6f m heading_error=%.6f rad | read-only",
    pose.position.x, pose.position.y, pose.position.y - route_y_, yaw,
    yaw * 180.0 / 3.141592653589793, left.c_str(), right.c_str(), age,
    -std::sin(route_yaw_) * (pose.position.x - route_x_) +
      std::cos(route_yaw_) * (pose.position.y - route_y_),
    heading_error(yaw));
}
