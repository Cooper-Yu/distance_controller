/** @file
 * @brief Initial selected-wall direction estimation (legacy right-heading member names) in the robot body frame.
 */
#include <algorithm>
#include <cmath>
#include <limits>

#include "distance_controller/distance_controller.hpp"

void DistanceController::estimate_right_heading(
  const std::vector<std::pair<double, double>> & points, std::size_t samples)
{
  const bool was_valid = right_heading_tracking_;
  right_heading_tracking_ = false;
  right_heading_stable_ = false;
  right_heading_valid_ = false;
  right_wall_rms_ = right_wall_span_ = 0.0;
  if (!received_odom_ || points.size() < 12 || points.size() * 2 < samples) return;
  double mean_x = 0.0, mean_y = 0.0;
  for (const auto & point : points) {
    mean_x += point.first;
    mean_y += point.second;
  }
  mean_x /= points.size();
  mean_y /= points.size();
  double xx = 0.0, xy = 0.0, yy = 0.0;
  for (const auto & point : points) {
    const double x = point.first - mean_x, y = point.second - mean_y;
    xx += x * x;
    xy += x * y;
    yy += y * y;
  }
  right_wall_angle_ = 0.5 * std::atan2(2.0 * xy, xx - yy);
  if (std::abs(right_wall_angle_) > 0.5235987755982988) return;
  const double c = std::cos(right_wall_angle_), s = std::sin(right_wall_angle_);
  double lower = std::numeric_limits<double>::infinity(), upper = -lower, residual = 0.0;
  for (const auto & point : points) {
    const double x = point.first - mean_x, y = point.second - mean_y;
    const double along = c * x + s * y, across = -s * x + c * y;
    lower = std::min(lower, along);
    upper = std::max(upper, along);
    residual += across * across;
  }
  right_wall_span_ = upper - lower;
  right_wall_rms_ = std::sqrt(residual / points.size());
  if (right_wall_span_ < wall_heading_min_span_ || right_wall_rms_ > wall_heading_max_rms_) return;
  right_heading_valid_ = true;
  const double world_angle =
    quaternion_to_yaw(last_odom_.pose.pose.orientation) + right_wall_angle_;
  const double difference = std::atan2(
    std::sin(world_angle - right_heading_anchor_), std::cos(world_angle - right_heading_anchor_));
  const auto receipt = std::chrono::steady_clock::now();
  if (
    !was_valid || std::abs(difference) > 0.02 ||
    std::chrono::duration<double>(receipt - wall_observation_time_).count() > scan_timeout_) {
    right_heading_anchor_ = world_angle;
    right_heading_since_ = receipt;
  }
  right_heading_tracking_ = true;
  right_heading_stable_ =
    std::chrono::duration<double>(receipt - right_heading_since_).count() >= 0.5;
}
