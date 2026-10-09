/** @file
 * @brief Validate motion parameters and compose named routes without ROS side effects.
 */
#include "distance_controller/route.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace distance_controller
{
PlanarMotion PlanarMotion::move_relative(double dx, double dy, double speed, double dwell)
{
  if (
    !std::isfinite(dx) || !std::isfinite(dy) || (dx == 0.0 && dy == 0.0) || !std::isfinite(speed) ||
    speed <= 0.0 || !std::isfinite(dwell) || dwell < 0.0) {
    throw std::invalid_argument(
      "Motion requires finite nonzero displacement, positive speed and nonnegative dwell");
  }
  return {dx, dy, speed, dwell};
}

PlanarMotion PlanarMotion::move_forward(double distance, double speed, double dwell)
{
  if (!std::isfinite(distance) || distance <= 0.0) {
    throw std::invalid_argument("move_forward: distance must be finite and positive");
  }
  return move_relative(distance, 0.0, speed, dwell);
}

PlanarMotion PlanarMotion::move_backward(double distance, double speed, double dwell)
{
  if (!std::isfinite(distance) || distance <= 0.0) {
    throw std::invalid_argument("move_backward: distance must be finite and positive");
  }
  return move_relative(-distance, 0.0, speed, dwell);
}

PlanarMotion PlanarMotion::move_left(double distance, double speed, double dwell)
{
  if (!std::isfinite(distance) || distance <= 0.0) {
    throw std::invalid_argument("move_left: distance must be finite and positive");
  }
  return move_relative(0.0, distance, speed, dwell);
}

PlanarMotion PlanarMotion::move_right(double distance, double speed, double dwell)
{
  if (!std::isfinite(distance) || distance <= 0.0) {
    throw std::invalid_argument("move_right: distance must be finite and positive");
  }
  return move_relative(0.0, -distance, speed, dwell);
}

std::vector<RouteSegment> compose_route(
  const std::vector<std::string> & names, double forward_distance, double lateral_distance,
  double speed, double dwell)
{
  const std::vector<RouteSegment> definitions = {
    make_ab_segment(forward_distance, speed, dwell),
    make_bc_segment(lateral_distance, speed, dwell),
    make_cb_segment(lateral_distance, speed, dwell),
    make_ba_segment(forward_distance, speed, dwell)};
  if (names.empty()) throw std::invalid_argument("route must contain at least one segment");
  std::vector<RouteSegment> route;
  std::string endpoint = "A";
  for (const auto & name : names) {
    const auto found = std::find_if(
      definitions.begin(), definitions.end(),
      [&name](const auto & edge) { return edge.from + edge.to == name; });
    if (found == definitions.end()) throw std::invalid_argument("Unknown route segment: " + name);
    if (found->from != endpoint) {
      throw std::invalid_argument("Disconnected route: " + name + " does not start at " + endpoint);
    }
    route.push_back(*found);
    endpoint = found->to;
  }
  return route;
}

RouteSegment make_ab_segment(double distance, double speed, double dwell)
{
  return {"A", "B", PlanarMotion::move_forward(distance, speed, dwell)};
}

RouteSegment make_bc_segment(double distance, double speed, double dwell)
{
  return {"B", "C", PlanarMotion::move_right(distance, speed, dwell)};
}

RouteSegment make_cb_segment(double distance, double speed, double dwell)
{
  return {"C", "B", PlanarMotion::move_left(distance, speed, dwell)};
}

RouteSegment make_ba_segment(double distance, double speed, double dwell)
{
  return {"B", "A", PlanarMotion::move_backward(distance, speed, dwell)};
}

void validate_segment(const RouteSegment & step)
{
  const auto & m = step.motion;
  if (
    !std::isfinite(m.dx) || !std::isfinite(m.dy) || !std::isfinite(m.max_speed) ||
    m.max_speed <= 0 || !std::isfinite(m.dwell) || m.dwell < 0 || !std::isfinite(step.timeout) ||
    step.timeout <= 0 || !std::isfinite(step.max_travel) || step.max_travel <= 0 ||
    !std::isfinite(step.front_clearance) || step.front_clearance <= 0.01) {
    throw std::invalid_argument(
      "Step requires finite coordinates, positive limits and clearance > 0.01 m");
  }
  if (step.frame != MotionFrame::Odom && m.dx == 0 && m.dy == 0) {
    throw std::invalid_argument("Relative step cannot have zero displacement");
  }
  if (step.side_centering && (step.frame == MotionFrame::Odom || m.dy != 0 || m.dx == 0)) {
    throw std::invalid_argument("Side centering is only available on forward/backward steps");
  }
  if (
    step.completion == CompletionKind::FrontWall &&
    (step.frame == MotionFrame::Odom || m.dx <= 0 || m.dy != 0)) {
    throw std::invalid_argument("Front-wall completion requires a forward step");
  }
}
}  // namespace distance_controller
