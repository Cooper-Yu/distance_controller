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
    {"A", "B", PlanarMotion::move_forward(forward_distance, speed, dwell)},
    {"B", "C", PlanarMotion::move_right(lateral_distance, speed, dwell)},
    {"C", "B", PlanarMotion::move_left(lateral_distance, speed, dwell)},
    {"B", "A", PlanarMotion::move_backward(forward_distance, speed, dwell)}};
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
}  // namespace distance_controller
