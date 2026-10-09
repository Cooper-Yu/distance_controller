/** @file
 * @brief Independent named segment configuration and validation.
 */
#include <cmath>
#include <limits>
#include <map>
#include <stdexcept>

#include "distance_controller/distance_controller.hpp"

void DistanceController::configure_route_steps(double forward_distance, double lateral_distance)
{
  using namespace distance_controller;
  manual_mode_ = declare_parameter<bool>("manual_mode", false);
  start_paused_ = declare_parameter<bool>("start_paused", false);
  front_body_extent_ = declare_parameter<double>("front_body_extent", -1.0);
  if (
    (start_paused_ && !manual_mode_) || !std::isfinite(front_body_extent_) ||
    (front_body_extent_ <= 0 && front_body_extent_ != -1.0)) {
    throw std::invalid_argument(
      "start_paused requires manual_mode; front_body_extent must be positive or -1");
  }
  const auto names = declare_parameter<std::vector<std::string>>("route", {"AB", "BC", "CB", "BA"});
  if (names.empty()) throw std::invalid_argument("route cannot be empty");
  std::map<std::string, RouteSegment> definitions = {
    {"AB", make_ab_segment(forward_distance, max_speed_, dwell_duration_)},
    {"BC", make_bc_segment(lateral_distance, max_speed_, dwell_duration_)},
    {"CB", make_cb_segment(lateral_distance, max_speed_, dwell_duration_)},
    {"BA", make_ba_segment(forward_distance, max_speed_, dwell_duration_)}};
  std::map<std::string, RouteSegment> configured;
  std::string endpoint = "A";
  for (const auto & name : names) {
    if (
      name.size() != 2 || name[0] < 'A' || name[0] > 'Z' || name[1] < 'A' || name[1] > 'Z' ||
      name[0] == name[1] || name.substr(0, 1) != endpoint) {
      throw std::invalid_argument("Unknown/disconnected route ID: " + name);
    }
    if (!configured.count(name)) {
      RouteSegment step{
        name.substr(0, 1),
        name.substr(1, 1),
        {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
         max_speed_, dwell_duration_}};
      if (definitions.count(name)) step = definitions.at(name);
      configure_segment(name, step);
      configured.emplace(name, step);
    }
    segments_.push_back(configured.at(name));
    endpoint = name.substr(1, 1);
  }
}

void DistanceController::configure_segment(
  const std::string & name, distance_controller::RouteSegment & step)
{
  using namespace distance_controller;
  const auto prefix = "segments." + name + ".";
  step.motion.dx = declare_parameter<double>(prefix + "dx", step.motion.dx);
  step.motion.dy = declare_parameter<double>(prefix + "dy", step.motion.dy);
  step.motion.max_speed = declare_parameter<double>(prefix + "speed", step.motion.max_speed);
  step.motion.dwell = declare_parameter<double>(prefix + "dwell", step.motion.dwell);
  step.side_centering = declare_parameter<bool>(prefix + "side_centering", false);
  step.timeout = declare_parameter<double>(prefix + "timeout", 60.0);
  step.max_travel = declare_parameter<double>(
    prefix + "max_travel", std::max(0.3, std::hypot(step.motion.dx, step.motion.dy) + 0.25));
  step.front_clearance = declare_parameter<double>(prefix + "front_clearance", 0.20);
  const auto goal = declare_parameter<std::string>(prefix + "completion", "position");
  if (goal != "position" && goal != "front_wall")
    throw std::invalid_argument("Unknown completion: " + goal);
  step.completion = goal == "front_wall" ? CompletionKind::FrontWall : CompletionKind::Position;
  if (step.completion == CompletionKind::FrontWall && front_body_extent_ <= 0) {
    throw std::invalid_argument("Front goal requires measured front_body_extent in base_frame");
  }
  validate_segment(step);
}
