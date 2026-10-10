/** @file
 * @brief Serialized manual continuation; only the timer executes accepted motion.
 */
#include <cmath>
#include <functional>
#include <sstream>
#include <stdexcept>

#include "distance_controller/distance_controller.hpp"

void DistanceController::configure_step_interface()
{
  if (!manual_mode_) return;
  step_service_ = create_service<StepService>(
    "~/step",
    std::bind(
      &DistanceController::on_step_request, this, std::placeholders::_1, std::placeholders::_2));
}

std::string DistanceController::step_status()
{
  std::ostringstream out;
  out << "state="
      << (fault_latched_         ? "FAULT"
          : odom_recovering_     ? "RECOVERING"
          : !centering_complete_ ? "PREPARING"
          : manual_waiting_      ? "WAITING"
                                 : "RUNNING")
      << " waypoint=" << current_waypoint_
      << " pending=" << (segments_.size() - current_segment_index_)
      << " return_remaining=" << return_remaining_;
  if (received_odom_) {
    out << " pose=(" << last_odom_.pose.pose.position.x << "," << last_odom_.pose.pose.position.y
        << "," << quaternion_to_yaw(last_odom_.pose.pose.orientation) << ")"
        << " heading_reference=" << heading_reference_;
  } else
    out << " pose=unavailable";
  return out.str();
}

void DistanceController::on_step_request(
  const StepService::Request::SharedPtr request, StepService::Response::SharedPtr response)
{
  response->accepted = false;
  if (request->action == "status" || request->action == "history") {
    response->accepted = true;
    response->message = request->action == "history" ? history_.describe() : step_status();
    return;
  }
  if (request->action == "cancel") {
    fail_step("MANUAL_CANCEL");
    response->accepted = true;
    response->message = "Stopped and latched; restart required";
    return;
  }
  const auto & velocity = last_odom_.twist.twist;
  const double age =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - last_odom_time_).count();
  if (
    fault_latched_ || odom_recovering_ || !centering_complete_ || !manual_waiting_ ||
    !received_odom_ || age > 0.5 || std::hypot(velocity.linear.x, velocity.linear.y) >= 0.01 ||
    std::abs(velocity.angular.z) >= 0.02 ||
    std::abs(heading_error(quaternion_to_yaw(last_odom_.pose.pose.orientation))) >
      heading_tolerance_) {
    response->message =
      "Requires initialized, fresh, stopped, aligned WAITING state; " + step_status();
    return;
  }
  if (!check_continuation_position()) {
    response->message =
      "WAIT_POSITION_CHANGED; restart after verifying placement and odom reference";
    return;
  }
  if (request->action == "finish") {
    finish_requested_ = true;
    response->accepted = true;
    response->message = "Shutdown requested";
    return;
  }
  try {
    if (finish_requested_) throw std::invalid_argument("Shutdown already requested");
    if (request->action == "backtrack" || request->action == "return_to") {
      prepare_history_return(*request);
    } else if (request->action == "resume") {
      if (current_segment_index_ >= segments_.size())
        throw std::invalid_argument("No pending route; submit a new step");
    } else {
      if (current_segment_index_ < segments_.size())
        throw std::invalid_argument("Pending named route: resume it first");
      const auto step = make_requested_step(*request);
      segments_.push_back(step);
    }
    manual_waiting_ = false;
    target_initialized_ = false;
    segment_completed_ = false;
    settling_ = false;
    reset_pid();
    response->accepted = true;
    response->message =
      (return_remaining_ > 0 ? "Return accepted; pending route replaced, laser policies disabled. "
                             : "Accepted; wait for endpoint completion. ") +
      step_status();
  } catch (const std::exception & error) {
    response->message = error.what();
  }
}

distance_controller::RouteSegment DistanceController::make_requested_step(
  const StepService::Request & request)
{
  using namespace distance_controller;
  for (double value :
       {request.distance, request.x, request.y, request.speed, request.dwell, request.max_travel,
        request.timeout}) {
    if (!std::isfinite(value))
      throw std::invalid_argument("All numeric request fields must be finite");
  }
  if (request.speed < 0 || request.timeout < 0 || request.max_travel < 0 || request.dwell < 0)
    throw std::invalid_argument("Limits cannot be negative");
  if (!request.frame.empty() && request.frame != "route" && request.frame != "heading")
    throw std::invalid_argument("frame must be route or heading; target action uses absolute odom");
  if (request.label.find_first_of("\r\n") != std::string::npos || request.label.size() > 40)
    throw std::invalid_argument("label must be a single line of at most 40 characters");
  const double speed = request.speed == 0 ? max_speed_ : request.speed;
  RouteSegment step{
    current_waypoint_,
    request.label.empty() ? "point_" + std::to_string(segments_.size() + 1) : request.label,
    {}};
  step.relative_to_start = true;
  step.frame = request.frame == "heading" ? MotionFrame::Heading : MotionFrame::Route;
  if (request.action == "forward")
    step.motion = PlanarMotion::move_forward(request.distance, speed, request.dwell);
  else if (request.action == "backward")
    step.motion = PlanarMotion::move_backward(request.distance, speed, request.dwell);
  else if (request.action == "left")
    step.motion = PlanarMotion::move_left(request.distance, speed, request.dwell);
  else if (request.action == "right")
    step.motion = PlanarMotion::move_right(request.distance, speed, request.dwell);
  else if (request.action == "relative")
    step.motion = PlanarMotion::move_relative(request.x, request.y, speed, request.dwell);
  else if (request.action == "target") {
    step.frame = MotionFrame::Odom;
    step.motion = {request.x, request.y, speed, request.dwell};
  } else if (request.action == "front_wall") {
    if (front_body_extent_ <= 0)
      throw std::invalid_argument("Configure measured front_body_extent before a front-wall step");
    step.completion = CompletionKind::FrontWall;
    step.front_clearance = request.distance;
    step.motion = PlanarMotion::move_forward(
      request.max_travel == 0 ? 1.0 : request.max_travel, speed, request.dwell);
  } else
    throw std::invalid_argument("Unsupported action (turn is not implemented)");
  const double length =
    step.frame == MotionFrame::Odom
      ? std::hypot(
          request.x - last_odom_.pose.pose.position.x, request.y - last_odom_.pose.pose.position.y)
      : std::hypot(step.motion.dx, step.motion.dy);
  step.side_centering = request.side_centering;
  step.timeout = request.timeout == 0 ? 60.0 : request.timeout;
  step.max_travel = request.max_travel == 0 ? std::max(0.3, length + 0.25) : request.max_travel;
  validate_segment(step);
  return step;
}

bool DistanceController::handle_manual_wait()
{
  if (!manual_mode_ || !manual_waiting_) return false;
  if (finish_requested_) {
    publish_stop();
    rclcpp::shutdown();
    return true;
  }
  if (!check_continuation_position()) return true;
  const double yaw = quaternion_to_yaw(last_odom_.pose.pose.orientation);
  if (std::abs(heading_error(yaw)) > heading_tolerance_)
    publish_heading_correction(heading_error(yaw));
  else
    publish_stop();
  return true;
}

void DistanceController::record_step_endpoint()
{
  using namespace distance_controller;
  const auto & step = segments_[current_segment_index_];
  const auto & pose = last_odom_.pose.pose;
  current_waypoint_ = step.to;
  if (step.completion == CompletionKind::FrontWall || step.side_centering) {
    planned_x_ = pose.position.x;
    planned_y_ = pose.position.y;
  } else {
    planned_x_ = target_x_;
    planned_y_ = target_y_;
  }
  RCLCPP_INFO(
    get_logger(), "Endpoint %s recorded=(%.6f, %.6f), yaw=%.6f heading_reference=%.6f",
    current_waypoint_.c_str(), pose.position.x, pose.position.y,
    quaternion_to_yaw(pose.orientation), heading_reference_);
}

bool DistanceController::handle_current_pose_start(const rclcpp::Time & current_time)
{
  if (!adopt_current_pose_ || centering_complete_) return false;
  publish_stop();
  const auto & velocity = last_odom_.twist.twist;
  if (
    std::hypot(velocity.linear.x, velocity.linear.y) >= 0.01 ||
    std::abs(velocity.angular.z) >= 0.02) {
    alignment_settling_ = false;
    return true;
  }
  if (!alignment_settling_) {
    alignment_settling_ = true;
    alignment_settle_start_ = current_time;
  }
  if ((current_time - alignment_settle_start_).seconds() < alignment_settle_duration_) return true;
  const double observed_heading = quaternion_to_yaw(last_odom_.pose.pose.orientation);
  if (
    adopt_planned_heading_ && std::abs(std::atan2(
                                std::sin(planned_heading_ - observed_heading),
                                std::cos(planned_heading_ - observed_heading))) > 0.10) {
    fail_step("PLANNED_HEADING_MISMATCH: execute a separate turn first");
    return true;
  }
  heading_reference_ = adopt_planned_heading_ ? planned_heading_ : observed_heading;
  initial_alignment_complete_ = true;
  segments_.clear();
  record_route_origin();
  RCLCPP_INFO(
    get_logger(),
    "Manual WAITING: adopted current pose as session-local A; "
    "old history unavailable; heading_reference=%.6f",
    heading_reference_);
  return true;
}
