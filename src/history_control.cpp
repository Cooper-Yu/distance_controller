/** @file
 * @brief Current-session return planning and continuation-position checks.
 */
#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "distance_controller/distance_controller.hpp"

distance_controller::RecordedPose DistanceController::current_recorded_pose()
{
  const auto & pose = last_odom_.pose.pose;
  return {pose.position.x, pose.position.y, quaternion_to_yaw(pose.orientation)};
}

bool DistanceController::check_continuation_position()
{
  if (!waiting_pose_valid_) {
    fail_step("WAIT_POSE_UNAVAILABLE");
    return false;
  }
  const auto pose = current_recorded_pose();
  const double drift = std::hypot(pose.x - waiting_pose_.x, pose.y - waiting_pose_.y);
  if (drift <= resume_position_tolerance_) return true;
  RCLCPP_ERROR(
    get_logger(), "Waiting position changed: %.4f m exceeds %.4f m; no automatic relocation", drift,
    resume_position_tolerance_);
  fail_step("WAIT_POSITION_CHANGED");
  return false;
}

void DistanceController::prepare_history_return(const StepService::Request & request)
{
  const auto count = request.action == "backtrack"
                       ? static_cast<std::size_t>(request.steps)
                       : history_.count_to(request.label, request.use_visit_id, request.visit_id);
  const auto plan = history_.reverse_plan(count);
  auto updated = segments_;
  updated.resize(current_segment_index_);
  updated.insert(updated.end(), plan.begin(), plan.end());
  segments_.swap(updated);
  return_remaining_ = count;
}

bool DistanceController::record_completed_history()
{
  if (!heading_control_enabled_) return true;
  const auto end = current_recorded_pose();
  try {
    auto executed = segments_[current_segment_index_];
    executed.motion.max_speed = std::min(executed.motion.max_speed, max_speed_);
    if (executed.completion == distance_controller::CompletionKind::FrontWall)
      executed.motion.max_speed = std::min(executed.motion.max_speed, centering_max_speed_);
    history_.complete(executed, step_start_pose_, end, heading_reference_);
  } catch (const std::exception & error) {
    fail_step(std::string("HISTORY_INCONSISTENT: ") + error.what());
    return false;
  }
  waiting_pose_ = end;
  waiting_pose_valid_ = true;
  return true;
}
