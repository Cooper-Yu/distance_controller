/** @file
 * @brief Completed-only history; return through recorded starts rather than replaying commands.
 */
#include "distance_controller/route_history.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace distance_controller
{
void RouteHistory::initialize(const RecordedPose & origin)
{
  origin_ = origin;
  initialized_ = true;
  completed_.clear();
  outward_.clear();
  current_visit_ = 0;
}

void RouteHistory::complete(
  const RouteSegment & step, const RecordedPose & start, const RecordedPose & end, double heading)
{
  if (!initialized_) throw std::logic_error("History origin is not initialized");
  const auto id = static_cast<std::uint64_t>(completed_.size() + 1);
  auto destination = id;
  if (step.reverse_of != 0) {
    if (outward_.empty() || completed_[outward_.back()].id != step.reverse_of)
      throw std::logic_error("Reverse completion does not match active history");
    destination = completed_[outward_.back()].from_visit;
  }
  completed_.push_back({id, current_visit_, destination, step, start, end, heading});
  if (step.reverse_of != 0)
    outward_.pop_back();
  else
    outward_.push_back(completed_.size() - 1);
  current_visit_ = destination;
}

std::size_t RouteHistory::count_to(
  const std::string & label, bool use_visit_id, std::uint64_t visit_id) const
{
  if (!initialized_ || outward_.empty())
    throw std::invalid_argument("No outward history to return");
  std::size_t matches = 0, count = 0;
  for (std::size_t i = 0; i <= outward_.size(); ++i) {
    const auto id = i == 0 ? 0 : completed_[outward_[i - 1]].to_visit;
    const auto name = i == 0 ? std::string("A") : completed_[outward_[i - 1]].step.to;
    if (use_visit_id ? id == visit_id : name == label) {
      ++matches;
      count = outward_.size() - i;
    }
  }
  if (matches == 0) throw std::invalid_argument("Destination is not on the active return path");
  if (matches > 1) throw std::invalid_argument("Repeated label; choose --visit-id from history");
  if (count == 0) throw std::invalid_argument("Already at the selected visit");
  return count;
}

std::vector<RouteSegment> RouteHistory::reverse_plan(std::size_t count) const
{
  if (count == 0 || count > outward_.size()) throw std::invalid_argument("Invalid backtrack count");
  std::vector<RouteSegment> plan;
  for (std::size_t i = 0; i < count; ++i) {
    const auto & edge = completed_[outward_[outward_.size() - 1 - i]];
    RouteSegment step = edge.step;
    std::swap(step.from, step.to);
    step.motion.dx = edge.start.x;
    step.motion.dy = edge.start.y;
    step.frame = MotionFrame::Odom;
    step.relative_to_start = false;
    step.completion = CompletionKind::Position;
    step.side_centering = false;
    step.reverse_of = edge.id;
    validate_segment(step);
    plan.push_back(step);
  }
  return plan;
}

std::string RouteHistory::describe() const
{
  if (!initialized_) return "History unavailable: initialization has not completed";
  std::ostringstream out;
  out << "session=current_process current_visit=" << current_visit_
      << " completed=" << completed_.size() << " outward=" << outward_.size()
      << "\nvisit=0 label=A pose=(" << origin_.x << "," << origin_.y << "," << origin_.yaw << ")\n";
  for (const auto & edge : completed_) {
    out << "edge=" << edge.id << " " << edge.step.from << "->" << edge.step.to
        << " visits=" << edge.from_visit << "->" << edge.to_visit
        << " reverse_of=" << edge.step.reverse_of << " start=(" << edge.start.x << ","
        << edge.start.y << "," << edge.start.yaw << ")"
        << " end=(" << edge.end.x << "," << edge.end.y << "," << edge.end.yaw << ")"
        << " heading=" << edge.heading << " speed=" << edge.step.motion.max_speed
        << " dwell=" << edge.step.motion.dwell << " side_centering=" << edge.step.side_centering
        << " completion="
        << (edge.step.completion == CompletionKind::FrontWall ? "front_wall" : "position") << "\n";
  }
  out << "active_visits=0:A";
  for (auto index : outward_)
    out << "," << completed_[index].to_visit << ":" << completed_[index].step.to;
  return out.str();
}
}  // namespace distance_controller
