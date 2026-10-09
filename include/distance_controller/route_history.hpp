/** @file
 * @brief In-process completed-edge history and waypoint-chain return planning.
 */
#ifndef DISTANCE_CONTROLLER__ROUTE_HISTORY_HPP_
#define DISTANCE_CONTROLLER__ROUTE_HISTORY_HPP_
#include <cstdint>

#include "distance_controller/route.hpp"

namespace distance_controller
{
/// Measured planar pose in the unchanged odom frame; angles are radians.
struct RecordedPose
{
  double x{};    ///< Odom x in meters.
  double y{};    ///< Odom y in meters.
  double yaw{};  ///< Measured odom heading in radians.
};

/// Immutable completed traversal, including reverse traversals for audit.
struct CompletedEdge
{
  std::uint64_t id{};  ///< Unique traversal ID; also destination visit ID for forward steps.
  std::uint64_t from_visit{};  ///< Visit from which this traversal started.
  std::uint64_t to_visit{};  ///< New visit for forward motion, historical visit for reverse motion.
  RouteSegment step{};       ///< Executed policy and limits, never replayed as velocity commands.
  RecordedPose start{};      ///< Measured pose captured once by target initialization.
  RecordedPose end{};        ///< Measured pose after joint arrival and dwell.
  double heading{};          ///< Held heading reference, radians in odom.
};

/**
 * @brief Keep an audit log and the remaining outward path for sequential return.
 * @par Session boundary
 * Initialize at A. Only verified completed traversals are committed. Reverse completion
 * pops one outward edge but remains in the immutable audit log. No disk restoration.
 * @note Odom reference must remain unchanged; waypoint-chain return is not obstacle avoidance.
 */
class RouteHistory
{
public:
  /** @brief Initialize the session origin before any traversal.
   * @param[in] origin Pose from record_route_origin(); copied as visit 0.
   * @note Clears history; only initialization may call this method.
   */
  void initialize(const RecordedPose & origin);
  /** @brief Commit one completed traversal and advance the active visit.
   * @param[in] step Policy from record_completed_history(); copied for audit.
   * @param[in] start Initial pose captured by initialize_segment_target(); copied.
   * @param[in] end Stopped pose from record_completed_history(); copied.
   * @param[in] heading Held reference from the controller; copied in radians.
   * @note Throws on an inconsistent reverse ID before changing history.
   */
  void complete(
    const RouteSegment & step, const RecordedPose & start, const RecordedPose & end,
    double heading);
  /** @brief Find the number of outward edges between the current visit and a destination.
   * @param[in] label Destination forwarded by prepare_history_return(); read only for named lookup.
   * @param[in] use_visit_id Select exact visit lookup instead of label matching.
   * @param[in] visit_id Stable ID shown by describe() and forwarded by prepare_history_return(); read for exact lookup.
   * @return Positive edge count; throws for missing, current or ambiguous destinations.
   * @note Searches only the active ancestry; repeated labels require an exact visit ID.
   */
  std::size_t count_to(const std::string & label, bool use_visit_id, std::uint64_t visit_id) const;
  /** @brief Construct a bounded reverse plan without mutating history.
   * @param[in] count Positive edge count from prepare_history_return(); read to select completed edges, unchanged.
   * @return Absolute-odom steps through recorded starts, newest edge first.
   * @note Front-wall/side policies become position feedback; speed/dwell/time limits are retained.
   */
  std::vector<RouteSegment> reverse_plan(std::size_t count) const;
  /** @brief Format all completed traversals and currently returnable visit IDs.
   * @return Read-only service response for the history command.
   * @note Includes initial pose, actual starts/ends, policy, held heading and return audit.
   */
  std::string describe() const;

private:
  RecordedPose origin_{};    ///< A pose in odom, captured once during initialization.
  bool initialized_{false};  ///< True only after the current process records A.
  std::vector<CompletedEdge> completed_{};  ///< Append-only completed traversal audit.
  std::vector<std::size_t> outward_{};      ///< Indices of the active outward edge chain.
  std::uint64_t current_visit_{};           ///< Visit currently occupied; root is zero.
};
}  // namespace distance_controller
#endif
