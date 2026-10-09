/** @file
 * @brief Reusable planar motion descriptions and named route composition.
 */
#ifndef DISTANCE_CONTROLLER__ROUTE_HPP_
#define DISTANCE_CONTROLLER__ROUTE_HPP_
#include <cstdint>
#include <string>
#include <vector>

namespace distance_controller
{
/// One nonblocking motion request; axes belong to the fixed initialized route frame.
struct PlanarMotion
{
  double dx{};         ///< Forward displacement in meters; negative means backward.
  double dy{};         ///< Left displacement in meters; negative means right.
  double max_speed{};  ///< Positive segment speed cap in m/s, also bounded by the controller cap.
  double dwell{};      ///< Nonnegative extra standstill time in node-clock seconds.

  /**
   * @brief Describe a planar translation without starting or waiting for motion.
   * @param[in] dx Caller-supplied forward displacement (m); copy to the returned request.
   * @param[in] dy Caller-supplied left displacement (m); copy to the returned request.
   * @param[in] speed Caller-supplied positive speed cap (m/s); copy for limit_velocity().
   * @param[in] dwell Caller-supplied nonnegative dwell (s); copy for handle_completed_segment().
   * @return Validated request consumed by the route builder and segment executor.
   * @throws std::invalid_argument For nonfinite values, zero displacement or invalid limits.
   * @note No ROS state or caller input changes; begin/execute logic is in DistanceController.
   */
  static PlanarMotion move_relative(double dx, double dy, double speed, double dwell);

  /**
   * @brief Describe a forward translation along the fixed route axes.
   * @param[in] distance Positive meters supplied by compose_route(); read to set positive dx, unchanged.
   * @param[in] speed Positive m/s from compose_route(); copy into the request for the executor.
   * @param[in] dwell Nonnegative seconds from compose_route(); copy into the request for the executor.
   * @return Validated motion description; does not publish or block.
   * @throws std::invalid_argument For an invalid distance, speed or dwell.
   * @note Delegates to move_relative(); current robot yaw does not change these route axes.
   */
  static PlanarMotion move_forward(double distance, double speed, double dwell);

  /**
   * @brief Describe a backward translation along the fixed route axes.
   * @param[in] distance Positive meters supplied by compose_route(); read to set negative dx, unchanged.
   * @param[in] speed Positive m/s from compose_route(); copy into the request for the executor.
   * @param[in] dwell Nonnegative seconds from compose_route(); copy into the request for the executor.
   * @return Validated motion description; does not publish or block.
   * @throws std::invalid_argument For an invalid distance, speed or dwell.
   * @note Delegates to move_relative(); current robot yaw does not change these route axes.
   */
  static PlanarMotion move_backward(double distance, double speed, double dwell);

  /**
   * @brief Describe a left translation along the fixed route axes.
   * @param[in] distance Positive meters supplied by compose_route(); read to set positive dy, unchanged.
   * @param[in] speed Positive m/s from compose_route(); copy into the request for the executor.
   * @param[in] dwell Nonnegative seconds from compose_route(); copy into the request for the executor.
   * @return Validated motion description; does not publish or block.
   * @throws std::invalid_argument For an invalid distance, speed or dwell.
   * @note Delegates to move_relative(); current robot yaw does not change these route axes.
   */
  static PlanarMotion move_left(double distance, double speed, double dwell);

  /**
   * @brief Describe a right translation along the fixed route axes.
   * @param[in] distance Positive meters supplied by compose_route(); read to set negative dy, unchanged.
   * @param[in] speed Positive m/s from compose_route(); copy into the request for the executor.
   * @param[in] dwell Nonnegative seconds from compose_route(); copy into the request for the executor.
   * @return Validated motion description; does not publish or block.
   * @throws std::invalid_argument For an invalid distance, speed or dwell.
   * @note Delegates to move_relative(); current robot yaw does not change these route axes.
   */
  static PlanarMotion move_right(double distance, double speed, double dwell);
};

/// Position arrival or independently measured front-body clearance arrival.
enum class CompletionKind { Position, FrontWall };
/// Fixed initial route axes, held heading axes, or absolute odom coordinates.
enum class MotionFrame { Route, Heading, Odom };

/// Independently reusable named edge; it does not select its successor or shut down ROS.
struct RouteSegment
{
  std::string from{};     ///< Planned start label, used for continuity and logging.
  std::string to{};       ///< Destination label recorded after verified completion.
  PlanarMotion motion{};  ///< Displacement, speed and dwell for shared execution.
  CompletionKind completion{CompletionKind::Position};  ///< Selects the arrival measurement.
  MotionFrame frame{MotionFrame::Route};  ///< Translation axes; never changes heading_reference_.
  bool relative_to_start{false};          ///< Manual steps anchor to the accepted starting pose.
  bool side_centering{
    false};  ///< Side walls replace odom lateral tracking when explicitly enabled.
  double front_clearance{0.2};  ///< Target clearance from front body edge, meters.
  double timeout{60.0};         ///< Steady execution budget including recovery, settling and dwell.
  std::uint64_t reverse_of{};   ///< Nonzero completed edge ID only for a planned return.
  double max_travel{100.0};     ///< Maximum accumulated odom path length, meters.
};

/**
 * @brief Assemble a connected route from reusable AB, BC, CB and BA edges.
 * @param[in] names Startup route parameter from select_waypoints(); read ordered edge IDs, unchanged.
 * @param[in] forward_distance Positive A-to-B distance (m) from select_waypoints(); read for AB/BA.
 * @param[in] lateral_distance Positive B-to-C distance (m) from select_waypoints(); read for BC/CB.
 * @param[in] speed Positive speed cap (m/s) from select_waypoints(); copied into each motion.
 * @param[in] dwell Nonnegative dwell (s) from select_waypoints(); copied into each motion.
 * @return Validated route for select_waypoints() to store in segments_.
 * @throws std::invalid_argument For invalid motion parameters, empty/unknown/disconnected routes.
 * @note Every scene-2 run initializes at A. A route must start there; no implicit relocation occurs.
 */
std::vector<RouteSegment> compose_route(
  const std::vector<std::string> & names, double forward_distance, double lateral_distance,
  double speed, double dwell);
/**
 * @brief Define the independently editable forward A-to-B segment.
 * @par Segment definition
 * Build motion data only; the timer executor owns feedback and publication.
 * @param[in] distance Positive meters from compose_route()/configure_route_steps(); copied to motion.
 * @param[in] speed Positive m/s from the route builder; copied to the segment speed cap.
 * @param[in] dwell Nonnegative seconds from the route builder; copied to segment dwell.
 * @return Named segment, ready for policy overrides and shared execution.
 * @note No ROS side effects; invalid factory parameters throw std::invalid_argument.
 */
RouteSegment make_ab_segment(double distance, double speed, double dwell);

/**
 * @brief Define the independently editable right B-to-C segment.
 * @par Segment definition
 * Build motion data only; the timer executor owns feedback and publication.
 * @param[in] distance Positive meters from compose_route()/configure_route_steps(); copied to motion.
 * @param[in] speed Positive m/s from the route builder; copied to the segment speed cap.
 * @param[in] dwell Nonnegative seconds from the route builder; copied to segment dwell.
 * @return Named segment, ready for policy overrides and shared execution.
 * @note No ROS side effects; invalid factory parameters throw std::invalid_argument.
 */
RouteSegment make_bc_segment(double distance, double speed, double dwell);

/**
 * @brief Define the independently editable left C-to-B segment.
 * @par Segment definition
 * Build motion data only; the timer executor owns feedback and publication.
 * @param[in] distance Positive meters from compose_route()/configure_route_steps(); copied to motion.
 * @param[in] speed Positive m/s from the route builder; copied to the segment speed cap.
 * @param[in] dwell Nonnegative seconds from the route builder; copied to segment dwell.
 * @return Named segment, ready for policy overrides and shared execution.
 * @note No ROS side effects; invalid factory parameters throw std::invalid_argument.
 */
RouteSegment make_cb_segment(double distance, double speed, double dwell);

/**
 * @brief Define the independently editable backward B-to-A segment.
 * @par Segment definition
 * Build motion data only; the timer executor owns feedback and publication.
 * @param[in] distance Positive meters from compose_route()/configure_route_steps(); copied to motion.
 * @param[in] speed Positive m/s from the route builder; copied to the segment speed cap.
 * @param[in] dwell Nonnegative seconds from the route builder; copied to segment dwell.
 * @return Named segment, ready for policy overrides and shared execution.
 * @note No ROS side effects; invalid factory parameters throw std::invalid_argument.
 */
RouteSegment make_ba_segment(double distance, double speed, double dwell);

/**
 * @brief Validate a complete step before it can replace idle state.
 * @par Step contract
 * Reject nonfinite bounds, unsupported side-centering directions and incompatible frames.
 * @param[in] step Candidate from configure_route_steps() or on_step_request(); read only.
 * @note Throws std::invalid_argument on invalid input; no mutation or publication.
 */
void validate_segment(const RouteSegment & step);
}  // namespace distance_controller
#endif
