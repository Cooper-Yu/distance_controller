/** @file
 * @brief Reusable planar motion descriptions and named route composition.
 */
#ifndef DISTANCE_CONTROLLER__ROUTE_HPP_
#define DISTANCE_CONTROLLER__ROUTE_HPP_
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

/// Independently reusable named edge; it does not select its successor or shut down ROS.
struct RouteSegment
{
  std::string from{};  ///< Named planned start, such as A; used for continuity validation/logs.
  std::string to{};  ///< Named planned destination, such as B; used for continuity validation/logs.
  PlanarMotion
    motion{};  ///< Relative displacement, speed and dwell applied by the shared executor.
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
}  // namespace distance_controller
#endif
