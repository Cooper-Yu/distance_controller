/** @file
 * @brief DistanceController interface and control-state documentation.
 */
#ifndef DISTANCE_CONTROLLER__DISTANCE_CONTROLLER_HPP_
#define DISTANCE_CONTROLLER__DISTANCE_CONTROLLER_HPP_

#include <chrono>
#include <cstddef>
#include <vector>

#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

/**
 * @brief Track a selected planar route using odometry and body-frame velocity commands.
 * @details Targets use the fixed initial route frame. Feedback receipt uses a steady
 * clock; PID, settling, and dwell use the node clock (simulation time when enabled).
 * The single-threaded executor in main() serializes callbacks. No obstacle avoidance
 * or target-heading controller is implemented. Scene 2 requires route configuration.
 */
class DistanceController : public rclcpp::Node
{
public:
  /**
   * @brief Configure the selected route and create the ROS control interfaces.
   *
   * @par Node setup
   * Validate the route and parameters before creating the subscription, wall timer, and publisher.
   * Scene 1 defaults to simulation time; scene 2 defaults to system-backed node time.
   * An explicit use_sim_time override takes precedence.
   *
   * @param[in] scene_number Scene selected by main() from non-ROS command-line arguments: 1 for simulation or 2 for CyberWorld. Read it to choose segments, clock defaults, and speed limits; no result is written back.
   * @throws std::invalid_argument If the scene, route coordinates, or numeric control parameters are invalid.
   * @note Construction declares ROS parameters and creates a 50 ms wall timer. Callbacks run serially under main().
   */
  explicit DistanceController(int scene_number = 1);

private:
  /// Per-tick inputs from on_timer() and command outputs retained for log_control_state().
  struct ControlDiagnostics
  {
    double x{};            ///< Current odom x position in meters.
    double y{};            ///< Current odom y position in meters.
    double yaw{};          ///< Current robot yaw relative to odom, in radians.
    double ex{};           ///< Target minus current odom x position in meters.
    double ey{};           ///< Target minus current odom y position in meters.
    double pid_dt{};       ///< Validated node-clock PID interval in seconds.
    double vx_odom_raw{};  ///< Odom x velocity before limiting, in meters per second.
    double vy_odom_raw{};  ///< Odom y velocity before limiting, in meters per second.
    double vx_odom{};      ///< Odom x velocity after limiting, in meters per second.
    double vy_odom{};      ///< Odom y velocity after limiting, in meters per second.
    double vx_robot{};     ///< Commanded body x velocity in meters per second.
    double vy_robot{};     ///< Commanded body y velocity in meters per second.
  };

  /**
   * @brief Print control status logs.
   *
   * @par Status logging
   * Print the current pose, measured velocity, target, and position errors.
   * Also print the PID interval, integrals, and velocity before and after limiting.
   * Include the final velocity command in the robot frame.
   *
   * @param[in] data Read the object passed by on_timer() to print this tick's values.
   * on_timer() fills the pose, errors, and PID interval; compute_and_publish_command()
   * fills the six velocity fields before this call. log_control_state() only reads
   * the object and does not write results back into it.
   * @note None at present.
   */
  void log_control_state(const ControlDiagnostics & data);

  /**
   * @brief Store the latest odometry and its local receipt time.
   *
   * @par Receive feedback
   * Copy the message to last_odom_, record steady-clock receipt time, and set received_odom_.
   *
   * @param[in] msg Message supplied by the subscription created in DistanceController(). Read its pose and twist into last_odom_ for on_timer(), log_control_state(), and check_completion(); do not modify the received message.
   * @note Freshness measures callback receipt, not header.stamp. Frame names and quaternion validity are assumed, not checked.
   */
  void on_odom(nav_msgs::msg::Odometry::SharedPtr msg);

  /**
   * @brief Stop when odometry is missing or reception has timed out.
   *
   * @par Check odometry feedback
   * Keep sending zero velocity until the first odometry message arrives.
   * If no update arrives within the timeout, latch a fault, reset PID, and stop.
   * Otherwise, allow the current timer callback to continue.
   *
   * @param[in] now Steady-clock time captured by on_timer(). Read it to calculate
   * the time since last_odom_time_, recorded by on_odom() when feedback is handled.
   * This function does not change on_timer()'s time variable.
   * @param[in] should_log Flag calculated by on_timer() from its logging interval.
   * Read it to decide whether to print the waiting message; do not write it back.
   * @return True to end this timer callback; false to continue control.
   * @note A timeout fault stays latched until the node is restarted.
   */
  bool handle_odom_wait_or_timeout(
    const std::chrono::steady_clock::time_point & now, bool should_log);

  /**
   * @brief Record ROS time and handle abnormal time changes.
   *
   * @par Check ROS time
   * Record the current time on the first call and continue.
   * On later calls, check the time elapsed since the previous observation.
   * If time moves backwards, latch a fault, reset PID, and stop.
   * If the interval exceeds 0.2 seconds, reset timing-related state and stop this tick.
   * Otherwise, continue the current timer callback.
   *
   * @param[in] current_time ROS time captured by on_timer(). Read it to compare
   * against last_ros_time_ and update the stored observation for the next check.
   * Updating last_ros_time_ does not change on_timer()'s current_time variable.
   * @return True to end this timer callback; false to continue.
   * @note This check also runs during settling and dwell, when PID is not updated. A large forward interval also restarts completed-segment dwell.
   */
  bool handle_ros_time_jump(const rclcpp::Time & current_time);

  /**
   * @brief Initialize PID history and check the time between PID updates.
   *
   * @par Prepare the next PID update
   * If PID is not initialized, initialize it and stop this tick.
   * Otherwise, calculate the time since the previous PID update.
   * If no time has passed, stop without resetting PID.
   * If the interval is negative or exceeds 0.2 seconds, reset PID and stop.
   * For a valid interval, update the previous PID time and allow control to continue.
   *
   * @param[in] ex Odom x error in meters, calculated as target_x_ minus x in on_timer().
   * Pass it to initialize_pid() when history is missing; do not modify the input.
   * @param[in] ey Odom y error in meters, calculated as target_y_ minus y in on_timer().
   * Pass it to initialize_pid() when history is missing; do not modify the input.
   * @param[in] current_time ROS time captured by on_timer(). Read it to initialize
   * PID history or calculate the interval since last_pid_time_. Store it in
   * last_pid_time_ when appropriate; do not modify on_timer()'s time variable.
   * @param[out] pid_dt Write the interval in seconds into on_timer()'s local pid_dt
   * variable through this reference; its incoming value is not used. Only a false
   * return makes the result valid for PID calculation. on_timer() then copies it
   * into data.pid_dt for compute_and_publish_command(), which passes it to
   * compute_pid() and limit_acceleration(). Initialization leaves pid_dt unchanged;
   * other early-return paths may write an invalid interval.
   * @return True to end this timer callback; false to continue PID calculation.
   * @note None at present.
   */
  bool handle_pid_timing(double ex, double ey, const rclcpp::Time & current_time, double & pid_dt);

  /**
   * @brief Calculate, limit, and publish the velocity command.
   *
   * @par Calculate and publish velocity
   * Calculate PID velocities in the odom frame and save the values before limiting.
   * Apply velocity and acceleration limits, then convert to the robot frame.
   * Save the resulting velocities and publish the body-frame velocity command.
   *
   * @param[in,out] data Reference to the local object created by on_timer().
   * Read data.ex, data.ey, data.yaw, and data.pid_dt, which on_timer() fills.
   * Write PID output into data.vx_odom/data.vy_odom, copy it into
   * data.vx_odom_raw/data.vy_odom_raw before limiting, and update
   * data.vx_odom/data.vy_odom with the limited values. Write the converted
   * command into data.vx_robot/data.vy_robot. All writes update the same object
   * in on_timer(), which then passes it to log_control_state(). The velocity
   * results are available after this function returns; data.x/data.y stay unchanged.
   * @note Run only after feedback and PID timing checks pass. Command publication
   * is independent of the logging rate. Updates PID and acceleration history; angular command is zero.
   */
  void compute_and_publish_command(ControlDiagnostics & data);

  /**
   * @brief Coordinate one control tick from feedback checks to stopping or tracking.
   *
   * @par Control tick
   * Check latched faults, feedback age, and node time before target initialization.
   * Handle completed-segment dwell, settling, and PID timing before computing a command.
   * Only routine logging is throttled; command calculation runs on each eligible tick.
   *
   * @note Mutates route and PID state through helpers, publishes velocity, and may shut down the ROS context at route completion. A wall timer continues firing when simulation time pauses.
   */
  void on_timer();

  /**
   * @brief Handle waiting and switching after a segment completes.
   *
   * @par Wait and switch
   * If the current segment is not completed, return false to continue tracking it.
   * Otherwise, keep the robot stopped until the waiting time has elapsed.
   * Then prepare the next segment, or shut down the node if all segments are completed.
   *
   * @param[in] current_time ROS time captured by on_timer(). Read it with
   * dwell_start_time_ to calculate how long the completed segment has waited.
   * This function does not change on_timer()'s current_time variable.
   * @return False to continue tracking; true to end the current timer callback.
   * @note Publishes zero, advances the index, and resets PID/segment flags for the next segment. After the final dwell, shuts down the default ROS context.
   */
  bool handle_completed_segment(const rclcpp::Time & current_time);

  /**
   * @brief Publish zero velocity and clear the velocity-ramp history.
   *
   * @par Stop output
   * Write all six Twist components as zero and publish through cmd_pub_.
   * Reset previous_vx_odom_ and previous_vy_odom_ so the next ramp starts from zero.
   *
   * @note Called by on_timer() and its stop/guard helpers. Stops bypass acceleration limiting; publishing does not itself prove physical standstill.
   */
  void publish_stop();

  /// One relative displacement in the fixed initial body frame, accumulated into route targets.
  struct Segment
  {
    double dx{};  ///< Forward displacement in meters in the initial body frame.
    double dy{};  ///< Left displacement in meters in the initial body frame.
  };

  /**
   * @brief Load and validate the selected sequence of relative displacements.
   *
   * @par Route selection
   * Replace segments_ with the selected route before control interfaces are created.
   * Displacements accumulate in the fixed initial body frame, not the changing robot frame.
   *
   * @param[in] scene_number Scene number forwarded by DistanceController() from main(). Read it to choose the route stored in segments_ for initialize_segment_target(); the caller value is unchanged.
   * @throws std::invalid_argument If the scene is unsupported or any displacement is nonfinite.
   * @note Scene 2 uses nominal corrected displacements; initial alignment and wall clearance are not verified.
   */
  void select_waypoints(int scene_number);

  /**
   * @brief Extract planar yaw from an odometry orientation.
   *
   * @par Orientation conversion
   * Convert the quaternion to roll, pitch, and yaw and return only yaw.
   *
   * @param[in] q Orientation from last_odom_.pose.pose.orientation passed by on_timer() or initialize_segment_target(). Read it for conversion without changing the stored feedback.
   * @return Yaw in radians relative to odom, used by on_timer() for velocity conversion or by initialize_segment_target() for route_yaw_.
   * @note The caller must supply a valid orientation; this helper does not validate or normalize the quaternion.
   */
  double quaternion_to_yaw(const geometry_msgs::msg::Quaternion & q);

  /**
   * @brief Freeze the current target from the route origin and cumulative displacement.
   *
   * @par Target setup
   * Called by on_timer() when target_initialized_ is false. Capture route_x_, route_y_,
   * and route_yaw_ once from last_odom_, sum segments through the current index, then
   * rotate the sum into odom and store target_x_, target_y_, and target_initialized_.
   *
   * @note An out-of-range index only logs and returns. Actual segment stopping error is not accumulated into later targets.
   */
  void initialize_segment_target();

  /**
   * @brief Clear PID error history and require reinitialization.
   *
   * @par PID reset
   * Clear integral_x_, integral_y_, prev_error_x_, and prev_error_y_; set pid_initialized_ false.
   * Called by on_timer() and timing, timeout, and segment-transition helpers.
   *
   * @note Does not reset route state, clock observations, or acceleration history; publish_stop() clears acceleration history separately.
   */
  void reset_pid();

  /**
   * @brief Calculate independent planar PID velocity components.
   *
   * @par PID update
   * Integrate and clamp both position errors, calculate their finite-difference derivatives,
   * then write the weighted outputs and retain errors for the next update.
   *
   * @param[in] ex Odom x error (m) from on_timer() data.ex, forwarded by compute_and_publish_command(); read for the x-axis PID and retain in prev_error_x_ without changing the input.
   * @param[in] ey Odom y error (m) from on_timer() data.ey, forwarded by compute_and_publish_command(); read for the y-axis PID and retain in prev_error_y_ without changing the input.
   * @param[in] dt Positive node-clock interval (s), validated by handle_pid_timing() and passed as data.pid_dt by compute_and_publish_command(); read for integration and differentiation.
   * @param[out] vx_odom Write raw x velocity (m/s, odom) into on_timer() data.vx_odom via compute_and_publish_command(). Incoming value is unused; on return the caller copies it to data.vx_odom_raw before limiting.
   * @param[out] vy_odom Write raw y velocity (m/s, odom) into on_timer() data.vy_odom via compute_and_publish_command(). Incoming value is unused; on return the caller copies it to data.vy_odom_raw before limiting.
   * @note Updates integral and previous-error members even when ki_ or kd_ is zero; does not publish or enforce speed limits.
   */
  void compute_pid(double ex, double ey, double dt, double & vx_odom, double & vy_odom);

  /**
   * @brief Limit planar speed while preserving the velocity direction.
   *
   * @par Speed limit
   * Scale both components equally only when their norm exceeds max_speed_.
   *
   * @param[in,out] vx Read raw odom x velocity (m/s) from data.vx_odom in compute_and_publish_command(); write the speed-limited value to the same field for limit_acceleration().
   * @param[in,out] vy Read raw odom y velocity (m/s) from data.vy_odom in compute_and_publish_command(); write the speed-limited value to the same field for limit_acceleration().
   * @note None at present.
   */
  void limit_velocity(double & vx, double & vy);

  /**
   * @brief Limit the planar velocity change and retain the resulting command.
   *
   * @par Velocity ramp
   * Compare the requested odom velocity with previous_vx_odom_ and previous_vy_odom_.
   * Limit the vector change to max_acceleration_ times dt and store the resulting history.
   *
   * @param[in,out] vx Read speed-limited odom x velocity (m/s) from compute_and_publish_command() data.vx_odom; write the ramp-limited result to the same field for odom_to_robot_velocity() and log_control_state().
   * @param[in,out] vy Read speed-limited odom y velocity (m/s) from compute_and_publish_command() data.vy_odom; write the ramp-limited result to the same field for odom_to_robot_velocity() and log_control_state().
   * @param[in] dt Positive interval (s) validated by handle_pid_timing(), stored by on_timer() in data.pid_dt, and forwarded by compute_and_publish_command(); read to bound the change, with no writeback.
   * @note Limits commanded acceleration and deceleration, not measured acceleration. publish_stop() bypasses this ramp and clears its history.
   */
  void limit_acceleration(double & vx, double & vy, double dt);

  /**
   * @brief Rotate a planar odom velocity into the current robot frame.
   *
   * @par Frame conversion
   * Apply the inverse planar yaw rotation; do not modify PID or route state.
   *
   * @param[in] vx_odom Limited odom x velocity (m/s) from compute_and_publish_command() data.vx_odom; read as a rotation input without modifying the field.
   * @param[in] vy_odom Limited odom y velocity (m/s) from compute_and_publish_command() data.vy_odom; read as a rotation input without modifying the field.
   * @param[in] yaw Robot yaw relative to odom (rad), extracted in on_timer() and forwarded as data.yaw by compute_and_publish_command(); read for the inverse rotation.
   * @param[out] vx_robot Write body x velocity (m/s) into compute_and_publish_command() local vx_robot. Incoming value is unused; on return that caller copies the result to data.vx_robot and cmd.linear.x.
   * @param[out] vy_robot Write body y velocity (m/s) into compute_and_publish_command() local vy_robot. Incoming value is unused; on return that caller copies the result to data.vy_robot and cmd.linear.y.
   * @note Assumes the command receiver accepts body-frame planar x/y velocities. No angular command is calculated.
   */
  void odom_to_robot_velocity(
    double vx_odom, double vy_odom, double yaw, double & vx_robot, double & vy_robot);

  /**
   * @brief Seed PID history from the current errors and node time.
   *
   * @par PID initialization
   * Store previous errors and last_pid_time_, clear both integrals, and mark PID initialized.
   *
   * @param[in] ex Odom x error (m) calculated by on_timer() and forwarded by handle_pid_timing(); copy to prev_error_x_ for the next compute_pid() derivative.
   * @param[in] ey Odom y error (m) calculated by on_timer() and forwarded by handle_pid_timing(); copy to prev_error_y_ for the next compute_pid() derivative.
   * @param[in] current_time Node time captured by on_timer() and forwarded by handle_pid_timing(); copy into last_pid_time_ for its next interval calculation. The caller time remains unchanged.
   * @note Initialization itself does not publish; handle_pid_timing() sends zero and ends this tick.
   */
  void initialize_pid(double ex, double ey, const rclcpp::Time & current_time);

  /**
   * @brief Mark the segment complete after continuous position and speed acceptance.
   *
   * @par Verify standstill
   * Require position error below 0.01 m, planar feedback speed below 0.01 m/s, and
   * absolute yaw rate below 0.02 rad/s for at least 0.5 node-clock seconds.
   * Start or reset settling_; on completion set segment_completed_ and dwell_start_time_.
   *
   * @param[in] ex Odom x position error (m) calculated and passed by on_timer(); read to test distance from the target without modifying the caller error.
   * @param[in] ey Odom y position error (m) calculated and passed by on_timer(); read to test distance from the target without modifying the caller error.
   * @param[in] current_time Node time captured by on_timer(); read for settling duration and copy to settle_start_time_ or dwell_start_time_ on state transitions. handle_completed_segment() later reads dwell_start_time_; the input is unchanged.
   * @note on_timer() publishes zero before calling. Feedback speed comes from last_odom_.twist in the child frame; target yaw is not checked or controlled.
   */
  void check_completion(double ex, double ey, const rclcpp::Time & current_time);

  /// Owns the feedback subscription that invokes on_odom().
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_{};
  /// Owns the 50 ms wall timer; on_timer() still runs while simulation time is paused.
  rclcpp::TimerBase::SharedPtr timer_{};
  /// Publishes body-frame Twist commands; unused axes and angular velocity stay zero.
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_{};

  /// Steady-clock time seeded at construction, updated by on_odom(); timeout checked only after reception.
  std::chrono::steady_clock::time_point last_odom_time_{std::chrono::steady_clock::now()};
  /// Latest copied feedback: pose assumed in odom, twist assumed in child_frame_id (base_link).
  nav_msgs::msg::Odometry last_odom_{};
  /// True after on_odom() first stores feedback; false keeps the robot waiting at zero velocity.
  bool received_odom_{false};
  /// Steady-clock time seeded at construction and updated for routine log opportunities; throttles at 1 s.
  std::chrono::steady_clock::time_point last_log_time_{std::chrono::steady_clock::now()};

  /// Ordered relative displacements (m) in the fixed initial body frame, selected at construction.
  std::vector<Segment> segments_{};
  /// Zero-based active segment index; reaches segments_.size() only at route shutdown.
  std::size_t current_segment_index_{0};

  /// Fixed segment target x in odom (m); valid only while target_initialized_ is true.
  double target_x_{0.0};
  /// Fixed segment target y in odom (m); valid only while target_initialized_ is true.
  double target_y_{0.0};

  /// True when target_x_/target_y_ belong to the current segment; cleared on advancement.
  bool target_initialized_{false};

  /// Proportional velocity gain (1/s); ROS parameter kp, shared by both odom axes.
  double kp_{1.5};
  /// Integral velocity gain (1/s^2); ROS parameter ki, default zero.
  double ki_{0.0};
  /// Derivative velocity gain (dimensionless); ROS parameter kd, default zero.
  double kd_{0.0};

  /// Clamped integral of odom x error (m*s), accumulated by compute_pid().
  double integral_x_{0.0};
  /// Clamped integral of odom y error (m*s), accumulated by compute_pid().
  double integral_y_{0.0};

  /// Previous odom x error (m) for differentiation; valid when pid_initialized_ is true.
  double prev_error_x_{0.0};
  /// Previous odom y error (m) for differentiation; valid when pid_initialized_ is true.
  double prev_error_y_{0.0};

  /// True when error and time history are seeded; false requires an initialization tick.
  bool pid_initialized_{false};

  /// Symmetric per-axis integral bound (m*s), applied even when ki_ is zero.
  double integral_limit_{0.5};

  /// Positive planar command speed limit (m/s); scene-dependent ROS parameter max_speed.
  double max_speed_{0.40};
  /// Positive odom command-change rate limit (m/s^2); stops bypass this limit.
  double max_acceleration_{0.60};
  /// Previous ramp output x in odom (m/s); publish_stop() resets it to zero.
  double previous_vx_odom_{0.0};
  /// Previous ramp output y in odom (m/s); publish_stop() resets it to zero.
  double previous_vy_odom_{0.0};

  /// Node-clock timestamp of PID initialization or last accepted interval; guarded by pid_initialized_.
  rclcpp::Time last_pid_time_{};

  /// True after feedback timeout or backwards node time; on_timer() stops until node restart.
  bool fault_latched_{false};

  /// True while the current uninterrupted standstill interval is being measured.
  bool settling_{false};
  /// Node-clock start of standstill verification; meaningful while settling_ is true.
  rclcpp::Time settle_start_time_{};
  /// True after 0.5 s settling; disables tracking and starts the extra dwell phase.
  bool segment_completed_{false};

  /// True after the first node-time observation, allowing comparison with last_ros_time_.
  bool ros_time_initialized_{false};
  /// Previous node-clock observation, independent of PID execution, including settling/dwell.
  rclcpp::Time last_ros_time_{};

  /// True after the fixed route origin and heading are captured once from feedback.
  bool route_initialized_{false};
  /// Fixed route origin x in odom (m); valid after route_initialized_.
  double route_x_{0.0};
  /// Fixed route origin y in odom (m); valid after route_initialized_.
  double route_y_{0.0};
  /// Initial body heading relative to odom (rad); rotates all nominal segment displacements.
  double route_yaw_{0.0};

  /// Node-clock start of extra dwell after completion; restarted after a large forward interval.
  rclcpp::Time dwell_start_time_{};
  /// Extra dwell after settling in node-clock seconds; nonnegative parameter, default 1 s.
  double dwell_duration_{1.0};
};

#endif
