/** @file
 * @brief DistanceController interface and control-state documentation.
 */
#ifndef DISTANCE_CONTROLLER__DISTANCE_CONTROLLER_HPP_
#define DISTANCE_CONTROLLER__DISTANCE_CONTROLLER_HPP_

#include <chrono>
#include <cstddef>
#include <utility>
#include <vector>

#include "distance_controller/route.hpp"
#include "distance_controller/route_history.hpp"
#include "distance_controller/srv/execute_step.hpp"
#include "distance_controller/wall_heading_window.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

/**
 * @brief Track a selected planar route using odometry and body-frame velocity commands.
 * @details Targets use the fixed initial route frame. Feedback receipt uses a steady
 * clock; PID, settling, and dwell use the node clock (simulation time when enabled).
 * The single-threaded executor in main() serializes callbacks. No general obstacle avoidance
 * is implemented; selected steps can use a front-clearance goal. Scene 2 aligns to the initial selected wall and holds its captured odom heading.
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

  /** @brief Return process status to main() after shutdown.
   * @return Two for exhausted odom recovery; zero unless a terminal failure set it.
   * @note No state mutation; an unverified base watchdog keeps the node latched alive.
   */
  int process_exit_code() const
  {
    return process_exit_code_;
  }

private:
  /** @brief Start a bounded stop after a feedback gap detected by on_odom() or the timer.
   * @param[in] receipt Steady time from the detecting callback; copied as the fixed 2 s deadline origin.
   * @note Reads last_odom_ as the continuity anchor; clears PID and settling, preserves target/history.
   */
  void begin_odom_recovery(const std::chrono::steady_clock::time_point & receipt);
  /** @brief Check incoming feedback before on_odom() writes it to last_odom_.
   * @param[in] msg New on_odom() sample; read source stamp, frames, pose and velocity.
   * @param[in] receipt Current steady callback time; used for gap and recovery stability checks.
   * @return True to accept the sample; false keeps the prior feedback and commands zero on recovery.
   * @note Updates recovery stability only; rejects stale/reordered feedback and latches pose/frame jumps.
   */
  bool validate_odom_recovery(
    const nav_msgs::msg::Odometry & msg, const std::chrono::steady_clock::time_point & receipt);
  /** @brief Keep zero commands until recovery succeeds or its fixed deadline expires.
   * @param[in] receipt Steady timer time from handle_odom_wait_or_timeout(); read for deadlines.
   * @return True: this tick always stays stopped, including the recovery transition.
   * @note Does not extend preparation or segment budgets, change targets, or add history entries.
   */
  bool handle_odom_recovery(const std::chrono::steady_clock::time_point & receipt);
  /** @brief Latch unsuccessful recovery and optionally terminate with nonzero status.
   * @param[in] reason Diagnostic literal from recovery checks; copied to the ROS error log.
   * @note Publishes zero; exits by default; an explicit false watchdog parameter keeps zero commands latched.
   */
  void fail_odom_recovery(const char * reason);
  bool odom_recovering_{
    false};  ///< Scene 2 feedback gap is in the bounded zero-command recovery phase.
  bool odom_recovery_stable_{
    false};  ///< Fresh stopped samples have remained continuous since recovery_since_.
  bool base_command_watchdog_verified_{
    true};  ///< Task2 exit policy assumes base timeout; false keeps latched zero commands.
  int process_exit_code_{0};  ///< Exit status read by main(); 2 denotes unrecovered odom failure.
  std::chrono::steady_clock::time_point
    recovery_start_{};  ///< Fixed steady-clock start of the 2 s recovery budget.
  std::chrono::steady_clock::time_point
    recovery_since_{};  ///< First sample of a continuous stopped recovery sequence.
  nav_msgs::msg::Odometry
    recovery_anchor_{};  ///< Last good pre-gap pose/frames; recovery jump comparison in odom.

  /**
   * @brief Calculate lateral centering velocity without publishing.
   * @par Centering calculation
   * Use the latest accepted side distances and the shared positioning deadband.
   * @return Body-y m/s consumed by handle_initial_positioning().
   * @note Reads left_wall_/right_wall_; no state changes.
   */
  double compute_centering_velocity() const;
  /**
   * @brief Calculate rear-distance positioning velocity without publishing.
   * @par Rear positioning
   * Positive output moves away from the rear wall toward the configured distance.
   * @return Body-x m/s consumed by handle_initial_positioning().
   * @note Reads rear_wall_/rear_target_distance_; no state changes.
   */
  double compute_rear_position_velocity() const;
  /**
   * @brief Calculate the yaw command toward the persistent heading reference.
   * @par Heading hold
   * Apply the existing heading deadband, gain and angular limit.
   * @param[in] yaw Odom heading (rad) from the calling positioning, waiting or command method;
   * read against heading_reference_ without redefining it.
   * @return Body angular-z rad/s; caller combines it with its selected planar command.
   * @note No publication or reference update. Future turn completion owns reference changes.
   */
  double compute_heading_hold_velocity(double yaw) const;
  /**
   * @brief Capture A only after joint initial positioning and standstill acceptance.
   * @par Route origin
   * Copy the latest odom position and previously aligned heading into the fixed route origin.
   * @note Called by handle_initial_positioning(); initializes planned endpoint and logs A.
   */
  void record_route_origin();

  /**
   * @brief Declare and validate scene-2 heading parameters before ROS interfaces exist.
   * @par Configuration
   * Called by DistanceController(); store startup overrides in heading members.
   * @throws std::invalid_argument For nonfinite, nonpositive, or inconsistent limits.
   * @note Scene 1 bypasses heading configuration and preserves its original behavior.
   */
  void configure_heading_control();

  /**
   * @brief Hold translation until selected-wall alignment and standstill remain accepted.
   * @par Initial alignment
   * Read the stable selected-wall fit from on_scan(); rotate only outside heading_tolerance_, otherwise
   * stop and require low measured speeds for alignment_settle_duration_. Mark
   * initial_alignment_complete_ and capture heading_reference_ once; on_timer() begins laser centering on the following tick.
   * @param[in] current_time Node time supplied by on_timer(); read elapsed settling time
   * and copy into alignment_settle_start_ when the interval begins. Input is unchanged.
   * @return True to end this tick; false when disabled or already completed.
   * @note Publishes commands and resets PID through publish_heading_correction(). Centering follows through handle_initial_positioning().
   */
  bool handle_initial_alignment(const rclcpp::Time & current_time);

  /**
   * @brief Calculate a bounded proportional yaw rate to reduce a current-minus-target heading error.
   * @par Heading error
   * Wrap target-minus-current error into [-pi, pi] before symmetric rate limiting.
   * @param[in] yaw Wrapped current-minus-target error (rad) from the calling
   * heading-control method; read only. Initial alignment supplies negative wall angle;
   * later callers use heading_error() against the captured reference.
   * @param[in] gain Positive gain (1/s), supplied from heading_gain_; scales error, unchanged.
   * @param[in] max_yaw_rate Positive cap (rad/s), supplied from max_yaw_rate_; bounds output, unchanged.
   * @return Angular command for the caller's cmd.angular.z; does not publish itself.
   * @note Callers ensure finite inputs and positive limits. The target comes from the initial selected wall.
   */
  static double compute_heading_command(double yaw, double gain, double max_yaw_rate);

  /**
   * @brief Publish rotation only and clear planar PID and command-ramp history.
   * @par Rotation without translation
   * Used by handle_initial_alignment(), handle_initial_positioning(), and handle_heading_recovery().
   * @param[in] yaw Current-minus-target heading error (rad) from the caller; read to
   * compute cmd.angular.z using heading_gain_ and max_yaw_rate_. No input is modified.
   * @note Publishes zero linear velocity; all unused Twist components remain zero.
   */
  void publish_heading_correction(double yaw);

  /**
   * @brief Pause translation for large heading errors or rotate at an arrived position.
   * @par Route recovery
   * Cancel completed-segment dwell if position or heading leaves acceptance. Rotate
   * only at a position target or beyond translation_pause_angle_; otherwise permit tracking.
   * @param[in] yaw Current odom heading (rad) passed by execute_current_segment(); read to check
   * heading acceptance and compute rotation commands; caller value is unchanged.
   * @param[in] position_error Position-error norm (m) computed by execute_current_segment(); read to
   * choose rotation-only recovery or cancel dwell; caller value is unchanged.
   * @return True after publishing rotation only; false to continue this tick.
   * @note May clear segment_completed_, settling_, PID and ramp history. Scene 1 bypasses.
   */
  bool handle_heading_recovery(double yaw, double position_error);

  /**
   * @brief Configure initial laser side/rear positioning and create scene-2 scan/TF interfaces.
   * @par Startup
   * Called by DistanceController(); validate parameters before scan subscription creation.
   * @throws std::invalid_argument For invalid limits or obsolete fixed-start overrides.
   * @note SensorDataQoS accepts reliable and best-effort scan publishers; scene 1 bypasses.
   */
  void configure_centering();

  /**
   * @brief Estimate left/right/rear body-frame wall distances from one fresh scan.
   * @par Scan input
   * Transform scan points into base_frame_ using the latest fixed mounting TF, then
   * select side/rear windows and validate finite coverage and median absolute deviation.
   * @param[in] msg LaserScan supplied by the scan subscription; read ranges, angles,
   * frame and stamp, without modifying the message. Write estimates into left_wall_/right_wall_/rear_wall_.
   * @note Invalid scans clear scan_valid_; guard logic stops motion. Initial selected-wall direction is estimated separately. After initialization, scans update read-only observations and do not gate the route.
   */
  void on_scan(sensor_msgs::msg::LaserScan::ConstSharedPtr msg);

  /**
   * @brief Reduce a wall window to a robust horizontal wall distance.
   * @param[in,out] points measure_wall_windows() supplies positive body-y or rearward body-x distances (m); read and
   * reorder the caller's vector to calculate median and dispersion. Not used after this call.
   * @param[in] samples measure_wall_windows() counts every ray in this window, including invalid ranges;
   * read as the coverage denominator; caller value unchanged.
   * @param[out] distance Write the median distance (m) to measure_wall_windows()'s left_wall_, right_wall_ or rear_wall_; valid only when true is returned.
   * @return True for sufficient coverage, low dispersion and accepted wall distance.
   * @note This assumes each window observes one nearby wall; it is not obstacle recognition.
   */
  bool estimate_side(std::vector<double> & points, std::size_t samples, double & distance) const;

  /**
   * @brief Stop initial adjustment when scan, travel or duration limits are violated.
   * @param[in] now Steady-clock observation from on_timer(); read receipt age and
   * initialization duration, copied to preparation_start_ on first entry.
   * @param[in] should_log on_timer() supplies the log throttle decision; read only.
   * @return True after publishing stop; false if disabled, complete, or ready to adjust.
   * @note Starts preparation position tracking from last_odom_; latches faults after
   * accepted scans become unavailable, or stage/total time/travel bounds are exceeded.
   * Reports a distinct reason and stage; missing initial input waits only within its deadline.
   */
  bool handle_centering_guard(const std::chrono::steady_clock::time_point & now, bool should_log);

  /**
   * @brief Position from side/rear walls while holding the captured wall heading, then capture A after standstill.
   * @param[in] current_time Node time from on_timer(); read continuous settling duration
   * and copy to centering_settle_start_ when settling begins; caller time unchanged.
   * @return True to end this tick during preparation; false once complete or in scene 1.
   * @note Reads on_scan() distances and on_odom() pose/twist. Publishes bounded body-x/body-y/yaw commands; on success writes route_x_/route_y_/route_yaw_ and
   * route_initialized_. Controls rear distance only during preparation; does not perform route obstacle avoidance.
   */
  bool handle_initial_positioning(const rclcpp::Time & current_time);

  /**
   * @brief Log side-wall observations and odometry during A-to-B without affecting commands.
   * @par Observation
   * Called by execute_current_segment() at the on_timer() log cadence and by check_completion() at B acceptance.
   * Read on_scan() side estimates, validity and receipt age independently of rear validity;
   * read on_odom() pose and route_y_ to report lateral odom displacement from A.
   * @note Missing, invalid or stale sides print unavailable. No control flags or commands
   * are changed; scene 1 and other route segments produce no output.
   */
  void log_route_wall_observation();

  /**
   * @brief Fit a single selected-wall line and require a stable odom-frame direction.
   * @par Startup measurement
   * on_scan() supplies a wider right window than the distance windows. Reject poor
   * coverage, short span, excessive perpendicular residual or ambiguous orientation.
   * @param[in] points Body-frame (x,y) endpoints in meters from measure_wall_windows(); read only
   * to estimate the forward wall tangent. No point is changed.
   * @param[in] samples Total window rays from measure_wall_windows(), including invalid ranges;
   * read to check coverage, unchanged.
   * @note Writes right_wall_angle_/right_wall_rms_/right_wall_span_ and quality flags
   * for handle_initial_alignment(). Latest on_odom() yaw supplies the temporal
   * consistency reference; scan and odom must describe the same slowly moving robot.
   */
  void estimate_right_heading(
    const std::vector<std::pair<double, double>> & points, std::size_t samples);

  /**
   * @brief Express odom heading relative to the captured selected-wall reference.
   * @param[in] yaw Current odom yaw from the control or initialization caller;
   * read and subtract heading_reference_, without modifying caller feedback.
   * @return Wrapped current-minus-reference error in radians for acceptance/control.
   * @note Actual yaw remains necessary for odom-to-body velocity conversion.
   */
  double heading_error(double yaw) const;

  /// Captured odom yaw (rad) after initial wall-parallel standstill; fixed for this run.
  double heading_reference_{0.0};
  /// Selected-wall forward tangent relative to body x (rad); valid only with right_heading_valid_.
  double right_wall_angle_{0.0};
  /// Latest perpendicular line-fit RMS (m), diagnostic even when rejected.
  double right_wall_rms_{0.0};
  /// Latest fitted tangent span (m), diagnostic even when rejected.
  double right_wall_span_{0.0};
  /// Latest scan passes right-line geometric quality checks.
  bool right_heading_valid_{false};
  /// Consecutive accepted fits have a consistent odom-frame direction for 0.5 steady seconds.
  bool right_heading_stable_{false};
  /// True during an uninterrupted series of accepted selected-wall fits.
  bool right_heading_tracking_{false};
  /// Odom-frame wall direction (rad) at the start of the current consistency interval.
  double right_heading_anchor_{0.0};
  /// Steady-clock start of the current wall-direction consistency interval.
  std::chrono::steady_clock::time_point right_heading_since_{};
  /// Steady-clock throttle for initial wall-angle diagnostic logs.
  std::chrono::steady_clock::time_point heading_log_time_{};
  /// Initial heading-fit side: right by default; wall-guided sessions select left.
  distance_controller::WallHeadingWindow wall_direction_window_;  ///< Stopped scan evidence.
  bool wall_target_frozen_{false};  ///< Odom turn target no longer follows individual scans.
  bool wall_verifying_{false};      ///< Post-turn fresh scan verification phase.
  double frozen_wall_yaw_{0.0};     ///< Frozen odom heading in radians.
  unsigned wall_refinements_{0};    ///< At most two verified target refinements.
  /// @brief Run multi-scan alignment.
  /// @param current_time ROS time used for stopped dwell.
  /// @return True while initialization owns the control tick.
  bool handle_frozen_alignment(const rclcpp::Time & current_time);
  /// @brief Commit or refine from fresh stopped scans.
  /// @param yaw Current odom yaw in radians.
  /// @return True while initialization owns the control tick.
  bool verify_frozen_alignment(double yaw);
  bool robust_wall_heading_{false};          ///< Opt-in dominant-line heading selection.
  std::string alignment_wall_{
    "right"};  ///< Initial heading-fit side; left is used by wall sessions.
  /// Selected-wall fitting half-width (rad) around body +pi/2 or -pi/2.
  double wall_heading_half_angle_{0.5235987755982988};
  /// Minimum observed tangent extent (m); prevents a short cluster defining heading.
  double wall_heading_min_span_{0.18};
  /// Maximum perpendicular line-fit RMS (m); no automatic fallback to odom zero.
  double wall_heading_max_rms_{0.012};

  /// Latest scan left window passed coverage, dispersion and distance checks; logging only.
  bool left_wall_valid_{false};
  /// Latest scan right window passed coverage, dispersion and distance checks; logging only.
  bool right_wall_valid_{false};
  /// Steady receipt time of the last frame transformed for wall observation, for log freshness.
  std::chrono::steady_clock::time_point wall_observation_time_{};

  /// Scan-to-body transform cache; used only for the fixed laser mounting in scene 2.
  std::unique_ptr<tf2_ros::Buffer> scan_tf_buffer_{};
  /// Subscribes to TF and fills scan_tf_buffer_; owns its default TF listener thread.
  std::shared_ptr<tf2_ros::TransformListener> scan_tf_listener_{};
  /// Scene-2 filtered LaserScan input with sensor-data QoS.
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_{};
  /// Body frame used for side-window selection and horizontal distance, normally base_link.
  std::string base_frame_{"base_link"};
  /// True after centered pose and standstill acceptance; disables initial scan gating thereafter.
  bool centering_complete_{false};
  /// True only when the latest scan passed transform, timestamp and window checks.
  bool scan_valid_{false};
  /// True after at least one valid scan; subsequent loss during preparation latches stop.
  bool accepted_scan_{false};
  /// Last accepted message timestamp in nanoseconds; rejects repeated or reversed scan stamps.
  int64_t last_scan_stamp_ns_{0};
  /// Steady receipt time of the latest accepted scan; valid after accepted_scan_.
  std::chrono::steady_clock::time_point last_scan_time_{};
  /// Estimated positive horizontal distances (m) from body origin to left and right walls.
  double left_wall_{0.0};
  /// Latest accepted right-wall horizontal distance from body origin (m).
  double right_wall_{0.0};
  /// Latest accepted rearward body-x distance from base_frame_ origin to rear wall (m).
  double rear_wall_{0.0};
  /// Desired rear-wall distance from body origin, not rear bumper clearance (m).
  double rear_target_distance_{0.28};
  /// Minimum accepted rear-wall distance from body origin (m); below it scan gating stops.
  double rear_min_distance_{0.22};
  /// Rear window half-width around body +/-pi (rad); narrower than side windows.
  double rear_window_half_angle_{0.08726646259971647};
  /// True while centered yaw and low measured velocities remain uninterrupted.
  bool centering_settling_{false};
  /// Node-clock beginning of centered standstill, valid while centering_settling_.
  rclcpp::Time centering_settle_start_{};
  /// Monotonic preparation stages; intermittent fit loss never restarts a stage deadline.
  enum class PreparationStage { WaitingForWall, Aligning, Positioning };
  /// Current preparation stage; evaluated by handle_centering_guard() on every eligible tick.
  PreparationStage preparation_stage_{PreparationStage::WaitingForWall};
  /// Steady-clock entry time of the current stage; valid after preparation_started_.
  std::chrono::steady_clock::time_point preparation_stage_start_{};
  /// Maximum initial scan/TF/selected-wall quality acquisition duration (steady seconds).
  double wall_measurement_timeout_{20.0};
  /// Maximum rotation and aligned-standstill duration after measurement acceptance (steady seconds).
  double alignment_timeout_{30.0};
  /// Maximum centering/rear-distance and standstill duration after alignment (steady seconds).
  double positioning_timeout_{30.0};
  /// Latest on_scan() rejection reason, used in waiting/fault logs; no motion policy encoded here.
  std::string scan_rejection_reason_{"no scan received"};

  /**
   * @brief Return the stable log identifier of the current preparation stage.
   * @return Static stage label read by preparation fault/transition logging.
   * @note Does not change state or clocks.
   */
  const char * preparation_stage_name() const;

  /**
   * @brief Return the independent deadline of the current preparation stage.
   * @return Positive steady-clock seconds read by handle_centering_guard().
   * @note Limits are startup parameters validated by configure_centering().
   */
  double preparation_stage_timeout() const;

  /**
   * @brief Advance preparation stage without restarting deadlines on transient fit loss.
   * @param[in] now Steady time from handle_centering_guard(); copy to the stage clock only on transition.
   * @note Reads accepted scan/fit/alignment flags and logs stage entry; never commands motion.
   */
  void update_preparation_stage(const std::chrono::steady_clock::time_point & now);

  /**
   * @brief Latch a preparation fault, clear control histories and publish stop.
   * @param[in] reason Stable failure identifier from handle_centering_guard(); read for the log, unchanged.
   * @param[in] now Steady time from handle_centering_guard(); read elapsed stage/total durations, unchanged.
   * @note Logs current stage, scan reason and selected-wall quality; restart is required after failure.
   */
  void fail_preparation(const char * reason, const std::chrono::steady_clock::time_point & now);

  /// True after first initial-adjustment guard entry records pose and steady start time.
  bool preparation_started_{false};
  /// Initial preparation odom position (m), used only to limit total displacement.
  double preparation_x_{0.0};
  /// Initial preparation odom y (m), paired with preparation_x_ for the travel bound.
  double preparation_y_{0.0};
  /// Steady-clock start of initial adjustment, independent of simulation pauses.
  std::chrono::steady_clock::time_point preparation_start_{};
  /// Proportional lateral offset gain (1/s); positive left-minus-right error commands left.
  double centering_gain_{0.5};
  /// Maximum initial combined planar command (m/s), additionally capped by max_speed_.
  double centering_max_speed_{0.03};
  /// Allowed center offset and rear-target error, meters.
  double centering_tolerance_{0.01};
  /// Side-window angular half-width around body +/-pi/2, radians.
  double side_window_half_angle_{0.17453292519943295};
  /// Valid horizontal wall-distance interval from body origin, meters.
  double side_min_distance_{0.18};
  /// Maximum accepted horizontal side-wall distance from body origin (m).
  double side_max_distance_{1.5};
  /// Maximum median absolute deviation of a side's body-y distances, meters.
  double side_max_mad_{0.03};
  /// Maximum scan receipt/stamp age during preparation, seconds.
  double scan_timeout_{0.5};
  /// Maximum preparation duration (steady seconds) and displacement from initial pose (m).
  double preparation_timeout_{60.0};
  /// Maximum preparation displacement from the first observed odom pose (m).
  double preparation_max_travel_{0.20};

  /// Constructor sets true for scene 2 only; gates all new heading behavior.
  bool heading_control_enabled_{false};
  /// True after initial yaw acceptance and standstill; latched until node restart.
  bool initial_alignment_complete_{false};
  /// True while initial yaw and measured speed acceptance remain uninterrupted.
  bool alignment_settling_{false};
  /// Node-clock beginning of the current initial standstill interval; valid while settling.
  rclcpp::Time alignment_settle_start_{};
  /// Positive proportional yaw gain (1/s), read by both heading-control paths.
  double heading_gain_{1.0};
  /// Positive symmetric angular command cap (rad/s); stops bypass limiting.
  double max_yaw_rate_{0.25};
  /// Absolute heading-error tolerance (rad), used at startup, during motion and at completion.
  double heading_tolerance_{0.01};
  /// Above this absolute yaw error (rad), translation pauses for rotation-only recovery.
  double translation_pause_angle_{0.15};
  /// Continuous initial standstill duration in node-clock seconds.
  double alignment_settle_duration_{0.5};

  /// Per-tick inputs from execute_current_segment() and command outputs retained for log_control_state().
  struct ControlDiagnostics
  {
    double wz_robot{};     ///< Published body yaw rate (rad/s), written by command calculation.
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
   * @param[in] data Read the object passed by execute_current_segment() to print this tick's values.
   * execute_current_segment() fills the pose, errors, and PID interval; compute_and_publish_command()
   * fills the seven velocity fields before this call. log_control_state() only reads
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
   * @note Scene 2 also rejects stale/nonincreasing header stamps and validates frame continuity during recovery. Receipt deadlines use the steady clock. Scene 2 rejects nonfinite consumed values or quaternion squared norm more than 0.01 from unity, latches a fault, and publishes stop.
   */
  void on_odom(nav_msgs::msg::Odometry::SharedPtr msg);

  /**
   * @brief Stop when odometry is missing or reception has timed out.
   *
   * @par Check odometry feedback
   * Keep sending zero velocity until the first odometry message arrives.
   * If no update arrives within 0.5 s, reset PID and stop; scene 2 enters bounded recovery, scene 1 latches.
   * Otherwise, allow the current timer callback to continue.
   *
   * @param[in] now Steady-clock time captured by on_timer(). Read it to calculate
   * the time since last_odom_time_, recorded by on_odom() when feedback is handled.
   * This function does not change on_timer()'s time variable.
   * @param[in] should_log Flag calculated by on_timer() from its logging interval.
   * Read it to decide whether to print the waiting message; do not write it back.
   * @return True to end this timer callback; false to continue control.
   * @note Scene 2 stops for up to 2 s to qualify fresh stopped feedback; otherwise latches or exits with a verified base watchdog. Scene 1 retains immediate latching.
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
   * @note This check also runs during settling and dwell, when PID is not updated. A large forward interval also resets initial settling and restarts completed-segment dwell. Scene 2 publishes stop when node time does not advance.
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
   * @param[in] ex Odom-axis x control error (m) from execute_current_segment(), after apply_step_feedback() selects position or laser feedback.
   * Pass it to initialize_pid() when history is missing; do not modify the input.
   * @param[in] ey Odom-axis y control error (m) from execute_current_segment(), after apply_step_feedback() selects position or laser feedback.
   * Pass it to initialize_pid() when history is missing; do not modify the input.
   * @param[in] current_time ROS time captured by on_timer(). Read it to initialize
   * PID history or calculate the interval since last_pid_time_. Store it in
   * last_pid_time_ when appropriate; do not modify on_timer()'s time variable.
   * @param[out] pid_dt Write the interval in seconds into execute_current_segment()'s data.pid_dt
   * field directly through this reference; its incoming value is not used. Only a false
   * return makes the result valid for PID calculation. execute_current_segment() passes the same data object
   * to compute_and_publish_command(), which forwards this interval to
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
   * @param[in,out] data Reference to the local object created by execute_current_segment().
   * Read data.ex, data.ey, data.yaw, and data.pid_dt, which execute_current_segment() fills.
   * Write PID output into data.vx_odom/data.vy_odom, copy it into
   * data.vx_odom_raw/data.vy_odom_raw before limiting, and update
   * data.vx_odom/data.vy_odom with the limited values. Write the converted
   * command into data.vx_robot/data.vy_robot and the yaw command into data.wz_robot. All writes update the same object
   * in execute_current_segment(), which then passes it to log_control_state(). The velocity
   * results are available after this function returns; data.x/data.y stay unchanged.
   * @note Run only after feedback and PID timing checks pass. Command publication
   * is independent of the logging rate. Updates PID and acceleration history; scene 2 adds a bounded yaw correction outside heading_tolerance_, while scene 1 keeps angular velocity zero.
   */
  void compute_and_publish_command(ControlDiagnostics & data);

  /**
   * @brief Coordinate one control tick from feedback checks to stopping or tracking.
   *
   * @par Control tick
   * Check latched faults, feedback age, and node time before target initialization.
   * Gate initial rotation and laser centering before recording the current pose as A; then execute the selected route segments. In scene 2, recover heading and revoke drifted dwell before handling completion, settling, and PID timing.
   * Only routine logging is throttled; command calculation runs on each eligible tick.
   *
   * @note Mutates route and PID state through helpers, publishes velocity, and may shut down the ROS context at route completion. A wall timer continues firing when simulation time pauses.
   */
  void on_timer();

  /// Result of a single nonblocking segment tick; only Completed permits route advancement.
  enum class SegmentResult { Running, Completed, Failed };

  /**
   * @brief Advance only the current segment by one control tick.
   * @param[in] current_time Node time from on_timer(); read/copy into PID, settling and dwell clocks.
   * @param[in] should_log Log cadence from on_timer(); read to emit diagnostics, unchanged.
   * @return Running during tracking/settling/dwell, Completed after dwell, or Failed on invalid state.
   * @note Initializes the target once, publishes commands and updates PID/segment state;
   * never selects another segment or shuts down ROS. on_timer() owns those decisions.
   */
  SegmentResult execute_current_segment(const rclcpp::Time & current_time, bool should_log);

  /**
   * @brief Advance the route after the executor reports Completed.
   * @note Called only by on_timer(); clears segment/PID state for the next tick,
   * or publishes stop and shuts down ROS when the route list is exhausted.
   */
  void advance_route();

  /**
   * @brief Keep an accepted segment stopped until its configured dwell finishes.
   * @param[in] current_time Node time from execute_current_segment(); read against
   * dwell_start_time_ and the current motion.dwell; no input is modified.
   * @return True only after accepted standstill and complete dwell; false otherwise.
   * @note Publishes stop without advancing route state. Heading/position recovery is checked first.
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

  /**
   * @brief Load and validate the selected sequence of relative displacements.
   *
   * @par Route selection
   * Replace segments_ with the selected route before control interfaces are created.
   * Displacements accumulate in the fixed route frame, not the changing body frame. Scene 2 reads positive finite forward_distance and lateral_distance and a connected route list starting at A.
   *
   * @param[in] scene_number Scene number forwarded by DistanceController() from main(). Read it to choose the route stored in segments_ for initialize_segment_target(); the caller value is unchanged.
   * @throws std::invalid_argument If the scene is unsupported, a displacement is nonfinite, or a scene-2 distance is not positive, or route IDs are unknown/disconnected.
   * @note Scene 2 uses adjustable nominal distances; no wall clearance or obstacle detection is provided.
   */
  void select_waypoints(int scene_number);

  /**
   * @brief Extract planar yaw from an odometry orientation.
   *
   * @par Orientation conversion
   * Convert the quaternion to roll, pitch, and yaw and return only yaw.
   *
   * @param[in] q Orientation from last_odom_.pose.pose.orientation passed by execute_current_segment(), handle_initial_alignment(), check_completion(), or initialize_segment_target(). Read it for conversion without changing the stored feedback.
   * @return Yaw in radians relative to odom, used by execute_current_segment() for heading acceptance and command calculation, or by initialize_segment_target() for scene-1 route_yaw_.
   * @note The caller must supply a valid orientation; this helper does not validate or normalize the quaternion.
   */
  double quaternion_to_yaw(const geometry_msgs::msg::Quaternion & q);

  /**
   * @brief Report initialization measurements, errors and settling state at the log cadence.
   * @par Initialization progress
   * Called by handle_centering_guard() after accepting fresh scan data. Read pose from
   * on_odom(), wall measurements from on_scan(), and alignment/positioning state from
   * their handlers. Log meters, radians and configured thresholds without changing control.
   * @par Output
   * Progress is written to the ROS logger; this function has no return value.
   * @note Settling flags describe the preceding tick; pre-alignment heading uses the selected wall.
   */
  void log_preparation_progress();

  /**
   * @brief Print fixed route waypoints after initial centering records A.
   * @par Planned pose list
   * Read route_x_, route_y_, route_yaw_ and heading_reference_ captured by initial
   * alignment/centering, and segments_ supplied by configure_route_steps(). Print
   * cumulative goals in the odometry header frame, meters and radians. Stop the
   * preview before a feedback-dependent endpoint; execution logs its actual target.
   * @par Output
   * Informational ROS log output only; this function has no return value.
   * @note Called after initial centering records A; does not modify targets or control state.
   */
  void log_route_waypoints() const;

  /**
   * @brief Freeze the current target from the route origin and cumulative displacement.
   *
   * @par Target setup
   * Called by execute_current_segment() when target_initialized_ is false. Capture route_x_, route_y_,
   * once from last_odom_ (after centering in scene 2). Set route_yaw_
   * to heading_reference_ in scene 2 or measured yaw in scene 1. Sum segments through the current index and
   * rotate the sum into odom and store target_x_, target_y_, and target_initialized_.
   *
   * @note An out-of-range index logs and returns; execute_current_segment() then reports Failed. Actual segment stopping error is not accumulated into later targets.
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
   * @param[in] ex Odom x error (m) from execute_current_segment() data.ex, forwarded by compute_and_publish_command(); read for the x-axis PID and retain in prev_error_x_ without changing the input.
   * @param[in] ey Odom y error (m) from execute_current_segment() data.ey, forwarded by compute_and_publish_command(); read for the y-axis PID and retain in prev_error_y_ without changing the input.
   * @param[in] dt Positive node-clock interval (s), validated by handle_pid_timing() and passed as data.pid_dt by compute_and_publish_command(); read for integration and differentiation.
   * @param[out] vx_odom Write raw x velocity (m/s, odom) into execute_current_segment() data.vx_odom via compute_and_publish_command(). Incoming value is unused; on return the caller copies it to data.vx_odom_raw before limiting.
   * @param[out] vy_odom Write raw y velocity (m/s, odom) into execute_current_segment() data.vy_odom via compute_and_publish_command(). Incoming value is unused; on return the caller copies it to data.vy_odom_raw before limiting.
   * @note Updates integral and previous-error members even when ki_ or kd_ is zero; does not publish or enforce speed limits.
   */
  void compute_pid(double ex, double ey, double dt, double & vx_odom, double & vy_odom);

  /**
   * @brief Limit planar speed while preserving the velocity direction.
   *
   * @par Speed limit
   * Scale both components equally when their norm exceeds the smaller of max_speed_ and the active segment speed cap.
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
   * @param[in] dt Positive interval (s) validated by handle_pid_timing(), written into execute_current_segment()'s data.pid_dt, and forwarded by compute_and_publish_command(); read to bound the change, with no writeback.
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
   * @param[in] yaw Robot yaw relative to odom (rad), extracted in execute_current_segment() and forwarded as data.yaw by compute_and_publish_command(); read for the inverse rotation.
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
   * @param[in] ex Odom x error (m) calculated by execute_current_segment() and forwarded by handle_pid_timing(); copy to prev_error_x_ for the next compute_pid() derivative.
   * @param[in] ey Odom y error (m) calculated by execute_current_segment() and forwarded by handle_pid_timing(); copy to prev_error_y_ for the next compute_pid() derivative.
   * @param[in] current_time Node time captured by on_timer() and forwarded by handle_pid_timing(); copy into last_pid_time_ for its next interval calculation. The caller time remains unchanged.
   * @note Initialization itself does not publish; handle_pid_timing() sends zero and ends this tick.
   */
  void initialize_pid(double ex, double ey, const rclcpp::Time & current_time);

  /**
   * @brief Mark the segment complete after continuous position and speed acceptance.
   *
   * @par Verify standstill
   * Require position error below 0.01 m, planar feedback speed below 0.01 m/s, and
   * absolute yaw rate below 0.02 rad/s for at least 0.5 node-clock seconds. Scene 2 also requires yaw error within heading_tolerance_.
   * Start or reset settling_. Route arrivals set segment_completed_ and dwell_start_time_; initial centering uses its own settling state.
   *
   * @param[in] ex Odom-axis x control error (m) selected by apply_step_feedback() and passed by execute_current_segment(); read for the joint arrival test without modifying the caller error.
   * @param[in] ey Odom-axis y control error (m) selected by apply_step_feedback() and passed by execute_current_segment(); read for the joint arrival test without modifying the caller error.
   * @param[in] current_time Node time captured by on_timer(); read for settling duration and copy to settle_start_time_ or dwell_start_time_ on state transitions. handle_completed_segment() later reads dwell_start_time_; the input is unchanged.
   * @note execute_current_segment() publishes zero before calling. Feedback speed comes from last_odom_.twist in the child frame; scene 2 also requires heading error relative to heading_reference_ within heading_tolerance_.
   */
  void check_completion(double ex, double ey, const rclcpp::Time & current_time);

  /// Owns the feedback subscription that invokes on_odom().
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_{};
  /// Owns the 50 ms wall timer; on_timer() still runs while simulation time is paused.
  rclcpp::TimerBase::SharedPtr timer_{};
  /// Publishes body-frame Twist commands; unused axes stay zero; scene 2 also publishes yaw correction.
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_{};

  /// Steady-clock time seeded at construction, updated by on_odom(); timeout checked only after reception.
  std::chrono::steady_clock::time_point last_odom_time_{std::chrono::steady_clock::now()};
  /// Latest copied feedback: pose assumed in odom, twist assumed in child_frame_id (base_link).
  nav_msgs::msg::Odometry last_odom_{};
  /// True after on_odom() first stores feedback; false keeps the robot waiting at zero velocity.
  bool received_odom_{false};
  /// Steady-clock time seeded at construction and updated for routine log opportunities; throttles at 1 s.
  std::chrono::steady_clock::time_point last_log_time_{std::chrono::steady_clock::now()};

  /// Ordered relative displacements (m) in the fixed route frame, selected at construction.
  std::vector<distance_controller::RouteSegment> segments_{};
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

  /// True after unrecovered feedback timeout, backwards time, or invalid scene-2 odometry; stops until node restart.
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

  /// True after the fixed route frame is initialized: centered feedback/captured wall heading in scene 2, initial feedback in scene 1.
  bool route_initialized_{false};
  /// Fixed route origin x in odom (m); captured after centering in scene 2, valid after route_initialized_.
  double route_x_{0.0};
  /// Fixed route origin y in odom (m); captured after centering in scene 2, valid after route_initialized_.
  double route_y_{0.0};
  /// Fixed route heading in odom (rad): captured wall heading for scene 2, initial measured yaw for scene 1.
  double route_yaw_{0.0};

  /// Node-clock start of extra dwell after completion; restarted after a large forward interval.
  rclcpp::Time dwell_start_time_{};
  /// Extra dwell after settling in node-clock seconds; nonnegative parameter, default 1 s.
  double dwell_duration_{1.0};

  /**
   * @brief Transform scan rays and update independent wall windows.
   * @par Wall measurement
   * Separate side/rear initialization validity from front and side observations used by steps.
   * @param[in] scan Header/ranges validated by on_scan(); read ray geometry, unchanged.
   * @param[in] mounting Fixed transform validated by on_scan(); read to convert rays to base_frame.
   * @note Updates distance/quality/receipt state; selected-wall fitting is initial alignment only.
   */
  void measure_wall_windows(
    const sensor_msgs::msg::LaserScan & scan,
    const geometry_msgs::msg::TransformStamped & mounting);
  /**
   * @brief Validate front-sector coverage and median spread.
   * @par Front measurement
   * Unlike side initialization limits, do not apply side_min_distance to front-body coordinates.
   * @param[in,out] points Positive body-x endpoints collected by measure_wall_windows(); read and
   * sort the same local vector to obtain its median; no member vector is retained.
   * @param[in] samples Total rays in the front window from measure_wall_windows(); read for coverage.
   * @param[out] distance Write the median body-x distance to front_wall_; usable only on true.
   * @return True for at least six points, 50 percent coverage and accepted median deviation.
   * @note Does not infer wall identity or clearance of the entire robot footprint.
   */
  bool estimate_front(std::vector<double> & points, std::size_t samples, double & distance) const;

  /// ROS request type; acceptance and execution are separate events.
  using StepService = distance_controller::srv::ExecuteStep;
  /// Optional scene-2 service; callbacks only select data, timer owns normal control publication.
  rclcpp::Service<StepService>::SharedPtr step_service_{};
  /// True enables one-step-at-a-time continuation and preserves the node after arrival.
  bool manual_mode_{false};
  /// True pauses after initial A capture before executing the first configured segment.
  bool start_paused_{false};
  /// True while waiting for a resume/new-step request; linear command remains zero.
  bool manual_waiting_{false};
  /// Finish request is consumed by the next timer tick after the service response.
  bool finish_requested_{false};
  /// Label of the last completed endpoint; never used as a coordinate lookup.
  std::string current_waypoint_{"A"};
  /// Planned endpoint in odom meters; ordinary fixed-distance residuals do not alter it.
  double planned_x_{};
  /// Planned endpoint odom y, used with planned_x_ by target initialization.
  double planned_y_{};
  /// Steady start of active segment, including heading recovery, settling and dwell.
  std::chrono::steady_clock::time_point step_started_{};
  /// Accumulated odom path length since active-step initialization, meters.
  double step_travel_{};
  /// Last observed active-step odom x/y, meters, for accumulating path length.
  double step_last_x_{};
  /// Last observed active-step odom y, meters; paired with step_last_x_.
  double step_last_y_{};
  /// Frontmost body coordinate (m in base_frame); -1 means unconfigured and forbids front goals.
  double front_body_extent_{-1.0};
  /// Front +/-5 degree median body-x wall position, meters; valid only with fresh front_wall_valid_.
  double front_wall_{};
  /// Validity of the front sector in the latest processed scan; independent of rear availability.
  bool front_wall_valid_{false};
  /// Latest structurally accepted scan stamp, including route scans without rear returns.
  int64_t observation_stamp_ns_{};

  /**
   * @brief Load named edges and per-edge policies before creating motion interfaces.
   * @par Configuration
   * Resolve standard AB/BC/CB/BA factories or custom two-letter edges with explicit dx/dy.
   * @param[in] forward_distance Distance from select_waypoints(); read for AB/BA defaults, meters.
   * @param[in] lateral_distance Distance from select_waypoints(); read for BC/CB defaults, meters.
   * @note Writes segments_ and manual settings; rejects invalid configuration before execution.
   */
  void configure_route_steps(double forward_distance, double lateral_distance);
  /**
   * @brief Apply one named edge's startup parameter overrides.
   * @par Policy selection
   * Read segments.ID.* parameters and validate directions, feedback choice and bounds.
   * @param[in] name Two-letter ID from configure_route_steps(); read to form the parameter prefix.
   * @param[in,out] step Factory/default fields from configure_route_steps(); replace fields with
   * validated overrides in the same object, subsequently copied into segments_.
   * @note Parameter declarations are startup-only, not a live motion command interface.
   */
  void configure_segment(const std::string & name, distance_controller::RouteSegment & step);
  /**
   * @brief Create the optional manual-step service after startup validation.
   * @par Interface creation
   * Register ~/step only when scene-2 manual mode is enabled.
   * @note Called by DistanceController(); does not start motion.
   */
  void configure_step_interface();
  /**
   * @brief Accept a validated idle step, history-return batch or management request.
   * @par Serialized request
   * Reject busy/faulted/uninitialized states and never queue surprise commands behind an active step.
   * @param[in] request ROS service input; read an action, step parameters or a history destination.
   * @param[out] response Write acceptance and reason for the service client; acceptance is not arrival.
   * @note Mutates route/wait state after validation; return replaces pending steps, cancel latches stop.
   */
  void on_step_request(
    const StepService::Request::SharedPtr request, StepService::Response::SharedPtr response);
  /**
   * @brief Convert a service request into one bounded planar step.
   * @par Request conversion
   * Resolve direction, fixed axes, goal kind and limits without changing active control state.
   * @param[in] request Candidate supplied by on_step_request(); read all fields, unchanged.
   * @return Validated segment for on_step_request() to install; throws on unsupported/invalid input.
   * @note Turns are explicitly unsupported. Reads current odom only for default travel allowance.
   */
  distance_controller::RouteSegment make_requested_step(const StepService::Request & request);
  /**
   * @brief Keep manual idle stopped in translation while holding the persistent heading.
   * @par Idle ownership
   * Called after feedback/time/initialization guards; check waiting-position drift, then hold yaw.
   * @return True when this tick is handled by idle/finish; false to execute the active segment.
   * @note Never captures a new heading from drift; idle does not resume automatically.
   */
  bool handle_manual_wait();
  /**
   * @brief Report current pose and active/idle/fault state to the service client.
   * @par Observation
   * Format latest odometry and persistent heading without modifying control state.
   * @return Human-readable status; pose is unavailable until received_odom_.
   * @note Caller on_step_request() uses the result for status and acceptance responses.
   */
  std::string step_status();
  /**
   * @brief Record a completed endpoint after settling and dwell.
   * @par Endpoint evidence
   * Log actual stopped odom pose and planned label; sensor-based goals become the new planned anchor.
   * @note Called by advance_route(); writes planned_x_/planned_y_ and current_waypoint_.
   */
  void record_step_endpoint();
  /**
   * @brief Apply bounded execution guards and the selected arrival measurement.
   * @par Active feedback
   * Check steady timeout/path length and only the laser windows requested by this segment.
   * @param[in,out] data Pose/errors from execute_current_segment(); replace errors with front
   * clearance/side-centering errors when selected, for the same caller's PID and completion checks.
   * @return False after a latched fault/stop; true when errors are valid for this tick.
   * @note Does not change heading reference. Laser faults never silently fall back to odom.
   */
  bool apply_step_feedback(ControlDiagnostics & data);
  /**
   * @brief Bound front-approach speed after the ordinary acceleration calculation.
   * @par Approach deceleration
   * Use remaining clearance error to cap positive body-x speed near the wall.
   * @param[in,out] cmd Command from compute_and_publish_command(); read/limit body x in the
   * same object before publication; other components remain unchanged.
   * @note Applies only to front-wall goals; stricter deceleration can bypass the acceleration ramp.
   */
  void apply_front_speed_bound(geometry_msgs::msg::Twist & cmd);
  /**
   * @brief Latch a step fault and publish zero without advancing the route.
   * @par Fault ownership
   * Reset PID/settling before recording the failure reason.
   * @param[in] reason Description from apply_step_feedback()/on_step_request(); read for logging.
   * @note Restart is required; no automatic retry or fallback.
   */
  void fail_step(const std::string & reason);
  distance_controller::RouteHistory history_{};  ///< Current-process completed traversal audit.
  distance_controller::RecordedPose
    step_start_pose_{};  ///< Actual pose before the active step moves.
  distance_controller::RecordedPose
    waiting_pose_{};                ///< Last accepted stopped pose for continuation.
  bool waiting_pose_valid_{false};  ///< Set after A or a verified endpoint; never from a request.
  double resume_position_tolerance_{0.02};  ///< Maximum displacement while waiting, meters in odom.
  std::size_t return_remaining_{};  ///< Return legs left in the accepted batch; zero otherwise.

  /** @brief Read the current validated odom pose for history and continuation checks.
   * @return A copy of last_odom_ position and yaw, consumed by target/history methods.
   * @note Caller must first pass the existing odom guard; does not redefine heading.
   */
  distance_controller::RecordedPose current_recorded_pose();
  /** @brief Stop and latch when the robot has moved from its last accepted wait pose.
   * @par Continuation gate
   * Compare current odom with waiting_pose_; do not automatically relocate or recenter.
   * @return True when displacement is within resume_position_tolerance_; false after fault.
   * @note Called by on_step_request(), handle_manual_wait() and reverse-leg startup.
   */
  bool check_continuation_position();
  /** @brief Validate a return plan before replacing the unexecuted route tail.
   * @param[in] request Service input from on_step_request(); read count or destination visit.
   * @note Sets return_remaining_ and segments_ only after full validation; pending route is replaced.
   */
  void prepare_history_return(const StepService::Request & request);
  /** @brief Commit the completed scene-2 step before any route index advances.
   * @return True on success or scene-1 bypass; false after a history consistency fault.
   * @note advance_route() supplies current members; copies poses/policy to history_ and updates wait pose.
   */
  bool record_completed_history();
  /** @brief Adopt a stopped intermediate pose before any wall preparation.
   * @param[in] current_time ROS time from on_timer(), used for continuous stopped qualification.
   * @return True while startup is consuming this tick; false after adoption or when disabled.
   * @note Reads validated odom; writes heading_reference_ and a fresh local history origin A.
   * Clears pending routes and enters WAITING. Never restores old history. Optional planned yaw
   * is subsequently held by handle_manual_wait(); differences over 0.10 rad latch a fault.
   */
  bool handle_current_pose_start(const rclcpp::Time & current_time);
  bool adopt_planned_heading_{false};  ///< Opt-in: hold the route session's planned odom yaw.
  double planned_heading_{};  ///< Planned odom yaw (rad); adoption rejects errors above 0.10 rad.
  bool adopt_current_pose_{false};  ///< Startup-only opt-in for scene-2 manual intermediate stops.
};

#endif
