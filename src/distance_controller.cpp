#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include <functional>
#include <chrono>
#include <cstddef>
#include <array>
#include <cmath>
#include <algorithm>

// Control the first planar segment; stop while waiting, settling, completed, or faulted.
class DistanceController : public rclcpp::Node {
public:
    DistanceController() :
        Node{"distance_controller"},
        last_odom_time_{std::chrono::steady_clock::now()},
        received_odom_{false},
        last_log_time_{std::chrono::steady_clock::now()}
    {
        // Bind incoming odometry messages to on_odom.
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered",
            10,
            std::bind(&DistanceController::on_odom, this, std::placeholders::_1)
        );

        // Check feedback every 50 ms; this wall timer keeps running when simulation time pauses.
        timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            [this]() {
                on_timer();
            }
        );

        // Publish commanded base velocity, independently of the velocity reported by odometry.
        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel",
            10
        );

    }
private:

    // ===== Feedback reception =====
    void on_odom(nav_msgs::msg::Odometry::SharedPtr msg) {
        // Copy the latest feedback; the current single-threaded spin executes callbacks serially.
        last_odom_ = *msg;
        // Record the local receipt time using a clock unaffected by simulation pauses.
        last_odom_time_  = std::chrono::steady_clock::now();
        received_odom_ = true;
    }

    // ===== Periodic single-segment control pipeline =====
    void on_timer() {

        // --- 1. Keep a latched fault stopped until the node is restarted ---
        if (fault_latched_)
        {
            publish_stop();
            return;
        }
        // --- 2. Check feedback age and routine log timing using steady time ---
        auto now = std::chrono::steady_clock::now();
        // Throttle logs only; feedback checks and command publication still run every tick.
        bool should_log =
            std::chrono::duration<double>(now - last_log_time_).count() >= 1.0;

        // Throttle routine logs; state transitions and fault reports are logged separately.
        if (should_log) {
            last_log_time_ = now;
        }

        // Do not treat the default-constructed message as feedback before the first reception.
        if (!received_odom_)
        {
            if (should_log) {
                RCLCPP_INFO(get_logger(), "Waiting for odom...");
            }
            publish_stop();
            return;
        }

        // Convert elapsed time to seconds; dt here is feedback age, not the PID timestep.
        double dt = std::chrono::duration<double>(now - last_odom_time_).count();
        if (dt > 0.5) {
            RCLCPP_WARN(get_logger(), "Odom timeout: %.2f seconds since last update", dt);
            fault_latched_ = true;
            reset_pid();
            publish_stop();
            return;
        }

        // --- 3. Observe ROS time before both tracking and settling ---
        auto current_time = this->now();

        if (!ros_time_initialized_) {
            last_ros_time_ = current_time;
            ros_time_initialized_ = true;
        } else {
            const double ros_dt =
                (current_time - last_ros_time_).seconds();

            // Update before any early return so the next tick uses this observation.
            last_ros_time_ = current_time;

            if (ros_dt < 0) {
                fault_latched_ = true;
                settling_ = false;
                reset_pid();

                RCLCPP_ERROR(
                    get_logger(),
                    "ROS time moved backwards: ros_dt=%.6f",
                    ros_dt
                );

                publish_stop();
                return;
            }

            // Drop accumulated settling time after a large jump; retry on a later tick.
            if (ros_dt > 0.2)
            {
                if (segment_completed_) {
                    dwell_start_time_ = current_time;
                }
                settling_ = false;
                reset_pid();

                publish_stop();
                return;
            }
        }

        // --- 4. Freeze the segment target once; never move it with the feedback ---
        if (!target_initialized_)
        {
            initialize_segment_target();
        }

        if (!target_initialized_)
        {
            publish_stop();
            return;
        }

        double x = last_odom_.pose.pose.position.x;
        double y = last_odom_.pose.pose.position.y;

        double yaw = quaternion_to_yaw(
            last_odom_.pose.pose.orientation
        );

        double ex = target_x_ - x;
        double ey = target_y_ - y;

        // --- 5. Keep a completed segment stopped; do not restart tracking ---
        if (segment_completed_)
        {
            publish_stop();
            const double dwell_elapsed =
                (current_time - dwell_start_time_).seconds();

            // Keep sending zero velocity during the dwell period.
            if (dwell_elapsed < dwell_duration_) {
                return;
            }
            ++current_segment_index_;
            if (current_segment_index_ >= segments_.size()) {
                RCLCPP_INFO(get_logger(), "Route completed.");
                rclcpp::shutdown();
                return;
            }

            // Prepare the next segment without changing the route reference.
            reset_pid();
            settling_ = false;
            segment_completed_ = false;
            target_initialized_ = false;
            return;
        }

        double position_error = std::sqrt(ex * ex + ey * ey);

        // --- 6. Stop inside position tolerance and verify continuous settling ---
        if (position_error < 0.01)
        {
            publish_stop();
            reset_pid();
            check_completion(
                ex,
                ey,
                current_time
            );

            return;
        }

        settling_ = false;

        // --- 7. Initialize PID history or validate its own integration interval ---
        if (!pid_initialized_)
        {
            initialize_pid(ex, ey, current_time);
            publish_stop();
            return;
        }

        double pid_dt =
            (current_time - last_pid_time_).seconds();

        // An unchanged timestamp must not advance PID history.
        if (pid_dt == 0.0)
        {
            publish_stop();
            return;
        }

        // Discard history after a backward clock jump.
        if (pid_dt < 0.0)
        {
            reset_pid();
            publish_stop();
            return;
        }

        // Discard history after an excessive control interval.
        if (pid_dt > 0.2)
        {
            reset_pid();
            publish_stop();
            return;
        }

        last_pid_time_ = current_time;

        // --- Compute PID candidates in the odom frame ---
        double vx_odom = 0.0;
        double vy_odom = 0.0;

        compute_pid(
            ex,
            ey,
            pid_dt,
            vx_odom,
            vy_odom
        );

        // --- Preserve raw candidates before limiting their magnitude ---
        double vx_odom_raw = vx_odom;
        double vy_odom_raw = vy_odom;

        limit_velocity(vx_odom, vy_odom);

        // --- Convert the limited odom velocity into the current body frame ---
        double vx_robot = 0.0;
        double vy_robot = 0.0;

        odom_to_robot_velocity(
            vx_odom,
            vy_odom,
            yaw,
            vx_robot,
            vy_robot
        );

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = vx_robot;
        cmd.linear.y = vy_robot;

        // Publish the limited body-frame velocity; unused Twist components default to zero.
        cmd_pub_->publish(cmd);



        if (should_log) {
            RCLCPP_INFO(
                get_logger(),
                "Pose: x=%.3f, y=%.3f, yaw=%.3f | "
                "Velocity: vx=%.3f, vy=%.3f, wz=%.3f | "
                "Target: x=%.3f, y=%.3f | "
                "Current segment index:%zu | "
                "Error: ex=%.3f, ey=%.3f | "
                "pid_dt=%.3f | "
                "integral=(%.3f, %.3f) | "
                "odom_raw=(%.3f, %.3f) | "
                "odom_limited=(%.3f, %.3f) | "
                "robot_cmd=(%.3f, %.3f)",
                x,
                y,
                yaw,
                last_odom_.twist.twist.linear.x,
                last_odom_.twist.twist.linear.y,
                last_odom_.twist.twist.angular.z,
                target_x_,
                target_y_,
                current_segment_index_,
                ex,
                ey,
                pid_dt,
                integral_x_, integral_y_,
                vx_odom_raw, vy_odom_raw,
                vx_odom, vy_odom,
                vx_robot, vy_robot
            );
        }

    }

    // Publish all six components as zero; logging zero velocity alone does not send a stop command.
    // ===== Stop-command output =====
    void publish_stop() {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.linear.y = 0.0;
        cmd.linear.z = 0.0;

        cmd.angular.x = 0.0;
        cmd.angular.y = 0.0;
        cmd.angular.z = 0.0;

        cmd_pub_->publish(cmd);
    }

    // ===== Route data type: relative forward/left displacement in meters =====
    struct Segment
    {
        double dx;
        double dy;
    };

    // ===== Coordinate helper: extract yaw in radians =====
    double quaternion_to_yaw(const geometry_msgs::msg::Quaternion &q)
    {
        tf2::Quaternion quat(
            q.x,
            q.y,
            q.z,
            q.w
        );

        double roll, pitch, yaw;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

        return yaw;
    }

    // ===== Target initialization: rotate displacement and freeze the odom target =====
    void initialize_segment_target() {
        if (current_segment_index_ >= segments_.size()) {
            RCLCPP_WARN(get_logger(), "Segment index out of range");
            return;
        }

        if (!route_initialized_) {
            route_x_ = last_odom_.pose.pose.position.x;
            route_y_ = last_odom_.pose.pose.position.y;
            route_yaw_ = quaternion_to_yaw(last_odom_.pose.pose.orientation);
            route_initialized_ = true;
        }

        double total_dx = 0.0;
        double total_dy = 0.0;
        for (std::size_t i = 0; i <= current_segment_index_; ++i) {
            total_dx += segments_[i].dx;
            total_dy += segments_[i].dy;

        }
        target_x_ =
            route_x_
            + std::cos(route_yaw_) * total_dx
            - std::sin(route_yaw_) * total_dy;

        target_y_ =
            route_y_
            + std::sin(route_yaw_) * total_dx
            + std::cos(route_yaw_) * total_dy;

        target_initialized_ = true;
    }
    // ===== PID reset: invalidate history before reinitialization =====
    void reset_pid() {
        integral_x_ = 0.0;
        integral_y_ = 0.0;
        prev_error_x_ = 0.0;
        prev_error_y_ = 0.0;

        pid_initialized_ = false;
    }
    // ===== PID calculation: independent axes; caller guarantees valid positive dt =====
    void compute_pid(
        double ex,
        double ey,
        double dt,
        double & vx_odom,
        double & vy_odom) {
        integral_x_ += ex * dt;
        integral_y_ += ey * dt;

        integral_x_ = std::clamp(integral_x_, -integral_limit_, integral_limit_);
        integral_y_ = std::clamp(integral_y_, -integral_limit_, integral_limit_);

        double derivative_x = (ex - prev_error_x_) / dt;
        double derivative_y = (ey - prev_error_y_) / dt;

        vx_odom = kp_ * ex
        + ki_ * integral_x_
        + kd_ * derivative_x;

        vy_odom = kp_ * ey
                + ki_ * integral_y_
                + kd_ * derivative_y;

        prev_error_x_ = ex;
        prev_error_y_ = ey;
    }
    // ===== Speed limiting: scale both axes equally to preserve direction =====
    void limit_velocity(
        double & vx,
        double & vy) {
        double speed = std::sqrt(vx * vx + vy * vy);
        if (speed > max_speed_)
        {
            double scale = max_speed_ / speed;

            vx *= scale;
            vy *= scale;
        }

    }
    // ===== Velocity conversion: odom to current body frame =====
    void odom_to_robot_velocity(
        double vx_odom,
        double vy_odom,
        double yaw,
        double & vx_robot,
        double & vy_robot) {
        vx_robot =
            std::cos(yaw) * vx_odom
            + std::sin(yaw) * vy_odom;

        vy_robot =
            -std::sin(yaw) * vx_odom
            + std::cos(yaw) * vy_odom;

    }

    // ===== PID initialization: seed current errors and node ROS time =====
    void initialize_pid(
        double ex,
        double ey,
        const rclcpp::Time &current_time)
    {
        prev_error_x_ = ex;
        prev_error_y_ = ey;

        last_pid_time_ = current_time;

        integral_x_ = 0.0;
        integral_y_ = 0.0;

        pid_initialized_ = true;
    }

    // ===== Completion check: position and feedback speed must remain within tolerance =====
    void check_completion(
        double ex,
        double ey,
        const rclcpp::Time &current_time)
    {
        double position_error =
            std::sqrt(ex * ex + ey * ey);

        if (position_error >= 0.01)
        {
            settling_ = false;
            return;
        }

        double vx = last_odom_.twist.twist.linear.x;
        double vy = last_odom_.twist.twist.linear.y;

        double linear_speed =
            std::sqrt(vx * vx + vy * vy);

        double angular_speed =
            std::abs(last_odom_.twist.twist.angular.z);

        if (linear_speed < 0.01 &&
            angular_speed < 0.02)
        {
            if (!settling_)
            {
                settle_start_time_ = current_time;
                // Log only when entering the stable interval, not on every timer tick.
                settling_ = true;
                RCLCPP_INFO(
                    get_logger(),
                    "Entering SETTLING state"
                );
            }
            else if (
                (current_time - settle_start_time_).seconds() >= 0.5)
            {
                // The caller keeps publishing zero after completion.
                segment_completed_ = true;
                dwell_start_time_ = current_time;
                RCLCPP_INFO(
                    get_logger(),
                    "Entering DONE state"
                );
            }
        }
        else
        {
            settling_ = false;
        }
    }

    // Keep the ROS interfaces alive for the lifetime of the node.
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    // Persist receipt time, latest feedback, reception flag, and log time across callbacks.
    std::chrono::steady_clock::time_point last_odom_time_;
    nav_msgs::msg::Odometry last_odom_;
    bool received_odom_;
    std::chrono::steady_clock::time_point last_log_time_;
    // ===== Route and segment state =====
    std::array<Segment, 10> segments_{{
        { 0.0,  1.0},
        { 0.0, -1.0},
        { 0.0, -1.0},
        { 0.0,  1.0},
        { 1.0,  1.0},
        {-1.0, -1.0},
        { 1.0, -1.0},
        {-1.0,  1.0},
        { 1.0,  0.0},
        {-1.0,  0.0}
    }};
    std::size_t current_segment_index_{0};
    double start_x_;
    double start_y_;
    double start_yaw_;

    double target_x_;
    double target_y_;

    bool target_initialized_{false};

    // ===== PID gains and history: current tuning uses only the P contribution =====
    double kp_{0.5};
    double ki_{0.0};
    double kd_{0.0};

    double integral_x_ = 0.0;
    double integral_y_ = 0.0;

    double prev_error_x_ = 0.0;
    double prev_error_y_ = 0.0;

    bool pid_initialized_ = false;

    double integral_limit_ = 0.5;

    double max_speed_ = 0.15;

    // initialize_pid assigns node-clock time before this value is subtracted.
    rclcpp::Time last_pid_time_;
    // ===== Fault and completion state =====
    bool fault_latched_ = false;

    // A separate ROS-time interval tracks continuous position and speed acceptance.
    bool settling_ = false;
    rclcpp::Time settle_start_time_;
    bool segment_completed_ = false;

    // ===== ROS-time observation, independent of whether PID runs this tick =====
    bool ros_time_initialized_{false};
    rclcpp::Time last_ros_time_;

    // Fixed reference for the entire route.
    bool route_initialized_{false};
    double route_x_{0.0};
    double route_y_{0.0};
    double route_yaw_{0.0};

    // Dwell timing after a segment has settled.
    rclcpp::Time dwell_start_time_;
    double dwell_duration_{3.0};
};

// ===== Process entry point =====
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DistanceController>();

    // Process ready callbacks after construction; source order does not determine callback order.
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
