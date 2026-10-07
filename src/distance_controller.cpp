#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include <functional>
#include <chrono>

// Monitor odometry and publish zero velocity; distance PID will be added later.
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

    void on_odom(nav_msgs::msg::Odometry::SharedPtr msg) {
        // Copy the latest feedback; the current single-threaded spin executes callbacks serially.
        last_odom_ = *msg;
        // Record the local receipt time using a clock unaffected by simulation pauses.
        last_odom_time_  = std::chrono::steady_clock::now();
        received_odom_ = true;
    }

    void on_timer() {
        // All current states command zero velocity; PID control will require state-specific output.
        publish_stop();
        auto now = std::chrono::steady_clock::now();
        // Throttle logs only; feedback checks and command publication still run every tick.
        bool should_log =
            std::chrono::duration<double>(now - last_log_time_).count() >= 1.0;

        // Share one log timestamp across all three branches; should_log retains its computed value.
        if (should_log) {
            last_log_time_ = now;
        }

        // Do not treat the default-constructed message as feedback before the first reception.
        if (!received_odom_)
        {
            if (should_log) {
                RCLCPP_INFO(get_logger(), "Waiting for odom...");
            }

            return;
        }

        // Convert elapsed time to seconds; dt here is feedback age, not the PID timestep.
        double dt = std::chrono::duration<double>(now - last_odom_time_).count();
        if (dt > 0.5) {
            if (should_log) {
                RCLCPP_WARN(get_logger(), "Odom timeout: %.2f seconds since last update", dt);
            }

            return;
        }

        // Convert the quaternion to angles in radians; orientation.z alone is not yaw.
        auto q = last_odom_.pose.pose.orientation;
        tf2::Quaternion quat(q.x, q.y, q.z, q.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        if (should_log) {
            RCLCPP_INFO(
                get_logger(),
                "Pose: x=%.3f, y=%.3f, yaw=%.3f | Velocity: vx=%.3f, vy=%.3f, wz=%.3f",
                last_odom_.pose.pose.position.x,
                last_odom_.pose.pose.position.y,
                yaw,
                last_odom_.twist.twist.linear.x,
                last_odom_.twist.twist.linear.y,
                last_odom_.twist.twist.angular.z
            );
        }

    }

    // Publish all six components as zero; logging zero velocity alone does not send a stop command.
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


    // Keep the ROS interfaces alive for the lifetime of the node.
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    // Persist receipt time, latest feedback, reception flag, and log time across callbacks.
    std::chrono::steady_clock::time_point last_odom_time_;
    nav_msgs::msg::Odometry last_odom_;
    bool received_odom_;
    std::chrono::steady_clock::time_point last_log_time_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DistanceController>();

    // Process ready callbacks after construction; source order does not determine callback order.
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
