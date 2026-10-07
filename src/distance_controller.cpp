#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include <functional>
#include <chrono>

class DistanceController : public rclcpp::Node {
public:
    DistanceController() : 
        Node{"distance_controller"},
        last_odom_time_{std::chrono::steady_clock::now()},
        received_odom_{false},
        last_log_time_{std::chrono::steady_clock::now()}
    {
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered",
            10,
            std::bind(&DistanceController::on_odom, this, std::placeholders::_1)
        );

        timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            [this]() {
                on_timer();
            }
        );

        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel",
            10
        );

    }
private:

    void on_odom(nav_msgs::msg::Odometry::SharedPtr msg) {
        last_odom_ = *msg;
        last_odom_time_  = std::chrono::steady_clock::now();
        received_odom_ = true;
    }

    void on_timer() {
        publish_stop();
        auto now = std::chrono::steady_clock::now();
        bool should_log =
            std::chrono::duration<double>(now - last_log_time_).count() >= 1.0;

        if (should_log) {
            last_log_time_ = now;
        }

        if (!received_odom_)
        {
            if (should_log) {
                RCLCPP_INFO(get_logger(), "Waiting for odom...");
            }
            
            return;
        }

        double dt = std::chrono::duration<double>(now - last_odom_time_).count();
        if (dt > 0.5) {
            if (should_log) {
                RCLCPP_WARN(get_logger(), "Odom timeout: %.2f seconds since last update", dt);
            }
            
            return;
        }

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


    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    std::chrono::steady_clock::time_point last_odom_time_;
    nav_msgs::msg::Odometry last_odom_;
    bool received_odom_;
    std::chrono::steady_clock::time_point last_log_time_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DistanceController>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}