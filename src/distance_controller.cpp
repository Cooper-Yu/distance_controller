#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include <functional>
#include <chrono>

// 当前阶段只监测里程计并发布零速度；距离 PID 将在此节点中继续实现。
class DistanceController : public rclcpp::Node {
public:
    DistanceController() :
        Node{"distance_controller"},
        last_odom_time_{std::chrono::steady_clock::now()},
        received_odom_{false},
        last_log_time_{std::chrono::steady_clock::now()}
    {
        // 消息到达后调用 on_odom；回调名称由订阅绑定决定。
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered",
            10,
            std::bind(&DistanceController::on_odom, this, std::placeholders::_1)
        );

        // 计划每 50 ms 检查一次反馈；wall timer 不随仿真时间暂停。
        timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            [this]() {
                on_timer();
            }
        );

        // 发布底盘速度要求，与里程计中的实际反馈速度相互独立。
        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel",
            10
        );

    }
private:

    void on_odom(nav_msgs::msg::Odometry::SharedPtr msg) {
        // 复制最新反馈供定时回调读取；当前单线程 spin 串行执行回调。
        last_odom_ = *msg;
        // 记录本机接收时刻，不使用可能暂停的仿真时间或消息时间戳。
        last_odom_time_  = std::chrono::steady_clock::now();
        received_odom_ = true;
    }

    void on_timer() {
        // 当前所有状态都输出零速度；加入 PID 后需按状态选择输出。
        publish_stop();
        auto now = std::chrono::steady_clock::now();
        // 只限制打印频率，反馈检查与速度发布仍每次回调执行。
        bool should_log =
            std::chrono::duration<double>(now - last_log_time_).count() >= 1.0;

        // 三个日志分支共用打印时刻；本次 should_log 的值不会因此改变。
        if (should_log) {
            last_log_time_ = now;
        }

        // 首条消息到达前，不把默认构造的消息当作真实反馈。
        if (!received_odom_)
        {
            if (should_log) {
                RCLCPP_INFO(get_logger(), "Waiting for odom...");
            }

            return;
        }

        // duration<double> 将间隔转换为秒；这里的 dt 是收包间隔，不是 PID 步长。
        double dt = std::chrono::duration<double>(now - last_odom_time_).count();
        if (dt > 0.5) {
            if (should_log) {
                RCLCPP_WARN(get_logger(), "Odom timeout: %.2f seconds since last update", dt);
            }

            return;
        }

        // orientation 是四元数；getRPY 得到弧度，不能直接把 orientation.z 当 yaw。
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

    // 六个分量明确置零，然后实际发送消息；打印零速度不等于发出停止命令。
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


    // 保存 ROS 接口对象，使它们在节点存活期间持续有效。
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    // 跨回调保存的数据：接收时刻、最新反馈、接收标记和日志时刻。
    std::chrono::steady_clock::time_point last_odom_time_;
    nav_msgs::msg::Odometry last_odom_;
    bool received_odom_;
    std::chrono::steady_clock::time_point last_log_time_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DistanceController>();

    // 构造完成后开始执行就绪的消息/定时回调，不按函数在文件中的顺序调用。
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
