// 武馆区原地旋转动作：发布 cmd_vel.angular.z，订阅里程计 yaw 增量积分实现闭环转角。
#pragma once

#include <chrono>
#include <string>

#include <behaviortree_cpp/bt_factory.h>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mc_params.hpp"

namespace rc26_decision {

class RotateInPlaceAction : public BT::StatefulActionNode {
public:
    RotateInPlaceAction(const std::string& name, const BT::NodeConfig& config);

    static BT::PortsList providedPorts() {
        return {BT::InputPort<double>("target_yaw_rad", "Optional absolute target yaw in radians")};
    }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    using TwistMsg = geometry_msgs::msg::Twist;
    using OdomMsg = nav_msgs::msg::Odometry;

    void publishStop();

    McParams params_;
    rclcpp::Node* node_{nullptr};
    rclcpp::Publisher<TwistMsg>::SharedPtr cmd_pub_;
    rclcpp::Subscription<OdomMsg>::SharedPtr odom_sub_;

    double target_rad_{0.0};
    double signed_target_rad_{0.0};
    double absolute_target_yaw_rad_{0.0};
    double tolerance_rad_{0.0};
    double min_speed_radps_{0.0};
    double slowdown_rad_{0.0};
    double accumulated_rad_{0.0};
    double current_yaw_{0.0};
    double last_yaw_{0.0};
    bool absolute_target_mode_{false};
    bool has_yaw_{false};
    std::chrono::steady_clock::time_point last_odom_tp_{};
    rclcpp::Time start_time_;
};

class RotateRetreatAction : public BT::StatefulActionNode {
public:
    RotateRetreatAction(const std::string& name, const BT::NodeConfig& config);

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<double>("retreat_x_m", -0.4, "Retreat X displacement in final yaw frame"),
            BT::InputPort<double>("retreat_y_m", -0.4, "Retreat Y displacement in final yaw frame"),
            BT::InputPort<std::string>("cmd_vel_topic", "cmd_vel", "Velocity command topic"),
            BT::InputPort<std::string>("odom_topic", "odom", "Odometry topic"),
            BT::InputPort<double>("max_speed_mps", 0.20, "Maximum planar speed in m/s"),
            BT::InputPort<double>("min_speed_mps", 0.03, "Minimum planar speed before tolerance"),
            BT::InputPort<double>("xy_kp", 0.8, "Planar position error to speed gain"),
            BT::InputPort<double>("heading_kp", 1.2, "Yaw error to angular speed gain"),
            BT::InputPort<double>("heading_max_speed_radps", 0.30, "Maximum angular speed"),
            BT::InputPort<double>("xy_tolerance_m", 0.03, "Planar success tolerance"),
            BT::InputPort<double>("yaw_tolerance_deg", 3.0, "Yaw success tolerance"),
            BT::InputPort<int>("stable_ticks", 3, "Consecutive stable ticks required"),
            BT::InputPort<double>("odom_timeout_s", 0.5, "Maximum accepted odom age"),
            BT::InputPort<double>("timeout_s", 60.0, "Action timeout in seconds"),
        };
    }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    using TwistMsg = geometry_msgs::msg::Twist;
    using OdomMsg = nav_msgs::msg::Odometry;

    void publishStop();
    void releaseRuntime();
    bool odomReady() const;
    bool timedOut() const;
    bool prepareTargetFromCurrentOdom();
    BT::NodeStatus tickTowardTarget();
    BT::NodeStatus failWithStop(const std::string& reason);

    McParams params_;
    rclcpp::Node* node_{nullptr};
    rclcpp::Publisher<TwistMsg>::SharedPtr cmd_pub_;
    rclcpp::Subscription<OdomMsg>::SharedPtr odom_sub_;
    rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};
    std::chrono::steady_clock::time_point last_odom_tp_{};
    std::string cmd_vel_topic_{"cmd_vel"};
    std::string odom_topic_{"odom"};
    double retreat_x_m_{-0.4};
    double retreat_y_m_{-0.4};
    double max_speed_mps_{0.20};
    double min_speed_mps_{0.03};
    double xy_kp_{0.8};
    double heading_kp_{1.2};
    double heading_max_speed_radps_{0.30};
    double xy_tolerance_m_{0.03};
    double yaw_tolerance_rad_{0.05235987755982989};
    int stable_ticks_required_{3};
    int stable_ticks_{0};
    double odom_timeout_s_{0.5};
    double timeout_s_{60.0};
    double current_x_{0.0};
    double current_y_{0.0};
    double current_yaw_{0.0};
    double start_x_{0.0};
    double start_y_{0.0};
    double start_yaw_{0.0};
    double target_x_{0.0};
    double target_y_{0.0};
    double target_yaw_{0.0};
    bool has_odom_{false};
    bool target_ready_{false};
};

}  // namespace rc26_decision
