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

}  // namespace rc26_decision
