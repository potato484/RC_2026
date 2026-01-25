#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <optional>
#include <string>

#include "rc26_vision/types.hpp"

namespace rc26_vision {
class VisionInferenceManager;
}

namespace rc26_decision {

class VisionStartAction : public BT::StatefulActionNode {
public:
    VisionStartAction(const std::string& name, const BT::NodeConfig& config);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
};

class VisionStopAction : public BT::StatefulActionNode {
public:
    VisionStopAction(const std::string& name, const BT::NodeConfig& config);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
};

class WaitVisionTargetAction : public BT::StatefulActionNode {
public:
    WaitVisionTargetAction(const std::string& name, const BT::NodeConfig& config);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    std::shared_ptr<rc26_vision::VisionInferenceManager> manager_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Time start_time_;
    int timeout_ms_ = 1000;
    std::optional<rc26_vision::AttributeKind> target_attr_;
    std::optional<double> max_dist_m_;
};

void registerVisionNodes(BT::BehaviorTreeFactory& factory);

}  // namespace rc26_decision

