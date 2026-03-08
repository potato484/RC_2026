#pragma once

#include <behaviortree_cpp/bt_factory.h>

namespace rc26_decision {

class CheckLocalizationHealth : public BT::ConditionNode {
public:
    CheckLocalizationHealth(const std::string& name, const BT::NodeConfig& config);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;
};

}  // namespace rc26_decision

