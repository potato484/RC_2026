#pragma once

#include <behaviortree_cpp/bt_factory.h>

namespace rc26_decision {

class NavToSmartPointAction : public BT::StatefulActionNode {
public:
    NavToSmartPointAction(const std::string& name, const BT::NodeConfig& config);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
};

void registerNavigationNodes(BT::BehaviorTreeFactory& factory);

}  // namespace rc26_decision
