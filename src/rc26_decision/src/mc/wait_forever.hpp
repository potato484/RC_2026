// 武馆区无限期等待动作：恒返回 RUNNING，使行为树停留并持续 tick。
#pragma once

#include <string>

#include <behaviortree_cpp/bt_factory.h>

namespace rc26_decision {

class WaitForeverAction : public BT::StatefulActionNode {
public:
    WaitForeverAction(const std::string& name, const BT::NodeConfig& config);

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
};

}  // namespace rc26_decision
