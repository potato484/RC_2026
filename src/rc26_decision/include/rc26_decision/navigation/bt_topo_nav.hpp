#pragma once

#include "rc26_decision/common/bt_action_node.hpp"
#include "rc26_interfaces/action/navigate_topo_target.hpp"

namespace rc26_decision {

using NavigateTopoTarget = rc26_interfaces::action::NavigateTopoTarget;

class NavToTopoNodeAction : public BtActionNode<NavigateTopoTarget> {
public:
    NavToTopoNodeAction(const std::string& name, const BT::NodeConfig& config);
    static BT::PortsList providedPorts();
protected:
    bool buildGoal(Goal& goal) override;
    BT::NodeStatus handleResult(const WrappedResult& result, uint16_t& error_code) override;
};

class NavToTaskPoseAction : public BtActionNode<NavigateTopoTarget> {
public:
    NavToTaskPoseAction(const std::string& name, const BT::NodeConfig& config);
    static BT::PortsList providedPorts();
protected:
    bool buildGoal(Goal& goal) override;
    BT::NodeStatus handleResult(const WrappedResult& result, uint16_t& error_code) override;
private:
    static std::string merlinGridNodeId(int grid_id);
};

class ExecuteTopoRouteAction : public BtActionNode<NavigateTopoTarget> {
public:
    ExecuteTopoRouteAction(const std::string& name, const BT::NodeConfig& config);
    static BT::PortsList providedPorts();
protected:
    bool buildGoal(Goal& goal) override;
    BT::NodeStatus handleResult(const WrappedResult& result, uint16_t& error_code) override;
};

void registerTopoNavNodes(BT::BehaviorTreeFactory& factory);

}  // namespace rc26_decision
