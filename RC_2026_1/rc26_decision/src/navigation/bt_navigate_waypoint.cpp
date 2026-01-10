#include "rc26_decision/navigation/bt_navigate_waypoint.hpp"

#include <memory>

#include "rc26_decision/navigation/waypoint_navigator.hpp"

namespace rc26_decision
{

NavigateWaypointAction::NavigateWaypointAction(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config)
{
}

BT::PortsList NavigateWaypointAction::providedPorts()
{
    return {
        BT::InputPort<int>("waypoint_id", "Hardcoded waypoint id (1-10)"),
    };
}

BT::NodeStatus NavigateWaypointAction::onStart()
{
    int waypoint_id_int = 0;
    if (!getInput("waypoint_id", waypoint_id_int))
    {
        throw BT::RuntimeError("NavigateWaypoint missing required input [waypoint_id]");
    }

    if (waypoint_id_int < 1 || waypoint_id_int > 10)
    {
        return BT::NodeStatus::FAILURE;
    }

    std::shared_ptr<WaypointNavigator> navigator;
    if (!config().blackboard->get("waypoint_navigator", navigator) || !navigator)
    {
        throw BT::RuntimeError("NavigateWaypoint: blackboard missing [waypoint_navigator]");
    }

    return navigator->start(static_cast<uint8_t>(waypoint_id_int))
        ? BT::NodeStatus::RUNNING
        : BT::NodeStatus::FAILURE;
}

BT::NodeStatus NavigateWaypointAction::onRunning()
{
    std::shared_ptr<WaypointNavigator> navigator;
    if (!config().blackboard->get("waypoint_navigator", navigator) || !navigator)
    {
        return BT::NodeStatus::FAILURE;
    }

    const auto st = navigator->tick();
    switch (st)
    {
        case WaypointNavigator::Status::Running:
            return BT::NodeStatus::RUNNING;
        case WaypointNavigator::Status::Succeeded:
            return BT::NodeStatus::SUCCESS;
        default:
            return BT::NodeStatus::FAILURE;
    }
}

void NavigateWaypointAction::onHalted()
{
    std::shared_ptr<WaypointNavigator> navigator;
    if (config().blackboard->get("waypoint_navigator", navigator) && navigator)
    {
        navigator->cancelAndStop();
    }
}

void registerNavigationNodes(BT::BehaviorTreeFactory& factory)
{
    factory.registerNodeType<NavigateWaypointAction>("NavigateWaypoint");
}

}  // namespace rc26_decision
