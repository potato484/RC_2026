#pragma once

#include <memory>
#include <string>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>

namespace rc26_decision {

class SmartWaypointNavigator;
class WaypointManager;

class LocalizationAnchorRecover : public BT::StatefulActionNode {
public:
    LocalizationAnchorRecover(const std::string& name, const BT::NodeConfig& config);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    bool loadRecoveryTargets();
    bool startCurrentTarget();
    bool checkRecovered() const;

    std::shared_ptr<SmartWaypointNavigator> navigator_;
    std::shared_ptr<WaypointManager> waypoint_manager_;
    std::vector<std::string> recovery_targets_;
    size_t current_target_idx_{0U};
    bool active_goal_{false};
};

}  // namespace rc26_decision

