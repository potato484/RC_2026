#pragma once

#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

#include "rc26_decision/navigation/smart_waypoint_types.hpp"

namespace rc26_decision {

class WaypointManager {
public:
    WaypointManager() : logger_(rclcpp::get_logger("WaypointManager")) {}
    explicit WaypointManager(rclcpp::Logger logger) : logger_(logger) {}

    bool loadFromYamlFile(const std::string& yaml_path);

    const SmartWaypointSpec* find(const std::string& name) const;
    std::vector<std::string> listNames() const;

    bool hasMerlinGraph() const { return merlin_graph_.has_value(); }
    const MerlinGraph& merlinGraph() const;

    void injectPointsToBlackboard(const BT::Blackboard::Ptr& blackboard,
                                  const std::string& key_prefix = "nav.points.") const;

private:
    rclcpp::Logger logger_;
    std::unordered_map<std::string, SmartWaypointSpec> points_;
    std::optional<MerlinGraph> merlin_graph_;
};

}  // namespace rc26_decision
