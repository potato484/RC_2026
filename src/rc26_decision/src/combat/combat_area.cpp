// 对抗区 (Combat Area) 行为树节点实现
#include "rc26_decision/combat/combat_area.hpp"

#include <chrono>
#include <memory>

#include "rc26_serial/protocol.hpp"

namespace rc26_decision {

namespace {

BT::NodeStatus handleExecuteMechanismResult(
    const BtActionNode<rc26_interfaces::action::ExecuteMechanism>::WrappedResult& result,
    uint16_t& error_code) {
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED || !result.result || !result.result->success) {
        error_code = (result.result ? result.result->error_code : 0);
        return BT::NodeStatus::FAILURE;
    }
    error_code = 0;
    return BT::NodeStatus::SUCCESS;
}

}  // namespace

// ============================================================================
// PlaceKFSGridAction - 放置 KFS 到九宫格
// ============================================================================
PlaceKFSGridAction::PlaceKFSGridAction(const std::string& name, const BT::NodeConfig& config)
    : BtActionNode<rc26_interfaces::action::ExecuteMechanism>(
          name, config, "/mechanism/run_command", std::chrono::seconds(8)) {}

BT::PortsList PlaceKFSGridAction::providedPorts() {
    auto ports = BtActionNode<rc26_interfaces::action::ExecuteMechanism>::basePorts(8.0);
    ports.insert(BT::InputPort<int>("grid_position", "九宫格位置 (1-9)"));
    ports.insert(BT::InputPort<int>("layer", 0, "目标层 (1-3)，0 表示由 BattleGridState 自动决策"));
    ports.insert(BT::OutputPort<int>("selected_layer", "实际下发层号"));
    return ports;
}

bool PlaceKFSGridAction::buildGoal(Goal& goal) {
    int grid_position = 0;
    if (!getInput("grid_position", grid_position) || grid_position < 1 || grid_position > 9) {
        return false;
    }

    double timeout_sec = 8.0;
    (void)getInput("timeout_sec", timeout_sec);

    int selected_layer = 0;
    int requested_layer = 0;
    (void)getInput("layer", requested_layer);

    if (requested_layer > 0) {
        if (!BattleGridState::isValidLayer(requested_layer)) {
            return false;
        }
        selected_layer = requested_layer;
    } else {
        bool is_lifted = false;
        if (config().blackboard) {
            (void)config().blackboard->get("is_lifted", is_lifted);
        }

        std::shared_ptr<BattleGridState> battle_state;
        const bool has_state =
            config().blackboard &&
            config().blackboard->get("battle_grid_state", battle_state) &&
            battle_state;
        if (has_state) {
            const auto layer = battle_state->selectPlacementLayer(grid_position, is_lifted);
            if (!layer.has_value()) {
                return false;
            }
            selected_layer = *layer;
        } else {
            selected_layer = 1;
        }
    }

    goal.command_id = static_cast<uint8_t>(CommandID::PLACE_KFS_GRID);
    goal.payload = {static_cast<uint8_t>(grid_position), static_cast<uint8_t>(selected_layer)};
    goal.timeout_sec = static_cast<float>(timeout_sec);
    pending_grid_position_ = static_cast<uint8_t>(grid_position);
    pending_layer_ = static_cast<uint8_t>(selected_layer);
    setOutput("selected_layer", selected_layer);
    return true;
}

BT::NodeStatus PlaceKFSGridAction::handleResult(const WrappedResult& result, uint16_t& error_code) {
    const auto status = handleExecuteMechanismResult(result, error_code);
    if (status != BT::NodeStatus::SUCCESS) {
        pending_grid_position_ = 0;
        pending_layer_ = 0;
        return status;
    }

    if (config().blackboard && pending_grid_position_ > 0U && pending_layer_ > 0U) {
        std::shared_ptr<BattleGridState> battle_state;
        if (config().blackboard->get("battle_grid_state", battle_state) && battle_state) {
            battle_state->markOccupied(static_cast<int>(pending_grid_position_),
                                       static_cast<int>(pending_layer_));
        }
    }
    pending_grid_position_ = 0;
    pending_layer_ = 0;
    return status;
}

// ============================================================================
// GimbalMoveAction - 云台控制
// ============================================================================
GimbalMoveAction::GimbalMoveAction(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config) {}

BT::PortsList GimbalMoveAction::providedPorts() {
    return {
        BT::InputPort<float>("pitch", "俯仰角"),
        BT::InputPort<float>("yaw", "偏航角"),
    };
}

BT::NodeStatus GimbalMoveAction::onStart() {
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GimbalMoveAction::onRunning() {
    return BT::NodeStatus::RUNNING;
}

void GimbalMoveAction::onHalted() {}

// ============================================================================
// FollowManualRobotAction - 跟随手动机器人
// ============================================================================
FollowManualRobotAction::FollowManualRobotAction(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config) {}

BT::PortsList FollowManualRobotAction::providedPorts() {
    return {
        BT::InputPort<double>("follow_distance", 1.5, "跟随距离(米)"),
        BT::InputPort<double>("lost_timeout", 5.0, "丢失超时(秒)"),
    };
}

BT::NodeStatus FollowManualRobotAction::onStart() {
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus FollowManualRobotAction::onRunning() {
    return BT::NodeStatus::RUNNING;
}

void FollowManualRobotAction::onHalted() {}

// ============================================================================
// 注册函数
// ============================================================================
void registerCombatAreaNodes(BT::BehaviorTreeFactory& factory) {
    factory.registerNodeType<PlaceKFSGridAction>("PlaceKFSGrid");
    factory.registerNodeType<GimbalMoveAction>("GimbalMove");
    factory.registerNodeType<FollowManualRobotAction>("FollowManualRobot");
}

}  // namespace rc26_decision
