// 梅林区 (MF Area) 行为树节点实现
#include "rc26_decision/mf/mf_area.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>

#include "rc26_decision/navigation/smart_waypoint_navigator.hpp"
#include "rc26_decision/navigation/waypoint_manager.hpp"
#include "rc26_serial/protocol.hpp"
#include "rc26_vision/types.hpp"

namespace rc26_decision {

// ============================================================================
// MerlinMapManager 实现
// ============================================================================
MerlinMapManager::MerlinMapManager() {
    for (auto& cell : cells_) {
        cell.depth = 0;
        cell.kfs = KFSType::UNKNOWN;
    }
}

void MerlinMapManager::initRedMap() {
    // 红方高度矩阵 (已确认):
    // 格子编号:        高度(mm):        深度编码:
    // 10  11  12       200  400  200    1  2  1
    //  7   8   9       400  600  400    2  3  2
    //  4   5   6       600  400  200    3  2  1
    //  1   2   3       400  200  400    2  1  2
    cells_[1] = {2, KFSType::UNKNOWN};
    cells_[2] = {1, KFSType::UNKNOWN};
    cells_[3] = {2, KFSType::UNKNOWN};
    cells_[4] = {3, KFSType::UNKNOWN};
    cells_[5] = {2, KFSType::UNKNOWN};
    cells_[6] = {1, KFSType::UNKNOWN};
    cells_[7] = {2, KFSType::UNKNOWN};
    cells_[8] = {3, KFSType::UNKNOWN};
    cells_[9] = {2, KFSType::UNKNOWN};
    cells_[10] = {1, KFSType::UNKNOWN};
    cells_[11] = {2, KFSType::UNKNOWN};
    cells_[12] = {1, KFSType::UNKNOWN};
}

void MerlinMapManager::initBlueMap() {
    // 蓝方高度矩阵 (与红方对称):
    // 格子编号:        高度(mm):        深度编码:
    // 10  11  12       200  400  200    1  2  1
    //  7   8   9       400  600  400    2  3  2
    //  4   5   6       200  400  600    1  2  3
    //  1   2   3       400  200  400    2  1  2
    cells_[1] = {2, KFSType::UNKNOWN};
    cells_[2] = {1, KFSType::UNKNOWN};
    cells_[3] = {2, KFSType::UNKNOWN};
    cells_[4] = {1, KFSType::UNKNOWN};
    cells_[5] = {2, KFSType::UNKNOWN};
    cells_[6] = {3, KFSType::UNKNOWN};
    cells_[7] = {2, KFSType::UNKNOWN};
    cells_[8] = {3, KFSType::UNKNOWN};
    cells_[9] = {2, KFSType::UNKNOWN};
    cells_[10] = {1, KFSType::UNKNOWN};
    cells_[11] = {2, KFSType::UNKNOWN};
    cells_[12] = {1, KFSType::UNKNOWN};
}

int MerlinMapManager::getDepth(int grid_id) const {
    if (grid_id < 1 || grid_id > 12) return -1;
    return cells_[grid_id].depth;
}

KFSType MerlinMapManager::getKFS(int grid_id) const {
    if (grid_id < 1 || grid_id > 12) return KFSType::UNKNOWN;
    return cells_[grid_id].kfs;
}

void MerlinMapManager::setKFS(int grid_id, KFSType type) {
    if (grid_id >= 1 && grid_id <= 12) {
        cells_[grid_id].kfs = type;
    }
}

bool MerlinMapManager::canTraverse(int from, int to) const {
    int d1 = getDepth(from);
    int d2 = getDepth(to);
    if (d1 < 0 || d2 < 0) return false;
    return std::abs(d1 - d2) <= 1;  // 高度差 ≤ 200mm
}

bool MerlinMapManager::isWalkable(int from, int to) const {
    KFSType kfs = getKFS(to);
    // 只有 NONE 状态才可行走，UNKNOWN 需要先扫描确认
    return (kfs == KFSType::NONE) && canTraverse(from, to);
}

int MerlinMapManager::getAdjacentGrid(int current, MFDirection dir) const {
    if (current < 1 || current > 12) return -1;
    int row = (current - 1) / 3;  // 0-3
    int col = (current - 1) % 3;  // 0-2

    switch (dir) {
    case MFDirection::LEFT:
        return (col > 0) ? current - 1 : -1;
    case MFDirection::RIGHT:
        return (col < 2) ? current + 1 : -1;
    case MFDirection::FRONT:
        return (row < 3) ? current + 3 : -1;
    case MFDirection::BACK:
        return (row > 0) ? current - 3 : -1;
    }
    return -1;
}

// ============================================================================
// StairClimbAction - 上阶梯
// ============================================================================
StairClimbAction::StairClimbAction(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config) {}

BT::PortsList StairClimbAction::providedPorts() {
    return {};
}

BT::NodeStatus StairClimbAction::onStart() {
    level_start_set_ = false;
    int32_t current_level = 0;
    if (config().blackboard->get("current_level", current_level)) {
        level_start_ = current_level;
        level_start_set_ = true;
        config().blackboard->set("level_start", level_start_);
    }
    config().blackboard->set("stair_climb_done", false);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus StairClimbAction::onRunning() {
    int32_t current_level = 0;
    if (!level_start_set_) {
        if (config().blackboard->get("current_level", current_level)) {
            level_start_ = current_level;
            level_start_set_ = true;
            config().blackboard->set("level_start", level_start_);
        } else {
            return BT::NodeStatus::RUNNING;
        }
    } else if (!config().blackboard->get("current_level", current_level)) {
        return BT::NodeStatus::RUNNING;
    }

    if (current_level >= level_start_ + kStairLevelDelta) {
        config().blackboard->set("stair_climb_done", true);
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}

void StairClimbAction::onHalted() {
    level_start_set_ = false;
}

// ============================================================================
// StairDescendAction - 下阶梯
// ============================================================================
StairDescendAction::StairDescendAction(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config) {}

BT::PortsList StairDescendAction::providedPorts() {
    return {};
}

BT::NodeStatus StairDescendAction::onStart() {
    level_start_set_ = false;
    int32_t current_level = 0;
    if (config().blackboard->get("current_level", current_level)) {
        level_start_ = current_level;
        level_start_set_ = true;
        config().blackboard->set("level_start", level_start_);
    }
    config().blackboard->set("stair_descend_done", false);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus StairDescendAction::onRunning() {
    int32_t current_level = 0;
    if (!level_start_set_) {
        if (config().blackboard->get("current_level", current_level)) {
            level_start_ = current_level;
            level_start_set_ = true;
            config().blackboard->set("level_start", level_start_);
        } else {
            return BT::NodeStatus::RUNNING;
        }
    } else if (!config().blackboard->get("current_level", current_level)) {
        return BT::NodeStatus::RUNNING;
    }

    if (current_level <= level_start_ - kStairLevelDelta) {
        config().blackboard->set("stair_descend_done", true);
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}

void StairDescendAction::onHalted() {
    level_start_set_ = false;
}

// ============================================================================
// GrabKFSAction - 夹取 KFS
// ============================================================================
GrabKFSAction::GrabKFSAction(const std::string& name, const BT::NodeConfig& config)
    : BtActionNode<rc26_interfaces::action::ExecuteMechanism>(
          name, config, "/mechanism/execute", std::chrono::seconds(8)) {}

BT::PortsList GrabKFSAction::providedPorts() {
    return BtActionNode<rc26_interfaces::action::ExecuteMechanism>::basePorts(8.0);
}

bool GrabKFSAction::buildGoal(Goal& goal) {
    double timeout_sec = 8.0;
    (void)getInput("timeout_sec", timeout_sec);
    goal.command_id = static_cast<uint8_t>(CommandID::GRAB_KFS);
    goal.payload.clear();
    goal.timeout_sec = static_cast<float>(timeout_sec);
    return true;
}

BT::NodeStatus GrabKFSAction::handleResult(const WrappedResult& result, uint16_t& error_code) {
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED || !result.result || !result.result->success) {
        error_code = (result.result ? result.result->error_code : 0);
        return BT::NodeStatus::FAILURE;
    }
    error_code = 0;
    return BT::NodeStatus::SUCCESS;
}

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
// MechUpMerlinAction - 梅林区机构抬升
// ============================================================================
MechUpMerlinAction::MechUpMerlinAction(const std::string& name, const BT::NodeConfig& config)
    : BtActionNode<rc26_interfaces::action::ExecuteMechanism>(
          name, config, "/mechanism/execute", std::chrono::seconds(8)) {}

BT::PortsList MechUpMerlinAction::providedPorts() {
    return BtActionNode<rc26_interfaces::action::ExecuteMechanism>::basePorts(8.0);
}

bool MechUpMerlinAction::buildGoal(Goal& goal) {
    double timeout_sec = 8.0;
    (void)getInput("timeout_sec", timeout_sec);
    goal.command_id = static_cast<uint8_t>(CommandID::MECH_UP_MERLIN);
    goal.payload.clear();
    goal.timeout_sec = static_cast<float>(timeout_sec);
    return true;
}

BT::NodeStatus MechUpMerlinAction::handleResult(const WrappedResult& result, uint16_t& error_code) {
    return handleExecuteMechanismResult(result, error_code);
}

// ============================================================================
// MechDownMerlinAction - 梅林区机构下降
// ============================================================================
MechDownMerlinAction::MechDownMerlinAction(const std::string& name, const BT::NodeConfig& config)
    : BtActionNode<rc26_interfaces::action::ExecuteMechanism>(
          name, config, "/mechanism/execute", std::chrono::seconds(8)) {}

BT::PortsList MechDownMerlinAction::providedPorts() {
    return BtActionNode<rc26_interfaces::action::ExecuteMechanism>::basePorts(8.0);
}

bool MechDownMerlinAction::buildGoal(Goal& goal) {
    double timeout_sec = 8.0;
    (void)getInput("timeout_sec", timeout_sec);
    goal.command_id = static_cast<uint8_t>(CommandID::MECH_DOWN_MERLIN);
    goal.payload.clear();
    goal.timeout_sec = static_cast<float>(timeout_sec);
    return true;
}

BT::NodeStatus MechDownMerlinAction::handleResult(const WrappedResult& result, uint16_t& error_code) {
    return handleExecuteMechanismResult(result, error_code);
}

// ============================================================================
// RotateAction - 旋转
// ============================================================================
RotateAction::RotateAction(const std::string& name, const BT::NodeConfig& config)
    : BtActionNode<rc26_interfaces::action::ExecuteMechanism>(
          name, config, "/mechanism/execute", std::chrono::seconds(8)) {}

BT::PortsList RotateAction::providedPorts() {
    auto ports = BtActionNode<rc26_interfaces::action::ExecuteMechanism>::basePorts(8.0);
    ports.insert(BT::InputPort<int>("angle", "旋转角度 (90, -90, 180, -180)"));
    return ports;
}

bool RotateAction::buildGoal(Goal& goal) {
    int angle = 0;
    if (!getInput("angle", angle)) {
        return false;
    }
    CommandID cmd = CommandID::ROTATE_POS_90;
    switch (angle) {
    case 90:   cmd = CommandID::ROTATE_POS_90; break;
    case -90:  cmd = CommandID::ROTATE_NEG_90; break;
    case 180:  cmd = CommandID::ROTATE_POS_180; break;
    case -180: cmd = CommandID::ROTATE_NEG_180; break;
    default:   return false;
    }
    double timeout_sec = 8.0;
    (void)getInput("timeout_sec", timeout_sec);
    goal.command_id = static_cast<uint8_t>(cmd);
    goal.payload.clear();
    goal.timeout_sec = static_cast<float>(timeout_sec);
    return true;
}

BT::NodeStatus RotateAction::handleResult(const WrappedResult& result, uint16_t& error_code) {
    return handleExecuteMechanismResult(result, error_code);
}

// ============================================================================
// CheckKFSCondition - 检查 KFS 存在
// ============================================================================
CheckKFSCondition::CheckKFSCondition(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config) {}

BT::PortsList CheckKFSCondition::providedPorts() {
    return {
        BT::InputPort<int>("grid_id", "目标格子 ID (1-12)"),
        BT::InputPort<std::string>("expected_state", "AUTO_KFS", "期望状态"),
    };
}

BT::NodeStatus CheckKFSCondition::tick() {
    int grid_id = 0;
    if (!getInput("grid_id", grid_id)) {
        return BT::NodeStatus::FAILURE;
    }
    std::shared_ptr<MerlinMapManager> map;
    if (!config().blackboard->get("merlin_map", map) || !map) {
        return BT::NodeStatus::FAILURE;
    }
    std::string expected = "AUTO_KFS";
    getInput("expected_state", expected);
    KFSType kfs = map->getKFS(grid_id);
    if (expected == "AUTO_KFS" && kfs == KFSType::R2) {
        return BT::NodeStatus::SUCCESS;
    }
    if (expected == "EMPTY" && kfs == KFSType::NONE) {
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}

// ============================================================================
// CheckLoadCondition - 检查装载数量
// ============================================================================
CheckLoadCondition::CheckLoadCondition(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config) {}

BT::PortsList CheckLoadCondition::providedPorts() {
    return {
        BT::InputPort<int>("min_load", 0, "最小装载数"),
        BT::InputPort<int>("max_load", 3, "最大装载数"),
    };
}

BT::NodeStatus CheckLoadCondition::tick() {
    int min_load = 0, max_load = 3;
    getInput("min_load", min_load);
    getInput("max_load", max_load);
    int kfs_count = 0;
    config().blackboard->get("kfs_on_board", kfs_count);
    return (kfs_count >= min_load && kfs_count <= max_load)
           ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ============================================================================
// SetNavModeAction - 导航模式切换
// ============================================================================
SetNavModeAction::SetNavModeAction(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config) {}

BT::PortsList SetNavModeAction::providedPorts() {
    return {
        BT::InputPort<std::string>("mode", "MF_SAFE", "导航模式"),
    };
}

BT::NodeStatus SetNavModeAction::onStart() {
    rclcpp::Node* node = nullptr;
    if (!config().blackboard->get("node", node) || !node) {
        return BT::NodeStatus::FAILURE;
    }
    std::string mode = "MF_SAFE";
    getInput("mode", mode);
    std::string profile = mode;
    std::transform(profile.begin(), profile.end(), profile.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (profile == "mf_safe") {
        profile = "safe";
    } else if (profile == "normal") {
        profile = "normal";
    }

    client_ = node->create_client<rc26_interfaces::srv::SetNavMode>("set_nav_mode");
    if (!client_->wait_for_service(std::chrono::milliseconds(100))) {
        return BT::NodeStatus::FAILURE;
    }
    auto request = std::make_shared<rc26_interfaces::srv::SetNavMode::Request>();
    request->profile = profile;
    request->timeout = 0.0F;
    request->reason = "mf_area_bt";
    auto future_and_id = client_->async_send_request(request);
    future_ = future_and_id.future.share();
    waiting_ = true;
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SetNavModeAction::onRunning() {
    if (!waiting_) return BT::NodeStatus::FAILURE;
    auto status = future_.wait_for(std::chrono::milliseconds(10));
    if (status == std::future_status::ready) {
        waiting_ = false;
        auto result = future_.get();
        return result->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
}

void SetNavModeAction::onHalted() {
    waiting_ = false;
}

// ============================================================================
// ScanSurroundingsAction - 扫描周围环境
// ============================================================================
ScanSurroundingsAction::ScanSurroundingsAction(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config) {}

BT::PortsList ScanSurroundingsAction::providedPorts() {
    return {};
}

BT::NodeStatus ScanSurroundingsAction::onStart() {
    (void)config().blackboard->get("merlin_map", map_);
    if (!map_) {
        map_ = std::make_shared<MerlinMapManager>();
        std::string team = "red";
        (void)config().blackboard->get("team", team);
        if (team == "blue") {
            map_->initBlueMap();
        } else {
            map_->initRedMap();
        }
        config().blackboard->set("merlin_map", map_);
    }

    int current_grid = 2;
    (void)config().blackboard->get("current_grid", current_grid);

    const int front = map_->getAdjacentGrid(current_grid, MFDirection::FRONT);
    const int left = map_->getAdjacentGrid(current_grid, MFDirection::LEFT);
    const int right = map_->getAdjacentGrid(current_grid, MFDirection::RIGHT);

    for (const int grid_id : {front, left, right}) {
        if (grid_id < 1 || grid_id > 12) {
            continue;
        }
        if (map_->getKFS(grid_id) == KFSType::UNKNOWN) {
            map_->setKFS(grid_id, KFSType::NONE);
        }
    }

    bool vision_has_target = false;
    int vision_attr_kind = static_cast<int>(rc26_vision::AttributeKind::Unknown);
    (void)config().blackboard->get("vision_has_target", vision_has_target);
    (void)config().blackboard->get("vision_attr_kind", vision_attr_kind);
    if (vision_has_target && front >= 1 && front <= 12) {
        switch (static_cast<rc26_vision::AttributeKind>(vision_attr_kind)) {
        case rc26_vision::AttributeKind::R_R1:
        case rc26_vision::AttributeKind::B_R1:
            map_->setKFS(front, KFSType::R1);
            break;
        case rc26_vision::AttributeKind::Truth:
            map_->setKFS(front, KFSType::R2);
            break;
        case rc26_vision::AttributeKind::False:
            map_->setKFS(front, KFSType::FAKE);
            break;
        default:
            break;
        }
    }

    phase_ = ScanPhase::DONE;
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ScanSurroundingsAction::onRunning() {
    return BT::NodeStatus::SUCCESS;
}

void ScanSurroundingsAction::onHalted() {
    phase_ = ScanPhase::ROTATE_LEFT;
}

// ============================================================================
// SelectNextGridAction - 选择下一个目标格子
// ============================================================================
SelectNextGridAction::SelectNextGridAction(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config) {}

BT::PortsList SelectNextGridAction::providedPorts() {
    return {
        BT::OutputPort<std::string>("next_action"),
        BT::OutputPort<int>("target_grid"),
    };
}

BT::NodeStatus SelectNextGridAction::tick() {
    int current = 2, kfs_count = 0, target_kfs = 2, exit_grid = 10;
    (void)config().blackboard->get("current_grid", current);
    (void)config().blackboard->get("kfs_on_board", kfs_count);
    (void)config().blackboard->get("target_kfs_count", target_kfs);
    (void)config().blackboard->get("exit_grid", exit_grid);

    std::shared_ptr<MerlinMapManager> map;
    (void)config().blackboard->get("merlin_map", map);
    if (!map) return BT::NodeStatus::FAILURE;

    // P1: 贪婪抓取
    if (kfs_count < target_kfs) {
        int grab = findAdjacentR2KFS(current, map);
        if (grab > 0) {
            setOutput("next_action", std::string("GRAB"));
            setOutput("target_grid", grab);
            return BT::NodeStatus::SUCCESS;
        }
    }
    // P2: 赶路
    int next = findBestPathToExit(current, exit_grid, map);
    if (next > 0) {
        setOutput("next_action", std::string("MOVE"));
        setOutput("target_grid", next);
        return BT::NodeStatus::SUCCESS;
    }
    // P3: 等待
    setOutput("next_action", std::string("WAIT"));
    setOutput("target_grid", current);
    return BT::NodeStatus::SUCCESS;
}

int SelectNextGridAction::findAdjacentR2KFS(int current, std::shared_ptr<MerlinMapManager> map) {
    for (auto dir : {MFDirection::FRONT, MFDirection::LEFT, MFDirection::RIGHT}) {
        int adj = map->getAdjacentGrid(current, dir);
        if (adj > 0 && map->getKFS(adj) == KFSType::R2 && map->canTraverse(current, adj)) {
            return adj;
        }
    }
    return -1;
}

int SelectNextGridAction::findBestPathToExit(int current, int exit_grid, std::shared_ptr<MerlinMapManager> map) {
    int best = -1, min_dist = 100;
    for (auto dir : {MFDirection::FRONT, MFDirection::LEFT, MFDirection::RIGHT}) {
        int adj = map->getAdjacentGrid(current, dir);
        if (adj > 0 && map->isWalkable(current, adj)) {
            int dist = std::abs((adj - 1) / 3 - (exit_grid - 1) / 3) +
                       std::abs((adj - 1) % 3 - (exit_grid - 1) % 3);
            if (dist < min_dist) {
                min_dist = dist;
                best = adj;
            }
        }
    }
    return best;
}

// ============================================================================
// CheckExitCondition - 检查退出条件
// ============================================================================
CheckExitCondition::CheckExitCondition(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config) {}

BT::PortsList CheckExitCondition::providedPorts() {
    return {};
}

BT::NodeStatus CheckExitCondition::tick() {
    int current = 0, kfs_count = 0, target_kfs = 2;
    config().blackboard->get("current_grid", current);
    config().blackboard->get("kfs_on_board", kfs_count);
    config().blackboard->get("target_kfs_count", target_kfs);
    bool at_exit = (current == 10 || current == 11 || current == 12);
    return (at_exit && kfs_count >= target_kfs) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ============================================================================
// CheckR1BlockingCondition - 检查R1阻挡
// ============================================================================
CheckR1BlockingCondition::CheckR1BlockingCondition(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config) {}

BT::PortsList CheckR1BlockingCondition::providedPorts() {
    return {};
}

BT::NodeStatus CheckR1BlockingCondition::tick() {
    std::shared_ptr<MerlinMapManager> map;
    config().blackboard->get("merlin_map", map);
    if (!map) return BT::NodeStatus::FAILURE;
    int current = 2;
    config().blackboard->get("current_grid", current);
    int front = map->getAdjacentGrid(current, MFDirection::FRONT);
    if (front > 0 && map->getKFS(front) == KFSType::R1) {
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}

// ============================================================================
// IncrementKFSCountAction - 更新KFS计数
// ============================================================================
IncrementKFSCountAction::IncrementKFSCountAction(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config) {}

BT::PortsList IncrementKFSCountAction::providedPorts() {
    return {};
}

BT::NodeStatus IncrementKFSCountAction::tick() {
    int count = 0;
    config().blackboard->get("kfs_on_board", count);
    config().blackboard->set("kfs_on_board", count + 1);
    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// UpdateMapKFSAction - 更新地图KFS状态
// ============================================================================
UpdateMapKFSAction::UpdateMapKFSAction(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config) {}

BT::PortsList UpdateMapKFSAction::providedPorts() {
    return {
        BT::InputPort<int>("grid_id"),
        BT::InputPort<int>("kfs_type", static_cast<int>(0), "目标 KFS 类型"),
    };
}

BT::NodeStatus UpdateMapKFSAction::tick() {
    int grid_id = 0, kfs_type = 0;
    if (!getInput("grid_id", grid_id)) return BT::NodeStatus::FAILURE;
    getInput("kfs_type", kfs_type);
    std::shared_ptr<MerlinMapManager> map;
    if (!config().blackboard->get("merlin_map", map) || !map) {
        return BT::NodeStatus::FAILURE;
    }
    map->setKFS(grid_id, static_cast<KFSType>(kfs_type));
    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// NavToMerlinGridAction - 导航到梅林格子
// ============================================================================
NavToMerlinGridAction::NavToMerlinGridAction(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config) {}

BT::PortsList NavToMerlinGridAction::providedPorts() {
    return {
        BT::InputPort<int>("grid_id", "目标格子编号 (1-12)"),
    };
}

BT::NodeStatus NavToMerlinGridAction::onStart() {
    int grid_id = 0;
    if (!getInput("grid_id", grid_id) || grid_id < 1 || grid_id > 12) {
        return BT::NodeStatus::FAILURE;
    }

    auto log_guard_fail = [&](const std::string& reason) {
        rclcpp::Node* node = nullptr;
        if (config().blackboard->get("node", node) && node) {
            RCLCPP_WARN(node->get_logger(), "NavToMerlinGrid guard reject: %s", reason.c_str());
        }
        return BT::NodeStatus::FAILURE;
    };

    int current_grid = 0;
    if (!config().blackboard->get("current_grid", current_grid) ||
        current_grid < 1 || current_grid > 12) {
        return log_guard_fail("invalid current_grid");
    }

    std::shared_ptr<MerlinMapManager> map;
    if (!config().blackboard->get("merlin_map", map) || !map) {
        return log_guard_fail("missing merlin_map");
    }

    if (grid_id != current_grid) {
        const int cur_row = (current_grid - 1) / 3;
        const int cur_col = (current_grid - 1) % 3;
        const int dst_row = (grid_id - 1) / 3;
        const int dst_col = (grid_id - 1) % 3;
        const int manhattan = std::abs(dst_row - cur_row) + std::abs(dst_col - cur_col);
        if (manhattan != 1) {
            return log_guard_fail("non-adjacent target (diagonal/cross-grid move blocked)");
        }
        if (!map->canTraverse(current_grid, grid_id)) {
            return log_guard_fail("height delta too large for traverse");
        }
    }

    std::string target_name = "mf_grid_" + std::to_string(grid_id);

    std::shared_ptr<WaypointManager> waypoint_manager;
    if (!config().blackboard->get("waypoint_manager", waypoint_manager) || !waypoint_manager) {
        return BT::NodeStatus::FAILURE;
    }
    std::shared_ptr<SmartWaypointNavigator> navigator;
    if (!config().blackboard->get("smart_waypoint_navigator", navigator) || !navigator) {
        return BT::NodeStatus::FAILURE;
    }
    const SmartWaypointSpec* wp = waypoint_manager->find(target_name);
    if (!wp) {
        return BT::NodeStatus::FAILURE;
    }
    return navigator->start(*wp) ? BT::NodeStatus::RUNNING : BT::NodeStatus::FAILURE;
}

BT::NodeStatus NavToMerlinGridAction::onRunning() {
    std::shared_ptr<SmartWaypointNavigator> navigator;
    if (!config().blackboard->get("smart_waypoint_navigator", navigator) || !navigator) {
        return BT::NodeStatus::FAILURE;
    }
    auto st = navigator->tick();
    if (st == SmartWaypointNavigator::Status::Running) return BT::NodeStatus::RUNNING;
    if (st == SmartWaypointNavigator::Status::Succeeded) return BT::NodeStatus::SUCCESS;
    return BT::NodeStatus::FAILURE;
}

void NavToMerlinGridAction::onHalted() {
    std::shared_ptr<SmartWaypointNavigator> navigator;
    if (config().blackboard->get("smart_waypoint_navigator", navigator) && navigator) {
        navigator->cancelAndStop();
    }
}

// ============================================================================
// 注册函数
// ============================================================================
void registerMFAreaNodes(BT::BehaviorTreeFactory& factory) {
    factory.registerNodeType<StairClimbAction>("StairClimb");
    factory.registerNodeType<StairDescendAction>("StairDescend");
    factory.registerNodeType<GrabKFSAction>("GrabKFS");
    factory.registerNodeType<MechUpMerlinAction>("MechUpMerlin");
    factory.registerNodeType<MechDownMerlinAction>("MechDownMerlin");
    factory.registerNodeType<RotateAction>("Rotate");
    factory.registerNodeType<CheckKFSCondition>("CheckKFS");
    factory.registerNodeType<CheckLoadCondition>("CheckLoad");
    factory.registerNodeType<SetNavModeAction>("SetNavMode");
    factory.registerNodeType<ScanSurroundingsAction>("ScanSurroundings");
    factory.registerNodeType<SelectNextGridAction>("SelectNextGrid");
    factory.registerNodeType<CheckExitCondition>("CheckExitCondition");
    factory.registerNodeType<CheckR1BlockingCondition>("CheckR1Blocking");
    factory.registerNodeType<IncrementKFSCountAction>("IncrementKFSCount");
    factory.registerNodeType<UpdateMapKFSAction>("UpdateMapKFS");
    factory.registerNodeType<NavToMerlinGridAction>("NavToMerlinGrid");
}

}  // namespace rc26_decision
