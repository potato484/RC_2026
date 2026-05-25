// 梅林区 (MF Area) 行为树节点实现
#include "rc26_decision/mf/mf_area.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <filesystem>

#include "rc26_serial/protocol.hpp"
#include "rc26_vision/shared/vision_types.hpp"
#include "yaml-cpp/yaml.h"

namespace {

std::string toLowerCopy(std::string value) {
  std::transform(
      value.begin(), value.end(), value.begin(),
      [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return value;
}

int mirrorGridIdAcrossColumns(int grid_id) {
  if (grid_id < 1 || grid_id > 12) {
    return grid_id;
  }
  const int row = (grid_id - 1) / 3;
  const int col = (grid_id - 1) % 3;
  return row * 3 + (2 - col) + 1;
}

bool resolveWorldLayoutRedirect(const std::string &raw_path,
                                std::string &resolved_path, std::string &err) {
  if (raw_path.empty()) {
    err = "world layout path is empty";
    return false;
  }

  std::filesystem::path current(raw_path);
  if (current.is_relative()) {
    current = std::filesystem::current_path() / current;
  }
  current = current.lexically_normal();

  for (int hop = 0; hop < 4; ++hop) {
    YAML::Node root = YAML::LoadFile(current.string());
    if (root["world_layout_file"]) {
      std::filesystem::path next = root["world_layout_file"].as<std::string>();
      if (next.empty()) {
        err = "world_layout_file is empty";
        return false;
      }
      if (next.is_relative()) {
        next = current.parent_path() / next;
      }
      current = next.lexically_normal();
      continue;
    }
    resolved_path = current.string();
    return true;
  }

  err = "too many world_layout_file redirects";
  return false;
}

bool loadMerlinCellsFromWorldLayout(
    const std::string &requested_team,
    std::array<rc26_decision::MerlinGridCell, 13> &out_cells,
    std::vector<int> &out_exit_blocks, std::string &out_status) {
  std::string layout_path;
  try {
    layout_path =
        ament_index_cpp::get_package_share_directory("rc26_kfs_keepout") +
        "/config/r2_mf_world.yaml";
  } catch (const std::exception &ex) {
    out_status = std::string("world layout path resolve failed: ") + ex.what();
    return false;
  }

  try {
    std::string resolved_path;
    std::string err;
    if (!resolveWorldLayoutRedirect(layout_path, resolved_path, err)) {
      out_status = err;
      return false;
    }

    const YAML::Node root = YAML::LoadFile(resolved_path);
    const YAML::Node meta = root["meta"];
    if (!meta) {
      out_status = "world layout missing meta";
      return false;
    }

    const std::string layout_team =
        meta["team"] ? toLowerCopy(meta["team"].as<std::string>()) : "";
    const std::string normalized_team = toLowerCopy(requested_team);
    const bool mirror_columns = !layout_team.empty() &&
                                !normalized_team.empty() &&
                                normalized_team != layout_team;

    const YAML::Node blocks = root["blocks"] ? root["blocks"] : root["grids"];
    if (!blocks || !blocks.IsSequence()) {
      out_status = "world layout missing blocks/grids sequence";
      return false;
    }

    for (auto &cell : out_cells) {
      cell.depth = 0;
      cell.kfs = rc26_decision::KFSType::UNKNOWN;
    }

    std::array<bool, 13> seen{};
    for (const auto &block : blocks) {
      const int id = block["id"].as<int>();
      if (id < 1 || id > 12) {
        out_status = "world layout grid id out of range";
        return false;
      }

      int depth = 0;
      if (block["depth"]) {
        depth = block["depth"].as<int>();
      } else if (block["expected_height_m"]) {
        depth = static_cast<int>(
            std::lround(block["expected_height_m"].as<double>() / 0.2));
      }
      if (depth < 0) {
        out_status = "world layout depth must be non-negative";
        return false;
      }

      const int effective_id =
          mirror_columns ? mirrorGridIdAcrossColumns(id) : id;
      out_cells[static_cast<size_t>(effective_id)].depth = depth;
      out_cells[static_cast<size_t>(effective_id)].kfs =
          rc26_decision::KFSType::UNKNOWN;
      seen[static_cast<size_t>(effective_id)] = true;
    }

    for (int id = 1; id <= 12; ++id) {
      if (!seen[static_cast<size_t>(id)]) {
        out_status = "world layout must define ids 1..12";
        return false;
      }
    }

    out_exit_blocks.clear();
    const YAML::Node exit_blocks = root["exit_blocks"];
    if (exit_blocks && exit_blocks.IsSequence()) {
      out_exit_blocks.reserve(exit_blocks.size());
      for (const auto &value : exit_blocks) {
        const int id = value.as<int>();
        if (id < 1 || id > 12) {
          out_status = "exit_blocks contains invalid id";
          return false;
        }
        out_exit_blocks.push_back(mirror_columns ? mirrorGridIdAcrossColumns(id)
                                                 : id);
      }
      std::sort(out_exit_blocks.begin(), out_exit_blocks.end());
      out_exit_blocks.erase(
          std::unique(out_exit_blocks.begin(), out_exit_blocks.end()),
          out_exit_blocks.end());
    } else {
      out_exit_blocks = {10, 11, 12};
    }

    out_status = "world_layout:" + resolved_path;
    if (mirror_columns) {
      out_status += " (applied runtime team mirror)";
    }
    return true;
  } catch (const std::exception &ex) {
    out_status = std::string("world layout parse failed: ") + ex.what();
    return false;
  }
}

} // namespace

namespace rc26_decision {

// ============================================================================
// MerlinMapManager 实现
// ============================================================================
MerlinMapManager::MerlinMapManager() {
  for (auto &cell : cells_) {
    cell.depth = 0;
    cell.kfs = KFSType::UNKNOWN;
  }
  exit_blocks_ = {10, 11, 12};
  layout_status_ = "uninitialized";
}

void MerlinMapManager::initLegacyRedMapLocked() {
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

void MerlinMapManager::initLegacyBlueMapLocked() {
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

bool MerlinMapManager::initFromWorldLayout(const std::string &team) {
  std::lock_guard<std::mutex> lock(mutex_);

  std::array<MerlinGridCell, 13> next_cells{};
  std::vector<int> next_exit_blocks;
  std::string next_status;
  if (loadMerlinCellsFromWorldLayout(team, next_cells, next_exit_blocks,
                                     next_status)) {
    cells_ = std::move(next_cells);
    exit_blocks_ = std::move(next_exit_blocks);
    layout_status_ = std::move(next_status);
    return true;
  }

  if (toLowerCopy(team) == "red") {
    initLegacyRedMapLocked();
  } else {
    initLegacyBlueMapLocked();
  }
  exit_blocks_ = {10, 11, 12};
  layout_status_ = std::move(next_status) + " | fallback=legacy_depth_table";
  return false;
}

bool MerlinMapManager::initRedMap() { return initFromWorldLayout("red"); }

bool MerlinMapManager::initBlueMap() { return initFromWorldLayout("blue"); }

int MerlinMapManager::getDepth(int grid_id) const {
  if (grid_id < 1 || grid_id > 12)
    return -1;
  std::lock_guard<std::mutex> lock(mutex_);
  return cells_[static_cast<size_t>(grid_id)].depth;
}

KFSType MerlinMapManager::getKFS(int grid_id) const {
  if (grid_id < 1 || grid_id > 12)
    return KFSType::UNKNOWN;
  std::lock_guard<std::mutex> lock(mutex_);
  return cells_[static_cast<size_t>(grid_id)].kfs;
}

void MerlinMapManager::setKFS(int grid_id, KFSType type) {
  if (grid_id >= 1 && grid_id <= 12) {
    std::lock_guard<std::mutex> lock(mutex_);
    cells_[static_cast<size_t>(grid_id)].kfs = type;
  }
}

bool MerlinMapManager::canTraverse(int from, int to) const {
  int d1 = getDepth(from);
  int d2 = getDepth(to);
  if (d1 < 0 || d2 < 0)
    return false;
  return std::abs(d1 - d2) <= 1; // 高度差 ≤ 200mm
}

bool MerlinMapManager::isWalkable(int from, int to) const {
  KFSType kfs = getKFS(to);
  // 只有 NONE 状态才可行走，UNKNOWN 需要先扫描确认
  return (kfs == KFSType::NONE) && canTraverse(from, to);
}

int MerlinMapManager::getAdjacentGrid(int current, MFDirection dir) const {
  if (current < 1 || current > 12)
    return -1;
  int row = (current - 1) / 3; // 0-3
  int col = (current - 1) % 3; // 0-2

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

bool MerlinMapManager::isExitBlock(int grid_id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return std::find(exit_blocks_.begin(), exit_blocks_.end(), grid_id) !=
         exit_blocks_.end();
}

std::string MerlinMapManager::layoutStatus() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return layout_status_;
}

// ============================================================================
// StairClimbAction - 上阶梯
// ============================================================================
StairClimbAction::StairClimbAction(const std::string &name,
                                   const BT::NodeConfig &config)
    : BT::StatefulActionNode(name, config) {}

BT::PortsList StairClimbAction::providedPorts() { return {}; }

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

void StairClimbAction::onHalted() { level_start_set_ = false; }

// ============================================================================
// StairDescendAction - 下阶梯
// ============================================================================
StairDescendAction::StairDescendAction(const std::string &name,
                                       const BT::NodeConfig &config)
    : BT::StatefulActionNode(name, config) {}

BT::PortsList StairDescendAction::providedPorts() { return {}; }

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

void StairDescendAction::onHalted() { level_start_set_ = false; }

// ============================================================================
// GrabKFSAction - 夹取 KFS
// ============================================================================
GrabKFSAction::GrabKFSAction(const std::string &name,
                             const BT::NodeConfig &config)
    : BtActionNode<rc26_interfaces::action::ExecuteMechanism>(
          name, config, "/mechanism/execute", std::chrono::seconds(8)) {}

BT::PortsList GrabKFSAction::providedPorts() {
  return BtActionNode<rc26_interfaces::action::ExecuteMechanism>::basePorts(
      8.0);
}

bool GrabKFSAction::buildGoal(Goal &goal) {
  double timeout_sec = 8.0;
  (void)getInput("timeout_sec", timeout_sec);
  goal.command_id = static_cast<uint8_t>(CommandID::GRAB_KFS);
  goal.payload.clear();
  goal.timeout_sec = static_cast<float>(timeout_sec);
  return true;
}

BT::NodeStatus GrabKFSAction::handleResult(const WrappedResult &result,
                                           uint16_t &error_code) {
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED || !result.result ||
      !result.result->success) {
    error_code = (result.result ? result.result->error_code : 0);
    return BT::NodeStatus::FAILURE;
  }
  error_code = 0;
  return BT::NodeStatus::SUCCESS;
}

namespace {

BT::NodeStatus handleExecuteMechanismResult(
    const BtActionNode<rc26_interfaces::action::ExecuteMechanism>::WrappedResult
        &result,
    uint16_t &error_code) {
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED || !result.result ||
      !result.result->success) {
    error_code = (result.result ? result.result->error_code : 0);
    return BT::NodeStatus::FAILURE;
  }
  error_code = 0;
  return BT::NodeStatus::SUCCESS;
}

} // namespace

// ============================================================================
// MechUpMerlinAction - 梅林区机构抬升
// ============================================================================
MechUpMerlinAction::MechUpMerlinAction(const std::string &name,
                                       const BT::NodeConfig &config)
    : BtActionNode<rc26_interfaces::action::ExecuteMechanism>(
          name, config, "/mechanism/execute", std::chrono::seconds(8)) {}

BT::PortsList MechUpMerlinAction::providedPorts() {
  return BtActionNode<rc26_interfaces::action::ExecuteMechanism>::basePorts(
      8.0);
}

bool MechUpMerlinAction::buildGoal(Goal &goal) {
  double timeout_sec = 8.0;
  (void)getInput("timeout_sec", timeout_sec);
  goal.command_id = static_cast<uint8_t>(CommandID::MECH_UP_MERLIN);
  goal.payload.clear();
  goal.timeout_sec = static_cast<float>(timeout_sec);
  return true;
}

BT::NodeStatus MechUpMerlinAction::handleResult(const WrappedResult &result,
                                                uint16_t &error_code) {
  return handleExecuteMechanismResult(result, error_code);
}

// ============================================================================
// MechDownMerlinAction - 梅林区机构下降
// ============================================================================
MechDownMerlinAction::MechDownMerlinAction(const std::string &name,
                                           const BT::NodeConfig &config)
    : BtActionNode<rc26_interfaces::action::ExecuteMechanism>(
          name, config, "/mechanism/execute", std::chrono::seconds(8)) {}

BT::PortsList MechDownMerlinAction::providedPorts() {
  return BtActionNode<rc26_interfaces::action::ExecuteMechanism>::basePorts(
      8.0);
}

bool MechDownMerlinAction::buildGoal(Goal &goal) {
  double timeout_sec = 8.0;
  (void)getInput("timeout_sec", timeout_sec);
  goal.command_id = static_cast<uint8_t>(CommandID::MECH_DOWN_MERLIN);
  goal.payload.clear();
  goal.timeout_sec = static_cast<float>(timeout_sec);
  return true;
}

BT::NodeStatus MechDownMerlinAction::handleResult(const WrappedResult &result,
                                                  uint16_t &error_code) {
  return handleExecuteMechanismResult(result, error_code);
}

// ============================================================================
// RotateAction - 旋转
// ============================================================================
RotateAction::RotateAction(const std::string &name,
                           const BT::NodeConfig &config)
    : BtActionNode<rc26_interfaces::action::ExecuteMechanism>(
          name, config, "/mechanism/execute", std::chrono::seconds(8)) {}

BT::PortsList RotateAction::providedPorts() {
  auto ports =
      BtActionNode<rc26_interfaces::action::ExecuteMechanism>::basePorts(8.0);
  ports.insert(BT::InputPort<int>("angle", "旋转角度 (90, -90, 180, -180)"));
  return ports;
}

bool RotateAction::buildGoal(Goal &goal) {
  int angle = 0;
  if (!getInput("angle", angle)) {
    return false;
  }
  CommandID cmd = CommandID::ROTATE_POS_90;
  switch (angle) {
  case 90:
    cmd = CommandID::ROTATE_POS_90;
    break;
  case -90:
    cmd = CommandID::ROTATE_NEG_90;
    break;
  case 180:
    cmd = CommandID::ROTATE_POS_180;
    break;
  case -180:
    cmd = CommandID::ROTATE_NEG_180;
    break;
  default:
    return false;
  }
  double timeout_sec = 8.0;
  (void)getInput("timeout_sec", timeout_sec);
  goal.command_id = static_cast<uint8_t>(cmd);
  goal.payload.clear();
  goal.timeout_sec = static_cast<float>(timeout_sec);
  return true;
}

BT::NodeStatus RotateAction::handleResult(const WrappedResult &result,
                                          uint16_t &error_code) {
  return handleExecuteMechanismResult(result, error_code);
}

// ============================================================================
// CheckKFSCondition - 检查 KFS 存在
// ============================================================================
CheckKFSCondition::CheckKFSCondition(const std::string &name,
                                     const BT::NodeConfig &config)
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
CheckLoadCondition::CheckLoadCondition(const std::string &name,
                                       const BT::NodeConfig &config)
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
  (void)config().blackboard->get("kfs_on_board", kfs_count);
  return (kfs_count >= min_load && kfs_count <= max_load)
             ? BT::NodeStatus::SUCCESS
             : BT::NodeStatus::FAILURE;
}

// ============================================================================
// ScanSurroundingsAction - 扫描周围环境
// ============================================================================
ScanSurroundingsAction::ScanSurroundingsAction(const std::string &name,
                                               const BT::NodeConfig &config)
    : BT::StatefulActionNode(name, config) {}

BT::PortsList ScanSurroundingsAction::providedPorts() { return {}; }

BT::NodeStatus ScanSurroundingsAction::onStart() {
  (void)config().blackboard->get("merlin_map", map_);
  if (!map_) {
    map_ = std::make_shared<MerlinMapManager>();
    std::string team = "red";
    (void)config().blackboard->get("team", team);
    const bool layout_loaded =
        (team == "blue") ? map_->initBlueMap() : map_->initRedMap();
    rclcpp::Node *node = nullptr;
    if (config().blackboard->get("node", node) && node) {
      if (layout_loaded) {
        RCLCPP_INFO(node->get_logger(),
                    "ScanSurroundings attached merlin_map: %s",
                    map_->layoutStatus().c_str());
      } else {
        RCLCPP_WARN(node->get_logger(),
                    "ScanSurroundings using merlin_map fallback: %s",
                    map_->layoutStatus().c_str());
      }
    }
    config().blackboard->set("merlin_map", map_);
  }

  int current_grid = 2;
  (void)config().blackboard->get("current_grid", current_grid);
  std::shared_ptr<MerlinRuleWorldModel> world_model;
  (void)config().blackboard->get("merlin_rule_world_model", world_model);
  if (world_model) {
    int resolved_grid = -1;
    if (world_model->resolveCurrentBlock(resolved_grid) && resolved_grid >= 1 &&
        resolved_grid <= 12) {
      current_grid = resolved_grid;
      config().blackboard->set("current_grid", current_grid);
    }
  }

  const int front = map_->getAdjacentGrid(current_grid, MFDirection::FRONT);

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

void ScanSurroundingsAction::onHalted() { phase_ = ScanPhase::ROTATE_LEFT; }

// ============================================================================
// SelectNextGridAction - 选择下一个目标格子
// ============================================================================
SelectNextGridAction::SelectNextGridAction(const std::string &name,
                                           const BT::NodeConfig &config)
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
  if (!map)
    return BT::NodeStatus::FAILURE;

  std::shared_ptr<MerlinRuleWorldModel> world_model;
  (void)config().blackboard->get("merlin_rule_world_model", world_model);
  if (world_model) {
    int resolved_grid = -1;
    if (world_model->resolveCurrentBlock(resolved_grid) && resolved_grid >= 1 &&
        resolved_grid <= 12) {
      current = resolved_grid;
      config().blackboard->set("current_grid", current);
    }
  }
  if (map->isExitBlock(current)) {
    exit_grid = current;
  }

  // P1: 贪婪抓取
  if (kfs_count < target_kfs) {
    int grab = findAdjacentR2KFS(current, map, world_model);
    if (grab > 0) {
      config().blackboard->set("merlin_last_transition_reason",
                               std::string("select_grab_target"));
      setOutput("next_action", std::string("GRAB"));
      setOutput("target_grid", grab);
      return BT::NodeStatus::SUCCESS;
    }
  }
  // P2: 赶路
  int next = findBestPathToExit(current, exit_grid, map, world_model);
  if (next > 0) {
    config().blackboard->set("merlin_last_transition_reason",
                             std::string("select_move_target"));
    setOutput("next_action", std::string("MOVE"));
    setOutput("target_grid", next);
    return BT::NodeStatus::SUCCESS;
  }
  // P3: 等待
  config().blackboard->set("merlin_last_transition_reason",
                           std::string("select_wait_no_legal_target"));
  setOutput("next_action", std::string("WAIT"));
  setOutput("target_grid", current);
  return BT::NodeStatus::SUCCESS;
}

int SelectNextGridAction::findAdjacentR2KFS(
    int current, std::shared_ptr<MerlinMapManager> map,
    const std::shared_ptr<MerlinRuleWorldModel> &world_model) {
  const auto can_move = [&](int from, int to) -> bool {
    if (world_model && world_model->isReady()) {
      return world_model->canMove(from, to).allowed;
    }
    return map->canTraverse(from, to);
  };

  for (auto dir : {MFDirection::FRONT, MFDirection::LEFT, MFDirection::RIGHT}) {
    int adj = map->getAdjacentGrid(current, dir);
    if (adj > 0 && map->getKFS(adj) == KFSType::R2 && can_move(current, adj)) {
      return adj;
    }
  }
  return -1;
}

int SelectNextGridAction::findBestPathToExit(
    int current, int exit_grid, std::shared_ptr<MerlinMapManager> map,
    const std::shared_ptr<MerlinRuleWorldModel> &world_model) {
  int best = -1, min_dist = 100;
  for (auto dir : {MFDirection::FRONT, MFDirection::LEFT, MFDirection::RIGHT}) {
    int adj = map->getAdjacentGrid(current, dir);
    if (adj <= 0) {
      continue;
    }

    const KFSType tracked_state = map->getKFS(adj);
    if (tracked_state == KFSType::R1 || tracked_state == KFSType::R2 ||
        tracked_state == KFSType::FAKE) {
      continue;
    }

    bool allow_transition = false;
    if (world_model && world_model->isReady()) {
      const auto verdict = world_model->canMove(current, adj);
      allow_transition = verdict.allowed;
    } else {
      allow_transition =
          (tracked_state == KFSType::NONE) && map->canTraverse(current, adj);
    }
    if (!allow_transition) {
      continue;
    }

    const int dist = std::abs((adj - 1) / 3 - (exit_grid - 1) / 3) +
                     std::abs((adj - 1) % 3 - (exit_grid - 1) % 3);
    if (dist < min_dist) {
      min_dist = dist;
      best = adj;
    }
  }
  return best;
}

// ============================================================================
// CheckExitCondition - 检查退出条件
// ============================================================================
CheckExitCondition::CheckExitCondition(const std::string &name,
                                       const BT::NodeConfig &config)
    : BT::ConditionNode(name, config) {}

BT::PortsList CheckExitCondition::providedPorts() { return {}; }

BT::NodeStatus CheckExitCondition::tick() {
  int current = 0, kfs_count = 0, target_kfs = 2;
  (void)config().blackboard->get("current_grid", current);
  (void)config().blackboard->get("kfs_on_board", kfs_count);
  (void)config().blackboard->get("target_kfs_count", target_kfs);
  std::shared_ptr<MerlinRuleWorldModel> world_model;
  (void)config().blackboard->get("merlin_rule_world_model", world_model);
  if (world_model) {
    int resolved_grid = -1;
    if (world_model->resolveCurrentBlock(resolved_grid) && resolved_grid >= 1 &&
        resolved_grid <= 12) {
      current = resolved_grid;
      config().blackboard->set("current_grid", current);
    }
  }
  std::shared_ptr<MerlinMapManager> map;
  if (!config().blackboard->get("merlin_map", map) || !map) {
    return BT::NodeStatus::FAILURE;
  }
  const bool at_exit = map->isExitBlock(current);
  return (at_exit && kfs_count >= target_kfs) ? BT::NodeStatus::SUCCESS
                                              : BT::NodeStatus::FAILURE;
}

// ============================================================================
// CheckR1BlockingCondition - 检查R1阻挡
// ============================================================================
CheckR1BlockingCondition::CheckR1BlockingCondition(const std::string &name,
                                                   const BT::NodeConfig &config)
    : BT::ConditionNode(name, config) {}

BT::PortsList CheckR1BlockingCondition::providedPorts() { return {}; }

BT::NodeStatus CheckR1BlockingCondition::tick() {
  std::shared_ptr<MerlinMapManager> map;
  (void)config().blackboard->get("merlin_map", map);
  if (!map)
    return BT::NodeStatus::FAILURE;
  int current = 2;
  (void)config().blackboard->get("current_grid", current);
  std::shared_ptr<MerlinRuleWorldModel> world_model;
  (void)config().blackboard->get("merlin_rule_world_model", world_model);
  if (world_model) {
    int resolved_grid = -1;
    if (world_model->resolveCurrentBlock(resolved_grid) && resolved_grid >= 1 &&
        resolved_grid <= 12) {
      current = resolved_grid;
      config().blackboard->set("current_grid", current);
    }
  }
  int front = map->getAdjacentGrid(current, MFDirection::FRONT);
  if (front > 0 && map->getKFS(front) == KFSType::R1) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

// ============================================================================
// IncrementKFSCountAction - 更新KFS计数
// ============================================================================
IncrementKFSCountAction::IncrementKFSCountAction(const std::string &name,
                                                 const BT::NodeConfig &config)
    : BT::SyncActionNode(name, config) {}

BT::PortsList IncrementKFSCountAction::providedPorts() { return {}; }

BT::NodeStatus IncrementKFSCountAction::tick() {
  int count = 0;
  (void)config().blackboard->get("kfs_on_board", count);
  config().blackboard->set("kfs_on_board", count + 1);
  return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// UpdateMapKFSAction - 更新地图KFS状态
// ============================================================================
UpdateMapKFSAction::UpdateMapKFSAction(const std::string &name,
                                       const BT::NodeConfig &config)
    : BT::SyncActionNode(name, config) {}

BT::PortsList UpdateMapKFSAction::providedPorts() {
  return {
      BT::InputPort<int>("grid_id"),
      BT::InputPort<int>("kfs_type", static_cast<int>(0), "目标 KFS 类型"),
  };
}

BT::NodeStatus UpdateMapKFSAction::tick() {
  int grid_id = 0, kfs_type = 0;
  if (!getInput("grid_id", grid_id))
    return BT::NodeStatus::FAILURE;
  getInput("kfs_type", kfs_type);
  if (grid_id < 1 || grid_id > 12 ||
      kfs_type < static_cast<int>(KFSType::NONE) ||
      kfs_type > static_cast<int>(KFSType::UNKNOWN)) {
    return BT::NodeStatus::FAILURE;
  }
  std::shared_ptr<MerlinMapManager> map;
  if (!config().blackboard->get("merlin_map", map) || !map) {
    return BT::NodeStatus::FAILURE;
  }
  map->setKFS(grid_id, static_cast<KFSType>(kfs_type));
  return BT::NodeStatus::SUCCESS;
}

// 注册函数
// ============================================================================
void registerMFAreaNodes(BT::BehaviorTreeFactory &factory) {
  factory.registerNodeType<StairClimbAction>("StairClimb");
  factory.registerNodeType<StairDescendAction>("StairDescend");
  factory.registerNodeType<GrabKFSAction>("GrabKFS");
  factory.registerNodeType<MechUpMerlinAction>("MechUpMerlin");
  factory.registerNodeType<MechDownMerlinAction>("MechDownMerlin");
  factory.registerNodeType<RotateAction>("Rotate");
  factory.registerNodeType<CheckKFSCondition>("CheckKFS");
  factory.registerNodeType<CheckLoadCondition>("CheckLoad");
  factory.registerNodeType<ScanSurroundingsAction>("ScanSurroundings");
  factory.registerNodeType<SelectNextGridAction>("SelectNextGrid");
  factory.registerNodeType<CheckExitCondition>("CheckExitCondition");
  factory.registerNodeType<CheckR1BlockingCondition>("CheckR1Blocking");
  factory.registerNodeType<IncrementKFSCountAction>("IncrementKFSCount");
  factory.registerNodeType<UpdateMapKFSAction>("UpdateMapKFS");
}

} // namespace rc26_decision
