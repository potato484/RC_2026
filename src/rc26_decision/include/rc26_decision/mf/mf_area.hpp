// 梅林区 (MF Area) 行为树节点
#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

#include "rc26_decision/common/bt_action_node.hpp"
#include "rc26_interfaces/action/execute_mechanism.hpp"

namespace rc26_decision {

// ============================================================================
// 梅林地图数据结构
// ============================================================================
enum class KFSType : uint8_t {
  NONE = 0,
  R1 = 1,
  R2 = 2,
  FAKE = 3,
  UNKNOWN = 4
};

enum class MFDirection : uint8_t { LEFT = 0, FRONT = 1, RIGHT = 2, BACK = 3 };

struct MerlinGridCell {
  int depth; // 0=0mm, 1=200mm, 2=400mm, 3=600mm
  KFSType kfs;
};

class MerlinMapManager {
public:
  MerlinMapManager();
  bool initRedMap();
  bool initBlueMap();
  int getDepth(int grid_id) const;
  KFSType getKFS(int grid_id) const;
  void setKFS(int grid_id, KFSType type);
  bool canTraverse(int from, int to) const;
  bool isWalkable(int from, int to) const;
  int getAdjacentGrid(int current, MFDirection dir) const;
  bool isExitBlock(int grid_id) const;
  std::string layoutStatus() const;

private:
  bool initFromWorldLayout(const std::string &team);
  void initLegacyRedMapLocked();
  void initLegacyBlueMapLocked();

  mutable std::mutex mutex_;
  std::array<MerlinGridCell, 13> cells_; // index 0 unused, 1-12 for grids
  std::vector<int> exit_blocks_;
  std::string layout_status_;
};

// 上阶梯节点
class StairClimbAction : public BT::StatefulActionNode {
public:
  StairClimbAction(const std::string &name, const BT::NodeConfig &config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  bool level_start_set_{false};
  int32_t level_start_{0};
  static constexpr int32_t kStairLevelDelta = 3;
};

// 下阶梯节点
class StairDescendAction : public BT::StatefulActionNode {
public:
  StairDescendAction(const std::string &name, const BT::NodeConfig &config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  bool level_start_set_{false};
  int32_t level_start_{0};
  static constexpr int32_t kStairLevelDelta = 3;
};

// 夹取 KFS 节点
class GrabKFSAction
    : public BtActionNode<rc26_interfaces::action::ExecuteMechanism> {
public:
  GrabKFSAction(const std::string &name, const BT::NodeConfig &config);

  static BT::PortsList providedPorts();

protected:
  bool buildGoal(Goal &goal) override;
  BT::NodeStatus handleResult(const WrappedResult &result,
                              uint16_t &error_code) override;
};

// 检查 KFS 存在条件节点
class CheckKFSCondition : public BT::ConditionNode {
public:
  CheckKFSCondition(const std::string &name, const BT::NodeConfig &config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

// 检查装载数量条件节点
class CheckLoadCondition : public BT::ConditionNode {
public:
  CheckLoadCondition(const std::string &name, const BT::NodeConfig &config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

// ============================================================================
// 新增节点：扫描周围环境
// ============================================================================
class ScanSurroundingsAction : public BT::StatefulActionNode {
public:
  ScanSurroundingsAction(const std::string &name, const BT::NodeConfig &config);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  enum class ScanPhase {
    ROTATE_LEFT,
    SCAN_LEFT,
    ROTATE_FRONT,
    SCAN_FRONT,
    ROTATE_RIGHT,
    SCAN_RIGHT,
    ROTATE_CENTER,
    DONE
  };
  ScanPhase phase_{ScanPhase::ROTATE_LEFT};
  std::shared_ptr<MerlinMapManager> map_;
};

// ============================================================================
// 新增节点：选择下一个目标格子
// ============================================================================
class SelectNextGridAction : public BT::SyncActionNode {
public:
  SelectNextGridAction(const std::string &name, const BT::NodeConfig &config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  int findAdjacentR2KFS(int current, std::shared_ptr<MerlinMapManager> map);
  int findBestPathToExit(int current, int exit_grid,
                         std::shared_ptr<MerlinMapManager> map);
};

// ============================================================================
// 新增节点：检查退出条件
// ============================================================================
class CheckExitCondition : public BT::ConditionNode {
public:
  CheckExitCondition(const std::string &name, const BT::NodeConfig &config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

// ============================================================================
// 新增节点：检查R1阻挡
// ============================================================================
class CheckR1BlockingCondition : public BT::ConditionNode {
public:
  CheckR1BlockingCondition(const std::string &name,
                           const BT::NodeConfig &config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

// ============================================================================
// 新增节点：更新KFS计数
// ============================================================================
class IncrementKFSCountAction : public BT::SyncActionNode {
public:
  IncrementKFSCountAction(const std::string &name,
                          const BT::NodeConfig &config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

// ============================================================================
// 新增节点：更新地图KFS状态
// ============================================================================
class UpdateMapKFSAction : public BT::SyncActionNode {
public:
  UpdateMapKFSAction(const std::string &name, const BT::NodeConfig &config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

// 注册 MF 区域所有节点
void registerMFAreaNodes(BT::BehaviorTreeFactory &factory);

} // namespace rc26_decision
