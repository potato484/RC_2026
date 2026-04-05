#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "rc26_decision/bt/chinese_localization_module.hpp"
#include "rc26_interfaces/msg/behavior_tree_blackboard.hpp"
#include "rc26_interfaces/msg/behavior_tree_debug_state.hpp"
#include "rc26_interfaces/msg/behavior_tree_event_array.hpp"
#include "rc26_interfaces/msg/behavior_tree_localization.hpp"
#include "rc26_interfaces/msg/behavior_tree_model.hpp"
#include "rc26_interfaces/msg/behavior_tree_snapshot.hpp"
#include "rc26_interfaces/msg/behavior_tree_trace.hpp"
#include "rc26_interfaces/msg/behavior_tree_trace_line.hpp"

namespace rc26_decision {

class BtRuntimePublisher {
public:
  BtRuntimePublisher(rclcpp::Node *node, BT::Tree &tree,
                     BT::Blackboard::Ptr blackboard,
                     const std::string &tree_file);

  void beginTick();
  void completeTick(BT::NodeStatus tree_status, float tick_duration_ms);
  void publishSnapshotOnly(BT::NodeStatus tree_status = BT::NodeStatus::IDLE,
                           float tick_duration_ms = 0.0f);
  void publishDebugState(bool manual_mode, bool playing, bool terminal,
                         uint32_t play_interval_ms);
  void emitCustomTrace(const BT::TreeNode &node, const std::string &kind,
                       BT::NodeStatus status, const std::string &detail);
  uint64_t currentTickSeq() const { return tick_seq_; }
  uint8_t currentTreeStatus() const { return current_tree_status_; }

private:
  struct LocalizedNodeInfo {
    std::string display_name;
    std::string registration_display_name;
    std::string subtree_display_name;
  };

  void buildAndPublishModel();
  void publishSnapshot(BT::NodeStatus tree_status, float tick_duration_ms);
  void publishBlackboard();
  void publishLocalization();
  void publishTrace(BT::NodeStatus tree_status);
  void flushEvents();
  void refreshLocalizationCache();
  void installTraceCallbacks();

  BT::NodeStatus handlePreTick(BT::TreeNode &node);
  BT::NodeStatus handlePostTick(BT::TreeNode &node, BT::NodeStatus status);
  void appendTraceLine(const BT::TreeNode &node, const std::string &kind,
                       BT::NodeStatus status, const std::string &text,
                       const std::string &detail = "");
  std::string formatTraceText(const BT::TreeNode &node,
                              const std::string &text) const;
  std::string nodeDisplayName(const BT::TreeNode &node) const;
  std::string registrationDisplayName(const BT::TreeNode &node) const;
  std::string statusLabel(BT::NodeStatus status) const;
  std::string nodeTypeLabel(const BT::TreeNode &node) const;
  std::string controlNodeSummary(const BT::TreeNode &node,
                                 BT::NodeStatus status) const;
  std::string decoratorSummary(const BT::TreeNode &node,
                               BT::NodeStatus status) const;
  std::string conditionDetail(const BT::TreeNode &node,
                              BT::NodeStatus status) const;
  std::string actionStartDetail(const BT::TreeNode &node) const;
  std::string actionFinishDetail(const BT::TreeNode &node,
                                 BT::NodeStatus status) const;
  std::string collectPortSummary(const BT::TreeNode &node) const;
  std::string knownNodeDetail(const BT::TreeNode &node,
                              BT::NodeStatus status) const;
  size_t depthForNode(uint16_t uid) const;

  uint8_t toMsgNodeType(BT::NodeType t);
  uint8_t toMsgStatus(BT::NodeStatus s);
  void collectRunningPath(const BT::TreeNode *node,
                          std::vector<uint16_t> &path);

  rclcpp::Node *node_;
  BT::Tree &tree_;
  BT::Blackboard::Ptr blackboard_;
  std::string tree_file_;

  rclcpp::Publisher<rc26_interfaces::msg::BehaviorTreeModel>::SharedPtr
      pub_model_;
  rclcpp::Publisher<rc26_interfaces::msg::BehaviorTreeSnapshot>::SharedPtr
      pub_snapshot_;
  rclcpp::Publisher<rc26_interfaces::msg::BehaviorTreeBlackboard>::SharedPtr
      pub_blackboard_;
  rclcpp::Publisher<rc26_interfaces::msg::BehaviorTreeLocalization>::SharedPtr
      pub_localization_;
  rclcpp::Publisher<rc26_interfaces::msg::BehaviorTreeEventArray>::SharedPtr
      pub_events_;
  rclcpp::Publisher<rc26_interfaces::msg::BehaviorTreeTrace>::SharedPtr
      pub_trace_;
  rclcpp::Publisher<rc26_interfaces::msg::BehaviorTreeDebugState>::SharedPtr
      pub_debug_state_;

  rclcpp::TimerBase::SharedPtr bb_timer_;
  rclcpp::TimerBase::SharedPtr localization_timer_;

  uint64_t tick_seq_{0};
  uint32_t snapshot_decimation_{1};
  uint8_t current_tree_status_{0};
  bool trace_active_{false};
  uint32_t trace_line_seq_{0};

  std::mutex event_mutex_;
  std::vector<rc26_interfaces::msg::BehaviorTreeEvent> event_buffer_;
  std::vector<BT::TreeNode::StatusChangeSubscriber> subscribers_;

  std::vector<const BT::TreeNode *> all_nodes_;
  std::vector<rc26_interfaces::msg::BehaviorTreeTraceLine> trace_lines_;
  std::vector<std::string> bb_whitelist_;
  std::unique_ptr<ChineseLocalizationModule> localization_module_;
  std::unordered_map<uint16_t, uint16_t> parent_map_;
  std::unordered_map<uint16_t, std::vector<uint16_t>> children_map_;
  std::unordered_map<uint16_t, std::string> subtree_map_;
  std::unordered_map<uint16_t, size_t> depth_map_;
  std::unordered_map<uint16_t, LocalizedNodeInfo> localized_nodes_;
  std::unordered_map<uint16_t, BT::NodeStatus> pre_tick_status_;
};

} // namespace rc26_decision
