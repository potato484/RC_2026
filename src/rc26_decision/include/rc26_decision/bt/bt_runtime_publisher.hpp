#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

#include "rc26_decision/bt/chinese_localization_module.hpp"
#include "rc26_interfaces/msg/behavior_tree_blackboard.hpp"
#include "rc26_interfaces/msg/behavior_tree_event_array.hpp"
#include "rc26_interfaces/msg/behavior_tree_localization.hpp"
#include "rc26_interfaces/msg/behavior_tree_model.hpp"
#include "rc26_interfaces/msg/behavior_tree_snapshot.hpp"

#include <mutex>
#include <memory>
#include <string>
#include <vector>

namespace rc26_decision {

class BtRuntimePublisher {
public:
  BtRuntimePublisher(rclcpp::Node *node, BT::Tree &tree,
                     BT::Blackboard::Ptr blackboard,
                     const std::string &tree_file);

  void onTick(BT::NodeStatus tree_status, float tick_duration_ms);

private:
  void buildAndPublishModel();
  void publishSnapshot(BT::NodeStatus tree_status, float tick_duration_ms);
  void publishBlackboard();
  void publishLocalization();
  void flushEvents();

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

  rclcpp::TimerBase::SharedPtr bb_timer_;
  rclcpp::TimerBase::SharedPtr localization_timer_;

  uint64_t tick_seq_{0};
  uint32_t snapshot_decimation_{1};

  std::mutex event_mutex_;
  std::vector<rc26_interfaces::msg::BehaviorTreeEvent> event_buffer_;
  std::vector<BT::TreeNode::StatusChangeSubscriber> subscribers_;

  std::vector<const BT::TreeNode *> all_nodes_;
  std::vector<std::string> bb_whitelist_;
  std::unique_ptr<ChineseLocalizationModule> localization_module_;
};

} // namespace rc26_decision
