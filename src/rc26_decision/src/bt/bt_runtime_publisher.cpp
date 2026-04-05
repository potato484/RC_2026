#include "rc26_decision/bt/bt_runtime_publisher.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/control_node.h>
#include <behaviortree_cpp/decorator_node.h>
#include <behaviortree_cpp/decorators/subtree_node.h>

#include <algorithm>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>

#include "rc26_interfaces/msg/behavior_tree_event.hpp"
#include "rc26_interfaces/msg/behavior_tree_node_model.hpp"
#include "rc26_interfaces/msg/behavior_tree_node_state.hpp"
#include "rc26_interfaces/msg/behavior_tree_port.hpp"

namespace rc26_decision {

namespace {

template <typename T>
T declareParameterIfNeeded(rclcpp::Node *node, const std::string &name,
                           const T &default_value) {
  if (!node->has_parameter(name)) {
    return node->declare_parameter<T>(name, default_value);
  }
  return node->get_parameter(name).get_value<T>();
}

std::string formatTimestamp(const rclcpp::Time &stamp) {
  const auto ns = stamp.nanoseconds();
  const auto sec = static_cast<std::time_t>(ns / 1000000000LL);
  const auto ms = static_cast<int>((ns % 1000000000LL) / 1000000LL);

  std::tm local_tm{};
  localtime_r(&sec, &local_tm);

  std::ostringstream oss;
  oss << std::put_time(&local_tm, "%Y-%m-%d %H:%M:%S") << '.'
      << std::setw(3) << std::setfill('0') << ms;
  return oss.str();
}

template <typename T>
std::optional<T> getBlackboardValue(const BT::TreeNode &node,
                                    const std::string &key) {
  T value{};
  if (node.config().blackboard && node.config().blackboard->get(key, value)) {
    return value;
  }
  return std::nullopt;
}

std::string boolLabel(bool value) { return value ? "true" : "false"; }

std::string stringFromView(BT::StringView value) {
  return std::string(value.data(), value.size());
}

} // namespace

BtRuntimePublisher::BtRuntimePublisher(rclcpp::Node *node, BT::Tree &tree,
                                       BT::Blackboard::Ptr blackboard,
                                       const std::string &tree_file)
    : node_(node), tree_(tree), blackboard_(std::move(blackboard)),
      tree_file_(tree_file) {

  const bool runtime_enabled =
      declareParameterIfNeeded<bool>(node_, "bt_runtime.enable", true);
  const auto model_topic = declareParameterIfNeeded<std::string>(
      node_, "bt_runtime.model_topic", "r2/bt/model");
  const auto snapshot_topic = declareParameterIfNeeded<std::string>(
      node_, "bt_runtime.snapshot_topic", "r2/bt/snapshot");
  const auto bb_topic = declareParameterIfNeeded<std::string>(
      node_, "bt_runtime.blackboard_topic", "r2/bt/blackboard");
  const auto events_topic = declareParameterIfNeeded<std::string>(
      node_, "bt_runtime.events_topic", "r2/bt/events");
  const auto trace_topic = declareParameterIfNeeded<std::string>(
      node_, "bt_runtime.trace_topic", "r2/bt/trace");
  const auto debug_state_topic = declareParameterIfNeeded<std::string>(
      node_, "bt_runtime.debug_state_topic", "r2/bt/debug_state");
  const auto localization_topic = declareParameterIfNeeded<std::string>(
      node_, "bt_runtime.localization_topic", "r2/bt/localization");
  const auto localization_default =
      ament_index_cpp::get_package_share_directory("rc26_decision") +
      "/config/bt_localization.yaml";
  auto localization_config = declareParameterIfNeeded<std::string>(
      node_, "bt_runtime.localization_config", localization_default);
  snapshot_decimation_ = static_cast<uint32_t>(std::max<int64_t>(
      1, declareParameterIfNeeded<int>(node_, "bt_runtime.snapshot_decimation",
                                       1)));
  const int bb_ms = declareParameterIfNeeded<int>(
      node_, "bt_runtime.blackboard_publish_ms", 200);
  int localization_reload_ms = declareParameterIfNeeded<int>(
      node_, "bt_runtime.localization_reload_ms", 1000);
  bb_whitelist_ = declareParameterIfNeeded<std::vector<std::string>>(
      node_,
      "bt_runtime.blackboard_whitelist",
      std::vector<std::string>{
          "vision_running",
          "vision_ok",
          "vision_has_target",
          "vision_attr_kind",
          "vision_distance_m",
          "vision_score",
          "vision_bbox_cx",
          "vision_bbox_cy",
          "vision_current_model",
          "mechanism_tip_state",
          "mechanism_hal_open",
          "mechanism_locked_tip_slot",
          "mechanism_comm_health_level",
          "last_action_error_code",
          "system_error",
          "loc_level",
          "loc_reason",
          "loc_control_degraded",
          "loc_graph_health",
          "loc_optimizer_ready",
          "loc_route_score",
          "loc_route_risk_level",
          "loc_recommended_profile",
          "loc_last_profile",
          "loc_guard_required",
          "loc_guard_reason",
          "current_level",
          "stair_delta",
          "base_ground_stable",
          "is_lifted",
          "stable_operation",
          "stair_climb_done",
          "stair_descend_done",
          "level_start",
          "team",
          "target_kfs_count",
          "kfs_on_board",
          "current_grid",
          "next_action",
          "target_grid",
          "exit_grid",
          "merlin_last_transition_reason",
      });

  if (!runtime_enabled) {
    return;
  }

  if (localization_config.empty()) {
    localization_config = localization_default;
  }

  rclcpp::QoS model_qos(rclcpp::KeepLast(1));
  model_qos.reliable().transient_local();
  pub_model_ =
      node_->create_publisher<rc26_interfaces::msg::BehaviorTreeModel>(
          model_topic, model_qos);
  pub_localization_ =
      node_->create_publisher<rc26_interfaces::msg::BehaviorTreeLocalization>(
          localization_topic, model_qos);

  rclcpp::QoS runtime_qos(rclcpp::KeepLast(10));
  runtime_qos.reliable();
  pub_snapshot_ =
      node_->create_publisher<rc26_interfaces::msg::BehaviorTreeSnapshot>(
          snapshot_topic, runtime_qos);
  pub_blackboard_ =
      node_->create_publisher<rc26_interfaces::msg::BehaviorTreeBlackboard>(
          bb_topic, runtime_qos);
  pub_trace_ =
      node_->create_publisher<rc26_interfaces::msg::BehaviorTreeTrace>(
          trace_topic, runtime_qos);
  pub_debug_state_ =
      node_->create_publisher<rc26_interfaces::msg::BehaviorTreeDebugState>(
          debug_state_topic, runtime_qos);

  rclcpp::QoS events_qos(rclcpp::KeepLast(100));
  events_qos.reliable();
  pub_events_ =
      node_->create_publisher<rc26_interfaces::msg::BehaviorTreeEventArray>(
          events_topic, events_qos);

  tree_.applyVisitor([this](BT::TreeNode *n) {
    all_nodes_.push_back(n);

    auto subscriber = n->subscribeToStatusChange(
        [this, n](BT::TimePoint /*tp*/, const BT::TreeNode & /*node*/,
                  BT::NodeStatus prev, BT::NodeStatus cur) {
          rc26_interfaces::msg::BehaviorTreeEvent event;
          event.stamp = node_->get_clock()->now();
          event.uid = n->UID();
          event.node_name = n->name();
          event.full_path = n->fullPath();
          event.prev_status = toMsgStatus(prev);
          event.status = toMsgStatus(cur);

          std::lock_guard<std::mutex> lock(event_mutex_);
          event_buffer_.push_back(std::move(event));
        });
    subscribers_.push_back(std::move(subscriber));
  });

  bb_timer_ = node_->create_wall_timer(std::chrono::milliseconds(bb_ms),
                                       [this]() { publishBlackboard(); });

  localization_module_ = std::make_unique<ChineseLocalizationModule>(
      node_, std::move(localization_config));
  localization_reload_ms = std::max(localization_reload_ms, 200);
  localization_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(localization_reload_ms), [this]() {
        if (localization_module_ && localization_module_->reloadIfChanged()) {
          refreshLocalizationCache();
          publishLocalization();
        }
      });

  buildAndPublishModel();
  refreshLocalizationCache();
  publishLocalization();
  installTraceCallbacks();
}

void BtRuntimePublisher::beginTick() {
  ++tick_seq_;
  trace_active_ = true;
  trace_line_seq_ = 0;
  trace_lines_.clear();
  pre_tick_status_.clear();
}

void BtRuntimePublisher::completeTick(BT::NodeStatus tree_status,
                                      float tick_duration_ms) {
  if (!pub_snapshot_) {
    return;
  }

  current_tree_status_ = toMsgStatus(tree_status);

  const bool publish_snapshot =
      tick_seq_ == 1 ||
      ((tick_seq_ - 1) % snapshot_decimation_) == 0 ||
      tree_status != BT::NodeStatus::RUNNING;
  if (publish_snapshot) {
    publishSnapshot(tree_status, tick_duration_ms);
  }

  if (tree_.rootNode()) {
    appendTraceLine(*tree_.rootNode(), "tick_complete", tree_status,
                    "← 根节点 Tick 执行完成，最终状态: " +
                        statusLabel(tree_status));
  }
  publishTrace(tree_status);
  flushEvents();
  trace_active_ = false;
}

void BtRuntimePublisher::publishSnapshotOnly(BT::NodeStatus tree_status,
                                             float tick_duration_ms) {
  current_tree_status_ = toMsgStatus(tree_status);
  publishSnapshot(tree_status, tick_duration_ms);
}

void BtRuntimePublisher::publishDebugState(bool manual_mode, bool playing,
                                           bool terminal,
                                           uint32_t play_interval_ms) {
  if (!pub_debug_state_) {
    return;
  }

  rc26_interfaces::msg::BehaviorTreeDebugState msg;
  msg.header.stamp = node_->get_clock()->now();
  msg.manual_mode = manual_mode;
  msg.playing = playing;
  msg.terminal = terminal;
  msg.tick_seq = tick_seq_;
  msg.tree_status = current_tree_status_;
  msg.play_interval_ms = play_interval_ms;
  msg.tree_file = tree_file_;
  pub_debug_state_->publish(msg);
}

void BtRuntimePublisher::emitCustomTrace(const BT::TreeNode &node,
                                         const std::string &kind,
                                         BT::NodeStatus status,
                                         const std::string &detail) {
  if (!trace_active_) {
    return;
  }
  appendTraceLine(node, kind, status, detail, detail);
}

void BtRuntimePublisher::buildAndPublishModel() {
  rc26_interfaces::msg::BehaviorTreeModel model;
  model.header.stamp = node_->get_clock()->now();
  model.tree_file = tree_file_;

  parent_map_.clear();
  children_map_.clear();
  subtree_map_.clear();
  depth_map_.clear();
  std::unordered_map<uint16_t, uint16_t> child_index_map;

  for (auto &subtree : tree_.subtrees) {
    model.subtree_ids.push_back(subtree->tree_ID);
    for (auto &node : subtree->nodes) {
      subtree_map_[node->UID()] = subtree->tree_ID;

      if (auto *ctrl = dynamic_cast<const BT::ControlNode *>(node.get())) {
        uint16_t child_index = 0;
        for (auto *child : ctrl->children()) {
          children_map_[node->UID()].push_back(child->UID());
          parent_map_[child->UID()] = node->UID();
          child_index_map[child->UID()] = child_index++;
        }
      } else if (auto *dec =
                     dynamic_cast<const BT::DecoratorNode *>(node.get())) {
        if (dec->child()) {
          children_map_[node->UID()].push_back(dec->child()->UID());
          parent_map_[dec->child()->UID()] = node->UID();
          child_index_map[dec->child()->UID()] = 0;
        }
      }
    }
  }

  if (!tree_.subtrees.empty()) {
    model.main_tree_id = tree_.subtrees.front()->tree_ID;
  }
  if (tree_.rootNode()) {
    model.root_uid = tree_.rootNode()->UID();
  }

  for (auto *node : all_nodes_) {
    size_t depth = 0;
    uint16_t current_uid = node->UID();
    while (parent_map_.count(current_uid) > 0) {
      ++depth;
      current_uid = parent_map_[current_uid];
    }
    depth_map_[node->UID()] = depth;

    rc26_interfaces::msg::BehaviorTreeNodeModel model_node;
    model_node.uid = node->UID();
    model_node.instance_name = node->name();
    model_node.registration_name = node->registrationName();
    model_node.node_type = toMsgNodeType(node->type());
    model_node.full_path = node->fullPath();

    if (const auto subtree_it = subtree_map_.find(node->UID());
        subtree_it != subtree_map_.end()) {
      model_node.subtree_id = subtree_it->second;
    }
    if (const auto *subtree_node =
            dynamic_cast<const BT::SubTreeNode *>(node)) {
      model_node.subtree_ref_id = subtree_node->subtreeID();
    }
    if (const auto parent_it = parent_map_.find(node->UID());
        parent_it != parent_map_.end()) {
      model_node.parent_uid = parent_it->second;
    }
    if (const auto child_index_it = child_index_map.find(node->UID());
        child_index_it != child_index_map.end()) {
      model_node.child_index = child_index_it->second;
    }
    if (const auto depth_it = depth_map_.find(node->UID());
        depth_it != depth_map_.end()) {
      model_node.depth = static_cast<uint16_t>(
          std::min(depth_it->second,
                   static_cast<size_t>(std::numeric_limits<uint16_t>::max())));
    }
    if (const auto child_it = children_map_.find(node->UID());
        child_it != children_map_.end()) {
      model_node.children_uids = child_it->second;
    }
    model_node.child_count = static_cast<uint16_t>(
        std::min(model_node.children_uids.size(),
                 static_cast<size_t>(std::numeric_limits<uint16_t>::max())));

    const auto &config = node->config();
    const auto *manifest = config.manifest;
    for (const auto &[port_name, remapped] : config.input_ports) {
      rc26_interfaces::msg::BehaviorTreePort port;
      port.name = port_name;
      port.remapped_key = remapped;
      port.direction = rc26_interfaces::msg::BehaviorTreePort::INPUT;
      if (manifest) {
        if (const auto port_it = manifest->ports.find(port_name);
            port_it != manifest->ports.end()) {
          port.declared_type = BT::demangle(port_it->second.type());
          if (port_it->second.direction() == BT::PortDirection::INOUT) {
            port.direction = rc26_interfaces::msg::BehaviorTreePort::INOUT;
          }
        }
      }
      model_node.ports.push_back(std::move(port));
    }
    for (const auto &[port_name, remapped] : config.output_ports) {
      if (config.input_ports.count(port_name) > 0) {
        continue;
      }
      rc26_interfaces::msg::BehaviorTreePort port;
      port.name = port_name;
      port.remapped_key = remapped;
      port.direction = rc26_interfaces::msg::BehaviorTreePort::OUTPUT;
      if (manifest) {
        if (const auto port_it = manifest->ports.find(port_name);
            port_it != manifest->ports.end()) {
          port.declared_type = BT::demangle(port_it->second.type());
        }
      }
      model_node.ports.push_back(std::move(port));
    }

    model.nodes.push_back(std::move(model_node));
  }

  pub_model_->publish(model);
}

void BtRuntimePublisher::publishSnapshot(BT::NodeStatus tree_status,
                                         float tick_duration_ms) {
  rc26_interfaces::msg::BehaviorTreeSnapshot snapshot;
  snapshot.header.stamp = node_->get_clock()->now();
  snapshot.tick_seq = tick_seq_;
  snapshot.tree_status = toMsgStatus(tree_status);
  snapshot.tick_duration_ms = tick_duration_ms;

  for (auto *node : all_nodes_) {
    rc26_interfaces::msg::BehaviorTreeNodeState state;
    state.uid = node->UID();
    state.status = toMsgStatus(node->status());
    snapshot.nodes.push_back(state);
  }

  if (tree_.rootNode()) {
    collectRunningPath(tree_.rootNode(), snapshot.running_path_uids);
  }

  snapshot.active_subtree_id =
      tree_.subtrees.empty() ? "" : tree_.subtrees.front()->tree_ID;
  if (!snapshot.running_path_uids.empty()) {
    const uint16_t deepest = snapshot.running_path_uids.back();
    if (const auto subtree_it = subtree_map_.find(deepest);
        subtree_it != subtree_map_.end()) {
      snapshot.active_subtree_id = subtree_it->second;
    }
  }

  pub_snapshot_->publish(snapshot);
}

void BtRuntimePublisher::publishBlackboard() {
  if (!pub_blackboard_) {
    return;
  }

  rc26_interfaces::msg::BehaviorTreeBlackboard msg;
  msg.header.stamp = node_->get_clock()->now();

  for (const auto &key : bb_whitelist_) {
    auto entry = blackboard_->getEntry(key);
    if (!entry) {
      continue;
    }

    rc26_interfaces::msg::BehaviorTreeBlackboardEntry item;
    item.key = key;

    try {
      std::lock_guard<std::mutex> lock(entry->entry_mutex);
      if (entry->value.empty()) {
        continue;
      }
      item.type_name = BT::demangle(entry->value.type());
      if (entry->value.isType<bool>()) {
        item.value = entry->value.cast<bool>() ? "true" : "false";
      } else {
        item.value = entry->value.cast<std::string>();
      }
    } catch (...) {
      continue;
    }

    msg.entries.push_back(std::move(item));
  }

  pub_blackboard_->publish(msg);
}

void BtRuntimePublisher::publishLocalization() {
  if (!pub_localization_ || !localization_module_) {
    return;
  }

  auto msg =
      localization_module_->buildMessage(tree_file_, all_nodes_, bb_whitelist_);
  msg.header.stamp = node_->get_clock()->now();
  pub_localization_->publish(msg);
}

void BtRuntimePublisher::publishTrace(BT::NodeStatus tree_status) {
  if (!pub_trace_) {
    return;
  }

  rc26_interfaces::msg::BehaviorTreeTrace msg;
  msg.header.stamp = node_->get_clock()->now();
  msg.tick_seq = tick_seq_;
  msg.tree_status = toMsgStatus(tree_status);
  msg.lines = trace_lines_;
  pub_trace_->publish(msg);
}

void BtRuntimePublisher::flushEvents() {
  std::vector<rc26_interfaces::msg::BehaviorTreeEvent> events;
  {
    std::lock_guard<std::mutex> lock(event_mutex_);
    events.swap(event_buffer_);
  }
  if (events.empty()) {
    return;
  }

  rc26_interfaces::msg::BehaviorTreeEventArray msg;
  msg.header.stamp = node_->get_clock()->now();
  msg.events = std::move(events);
  pub_events_->publish(msg);
}

void BtRuntimePublisher::refreshLocalizationCache() {
  localized_nodes_.clear();
  if (!localization_module_) {
    return;
  }

  for (const auto *node : all_nodes_) {
    if (!node) {
      continue;
    }
    const auto localized = localization_module_->getChineseNode(*node);
    localized_nodes_[node->UID()] = LocalizedNodeInfo{
        localized.display_name, localized.registration_display_name,
        localized.subtree_display_name};
  }
}

void BtRuntimePublisher::installTraceCallbacks() {
  tree_.applyVisitor([this](BT::TreeNode *node) {
    node->setPreTickFunction(
        [this](BT::TreeNode &current) { return handlePreTick(current); });
    node->setPostTickFunction([this](BT::TreeNode &current,
                                     BT::NodeStatus status) {
      return handlePostTick(current, status);
    });
  });
}

BT::NodeStatus BtRuntimePublisher::handlePreTick(BT::TreeNode &node) {
  if (!trace_active_) {
    return BT::NodeStatus::IDLE;
  }

  pre_tick_status_[node.UID()] = node.status();
  appendTraceLine(node, "enter", node.status(),
                  "→ 进入" + nodeDisplayName(node) + " (" +
                      registrationDisplayName(node) + ")");

  if (node.type() == BT::NodeType::ACTION && node.status() == BT::NodeStatus::IDLE) {
    auto detail = actionStartDetail(node);
    std::string text = "→ 动作开始执行: " + nodeDisplayName(node);
    if (!detail.empty()) {
      text += " (" + detail + ")";
    }
    appendTraceLine(node, "action_start", BT::NodeStatus::RUNNING, text, detail);
  }

  return BT::NodeStatus::IDLE;
}

BT::NodeStatus BtRuntimePublisher::handlePostTick(BT::TreeNode &node,
                                                  BT::NodeStatus status) {
  if (!trace_active_) {
    return status;
  }

  if (node.type() == BT::NodeType::CONDITION) {
    auto detail = conditionDetail(node, status);
    std::string text = "→ 条件判断结果: ";
    text += (status == BT::NodeStatus::SUCCESS) ? "true" : "false";
    if (!detail.empty()) {
      text += " (" + detail + ")";
    }
    appendTraceLine(node, "condition_result", status, text, detail);
  }

  if (const auto summary = decoratorSummary(node, status); !summary.empty()) {
    appendTraceLine(node, "decorator", status, summary);
  }

  if (const auto summary = controlNodeSummary(node, status); !summary.empty()) {
    appendTraceLine(node, "control", status, summary);
  }

  if (node.type() == BT::NodeType::ACTION &&
      status == BT::NodeStatus::IDLE &&
      pre_tick_status_[node.UID()] == BT::NodeStatus::RUNNING) {
    const auto detail = actionFinishDetail(node, status);
    std::string text = "→ 动作执行中断";
    if (!detail.empty()) {
      text += " (" + detail + ")";
    }
    appendTraceLine(node, "action_abort", status, text, detail);
  } else if (node.type() == BT::NodeType::ACTION &&
             status != BT::NodeStatus::RUNNING) {
    const auto detail = actionFinishDetail(node, status);
    std::string text = "→ 动作执行结束: " + statusLabel(status);
    if (!detail.empty()) {
      text += " (" + detail + ")";
    }
    appendTraceLine(node, "action_finish", status, text, detail);
  }

  appendTraceLine(node, "return", status, "← " + statusLabel(status));
  return status;
}

void BtRuntimePublisher::appendTraceLine(const BT::TreeNode &node,
                                         const std::string &kind,
                                         BT::NodeStatus status,
                                         const std::string &text,
                                         const std::string &detail) {
  rc26_interfaces::msg::BehaviorTreeTraceLine line;
  line.stamp = node_->get_clock()->now();
  line.tick_seq = tick_seq_;
  line.line_seq = ++trace_line_seq_;
  line.uid = node.UID();
  line.node_name = node.name();
  line.display_name = nodeDisplayName(node);
  line.registration_name = node.registrationName();
  line.registration_display_name = registrationDisplayName(node);
  line.full_path = node.fullPath();
  line.depth = static_cast<uint16_t>(depthForNode(node.UID()));
  line.kind = kind;
  line.status = toMsgStatus(status);
  line.detail = detail;
  line.formatted_text = formatTraceText(node, text);
  trace_lines_.push_back(std::move(line));
}

std::string BtRuntimePublisher::formatTraceText(const BT::TreeNode &node,
                                                const std::string &text) const {
  const auto stamp = node_->get_clock()->now();
  return "[" + formatTimestamp(stamp) + "] [BT] " +
         std::string(depthForNode(node.UID()) * 2, ' ') + text;
}

std::string BtRuntimePublisher::nodeDisplayName(const BT::TreeNode &node) const {
  if (const auto it = localized_nodes_.find(node.UID());
      it != localized_nodes_.end() && !it->second.display_name.empty()) {
    return it->second.display_name;
  }
  return node.name();
}

std::string
BtRuntimePublisher::registrationDisplayName(const BT::TreeNode &node) const {
  if (const auto it = localized_nodes_.find(node.UID());
      it != localized_nodes_.end() &&
      !it->second.registration_display_name.empty()) {
    return it->second.registration_display_name;
  }
  return nodeTypeLabel(node);
}

std::string BtRuntimePublisher::statusLabel(BT::NodeStatus status) const {
  switch (status) {
  case BT::NodeStatus::IDLE:
    return "空闲";
  case BT::NodeStatus::RUNNING:
    return "运行中";
  case BT::NodeStatus::SUCCESS:
    return "成功";
  case BT::NodeStatus::FAILURE:
    return "失败";
  case BT::NodeStatus::SKIPPED:
    return "跳过";
  default:
    return "未知";
  }
}

std::string BtRuntimePublisher::nodeTypeLabel(const BT::TreeNode &node) const {
  switch (node.type()) {
  case BT::NodeType::ACTION:
    return "动作节点";
  case BT::NodeType::CONDITION:
    return "条件节点";
  case BT::NodeType::CONTROL:
    return "控制节点";
  case BT::NodeType::DECORATOR:
    return "装饰器";
  case BT::NodeType::SUBTREE:
    return "子树";
  default:
    return node.registrationName();
  }
}

std::string BtRuntimePublisher::controlNodeSummary(const BT::TreeNode &node,
                                                   BT::NodeStatus status) const {
  const auto *ctrl = dynamic_cast<const BT::ControlNode *>(&node);
  if (!ctrl) {
    return "";
  }

  const BT::TreeNode *running_child = nullptr;
  const BT::TreeNode *success_child = nullptr;
  const BT::TreeNode *failure_child = nullptr;
  for (const auto *child : ctrl->children()) {
    if (child->status() == BT::NodeStatus::RUNNING && !running_child) {
      running_child = child;
    }
    if (child->status() == BT::NodeStatus::SUCCESS) {
      success_child = child;
    }
    if (child->status() == BT::NodeStatus::FAILURE && !failure_child) {
      failure_child = child;
    }
  }

  const auto reg = node.registrationName();
  if ((reg == "Fallback" || reg == "ReactiveFallback") && running_child) {
    return "→ 选择子节点: " + nodeDisplayName(*running_child);
  }
  if ((reg == "Fallback" || reg == "ReactiveFallback") && success_child) {
    return "→ 选择子节点: " + nodeDisplayName(*success_child) + "（命中）";
  }
  if ((reg == "Fallback" || reg == "ReactiveFallback") &&
      status == BT::NodeStatus::FAILURE) {
    return "← 备选节点全部失败";
  }

  if (reg == "Sequence" && status == BT::NodeStatus::RUNNING) {
    return "← 序列节点提前返回（因运行中）";
  }
  if (reg == "Sequence" && failure_child) {
    return "← 序列节点提前返回（因失败: " + nodeDisplayName(*failure_child) + ")";
  }

  return "";
}

std::string BtRuntimePublisher::decoratorSummary(const BT::TreeNode &node,
                                                 BT::NodeStatus status) const {
  const auto *dec = dynamic_cast<const BT::DecoratorNode *>(&node);
  if (!dec) {
    return "";
  }

  const auto reg = node.registrationName();
  if (reg == "Inverter") {
    return "→ 装饰器执行结果: 取反后" + statusLabel(status);
  }
  if (reg == "ForceFailure") {
    return "→ 装饰器执行结果: 强制失败";
  }
  if (reg == "ForceSuccess") {
    return "→ 装饰器执行结果: 强制成功";
  }
  if (reg == "RetryUntilSuccessful") {
    if (status == BT::NodeStatus::RUNNING) {
      return "→ 装饰器执行结果: 重试中";
    }
    if (status == BT::NodeStatus::SUCCESS) {
      return "→ 装饰器执行结果: 重试成功";
    }
    return "→ 装饰器执行结果: 重试次数耗尽";
  }
  if (reg == "KeepRunningUntilFailure") {
    if (status == BT::NodeStatus::RUNNING) {
      return "→ 装饰器执行结果: 循环继续";
    }
    return "→ 装饰器执行结果: 循环结束";
  }
  if (reg == "Delay") {
    if (status == BT::NodeStatus::RUNNING) {
      return "→ 装饰器执行结果: 延时等待中";
    }
    return "→ 装饰器执行结果: 延时结束";
  }
  if (dec->child() && dec->child()->status() != status &&
      status != BT::NodeStatus::IDLE) {
    return "→ 装饰器执行结果: 状态调整为" + statusLabel(status);
  }
  return "";
}

std::string BtRuntimePublisher::conditionDetail(const BT::TreeNode &node,
                                                BT::NodeStatus status) const {
  if (const auto detail = knownNodeDetail(node, status); !detail.empty()) {
    return detail;
  }

  if (node.registrationName() == "ScriptCondition") {
    const auto raw = stringFromView(node.getRawPortValue("code"));
    if (!raw.empty()) {
      return "表达式: " + raw;
    }
  }

  return collectPortSummary(node);
}

std::string BtRuntimePublisher::actionStartDetail(const BT::TreeNode &node) const {
  if (const auto detail = knownNodeDetail(node, BT::NodeStatus::RUNNING);
      !detail.empty()) {
    return detail;
  }
  return collectPortSummary(node);
}

std::string BtRuntimePublisher::actionFinishDetail(const BT::TreeNode &node,
                                                   BT::NodeStatus status) const {
  if (const auto detail = knownNodeDetail(node, status); !detail.empty()) {
    return detail;
  }
  return collectPortSummary(node);
}

std::string BtRuntimePublisher::collectPortSummary(const BT::TreeNode &node) const {
  const auto &config = node.config();
  std::ostringstream oss;
  bool first = true;
  for (const auto &[port_name, mapped] : config.input_ports) {
    const auto raw = stringFromView(node.getRawPortValue(port_name));
    const auto value = raw.empty() ? mapped : raw;
    if (value.empty()) {
      continue;
    }
    if (!first) {
      oss << ", ";
    }
    first = false;
    oss << port_name << "=" << value;
  }
  return oss.str();
}

std::string BtRuntimePublisher::knownNodeDetail(const BT::TreeNode &node,
                                                BT::NodeStatus /*status*/) const {
  const auto &reg = node.registrationName();
  std::ostringstream oss;

  if (reg == "CheckLocalizationHealth") {
    const auto level = getBlackboardValue<int>(node, "loc_level").value_or(0);
    const auto reason =
        getBlackboardValue<std::string>(node, "loc_reason").value_or("unknown");
    const auto guard_required =
        getBlackboardValue<bool>(node, "loc_guard_required").value_or(false);
    const auto profile = getBlackboardValue<std::string>(node, "loc_recommended_profile")
                             .value_or("normal");
    oss << "loc_level=" << level << ", guard_required="
        << boolLabel(guard_required) << ", reason=" << reason
        << ", profile=" << profile;
    return oss.str();
  }

  if (reg == "CheckExitCondition") {
    const auto current_grid =
        getBlackboardValue<int>(node, "current_grid").value_or(0);
    const auto kfs_count =
        getBlackboardValue<int>(node, "kfs_on_board").value_or(0);
    const auto target_kfs =
        getBlackboardValue<int>(node, "target_kfs_count").value_or(0);
    oss << "current_grid=" << current_grid << ", kfs=" << kfs_count << "/"
        << target_kfs;
    return oss.str();
  }

  if (reg == "CheckLoad") {
    const auto load = getBlackboardValue<int>(node, "kfs_on_board").value_or(0);
    oss << "kfs_on_board=" << load;
    if (const auto ports = collectPortSummary(node); !ports.empty()) {
      oss << ", " << ports;
    }
    return oss.str();
  }

  if (reg == "CheckKFS") {
    if (const auto ports = collectPortSummary(node); !ports.empty()) {
      return ports;
    }
  }

  if (reg == "CheckR1Blocking") {
    const auto current_grid =
        getBlackboardValue<int>(node, "current_grid").value_or(0);
    oss << "current_grid=" << current_grid;
    return oss.str();
  }

  if (reg == "SelectNextGrid") {
    const auto next_action =
        getBlackboardValue<std::string>(node, "next_action").value_or("WAIT");
    const auto target_grid =
        getBlackboardValue<int>(node, "target_grid").value_or(0);
    const auto reason = getBlackboardValue<std::string>(
                            node, "merlin_last_transition_reason")
                            .value_or("");
    oss << "next_action=" << next_action << ", target_grid=" << target_grid;
    if (!reason.empty()) {
      oss << ", reason=" << reason;
    }
    return oss.str();
  }

  if (reg == "WaitVisionTarget") {
    const auto has_target =
        getBlackboardValue<bool>(node, "vision_has_target").value_or(false);
    const auto distance =
        getBlackboardValue<double>(node, "vision_distance_m").value_or(0.0);
    const auto attr =
        getBlackboardValue<int>(node, "vision_attr_kind").value_or(0);
    oss << "vision_has_target=" << boolLabel(has_target)
        << ", distance_m=" << distance << ", attr_kind=" << attr;
    return oss.str();
  }

  if (reg == "LocalizationObserveSpin") {
    return collectPortSummary(node);
  }

  return "";
}

size_t BtRuntimePublisher::depthForNode(uint16_t uid) const {
  if (const auto it = depth_map_.find(uid); it != depth_map_.end()) {
    return it->second;
  }
  return 0;
}

void BtRuntimePublisher::collectRunningPath(const BT::TreeNode *node,
                                            std::vector<uint16_t> &path) {
  if (node->status() != BT::NodeStatus::RUNNING) {
    return;
  }
  path.push_back(node->UID());

  if (const auto *ctrl = dynamic_cast<const BT::ControlNode *>(node)) {
    for (const auto *child : ctrl->children()) {
      if (child->status() == BT::NodeStatus::RUNNING) {
        collectRunningPath(child, path);
        return;
      }
    }
  } else if (const auto *dec = dynamic_cast<const BT::DecoratorNode *>(node)) {
    if (dec->child() && dec->child()->status() == BT::NodeStatus::RUNNING) {
      collectRunningPath(dec->child(), path);
    }
  }
}

uint8_t BtRuntimePublisher::toMsgNodeType(BT::NodeType type) {
  switch (type) {
  case BT::NodeType::ACTION:
    return rc26_interfaces::msg::BehaviorTreeNodeModel::TYPE_ACTION;
  case BT::NodeType::CONDITION:
    return rc26_interfaces::msg::BehaviorTreeNodeModel::TYPE_CONDITION;
  case BT::NodeType::CONTROL:
    return rc26_interfaces::msg::BehaviorTreeNodeModel::TYPE_CONTROL;
  case BT::NodeType::DECORATOR:
    return rc26_interfaces::msg::BehaviorTreeNodeModel::TYPE_DECORATOR;
  case BT::NodeType::SUBTREE:
    return rc26_interfaces::msg::BehaviorTreeNodeModel::TYPE_SUBTREE;
  default:
    return rc26_interfaces::msg::BehaviorTreeNodeModel::TYPE_UNDEFINED;
  }
}

uint8_t BtRuntimePublisher::toMsgStatus(BT::NodeStatus status) {
  switch (status) {
  case BT::NodeStatus::IDLE:
    return 0;
  case BT::NodeStatus::RUNNING:
    return 1;
  case BT::NodeStatus::SUCCESS:
    return 2;
  case BT::NodeStatus::FAILURE:
    return 3;
  case BT::NodeStatus::SKIPPED:
    return 4;
  default:
    return 0;
  }
}

} // namespace rc26_decision
