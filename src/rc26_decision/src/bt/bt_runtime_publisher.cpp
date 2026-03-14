#include "rc26_decision/bt/bt_runtime_publisher.hpp"

#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/control_node.h>
#include <behaviortree_cpp/decorator_node.h>

#include <algorithm>

#include "rc26_interfaces/msg/behavior_tree_event.hpp"
#include "rc26_interfaces/msg/behavior_tree_node_model.hpp"
#include "rc26_interfaces/msg/behavior_tree_node_state.hpp"
#include "rc26_interfaces/msg/behavior_tree_port.hpp"

namespace rc26_decision {

BtRuntimePublisher::BtRuntimePublisher(rclcpp::Node *node, BT::Tree &tree,
                                       BT::Blackboard::Ptr blackboard,
                                       const std::string &tree_file)
    : node_(node), tree_(tree), blackboard_(std::move(blackboard)),
      tree_file_(tree_file) {

  // read params
  node_->declare_parameter<bool>("bt_runtime.enable", true);
  node_->declare_parameter<std::string>("bt_runtime.model_topic",
                                        "r2/bt/model");
  node_->declare_parameter<std::string>("bt_runtime.snapshot_topic",
                                        "r2/bt/snapshot");
  node_->declare_parameter<std::string>("bt_runtime.blackboard_topic",
                                        "r2/bt/blackboard");
  node_->declare_parameter<std::string>("bt_runtime.events_topic",
                                        "r2/bt/events");
  node_->declare_parameter<int>("bt_runtime.snapshot_decimation", 1);
  node_->declare_parameter<int>("bt_runtime.blackboard_publish_ms", 200);
  node_->declare_parameter<std::vector<std::string>>(
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

  if (!node_->get_parameter("bt_runtime.enable").as_bool()) {
    return;
  }

  bb_whitelist_ =
      node_->get_parameter("bt_runtime.blackboard_whitelist").as_string_array();
  snapshot_decimation_ = static_cast<uint32_t>(std::max<int64_t>(
      1, node_->get_parameter("bt_runtime.snapshot_decimation").as_int()));

  // publishers
  auto model_topic =
      node_->get_parameter("bt_runtime.model_topic").as_string();
  auto snapshot_topic =
      node_->get_parameter("bt_runtime.snapshot_topic").as_string();
  auto bb_topic =
      node_->get_parameter("bt_runtime.blackboard_topic").as_string();
  auto events_topic =
      node_->get_parameter("bt_runtime.events_topic").as_string();

  rclcpp::QoS model_qos(rclcpp::KeepLast(1));
  model_qos.reliable().transient_local();
  pub_model_ =
      node_->create_publisher<rc26_interfaces::msg::BehaviorTreeModel>(
          model_topic, model_qos);

  rclcpp::QoS volatile_qos(rclcpp::KeepLast(5));
  volatile_qos.reliable();
  pub_snapshot_ =
      node_->create_publisher<rc26_interfaces::msg::BehaviorTreeSnapshot>(
          snapshot_topic, volatile_qos);
  pub_blackboard_ =
      node_->create_publisher<rc26_interfaces::msg::BehaviorTreeBlackboard>(
          bb_topic, volatile_qos);

  rclcpp::QoS events_qos(rclcpp::KeepLast(100));
  events_qos.reliable();
  pub_events_ =
      node_->create_publisher<rc26_interfaces::msg::BehaviorTreeEventArray>(
          events_topic, events_qos);

  // cache all nodes and subscribe to status changes
  tree_.applyVisitor([this](const BT::TreeNode *n) {
    all_nodes_.push_back(n);

    auto sub = const_cast<BT::TreeNode *>(n)->subscribeToStatusChange(
        [this, n](BT::TimePoint /*tp*/, const BT::TreeNode & /*node*/,
                  BT::NodeStatus prev, BT::NodeStatus cur) {
          rc26_interfaces::msg::BehaviorTreeEvent ev;
          ev.stamp = node_->get_clock()->now();
          ev.uid = n->UID();
          ev.node_name = n->name();
          ev.full_path = n->fullPath();
          ev.prev_status = toMsgStatus(prev);
          ev.status = toMsgStatus(cur);
          std::lock_guard<std::mutex> lk(event_mutex_);
          event_buffer_.push_back(std::move(ev));
        });
    subscribers_.push_back(std::move(sub));
  });

  // blackboard timer
  int bb_ms =
      node_->get_parameter("bt_runtime.blackboard_publish_ms").as_int();
  bb_timer_ = node_->create_wall_timer(std::chrono::milliseconds(bb_ms),
                                       [this]() { publishBlackboard(); });

  buildAndPublishModel();
}

void BtRuntimePublisher::onTick(BT::NodeStatus tree_status,
                                float tick_duration_ms) {
  if (!pub_snapshot_) {
    return;
  }
  ++tick_seq_;
  const bool publish_snapshot =
      tick_seq_ == 1 ||
      ((tick_seq_ - 1) % snapshot_decimation_) == 0 ||
      tree_status != BT::NodeStatus::RUNNING;
  if (publish_snapshot) {
    publishSnapshot(tree_status, tick_duration_ms);
  }
  flushEvents();
}

// ─── model ──────────────────────────────────────────────────────────────

void BtRuntimePublisher::buildAndPublishModel() {
  rc26_interfaces::msg::BehaviorTreeModel model;
  model.header.stamp = node_->get_clock()->now();
  model.tree_file = tree_file_;

  // subtree IDs
  for (auto &st : tree_.subtrees) {
    model.subtree_ids.push_back(st->tree_ID);
  }
  if (!tree_.subtrees.empty()) {
    model.main_tree_id = tree_.subtrees.front()->tree_ID;
  }

  // root uid
  if (tree_.rootNode()) {
    model.root_uid = tree_.rootNode()->UID();
  }

  // build uid -> parent map via tree traversal
  std::unordered_map<uint16_t, uint16_t> parent_map;
  std::unordered_map<uint16_t, std::vector<uint16_t>> children_map;
  std::unordered_map<uint16_t, std::string> subtree_map;

  for (auto &st : tree_.subtrees) {
    for (auto &n : st->nodes) {
      subtree_map[n->UID()] = st->tree_ID;

      if (auto *ctrl = dynamic_cast<const BT::ControlNode *>(n.get())) {
        for (auto *child : ctrl->children()) {
          children_map[n->UID()].push_back(child->UID());
          parent_map[child->UID()] = n->UID();
        }
      } else if (auto *dec =
                     dynamic_cast<const BT::DecoratorNode *>(n.get())) {
        if (dec->child()) {
          children_map[n->UID()].push_back(dec->child()->UID());
          parent_map[dec->child()->UID()] = n->UID();
        }
      }
    }
  }

  // build node models
  for (auto *n : all_nodes_) {
    rc26_interfaces::msg::BehaviorTreeNodeModel nm;
    nm.uid = n->UID();
    nm.instance_name = n->name();
    nm.registration_name = n->registrationName();
    nm.node_type = toMsgNodeType(n->type());
    nm.full_path = n->fullPath();

    auto sit = subtree_map.find(n->UID());
    if (sit != subtree_map.end()) {
      nm.subtree_id = sit->second;
    }

    auto pit = parent_map.find(n->UID());
    nm.parent_uid = (pit != parent_map.end()) ? pit->second : 0;

    auto cit = children_map.find(n->UID());
    if (cit != children_map.end()) {
      nm.children_uids = cit->second;
    }

    // ports
    const auto &cfg = n->config();
    const auto *manifest = cfg.manifest;
    // input ports
    for (auto &[port_name, remapped] : cfg.input_ports) {
      rc26_interfaces::msg::BehaviorTreePort p;
      p.name = port_name;
      p.remapped_key = remapped;
      p.direction = rc26_interfaces::msg::BehaviorTreePort::INPUT;
      if (manifest) {
        auto it = manifest->ports.find(port_name);
        if (it != manifest->ports.end()) {
          p.declared_type = BT::demangle(it->second.type());
          if (it->second.direction() == BT::PortDirection::INOUT) {
            p.direction = rc26_interfaces::msg::BehaviorTreePort::INOUT;
          }
        }
      }
      nm.ports.push_back(std::move(p));
    }
    // output ports (skip those already added as INOUT)
    for (auto &[port_name, remapped] : cfg.output_ports) {
      if (cfg.input_ports.count(port_name)) {
        continue;
      }
      rc26_interfaces::msg::BehaviorTreePort p;
      p.name = port_name;
      p.remapped_key = remapped;
      p.direction = rc26_interfaces::msg::BehaviorTreePort::OUTPUT;
      if (manifest) {
        auto it = manifest->ports.find(port_name);
        if (it != manifest->ports.end()) {
          p.declared_type = BT::demangle(it->second.type());
        }
      }
      nm.ports.push_back(std::move(p));
    }

    model.nodes.push_back(std::move(nm));
  }

  pub_model_->publish(model);
  RCLCPP_INFO(node_->get_logger(),
              "BT model published: %zu nodes, %zu subtrees",
              model.nodes.size(), model.subtree_ids.size());
}

// ─── snapshot ───────────────────────────────────────────────────────────

void BtRuntimePublisher::publishSnapshot(BT::NodeStatus tree_status,
                                         float tick_duration_ms) {
  rc26_interfaces::msg::BehaviorTreeSnapshot snap;
  snap.header.stamp = node_->get_clock()->now();
  snap.tick_seq = tick_seq_;
  snap.tree_status = toMsgStatus(tree_status);
  snap.tick_duration_ms = tick_duration_ms;

  // node states
  for (auto *n : all_nodes_) {
    rc26_interfaces::msg::BehaviorTreeNodeState ns;
    ns.uid = n->UID();
    ns.status = toMsgStatus(n->status());
    snap.nodes.push_back(ns);
  }

  // running path
  if (tree_.rootNode()) {
    collectRunningPath(tree_.rootNode(), snap.running_path_uids);
  }

  // active subtree: find deepest running node's subtree
  snap.active_subtree_id =
      tree_.subtrees.empty() ? "" : tree_.subtrees.front()->tree_ID;
  if (!snap.running_path_uids.empty()) {
    uint16_t deepest = snap.running_path_uids.back();
    for (auto &st : tree_.subtrees) {
      for (auto &n : st->nodes) {
        if (n->UID() == deepest) {
          snap.active_subtree_id = st->tree_ID;
          break;
        }
      }
    }
  }

  pub_snapshot_->publish(snap);
}

void BtRuntimePublisher::collectRunningPath(const BT::TreeNode *node,
                                            std::vector<uint16_t> &path) {
  if (node->status() != BT::NodeStatus::RUNNING) {
    return;
  }
  path.push_back(node->UID());

  if (auto *ctrl = dynamic_cast<const BT::ControlNode *>(node)) {
    for (auto *child : ctrl->children()) {
      if (child->status() == BT::NodeStatus::RUNNING) {
        collectRunningPath(child, path);
        return;
      }
    }
  } else if (auto *dec = dynamic_cast<const BT::DecoratorNode *>(node)) {
    if (dec->child() && dec->child()->status() == BT::NodeStatus::RUNNING) {
      collectRunningPath(dec->child(), path);
    }
  }
}

// ─── blackboard ─────────────────────────────────────────────────────────

void BtRuntimePublisher::publishBlackboard() {
  if (!pub_blackboard_) {
    return;
  }
  rc26_interfaces::msg::BehaviorTreeBlackboard msg;
  msg.header.stamp = node_->get_clock()->now();

  for (auto &key : bb_whitelist_) {
    auto entry = blackboard_->getEntry(key);
    if (!entry) {
      continue;
    }

    rc26_interfaces::msg::BehaviorTreeBlackboardEntry e;
    e.key = key;

    try {
      std::lock_guard<std::mutex> lk(entry->entry_mutex);
      if (entry->value.empty()) {
        continue;
      }
      e.type_name = BT::demangle(entry->value.type());
      if (entry->value.isType<bool>()) {
        e.value = entry->value.cast<bool>() ? "true" : "false";
      } else {
        e.value = entry->value.cast<std::string>();
      }
    } catch (...) {
      continue;
    }
    msg.entries.push_back(std::move(e));
  }

  pub_blackboard_->publish(msg);
}

// ─── events ─────────────────────────────────────────────────────────────

void BtRuntimePublisher::flushEvents() {
  std::vector<rc26_interfaces::msg::BehaviorTreeEvent> events;
  {
    std::lock_guard<std::mutex> lk(event_mutex_);
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

// ─── helpers ────────────────────────────────────────────────────────────

uint8_t BtRuntimePublisher::toMsgNodeType(BT::NodeType t) {
  switch (t) {
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

uint8_t BtRuntimePublisher::toMsgStatus(BT::NodeStatus s) {
  switch (s) {
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
