#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>

#include "rc26_decision/mc/mc_area.hpp"
#include "rc26_decision/kfs/kfs_stair_pickup.hpp"
#include "rc26_decision/mf/mf_area.hpp"
#include "rc26_decision/mf_preselection/mf_preselection_flow.hpp"
#include "rc26_decision/navigation/bt_nav2_pose.hpp"
#include "rc26_decision/stair/stair_area.hpp"

namespace rc26_decision {

class DecisionNode : public rclcpp::Node {
public:
  DecisionNode() : Node("rc26_decision") {
    declareParameters();
    initializeBlackboard();
    initializeMerlinState();
    initializeRuntimeState();

    loadMCParams(*this, blackboard_);
    loadKfsParams(*this, blackboard_);
    loadGridHeadingParams(*this, blackboard_);
    loadGridCenterParams(*this, blackboard_);
    loadStairParams(*this, blackboard_);
    loadMfPreselectionParams(*this, blackboard_);

    registerMCAreaNodes(factory_);
    registerKfsNodes(factory_);
    registerMFAreaNodes(factory_);
    registerMfPreselectionNodes(factory_);
    registerNav2PoseNodes(factory_);
    registerStairNodes(factory_);

    tick_rate_ms_ = this->get_parameter("tick_rate_ms").as_int();
    tree_file_ = this->get_parameter("tree_file").as_string();
    const std::filesystem::path configured_tree(tree_file_);
    tree_path_ = configured_tree.is_absolute()
                     ? configured_tree.lexically_normal().string()
                     : (std::filesystem::path(
                            ament_index_cpp::get_package_share_directory("rc26_decision")) /
                        "behavior_trees" / configured_tree)
                           .lexically_normal()
                           .string();

    RCLCPP_INFO(this->get_logger(), "加载行为树: %s", tree_path_.c_str());
    loadBehaviorTree();
    startAutoTicking();

    RCLCPP_INFO(this->get_logger(), "决策节点已启动, tick_rate_ms=%d",
                tick_rate_ms_);
  }

private:
  void declareParameters() {
    this->declare_parameter<std::string>("tree_file", "main_tree.xml");
    this->declare_parameter<int>("tick_rate_ms", 100);
    this->declare_parameter<std::string>("team", "blue");
  }

  void initializeBlackboard() {
    blackboard_ = BT::Blackboard::create();
    rclcpp::Node *node_ptr = this;
    blackboard_->set("node", node_ptr);
  }

  void initializeMerlinState() {
    const std::string team = this->get_parameter("team").as_string();
    blackboard_->set("team", team);

    auto merlin_map = std::make_shared<MerlinMapManager>();
    const bool layout_loaded =
        (team == "blue") ? merlin_map->initBlueMap() : merlin_map->initRedMap();
    blackboard_->set("merlin_map", merlin_map);

    if (layout_loaded) {
      RCLCPP_INFO(this->get_logger(),
                  "Cold-start merlin_map initialized for team=%s using %s",
                  team.c_str(), merlin_map->layoutStatus().c_str());
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Cold-start merlin_map fell back to legacy depths for "
                  "team=%s: %s",
                  team.c_str(), merlin_map->layoutStatus().c_str());
    }
  }

  void initializeRuntimeState() {
    blackboard_->set("stair_climb_done", false);
    blackboard_->set("stair_descend_done", false);
    blackboard_->set("last_action_error_code", 0);
    blackboard_->set("system_error", false);
    blackboard_->set("stable_operation", false);
    blackboard_->set("nav_last_exec_state", std::string("IDLE"));
    blackboard_->set("nav_last_failure_code", std::string(""));
    blackboard_->set("nav_last_failure_reason", std::string(""));
    blackboard_->set("nav_last_distance_remaining", 0.0);
    blackboard_->set("nav_last_recovery_count", static_cast<int>(0));
  }

  void loadBehaviorTree() {
    stopAutoTicking();
    if (tree_.rootNode()) {
      tree_.haltTree();
    }

    tree_ = factory_.createTreeFromFile(tree_path_, blackboard_);
    terminal_ = false;
  }

  void startAutoTicking() {
    stopAutoTicking();
    auto_tick_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(tick_rate_ms_),
        [this]() { (void)tickTree(); });
  }

  void stopAutoTicking() {
    if (auto_tick_timer_) {
      auto_tick_timer_->cancel();
      auto_tick_timer_.reset();
    }
  }

  bool tickTree() {
    if (terminal_) {
      return false;
    }

    const BT::NodeStatus status = tree_.tickOnce();
    terminal_ =
        (status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::FAILURE);

    if (status == BT::NodeStatus::SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "行为树执行完成: SUCCESS");
    } else if (status == BT::NodeStatus::FAILURE) {
      RCLCPP_ERROR(this->get_logger(), "行为树执行失败: FAILURE");
    }

    if (terminal_) {
      stopAutoTicking();
    }
    return true;
  }

  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
  BT::Blackboard::Ptr blackboard_;
  rclcpp::TimerBase::SharedPtr auto_tick_timer_;
  std::string tree_file_;
  std::string tree_path_;
  int tick_rate_ms_{100};
  bool terminal_{false};
};

} // namespace rc26_decision

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rc26_decision::DecisionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
