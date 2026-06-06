#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cstdint>
#include <geometry_msgs/msg/point.hpp>
#include <memory>
#include <string>

#include "rc26_decision/combat/combat_area.hpp"
#include "rc26_decision/mc/mc_area.hpp"
#include "rc26_decision/mf/mf_area.hpp"
#include "rc26_decision/navigation/bt_nav2_pose.hpp"
#include "rc26_decision/vision/bt_nodes.hpp"
#include "rc26_interfaces/msg/mechanism_state.hpp"
#include "rc26_vision/inference/config/model_profile_loader.hpp"
#include "rc26_vision/inference/runtime/vision_inference_manager.hpp"

namespace rc26_decision {

class DecisionNode : public rclcpp::Node {
public:
  DecisionNode() : Node("rc26_decision") {
    declareParameters();
    initializeBlackboard();
    initializeVision();
    initializeMerlinState();
    initializeRuntimeState();
    subscribeMechanismState();

    registerMCAreaNodes(factory_);
    registerMFAreaNodes(factory_);
    registerCombatAreaNodes(factory_);
    registerNav2PoseNodes(factory_);
    registerVisionNodes(factory_);

    tick_rate_ms_ = this->get_parameter("tick_rate_ms").as_int();
    tree_file_ = this->get_parameter("tree_file").as_string();
    tree_path_ =
        ament_index_cpp::get_package_share_directory("rc26_decision") +
        "/behavior_trees/" + tree_file_;

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
    this->declare_parameter<std::string>("mechanism_state_topic",
                                         "/mechanism/status");
    this->declare_parameter<std::string>("team", "blue");
    this->declare_parameter<double>("tip_rack_center_x", 0.0);
    this->declare_parameter<double>("tip_rack_center_y", 0.0);
    this->declare_parameter<bool>("enable_vision", false);
    this->declare_parameter<std::string>("vision_config_file", "");
  }

  void initializeBlackboard() {
    blackboard_ = BT::Blackboard::create();
    rclcpp::Node *node_ptr = this;
    blackboard_->set("node", node_ptr);

    geometry_msgs::msg::Point rack_center;
    rack_center.x = this->get_parameter("tip_rack_center_x").as_double();
    rack_center.y = this->get_parameter("tip_rack_center_y").as_double();
    rack_center.z = 0.0;
    blackboard_->set("tip_rack_center", rack_center);
  }

  void initializeVision() {
    blackboard_->set("vision_running", false);
    blackboard_->set("vision_ok", false);
    blackboard_->set("vision_has_target", false);
    blackboard_->set("vision_attr_kind", static_cast<int>(0));
    blackboard_->set("vision_distance_m", 0.0);
    blackboard_->set("vision_score", 0.0);
    blackboard_->set("vision_bbox_cx", 0);
    blackboard_->set("vision_bbox_cy", 0);
    blackboard_->set("vision_current_model", std::string(""));

    if (!this->get_parameter("enable_vision").as_bool()) {
      return;
    }

    const std::string config_file =
        this->get_parameter("vision_config_file").as_string();
    if (config_file.empty()) {
      RCLCPP_WARN(this->get_logger(),
                  "enable_vision=true 但 vision_config_file 为空");
      return;
    }

    vision_manager_ = std::make_shared<rc26_vision::VisionInferenceManager>(
        *this);

    try {
      const auto config = rc26_vision::ProfileLoader::loadFromYaml(config_file);
      vision_manager_->loadConfig(config);
      if (!config.default_model.empty()) {
        vision_manager_->selectModel(config.default_model);
        blackboard_->set("vision_current_model", config.default_model);
      }
      blackboard_->set("vision_manager", vision_manager_);
      RCLCPP_INFO(this->get_logger(), "视觉配置已加载: %s",
                  config_file.c_str());
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "视觉配置加载失败: %s", e.what());
      vision_manager_.reset();
    }
  }

  void initializeMerlinState() {
    const std::string team = this->get_parameter("team").as_string();
    blackboard_->set("team", team);

    auto merlin_map = std::make_shared<MerlinMapManager>();
    const bool layout_loaded =
        (team == "blue") ? merlin_map->initBlueMap() : merlin_map->initRedMap();
    blackboard_->set("merlin_map", merlin_map);
    blackboard_->set("battle_grid_state",
                     std::make_shared<BattleGridState>());

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
    blackboard_->set("current_level", static_cast<int32_t>(0));
    blackboard_->set("stair_delta", static_cast<int8_t>(0));
    blackboard_->set("is_lifted", false);
    blackboard_->set("stable_operation", false);
    blackboard_->set("level_start", static_cast<int32_t>(0));
    blackboard_->set("nav_last_exec_state", std::string("IDLE"));
    blackboard_->set("nav_last_failure_code", std::string(""));
    blackboard_->set("nav_last_failure_reason", std::string(""));
    blackboard_->set("nav_last_distance_remaining", 0.0);
    blackboard_->set("nav_last_recovery_count", static_cast<int>(0));
    blackboard_->set("mechanism_hal_open", false);
    blackboard_->set("mechanism_current_cmd_id", 0);
  }

  void subscribeMechanismState() {
    const auto mechanism_state_topic =
        this->get_parameter("mechanism_state_topic").as_string();
    mechanism_state_sub_ =
        this->create_subscription<rc26_interfaces::msg::MechanismState>(
            mechanism_state_topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
            [blackboard = blackboard_](
                const rc26_interfaces::msg::MechanismState::SharedPtr msg) {
              blackboard->set("mechanism_hal_open", msg->hal_open);
              blackboard->set("mechanism_current_cmd_id",
                              static_cast<int>(msg->current_cmd_id));
              blackboard->set("last_action_error_code",
                              static_cast<int>(msg->last_error_code));
            });
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
  rclcpp::Subscription<rc26_interfaces::msg::MechanismState>::SharedPtr
      mechanism_state_sub_;
  std::shared_ptr<rc26_vision::VisionInferenceManager> vision_manager_;
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
