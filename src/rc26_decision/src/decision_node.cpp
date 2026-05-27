#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <geometry_msgs/msg/point.hpp>
#include <memory>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int8.hpp>
#include <string>
#include <vector>

#include "rc26_decision/bt/bt_runtime_publisher.hpp"
#include "rc26_decision/combat/combat_area.hpp"
#include "rc26_decision/mc/mc_area.hpp"
#include "rc26_decision/mf/keepout_runtime.hpp"
#include "rc26_decision/mf/mf_area.hpp"
#include "rc26_decision/navigation/bt_topo_nav.hpp"
#include "rc26_decision/vision/bt_nodes.hpp"
#include "rc26_interfaces/msg/mechanism_state.hpp"
#include "rc26_interfaces/msg/mf_kfs_cell.hpp"
#include "rc26_interfaces/msg/mf_kfs_state.hpp"
#include "rc26_interfaces/srv/control_behavior_tree.hpp"
#include "rc26_vision/inference/config/model_profile_loader.hpp"
#include "rc26_vision/inference/runtime/vision_inference_manager.hpp"

namespace rc26_decision {

class DecisionNode : public rclcpp::Node {
public:
  DecisionNode() : Node("rc26_decision") {
    // 声明参数
    this->declare_parameter<std::string>("tree_file", "main_tree.xml");
    this->declare_parameter<int>("tick_rate_ms", 100);
    this->declare_parameter<std::string>("tick_mode", "auto");
    this->declare_parameter<int>("manual_play_interval_ms", 100);
    this->declare_parameter<std::string>("bt_control_service",
                                         "r2/bt/control");
    this->declare_parameter<std::string>("mechanism_state_topic",
                                         "/mechanism/status");
    this->declare_parameter<std::string>("team", "blue");
    this->declare_parameter<std::string>("base_ground_level_topic",
                                         "base_ground/level");
    this->declare_parameter<std::string>("base_ground_stair_delta_topic",
                                         "base_ground/stair_delta");
    this->declare_parameter<std::string>("base_ground_stable_topic",
                                         "base_ground/stable_terrain");
    this->declare_parameter<std::string>("base_ground_is_lifted_topic",
                                         "base_ground/is_lifted");
    this->declare_parameter<std::string>("base_ground_stable_operation_topic",
                                         "base_ground/stable_operation");
    this->declare_parameter<std::string>("kfs_state_topic", "mf_kfs_state");
    this->declare_parameter<std::string>("keepout_runtime_service",
                                         "/kfs_keepout/set_runtime");
    this->declare_parameter<double>("tip_rack_center_x", 0.0);
    this->declare_parameter<double>("tip_rack_center_y", 0.0);

    // 创建黑板并共享
    blackboard_ = BT::Blackboard::create();
    const auto &blackboard = blackboard_;
    {
      rclcpp::Node *node_ptr = this;
      blackboard->set("node", node_ptr);
    }
    geometry_msgs::msg::Point rack_center;
    rack_center.x = this->get_parameter("tip_rack_center_x").as_double();
    rack_center.y = this->get_parameter("tip_rack_center_y").as_double();
    rack_center.z = 0.0;
    blackboard->set("tip_rack_center", rack_center);

    // 视觉模块黑板键初始化（即使未启用/未注入 manager 也保持可读）
    blackboard->set("vision_running", false);
    blackboard->set("vision_ok", false);
    blackboard->set("vision_has_target", false);
    blackboard->set("vision_attr_kind", static_cast<int>(0));
    blackboard->set("vision_distance_m", 0.0);
    blackboard->set("vision_score", 0.0);
    blackboard->set("vision_bbox_cx", 0);
    blackboard->set("vision_bbox_cy", 0);

    // 创建 VisionInferenceManager (视觉推理模块)
    this->declare_parameter<bool>("enable_vision", false);
    this->declare_parameter<std::string>("vision_config_file", "");
    blackboard->set("vision_current_model", std::string(""));

    if (this->get_parameter("enable_vision").as_bool()) {
      std::string config_file =
          this->get_parameter("vision_config_file").as_string();
      vision_manager_ =
          std::make_shared<rc26_vision::VisionInferenceManager>(*this);

      if (!config_file.empty()) {
        try {
          auto config = rc26_vision::ProfileLoader::loadFromYaml(config_file);
          vision_manager_->loadConfig(config);
          if (!config.default_model.empty()) {
            vision_manager_->selectModel(config.default_model);
            blackboard->set("vision_current_model", config.default_model);
          }
          blackboard->set("vision_manager", vision_manager_);
          RCLCPP_INFO(this->get_logger(), "视觉配置已加载: %s",
                      config_file.c_str());
        } catch (const std::exception &e) {
          RCLCPP_ERROR(this->get_logger(), "视觉配置加载失败: %s", e.what());
          vision_manager_.reset();
        }
      } else {
        RCLCPP_WARN(this->get_logger(),
                    "enable_vision=true 但 vision_config_file 为空");
        vision_manager_.reset();
      }
    }

    {
      const std::string team = this->get_parameter("team").as_string();
      blackboard->set("team", team);

      auto merlin_map = std::make_shared<MerlinMapManager>();
      const bool layout_loaded = (team == "blue") ? merlin_map->initBlueMap()
                                                  : merlin_map->initRedMap();
      blackboard->set("merlin_map", merlin_map);

      auto merlin_rule_world_model =
          std::make_shared<MerlinRuleWorldModel>(*this);
      blackboard->set("merlin_rule_world_model", merlin_rule_world_model);
      auto battle_grid_state = std::make_shared<BattleGridState>();
      blackboard->set("battle_grid_state", battle_grid_state);
      if (layout_loaded) {
        RCLCPP_INFO(this->get_logger(),
                    "Cold-start merlin_map initialized for team=%s using %s",
                    team.c_str(), merlin_map->layoutStatus().c_str());
      } else {
        RCLCPP_WARN(
            this->get_logger(),
            "Cold-start merlin_map fell back to legacy depths for team=%s: %s",
            team.c_str(), merlin_map->layoutStatus().c_str());
      }
    }

    // 初始化运行状态
    blackboard->set("stair_climb_done", false);
    blackboard->set("stair_descend_done", false);
    blackboard->set("last_action_error_code", 0);
    blackboard->set("system_error", false);
    blackboard->set("current_level", static_cast<int32_t>(0));
    blackboard->set("stair_delta", static_cast<int8_t>(0));
    blackboard->set("base_ground_stable", false);
    blackboard->set("is_lifted", false);
    blackboard->set("stable_operation", false);
    blackboard->set("level_start", static_cast<int32_t>(0));
    blackboard->set("nav_last_exec_state", std::string("IDLE"));
    blackboard->set("nav_last_failure_code", std::string(""));
    blackboard->set("nav_last_failure_reason", std::string(""));
    blackboard->set("nav_last_active_node_id", std::string(""));
    blackboard->set("nav_last_active_edge_id", std::string(""));
    blackboard->set("nav_last_replan_count", static_cast<int>(0));

    // 机制状态可观测键（供 Groot2/诊断查看）
    blackboard->set("mechanism_hal_open", false);
    blackboard->set("mechanism_current_cmd_id", 0);


    // 订阅 base_ground 话题
    const auto level_topic =
        this->get_parameter("base_ground_level_topic").as_string();
    base_ground_level_sub_ = this->create_subscription<std_msgs::msg::Int32>(
        level_topic, 10,
        [blackboard](const std_msgs::msg::Int32::SharedPtr msg) {
          blackboard->set("current_level", msg->data);
        });

    const auto stair_delta_topic =
        this->get_parameter("base_ground_stair_delta_topic").as_string();
    base_ground_stair_delta_sub_ =
        this->create_subscription<std_msgs::msg::Int8>(
            stair_delta_topic, 10,
            [blackboard](const std_msgs::msg::Int8::SharedPtr msg) {
              blackboard->set("stair_delta", static_cast<int8_t>(msg->data));
            });

    const auto stable_topic =
        this->get_parameter("base_ground_stable_topic").as_string();
    base_ground_stable_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        stable_topic, 10,
        [blackboard](const std_msgs::msg::Bool::SharedPtr msg) {
          blackboard->set("base_ground_stable", msg->data);
        });

    const auto is_lifted_topic =
        this->get_parameter("base_ground_is_lifted_topic").as_string();
    base_ground_is_lifted_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        is_lifted_topic, 10,
        [blackboard](const std_msgs::msg::Bool::SharedPtr msg) {
          blackboard->set("is_lifted", msg->data);
        });

    const auto stable_op_topic =
        this->get_parameter("base_ground_stable_operation_topic").as_string();
    base_ground_stable_operation_sub_ =
        this->create_subscription<std_msgs::msg::Bool>(
            stable_op_topic, 10,
            [blackboard](const std_msgs::msg::Bool::SharedPtr msg) {
              blackboard->set("stable_operation", msg->data);
            });

    // 订阅机制状态（决策侧不再直接处理串口反馈）
    const auto mechanism_state_topic =
        this->get_parameter("mechanism_state_topic").as_string();
    mechanism_state_sub_ =
        this->create_subscription<rc26_interfaces::msg::MechanismState>(
            mechanism_state_topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
            [blackboard](
                const rc26_interfaces::msg::MechanismState::SharedPtr msg) {
              blackboard->set("mechanism_hal_open", msg->hal_open);
              blackboard->set("mechanism_current_cmd_id",
                              static_cast<int>(msg->current_cmd_id));
              blackboard->set("last_action_error_code",
                              static_cast<int>(msg->last_error_code));
            });




    // 注册所有行为树节点
    registerMCAreaNodes(factory_);
    registerMFAreaNodes(factory_);
    registerKeepoutRuntimeNodes(factory_);
    registerCombatAreaNodes(factory_);
    registerTopoNavNodes(factory_);
    registerVisionNodes(factory_);

    tick_rate_ms_ = this->get_parameter("tick_rate_ms").as_int();
    tick_mode_ = this->get_parameter("tick_mode").as_string();
    if (tick_mode_ != "auto" && tick_mode_ != "manual") {
      RCLCPP_WARN(this->get_logger(),
                  "未知 tick_mode=%s，回退为 auto",
                  tick_mode_.c_str());
      tick_mode_ = "auto";
    }
    manual_mode_ = (tick_mode_ == "manual");
    play_interval_ms_ = static_cast<uint32_t>(
        std::max<int64_t>(1, this->get_parameter("manual_play_interval_ms").as_int()));

    tree_file_ = this->get_parameter("tree_file").as_string();
    std::string package_path =
        ament_index_cpp::get_package_share_directory("rc26_decision");
    tree_path_ = package_path + "/behavior_trees/" + tree_file_;
    RCLCPP_INFO(this->get_logger(), "加载行为树: %s", tree_path_.c_str());
    rebuildBehaviorTree();
    configureTickMode();

    const auto bt_control_service =
        this->get_parameter("bt_control_service").as_string();
    control_service_ =
        this->create_service<rc26_interfaces::srv::ControlBehaviorTree>(
            bt_control_service,
            [this](
                const std::shared_ptr<rmw_request_id_t> /*request_header*/,
                const std::shared_ptr<
                    rc26_interfaces::srv::ControlBehaviorTree::Request> request,
                std::shared_ptr<
                    rc26_interfaces::srv::ControlBehaviorTree::Response> response) {
              handleControlRequest(*request, *response);
            });

    // /mf_kfs_state 发布器（5Hz）
    const auto kfs_topic = this->get_parameter("kfs_state_topic").as_string();
    pub_kfs_state_ = this->create_publisher<rc26_interfaces::msg::MfKfsState>(
        kfs_topic, rclcpp::QoS(rclcpp::KeepLast(3)).reliable());
    (void)publishKfsState(true);
    kfs_timer_ =
        this->create_wall_timer(std::chrono::milliseconds(200),
                                [this]() { (void)publishKfsState(true); });

    RCLCPP_INFO(this->get_logger(),
                "决策节点已启动, tick_mode=%s, tick_rate_ms=%d, manual_play_interval_ms=%u",
                tick_mode_.c_str(), tick_rate_ms_, play_interval_ms_);
  }

private:
  void publishDebugState() {
    if (!bt_runtime_publisher_) {
      return;
    }
    bt_runtime_publisher_->publishDebugState(manual_mode_, playing_, terminal_,
                                             play_interval_ms_);
  }

  void stopAutoTicking() {
    if (auto_tick_timer_) {
      auto_tick_timer_->cancel();
      auto_tick_timer_.reset();
    }
  }

  void stopManualPlay(bool publish_state = true) {
    if (manual_play_timer_) {
      manual_play_timer_->cancel();
      manual_play_timer_.reset();
    }
    playing_ = false;
    if (publish_state) {
      publishDebugState();
    }
  }

  void resetRuntimeBlackboardState() {
    if (!blackboard_) {
      return;
    }

    blackboard_->set("stair_climb_done", false);
    blackboard_->set("stair_descend_done", false);
    blackboard_->set("system_error", false);
    blackboard_->set("level_start", static_cast<int32_t>(0));
    blackboard_->set("loc_guard_required", false);
    blackboard_->set("loc_guard_reason", std::string(""));
    blackboard_->set("loc_last_profile",
                     this->get_parameter("loc_profile_normal").as_string());
    blackboard_->set("target_kfs_count", 0);
    blackboard_->set("kfs_on_board", 0);
    blackboard_->set("current_grid", 0);
    blackboard_->set("next_action", std::string(""));
    blackboard_->set("target_grid", 0);
    blackboard_->set("exit_grid", 0);
    blackboard_->set("merlin_last_transition_reason", std::string(""));

    const std::string team = this->get_parameter("team").as_string();
    blackboard_->set("team", team);

    auto merlin_map = std::make_shared<MerlinMapManager>();
    const bool layout_loaded = (team == "blue") ? merlin_map->initBlueMap()
                                                : merlin_map->initRedMap();
    blackboard_->set("merlin_map", merlin_map);
    blackboard_->set("merlin_rule_world_model",
                     std::make_shared<MerlinRuleWorldModel>(*this));
    blackboard_->set("battle_grid_state",
                     std::make_shared<BattleGridState>());

    if (layout_loaded) {
      RCLCPP_INFO(this->get_logger(),
                  "Reset merlin_map initialized for team=%s using %s",
                  team.c_str(), merlin_map->layoutStatus().c_str());
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Reset merlin_map fell back to legacy depths for team=%s: %s",
                  team.c_str(), merlin_map->layoutStatus().c_str());
    }
  }

  void rebuildBehaviorTree() {
    stopAutoTicking();
    stopManualPlay(false);
    if (tree_.rootNode()) {
      tree_.haltTree();
    }

    bt_runtime_publisher_.reset();
    tree_ = factory_.createTreeFromFile(tree_path_, blackboard_);
    bt_runtime_publisher_ = std::make_unique<BtRuntimePublisher>(
        this, tree_, blackboard_, tree_file_);
    terminal_ = false;
    playing_ = false;
    have_last_kfs_snapshot_ = false;

    if (bt_runtime_publisher_) {
      bt_runtime_publisher_->publishSnapshotOnly(BT::NodeStatus::IDLE, 0.0f);
    }
    (void)publishKfsState(true);
    publishDebugState();
  }

  void configureTickMode() {
    stopAutoTicking();
    stopManualPlay(false);

    if (!manual_mode_) {
      auto_tick_timer_ = this->create_wall_timer(
          std::chrono::milliseconds(tick_rate_ms_),
          [this]() { (void)tickTree(); });
    }
    publishDebugState();
  }

  void startManualPlay(uint32_t interval_ms) {
    stopManualPlay(false);
    play_interval_ms_ = std::max<uint32_t>(1, interval_ms);
    manual_play_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(play_interval_ms_),
        [this]() { (void)tickTree(); });
    playing_ = true;
    publishDebugState();
  }

  bool tickTree() {
    if (terminal_) {
      return false;
    }

    auto t0 = std::chrono::steady_clock::now();
    if (bt_runtime_publisher_) {
      bt_runtime_publisher_->beginTick();
    }

    BT::NodeStatus status = tree_.tickOnce();
    auto t1 = std::chrono::steady_clock::now();
    float dur_ms = std::chrono::duration<float, std::milli>(t1 - t0).count();

    if (bt_runtime_publisher_) {
      bt_runtime_publisher_->completeTick(status, dur_ms);
    }

    // KFS 状态变化立即发布（周期定时器仍保留 5Hz 保底）
    (void)publishKfsState(false);

    terminal_ =
        (status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::FAILURE);
    if (status == BT::NodeStatus::SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "行为树执行完成: SUCCESS");
    } else if (status == BT::NodeStatus::FAILURE) {
      RCLCPP_ERROR(this->get_logger(), "行为树执行失败: FAILURE");
    }

    if (terminal_) {
      if (manual_mode_) {
        stopManualPlay(false);
      } else {
        stopAutoTicking();
      }
    }

    publishDebugState();
    return true;
  }

  void fillControlResponse(
      rc26_interfaces::srv::ControlBehaviorTree::Response &response,
      bool accepted, const std::string &message) {
    response.accepted = accepted;
    response.message = message;
    response.tick_seq =
        bt_runtime_publisher_ ? bt_runtime_publisher_->currentTickSeq() : 0;
    response.tree_status =
        bt_runtime_publisher_ ? bt_runtime_publisher_->currentTreeStatus() : 0;
    response.playing = playing_;
  }

  void handleControlRequest(
      const rc26_interfaces::srv::ControlBehaviorTree::Request &request,
      rc26_interfaces::srv::ControlBehaviorTree::Response &response) {
    switch (request.command) {
    case rc26_interfaces::srv::ControlBehaviorTree::Request::STEP:
      if (!manual_mode_) {
        fillControlResponse(response, false, "当前非手动单步模式");
        return;
      }
      if (playing_) {
        fillControlResponse(response, false, "当前正在连续执行，请先暂停");
        return;
      }
      if (terminal_) {
        fillControlResponse(response, false, "行为树已结束，请先重置");
        return;
      }
      (void)tickTree();
      fillControlResponse(response, true, "已执行 1 次 Tick");
      return;

    case rc26_interfaces::srv::ControlBehaviorTree::Request::PLAY: {
      if (!manual_mode_) {
        fillControlResponse(response, false, "当前非手动单步模式");
        return;
      }
      if (terminal_) {
        fillControlResponse(response, false, "行为树已结束，请先重置");
        return;
      }
      const uint32_t interval_ms =
          request.play_interval_ms > 0 ? request.play_interval_ms
                                       : play_interval_ms_;
      startManualPlay(interval_ms);
      fillControlResponse(response, true, "已开始连续执行");
      return;
    }

    case rc26_interfaces::srv::ControlBehaviorTree::Request::PAUSE:
      if (!manual_mode_) {
        fillControlResponse(response, false, "当前非手动单步模式");
        return;
      }
      stopManualPlay(true);
      fillControlResponse(response, true, "已暂停连续执行");
      return;

    case rc26_interfaces::srv::ControlBehaviorTree::Request::RESET:
      resetRuntimeBlackboardState();
      rebuildBehaviorTree();
      configureTickMode();
      fillControlResponse(response, true, "行为树已重置");
      return;

    default:
      fillControlResponse(response, false, "未知控制命令");
      return;
    }
  }

  bool publishKfsState(bool force_publish) {
    if (!pub_kfs_state_) {
      return false;
    }

    std::shared_ptr<MerlinMapManager> merlin_map;
    if (!blackboard_->get("merlin_map", merlin_map) || !merlin_map) {
      return false;
    }

    std::string team;
    if (!blackboard_->get("team", team)) {
      team.clear();
    }

    std::array<uint8_t, 13> kfs_type{};
    std::array<float, 13> kfs_confidence{};
    rc26_interfaces::msg::MfKfsState msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "map";
    msg.team = team;

    for (int grid = 1; grid <= 12; grid++) {
      const auto kfs = merlin_map->getKFS(grid);
      rc26_interfaces::msg::MfKfsCell cell;
      cell.grid_id = static_cast<uint8_t>(grid);
      cell.kfs_type = static_cast<uint8_t>(kfs);
      cell.confidence = (kfs == KFSType::UNKNOWN) ? 0.0f : 1.0f;
      kfs_type[static_cast<size_t>(grid)] = cell.kfs_type;
      kfs_confidence[static_cast<size_t>(grid)] = cell.confidence;
      msg.cells.push_back(cell);
    }

    bool changed = !have_last_kfs_snapshot_ || (team != last_kfs_team_);
    for (int grid = 1; grid <= 12 && !changed; grid++) {
      const size_t idx = static_cast<size_t>(grid);
      if (kfs_type[idx] != last_kfs_type_[idx] ||
          std::fabs(kfs_confidence[idx] - last_kfs_confidence_[idx]) > 1e-5F) {
        changed = true;
      }
    }

    if (!force_publish && !changed) {
      return false;
    }

    pub_kfs_state_->publish(msg);
    last_kfs_type_ = kfs_type;
    last_kfs_confidence_ = kfs_confidence;
    last_kfs_team_ = team;
    have_last_kfs_snapshot_ = true;
    return true;
  }

  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
  BT::Blackboard::Ptr blackboard_;
  rclcpp::TimerBase::SharedPtr auto_tick_timer_;
  rclcpp::TimerBase::SharedPtr manual_play_timer_;
  rclcpp::Service<rc26_interfaces::srv::ControlBehaviorTree>::SharedPtr
      control_service_;
  rclcpp::Publisher<rc26_interfaces::msg::MfKfsState>::SharedPtr pub_kfs_state_;
  rclcpp::TimerBase::SharedPtr kfs_timer_;
  std::array<uint8_t, 13> last_kfs_type_{};
  std::array<float, 13> last_kfs_confidence_{};
  std::string last_kfs_team_;
  bool have_last_kfs_snapshot_{false};
  std::string tree_file_;
  std::string tree_path_;
  std::string tick_mode_{"auto"};
  int tick_rate_ms_{100};
  uint32_t play_interval_ms_{100};
  bool manual_mode_{false};
  bool playing_{false};
  bool terminal_{false};
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr base_ground_level_sub_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr
      base_ground_stair_delta_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr base_ground_stable_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
      base_ground_is_lifted_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
      base_ground_stable_operation_sub_;
  rclcpp::Subscription<rc26_interfaces::msg::MechanismState>::SharedPtr
      mechanism_state_sub_;
  std::shared_ptr<rc26_vision::VisionInferenceManager> vision_manager_;
  std::unique_ptr<BtRuntimePublisher> bt_runtime_publisher_;
};

} // namespace rc26_decision

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rc26_decision::DecisionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
