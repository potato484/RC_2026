#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

#include <array>
#include <cmath>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int8.hpp>
#include <vector>

#include "rc26_decision/combat/combat_area.hpp"
#include "rc26_decision/mc/mc_area.hpp"
#include "rc26_decision/mf/mf_area.hpp"
#include "rc26_decision/navigation/bt_nav_to_smart_point.hpp"
#include "rc26_decision/navigation/smart_waypoint_navigator.hpp"
#include "rc26_decision/navigation/waypoint_manager.hpp"
#include "rc26_decision/vision/bt_nodes.hpp"
#include "rc26_interfaces/msg/localization_backend_status.hpp"
#include "rc26_interfaces/msg/localization_health.hpp"
#include "rc26_interfaces/msg/mechanism_state.hpp"
#include "rc26_interfaces/msg/mf_kfs_cell.hpp"
#include "rc26_interfaces/msg/mf_kfs_state.hpp"
#include "rc26_interfaces/msg/route_observability.hpp"
#include "rc26_vision/profile_loader.hpp"
#include "rc26_vision/vision_inference_manager.hpp"

namespace rc26_decision {

class DecisionNode : public rclcpp::Node {
public:
  DecisionNode() : Node("rc26_decision") {
    // 声明参数
    this->declare_parameter<std::string>("tree_file", "main_tree.xml");
    this->declare_parameter<int>("tick_rate_ms", 100);
    this->declare_parameter<std::string>("mechanism_state_topic",
                                         "/mechanism/state");
    this->declare_parameter<std::string>("nav2_action_name",
                                         "navigate_to_pose");
    this->declare_parameter<std::string>("nav2_goal_frame", "map");
    this->declare_parameter<std::string>("controller_server_node",
                                         "controller_server");
    this->declare_parameter<std::string>("odom_topic", "odom");
    this->declare_parameter<std::string>("team", "blue");
    this->declare_parameter<std::string>("waypoints_file", "");
    this->declare_parameter<double>("stop_linear_eps_mps", 0.05);
    this->declare_parameter<double>("stop_angular_eps_rps", 0.1);
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
    this->declare_parameter<double>("tip_rack_center_x", 0.0);
    this->declare_parameter<double>("tip_rack_center_y", 0.0);
    this->declare_parameter<std::string>("localization_health_topic",
                                         "/localization/health");
    this->declare_parameter<std::string>("localization_backend_status_topic",
                                         "/localization/backend_status");
    this->declare_parameter<std::string>(
        "localization_route_observability_topic",
        "/localization/route_observability");
    this->declare_parameter<std::string>("loc_profile_normal", "normal");
    this->declare_parameter<std::string>("loc_profile_yellow", "loc_yellow");
    this->declare_parameter<std::string>("loc_profile_orange", "loc_orange");
    this->declare_parameter<std::string>("loc_profile_red", "loc_red_hold");
    this->declare_parameter<std::string>("loc_spin_action_name", "spin");
    this->declare_parameter<double>("loc_spin_angle_deg", 15.0);
    this->declare_parameter<double>("loc_spin_time_allowance_sec", 3.0);
    this->declare_parameter<std::vector<std::string>>(
        "loc_retry_waypoints", std::vector<std::string>{"loc_retry_zone_1"});
    this->declare_parameter<std::vector<std::string>>(
        "loc_anchor_waypoints",
        std::vector<std::string>{"loc_anchor_1", "loc_anchor_2"});

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

    // 创建 WaypointManager 并加载配置
    {
      std::string waypoints_file =
          this->get_parameter("waypoints_file").as_string();
      if (waypoints_file.empty()) {
        std::string team = this->get_parameter("team").as_string();
        std::string package_path =
            ament_index_cpp::get_package_share_directory("rc26_decision");
        waypoints_file =
            package_path + "/config/waypoints/waypoints_" + team + ".yaml";
      }
      waypoint_manager_ = std::make_shared<WaypointManager>();
      if (!waypoint_manager_->loadFromYamlFile(waypoints_file)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load waypoints: %s",
                     waypoints_file.c_str());
      } else {
        RCLCPP_INFO(this->get_logger(), "Loaded waypoints from: %s",
                    waypoints_file.c_str());
      }
      blackboard->set("waypoint_manager", waypoint_manager_);

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

    // 创建 SmartWaypointNavigator 并共享到黑板
    {
      const auto nav2_action_name =
          this->get_parameter("nav2_action_name").as_string();
      const auto nav2_goal_frame =
          this->get_parameter("nav2_goal_frame").as_string();
      const auto controller_node =
          this->get_parameter("controller_server_node").as_string();
      const auto odom_topic = this->get_parameter("odom_topic").as_string();
      const double stop_lin =
          this->get_parameter("stop_linear_eps_mps").as_double();
      const double stop_ang =
          this->get_parameter("stop_angular_eps_rps").as_double();
      smart_waypoint_navigator_ = std::make_shared<SmartWaypointNavigator>(
          *this, nav2_action_name, nav2_goal_frame, controller_node, odom_topic,
          stop_lin, stop_ang);
      blackboard->set("smart_waypoint_navigator", smart_waypoint_navigator_);
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

    // 机制状态可观测键（供 Groot2/诊断查看）
    blackboard->set("mechanism_tip_state", 0);
    blackboard->set("mechanism_hal_open", false);
    blackboard->set("mechanism_locked_tip_slot", 255);
    blackboard->set("mechanism_comm_health_level", 0);

    // 定位 guard 黑板键初始化
    blackboard->set("loc_level", 0);
    blackboard->set("loc_reason", std::string("startup"));
    blackboard->set("loc_control_degraded", false);
    blackboard->set("loc_graph_health", 0.0);
    blackboard->set("loc_optimizer_ready", false);
    blackboard->set("loc_route_score", 1.0);
    blackboard->set("loc_route_risk_level", 0);
    blackboard->set("loc_recommended_profile",
                    this->get_parameter("loc_profile_normal").as_string());
    blackboard->set("loc_last_profile",
                    this->get_parameter("loc_profile_normal").as_string());
    blackboard->set("loc_profile_normal",
                    this->get_parameter("loc_profile_normal").as_string());
    blackboard->set("loc_profile_yellow",
                    this->get_parameter("loc_profile_yellow").as_string());
    blackboard->set("loc_profile_orange",
                    this->get_parameter("loc_profile_orange").as_string());
    blackboard->set("loc_profile_red",
                    this->get_parameter("loc_profile_red").as_string());
    blackboard->set("loc_spin_action_name",
                    this->get_parameter("loc_spin_action_name").as_string());
    blackboard->set("loc_spin_angle_deg",
                    this->get_parameter("loc_spin_angle_deg").as_double());
    blackboard->set(
        "loc_spin_time_allowance_sec",
        this->get_parameter("loc_spin_time_allowance_sec").as_double());
    blackboard->set(
        "loc_retry_waypoints",
        this->get_parameter("loc_retry_waypoints").as_string_array());
    blackboard->set(
        "loc_anchor_waypoints",
        this->get_parameter("loc_anchor_waypoints").as_string_array());

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
              blackboard->set("mechanism_tip_state",
                              static_cast<int>(msg->tip_state));
              blackboard->set("mechanism_hal_open", msg->hal_open);
              blackboard->set("mechanism_locked_tip_slot",
                              static_cast<int>(msg->locked_tip_slot));
              blackboard->set("last_action_error_code",
                              static_cast<int>(msg->last_error_code));
              blackboard->set("mechanism_comm_health_level",
                              static_cast<int>(msg->comm_health_level));
            });

    const auto loc_health_topic =
        this->get_parameter("localization_health_topic").as_string();
    localization_health_sub_ =
        this->create_subscription<rc26_interfaces::msg::LocalizationHealth>(
            loc_health_topic, rclcpp::SensorDataQoS(),
            [blackboard](
                const rc26_interfaces::msg::LocalizationHealth::SharedPtr msg) {
              blackboard->set("loc_level", static_cast<int>(msg->level));
              blackboard->set("loc_reason", msg->reason);
              blackboard->set("loc_control_degraded", msg->control_degraded);
            });

    const auto loc_backend_topic =
        this->get_parameter("localization_backend_status_topic").as_string();
    localization_backend_sub_ = this->create_subscription<
        rc26_interfaces::msg::LocalizationBackendStatus>(
        loc_backend_topic, rclcpp::SensorDataQoS(),
        [blackboard](
            const rc26_interfaces::msg::LocalizationBackendStatus::SharedPtr
                msg) {
          blackboard->set("loc_graph_health", msg->graph_health);
          blackboard->set("loc_optimizer_ready", msg->optimizer_ready);
        });

    const auto loc_route_topic =
        this->get_parameter("localization_route_observability_topic")
            .as_string();
    localization_route_sub_ =
        this->create_subscription<rc26_interfaces::msg::RouteObservability>(
            loc_route_topic, rclcpp::SensorDataQoS(),
            [blackboard](
                const rc26_interfaces::msg::RouteObservability::SharedPtr msg) {
              blackboard->set("loc_route_score", msg->score);
              blackboard->set("loc_route_risk_level",
                              static_cast<int>(msg->risk_level));
              if (!msg->recommended_nav_profile.empty()) {
                blackboard->set("loc_recommended_profile",
                                msg->recommended_nav_profile);
              }
            });

    // 注册所有行为树节点
    registerMCAreaNodes(factory_);
    registerMFAreaNodes(factory_);
    registerCombatAreaNodes(factory_);
    registerNavigationNodes(factory_);
    registerVisionNodes(factory_);

    // 加载行为树 XML
    std::string tree_file = this->get_parameter("tree_file").as_string();
    std::string package_path =
        ament_index_cpp::get_package_share_directory("rc26_decision");
    std::string tree_path = package_path + "/behavior_trees/" + tree_file;

    RCLCPP_INFO(this->get_logger(), "加载行为树: %s", tree_path.c_str());
    tree_ = factory_.createTreeFromFile(tree_path, blackboard);

    // 创建定时器执行 tick
    int tick_rate_ms = this->get_parameter("tick_rate_ms").as_int();
    timer_ = this->create_wall_timer(std::chrono::milliseconds(tick_rate_ms),
                                     std::bind(&DecisionNode::tickTree, this));

    // /mf_kfs_state 发布器（5Hz）
    const auto kfs_topic = this->get_parameter("kfs_state_topic").as_string();
    pub_kfs_state_ = this->create_publisher<rc26_interfaces::msg::MfKfsState>(
        kfs_topic, rclcpp::QoS(rclcpp::KeepLast(3)).reliable());
    kfs_timer_ =
        this->create_wall_timer(std::chrono::milliseconds(200),
                                [this]() { (void)publishKfsState(true); });

    RCLCPP_INFO(this->get_logger(), "决策节点已启动, tick 频率: %d ms",
                tick_rate_ms);
  }

private:
  void tickTree() {
    BT::NodeStatus status = tree_.tickOnce();

    // KFS 状态变化立即发布（周期定时器仍保留 5Hz 保底）
    (void)publishKfsState(false);

    if (status == BT::NodeStatus::SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "行为树执行完成: SUCCESS");
      timer_->cancel();
    } else if (status == BT::NodeStatus::FAILURE) {
      RCLCPP_ERROR(this->get_logger(), "行为树执行失败: FAILURE");
      timer_->cancel();
    }
  }

  bool publishKfsState(bool force_publish) {
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
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<rc26_interfaces::msg::MfKfsState>::SharedPtr pub_kfs_state_;
  rclcpp::TimerBase::SharedPtr kfs_timer_;
  std::array<uint8_t, 13> last_kfs_type_{};
  std::array<float, 13> last_kfs_confidence_{};
  std::string last_kfs_team_;
  bool have_last_kfs_snapshot_{false};
  std::shared_ptr<WaypointManager> waypoint_manager_;
  std::shared_ptr<SmartWaypointNavigator> smart_waypoint_navigator_;
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
  rclcpp::Subscription<rc26_interfaces::msg::LocalizationHealth>::SharedPtr
      localization_health_sub_;
  rclcpp::Subscription<rc26_interfaces::msg::LocalizationBackendStatus>::
      SharedPtr localization_backend_sub_;
  rclcpp::Subscription<rc26_interfaces::msg::RouteObservability>::SharedPtr
      localization_route_sub_;
  std::shared_ptr<rc26_vision::VisionInferenceManager> vision_manager_;
};

} // namespace rc26_decision

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rc26_decision::DecisionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
