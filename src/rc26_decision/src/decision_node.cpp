#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <array>
#include <cmath>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int8.hpp>

#include "rc26_decision/combat/combat_area.hpp"
#include "rc26_decision/mc/mc_area.hpp"
#include "rc26_decision/mf/mf_area.hpp"
#include "rc26_decision/navigation/bt_nav_to_smart_point.hpp"
#include "rc26_decision/navigation/smart_waypoint_navigator.hpp"
#include "rc26_decision/navigation/waypoint_manager.hpp"
#include "rc26_decision/vision/bt_nodes.hpp"
#include "rc26_serial/serial_driver.hpp"
#include "rc26_vision/vision_inference_manager.hpp"
#include "rc26_vision/profile_loader.hpp"
#include "rc26_interfaces/msg/mf_kfs_state.hpp"
#include "rc26_interfaces/msg/mf_kfs_cell.hpp"

namespace rc26_decision {

class DecisionNode : public rclcpp::Node {
public:
    DecisionNode() : Node("rc26_decision") {
        // 声明参数
        this->declare_parameter<std::string>("tree_file", "main_tree.xml");
        this->declare_parameter<int>("tick_rate_ms", 100);
        this->declare_parameter<bool>("enable_cmd_serial", true);
        this->declare_parameter<std::string>("cmd_serial_port", "/dev/ttyUSB1");
        this->declare_parameter<int>("cmd_baudrate", 115200);
        this->declare_parameter<bool>("enable_heartbeat", true);
        this->declare_parameter<int>("heartbeat_rate_hz", 1);
        this->declare_parameter<std::string>("nav2_action_name", "navigate_to_pose");
        this->declare_parameter<std::string>("nav2_goal_frame", "map");
        this->declare_parameter<std::string>("controller_server_node", "controller_server");
        this->declare_parameter<std::string>("odom_topic", "odom");
        this->declare_parameter<std::string>("team", "blue");
        this->declare_parameter<std::string>("waypoints_file", "");
        this->declare_parameter<double>("stop_linear_eps_mps", 0.05);
        this->declare_parameter<double>("stop_angular_eps_rps", 0.1);
        this->declare_parameter<std::string>("base_ground_level_topic", "base_ground/level");
        this->declare_parameter<std::string>("base_ground_stair_delta_topic", "base_ground/stair_delta");
        this->declare_parameter<std::string>("base_ground_stable_topic", "base_ground/stable");
        this->declare_parameter<std::string>("kfs_state_topic", "mf_kfs_state");

        // 初始化命令串口 (串口2)
        if (this->get_parameter("enable_cmd_serial").as_bool()) {
            cmd_serial_ = std::make_shared<SerialDriver>();
            std::string cmd_port = this->get_parameter("cmd_serial_port").as_string();
            int cmd_baud = this->get_parameter("cmd_baudrate").as_int();

            if (!cmd_serial_->open(cmd_port, cmd_baud)) {
                RCLCPP_ERROR(this->get_logger(), "无法打开命令串口: %s", cmd_port.c_str());
            } else {
                RCLCPP_INFO(this->get_logger(), "命令串口已打开: %s", cmd_port.c_str());

                // 启动心跳
                if (this->get_parameter("enable_heartbeat").as_bool()) {
                    const int heartbeat_rate_hz = this->get_parameter("heartbeat_rate_hz").as_int();
                    if (heartbeat_rate_hz <= 0) {
                        RCLCPP_WARN(this->get_logger(),
                                    "heartbeat_rate_hz=%d <= 0, heartbeat disabled",
                                    heartbeat_rate_hz);
                    } else {
                        const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::duration<double>(1.0 / static_cast<double>(heartbeat_rate_hz)));
                        heartbeat_timer_ = this->create_wall_timer(period, [this]() { handleHeartbeat(); });
                    }
                }
            }
        }

        // 创建黑板并共享
        blackboard_ = BT::Blackboard::create();
        const auto& blackboard = blackboard_;
        blackboard->set("cmd_serial", cmd_serial_);
        {
            rclcpp::Node* node_ptr = this;
            blackboard->set("node", node_ptr);
        }

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
        // 注意: 需要用户配置 enable_vision 参数和模型路径
        this->declare_parameter<bool>("enable_vision", false);
        this->declare_parameter<std::string>("vision_config_file", "");

        // 初始化 vision_current_model 黑板键
        blackboard->set("vision_current_model", std::string(""));

        if (this->get_parameter("enable_vision").as_bool()) {
            std::string config_file = this->get_parameter("vision_config_file").as_string();

            vision_manager_ = std::make_shared<rc26_vision::VisionInferenceManager>(*this);

            if (!config_file.empty()) {
                // 从 YAML 配置文件加载多 Profile
                try {
                    auto config = rc26_vision::ProfileLoader::loadFromYaml(config_file);
                    vision_manager_->loadConfig(config);
                    if (!config.default_model.empty()) {
                        vision_manager_->selectModel(config.default_model);
                        blackboard->set("vision_current_model", config.default_model);
                    }
                    blackboard->set("vision_manager", vision_manager_);
                    RCLCPP_INFO(this->get_logger(), "视觉配置已加载: %s", config_file.c_str());
                } catch (const std::exception& e) {
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
            std::string waypoints_file = this->get_parameter("waypoints_file").as_string();
            if (waypoints_file.empty()) {
                std::string team = this->get_parameter("team").as_string();
                std::string package_path = ament_index_cpp::get_package_share_directory("rc26_decision");
                waypoints_file = package_path + "/config/waypoints/waypoints_" + team + ".yaml";
            }
            waypoint_manager_ = std::make_shared<WaypointManager>();
            if (!waypoint_manager_->loadFromYamlFile(waypoints_file)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to load waypoints: %s", waypoints_file.c_str());
            } else {
                RCLCPP_INFO(this->get_logger(), "Loaded waypoints from: %s", waypoints_file.c_str());
            }
            blackboard->set("waypoint_manager", waypoint_manager_);
            // 将 team 参数设置到黑板，供 MF 区域节点使用
            const std::string team = this->get_parameter("team").as_string();
            blackboard->set("team", team);

            auto merlin_map = std::make_shared<MerlinMapManager>();
            if (team == "blue") {
                merlin_map->initBlueMap();
            } else {
                merlin_map->initRedMap();
            }
            blackboard->set("merlin_map", merlin_map);
            RCLCPP_INFO(this->get_logger(), "Cold-start merlin_map initialized for team=%s", team.c_str());
        }

        // 创建 SmartWaypointNavigator 并共享到黑板
        {
            const auto nav2_action_name = this->get_parameter("nav2_action_name").as_string();
            const auto nav2_goal_frame = this->get_parameter("nav2_goal_frame").as_string();
            const auto controller_node = this->get_parameter("controller_server_node").as_string();
            const auto odom_topic = this->get_parameter("odom_topic").as_string();
            const double stop_lin = this->get_parameter("stop_linear_eps_mps").as_double();
            const double stop_ang = this->get_parameter("stop_angular_eps_rps").as_double();
            smart_waypoint_navigator_ = std::make_shared<SmartWaypointNavigator>(
                *this, nav2_action_name, nav2_goal_frame, controller_node, odom_topic, stop_lin, stop_ang);
            blackboard->set("smart_waypoint_navigator", smart_waypoint_navigator_);
        }

        // 初始化重连状态（行为树可直接查询）
        blackboard->set("cmd_serial_reconnecting", false);
        blackboard->set("cmd_serial_reconnect_failed", false);

        // 初始化反馈状态
        blackboard->set("grab_tip_done", false);
        blackboard->set("assemble_done", false);
        blackboard->set("climbing_slope", false);
        blackboard->set("slope_done", false);
        blackboard->set("rotate_done", false);
        blackboard->set("mech_up_merlin_done", false);
        blackboard->set("mech_down_merlin_done", false);
        blackboard->set("grab_kfs_done", false);
        blackboard->set("mech_up_duel_done", false);
        blackboard->set("place_kfs_grid_done", false);
        blackboard->set("place_kfs_ground_done", false);
        blackboard->set("stair_climb_done", false);
        blackboard->set("stair_descend_done", false);
        blackboard->set("action_fail", false);
        blackboard->set("system_error", false);
        blackboard->set("current_level", static_cast<int32_t>(0));
        blackboard->set("stair_delta", static_cast<int8_t>(0));
        blackboard->set("base_ground_stable", false);
        blackboard->set("level_start", static_cast<int32_t>(0));

        // 订阅 base_ground 话题
        const auto level_topic = this->get_parameter("base_ground_level_topic").as_string();
        base_ground_level_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            level_topic, 10, [blackboard](const std_msgs::msg::Int32::SharedPtr msg) {
                blackboard->set("current_level", msg->data);
            });

        const auto stair_delta_topic = this->get_parameter("base_ground_stair_delta_topic").as_string();
        base_ground_stair_delta_sub_ = this->create_subscription<std_msgs::msg::Int8>(
            stair_delta_topic, 10, [blackboard](const std_msgs::msg::Int8::SharedPtr msg) {
                blackboard->set("stair_delta", static_cast<int8_t>(msg->data));
            });

        const auto stable_topic = this->get_parameter("base_ground_stable_topic").as_string();
        base_ground_stable_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            stable_topic, 10, [blackboard](const std_msgs::msg::Bool::SharedPtr msg) {
                blackboard->set("base_ground_stable", msg->data);
            });

        // 为命令串口挂载重连回调（将重连状态写入黑板）
        if (cmd_serial_) {
            cmd_serial_->setReconnectStartCallback([this, blackboard]() {
                blackboard->set("cmd_serial_reconnecting", true);
                blackboard->set("cmd_serial_reconnect_failed", false);
                RCLCPP_WARN(this->get_logger(), "命令串口开始重连，已写入黑板");
            });

            cmd_serial_->setReconnectCallback([this, blackboard]() {
                blackboard->set("cmd_serial_reconnecting", false);
                blackboard->set("cmd_serial_reconnect_failed", false);
                RCLCPP_INFO(this->get_logger(), "命令串口重连成功，已更新黑板");
            });

            cmd_serial_->setReconnectFailedCallback([this, blackboard]() {
                blackboard->set("cmd_serial_reconnecting", false);
                blackboard->set("cmd_serial_reconnect_failed", true);
                RCLCPP_ERROR(this->get_logger(), "命令串口重连失败，已写入黑板");
            });
        }

        // 为命令串口挂载反馈回调 (将反馈 ID 存入黑板)
        if (cmd_serial_) {
            cmd_serial_->setReceiveCallback(
                [this, blackboard](uint8_t seq, uint8_t cmd, const std::vector<uint8_t>& payload) {
                    (void)seq;
                    (void)payload;

                    // 心跳反馈：HEARTBEAT_ACK(0x10) 已由 SerialDriver 内部处理
                    if (cmd == static_cast<uint8_t>(FeedbackID::HEARTBEAT_ACK)) {
                        return;
                    }

                    // 1. 通用映射：以十六进制 ID 为键 (如 feedback_0x02)
                    char hex_str[16];
                    std::snprintf(hex_str, sizeof(hex_str), "feedback_0x%02X", cmd);
                    blackboard->set(std::string(hex_str), true);

                    // 2. 语义映射：将常用 ID 转换为直观的布尔值
                    switch (static_cast<FeedbackID>(cmd)) {
                    case FeedbackID::GRAB_TIP_DONE:
                        blackboard->set("grab_tip_done", true);
                        break;
                    case FeedbackID::ASSEMBLE_DONE:
                        blackboard->set("assemble_done", true);
                        break;
                    case FeedbackID::CLIMBING_SLOPE:
                        blackboard->set("climbing_slope", true);
                        break;
                    case FeedbackID::SLOPE_DONE:
                        blackboard->set("slope_done", true);
                        break;
                    case FeedbackID::ROTATE_POS_90_DONE:
                    case FeedbackID::ROTATE_NEG_90_DONE:
                    case FeedbackID::ROTATE_POS_180_DONE:
                    case FeedbackID::ROTATE_NEG_180_DONE:
                        blackboard->set("rotate_done", true);
                        break;
                    case FeedbackID::MECH_UP_MERLIN_DONE:
                        blackboard->set("mech_up_merlin_done", true);
                        break;
                    case FeedbackID::MECH_DOWN_MERLIN_DONE:
                        blackboard->set("mech_down_merlin_done", true);
                        break;
                    case FeedbackID::GRAB_KFS_DONE:
                        blackboard->set("grab_kfs_done", true);
                        break;
                    case FeedbackID::MECH_UP_DUEL_DONE:
                        blackboard->set("mech_up_duel_done", true);
                        break;
                    case FeedbackID::PLACE_KFS_GRID_DONE:
                        blackboard->set("place_kfs_grid_done", true);
                        break;
                    case FeedbackID::PLACE_KFS_GROUND_DONE:
                        blackboard->set("place_kfs_ground_done", true);
                        break;
                    case FeedbackID::STAIR_CLIMB_DONE:
                        blackboard->set("stair_climb_done", true);
                        break;
                    case FeedbackID::STAIR_DESCEND_DONE:
                        blackboard->set("stair_descend_done", true);
                        break;
                    case FeedbackID::ACTION_FAIL:
                        blackboard->set("action_fail", true);
                        break;
                    case FeedbackID::ERROR:
                        blackboard->set("system_error", true);
                        break;
                    default:
                        break;
                    }
                });
        }

        // 注册所有行为树节点
        registerMCAreaNodes(factory_);
        registerMFAreaNodes(factory_);
        registerCombatAreaNodes(factory_);
        registerNavigationNodes(factory_);
        registerVisionNodes(factory_);

        // 加载行为树 XML
        std::string tree_file = this->get_parameter("tree_file").as_string();
        std::string package_path = ament_index_cpp::get_package_share_directory("rc26_decision");
        std::string tree_path = package_path + "/behavior_trees/" + tree_file;

        RCLCPP_INFO(this->get_logger(), "加载行为树: %s", tree_path.c_str());
        tree_ = factory_.createTreeFromFile(tree_path, blackboard);

        // 创建定时器执行 tick
        int tick_rate_ms = this->get_parameter("tick_rate_ms").as_int();
        timer_ =
            this->create_wall_timer(std::chrono::milliseconds(tick_rate_ms), std::bind(&DecisionNode::tickTree, this));

        // /mf_kfs_state 发布器（5Hz）
        const auto kfs_topic = this->get_parameter("kfs_state_topic").as_string();
        pub_kfs_state_ = this->create_publisher<rc26_interfaces::msg::MfKfsState>(
            kfs_topic, rclcpp::QoS(rclcpp::KeepLast(3)).reliable());
        kfs_timer_ = this->create_wall_timer(std::chrono::milliseconds(200), [this]() {
            (void)publishKfsState(true);
        });

        RCLCPP_INFO(this->get_logger(), "决策节点已启动, tick 频率: %d ms", tick_rate_ms);
    }

private:
    void handleHeartbeat() {
        if (!cmd_serial_ || !cmd_serial_->isOpen()) {
            return;
        }
        // SerialDriver::sendHeartbeat() 内部已处理心跳失败计数与重连逻辑
        cmd_serial_->sendHeartbeat();
    }

    void tickTree() {
        BT::NodeStatus status = tree_.tickOnce();

        // KFS 状态变化立即发布（周期定时器仍保留 2Hz 保底）
        (void)publishKfsState(false);

        if (status == BT::NodeStatus::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "行为树执行完成: SUCCESS");
            timer_->cancel();
        } else if (status == BT::NodeStatus::FAILURE) {
            RCLCPP_ERROR(this->get_logger(), "行为树执行失败: FAILURE");
            timer_->cancel();
        }
        // RUNNING 状态继续 tick
    }

    bool publishKfsState(bool force_publish) {
        std::shared_ptr<MerlinMapManager> merlin_map;
        if (!blackboard_->get("merlin_map", merlin_map) || !merlin_map) return false;

        std::string team;
        if (!blackboard_->get("team", team)) team.clear();

        std::array<uint8_t, 13> kfs_type{};
        std::array<float, 13> kfs_confidence{};
        rc26_interfaces::msg::MfKfsState msg;
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "map";
        msg.team = team;

        for (int grid = 1; grid <= 12; grid++) {
            const auto kfs = merlin_map->getKFS(grid);
            rc26_interfaces::msg::MfKfsCell cell;
            cell.grid_id    = static_cast<uint8_t>(grid);
            cell.kfs_type   = static_cast<uint8_t>(kfs);
            if (kfs == KFSType::UNKNOWN) {
                cell.confidence = 0.0f;
            } else {
                cell.confidence = 1.0f;
            }
            kfs_type[static_cast<size_t>(grid)] = cell.kfs_type;
            kfs_confidence[static_cast<size_t>(grid)] = cell.confidence;
            msg.cells.push_back(cell);
        }

        bool changed = !have_last_kfs_snapshot_ || (team != last_kfs_team_);
        for (int grid = 1; grid <= 12 && !changed; grid++) {
            const size_t idx = static_cast<size_t>(grid);
            if (kfs_type[idx] != last_kfs_type_[idx] ||
                std::fabs(kfs_confidence[idx] - last_kfs_confidence_[idx]) > 1e-5f) {
                changed = true;
            }
        }

        if (!force_publish && !changed) return false;

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
    std::shared_ptr<SerialDriver> cmd_serial_;
    std::shared_ptr<WaypointManager> waypoint_manager_;
    std::shared_ptr<SmartWaypointNavigator> smart_waypoint_navigator_;
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr base_ground_level_sub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr base_ground_stair_delta_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr base_ground_stable_sub_;
    std::shared_ptr<rc26_vision::VisionInferenceManager> vision_manager_;
};

}  // namespace rc26_decision

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rc26_decision::DecisionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
