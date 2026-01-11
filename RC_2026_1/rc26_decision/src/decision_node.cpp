#include <atomic>
#include <mutex>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

#include "rc26_decision/combat/combat_area.hpp"
#include "rc26_decision/mc/mc_area.hpp"
#include "rc26_decision/mf/mf_area.hpp"
#include "rc26_decision/navigation/bt_navigate_waypoint.hpp"
#include "rc26_decision/navigation/waypoint_navigator.hpp"
#include "rc26_serial/serial_driver.hpp"

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

        // 初始化命令串口 (串口2)
        if (this->get_parameter("enable_cmd_serial").as_bool()) {
            cmd_serial_ = std::make_shared<SerialDriver>();
            std::string cmd_port = this->get_parameter("cmd_serial_port").as_string();
            int cmd_baud = this->get_parameter("cmd_baudrate").as_int();

            if (!cmd_serial_->open(cmd_port, cmd_baud)) {
                RCLCPP_ERROR(this->get_logger(), "无法打开命令串口: %s", cmd_port.c_str());
            } else {
                RCLCPP_INFO(this->get_logger(), "命令串口已打开: %s", cmd_port.c_str());

                // 启动心跳（每秒1次）
                if (this->get_parameter("enable_heartbeat").as_bool()) {
                    heartbeat_timer_ =
                        this->create_wall_timer(std::chrono::milliseconds(1000), [this]() { handleHeartbeat(); });
                }
            }
        }

        // 创建黑板并共享
        auto blackboard = BT::Blackboard::create();
        blackboard->set("cmd_serial", cmd_serial_);

        // 创建 WaypointNavigator 并共享到黑板
        {
            const auto nav2_action_name = this->get_parameter("nav2_action_name").as_string();
            const auto nav2_goal_frame = this->get_parameter("nav2_goal_frame").as_string();
            waypoint_navigator_ =
                std::make_shared<WaypointNavigator>(*this, cmd_serial_, nav2_action_name, nav2_goal_frame);
            blackboard->set("waypoint_navigator", waypoint_navigator_);
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

        if (status == BT::NodeStatus::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "行为树执行完成: SUCCESS");
            timer_->cancel();
        } else if (status == BT::NodeStatus::FAILURE) {
            RCLCPP_ERROR(this->get_logger(), "行为树执行失败: FAILURE");
            timer_->cancel();
        }
        // RUNNING 状态继续 tick
    }

    BT::BehaviorTreeFactory factory_;
    BT::Tree tree_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<SerialDriver> cmd_serial_;
    std::shared_ptr<WaypointNavigator> waypoint_navigator_;
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;
};

}  // namespace rc26_decision

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rc26_decision::DecisionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
