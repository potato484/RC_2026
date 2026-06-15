#include "rc26_decision/stair/stair_area.hpp"

#include <algorithm>
#include <cmath>

#include "rc26_decision/stair/stair_climb.hpp"
#include "rc26_decision/stair/stair_descend.hpp"

namespace rc26_decision {

// loadStairParams() 在 decision_node 构造阶段调用，把 ROS 参数声明、读取并写入 BT 黑板。
void loadStairParams(rclcpp::Node &node,
                     const BT::Blackboard::Ptr &blackboard) {
  // 先创建默认参数集合；每个字段的默认值定义在 StairParams 结构体里。
  StairParams p;
  // 读取台阶动作发布速度的话题；默认 cmd_vel，单独运行台阶树时必须确保没有其它速度权威。
  p.cmd_vel_topic =
      node.declare_parameter<std::string>("stair_cmd_vel_topic",
                                          p.cmd_vel_topic);
  // 读取共享串口推杆命令 service；台阶动作通过它下发 FRONT/REAR_PUSHROD_*。
  p.send_command_service =
      node.declare_parameter<std::string>("stair_send_command_service",
                                          p.send_command_service);
  // 读取 MCU 业务反馈 topic；0x17/0x18 激光高度突变事件从这里透传过来。
  p.feedback_topic =
      node.declare_parameter<std::string>("stair_feedback_topic",
                                          p.feedback_topic);
  // 读取台阶直行速度绝对值；具体方向由 StairClimb/StairDescend 状态机决定。
  p.drive_speed_mps =
      node.declare_parameter<double>("stair_drive_speed_mps",
                                     p.drive_speed_mps);
  // 读取速度命令发布频率；公共基类会按这个频率限流 cmd_vel。
  p.command_rate_hz =
      node.declare_parameter<double>("stair_command_rate_hz",
                                     p.command_rate_hz);
  // 读取每条推杆命令等待 accepted=true 的超时时间。
  p.command_timeout_s =
      node.declare_parameter<double>("stair_command_timeout_s",
                                     p.command_timeout_s);
  // 读取等待前轮激光高度突变事件的超时时间。
  p.front_event_timeout_s =
      node.declare_parameter<double>("stair_front_event_timeout_s",
                                     p.front_event_timeout_s);
  // 读取等待后轮激光高度突变事件的超时时间。
  p.rear_event_timeout_s =
      node.declare_parameter<double>("stair_rear_event_timeout_s",
                                     p.rear_event_timeout_s);
  // 读取下台阶末段固定时间直行的持续时间。
  p.descend_finish_drive_time_s =
      node.declare_parameter<double>("stair_descend_finish_drive_time_s",
                                     p.descend_finish_drive_time_s);

  // 速度只保留绝对值；方向在具体动作里显式加正负号，避免参数符号造成流程反向。
  p.drive_speed_mps = std::abs(p.drive_speed_mps);
  // 发布频率至少 1Hz，避免除零和过低频率导致动作看起来卡死。
  p.command_rate_hz = std::max(1.0, p.command_rate_hz);
  // 命令超时至少 1ms，避免配置成 0 或负数后立即出现不可解释行为。
  p.command_timeout_s = std::max(0.001, p.command_timeout_s);
  // 前轮事件超时至少 1ms，保持超时逻辑始终有效。
  p.front_event_timeout_s = std::max(0.001, p.front_event_timeout_s);
  // 后轮事件超时至少 1ms，保持超时逻辑始终有效。
  p.rear_event_timeout_s = std::max(0.001, p.rear_event_timeout_s);
  // 下台阶末段行驶时间允许为 0；0 表示收到命令后直接结束并停车。
  p.descend_finish_drive_time_s =
      std::max(0.0, p.descend_finish_drive_time_s);

  // 把完整参数对象写入黑板；实际 BT 节点从黑板读取，避免每个节点重复声明 ROS 参数。
  blackboard->set("stair_params", p);
  // 启动日志只打印关键入口，便于确认独立台阶树使用的是哪个 topic/service。
  RCLCPP_INFO(node.get_logger(),
              "台阶动作参数已加载: cmd_vel=%s feedback=%s speed=%.3fm/s",
              p.cmd_vel_topic.c_str(), p.feedback_topic.c_str(),
              p.drive_speed_mps);
}

// registerStairNodes() 只把节点类型注册进 BehaviorTreeFactory，不会让默认主流程自动执行它们。
void registerStairNodes(BT::BehaviorTreeFactory &factory) {
  // 注册上台阶动作；只有 XML 中出现 <StairClimb/> 时才会实例化。
  factory.registerNodeType<StairClimbAction>("StairClimb");
  // 注册下台阶动作；只有 XML 中出现 <StairDescend/> 时才会实例化。
  factory.registerNodeType<StairDescendAction>("StairDescend");
}

} // namespace rc26_decision
