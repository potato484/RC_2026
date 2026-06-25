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
  // 读取 MCU 业务反馈 topic；0x17/0x18/0x1A 激光高度突变事件从这里透传过来。
  p.feedback_topic =
      node.declare_parameter<std::string>("stair_feedback_topic",
                                          p.feedback_topic);
  // 读取上台阶普通直行速度绝对值；具体正方向由 StairClimb 状态机决定。
  p.climb_drive_speed_mps = node.declare_parameter<double>(
      "stair_climb_drive_speed_mps", p.climb_drive_speed_mps);
  // 读取上台阶第六阶段后轮上台阶速度规划；该阶段从 fast 线性降到 slow 后保持。
  p.climb_rear_drive_fast_speed_mps = node.declare_parameter<double>(
      "stair_climb_rear_drive_fast_speed_mps",
      p.climb_rear_drive_fast_speed_mps);
  p.climb_rear_drive_slow_speed_mps = node.declare_parameter<double>(
      "stair_climb_rear_drive_slow_speed_mps",
      p.climb_rear_drive_slow_speed_mps);
  p.climb_rear_drive_slowdown_duration_s = node.declare_parameter<double>(
      "stair_climb_rear_drive_slowdown_duration_s",
      p.climb_rear_drive_slowdown_duration_s);
  // 读取下台阶普通直行速度绝对值；具体负方向由 StairDescend 状态机决定。
  p.descend_drive_speed_mps = node.declare_parameter<double>(
      "stair_descend_drive_speed_mps", p.descend_drive_speed_mps);
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
  // 读取上台阶前推杆伸出 accepted 后的零速等待时间。
  p.climb_front_extend_delay_s =
      node.declare_parameter<double>("stair_climb_front_extend_delay_s",
                                     p.climb_front_extend_delay_s);
  // 读取上台阶前推杆收回 + 后推杆伸出 accepted 后的零速等待时间。
  p.climb_retract_rear_extend_delay_s = node.declare_parameter<double>(
      "stair_climb_retract_rear_extend_delay_s",
      p.climb_retract_rear_extend_delay_s);
  // 读取上台阶最后后推杆收回 accepted 后的零速等待时间。
  p.climb_rear_retract_delay_s = node.declare_parameter<double>(
      "stair_climb_rear_retract_delay_s", p.climb_rear_retract_delay_s);
  // 读取下台阶后推杆伸出 accepted 后的零速等待时间。
  p.descend_rear_extend_delay_s =
      node.declare_parameter<double>("stair_descend_rear_extend_delay_s",
                                     p.descend_rear_extend_delay_s);
  // 读取下台阶后推杆收回 + 前推杆伸出 accepted 后的零速等待时间。
  p.descend_retract_front_extend_delay_s = node.declare_parameter<double>(
      "stair_descend_retract_front_extend_delay_s",
      p.descend_retract_front_extend_delay_s);
  // 读取下台阶触发前推杆收回前的 x 负向定时行驶速度绝对值。
  p.descend_front_retract_drive_speed_mps = node.declare_parameter<double>(
      "stair_descend_front_retract_drive_speed_mps",
      p.descend_front_retract_drive_speed_mps);
  // 读取下台阶触发前推杆收回前的 x 负向定时行驶时长。
  p.descend_front_retract_drive_duration_s = node.declare_parameter<double>(
      "stair_descend_front_retract_drive_duration_s",
      p.descend_front_retract_drive_duration_s);
  // 读取下台阶前推杆收回 accepted 后的零速等待时间。
  p.descend_front_retract_delay_s = node.declare_parameter<double>(
      "stair_descend_front_retract_delay_s",
      p.descend_front_retract_delay_s);
  // 读取台阶姿态保持用 odom 话题；yaw 由四元数直接解算。
  p.odom_topic =
      node.declare_parameter<std::string>("stair_odom_topic", p.odom_topic);
  // 是否启用台阶直行阶段的 heading hold。
  p.heading_hold_enable = node.declare_parameter<bool>(
      "stair_heading_hold_enable", p.heading_hold_enable);
  // yaw 误差到 angular.z 的比例系数。
  p.heading_kp =
      node.declare_parameter<double>("stair_heading_kp", p.heading_kp);
  // heading hold 最大角速度。
  p.heading_max_speed_radps = node.declare_parameter<double>(
      "stair_heading_max_speed_radps", p.heading_max_speed_radps);
  // 进入该 yaw 容差并稳定若干 tick 后才开始跨阶梯。
  p.heading_tolerance_deg = node.declare_parameter<double>(
      "stair_heading_tolerance_deg", p.heading_tolerance_deg);
  // yaw gate 当前用于文档和调参口径，实际对齐阶段会转到 tolerance 内。
  p.heading_gate_deg =
      node.declare_parameter<double>("stair_heading_gate_deg",
                                     p.heading_gate_deg);
  p.heading_stable_ticks = node.declare_parameter<int>(
      "stair_heading_stable_ticks", p.heading_stable_ticks);
  p.heading_odom_timeout_s = node.declare_parameter<double>(
      "stair_heading_odom_timeout_s", p.heading_odom_timeout_s);
  p.heading_align_timeout_s = node.declare_parameter<double>(
      "stair_heading_align_timeout_s", p.heading_align_timeout_s);

  // 速度只保留绝对值；方向在具体动作里显式加正负号，避免参数符号造成流程反向。
  p.climb_drive_speed_mps = std::abs(p.climb_drive_speed_mps);
  p.climb_rear_drive_fast_speed_mps =
      std::abs(p.climb_rear_drive_fast_speed_mps);
  if (p.climb_rear_drive_fast_speed_mps <= 0.0) {
    p.climb_rear_drive_fast_speed_mps = p.climb_drive_speed_mps;
  }
  p.climb_rear_drive_slow_speed_mps =
      std::abs(p.climb_rear_drive_slow_speed_mps);
  p.climb_rear_drive_slow_speed_mps =
      std::min(p.climb_rear_drive_slow_speed_mps,
               p.climb_rear_drive_fast_speed_mps);
  p.climb_rear_drive_slowdown_duration_s =
      std::max(0.0, p.climb_rear_drive_slowdown_duration_s);
  p.descend_drive_speed_mps = std::abs(p.descend_drive_speed_mps);
  // 发布频率至少 1Hz，避免除零和过低频率导致动作看起来卡死。
  p.command_rate_hz = std::max(1.0, p.command_rate_hz);
  // 命令超时至少 1ms，避免配置成 0 或负数后立即出现不可解释行为。
  p.command_timeout_s = std::max(0.001, p.command_timeout_s);
  // 前轮事件超时至少 1ms，保持超时逻辑始终有效。
  p.front_event_timeout_s = std::max(0.001, p.front_event_timeout_s);
  // 后轮事件超时至少 1ms，保持超时逻辑始终有效。
  p.rear_event_timeout_s = std::max(0.001, p.rear_event_timeout_s);
  // 上台阶零速等待允许为 0；0 表示推杆命令 accepted 后立即进入下一阶段。
  p.climb_front_extend_delay_s = std::max(0.0, p.climb_front_extend_delay_s);
  p.climb_retract_rear_extend_delay_s =
      std::max(0.0, p.climb_retract_rear_extend_delay_s);
  p.climb_rear_retract_delay_s =
      std::max(0.0, p.climb_rear_retract_delay_s);
  // 下台阶零速等待允许为 0；0 表示推杆命令 accepted 后立即进入下一阶段。
  p.descend_rear_extend_delay_s =
      std::max(0.0, p.descend_rear_extend_delay_s);
  p.descend_retract_front_extend_delay_s =
      std::max(0.0, p.descend_retract_front_extend_delay_s);
  p.descend_front_retract_drive_speed_mps =
      std::abs(p.descend_front_retract_drive_speed_mps);
  p.descend_front_retract_drive_duration_s =
      std::max(0.0, p.descend_front_retract_drive_duration_s);
  p.descend_front_retract_delay_s =
      std::max(0.0, p.descend_front_retract_delay_s);
  p.heading_kp = std::max(0.0, p.heading_kp);
  p.heading_max_speed_radps = std::max(0.0, p.heading_max_speed_radps);
  p.heading_tolerance_deg = std::max(0.0, p.heading_tolerance_deg);
  p.heading_gate_deg = std::max(p.heading_tolerance_deg, p.heading_gate_deg);
  p.heading_stable_ticks = std::max(1, p.heading_stable_ticks);
  p.heading_odom_timeout_s = std::max(0.001, p.heading_odom_timeout_s);
  p.heading_align_timeout_s = std::max(0.001, p.heading_align_timeout_s);

  // 把完整参数对象写入黑板；实际 BT 节点从黑板读取，避免每个节点重复声明 ROS 参数。
  blackboard->set("stair_params", p);
  // 启动日志只打印关键入口，便于确认独立台阶树使用的是哪个 topic/service。
  RCLCPP_INFO(node.get_logger(),
              "台阶动作参数已加载: cmd_vel=%s feedback=%s odom=%s climb_speed=%.3fm/s climb_rear_profile=%.3f->%.3fm/s %.2fs descend_speed=%.3fm/s climb_delays=%.2f/%.2f/%.2fs descend_delays=%.2f/%.2f/%.2fs descend_front_retract_drive=%.3fm/s %.2fs heading=%s kp=%.2f max=%.2frad/s tol=%.1fdeg",
              p.cmd_vel_topic.c_str(), p.feedback_topic.c_str(),
              p.odom_topic.c_str(), p.climb_drive_speed_mps,
              p.climb_rear_drive_fast_speed_mps,
              p.climb_rear_drive_slow_speed_mps,
              p.climb_rear_drive_slowdown_duration_s,
              p.descend_drive_speed_mps,
              p.climb_front_extend_delay_s,
              p.climb_retract_rear_extend_delay_s,
              p.climb_rear_retract_delay_s,
              p.descend_rear_extend_delay_s,
              p.descend_retract_front_extend_delay_s,
              p.descend_front_retract_delay_s,
              p.descend_front_retract_drive_speed_mps,
              p.descend_front_retract_drive_duration_s,
              p.heading_hold_enable ? "on" : "off", p.heading_kp,
              p.heading_max_speed_radps, p.heading_tolerance_deg);
}

// registerStairNodes() 只把节点类型注册进 BehaviorTreeFactory，不会让默认主流程自动执行它们。
void registerStairNodes(BT::BehaviorTreeFactory &factory) {
  // 注册上台阶动作；只有 XML 中出现 <StairClimb/> 时才会实例化。
  factory.registerNodeType<StairClimbAction>("StairClimb");
  // 注册下台阶动作；只有 XML 中出现 <StairDescend/> 时才会实例化。
  factory.registerNodeType<StairDescendAction>("StairDescend");
}

} // namespace rc26_decision
