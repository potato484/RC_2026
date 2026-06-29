#include "rc26_decision/stair/stair_area.hpp"

#include <algorithm>
#include <cmath>

#include "rc26_decision/stair/stair_climb.hpp"
#include "rc26_decision/stair/stair_descend.hpp"

namespace rc26_decision {

StairSpeedProfile normalizeStairSpeedProfile(StairSpeedProfile profile) {
  profile.fast_speed_mps = std::abs(profile.fast_speed_mps);
  profile.slow_speed_mps = std::abs(profile.slow_speed_mps);
  profile.slow_speed_mps =
      std::min(profile.slow_speed_mps, profile.fast_speed_mps);
  profile.slowdown_duration_s = std::max(0.0, profile.slowdown_duration_s);
  return profile;
}

double sampleStairSpeedProfile(const StairSpeedProfile &profile,
                               double elapsed_s) {
  const StairSpeedProfile normalized = normalizeStairSpeedProfile(profile);
  if (normalized.slowdown_duration_s <= 0.0) {
    return normalized.slow_speed_mps;
  }
  const double ratio =
      std::clamp(std::max(0.0, elapsed_s) / normalized.slowdown_duration_s,
                 0.0, 1.0);
  return normalized.fast_speed_mps +
         (normalized.slow_speed_mps - normalized.fast_speed_mps) * ratio;
}

void normalizeStairParams(StairParams &params) {
  params.climb_front_drive_profile =
      normalizeStairSpeedProfile(params.climb_front_drive_profile);
  params.climb_rear_drive_profile =
      normalizeStairSpeedProfile(params.climb_rear_drive_profile);
  params.descend_rear_drive_speed_mps =
      std::abs(params.descend_rear_drive_speed_mps);
  params.descend_front_second_drive_profile =
      normalizeStairSpeedProfile(params.descend_front_second_drive_profile);
  params.descend_front_retract_timed_drive_speed_mps =
      std::abs(params.descend_front_retract_timed_drive_speed_mps);
  params.command_rate_hz = std::max(1.0, params.command_rate_hz);
  params.command_timeout_s = std::max(0.001, params.command_timeout_s);
  params.front_event_timeout_s = std::max(0.001, params.front_event_timeout_s);
  params.rear_event_timeout_s = std::max(0.001, params.rear_event_timeout_s);
  params.climb_front_extend_delay_s =
      std::max(0.0, params.climb_front_extend_delay_s);
  params.climb_retract_rear_extend_delay_s =
      std::max(0.0, params.climb_retract_rear_extend_delay_s);
  params.climb_rear_retract_delay_s =
      std::max(0.0, params.climb_rear_retract_delay_s);
  params.descend_rear_extend_delay_s =
      std::max(0.0, params.descend_rear_extend_delay_s);
  params.descend_retract_front_extend_delay_s =
      std::max(0.0, params.descend_retract_front_extend_delay_s);
  params.descend_front_retract_drive_duration_s =
      std::max(0.0, params.descend_front_retract_drive_duration_s);
  params.descend_front_retract_delay_s =
      std::max(0.0, params.descend_front_retract_delay_s);
  params.heading_kp = std::max(0.0, params.heading_kp);
  params.heading_max_speed_radps =
      std::max(0.0, params.heading_max_speed_radps);
  params.heading_tolerance_deg = std::max(0.0, params.heading_tolerance_deg);
  params.heading_gate_deg =
      std::max(params.heading_tolerance_deg, params.heading_gate_deg);
  params.heading_stable_ticks = std::max(1, params.heading_stable_ticks);
  params.heading_odom_timeout_s =
      std::max(0.001, params.heading_odom_timeout_s);
  params.heading_align_timeout_s =
      std::max(0.001, params.heading_align_timeout_s);
}

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
  // 读取 MCU 业务反馈 topic；0x04/0x05/0x07 激光高度突变事件从这里透传过来。
  p.feedback_topic =
      node.declare_parameter<std::string>("stair_feedback_topic",
                                          p.feedback_topic);
  // 读取上台阶前轮和后轮两个行驶段的速度规划；方向由状态机决定。
  p.climb_front_drive_profile.fast_speed_mps = node.declare_parameter<double>(
      "stair_climb_front_drive_fast_speed_mps",
      p.climb_front_drive_profile.fast_speed_mps);
  p.climb_front_drive_profile.slow_speed_mps = node.declare_parameter<double>(
      "stair_climb_front_drive_slow_speed_mps",
      p.climb_front_drive_profile.slow_speed_mps);
  p.climb_front_drive_profile.slowdown_duration_s =
      node.declare_parameter<double>(
          "stair_climb_front_drive_slowdown_duration_s",
          p.climb_front_drive_profile.slowdown_duration_s);
  p.climb_rear_drive_profile.fast_speed_mps = node.declare_parameter<double>(
      "stair_climb_rear_drive_fast_speed_mps",
      p.climb_rear_drive_profile.fast_speed_mps);
  p.climb_rear_drive_profile.slow_speed_mps = node.declare_parameter<double>(
      "stair_climb_rear_drive_slow_speed_mps",
      p.climb_rear_drive_profile.slow_speed_mps);
  p.climb_rear_drive_profile.slowdown_duration_s =
      node.declare_parameter<double>(
          "stair_climb_rear_drive_slowdown_duration_s",
          p.climb_rear_drive_profile.slowdown_duration_s);
  // 读取下台阶阶段专属速度：后轮段固定，前轮第二激光段规划。
  p.descend_rear_drive_speed_mps = node.declare_parameter<double>(
      "stair_descend_rear_drive_speed_mps",
      p.descend_rear_drive_speed_mps);
  p.descend_front_second_drive_profile.fast_speed_mps =
      node.declare_parameter<double>(
          "stair_descend_front_second_drive_fast_speed_mps",
          p.descend_front_second_drive_profile.fast_speed_mps);
  p.descend_front_second_drive_profile.slow_speed_mps =
      node.declare_parameter<double>(
          "stair_descend_front_second_drive_slow_speed_mps",
          p.descend_front_second_drive_profile.slow_speed_mps);
  p.descend_front_second_drive_profile.slowdown_duration_s =
      node.declare_parameter<double>(
          "stair_descend_front_second_drive_slowdown_duration_s",
          p.descend_front_second_drive_profile.slowdown_duration_s);
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
  // 读取下台阶触发前推杆收回前的 x 负向定时固定行驶速度绝对值。
  p.descend_front_retract_timed_drive_speed_mps =
      node.declare_parameter<double>(
          "stair_descend_front_retract_timed_drive_speed_mps",
          p.descend_front_retract_timed_drive_speed_mps);
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
  normalizeStairParams(p);

  // 把完整参数对象写入黑板；实际 BT 节点从黑板读取，避免每个节点重复声明 ROS 参数。
  blackboard->set("stair_params", p);
  // 启动日志只打印关键入口，便于确认独立台阶树使用的是哪个 topic/service。
  RCLCPP_INFO(node.get_logger(),
              "台阶动作参数已加载: cmd_vel=%s feedback=%s odom=%s climb_front_profile=%.3f->%.3fm/s %.2fs climb_rear_profile=%.3f->%.3fm/s %.2fs descend_rear_speed=%.3fm/s descend_front_second_profile=%.3f->%.3fm/s %.2fs descend_front_retract_timed=%.3fm/s %.2fs climb_delays=%.2f/%.2f/%.2fs descend_delays=%.2f/%.2f/%.2fs heading=%s kp=%.2f max=%.2frad/s tol=%.1fdeg",
              p.cmd_vel_topic.c_str(), p.feedback_topic.c_str(),
              p.odom_topic.c_str(),
              p.climb_front_drive_profile.fast_speed_mps,
              p.climb_front_drive_profile.slow_speed_mps,
              p.climb_front_drive_profile.slowdown_duration_s,
              p.climb_rear_drive_profile.fast_speed_mps,
              p.climb_rear_drive_profile.slow_speed_mps,
              p.climb_rear_drive_profile.slowdown_duration_s,
              p.descend_rear_drive_speed_mps,
              p.descend_front_second_drive_profile.fast_speed_mps,
              p.descend_front_second_drive_profile.slow_speed_mps,
              p.descend_front_second_drive_profile.slowdown_duration_s,
              p.descend_front_retract_timed_drive_speed_mps,
              p.descend_front_retract_drive_duration_s,
              p.climb_front_extend_delay_s,
              p.climb_retract_rear_extend_delay_s,
              p.climb_rear_retract_delay_s,
              p.descend_rear_extend_delay_s,
              p.descend_retract_front_extend_delay_s,
              p.descend_front_retract_delay_s,
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
