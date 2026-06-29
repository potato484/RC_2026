#pragma once

#include <string>

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

namespace rc26_decision {

struct StairSpeedProfile {
  double fast_speed_mps{0.0};
  double slow_speed_mps{0.0};
  double slowdown_duration_s{0.0};
};

struct StairParams {
  std::string cmd_vel_topic{"cmd_vel"};
  std::string send_command_service{"/mechanism/send_command"};
  std::string feedback_topic{"/mechanism/command_feedback"};
  StairSpeedProfile climb_front_drive_profile{0.40, 0.20, 2.0};
  StairSpeedProfile climb_rear_drive_profile{0.40, 0.10, 2.5};
  double descend_rear_drive_speed_mps{0.05};
  StairSpeedProfile descend_front_second_drive_profile{0.10, 0.05, 1.0};
  double command_rate_hz{20.0};
  double command_timeout_s{3.0};
  double front_event_timeout_s{10.0};
  double rear_event_timeout_s{10.0};
  double climb_front_extend_delay_s{2.0};
  double climb_retract_rear_extend_delay_s{2.5};
  double climb_rear_retract_delay_s{4.0};
  double descend_rear_extend_delay_s{2.5};
  double descend_retract_front_extend_delay_s{2.5};
  double descend_front_retract_timed_drive_speed_mps{0.10};
  double descend_front_retract_drive_duration_s{3.0};
  double descend_front_retract_delay_s{2.5};
  std::string odom_topic{"odom"};
  bool heading_hold_enable{true};
  double heading_kp{1.2};
  double heading_max_speed_radps{0.30};
  double heading_tolerance_deg{3.0};
  double heading_gate_deg{8.0};
  int heading_stable_ticks{3};
  double heading_odom_timeout_s{0.5};
  double heading_align_timeout_s{8.0};
};

StairSpeedProfile normalizeStairSpeedProfile(StairSpeedProfile profile);
double sampleStairSpeedProfile(const StairSpeedProfile &profile,
                               double elapsed_s);
void normalizeStairParams(StairParams &params);
void loadStairParams(rclcpp::Node &node, const BT::Blackboard::Ptr &blackboard);
void registerStairNodes(BT::BehaviorTreeFactory &factory);

} // namespace rc26_decision
