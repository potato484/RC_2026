#pragma once

#include <string>

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

namespace rc26_decision {

struct StairParams {
  std::string cmd_vel_topic{"cmd_vel"};
  std::string send_command_service{"/mechanism/send_command"};
  std::string feedback_topic{"/mechanism/command_feedback"};
  double drive_speed_mps{0.10};
  double command_rate_hz{20.0};
  double command_timeout_s{3.0};
  double front_event_timeout_s{10.0};
  double rear_event_timeout_s{10.0};
  double climb_front_extend_delay_s{2.0};
  double climb_retract_rear_extend_delay_s{2.5};
  double descend_rear_extend_delay_s{2.5};
  double descend_retract_front_extend_delay_s{2.5};
  double descend_front_retract_drive_speed_mps{0.025};
  double descend_front_retract_drive_duration_s{4.0};
};

void loadStairParams(rclcpp::Node &node, const BT::Blackboard::Ptr &blackboard);
void registerStairNodes(BT::BehaviorTreeFactory &factory);

} // namespace rc26_decision
