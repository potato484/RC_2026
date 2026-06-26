#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include "rc26_interfaces/msg/mechanism_transport_feedback.hpp"
#include "rc26_interfaces/srv/send_mechanism_transport_command.hpp"
#include "rc26_vision/inference/runtime/vision_inference_manager.hpp"

namespace rc26_decision {

struct KfsParams {
  std::string vision_config_file;
  std::string model_id{"kfs_default"};
  std::vector<std::string> blocking_labels{"R_R1", "B_R1"};
  double depth_min_m{0.6};
  double depth_max_m{1.2};
  int blocking_seen_stable_frames{1};
  int blocking_lost_stable_frames{5};
  double blocking_initial_detection_timeout_s{2.0};
  double blocking_wait_timeout_s{60.0};

  std::string cmd_vel_topic{"cmd_vel"};
  std::string send_command_service{"/mechanism/send_command"};
  std::string feedback_topic{"/mechanism/command_feedback"};
  double command_timeout_s{3.0};
  int arm_raise_command_id{0x04};
  int arm_lower_command_id{0x05};
  int arm_raise_done_feedback_id{0x02};
  int arm_lower_done_feedback_id{0x03};
  int grab_kfs_up_command_id{0x03};
  int grab_kfs_down_command_id{0x02};

  std::string odom_topic{"odom"};
  bool heading_hold_enable{true};
  double heading_kp{1.2};
  double heading_max_speed_radps{0.30};
  double heading_tolerance_deg{3.0};
  double heading_gate_deg{8.0};
  double heading_odom_timeout_s{0.5};
};

void loadKfsParams(rclcpp::Node &node, const BT::Blackboard::Ptr &blackboard);
void registerKfsNodes(BT::BehaviorTreeFactory &factory);

class KfsStairPickupAction : public BT::StatefulActionNode {
public:
  KfsStairPickupAction(const std::string &name, const BT::NodeConfig &config);
  ~KfsStairPickupAction() override;

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  using TwistMsg = geometry_msgs::msg::Twist;
  using OdomMsg = nav_msgs::msg::Odometry;
  using FeedbackMsg = rc26_interfaces::msg::MechanismTransportFeedback;
  using SendCommandSrv = rc26_interfaces::srv::SendMechanismTransportCommand;

  enum class Direction { Climb, Descend };
  enum class Phase { SendingPrep, WaitingPrepDone, DetectingInitial, WaitingBlockingGone, Done };

  bool setupRuntime();
  bool setupVision();
  void releaseRuntime();
  bool parseDirection();
  void normalizeParams();
  void publishStop();
  void setOutcome(const std::string &outcome, const std::string &error = "");
  BT::NodeStatus fail(const std::string &error);

  void sendPrepCommand();
  BT::NodeStatus tickPrepCommand();
  bool prepFeedbackReceived() const;

  bool isBlockingTargetPresent(double &distance_m,
                               std::string &label,
                               bool &observation_valid,
                               int64_t &observation_stamp_ns);
  bool detectionTimedOut() const;
  bool waitTimedOut() const;
  double elapsedSinceStart() const;

  void setupOdomSubscription();
  void handleOdom(const OdomMsg::SharedPtr msg);
  double headingAngularZ() const;
  static double normalizeAngle(double angle_rad);
  static double yawFromQuaternion(const geometry_msgs::msg::Quaternion &q);

  rclcpp::Node *node_{nullptr};
  KfsParams params_;
  Direction direction_{Direction::Climb};
  Phase phase_{Phase::Done};
  std::string direction_text_;
  std::string prep_label_;
  uint8_t prep_command_id_{0};
  uint8_t prep_done_feedback_id_{0};

  rclcpp::Publisher<TwistMsg>::SharedPtr cmd_pub_;
  rclcpp::Client<SendCommandSrv>::SharedPtr send_client_;
  rclcpp::Subscription<FeedbackMsg>::SharedPtr feedback_sub_;
  rclcpp::Subscription<OdomMsg>::SharedPtr odom_sub_;
  std::shared_ptr<rc26_vision::VisionInferenceManager> vision_;

  std::vector<int> blocking_class_ids_;
  std::chrono::steady_clock::time_point start_tp_{};
  std::chrono::steady_clock::time_point prep_send_tp_{};
  std::chrono::steady_clock::time_point last_odom_tp_{};
  std::atomic<bool> prep_response_seen_{false};
  std::atomic<bool> prep_accepted_{false};
  std::atomic<bool> prep_done_seen_{false};
  std::atomic<uint64_t> command_generation_{0};
  std::atomic<int> prep_seq_{0};
  std::atomic<int> latest_prep_done_seq_{-1};

  bool has_odom_yaw_{false};
  bool heading_target_set_{false};
  double current_yaw_rad_{0.0};
  double heading_target_yaw_rad_{0.0};

  int seen_stable_count_{0};
  int lost_stable_count_{0};
  int64_t last_observation_sequence_{0};
  bool ever_blocking_seen_{false};
};

} // namespace rc26_decision
