#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>

#include "rc26_interfaces/msg/mechanism_transport_feedback.hpp"
#include "rc26_interfaces/srv/send_mechanism_transport_command.hpp"
#include "rc26_vision/inference/runtime/vision_inference_manager.hpp"

namespace rc26_decision {

struct SecondPreselectionHsvRange {
  int hue_low{0};
  int hue_high{10};
  int saturation_min{80};
  int value_min{60};
};

struct SecondPreselectionParams {
  std::string send_command_service{"/mechanism/send_command"};
  std::string feedback_topic{"/mechanism/command_feedback"};
  double command_timeout_s{5.0};
  double done_timeout_s{5.0};
  double log_period_s{1.0};
  int start_command_id{0x11};
  int start_done_feedback_id{0x0D};
  int arm_high_raise_command_id{0x12};
  int arm_high_raise_done_feedback_id{0x0F};
  int place_kfs_command_id{0x13};

  double nav_x1_m{1.8};
  double nav_y1_m{1.2};
  double nav_x2_m{2.5};
  double search_y_positive_m{0.2};
  double search_y_negative_m{-0.6};
  double place_forward_x_m{0.7};
  double retreat_x_m{-0.7};
  double nav_timeout_s{180.0};

  std::string vision_config_file;
  std::string model_id{"kfs_default"};
  int roi_x{220};
  int roi_y{150};
  int roi_width{200};
  int roi_height{180};
  int occupied_min_area_px{1200};
  int occupied_stable_frames{2};
  double observe_timeout_s{2.0};
  double observe_log_period_s{0.5};
  SecondPreselectionHsvRange red_hsv1{0, 10, 80, 60};
  SecondPreselectionHsvRange red_hsv2{170, 180, 80, 60};
  SecondPreselectionHsvRange blue_hsv1{95, 130, 80, 50};
  SecondPreselectionHsvRange blue_hsv2{95, 130, 80, 50};
};

struct SecondPreselectionHsvObservation {
  bool occupied{false};
  double best_area_px{0.0};
};

SecondPreselectionHsvObservation evaluateSecondPreselectionOccupancy(
    const cv::Mat &frame_bgr, const SecondPreselectionParams &params,
    const std::string &team);

void loadSecondPreselectionParams(rclcpp::Node &node,
                                  const BT::Blackboard::Ptr &blackboard);
void registerSecondPreselectionNodes(BT::BehaviorTreeFactory &factory);

class SecondPreselectionCommandAction : public BT::StatefulActionNode {
public:
  SecondPreselectionCommandAction(const std::string &name,
                                  const BT::NodeConfig &config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  using FeedbackMsg = rc26_interfaces::msg::MechanismTransportFeedback;
  using SendCommandSrv = rc26_interfaces::srv::SendMechanismTransportCommand;

  enum class Phase { Sending, WaitingAck, WaitingDone };

  void handleFeedback(const FeedbackMsg::SharedPtr msg);
  bool sendCommand();
  BT::NodeStatus fail(const std::string &reason);
  void resetRuntimeHandles();
  static std::string byteHex(int value);

  SecondPreselectionParams params_;
  rclcpp::Node *node_{nullptr};
  rclcpp::Subscription<FeedbackMsg>::SharedPtr feedback_sub_;
  rclcpp::Client<SendCommandSrv>::SharedPtr send_client_;
  std::chrono::steady_clock::time_point phase_tp_{};
  std::chrono::steady_clock::time_point last_log_tp_{};
  std::atomic<bool> command_response_seen_{false};
  std::atomic<bool> command_accepted_{false};
  std::atomic<bool> done_feedback_seen_{false};
  std::atomic<int> command_seq_{-1};
  std::atomic<uint64_t> generation_{0};
  uint8_t command_id_{0};
  int done_feedback_id_{-1};
  double command_timeout_s_{5.0};
  double done_timeout_s_{5.0};
  std::string command_label_;
  Phase phase_{Phase::Sending};
};

class SecondPreselectionObserveAction : public BT::StatefulActionNode {
public:
  SecondPreselectionObserveAction(const std::string &name,
                                  const BT::NodeConfig &config);
  ~SecondPreselectionObserveAction() override;

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  BT::NodeStatus fail(const std::string &reason);
  bool setupVision();
  void releaseVision();

  SecondPreselectionParams params_;
  rclcpp::Node *node_{nullptr};
  std::shared_ptr<rc26_vision::VisionInferenceManager> vision_;
  std::string team_{"red"};
  std::chrono::steady_clock::time_point start_tp_{};
  std::chrono::steady_clock::time_point last_log_tp_{};
  int occupied_stable_count_{0};
};

class SecondPreselectionNoEmptyFailureAction : public BT::SyncActionNode {
public:
  SecondPreselectionNoEmptyFailureAction(const std::string &name,
                                         const BT::NodeConfig &config);

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override;
};

} // namespace rc26_decision
