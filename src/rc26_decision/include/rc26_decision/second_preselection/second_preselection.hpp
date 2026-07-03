#pragma once

#include <atomic>
#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>
#include <nav_msgs/msg/odometry.hpp>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>

#include "rc26_interfaces/msg/mechanism_transport_feedback.hpp"
#include "rc26_interfaces/srv/send_mechanism_transport_command.hpp"
#include "rc26_vision/inference/runtime/vision_inference_manager.hpp"

namespace rc26_decision {

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
  double place_forward_x_m{0.7};
  double retreat_x_m{-0.7};
  double nav_timeout_s{180.0};
  double ramp_approach_x_m{0.50};
  double ramp_climb_x_m{1.50};
  double ramp_max_speed_mps{0.30};
  double ramp_min_speed_mps{0.03};
  double ramp_timeout_s{180.0};
  double after_ramp_turn_delta_rad{-1.5708};
  double after_ramp_turn_timeout_s{30.0};

  std::string vision_config_file;
  std::string model_id{"kfs_default"};
  double grid_camera_fx_px{385.6756287};
  double grid_camera_fy_px{385.1935120};
  double grid_camera_ppx_px{323.6063232};
  double grid_camera_ppy_px{241.5680695};
  double grid_left_col_width_m{0.54};
  double grid_center_col_width_m{0.58};
  double grid_right_col_width_m{0.50};
  double grid_row_pitch_m{0.54};
  double grid_middle_center_height_m{1.21};
  double grid_safe_width_m{0.40};
  double grid_safe_height_m{0.40};
  double grid_camera_height_m{0.80};
  double grid_initial_distance_m{1.80};
  double grid_initial_lateral_offset_m{0.0};
  double grid_base_y_to_grid_x_sign{-1.0};
  double grid_place_lateral_bias_m{0.0};
  std::vector<std::string> grid_label_prefixes;
  std::vector<std::string> grid_label_exact_names;
  int occupied_stable_frames{2};
  double observe_timeout_s{2.0};
  double observe_log_period_s{0.5};
  std::string odom_topic{"odom"};
  double odom_timeout_s{0.5};
};

struct SecondPreselectionGridCellProjection {
  int col{0};
  int row{0};
  cv::Rect2f roi;
  cv::Point2f center;
};

struct SecondPreselectionOccupancyObservation {
  bool occupied{false};
  int matched_detections{0};
  std::string first_label;
  uint16_t grid_occupied_mask{0};
  std::array<int, 9> grid_detection_counts{};
  std::array<SecondPreselectionGridCellProjection, 9> grid_cells{};
  std::optional<int> selected_middle_col;
  double selected_lateral_m{0.0};
};

SecondPreselectionOccupancyObservation evaluateSecondPreselectionGridOccupancy(
    const std::vector<rc26_vision::Detection> &detections,
    const SecondPreselectionParams &params, double odom_delta_x_m,
    double odom_delta_y_m);

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
  using OdomMsg = nav_msgs::msg::Odometry;

  BT::NodeStatus fail(const std::string &reason);
  bool setupVision();
  void releaseVision();
  bool setupOdom();
  void releaseOdom();
  bool odomReady() const;
  void writeObservationToBlackboard(
      const SecondPreselectionOccupancyObservation &observation);

  SecondPreselectionParams params_;
  rclcpp::Node *node_{nullptr};
  std::shared_ptr<rc26_vision::VisionInferenceManager> vision_;
  rclcpp::Subscription<OdomMsg>::SharedPtr odom_sub_;
  std::string team_{"red"};
  std::chrono::steady_clock::time_point start_tp_{};
  std::chrono::steady_clock::time_point last_log_tp_{};
  std::chrono::steady_clock::time_point last_odom_tp_{};
  double start_odom_x_{0.0};
  double start_odom_y_{0.0};
  double current_odom_x_{0.0};
  double current_odom_y_{0.0};
  bool has_odom_{false};
  bool odom_reference_ready_{false};
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
