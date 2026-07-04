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
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>

#include "rc26_interfaces/msg/mechanism_transport_feedback.hpp"
#include "rc26_interfaces/srv/send_mechanism_transport_command.hpp"
#include "rc26_vision/inference/runtime/vision_inference_manager.hpp"
#include "rc26_vision/postprocess/alignment/tip_alignment.hpp"
#include "rc26_vision/shared/target/visual_target_match.hpp"

namespace rc26_decision {

struct SecondPreselectionParams {
  std::string send_command_service{"/mechanism/send_command"};
  std::string feedback_topic{"/mechanism/command_feedback"};
  double command_timeout_s{5.0};
  double done_timeout_s{5.0};
  double log_period_s{1.0};
  int start_command_id{0x11};
  int start_done_feedback_id{0x0D};
  int pickup_command_id{0x12};
  int place_kfs_command_id{0x13};

  std::string cmd_vel_topic{"cmd_vel"};
  double nav_y1_m{0.7};
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

  double search_forward_speed_mps{0.20};
  double search_timeout_s{20.0};
  std::vector<std::string> r2_target_label_prefixes{"T_"};
  std::vector<std::string> r2_target_labels;
  std::vector<std::string> r1_blocking_labels{"R_R1", "B_R1", "R1_KFS"};
  std::vector<std::string> r1_blocking_label_prefixes;
  double r1_kfs_min_score{0.50};
  double depth_min_m{0.6};
  double depth_max_m{1.2};
  int kfs_align_tolerance_px{20};
  int kfs_align_target_line_offset_px{0};
  int kfs_align_stable_frames{5};
  int kfs_align_max_jump_px{60};
  double kfs_align_kp{0.0010};
  double kfs_align_min_speed_mps{0.015};
  double kfs_align_max_speed_mps{0.06};
  double kfs_align_timeout_s{3.0};
  int kfs_align_timeout_pickup_tolerance_px{40};
  double kfs_align_heading_gate_deg{8.0};
  int kfs_lost_stop_frames{3};
  bool kfs_invert_lateral_direction{false};
  double kfs_odom_xy_kp{0.8};
  double kfs_approach_odom_tolerance_m{0.02};
  double kfs_odom_yaw_tolerance_deg{3.0};
  int kfs_odom_stable_ticks{3};
  double kfs_approach_speed_mps{0.10};
  double kfs_approach_min_speed_mps{0.03};
  int kfs_approach_x_sign{1};
  double kfs_approach_timeout_s{8.0};
  double kfs_grab_distance_m{0.50};
  double kfs_heading_kp{1.2};
  double kfs_heading_max_speed_radps{0.30};
  bool kfs_mono_distance_fallback_enable{true};
  double kfs_mono_target_width_m{0.35};
  double kfs_mono_target_height_m{0.35};
  double kfs_mono_fx_px{385.83319091796875};
  double kfs_mono_fy_px{385.83319091796875};
  int kfs_mono_min_bbox_px{40};
  double kfs_mono_max_delta_from_locked_m{0.25};
  int kfs_depth_roi_size{7};
  int kfs_depth_min_valid_count{10};
  std::vector<double> kfs_depth_bbox_sample_ratios{0.25, 0.50, 0.75};
  int kfs_depth_bbox_min_success_count{1};
  double grab_verify_timeout_s{3.0};
  int grab_verify_lost_stable_frames{3};
  double grab_verify_iou_threshold{0.30};
  double grab_settle_s{0.5};

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

double secondPreselectionKfsApproachDistance(
    double locked_depth_m, const SecondPreselectionParams &params);
bool secondPreselectionHasFrontKfs(
    const SecondPreselectionOccupancyObservation &observation);

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
  std::atomic<bool> command_error_seen_{false};
  std::atomic<bool> command_busy_seen_{false};
  std::atomic<int> command_seq_{-1};
  std::atomic<uint64_t> generation_{0};
  uint8_t command_id_{0};
  int done_feedback_id_{-1};
  double command_timeout_s_{5.0};
  double done_timeout_s_{5.0};
  std::string command_label_;
  std::string command_error_detail_;
  Phase phase_{Phase::Sending};
};

class SecondPreselectionKfsPickupAction : public BT::StatefulActionNode {
public:
  SecondPreselectionKfsPickupAction(const std::string &name,
                                    const BT::NodeConfig &config);
  ~SecondPreselectionKfsPickupAction() override;

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  using FeedbackMsg = rc26_interfaces::msg::MechanismTransportFeedback;
  using OdomMsg = nav_msgs::msg::Odometry;
  using SendCommandSrv = rc26_interfaces::srv::SendMechanismTransportCommand;

  enum class Phase {
    Search,
    VisualAlign,
    OdomApproach,
    SendingPickup,
    WaitingPickupAck,
    GrabVerify,
    Settle
  };

  struct KfsObservation {
    enum class Kind { R2, R1 };
    Kind kind{Kind::R2};
    rc26_vision::VisualTargetSnapshot target;
    rc26_vision::Detection detection;
    int offset_px{0};
    bool has_depth{false};
    bool real_depth{false};
    std::string depth_detail;
  };

  BT::NodeStatus fail(const std::string &reason);
  bool setupVision();
  void releaseVision();
  bool setupOdom();
  void releaseOdom();
  bool setupCommandIo();
  void releaseCommandIo();
  void publishStop();
  void publishTwist(double vx, double vy, double wz);
  bool odomReady() const;
  double headingAngularZ(double target_yaw_rad) const;
  rc26_vision::TipAlignmentConfig makeAlignmentConfig() const;
  std::optional<rc26_vision::TipHeadingControl> alignHeadingControl();
  std::optional<KfsObservation> findNearestKfs(bool allow_depthless_r2_align);
  void beginVisualAlign(const KfsObservation &observation);
  BT::NodeStatus tickSearch();
  BT::NodeStatus tickVisualAlign();
  BT::NodeStatus beginOdomApproach(const KfsObservation &observation);
  BT::NodeStatus tickOdomApproach();
  void beginPickupCommand();
  BT::NodeStatus tickSendingPickup();
  BT::NodeStatus tickWaitingPickupAck();
  void handleFeedback(const FeedbackMsg::SharedPtr msg);
  bool sendPickupCommand();
  BT::NodeStatus beginGrabVerify();
  BT::NodeStatus tickGrabVerify();
  BT::NodeStatus tickSettle();
  void clearRuntimeState();

  SecondPreselectionParams params_;
  rclcpp::Node *node_{nullptr};
  std::shared_ptr<rc26_vision::VisionInferenceManager> vision_;
  rclcpp::Subscription<OdomMsg>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Client<SendCommandSrv>::SharedPtr send_client_;
  rclcpp::Subscription<FeedbackMsg>::SharedPtr feedback_sub_;
  std::chrono::steady_clock::time_point phase_tp_{};
  std::chrono::steady_clock::time_point last_log_tp_{};
  std::chrono::steady_clock::time_point last_odom_tp_{};
  double odom_x_{0.0};
  double odom_y_{0.0};
  double odom_yaw_{0.0};
  bool has_odom_{false};
  bool search_yaw_captured_{false};
  double search_yaw_{0.0};
  bool align_yaw_captured_{false};
  double align_yaw_{0.0};
  bool align_waiting_verify_frame_{false};
  bool align_waiting_odom_logged_{false};
  int align_stable_count_{0};
  int align_lost_count_{0};
  int64_t align_last_sequence_{0};
  int64_t align_target_lock_sequence_{0};
  rc26_vision::TipTargetLockState align_lock_state_;
  std::optional<KfsObservation> align_last_observation_;
  rc26_vision::VisualTargetSnapshot pickup_target_;
  bool has_pickup_target_{false};
  double last_real_depth_m_{0.0};
  double approach_distance_m_{0.0};
  bool approach_started_{false};
  bool approach_start_captured_{false};
  double approach_start_x_{0.0};
  double approach_start_y_{0.0};
  double approach_start_yaw_{0.0};
  int approach_stable_ticks_{0};
  bool approach_waiting_odom_logged_{false};
  std::atomic<bool> command_response_seen_{false};
  std::atomic<bool> command_accepted_{false};
  std::atomic<bool> command_error_seen_{false};
  std::atomic<int> command_seq_{-1};
  std::atomic<uint64_t> command_generation_{0};
  std::string command_error_detail_;
  int grab_verify_lost_count_{0};
  int64_t grab_verify_last_sequence_{0};
  bool grab_verify_seen_new_frame_{false};
  bool grab_verify_visible_logged_{false};
  int grab_verify_last_logged_lost_count_{0};
  Phase phase_{Phase::Search};
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
