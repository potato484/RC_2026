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
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>

#include "rc26_decision/stair/stair_action_base.hpp"
#include "rc26_decision/stair/stair_area.hpp"
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
  int pickup_done_feedback_id{0x11};
  int pre_approach_lower_command_id{0x14};
  int pre_approach_lower_done_feedback_id{0x12};
  double pre_approach_lower_settle_s{0.5};
  int place_kfs_command_id{0x13};
  double post_place_retreat_x_m{-1.5};
  int post_place_front_pushrod_extend_command_id{0x08};
  double post_place_front_pushrod_extend_settle_s{15.0};
  int post_place_preload_pickup_command_id{0x15};
  int post_place_preload_pickup_done_feedback_id{0x14};
  int post_place_manual_front_laser_feedback_id{0x15};
  double post_place_manual_front_laser_timeout_s{1800.0};
  int post_place_front_pushrod_retract_command_id{0x09};
  int post_place_rear_pushrod_extend_command_id{0x0A};
  int post_place_rear_laser_feedback_id{0x05};
  int post_place_rear_pushrod_retract_command_id{0x0B};
  double post_place_final_delay_s{25.0};
  int post_place_final_command_id{0x13};

  double climb_place_forward_x_m{1.5};
  double climb_place_lateral_y_m{0.3};
  int climb_place_pre_climb_delay_msec{20000};
  int climb_place_front_pushrod_extend_command_id{0x08};
  int climb_place_manual_front_laser_feedback_id{0x15};
  int climb_place_front_pushrod_retract_command_id{0x09};
  int climb_place_rear_pushrod_extend_command_id{0x0A};
  double climb_place_rear_forward_x_m{0.6};
  double climb_place_rear_max_speed_mps{0.40};
  double climb_place_rear_min_speed_mps{0.10};
  double climb_place_rear_timeout_s{20.0};
  int climb_place_rear_pushrod_retract_command_id{0x0B};
  int climb_place_preload_pickup_command_id{0x15};
  int climb_place_preload_pickup_done_feedback_id{0x14};
  double climb_place_final_delay_s{25.0};
  int climb_place_final_command_id{0x13};

  std::string cmd_vel_topic{"cmd_vel"};
  int team_mirror_sign{1};
  double nav_y1_m{0.75};
  double post_pickup_forward_x_m{1.5};
  double nav_max_speed_mps{0.60};
  double nav_min_speed_mps{0.03};
  double total_x_target_m{4.2};
  double total_x_tolerance_m{0.03};
  double nav_timeout_s{180.0};
  double place_fixed_forward_x_m{1.8};
  double place_fixed_forward_timeout_s{30.0};
  double place_observe_timeout_s{5.0};
  double place_occupied_center_x_min_ratio{0.20};
  double place_occupied_center_x_max_ratio{0.80};
  double place_occupied_middle_y_min_ratio{0.12};
  double place_occupied_middle_y_max_ratio{0.45};
  double place_occupied_lower_y_min_ratio{0.45};
  double place_occupied_lower_y_max_ratio{1.00};
  int place_occupied_stable_frames{2};
  double place_occupied_first_lateral_m{0.56};
  double place_occupied_second_reverse_m{1.10};
  double place_occupied_lateral_max_speed_mps{0.15};
  double place_occupied_lateral_min_speed_mps{0.03};
  double place_occupied_lateral_timeout_s{15.0};
  double ramp_forward_x_m{5.0};
  double ramp_max_speed_mps{0.30};
  double ramp_min_speed_mps{0.03};
  double ramp_forward_timeout_s{5.0};
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
  double kfs_lost_servo_speed_scale{0.45};
  double kfs_align_offset_filter_alpha{0.45};
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

  bool dynamic_roi_ui_enable{false};
  std::string dynamic_roi_ui_window_name{"SecondPreselectionVision"};
  std::string odom_topic{"odom"};
  double odom_timeout_s{0.5};
};

double secondPreselectionKfsApproachDistance(
    double locked_depth_m, const SecondPreselectionParams &params);
double secondPreselectionProjectedX(double origin_x, double origin_y,
                                    double origin_yaw, double current_x,
                                    double current_y);
double secondPreselectionTotalXRemainingToDrive(
    double projected_x_m, const SecondPreselectionParams &params);
double secondPreselectionRampRemainingToDrive(
    double projected_x_m, const SecondPreselectionParams &params);
bool secondPreselectionRampTimedOut(double elapsed_s, double timeout_s);
double secondPreselectionPlaceAvoidanceDistance(
    int avoidance_stage, const SecondPreselectionParams &params);
struct SecondPreselectionLayerObservation {
  bool middle{false};
  bool lower{false};
};
SecondPreselectionLayerObservation secondPreselectionFrameLayers(
    const std::vector<rc26_vision::Detection> &detections, int frame_width,
    int frame_height, const SecondPreselectionParams &params);
bool secondPreselectionPlaceApproachTimedOut(double elapsed_s,
                                             double timeout_s);
bool secondPreselectionPlaceObserveTimedOut(double elapsed_s,
                                            double timeout_s);
bool secondPreselectionConsumeNewFrameSequence(int64_t sequence,
                                               int64_t &last_sequence);
bool secondPreselectionFrameOccupied(
    const std::vector<rc26_vision::Detection> &detections, int frame_width,
    int frame_height, const SecondPreselectionParams &params);
bool secondPreselectionFrameHasCenterKfs(
    const std::vector<rc26_vision::Detection> &detections, int frame_width,
    int frame_height, const SecondPreselectionParams &params);
bool secondPreselectionClimbPlaceReadyForFinal(double elapsed_s,
                                               double required_delay_s,
                                               bool pickup_done);

enum class SecondPreselectionOccupancyDecision { Pending, Occupied, Clear };

SecondPreselectionOccupancyDecision
secondPreselectionUpdateOccupancyStability(bool occupied, int stable_frames,
                                           int &occupied_count,
                                           int &clear_count);

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
    SendingPreApproachLower,
    WaitingPreApproachLowerAck,
    WaitingPreApproachLowerDone,
    PreApproachLowerSettle,
    OdomApproach,
    SendingPickup,
    WaitingPickupAck,
    WaitingPickupDone,
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
  bool setupUiIfNeeded();
  void releaseUi();
  void renderKfsUi(const std::string &stage,
                   const std::optional<KfsObservation> &observation =
                       std::nullopt,
                   const std::string &detail = std::string());
  void publishStop();
  void publishTwist(double vx, double vy, double wz);
  bool odomReady() const;
  double headingAngularZ(double target_yaw_rad) const;
  rc26_vision::TipAlignmentConfig makeAlignmentConfig() const;
  std::optional<rc26_vision::TipHeadingControl> alignHeadingControl();
  std::optional<KfsObservation> findNearestKfs(bool allow_depthless_r2_align);
  KfsObservation applyAlignmentObservationFilter(const KfsObservation &observation);
  void beginVisualAlign(const KfsObservation &observation);
  BT::NodeStatus tickSearch();
  BT::NodeStatus tickVisualAlign();
  void beginPreApproachLowerCommand(const KfsObservation &observation);
  BT::NodeStatus tickSendingPreApproachLower();
  BT::NodeStatus tickWaitingPreApproachLowerAck();
  BT::NodeStatus tickWaitingPreApproachLowerDone();
  BT::NodeStatus tickPreApproachLowerSettle();
  BT::NodeStatus beginOdomApproach(const KfsObservation &observation);
  BT::NodeStatus tickOdomApproach();
  void beginMechanismCommand(int command_id, int done_feedback_id,
                             const std::string &label);
  void beginPickupCommand();
  BT::NodeStatus tickSendingPickup();
  BT::NodeStatus tickWaitingPickupAck();
  BT::NodeStatus tickWaitingPickupDone();
  void handleFeedback(const FeedbackMsg::SharedPtr msg);
  bool sendActiveCommand();
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
  bool search_origin_captured_{false};
  double search_origin_x_{0.0};
  double search_origin_y_{0.0};
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
  bool align_filtered_offset_valid_{false};
  double align_filtered_offset_px_{0.0};
  rc26_vision::VisualTargetSnapshot pickup_target_;
  bool has_pickup_target_{false};
  std::optional<KfsObservation> pre_approach_observation_;
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
  std::atomic<bool> command_done_feedback_seen_{false};
  std::atomic<bool> command_error_seen_{false};
  std::atomic<bool> command_busy_seen_{false};
  std::atomic<int> command_seq_{-1};
  std::atomic<uint64_t> command_generation_{0};
  std::string command_error_detail_;
  uint8_t active_command_id_{0};
  int active_done_feedback_id_{-1};
  std::string active_command_label_;
  int grab_verify_lost_count_{0};
  int64_t grab_verify_last_sequence_{0};
  bool grab_verify_seen_new_frame_{false};
  bool grab_verify_visible_logged_{false};
  int grab_verify_last_logged_lost_count_{0};
  bool ui_window_active_{false};
  bool ui_disabled_after_error_{false};
  Phase phase_{Phase::Search};
};

class SecondPreselectionDriveToTotalXAction : public BT::StatefulActionNode {
public:
  SecondPreselectionDriveToTotalXAction(const std::string &name,
                                        const BT::NodeConfig &config);
  ~SecondPreselectionDriveToTotalXAction() override;

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  using OdomMsg = nav_msgs::msg::Odometry;

  BT::NodeStatus fail(const std::string &reason);
  void publishStop();
  void clearRuntimeState();
  bool odomReady() const;

  SecondPreselectionParams params_;
  rclcpp::Node *node_{nullptr};
  rclcpp::Subscription<OdomMsg>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  std::chrono::steady_clock::time_point start_tp_{};
  std::chrono::steady_clock::time_point last_odom_tp_{};
  double origin_x_{0.0};
  double origin_y_{0.0};
  double origin_yaw_{0.0};
  double odom_x_{0.0};
  double odom_y_{0.0};
  double odom_yaw_{0.0};
  bool has_odom_{false};
  int stable_ticks_{0};
};

class SecondPreselectionRampForwardAction : public BT::StatefulActionNode {
public:
  SecondPreselectionRampForwardAction(const std::string &name,
                                      const BT::NodeConfig &config);
  ~SecondPreselectionRampForwardAction() override;

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  using OdomMsg = nav_msgs::msg::Odometry;

  void publishStop();
  void clearRuntimeState();
  bool odomReady() const;
  void publishTwist(double vx, double wz);
  double headingAngularZ() const;
  BT::NodeStatus finishSuccess(const std::string &reason);

  SecondPreselectionParams params_;
  rclcpp::Node *node_{nullptr};
  rclcpp::Subscription<OdomMsg>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  std::chrono::steady_clock::time_point start_tp_{};
  std::chrono::steady_clock::time_point last_odom_tp_{};
  double start_x_{0.0};
  double start_y_{0.0};
  double start_yaw_{0.0};
  double odom_x_{0.0};
  double odom_y_{0.0};
  double odom_yaw_{0.0};
  bool has_odom_{false};
  bool start_captured_{false};
};

class SecondPreselectionKfsPlacePrepareAction
    : public BT::StatefulActionNode {
public:
  SecondPreselectionKfsPlacePrepareAction(const std::string &name,
                                          const BT::NodeConfig &config);
  ~SecondPreselectionKfsPlacePrepareAction() override;

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  using OdomMsg = nav_msgs::msg::Odometry;
  using FrameSnapshot = rc26_vision::VisionInferenceManager::FrameSnapshot;

  enum class Phase {
    ObserveInitial,
    LateralFirst,
    ObserveAfterFirst,
    LateralSecond,
    ObserveAfterSecond,
    AlignTarget
  };

  struct KfsObservation {
    rc26_vision::VisualTargetSnapshot target;
    rc26_vision::Detection detection;
    int offset_px{0};
    std::string depth_detail;
  };

  BT::NodeStatus fail(const std::string &reason);
  bool setupVision();
  void releaseVision();
  bool setupOdom();
  void releaseOdom();
  bool odomReady() const;
  bool setupUiIfNeeded();
  void releaseUi();
  void renderUi(const std::string &stage,
                const std::optional<KfsObservation> &observation =
                    std::nullopt,
                const std::string &detail = std::string());
  void publishStop();
  void publishTwist(double vx, double vy, double wz);
  rc26_vision::TipAlignmentConfig makeAlignmentConfig() const;
  std::optional<rc26_vision::TipHeadingControl> alignHeadingControl();
  bool latestSnapshot(FrameSnapshot &snapshot) const;
  std::optional<KfsObservation> findNearestKfs(const FrameSnapshot &snapshot);
  KfsObservation
  applyAlignmentObservationFilter(const KfsObservation &observation);
  BT::NodeStatus tickObserveCheckpoint(Phase lateral_phase,
                                       Phase clear_phase,
                                       bool final_checkpoint);
  void beginLateralMove(double distance_m, Phase phase,
                        Phase next_observe_phase);
  BT::NodeStatus tickLateralMove();
  BT::NodeStatus tickAlignTarget();
  BT::NodeStatus finishAlignedPlace(const KfsObservation &observation);
  BT::NodeStatus finishFixedForwardPlace(const std::string &reason);
  void resetObservationStability();
  void latchLatestObservationSequence();
  void resetAlignmentState();
  double headingAngularZ(double target_yaw_rad) const;
  void clearRuntimeState();

  SecondPreselectionParams params_;
  rclcpp::Node *node_{nullptr};
  std::shared_ptr<rc26_vision::VisionInferenceManager> vision_;
  rclcpp::Subscription<OdomMsg>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  std::chrono::steady_clock::time_point start_tp_{};
  std::chrono::steady_clock::time_point phase_tp_{};
  std::chrono::steady_clock::time_point last_log_tp_{};
  std::chrono::steady_clock::time_point last_odom_tp_{};
  double odom_x_{0.0};
  double odom_y_{0.0};
  double odom_yaw_{0.0};
  bool has_odom_{false};
  bool align_yaw_captured_{false};
  double align_yaw_{0.0};
  bool waiting_odom_logged_{false};
  int align_stable_count_{0};
  int align_lost_count_{0};
  int64_t align_last_sequence_{0};
  int64_t align_search_last_sequence_{0};
  rc26_vision::TipTargetLockState align_lock_state_;
  std::optional<KfsObservation> align_last_observation_;
  bool align_filtered_offset_valid_{false};
  double align_filtered_offset_px_{0.0};
  int occupied_stable_count_{0};
  int clear_stable_count_{0};
  int64_t occupancy_last_sequence_{0};
  double lateral_distance_m_{0.0};
  double lateral_start_x_{0.0};
  double lateral_start_y_{0.0};
  double lateral_start_yaw_{0.0};
  bool lateral_start_captured_{false};
  int lateral_stable_ticks_{0};
  Phase next_observe_phase_{Phase::ObserveInitial};
  Phase phase_{Phase::ObserveInitial};
  bool ui_window_active_{false};
  bool ui_disabled_after_error_{false};
};

class SecondPreselectionPlaceApproachAction : public BT::StatefulActionNode {
public:
  SecondPreselectionPlaceApproachAction(const std::string &name,
                                        const BT::NodeConfig &config);
  ~SecondPreselectionPlaceApproachAction() override;

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  using OdomMsg = nav_msgs::msg::Odometry;

  BT::NodeStatus fail(const std::string &reason);
  BT::NodeStatus finish(const std::string &result, bool timed_out);
  void publishStop();
  void clearRuntimeState();
  bool odomReady() const;

  SecondPreselectionParams params_;
  rclcpp::Node *node_{nullptr};
  rclcpp::Subscription<OdomMsg>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  std::chrono::steady_clock::time_point start_tp_{};
  std::chrono::steady_clock::time_point last_odom_tp_{};
  double distance_m_{0.0};
  double start_x_{0.0};
  double start_y_{0.0};
  double start_yaw_{0.0};
  double odom_x_{0.0};
  double odom_y_{0.0};
  double odom_yaw_{0.0};
  bool has_odom_{false};
  bool start_captured_{false};
  int stable_ticks_{0};
};

class SecondPreselectionClimbFrontStageAction : public StairActionBase {
public:
  SecondPreselectionClimbFrontStageAction(const std::string &name,
                                          const BT::NodeConfig &config);

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  using FeedbackMsg = rc26_interfaces::msg::MechanismTransportFeedback;

  enum class Phase {
    HeadingAlign,
    SendFrontExtend,
    HoldAfterFrontExtend,
    DriveUntilManualFrontLaser,
    SendFrontRetractAndRearExtend,
    HoldAfterFrontRetractAndRearExtend,
    Done
  };

  BT::NodeStatus fail(const char *reason);
  void beginManualFrontLaserDrive();
  void clearManualFeedbackRuntime();

  SecondPreselectionParams second_params_;
  rclcpp::Subscription<FeedbackMsg>::SharedPtr manual_feedback_sub_;
  std::chrono::steady_clock::time_point manual_event_tp_{};
  std::atomic<uint64_t> manual_front_laser_count_{0};
  uint64_t manual_front_laser_baseline_{0};
  Phase phase_{Phase::Done};
};

class SecondPreselectionRearRetractPickupPlaceAction
    : public BT::StatefulActionNode {
public:
  SecondPreselectionRearRetractPickupPlaceAction(
      const std::string &name, const BT::NodeConfig &config);

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  using FeedbackMsg = rc26_interfaces::msg::MechanismTransportFeedback;
  using SendCommandSrv = rc26_interfaces::srv::SendMechanismTransportCommand;

  enum class Phase {
    SendRearRetractAndPickup,
    WaitRearRetractAndPickupAck,
    WaitDelayAndPickupDone,
    SendFinalPlace,
    WaitFinalPlaceAck,
    Done
  };

  struct CommandRuntime {
    uint8_t command_id{0};
    int done_feedback_id{-1};
    std::string label;
    bool sent{false};
    std::atomic<bool> response_seen{false};
    std::atomic<bool> accepted{false};
    std::atomic<bool> rejected{false};
    std::atomic<bool> done_seen{false};
    std::atomic<int> seq{-1};
  };

  BT::NodeStatus fail(const std::string &reason);
  void clearRuntimeState();
  void resetCommand(CommandRuntime &command, int command_id,
                    int done_feedback_id, const std::string &label);
  bool sendCommand(CommandRuntime &command);
  bool commandAcked(const CommandRuntime &command) const;
  bool commandRejected(const CommandRuntime &command) const;
  bool commandDone(const CommandRuntime &command) const;
  void handleFeedback(const FeedbackMsg::SharedPtr msg);
  void publishStop();
  double phaseElapsed() const;

  SecondPreselectionParams params_;
  rclcpp::Node *node_{nullptr};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Client<SendCommandSrv>::SharedPtr send_client_;
  rclcpp::Subscription<FeedbackMsg>::SharedPtr feedback_sub_;
  std::chrono::steady_clock::time_point phase_tp_{};
  std::chrono::steady_clock::time_point final_gate_tp_{};
  std::chrono::steady_clock::time_point last_log_tp_{};
  CommandRuntime rear_retract_command_;
  CommandRuntime pickup_command_;
  CommandRuntime final_place_command_;
  std::atomic<bool> command_error_seen_{false};
  std::atomic<bool> command_busy_seen_{false};
  std::atomic<uint64_t> command_generation_{0};
  std::string command_error_detail_;
  Phase phase_{Phase::Done};
};

class SecondPreselectionPostPlaceClimbAction : public BT::StatefulActionNode {
public:
  SecondPreselectionPostPlaceClimbAction(const std::string &name,
                                         const BT::NodeConfig &config);

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  using FeedbackMsg = rc26_interfaces::msg::MechanismTransportFeedback;
  using OdomMsg = nav_msgs::msg::Odometry;
  using SendCommandSrv = rc26_interfaces::srv::SendMechanismTransportCommand;

  enum class Phase {
    SendFrontExtendAndPreloadPickup,
    WaitFrontExtendSettleAndPreloadDone,
    WaitManualFrontLaser,
    SendFrontRetractAndRearExtend,
    WaitFrontRetractAndRearExtendAck,
    HoldAfterFrontRetractAndRearExtend,
    DriveUntilRearEvent,
    SendRearRetract,
    WaitRearRetractAck,
    HoldAfterRearRetract,
    FinalDelay,
    SendFinalPlace,
    WaitFinalPlaceAck,
    Done
  };

  struct CommandRuntime {
    uint8_t command_id{0};
    int done_feedback_id{-1};
    std::string label;
    bool sent{false};
    std::atomic<bool> response_seen{false};
    std::atomic<bool> accepted{false};
    std::atomic<bool> rejected{false};
    std::atomic<bool> done_seen{false};
    std::atomic<int> seq{-1};
  };

  BT::NodeStatus fail(const std::string &reason);
  void clearRuntimeState();
  void resetCommand(CommandRuntime &command, int command_id,
                    int done_feedback_id, const std::string &label);
  bool sendCommand(CommandRuntime &command);
  bool commandAcked(const CommandRuntime &command) const;
  bool commandRejected(const CommandRuntime &command) const;
  bool commandDone(const CommandRuntime &command) const;
  void handleFeedback(const FeedbackMsg::SharedPtr msg);
  void publishStop();
  void publishDrive(double vx_mps);
  bool headingOdomReady() const;
  double headingError() const;
  double headingAngularZ() const;
  bool tickDriveYawGate(const char *label);
  double rearDriveSpeed() const;
  void beginManualFrontLaserWait();
  void beginFrontRetractRearExtend();
  void beginRearDrive();
  void beginRearRetract();
  void beginFinalDelay();
  void beginFinalPlace();
  double phaseElapsed() const;
  static std::string byteHex(int value);

  SecondPreselectionParams params_;
  StairParams stair_params_;
  rclcpp::Node *node_{nullptr};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<OdomMsg>::SharedPtr odom_sub_;
  rclcpp::Client<SendCommandSrv>::SharedPtr send_client_;
  rclcpp::Subscription<FeedbackMsg>::SharedPtr feedback_sub_;
  std::chrono::steady_clock::time_point phase_tp_{};
  std::chrono::steady_clock::time_point last_log_tp_{};
  std::chrono::steady_clock::time_point front_extend_ack_tp_{};
  std::chrono::steady_clock::time_point rear_drive_profile_tp_{};
  std::chrono::steady_clock::time_point last_odom_tp_{};
  CommandRuntime command_a_;
  CommandRuntime command_b_;
  std::atomic<bool> command_error_seen_{false};
  std::atomic<bool> command_busy_seen_{false};
  std::atomic<uint64_t> command_generation_{0};
  std::string command_error_detail_;
  std::atomic<uint64_t> manual_front_laser_count_{0};
  std::atomic<uint64_t> rear_laser_count_{0};
  uint64_t manual_front_laser_baseline_{0};
  uint64_t rear_laser_baseline_{0};
  double current_yaw_rad_{0.0};
  double target_yaw_rad_{0.0};
  bool has_odom_{false};
  bool target_yaw_set_{false};
  bool front_extend_settle_started_{false};
  Phase phase_{Phase::Done};
};

} // namespace rc26_decision
