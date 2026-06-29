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

#include "rc26_decision/mf/grid_center.hpp"
#include "rc26_decision/stair/stair_area.hpp"
#include "rc26_interfaces/msg/mechanism_transport_feedback.hpp"
#include "rc26_interfaces/srv/send_mechanism_transport_command.hpp"
#include "rc26_vision/inference/runtime/vision_inference_manager.hpp"
#include "rc26_vision/shared/target/visual_target_match.hpp"

namespace rc26_decision {

enum class MfPreselectionPickupSource { None, Stair1, Stair2, Stair3 };

using MfPreselectionTargetSnapshot = rc26_vision::VisualTargetSnapshot;

struct MfPreselectionParams {
  std::string vision_config_file;
  std::string model_id{"kfs_default"};
  std::vector<std::string> r2_target_label_prefixes{"T_"};
  std::vector<std::string> r2_target_labels;
  std::vector<std::string> r1_blocking_labels{"R_R1", "B_R1"};
  std::vector<std::string> r1_blocking_label_prefixes;
  std::vector<std::string> fake_label_prefixes{"F_"};
  std::vector<std::string> fake_labels;
  double depth_min_m{0.6};
  double depth_max_m{1.2};
  int detect_seen_stable_frames{1};
  int detect_lost_stable_frames{5};
  double entry_detect_timeout_s{2.0};
  double scan_detect_timeout_s{2.0};

  int kfs_align_tolerance_px{20};
  int kfs_align_stable_frames{5};
  double kfs_align_kp{0.0010};
  double kfs_align_min_speed_mps{0.015};
  double kfs_align_max_speed_mps{0.06};
  double kfs_align_target_offset_px{0.0};
  double kfs_align_px_to_m{0.0005};
  double kfs_align_timeout_s{3.0};
  int kfs_lost_stop_frames{3};
  bool kfs_invert_lateral_direction{false};
  double kfs_approach_speed_mps{0.10};
  int kfs_approach_x_sign{1};
  double kfs_approach_timeout_s{8.0};
  double kfs_grab_distance_m{0.50};

  int max_pickup_count{2};
  double grab_settle_s{0.5};
  double grab_verify_timeout_s{3.0};
  int grab_verify_lost_stable_frames{3};
  double grab_verify_iou_threshold{0.30};
  int grab_kfs_up_command_id{0x03};
  int grab_kfs_down_command_id{0x02};
  int entry_grab_kfs_up_command_id{0x0F};
  int entry_grab_kfs_up_done_feedback_id{0x0B};

  std::string cmd_vel_topic{"cmd_vel"};
  std::string odom_topic{"odom"};
  std::string send_command_service{"/mechanism/send_command"};
  std::string feedback_topic{"/mechanism/command_feedback"};
  double command_timeout_s{3.0};
  int arm_high_raise_command_id{0x0D};
  int arm_high_raise_done_feedback_id{0x09};
  int arm_raise_command_id{0x04};
  int arm_lower_command_id{0x05};
  int arm_raise_done_feedback_id{0x02};
  int arm_lower_done_feedback_id{0x03};
  int second_arm_lower_command_id{0x0E};
  int second_arm_lower_done_feedback_id{0x0A};

  double entry_probe_left_distance_m{1.2};
  double entry_probe_right_sweep_distance_m{2.4};
  double entry_probe_return_distance_m{1.2};
  double lateral_probe_speed_mps{0.35};
  double move_tolerance_m{0.03};
  double move_timeout_s{12.0};
  double direct_exit_drive_distance_m{4.8};
  double direct_exit_drive_speed_mps{0.25};

  double exit_yaw_rad{0.0};
  double stair1_direction_yaw_rad{1.5708};
  double stair3_direction_yaw_rad{-1.5708};
  double row_scan_left_yaw_delta_rad{1.5708};
  double row_scan_back_yaw_delta_rad{3.1416};
  double row4_exit_turn_yaw_rad{-1.5708};
  double turn_kp{1.2};
  double turn_max_speed_radps{0.50};
  double turn_tolerance_deg{3.0};
  int turn_stable_ticks{3};
  double turn_timeout_s{16.0};
  double odom_timeout_s{0.5};
  double heading_kp{1.2};
  double heading_max_speed_radps{0.30};

  double path_r1_lost_wait_timeout_s{0.0};
  double final_exit_center_offset_m{1.2};
};

struct MfPreselectionLogicResult {
  static bool labelMatches(const std::string &label,
                           const std::vector<std::string> &exact_labels,
                           const std::vector<std::string> &prefixes);
  static bool canPickup(int pickup_count, int max_pickup_count);
  static double kfsAlignVy(int offset_px, const MfPreselectionParams &params);
  static int kfsAlignOffsetPx(double bbox_center_x, double image_width_px,
                              const MfPreselectionParams &params);
  static double kfsAlignOpenLoopDistance(int offset_px,
                                         const MfPreselectionParams &params);
  static double kfsOpenLoopDistance(double locked_depth_m,
                                    double grab_distance_m);
  static double kfsOpenLoopDuration(double distance_m, double speed_mps);
  static double fakeAvoidanceYaw(MfPreselectionPickupSource source,
                                 const MfPreselectionParams &params);
  static uint8_t grabCommandForHighSide(bool high_side,
                                        const MfPreselectionParams &params);
  static uint8_t grabCommandForPickup(bool high_side,
                                      MfPreselectionPickupSource source,
                                      bool entry_high_protocol,
                                      const MfPreselectionParams &params);
  static int grabDoneFeedbackForPickup(bool high_side,
                                       MfPreselectionPickupSource source,
                                       bool entry_high_protocol,
                                       const MfPreselectionParams &params);
  static double bboxIou(const MfPreselectionTargetSnapshot &a,
                        const MfPreselectionTargetSnapshot &b);
  static bool isSameVisualTarget(const MfPreselectionTargetSnapshot &reference,
                                 const MfPreselectionTargetSnapshot &candidate,
                                 double iou_threshold);
  static bool isIgnoredTarget(
      const MfPreselectionTargetSnapshot &candidate,
      const std::vector<MfPreselectionTargetSnapshot> &ignored_targets,
      double iou_threshold);
  static std::optional<int>
  fakeAvoidanceTargetGrid(int current_grid,
                          MfPreselectionPickupSource source);
  static MfPreselectionPickupSource
  entryPickupSourceForLateralOffset(double lateral_offset_m,
                                    double tolerance_m);
  static bool entryReturnToCenterCommand(double lateral_offset_m,
                                         double tolerance_m,
                                         double speed_mps,
                                         double &vy, double &distance_m);
  static bool finalExitCenterTarget(double current_center_x,
                                    double current_center_y,
                                    double exit_heading_yaw_rad,
                                    double offset_m, double &target_x,
                                    double &target_y);
};

void loadMfPreselectionParams(rclcpp::Node &node,
                              const BT::Blackboard::Ptr &blackboard);
void registerMfPreselectionNodes(BT::BehaviorTreeFactory &factory);

class MfPreselectionFlowAction : public BT::StatefulActionNode {
public:
  MfPreselectionFlowAction(const std::string &name,
                           const BT::NodeConfig &config);
  ~MfPreselectionFlowAction() override;

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  using TwistMsg = geometry_msgs::msg::Twist;
  using OdomMsg = nav_msgs::msg::Odometry;
  using FeedbackMsg = rc26_interfaces::msg::MechanismTransportFeedback;
  using SendCommandSrv = rc26_interfaces::srv::SendMechanismTransportCommand;

  enum class Phase {
    EntryDetectStair2,
    EntryHighRaise,
    EntryMoveLeft,
    EntryDetectStair1,
    EntryReturnFromStair1,
    EntryMoveRightToStair3,
    EntryDetectStair3,
    EntryReturnFromStair3,
    EntryReturnToCenterAfterInterruptedPickup,
    EntryResumeInterruptedProbeMove,
    EntryPrepareClimb,
    EntryClimb,
    AfterEntry,
    RowFrontDetect,
    RowScanTurnLeft,
    RowScanDetectLeft,
    RowScanTurnBack,
    RowScanDetectBack,
    RowAlignExit,
    FakeAvoidTurn,
    FakeAvoidArmRaise,
    FakeAvoidClimb,
    FakeAvoidAlignExit,
    TransitionTurn,
    TransitionArmAdjust,
    DetectionArmAdjust,
    TransitionObserve,
    TransitionStair,
    Row4ForcedTurn,
    Row4DetectFake,
    Row4FakeTurnBack,
    FinalExitYawAlign,
    Row4DirectDescendPrep,
    Row4DirectDescend,
    DirectExitDrive,
    FinalStop,
    KfsVisualAlign,
    KfsSecondArmLower,
    KfsOpenLoopApproach,
    MechanismCommand,
    GrabVerify,
    MoveRelative,
    TurnYaw,
    ZeroHold,
    StairPrimitive,
    CenterAlign,
    Done
  };

  enum class DetectMode { Entry2, Stair1, Stair3, RowFront, Scan, Row4Fake, TransitionObserve };
  enum class StairMode { Climb, Descend };
  enum class StairCenterPolicy {
    None,
    EntryGrid2Reference,
    TransitionTargetGrid,
    FakeAvoidTargetGrid,
    FinalExitVirtual
  };
  enum class StairPhase {
    ClimbSendFrontExtend,
    ClimbHoldAfterFrontExtend,
    ClimbDriveUntilFrontFirstEvent,
    ClimbSendFrontRetractAndRearExtend,
    ClimbHoldAfterFrontRetractAndRearExtend,
    ClimbDriveUntilRearEvent,
    ClimbSendRearRetract,
    ClimbHoldAfterRearRetract,
    DescendDriveUntilRearEvent,
    DescendSendRearExtend,
    DescendHoldAfterRearExtend,
    DescendDriveUntilFrontSecondEvent,
    DescendSendRearRetractAndFrontExtend,
    DescendHoldAfterRearRetractAndFrontExtend,
    DescendTimedDriveBeforeFrontRetract,
    DescendSendFrontRetract,
    DescendHoldAfterFrontRetract,
    Complete
  };
  enum class WheelEvent { FrontFirst, FrontSecond, Rear };

  struct KfsVisualObservation {
    MfPreselectionTargetSnapshot target;
    int offset_px{0};
    bool has_depth{false};
  };

  bool setupRuntime();
  bool setupVision();
  void releaseRuntime();
  void normalizeParams();
  BT::NodeStatus fail(const std::string &reason);
  void publishStop();
  void publishTwist(double vx, double vy, double wz);
  void publishCenterStop();
  void publishCenterTwist(double vx, double vy, double wz);
  bool odomReady() const;
  void handleOdom(const OdomMsg::SharedPtr msg);
  void handleCenterOdom(const OdomMsg::SharedPtr msg);
  static double yawFromQuaternion(const geometry_msgs::msg::Quaternion &q);
  static double normalizeAngle(double angle_rad);
  double headingAngularZ(double target_yaw_rad) const;

  std::optional<MfPreselectionTargetSnapshot> findR2Target();
  std::optional<MfPreselectionTargetSnapshot> findR2TargetLabelOnly();
  std::optional<MfPreselectionTargetSnapshot> findR1BlockingTarget();
  std::optional<MfPreselectionTargetSnapshot> findFakeTarget();
  std::optional<MfPreselectionTargetSnapshot>
  findTarget(const std::vector<std::string> &exact,
             const std::vector<std::string> &prefixes, bool skip_ignored_r2);
  std::optional<int64_t> latestVisionSequence() const;
  bool canPickup() const;
  Phase detectionMissNextPhase() const;
  void rememberPickupSource(MfPreselectionPickupSource source);
  void writeBlackboardState(const std::string &state);
  static const char *detectModeText(DetectMode mode);
  static const char *stairModeText(StairMode mode);
  static const char *wheelEventText(WheelEvent event);
  static const char *phaseText(Phase phase);

  void beginDetection(DetectMode mode, double timeout_s);
  void beginPreparedDetection(DetectMode mode, double timeout_s,
                              Phase detection_phase);
  bool resolveDetectionHighSide(DetectMode mode, Phase detection_phase,
                                bool &high_side, int &target_grid,
                                int &height_delta) const;
  BT::NodeStatus tickDetection();
  void resetDetectionCounters();

  void beginMechanismCommand(uint8_t command_id, std::string label,
                             int done_feedback_id, Phase next_phase,
                             std::string failure_reason);
  BT::NodeStatus tickMechanismCommand();
  void beginCommandPair(uint8_t first_id, std::string first_label,
                        uint8_t second_id, std::string second_label,
                        Phase next_phase, std::string failure_reason);
  BT::NodeStatus tickCommandPair();

  void beginMoveRelative(double vx, double vy, double distance_m,
                         Phase next_phase, std::string label);
  BT::NodeStatus tickMoveRelative();
  void continueAfterMoveRelative(Phase next_phase);
  bool isEntryInterruptibleMove() const;
  std::optional<double> entryMoveTargetOffset() const;
  bool captureEntryLateralReferenceIfNeeded();
  double currentEntryLateralOffset() const;
  bool maybeInterruptEntryMoveForKfs();
  BT::NodeStatus beginEntryReturnToCenterAfterInterruptedPickup();
  BT::NodeStatus resumeInterruptedEntryMove();
  void beginDirectExitDrive();
  BT::NodeStatus tickDirectExitDrive();
  bool guardPathObstacles();
  void clearPathR1Wait();

  void beginTurnYaw(double target_yaw_rad, Phase next_phase, std::string label);
  BT::NodeStatus tickTurnYaw();
  void beginZeroHold(double duration_s, Phase next_phase, std::string label);
  BT::NodeStatus tickZeroHold();

  bool prepareTransitionTo(int target_grid);
  BT::NodeStatus startTransitionTo(int target_grid);
  bool continueAfterTransition();
  Phase phaseAfterTransition() const;
  double transitionYaw(int from_grid, int target_grid, int height_delta) const;

  void beginStair(StairMode mode, Phase next_phase, std::string label,
                  StairCenterPolicy center_policy = StairCenterPolicy::None);
  BT::NodeStatus tickStair();
  void beginWheelEvent(WheelEvent event, double timeout_s, std::string label);
  bool wheelEventReceived() const;
  BT::NodeStatus tickWheelEvent();
  void beginStairDriveProfile(const StairSpeedProfile &profile,
                              std::string label);
  double stairDriveProfileSpeed();
  void publishProfiledStairTwist(double direction_sign);

  bool beginEntryCenterAdvance(Phase next_phase, std::string label);
  bool beginGridCenterAlign(int target_grid, double target_yaw_rad,
                            Phase next_phase, std::string label);
  bool beginFinalExitCenterAlign(Phase next_phase, std::string label);
  BT::NodeStatus tickCenterAlign();
  bool centerOdomReady() const;
  bool prepareEntryCenterTarget();
  bool readCenterReference(int &grid_id, double &x, double &y,
                           double &yaw) const;
  void writeCenterReference(int grid_id, double x, double y, double yaw);
  bool computeGridCenterFromReference(int target_grid, double &target_x,
                                      double &target_y) const;
  void writeCenterTargetBlackboard() const;
  void setCenterError(const std::string &reason) const;

  std::optional<KfsVisualObservation> findR2LockObservation();
  bool configureKfsAlignPlan(const KfsVisualObservation &observation,
                             const char *context);
  void finishKfsAlignFailure(const std::string &reason);
  void beginKfsVisualPickup(bool high_side, MfPreselectionPickupSource source,
                            const KfsVisualObservation &observation,
                            Phase success_phase, Phase failure_phase,
                            bool direct_exit_on_success,
                            bool entry_high_protocol);
  BT::NodeStatus tickKfsVisualAlign();
  BT::NodeStatus beginKfsOpenLoopApproach(
      const KfsVisualObservation &observation);
  void startKfsOpenLoopApproach();
  BT::NodeStatus tickKfsOpenLoopApproach();
  double kfsApproachVx() const;
  void clearKfsVisualPickup();

  void beginGrab(bool high_side, MfPreselectionPickupSource source,
                 const MfPreselectionTargetSnapshot &target,
                 Phase success_phase, Phase failure_phase,
                 bool direct_exit_on_success, bool entry_high_protocol);
  void beginGrabVerify();
  BT::NodeStatus tickGrabVerify();
  void commitPendingGrab();
  void finishGrabVerificationFailure(const std::string &reason);

  uint8_t clampByte(int value) const;
  rclcpp::Duration seconds(double value) const;

  rclcpp::Node *node_{nullptr};
  MfPreselectionParams params_;
  StairParams stair_params_;
  GridCenterParams center_params_;

  rclcpp::Publisher<TwistMsg>::SharedPtr cmd_pub_;
  rclcpp::Publisher<TwistMsg>::SharedPtr center_cmd_pub_;
  rclcpp::Subscription<OdomMsg>::SharedPtr odom_sub_;
  rclcpp::Subscription<OdomMsg>::SharedPtr center_odom_sub_;
  rclcpp::Client<SendCommandSrv>::SharedPtr send_client_;
  rclcpp::Subscription<FeedbackMsg>::SharedPtr feedback_sub_;
  std::shared_ptr<rc26_vision::VisionInferenceManager> vision_;

  Phase phase_{Phase::Done};
  Phase command_next_phase_{Phase::Done};
  Phase move_next_phase_{Phase::Done};
  Phase turn_next_phase_{Phase::Done};
  Phase zero_hold_next_phase_{Phase::Done};
  Phase stair_next_phase_{Phase::Done};
  Phase center_next_phase_{Phase::Done};
  Phase grab_success_phase_{Phase::Done};
  Phase grab_failure_phase_{Phase::Done};
  Phase pending_detection_phase_{Phase::Done};

  DetectMode detect_mode_{DetectMode::Entry2};
  DetectMode pending_detection_mode_{DetectMode::Entry2};
  rclcpp::Time phase_start_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_cmd_publish_{0, 0, RCL_ROS_TIME};
  bool has_last_cmd_publish_{false};

  double odom_x_{0.0};
  double odom_y_{0.0};
  double odom_yaw_{0.0};
  bool has_odom_{false};
  std::chrono::steady_clock::time_point last_odom_tp_{};
  double center_odom_x_{0.0};
  double center_odom_y_{0.0};
  double center_odom_yaw_{0.0};
  bool has_center_odom_{false};
  std::chrono::steady_clock::time_point last_center_odom_tp_{};

  double move_start_x_{0.0};
  double move_start_y_{0.0};
  double move_start_yaw_{0.0};
  double move_vx_{0.0};
  double move_vy_{0.0};
  double move_distance_m_{0.0};
  std::string move_label_;
  bool move_start_captured_{false};
  bool move_waiting_odom_logged_{false};

  double turn_target_yaw_{0.0};
  int turn_stable_ticks_{0};
  std::string turn_label_;
  bool turn_waiting_odom_logged_{false};

  double zero_hold_duration_s_{0.0};
  std::string zero_hold_label_;

  std::atomic<uint64_t> command_generation_{0};
  std::atomic<bool> command_response_seen_{false};
  std::atomic<bool> command_accepted_{false};
  std::atomic<int> command_seq_{-1};
  std::atomic<bool> command_done_seen_{false};
  int command_done_feedback_id_{-1};
  uint8_t command_id_{0};
  std::string command_label_;
  std::string command_failure_reason_;
  bool command_sent_{false};
  bool command_waiting_service_logged_{false};
  bool command_ack_logged_{false};
  bool command_waiting_done_logged_{false};

  struct CommandSlot {
    uint8_t command_id{0};
    std::string label;
    bool sent{false};
    std::atomic<bool> response_seen{false};
    std::atomic<bool> accepted{false};
  };
  CommandSlot command_pair_[2];
  bool command_pair_active_{false};
  std::string command_pair_failure_reason_;
  bool command_pair_waiting_service_logged_{false};
  bool command_pair_ack_logged_[2]{false, false};

  std::atomic<uint64_t> front_first_event_count_{0};
  std::atomic<uint64_t> front_second_event_count_{0};
  std::atomic<uint64_t> rear_event_count_{0};
  uint64_t front_first_event_baseline_{0};
  uint64_t front_second_event_baseline_{0};
  uint64_t rear_event_baseline_{0};
  WheelEvent active_wheel_event_{WheelEvent::FrontFirst};
  double active_wheel_event_timeout_s_{0.0};
  std::string active_wheel_event_label_;
  bool active_wheel_event_started_{false};

  StairMode stair_mode_{StairMode::Climb};
  StairCenterPolicy stair_center_policy_{StairCenterPolicy::None};
  StairCenterPolicy center_policy_{StairCenterPolicy::None};
  StairPhase stair_phase_{StairPhase::Complete};
  std::string stair_label_;
  rclcpp::Time stair_drive_profile_start_{0, 0, RCL_ROS_TIME};
  bool stair_drive_profile_started_{false};
  StairSpeedProfile stair_drive_profile_;
  std::string stair_drive_profile_label_;
  double timed_drive_speed_mps_{0.0};
  double timed_drive_duration_s_{0.0};
  int center_target_grid_{0};
  double center_target_x_{0.0};
  double center_target_y_{0.0};
  double center_target_yaw_{0.0};
  int center_stable_ticks_{0};
  bool center_target_ready_{false};
  bool center_waiting_odom_logged_{false};
  std::string center_label_;
  double entry_heading_yaw_{0.0};

  int pickup_count_{0};
  bool entry_pickup_done_{false};
  bool direct_exit_mode_{false};
  bool arm_high_raised_{false};
  bool arm_high_side_{false};
  MfPreselectionPickupSource pickup_source_{MfPreselectionPickupSource::None};
  int current_grid_{2};
  int transition_from_grid_{2};
  int transition_target_grid_{2};
  int transition_height_delta_{0};
  bool transition_high_side_{true};
  int fake_avoid_target_grid_{0};
  bool row4_fake_detected_{false};
  bool direct_exit_move_active_{false};
  bool path_r1_waiting_{false};
  int path_r1_lost_count_{0};
  int64_t path_r1_last_sequence_{0};
  rclcpp::Time path_r1_wait_start_{0, 0, RCL_ROS_TIME};

  int detect_seen_count_{0};
  int detect_lost_count_{0};
  int64_t last_detection_sequence_{0};
  Phase active_detection_phase_{Phase::Done};
  bool detection_active_{false};
  bool active_detection_high_side_{true};
  bool pending_detection_high_side_{true};
  bool timed_drive_started_{false};
  bool kfs_pickup_active_{false};
  bool kfs_pickup_high_side_{true};
  MfPreselectionPickupSource kfs_pickup_source_{MfPreselectionPickupSource::None};
  Phase kfs_pickup_success_phase_{Phase::Done};
  Phase kfs_pickup_failure_phase_{Phase::Done};
  bool kfs_pickup_direct_exit_on_success_{false};
  bool kfs_pickup_entry_high_protocol_{false};
  std::optional<MfPreselectionTargetSnapshot> kfs_pickup_initial_target_;
  std::optional<MfPreselectionTargetSnapshot> kfs_locked_target_;
  std::optional<MfPreselectionTargetSnapshot> kfs_open_loop_target_;
  int kfs_align_stable_count_{0};
  int kfs_align_lost_count_{0};
  int64_t kfs_align_last_sequence_{0};
  int64_t kfs_align_verify_min_sequence_{0};
  rclcpp::Time kfs_align_total_start_{0, 0, RCL_ROS_TIME};
  double kfs_align_distance_m_{0.0};
  double kfs_align_duration_s_{0.0};
  double kfs_align_vy_{0.0};
  bool kfs_align_started_{false};
  bool kfs_align_waiting_verify_frame_{false};
  int kfs_open_loop_offset_px_{0};
  double kfs_open_loop_locked_depth_m_{0.0};
  double kfs_open_loop_distance_m_{0.0};
  double kfs_open_loop_duration_s_{0.0};
  bool kfs_open_loop_started_{false};
  bool entry_lateral_reference_captured_{false};
  double entry_lateral_reference_x_{0.0};
  double entry_lateral_reference_y_{0.0};
  double entry_lateral_reference_yaw_{0.0};
  bool entry_move_interrupted_active_{false};
  Phase interrupted_entry_move_next_phase_{Phase::Done};
  std::string interrupted_entry_move_label_;
  double interrupted_entry_move_target_offset_m_{0.0};
  int64_t entry_move_last_interrupt_sequence_{0};
  bool pending_grab_commit_{false};
  MfPreselectionPickupSource pending_grab_source_{MfPreselectionPickupSource::None};
  bool pending_grab_entry_high_protocol_{false};
  std::optional<MfPreselectionTargetSnapshot> pending_grab_target_;
  std::vector<MfPreselectionTargetSnapshot> ignored_r2_targets_;
  bool grab_success_direct_exit_{false};
  int grab_verify_lost_count_{0};
  int64_t grab_verify_last_sequence_{0};
  bool grab_verify_seen_new_frame_{false};
  bool grab_verify_visible_logged_{false};
  int grab_verify_last_logged_lost_count_{0};
};

} // namespace rc26_decision
