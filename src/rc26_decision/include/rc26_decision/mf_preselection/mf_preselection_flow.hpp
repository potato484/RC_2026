#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>

#include "rc26_decision/mf/grid_center.hpp"
#include "rc26_decision/stair/stair_area.hpp"
#include "rc26_interfaces/msg/mechanism_transport_feedback.hpp"
#include "rc26_interfaces/srv/send_mechanism_transport_command.hpp"
#include "rc26_serial/protocol.hpp"
#include "rc26_vision/inference/runtime/vision_inference_manager.hpp"
#include "rc26_vision/postprocess/alignment/tip_alignment.hpp"
#include "rc26_vision/shared/sensors/depth_roi_sampler.hpp"
#include "rc26_vision/shared/target/visual_target_match.hpp"

namespace rc26_decision {

enum class MfPreselectionPickupSource { None, Stair1, Stair2, Stair3 };

using MfPreselectionTargetSnapshot = rc26_vision::VisualTargetSnapshot;

struct MfPreselectionParams {
  std::string vision_config_file;
  std::string model_id{"kfs_default"};
  std::vector<std::string> r2_target_label_prefixes{"T_"};
  std::vector<std::string> r2_target_labels;
  std::vector<std::string> r1_blocking_labels{"R_R1", "B_R1", "R1_KFS"};
  std::vector<std::string> r1_blocking_label_prefixes;
  double r1_kfs_min_score{0.50};
  std::vector<std::string> fake_label_prefixes{"F_"};
  std::vector<std::string> fake_labels;
  double depth_min_m{0.6};
  double depth_max_m{1.2};
  double entry_depth_min_m{0.6};
  double entry_depth_max_m{1.2};
  int detect_seen_stable_frames{1};
  int detect_lost_stable_frames{5};
  double entry_detect_timeout_s{2.0};
  double scan_detect_timeout_s{2.0};
  int entry_interrupt_max_offset_px{180};
  bool entry_interrupt_dynamic_comp_enable{true};
  double entry_interrupt_latency_s{0.15};
  double entry_interrupt_fx_px{450.0};
  int entry_interrupt_extra_px_min{20};
  int entry_interrupt_extra_px_max{80};
  bool entry_mcu_stop_settle_enable{true};
  double entry_mcu_vy_acc_mps2{1.0};
  double entry_mcu_stop_margin_s{0.08};
  double entry_mcu_stop_max_wait_s{0.70};

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

  int max_pickup_count{2};
  double grab_settle_s{0.5};
  double grab_verify_timeout_s{3.0};
  int grab_verify_lost_stable_frames{3};
  double grab_verify_iou_threshold{0.30};
  int grab_kfs_up_command_id{
      static_cast<int>(rc26_serial::CommandID::GRAB_KFS_UP)};
  int grab_kfs_down_command_id{
      static_cast<int>(rc26_serial::CommandID::GRAB_KFS_DOWN)};
  int entry_grab_kfs_up_command_id{
      static_cast<int>(rc26_serial::CommandID::ENTRY_GRAB_KFS_UP)};
  int entry_grab_kfs_up_done_feedback_id{
      static_cast<int>(rc26_serial::FeedbackID::ENTRY_GRAB_KFS_UP_DONE)};

  std::string cmd_vel_topic{"cmd_vel"};
  std::string odom_topic{"odom"};
  std::string send_command_service{"/mechanism/send_command"};
  std::string feedback_topic{"/mechanism/command_feedback"};
  double command_timeout_s{3.0};
  int arm_high_raise_command_id{
      static_cast<int>(rc26_serial::CommandID::ARM_HIGH_RAISE)};
  int arm_high_raise_done_feedback_id{
      static_cast<int>(rc26_serial::FeedbackID::ARM_HIGH_RAISE_DONE)};
  int arm_raise_command_id{static_cast<int>(rc26_serial::CommandID::ARM_RAISE)};
  int arm_lower_command_id{static_cast<int>(rc26_serial::CommandID::ARM_LOWER)};
  int arm_raise_done_feedback_id{
      static_cast<int>(rc26_serial::FeedbackID::ARM_RAISE_DONE)};
  int arm_lower_done_feedback_id{
      static_cast<int>(rc26_serial::FeedbackID::ARM_LOWER_DONE)};
  int second_arm_lower_command_id{
      static_cast<int>(rc26_serial::CommandID::ARM_SECOND_LOWER)};
  int second_arm_lower_done_feedback_id{
      static_cast<int>(rc26_serial::FeedbackID::ARM_SECOND_LOWER_DONE)};

  double entry_probe_left_distance_m{1.2};
  double entry_probe_right_sweep_distance_m{2.4};
  double entry_probe_return_distance_m{1.2};
  double lateral_probe_speed_mps{0.35};
  double move_tolerance_m{0.03};
  double move_timeout_s{12.0};
  double direct_exit_drive_distance_m{4.8};
  double direct_exit_drive_speed_mps{0.25};
  int field_mirror_sign{1};

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
  enum class KfsDepthSource { None, CenterRoi, BboxMultiRoi, MonocularBbox };
  enum class GrabRetryAction { None, EntryBackoff, GridCenterRetry };

  struct DepthRoiDiagnostic {
    int cx{0};
    int cy{0};
    int roi_size{0};
    int min_valid_count{0};
    int depth_rows{0};
    int depth_cols{0};
    int depth_type{0};
    std::string depth_type_name;
    double min_depth_m{0.0};
    double max_depth_m{0.0};
    int total_pixels{0};
    int zero_depth_count{0};
    int non_finite_count{0};
    int below_min_count{0};
    int above_max_count{0};
    int window_valid_count{0};
    int raw_valid_count{0};
    double raw_min_m{0.0};
    double raw_max_m{0.0};
    double raw_median_m{0.0};
    bool depth_empty{false};
    bool unsupported_type{false};
    bool sampled{false};
    double sampled_depth_m{0.0};
    std::string primary_failure;
  };

  struct KfsBboxDepthSample {
    bool has_depth{false};
    double depth_m{0.0};
    KfsDepthSource source{KfsDepthSource::None};
    int sample_point_count{0};
    int success_count{0};
    DepthRoiDiagnostic representative_failure;
    std::string detail;
  };

  struct KfsMonocularDepthEstimate {
    bool usable{false};
    double depth_m{0.0};
    double bbox_width_px{0.0};
    double bbox_height_px{0.0};
    double z_width_m{0.0};
    double z_height_m{0.0};
    double locked_delta_m{0.0};
    std::string reject_reason;
    std::string detail;
  };

  static bool labelMatches(const std::string &label,
                           const std::vector<std::string> &exact_labels,
                           const std::vector<std::string> &prefixes);
  static bool r1KfsScoreAccepted(const std::string &label, double score,
                                 double min_score);
  static bool canPickup(int pickup_count, int max_pickup_count);
  static GrabRetryAction
  grabRetryAction(bool target_still_visible, MfPreselectionPickupSource source,
                  bool entry_high_protocol, bool path_blocking);
  static bool entryInterruptOffsetAcceptable(int offset_px,
                                             const MfPreselectionParams &params);
  static bool entryInterruptOffsetAcceptable(int offset_px,
                                             double lateral_speed_mps,
                                             double depth_m,
                                             const MfPreselectionParams &params);
  static double mcuSineStopTime(double speed_mps, double acc_mps2);
  static double mcuSineStopDistance(double speed_mps, double acc_mps2);
  static double entryReturnToCenterDistanceCompensation(
      double lateral_speed_mps, const MfPreselectionParams &params);
  static double entryReturnToCenterCompensatedDistance(
      double raw_distance_m, double lateral_speed_mps,
      const MfPreselectionParams &params, double &compensation_m);
  static int entryInterruptDynamicExtraPx(double lateral_speed_mps,
                                          double depth_m,
                                          const MfPreselectionParams &params);
  static int entryInterruptEffectiveOffsetLimitPx(
      double lateral_speed_mps, double depth_m,
      const MfPreselectionParams &params);
  static double entryMcuStopSettleDuration(double lateral_speed_mps,
                                           const MfPreselectionParams &params);
  static bool kfsAlignTimeoutPickupAllowed(int offset_px, bool has_depth,
                                           const MfPreselectionParams &params);
  static DepthRoiDiagnostic depthRoiDiagnostic(
      const cv::Mat &depth, int cx, int cy,
      const rc26_vision::DepthRoiSamplerConfig &config);
  static std::string depthRoiDiagnosticDetail(
      const DepthRoiDiagnostic &diagnostic);
  static const char *kfsDepthSourceText(KfsDepthSource source);
  static bool kfsDepthSourceIsReal(KfsDepthSource source);
  static KfsBboxDepthSample sampleKfsDepthFromBbox(
      const cv::Mat &depth, double x1, double y1, double x2, double y2,
      const rc26_vision::DepthRoiSamplerConfig &config,
      const std::vector<double> &sample_ratios, int min_success_count);
  static KfsMonocularDepthEstimate estimateKfsMonocularDepth(
      double bbox_width_px, double bbox_height_px, double locked_depth_m,
      const MfPreselectionParams &params, double min_depth_m,
      double max_depth_m);
  static rc26_vision::TipAlignmentConfig
  kfsAlignmentConfig(const MfPreselectionParams &params,
                     double target_yaw_rad);
  static double kfsOpenLoopDistance(double locked_depth_m,
                                    double grab_distance_m);
  static double kfsOpenLoopDuration(double distance_m, double speed_mps);
  static double kfsApproachOdomDistance(double locked_depth_m,
                                        const MfPreselectionParams &params);
  static void normalizeKfsOdomParams(MfPreselectionParams &params);
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
  static bool postGrabCenterAlignRequired(MfPreselectionPickupSource source,
                                          bool entry_high_protocol);
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
                          MfPreselectionPickupSource source,
                          int mirror_sign = 1);
  static std::optional<int> fakeAvoidanceForwardTargetGrid(int current_grid);
  static MfPreselectionPickupSource
  entryPickupSourceForLateralOffset(double lateral_offset_m,
                                    double tolerance_m,
                                    int mirror_sign = 1);
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
    FakeAvoidArmAdjust,
    FakeAvoidStair,
    FakeAvoidAlignExit,
    FakeAvoidForwardStep,
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
    EntryRetryBackoff,
    RetryPostGrabCenterAlign,
    KfsVisualAlign,
    KfsSecondArmLower,
    KfsOdomApproach,
    MechanismCommand,
    GrabVerify,
    MoveRelative,
    TurnYaw,
    ZeroHold,
    StairPrimitive,
    PostGrabCenterAlign,
    CenterAlign,
    Done
  };

  enum class DetectMode { Entry2, Stair1, Stair3, RowFront, Scan, Row4Fake, TransitionObserve };
  enum class R2DepthProfile { General, Entry };
  enum class StairMode { Climb, Descend };
  enum class KfsOdomAxis { X, Y };
  enum class KfsOdomMotionResult { Running, Succeeded, Failed };
  enum class R2LockObservationMode {
    RequireDepthForDetection,
    AllowDepthlessForAlign
  };
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
    MfPreselectionLogicResult::KfsDepthSource depth_source{
        MfPreselectionLogicResult::KfsDepthSource::None};
    std::string depth_detail;
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
  std::optional<MfPreselectionTargetSnapshot> findR1BlockingTarget();
  std::optional<MfPreselectionTargetSnapshot> findFakeTarget();
  std::optional<MfPreselectionTargetSnapshot>
  findTarget(const std::vector<std::string> &exact,
             const std::vector<std::string> &prefixes, bool skip_ignored_r2,
             bool filter_r1_kfs_score = false);
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
  void beginEntryMcuStopSettle(double lateral_speed_mps);
  bool tickEntryMcuStopSettle();
  BT::NodeStatus beginEntryReturnToCenterAfterInterruptedPickup();
  BT::NodeStatus resumeInterruptedEntryMove();
  void beginDirectExitDrive();
  BT::NodeStatus tickDirectExitDrive();
  bool guardPathObstacles();
  void clearPathR1Wait();
  bool lastGrabOriginPathBlocking() const;
  bool scheduleGrabRetryAfterVisibleFailure(const std::string &reason);
  BT::NodeStatus tickEntryRetryBackoff();
  BT::NodeStatus beginRetryPostGrabCenterAlign();
  void clearGrabRetryContext();

  void beginTurnYaw(double target_yaw_rad, Phase next_phase, std::string label);
  BT::NodeStatus tickTurnYaw();
  void beginZeroHold(double duration_s, Phase next_phase, std::string label);
  BT::NodeStatus tickZeroHold();

  bool prepareTransitionTo(int target_grid);
  BT::NodeStatus startTransitionTo(int target_grid);
  BT::NodeStatus startFakeAvoidForwardObservation();
  BT::NodeStatus startFakeAvoidForwardTransitionTo(int target_grid);
  bool continueAfterTransition();
  Phase phaseAfterTransition() const;
  double transitionEdgeYaw(int from_grid, int target_grid) const;
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
  BT::NodeStatus beginPostGrabCenterAlign();
  double postGrabCenterYaw() const;
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

  std::optional<KfsVisualObservation>
  findR2LockObservation(
      R2DepthProfile depth_profile = R2DepthProfile::General,
      R2LockObservationMode mode = R2LockObservationMode::RequireDepthForDetection);
  void recordR2LockReject(const std::string &reason,
                          const std::string &detail, int64_t sequence);
  void clearR2LockReject();
  std::string r2LockRejectSummary() const;
  rc26_vision::TipAlignmentConfig makeKfsAlignmentConfig() const;
  std::optional<rc26_vision::TipHeadingControl> kfsVisualAlignHeadingControl();
  std::optional<KfsVisualObservation> kfsAlignTimeoutObservation() const;
  void finishKfsAlignFailure(const std::string &reason);
  void publishKfsVisualAlignTwist(double vy, double wz);
  void beginKfsOdomAxisMotion(KfsOdomAxis axis, double distance_m,
                              double max_speed_mps, double min_speed_mps,
                              double tolerance_m, double timeout_s,
                              std::string label);
  KfsOdomMotionResult tickKfsOdomAxisMotion(std::string &failure_reason);
  void clearKfsOdomAxisMotion();
  void beginKfsVisualPickup(bool high_side, MfPreselectionPickupSource source,
                            const KfsVisualObservation &observation,
                            Phase success_phase, Phase failure_phase,
                            bool direct_exit_on_success,
                            bool entry_high_protocol,
                            R2DepthProfile depth_profile = R2DepthProfile::General);
  BT::NodeStatus tickKfsVisualAlign();
  BT::NodeStatus beginKfsOdomApproach(
      const KfsVisualObservation &observation);
  void startKfsOdomApproach();
  BT::NodeStatus tickKfsOdomApproach();
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
  Phase post_grab_center_next_phase_{Phase::Done};
  Phase pending_detection_phase_{Phase::Done};
  Phase kfs_pickup_origin_phase_{Phase::Done};
  Phase last_grab_origin_phase_{Phase::Done};
  Phase retry_grab_origin_phase_{Phase::Done};

  DetectMode detect_mode_{DetectMode::Entry2};
  DetectMode pending_detection_mode_{DetectMode::Entry2};
  DetectMode kfs_pickup_origin_detect_mode_{DetectMode::Entry2};
  DetectMode last_grab_origin_detect_mode_{DetectMode::Entry2};
  DetectMode retry_grab_origin_detect_mode_{DetectMode::Entry2};
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
    std::atomic<int> seq{-1};
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
  bool fake_avoid_forward_mode_{false};
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
  R2DepthProfile kfs_pickup_depth_profile_{R2DepthProfile::General};
  std::optional<MfPreselectionTargetSnapshot> kfs_pickup_initial_target_;
  std::optional<MfPreselectionTargetSnapshot> kfs_locked_target_;
  std::optional<MfPreselectionTargetSnapshot> kfs_odom_target_;
  rc26_vision::TipTargetLockState kfs_align_target_lock_state_;
  std::optional<KfsVisualObservation> kfs_align_last_observation_;
  int64_t kfs_align_target_lock_sequence_{0};
  int kfs_align_stable_count_{0};
  int kfs_align_lost_count_{0};
  int64_t kfs_align_last_sequence_{0};
  rclcpp::Time kfs_align_total_start_{0, 0, RCL_ROS_TIME};
  bool kfs_entry_mcu_stop_settle_active_{false};
  rclcpp::Time kfs_entry_mcu_stop_settle_until_{0, 0, RCL_ROS_TIME};
  double kfs_entry_mcu_stop_settle_duration_s_{0.0};
  double kfs_entry_mcu_stop_settle_speed_mps_{0.0};
  bool kfs_entry_mcu_stop_settle_done_logged_{false};
  bool kfs_align_waiting_verify_frame_{false};
  bool kfs_align_yaw_hold_captured_{false};
  double kfs_align_yaw_hold_target_{0.0};
  bool kfs_align_waiting_odom_logged_{false};
  int kfs_odom_offset_px_{0};
  double kfs_odom_locked_depth_m_{0.0};
  double kfs_last_real_depth_m_{0.0};
  double kfs_odom_approach_distance_m_{0.0};
  double kfs_odom_approach_estimated_duration_s_{0.0};
  bool kfs_odom_approach_started_{false};
  KfsOdomAxis kfs_odom_axis_{KfsOdomAxis::X};
  double kfs_odom_motion_distance_m_{0.0};
  double kfs_odom_motion_min_speed_mps_{0.0};
  double kfs_odom_motion_max_speed_mps_{0.0};
  double kfs_odom_motion_tolerance_m_{0.0};
  double kfs_odom_motion_timeout_s_{0.0};
  double kfs_odom_motion_start_x_{0.0};
  double kfs_odom_motion_start_y_{0.0};
  double kfs_odom_motion_start_yaw_{0.0};
  rclcpp::Time kfs_odom_motion_start_time_{0, 0, RCL_ROS_TIME};
  bool kfs_odom_motion_started_{false};
  bool kfs_odom_motion_start_captured_{false};
  bool kfs_odom_motion_waiting_logged_{false};
  int kfs_odom_motion_stable_ticks_{0};
  std::string kfs_odom_motion_label_;
  bool retry_grab_context_valid_{false};
  bool retry_grab_backoff_started_{false};
  bool retry_grab_high_side_{true};
  MfPreselectionPickupSource retry_grab_source_{MfPreselectionPickupSource::None};
  Phase retry_grab_success_phase_{Phase::Done};
  Phase retry_grab_failure_phase_{Phase::Done};
  bool retry_grab_direct_exit_on_success_{false};
  bool retry_grab_entry_high_protocol_{false};
  R2DepthProfile retry_grab_depth_profile_{R2DepthProfile::General};
  double retry_grab_approach_distance_m_{0.0};
  std::optional<MfPreselectionTargetSnapshot> retry_grab_target_;
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
  bool last_grab_retry_context_valid_{false};
  bool last_grab_high_side_{true};
  MfPreselectionPickupSource last_grab_source_{MfPreselectionPickupSource::None};
  Phase last_grab_success_phase_{Phase::Done};
  Phase last_grab_failure_phase_{Phase::Done};
  bool last_grab_direct_exit_on_success_{false};
  bool last_grab_entry_high_protocol_{false};
  R2DepthProfile last_grab_depth_profile_{R2DepthProfile::General};
  double last_grab_approach_distance_m_{0.0};
  std::optional<MfPreselectionTargetSnapshot> last_grab_target_;
  std::vector<MfPreselectionTargetSnapshot> ignored_r2_targets_;
  std::string last_r2_lock_reject_reason_;
  std::string last_r2_lock_reject_detail_;
  int64_t last_r2_lock_reject_sequence_{0};
  std::unordered_set<std::string> r2_lock_logged_reasons_this_detection_;
  std::string kfs_align_last_logged_reject_reason_;
  bool grab_success_direct_exit_{false};
  int grab_verify_lost_count_{0};
  int64_t grab_verify_last_sequence_{0};
  bool grab_verify_seen_new_frame_{false};
  bool grab_verify_visible_logged_{false};
  int grab_verify_last_logged_lost_count_{0};
  int64_t r1_kfs_low_score_last_logged_sequence_{0};
};

} // namespace rc26_decision
