#include "rc26_decision/mf_preselection/mf_preselection_flow.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <limits>
#include <stdexcept>
#include <utility>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "rc26_decision/mf/merlin_map.hpp"
#include "rc26_serial/protocol.hpp"
#include "rc26_vision/inference/config/model_profile_loader.hpp"
#include "rc26_vision/shared/sensors/depth_roi_sampler.hpp"

namespace rc26_decision {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg2Rad = kPi / 180.0;
constexpr double kMinTimeoutS = 0.001;
constexpr double kMinSpeed = 0.001;

std::string resolveVisionConfig(const std::string &configured) {
  namespace fs = std::filesystem;
  if (!configured.empty() && fs::exists(configured)) {
    return fs::path(configured).lexically_normal().string();
  }
  try {
    const fs::path share =
        ament_index_cpp::get_package_share_directory("rc26_vision");
    const fs::path candidate =
        configured.empty() ? (share / "config" / "vision_models.yaml")
                           : (share / configured);
    if (fs::exists(candidate)) {
      return candidate.lexically_normal().string();
    }
  } catch (...) {
  }
  return configured;
}

std::vector<std::string> sanitized(std::vector<std::string> values) {
  std::vector<std::string> out;
  out.reserve(values.size());
  for (auto &value : values) {
    const auto first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
      continue;
    }
    const auto last = value.find_last_not_of(" \t\r\n");
    const auto clean = value.substr(first, last - first + 1);
    if (!clean.empty()) {
      out.push_back(clean);
    }
  }
  return out;
}

int gridRow(int grid_id) { return (grid_id - 1) / 3; }
int gridCol(int grid_id) { return (grid_id - 1) % 3; }

const char *sourceName(MfPreselectionPickupSource source) {
  switch (source) {
  case MfPreselectionPickupSource::Stair1:
    return "stair1";
  case MfPreselectionPickupSource::Stair2:
    return "stair2";
  case MfPreselectionPickupSource::Stair3:
    return "stair3";
  case MfPreselectionPickupSource::None:
  default:
    return "none";
  }
}

} // namespace

bool MfPreselectionLogicResult::labelMatches(
    const std::string &label, const std::vector<std::string> &exact_labels,
    const std::vector<std::string> &prefixes) {
  for (const auto &exact : exact_labels) {
    if (!exact.empty() && label == exact) {
      return true;
    }
  }
  for (const auto &prefix : prefixes) {
    if (!prefix.empty() && label.rfind(prefix, 0) == 0) {
      return true;
    }
  }
  return false;
}

bool MfPreselectionLogicResult::canPickup(int pickup_count,
                                          int max_pickup_count) {
  return pickup_count < std::max(0, max_pickup_count);
}

double MfPreselectionLogicResult::fakeAvoidanceYaw(
    MfPreselectionPickupSource source, const MfPreselectionParams &params) {
  if (source == MfPreselectionPickupSource::Stair3) {
    return params.stair3_direction_yaw_rad;
  }
  return params.stair1_direction_yaw_rad;
}

uint8_t MfPreselectionLogicResult::grabCommandForHighSide(
    bool high_side, const MfPreselectionParams &params) {
  const int value =
      high_side ? params.grab_kfs_up_command_id : params.grab_kfs_down_command_id;
  return static_cast<uint8_t>(std::clamp(value, 0, 255));
}

MfPreselectionFlowAction::MfPreselectionFlowAction(
    const std::string &name, const BT::NodeConfig &config)
    : BT::StatefulActionNode(name, config) {}

MfPreselectionFlowAction::~MfPreselectionFlowAction() { releaseRuntime(); }

BT::PortsList MfPreselectionFlowAction::providedPorts() { return {}; }

BT::NodeStatus MfPreselectionFlowAction::onStart() {
  if (!setupRuntime()) {
    return BT::NodeStatus::FAILURE;
  }
  if (!setupVision()) {
    return fail("vision_start_failed");
  }

  pickup_count_ = 0;
  entry_pickup_done_ = false;
  direct_exit_mode_ = false;
  arm_high_raised_ = false;
  arm_high_side_ = false;
  pickup_source_ = MfPreselectionPickupSource::None;
  current_grid_ = 2;
  row4_fake_detected_ = false;
  direct_exit_move_active_ = false;
  clearPathR1Wait();
  pending_grab_commit_ = false;
  pending_grab_source_ = MfPreselectionPickupSource::None;
  writeBlackboardState("entry_detect_stair2");
  RCLCPP_INFO(node_->get_logger(),
              "梅林预选赛流程启动：当前位置按 grid2 / 2号入口处理，入口导航已由行为树前置控制，最大夹取数=%d",
              params_.max_pickup_count);
  beginDetection(DetectMode::Entry2, params_.entry_detect_timeout_s);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MfPreselectionFlowAction::onRunning() {
  switch (phase_) {
  case Phase::EntryDetectStair2:
  case Phase::EntryDetectStair1:
  case Phase::EntryDetectStair3:
  case Phase::RowFrontDetect:
  case Phase::RowScanDetectLeft:
  case Phase::RowScanDetectBack:
  case Phase::Row4DetectFake:
  case Phase::TransitionObserve:
    return tickDetection();

  case Phase::EntryHighRaise:
    beginMechanismCommand(clampByte(params_.arm_high_raise_command_id),
                          "ARM_HIGH_RAISE",
                          clampByte(params_.arm_high_raise_done_feedback_id),
                          Phase::EntryMoveLeft, "arm_high_raise_failed");
    arm_high_raised_ = true;
    arm_high_side_ = true;
    return BT::NodeStatus::RUNNING;

  case Phase::EntryMoveLeft:
    beginMoveRelative(0.0, std::abs(params_.lateral_probe_speed_mps),
                      params_.entry_probe_left_distance_m,
                      Phase::EntryDetectStair1, "entry_probe_left");
    return BT::NodeStatus::RUNNING;

  case Phase::EntryReturnFromStair1:
    beginMoveRelative(0.0, -std::abs(params_.lateral_probe_speed_mps),
                      params_.entry_probe_return_distance_m,
                      Phase::EntryPrepareClimb, "entry_return_from_stair1");
    return BT::NodeStatus::RUNNING;

  case Phase::EntryMoveRightToStair3:
    beginMoveRelative(0.0, -std::abs(params_.lateral_probe_speed_mps),
                      params_.entry_probe_right_sweep_distance_m,
                      Phase::EntryDetectStair3, "entry_probe_stair3");
    return BT::NodeStatus::RUNNING;

  case Phase::EntryReturnFromStair3:
    beginMoveRelative(0.0, std::abs(params_.lateral_probe_speed_mps),
                      params_.entry_probe_return_distance_m,
                      Phase::EntryPrepareClimb, "entry_return_from_stair3");
    return BT::NodeStatus::RUNNING;

  case Phase::EntryPrepareClimb:
    if (arm_high_raised_) {
      phase_ = Phase::EntryClimb;
      return BT::NodeStatus::RUNNING;
    }
    beginMechanismCommand(clampByte(params_.arm_raise_command_id), "ARM_RAISE",
                          clampByte(params_.arm_raise_done_feedback_id),
                          Phase::EntryClimb, "entry_arm_raise_failed");
    arm_high_side_ = true;
    return BT::NodeStatus::RUNNING;

  case Phase::EntryClimb:
    beginStair(StairMode::Climb, Phase::AfterEntry, "entry_climb");
    return BT::NodeStatus::RUNNING;

  case Phase::AfterEntry:
    config().blackboard->set("current_grid", current_grid_);
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛进入梅林内部决策：current_grid=%d，入场夹取=%s，直出模式=%s，计数=%d/%d",
                current_grid_, entry_pickup_done_ ? "是" : "否",
                direct_exit_mode_ ? "是" : "否", pickup_count_,
                params_.max_pickup_count);
    if (entry_pickup_done_ || direct_exit_mode_) {
      direct_exit_mode_ = true;
      if (current_grid_ == 2 || current_grid_ == 5 || current_grid_ == 8) {
        RCLCPP_INFO(node_->get_logger(),
                    "梅林预选赛直出模式：grid%d 前方守卫检测后继续中间列推进",
                    current_grid_);
        beginDetection(DetectMode::RowFront, params_.scan_detect_timeout_s);
      } else if (current_grid_ == 11) {
        RCLCPP_INFO(node_->get_logger(),
                    "梅林预选赛直出模式到达grid11，进入第四行强制转向收尾");
        phase_ = Phase::Row4ForcedTurn;
      } else if (current_grid_ == 12) {
        RCLCPP_INFO(node_->get_logger(),
                    "梅林预选赛到达grid12，准备下阶梯离场");
        phase_ = Phase::Row4DirectDescendPrep;
      } else {
        beginDirectExitDrive();
      }
    } else if (current_grid_ == 2) {
      RCLCPP_INFO(node_->get_logger(),
                  "梅林预选赛未入场夹取：第一行执行前方检测");
      beginDetection(DetectMode::RowFront, params_.scan_detect_timeout_s);
    } else if (current_grid_ == 5 || current_grid_ == 8) {
      RCLCPP_INFO(node_->get_logger(),
                  "梅林预选赛未入场夹取：第2/3行先做前方守卫检测");
      beginDetection(DetectMode::RowFront, params_.scan_detect_timeout_s);
    } else if (current_grid_ == 11) {
      RCLCPP_INFO(node_->get_logger(),
                  "梅林预选赛到达第四行grid11，进入强制收尾转向");
      phase_ = Phase::Row4ForcedTurn;
    } else {
      phase_ = Phase::TransitionTurn;
    }
    return BT::NodeStatus::RUNNING;

  case Phase::RowScanTurnLeft:
    beginTurnYaw(normalizeAngle(odom_yaw_ + params_.row_scan_left_yaw_delta_rad),
                 Phase::RowScanDetectLeft, "row_scan_turn_left");
    return BT::NodeStatus::RUNNING;

  case Phase::RowScanTurnBack:
    beginTurnYaw(normalizeAngle(odom_yaw_ + params_.row_scan_back_yaw_delta_rad),
                 Phase::RowScanDetectBack, "row_scan_turn_back");
    return BT::NodeStatus::RUNNING;

  case Phase::RowAlignExit:
    beginTurnYaw(params_.exit_yaw_rad, Phase::TransitionTurn,
                 "row_align_exit");
    return BT::NodeStatus::RUNNING;

  case Phase::FakeAvoidTurn:
    RCLCPP_WARN(node_->get_logger(),
                "梅林预选赛开始假KFS避障转向：pickup_source=%s，目标yaw=%.3f",
                sourceName(pickup_source_),
                MfPreselectionLogicResult::fakeAvoidanceYaw(pickup_source_,
                                                            params_));
    beginTurnYaw(MfPreselectionLogicResult::fakeAvoidanceYaw(pickup_source_,
                                                             params_),
                 Phase::FakeAvoidArmRaise, "fake_kfs_avoid_turn");
    return BT::NodeStatus::RUNNING;

  case Phase::FakeAvoidArmRaise:
    if (arm_high_raised_) {
      RCLCPP_INFO(node_->get_logger(),
                  "梅林预选赛假KFS避障：机械臂已处于高抬升保持态，直接执行上阶梯");
      phase_ = Phase::FakeAvoidClimb;
      return BT::NodeStatus::RUNNING;
    }
    beginMechanismCommand(clampByte(params_.arm_raise_command_id), "ARM_RAISE",
                          clampByte(params_.arm_raise_done_feedback_id),
                          Phase::FakeAvoidClimb, "fake_avoid_arm_raise_failed");
    arm_high_side_ = true;
    return BT::NodeStatus::RUNNING;

  case Phase::FakeAvoidClimb:
    beginStair(StairMode::Climb, Phase::FakeAvoidAlignExit,
               "fake_avoid_climb");
    return BT::NodeStatus::RUNNING;

  case Phase::FakeAvoidAlignExit:
    direct_exit_mode_ = true;
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛假KFS避障上阶梯完成，重新朝出口方向对齐并进入直行离场兜底");
    beginTurnYaw(params_.exit_yaw_rad, Phase::DirectExitDrive,
                 "fake_avoid_align_exit");
    return BT::NodeStatus::RUNNING;

  case Phase::TransitionTurn:
    if (current_grid_ == 2) {
      return startTransitionTo(5);
    } else if (current_grid_ == 5) {
      return startTransitionTo(8);
    } else if (current_grid_ == 8) {
      return startTransitionTo(11);
    } else if (current_grid_ == 11) {
      return startTransitionTo(12);
    } else {
      return fail("no_transition_target");
    }

  case Phase::TransitionArmAdjust:
    if (transition_height_delta_ > 0) {
      if (arm_high_raised_) {
        transition_high_side_ = true;
        RCLCPP_INFO(node_->get_logger(),
                    "梅林预选赛格间转换为低到高，入口高抬升仍保持，跳过普通ARM_RAISE并观察前方");
        beginDetection(DetectMode::TransitionObserve,
                       params_.scan_detect_timeout_s);
      } else {
        RCLCPP_INFO(node_->get_logger(),
                    "梅林预选赛格间转换为低到高，先执行普通ARM_RAISE后观察前方");
        beginMechanismCommand(clampByte(params_.arm_raise_command_id),
                              "ARM_RAISE",
                              clampByte(params_.arm_raise_done_feedback_id),
                              Phase::TransitionObserve,
                              "transition_arm_raise_failed");
        transition_high_side_ = true;
        arm_high_side_ = true;
      }
    } else {
      RCLCPP_INFO(node_->get_logger(),
                  "梅林预选赛格间转换为高到低，先执行ARM_LOWER后观察前方");
      beginMechanismCommand(clampByte(params_.arm_lower_command_id),
                            "ARM_LOWER",
                            clampByte(params_.arm_lower_done_feedback_id),
                            Phase::TransitionObserve,
                            "transition_arm_lower_failed");
      transition_high_side_ = false;
      arm_high_raised_ = false;
      arm_high_side_ = false;
    }
    return BT::NodeStatus::RUNNING;

  case Phase::TransitionStair:
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛开始执行格间台阶动作：grid%d -> grid%d，类型=%s",
                transition_from_grid_, transition_target_grid_,
                transition_height_delta_ > 0 ? "上阶梯" : "下阶梯");
    beginStair(transition_height_delta_ > 0 ? StairMode::Climb
                                            : StairMode::Descend,
               Phase::AfterEntry, "grid_transition");
    stair_next_phase_ = Phase::AfterEntry;
    phase_ = Phase::StairPrimitive;
    return BT::NodeStatus::RUNNING;

  case Phase::Row4ForcedTurn:
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛第四行强制转向：target_yaw=%.3f，随后检测正前方假KFS",
                params_.row4_exit_turn_yaw_rad);
    beginTurnYaw(params_.row4_exit_turn_yaw_rad, Phase::Row4DetectFake,
                 "row4_forced_turn");
    return BT::NodeStatus::RUNNING;

  case Phase::Row4FakeTurnBack:
    RCLCPP_WARN(node_->get_logger(),
                "梅林预选赛第四行假KFS分支：执行180度转向后准备下阶梯离场");
    beginTurnYaw(normalizeAngle(odom_yaw_ + kPi),
                 Phase::Row4DirectDescendPrep, "row4_fake_turn_back");
    return BT::NodeStatus::RUNNING;

  case Phase::Row4DirectDescendPrep:
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛离场前机械臂下降：准备下阶梯离开梅林");
    beginMechanismCommand(clampByte(params_.arm_lower_command_id),
                          "ARM_LOWER",
                          clampByte(params_.arm_lower_done_feedback_id),
                          Phase::Row4DirectDescend,
                          "row4_direct_arm_lower_failed");
    arm_high_raised_ = false;
    arm_high_side_ = false;
    return BT::NodeStatus::RUNNING;

  case Phase::Row4DirectDescend:
    beginStair(StairMode::Descend, Phase::FinalStop, "row4_direct_descend");
    return BT::NodeStatus::RUNNING;

  case Phase::DirectExitDrive:
    return tickDirectExitDrive();

  case Phase::FinalStop:
    publishStop();
    writeBlackboardState("final_stop");
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛已驶出梅林区域，进入最终停车保持");
    beginZeroHold(0.5, Phase::Done, "final_stop_hold");
    return BT::NodeStatus::RUNNING;

  case Phase::MechanismCommand:
    if (command_pair_active_) {
      return tickCommandPair();
    }
    return tickMechanismCommand();
  case Phase::MoveRelative:
    return tickMoveRelative();
  case Phase::TurnYaw:
    return tickTurnYaw();
  case Phase::ZeroHold:
    return tickZeroHold();
  case Phase::StairPrimitive:
    return tickStair();

  case Phase::Done:
    publishStop();
    if (node_) {
      RCLCPP_INFO(node_->get_logger(),
                  "梅林预选赛流程完成：已夹取=%d/%d，最终grid=%d，进入WaitForever永久静止",
                  pickup_count_, params_.max_pickup_count, current_grid_);
    }
    releaseRuntime();
    config().blackboard->set("mf_preselect_done", true);
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

void MfPreselectionFlowAction::onHalted() {
  if (node_) {
    RCLCPP_WARN(node_->get_logger(),
                "梅林预选赛流程被行为树中断：立即发布零速并释放运行接口");
  }
  releaseRuntime();
}

bool MfPreselectionFlowAction::setupRuntime() {
  if (!config().blackboard || !config().blackboard->get("node", node_) ||
      !node_) {
    return false;
  }
  if (!config().blackboard->get("mf_preselection_params", params_)) {
    RCLCPP_ERROR(node_->get_logger(), "梅林预选赛: 黑板缺少 mf_preselection_params");
    return false;
  }
  if (!config().blackboard->get("stair_params", stair_params_)) {
    RCLCPP_ERROR(node_->get_logger(), "梅林预选赛: 黑板缺少 stair_params");
    return false;
  }
  normalizeParams();

  cmd_pub_ =
      node_->create_publisher<TwistMsg>(params_.cmd_vel_topic, rclcpp::QoS(10));
  send_client_ = node_->create_client<SendCommandSrv>(params_.send_command_service);
  feedback_sub_ = node_->create_subscription<FeedbackMsg>(
      params_.feedback_topic, rclcpp::QoS(32).reliable(),
      [this](const FeedbackMsg::SharedPtr msg) {
        if (!msg) {
          return;
        }
        if (msg->feedback_id ==
            static_cast<uint8_t>(rc26_serial::FeedbackID::FRONT_LASER_HEIGHT_JUMP)) {
          front_first_event_count_.fetch_add(1, std::memory_order_relaxed);
        } else if (msg->feedback_id ==
                   static_cast<uint8_t>(rc26_serial::FeedbackID::FRONT_SECOND_LASER_HEIGHT_JUMP)) {
          front_second_event_count_.fetch_add(1, std::memory_order_relaxed);
        } else if (msg->feedback_id ==
                   static_cast<uint8_t>(rc26_serial::FeedbackID::REAR_LASER_HEIGHT_JUMP)) {
          rear_event_count_.fetch_add(1, std::memory_order_relaxed);
        }
        const int expected_feedback = command_done_feedback_id_;
        const int expected_seq = command_seq_.load(std::memory_order_relaxed);
        if (expected_feedback >= 0 && expected_seq >= 0 &&
            msg->feedback_id == static_cast<uint8_t>(expected_feedback) &&
            msg->seq == static_cast<uint8_t>(expected_seq)) {
          const bool already_done =
              command_done_seen_.exchange(true, std::memory_order_relaxed);
          if (!already_done && node_) {
            RCLCPP_INFO(node_->get_logger(),
                        "梅林预选赛收到机构完成反馈：feedback=0x%02X seq=%d",
                        static_cast<unsigned int>(msg->feedback_id),
                        static_cast<int>(msg->seq));
          }
        }
      });
  if (!params_.odom_topic.empty()) {
    odom_sub_ = node_->create_subscription<OdomMsg>(
        params_.odom_topic, rclcpp::QoS(rclcpp::KeepLast(10)),
        [this](const OdomMsg::SharedPtr msg) { handleOdom(msg); });
  }

  has_odom_ = false;
  command_generation_.fetch_add(1, std::memory_order_relaxed);
  config().blackboard->set("mf_preselect_pickup_count", 0);
  config().blackboard->set("mf_preselect_pickup_source", std::string("none"));
  config().blackboard->set("mf_preselect_done", false);
  RCLCPP_INFO(node_->get_logger(),
              "梅林预选赛运行接口就绪：cmd_vel=%s odom=%s command_service=%s feedback=%s",
              params_.cmd_vel_topic.c_str(), params_.odom_topic.c_str(),
              params_.send_command_service.c_str(), params_.feedback_topic.c_str());
  return true;
}

bool MfPreselectionFlowAction::setupVision() {
  try {
    params_.vision_config_file = resolveVisionConfig(params_.vision_config_file);
    auto config = rc26_vision::ProfileLoader::loadFromYaml(params_.vision_config_file);
    rc26_vision::ProfileLoader::validate(config);
    if (config.profiles.find(params_.model_id) == config.profiles.end()) {
      RCLCPP_ERROR(node_->get_logger(), "梅林预选赛 KFS profile 不存在: %s",
                   params_.model_id.c_str());
      return false;
    }
    vision_ = std::make_shared<rc26_vision::VisionInferenceManager>(*node_);
    vision_->loadConfig(config);
    vision_->selectModel(params_.model_id);
    if (!vision_->start()) {
      RCLCPP_ERROR(node_->get_logger(), "梅林预选赛 KFS 视觉启动失败");
      vision_.reset();
      return false;
    }
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛 KFS 视觉已启动：config=%s model=%s R2前缀数=%zu R1标签数=%zu 假KFS前缀数=%zu 深度=[%.2f, %.2f]m",
                params_.vision_config_file.c_str(), params_.model_id.c_str(),
                params_.r2_target_label_prefixes.size(),
                params_.r1_blocking_labels.size(),
                params_.fake_label_prefixes.size(), params_.depth_min_m,
                params_.depth_max_m);
    return true;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node_->get_logger(), "梅林预选赛 KFS 视觉初始化异常: %s",
                 e.what());
    vision_.reset();
    return false;
  }
}

void MfPreselectionFlowAction::releaseRuntime() {
  publishStop();
  if (vision_) {
    vision_->stop();
    vision_.reset();
  }
  command_generation_.fetch_add(1, std::memory_order_relaxed);
  feedback_sub_.reset();
  odom_sub_.reset();
  send_client_.reset();
  cmd_pub_.reset();
  node_ = nullptr;
  phase_ = Phase::Done;
  detection_active_ = false;
  direct_exit_move_active_ = false;
  pending_grab_commit_ = false;
}

void MfPreselectionFlowAction::normalizeParams() {
  params_.vision_config_file = resolveVisionConfig(params_.vision_config_file);
  params_.r2_target_label_prefixes =
      sanitized(std::move(params_.r2_target_label_prefixes));
  params_.r2_target_labels = sanitized(std::move(params_.r2_target_labels));
  params_.r1_blocking_labels = sanitized(std::move(params_.r1_blocking_labels));
  params_.r1_blocking_label_prefixes =
      sanitized(std::move(params_.r1_blocking_label_prefixes));
  params_.fake_label_prefixes = sanitized(std::move(params_.fake_label_prefixes));
  params_.fake_labels = sanitized(std::move(params_.fake_labels));
  params_.depth_min_m = std::max(0.0, params_.depth_min_m);
  params_.depth_max_m = std::max(params_.depth_min_m, params_.depth_max_m);
  params_.detect_seen_stable_frames =
      std::max(1, params_.detect_seen_stable_frames);
  params_.detect_lost_stable_frames =
      std::max(1, params_.detect_lost_stable_frames);
  params_.entry_detect_timeout_s =
      std::max(kMinTimeoutS, params_.entry_detect_timeout_s);
  params_.scan_detect_timeout_s =
      std::max(kMinTimeoutS, params_.scan_detect_timeout_s);
  params_.max_pickup_count = std::max(0, params_.max_pickup_count);
  params_.grab_settle_s = std::max(0.0, params_.grab_settle_s);
  params_.command_timeout_s = std::max(kMinTimeoutS, params_.command_timeout_s);
  params_.entry_probe_left_distance_m =
      std::max(0.0, std::abs(params_.entry_probe_left_distance_m));
  params_.entry_probe_right_sweep_distance_m =
      std::max(0.0, std::abs(params_.entry_probe_right_sweep_distance_m));
  params_.entry_probe_return_distance_m =
      std::max(0.0, std::abs(params_.entry_probe_return_distance_m));
  params_.lateral_probe_speed_mps =
      std::max(kMinSpeed, std::abs(params_.lateral_probe_speed_mps));
  params_.move_tolerance_m = std::max(0.0, std::abs(params_.move_tolerance_m));
  params_.move_timeout_s = std::max(kMinTimeoutS, params_.move_timeout_s);
  params_.direct_exit_drive_distance_m =
      std::max(0.0, std::abs(params_.direct_exit_drive_distance_m));
  params_.direct_exit_drive_speed_mps =
      std::max(kMinSpeed, std::abs(params_.direct_exit_drive_speed_mps));
  params_.turn_kp = std::max(0.0, params_.turn_kp);
  params_.turn_max_speed_radps =
      std::max(0.0, std::abs(params_.turn_max_speed_radps));
  params_.turn_tolerance_deg = std::max(0.0, params_.turn_tolerance_deg);
  params_.turn_stable_ticks = std::max(1, params_.turn_stable_ticks);
  params_.turn_timeout_s = std::max(kMinTimeoutS, params_.turn_timeout_s);
  params_.odom_timeout_s = std::max(kMinTimeoutS, params_.odom_timeout_s);
  params_.heading_kp = std::max(0.0, params_.heading_kp);
  params_.heading_max_speed_radps =
      std::max(0.0, std::abs(params_.heading_max_speed_radps));
  params_.path_r1_lost_wait_timeout_s =
      std::max(0.0, params_.path_r1_lost_wait_timeout_s);

  stair_params_.command_timeout_s =
      std::max(kMinTimeoutS, stair_params_.command_timeout_s);
  stair_params_.command_rate_hz = std::max(1.0, stair_params_.command_rate_hz);
  stair_params_.climb_drive_speed_mps =
      std::abs(stair_params_.climb_drive_speed_mps);
  stair_params_.climb_rear_drive_fast_speed_mps =
      std::abs(stair_params_.climb_rear_drive_fast_speed_mps);
  if (stair_params_.climb_rear_drive_fast_speed_mps <= 0.0) {
    stair_params_.climb_rear_drive_fast_speed_mps =
        stair_params_.climb_drive_speed_mps;
  }
  stair_params_.climb_rear_drive_slow_speed_mps =
      std::min(std::abs(stair_params_.climb_rear_drive_slow_speed_mps),
               stair_params_.climb_rear_drive_fast_speed_mps);
  stair_params_.descend_drive_speed_mps =
      std::abs(stair_params_.descend_drive_speed_mps);
}

BT::NodeStatus MfPreselectionFlowAction::fail(const std::string &reason) {
  if (node_) {
    RCLCPP_ERROR(node_->get_logger(), "梅林预选赛失败: %s", reason.c_str());
  }
  if (config().blackboard) {
    config().blackboard->set("mf_preselect_error", reason);
  }
  releaseRuntime();
  return BT::NodeStatus::FAILURE;
}

void MfPreselectionFlowAction::publishStop() { publishTwist(0.0, 0.0, 0.0); }

void MfPreselectionFlowAction::publishTwist(double vx, double vy, double wz) {
  if (!cmd_pub_) {
    return;
  }
  TwistMsg msg;
  msg.linear.x = vx;
  msg.linear.y = vy;
  msg.angular.z = wz;
  cmd_pub_->publish(msg);
  if (node_) {
    last_cmd_publish_ = node_->now();
    has_last_cmd_publish_ = true;
  }
}

bool MfPreselectionFlowAction::odomReady() const {
  if (!has_odom_) {
    return false;
  }
  const auto age_s = std::chrono::duration<double>(
                         std::chrono::steady_clock::now() - last_odom_tp_)
                         .count();
  return age_s <= params_.odom_timeout_s;
}

void MfPreselectionFlowAction::handleOdom(const OdomMsg::SharedPtr msg) {
  if (!msg) {
    return;
  }
  odom_x_ = msg->pose.pose.position.x;
  odom_y_ = msg->pose.pose.position.y;
  odom_yaw_ = yawFromQuaternion(msg->pose.pose.orientation);
  has_odom_ = true;
  last_odom_tp_ = std::chrono::steady_clock::now();
}

double MfPreselectionFlowAction::yawFromQuaternion(
    const geometry_msgs::msg::Quaternion &q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double MfPreselectionFlowAction::normalizeAngle(double angle_rad) {
  while (angle_rad > kPi) {
    angle_rad -= 2.0 * kPi;
  }
  while (angle_rad < -kPi) {
    angle_rad += 2.0 * kPi;
  }
  return angle_rad;
}

double MfPreselectionFlowAction::headingAngularZ(double target_yaw_rad) const {
  if (!odomReady()) {
    return 0.0;
  }
  const double error = normalizeAngle(target_yaw_rad - odom_yaw_);
  const double raw = params_.heading_kp * error;
  return std::clamp(raw, -params_.heading_max_speed_radps,
                    params_.heading_max_speed_radps);
}

std::optional<MfPreselectionFlowAction::Observation>
MfPreselectionFlowAction::findR2Target() {
  if (!canPickup()) {
    return std::nullopt;
  }
  return findTarget(params_.r2_target_labels, params_.r2_target_label_prefixes);
}

std::optional<MfPreselectionFlowAction::Observation>
MfPreselectionFlowAction::findR1BlockingTarget() {
  return findTarget(params_.r1_blocking_labels,
                    params_.r1_blocking_label_prefixes);
}

std::optional<MfPreselectionFlowAction::Observation>
MfPreselectionFlowAction::findFakeTarget() {
  return findTarget(params_.fake_labels, params_.fake_label_prefixes);
}

std::optional<MfPreselectionFlowAction::Observation>
MfPreselectionFlowAction::findTarget(const std::vector<std::string> &exact,
                                     const std::vector<std::string> &prefixes) {
  if (!vision_ || !vision_->isRunning()) {
    return std::nullopt;
  }
  rc26_vision::VisionInferenceManager::FrameSnapshot snapshot;
  if (!vision_->getLatestFrameSnapshot(snapshot) || !snapshot.has_display ||
      !snapshot.has_depth) {
    return std::nullopt;
  }

  const rc26_vision::Detection *best = nullptr;
  for (const auto &det : snapshot.detections) {
    const std::string name =
        det.class_name.empty() ? std::to_string(det.class_id) : det.class_name;
    if (!MfPreselectionLogicResult::labelMatches(name, exact, prefixes)) {
      continue;
    }
    if (!best || det.score > best->score) {
      best = &det;
    }
  }
  if (!best) {
    return std::nullopt;
  }

  const int cx = static_cast<int>((best->x1 + best->x2) * 0.5F);
  const int cy = static_cast<int>((best->y1 + best->y2) * 0.5F);
  rc26_vision::DepthRoiSamplerConfig depth_config;
  depth_config.roi_size = 7;
  depth_config.min_valid_count = 10;
  depth_config.min_depth_m = params_.depth_min_m;
  depth_config.max_depth_m = params_.depth_max_m;
  const auto sampled =
      rc26_vision::sampleMedianDepth(snapshot.depth, cx, cy, depth_config);
  if (!sampled.has_value()) {
    return std::nullopt;
  }

  Observation obs;
  obs.label =
      best->class_name.empty() ? std::to_string(best->class_id) : best->class_name;
  obs.distance_m = *sampled;
  obs.score = best->score;
  obs.sequence = snapshot.display_sequence;
  return obs;
}

std::optional<int64_t> MfPreselectionFlowAction::latestVisionSequence() const {
  if (!vision_ || !vision_->isRunning()) {
    return std::nullopt;
  }
  rc26_vision::VisionInferenceManager::FrameSnapshot snapshot;
  if (!vision_->getLatestFrameSnapshot(snapshot) || !snapshot.has_display ||
      snapshot.display_sequence <= 0) {
    return std::nullopt;
  }
  return snapshot.display_sequence;
}

bool MfPreselectionFlowAction::canPickup() const {
  return MfPreselectionLogicResult::canPickup(pickup_count_,
                                              params_.max_pickup_count);
}

void MfPreselectionFlowAction::rememberPickupSource(
    MfPreselectionPickupSource source) {
  if (source != MfPreselectionPickupSource::None) {
    pickup_source_ = source;
  }
  if (config().blackboard) {
    config().blackboard->set("mf_preselect_pickup_source",
                             std::string(sourceName(pickup_source_)));
  }
}

void MfPreselectionFlowAction::writeBlackboardState(const std::string &state) {
  if (!config().blackboard) {
    return;
  }
  config().blackboard->set("mf_preselect_state", state);
  config().blackboard->set("mf_preselect_pickup_count", pickup_count_);
  config().blackboard->set("mf_preselect_current_grid", current_grid_);
}

const char *MfPreselectionFlowAction::detectModeText(DetectMode mode) {
  switch (mode) {
  case DetectMode::Entry2:
    return "入口2号阶梯检测";
  case DetectMode::Stair1:
    return "入口1号阶梯检测";
  case DetectMode::Stair3:
    return "入口3号阶梯检测";
  case DetectMode::RowFront:
    return "当前行前方检测";
  case DetectMode::Scan:
    return "周身扫描检测";
  case DetectMode::Row4Fake:
    return "第四行假KFS检测";
  case DetectMode::TransitionObserve:
    return "高低阶梯切换前方观察";
  }
  return "未知检测";
}

const char *MfPreselectionFlowAction::stairModeText(StairMode mode) {
  return mode == StairMode::Climb ? "上阶梯" : "下阶梯";
}

const char *MfPreselectionFlowAction::wheelEventText(WheelEvent event) {
  switch (event) {
  case WheelEvent::FrontFirst:
    return "前轮第一激光高度突变";
  case WheelEvent::FrontSecond:
    return "前轮第二激光高度突变";
  case WheelEvent::Rear:
    return "后轮激光高度突变";
  }
  return "未知激光事件";
}

void MfPreselectionFlowAction::beginDetection(DetectMode mode,
                                              double timeout_s) {
  detect_mode_ = mode;
  if (mode == DetectMode::Scan &&
      (phase_ == Phase::RowScanDetectLeft ||
       phase_ == Phase::RowScanDetectBack)) {
    active_detection_phase_ = phase_;
  } else {
    active_detection_phase_ = Phase::Done;
  }
  resetDetectionCounters();
  detection_active_ = true;
  if (node_) {
    phase_start_ = node_->now();
  }
  switch (mode) {
  case DetectMode::Entry2:
    phase_ = Phase::EntryDetectStair2;
    writeBlackboardState("entry_detect_stair2");
    break;
  case DetectMode::Stair1:
    phase_ = Phase::EntryDetectStair1;
    writeBlackboardState("entry_detect_stair1");
    break;
  case DetectMode::Stair3:
    phase_ = Phase::EntryDetectStair3;
    writeBlackboardState("entry_detect_stair3");
    break;
  case DetectMode::RowFront:
    phase_ = Phase::RowFrontDetect;
    writeBlackboardState("row_front_detect");
    break;
  case DetectMode::Scan:
    if (active_detection_phase_ != Phase::RowScanDetectLeft &&
        active_detection_phase_ != Phase::RowScanDetectBack) {
      active_detection_phase_ = Phase::RowScanDetectLeft;
    }
    phase_ = active_detection_phase_;
    writeBlackboardState("row_scan_detect");
    break;
  case DetectMode::Row4Fake:
    phase_ = Phase::Row4DetectFake;
    writeBlackboardState("row4_fake_detect");
    break;
  case DetectMode::TransitionObserve:
    phase_ = Phase::TransitionObserve;
    writeBlackboardState("transition_observe");
    break;
  }
  if (node_) {
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛进入检测阶段：%s，grid=%d，直出模式=%s，已夹取=%d/%d，超时=%.2fs",
                detectModeText(mode), current_grid_,
                direct_exit_mode_ ? "是" : "否", pickup_count_,
                params_.max_pickup_count, timeout_s);
  }
}

BT::NodeStatus MfPreselectionFlowAction::tickDetection() {
  if (!detection_active_) {
    if (phase_ == Phase::RowScanDetectLeft) {
      beginDetection(DetectMode::Scan, params_.scan_detect_timeout_s);
    } else if (phase_ == Phase::RowScanDetectBack) {
      beginDetection(DetectMode::Scan, params_.scan_detect_timeout_s);
    } else if (phase_ == Phase::RowFrontDetect) {
      beginDetection(DetectMode::RowFront, params_.scan_detect_timeout_s);
    } else if (phase_ == Phase::TransitionObserve) {
      beginDetection(DetectMode::TransitionObserve, params_.scan_detect_timeout_s);
    } else if (phase_ == Phase::Row4DetectFake) {
      beginDetection(DetectMode::Row4Fake, params_.scan_detect_timeout_s);
    }
  }
  publishStop();
  const double elapsed =
      node_ ? (node_->now() - phase_start_).seconds() : 0.0;
  const double timeout =
      (detect_mode_ == DetectMode::Entry2) ? params_.entry_detect_timeout_s
                                           : params_.scan_detect_timeout_s;

  if (detect_mode_ == DetectMode::Row4Fake) {
    const auto fake = findFakeTarget();
    if (fake.has_value()) {
      RCLCPP_WARN(node_->get_logger(),
                  "梅林预选赛第四行转向后检测到假KFS：label=%s distance=%.3fm，执行180度转向后下阶梯离场",
                  fake->label.c_str(), fake->distance_m);
      row4_fake_detected_ = true;
      detection_active_ = false;
      phase_ = Phase::Row4FakeTurnBack;
      return BT::NodeStatus::RUNNING;
    }
    if (elapsed >= timeout) {
      RCLCPP_INFO(node_->get_logger(),
                  "梅林预选赛第四行转向后未检测到假KFS，继续常规 grid11 -> grid12 离场");
      row4_fake_detected_ = false;
      detection_active_ = false;
      phase_ = Phase::TransitionTurn;
      return BT::NodeStatus::RUNNING;
    }
    return BT::NodeStatus::RUNNING;
  }

  if (detect_mode_ != DetectMode::Scan) {
    if ((detect_mode_ == DetectMode::RowFront ||
         detect_mode_ == DetectMode::TransitionObserve) &&
        guardPathObstacles()) {
      return BT::NodeStatus::RUNNING;
    }
  }

  const auto r2 = findR2Target();
  if (r2.has_value()) {
    if (r2->sequence != last_detection_sequence_) {
      last_detection_sequence_ = r2->sequence;
      ++detect_seen_count_;
      detect_lost_count_ = 0;
    }
    if (detect_seen_count_ >= params_.detect_seen_stable_frames) {
      RCLCPP_INFO(node_->get_logger(),
                  "梅林预选赛检测到R2 KFS：阶段=%s label=%s distance=%.3fm score=%.3f stable=%d/%d",
                  detectModeText(detect_mode_), r2->label.c_str(),
                  r2->distance_m, r2->score, detect_seen_count_,
                  params_.detect_seen_stable_frames);
      switch (detect_mode_) {
      case DetectMode::Entry2:
        beginGrab(true, MfPreselectionPickupSource::Stair2,
                  Phase::EntryPrepareClimb);
        break;
      case DetectMode::Stair1:
        beginGrab(true, MfPreselectionPickupSource::Stair1,
                  Phase::EntryReturnFromStair1);
        break;
      case DetectMode::Stair3:
        beginGrab(true, MfPreselectionPickupSource::Stair3,
                  Phase::EntryReturnFromStair3);
        break;
      case DetectMode::RowFront:
      case DetectMode::Scan:
        direct_exit_mode_ = true;
        beginGrab(true, MfPreselectionPickupSource::None,
                  Phase::AfterEntry);
        break;
      case DetectMode::TransitionObserve:
        beginGrab(transition_high_side_, MfPreselectionPickupSource::None,
                  Phase::TransitionStair);
        break;
      case DetectMode::Row4Fake:
        break;
      }
      detection_active_ = false;
      return BT::NodeStatus::RUNNING;
    }
    return BT::NodeStatus::RUNNING;
  }

  if (detect_mode_ == DetectMode::RowFront && current_grid_ != 11) {
    const auto fake = findFakeTarget();
    if (fake.has_value()) {
      RCLCPP_WARN(node_->get_logger(),
                  "梅林预选赛当前行前方检测到假KFS：grid=%d label=%s distance=%.3fm，进入假KFS避障",
                  current_grid_, fake->label.c_str(), fake->distance_m);
      phase_ = Phase::FakeAvoidTurn;
      return BT::NodeStatus::RUNNING;
    }
  }

  const auto latest_sequence = latestVisionSequence();
  if ((!latest_sequence.has_value() ||
       *latest_sequence == last_detection_sequence_) &&
      elapsed < timeout) {
    return BT::NodeStatus::RUNNING;
  }
  if (latest_sequence.has_value()) {
    last_detection_sequence_ = *latest_sequence;
  }
  ++detect_lost_count_;
  detect_seen_count_ = 0;
  if (elapsed < timeout &&
      detect_lost_count_ < params_.detect_lost_stable_frames) {
    return BT::NodeStatus::RUNNING;
  }

  switch (detect_mode_) {
  case DetectMode::Entry2:
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛入口2号未发现R2 KFS，准备发送高抬升命令并横移探测1/3号阶梯");
    phase_ = Phase::EntryHighRaise;
    break;
  case DetectMode::Stair1:
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛入口1号未发现R2 KFS，准备横移到3号阶梯探测");
    phase_ = Phase::EntryMoveRightToStair3;
    break;
  case DetectMode::Stair3:
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛入口3号未发现R2 KFS，准备返回2号入口上阶梯");
    phase_ = Phase::EntryReturnFromStair3;
    break;
  case DetectMode::RowFront:
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛当前行前方未发现可处理目标：grid=%d，直出模式=%s",
                current_grid_, direct_exit_mode_ ? "是" : "否");
    if (direct_exit_mode_) {
      phase_ = (current_grid_ == 11) ? Phase::Row4ForcedTurn
                                    : Phase::TransitionTurn;
    } else if (current_grid_ == 2) {
      phase_ = Phase::TransitionTurn;
    } else if (current_grid_ == 11) {
      phase_ = Phase::Row4ForcedTurn;
    } else {
      phase_ = Phase::RowScanTurnLeft;
    }
    break;
  case DetectMode::Scan:
    if (active_detection_phase_ == Phase::RowScanDetectLeft) {
      RCLCPP_INFO(node_->get_logger(),
                  "梅林预选赛周身扫描左侧未发现R2 KFS，继续背向扫描");
      phase_ = Phase::RowScanTurnBack;
    } else {
      RCLCPP_INFO(node_->get_logger(),
                  "梅林预选赛周身扫描未发现R2 KFS，重新朝向出口方向并继续推进");
      phase_ = Phase::RowAlignExit;
    }
    break;
  case DetectMode::TransitionObserve:
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛高低阶梯切换前方观察未发现R2 KFS，继续执行格间台阶动作");
    phase_ = Phase::TransitionStair;
    break;
  case DetectMode::Row4Fake:
    break;
  }
  detection_active_ = false;
  return BT::NodeStatus::RUNNING;
}

void MfPreselectionFlowAction::resetDetectionCounters() {
  detect_seen_count_ = 0;
  detect_lost_count_ = 0;
  last_detection_sequence_ = 0;
}

void MfPreselectionFlowAction::beginMechanismCommand(
    uint8_t command_id, std::string label, int done_feedback_id,
    Phase next_phase, std::string failure_reason) {
  command_generation_.fetch_add(1, std::memory_order_relaxed);
  command_id_ = command_id;
  command_label_ = std::move(label);
  command_done_feedback_id_ = done_feedback_id;
  command_next_phase_ = next_phase;
  command_failure_reason_ = std::move(failure_reason);
  command_response_seen_ = false;
  command_accepted_ = false;
  command_seq_ = -1;
  command_done_seen_ = done_feedback_id < 0;
  command_sent_ = false;
  command_waiting_service_logged_ = false;
  command_ack_logged_ = false;
  command_waiting_done_logged_ = false;
  if (node_) {
    phase_start_ = node_->now();
    if (done_feedback_id >= 0) {
      RCLCPP_INFO(node_->get_logger(),
                  "梅林预选赛准备下发机构命令：%s(0x%02X)，等待完成反馈=0x%02X",
                  command_label_.c_str(),
                  static_cast<unsigned int>(command_id_),
                  static_cast<unsigned int>(done_feedback_id));
    } else {
      RCLCPP_INFO(node_->get_logger(),
                  "梅林预选赛准备下发机构命令：%s(0x%02X)，仅等待ACK",
                  command_label_.c_str(),
                  static_cast<unsigned int>(command_id_));
    }
  }
  phase_ = Phase::MechanismCommand;
}

BT::NodeStatus MfPreselectionFlowAction::tickMechanismCommand() {
  publishStop();
  if (!node_ || !send_client_) {
    return fail(command_failure_reason_);
  }
  if (command_response_seen_.load(std::memory_order_relaxed)) {
    if (!command_accepted_.load(std::memory_order_relaxed)) {
      return fail(command_failure_reason_);
    }
    if (!command_ack_logged_) {
      RCLCPP_INFO(node_->get_logger(),
                  "梅林预选赛机构命令ACK成功：%s(0x%02X)，seq=%d",
                  command_label_.c_str(),
                  static_cast<unsigned int>(command_id_),
                  command_seq_.load(std::memory_order_relaxed));
      command_ack_logged_ = true;
    }
    if (command_done_seen_.load(std::memory_order_relaxed)) {
      if (pending_grab_commit_) {
        ++pickup_count_;
        entry_pickup_done_ =
            entry_pickup_done_ ||
            pending_grab_source_ != MfPreselectionPickupSource::None;
        rememberPickupSource(pending_grab_source_);
        if (config().blackboard) {
          config().blackboard->set("mf_preselect_pickup_count", pickup_count_);
        }
        RCLCPP_INFO(node_->get_logger(),
                    "梅林预选赛R2 KFS夹取计数已更新：%d/%d，来源=%s",
                    pickup_count_, params_.max_pickup_count,
                    sourceName(pickup_source_));
        pending_grab_commit_ = false;
        pending_grab_source_ = MfPreselectionPickupSource::None;
      }
      if (command_next_phase_ == Phase::StairPrimitive) {
        if (stair_phase_ == StairPhase::ClimbSendFrontExtend) {
          stair_phase_ = StairPhase::ClimbHoldAfterFrontExtend;
        } else if (stair_phase_ == StairPhase::ClimbSendRearRetract) {
          stair_phase_ = StairPhase::ClimbHoldAfterRearRetract;
        } else if (stair_phase_ == StairPhase::DescendSendRearExtend) {
          stair_phase_ = StairPhase::DescendHoldAfterRearExtend;
        } else if (stair_phase_ == StairPhase::DescendSendFrontRetract) {
          stair_phase_ = StairPhase::DescendHoldAfterFrontRetract;
        }
      }
      RCLCPP_INFO(node_->get_logger(),
                  "梅林预选赛机构命令完成：%s，进入下一阶段",
                  command_label_.c_str());
      if (command_next_phase_ == Phase::ZeroHold) {
        if (node_) {
          phase_start_ = node_->now();
          RCLCPP_INFO(node_->get_logger(),
                      "梅林预选赛开始零速等待：%s，duration=%.2fs",
                      zero_hold_label_.c_str(), zero_hold_duration_s_);
        }
        phase_ = Phase::ZeroHold;
        return BT::NodeStatus::RUNNING;
      }
      phase_ = command_next_phase_;
      return BT::NodeStatus::RUNNING;
    }
    if (!command_waiting_done_logged_) {
      RCLCPP_INFO(node_->get_logger(),
                  "梅林预选赛机构命令ACK后等待完成反馈：%s，feedback=0x%02X，seq=%d",
                  command_label_.c_str(),
                  static_cast<unsigned int>(command_done_feedback_id_),
                  command_seq_.load(std::memory_order_relaxed));
      command_waiting_done_logged_ = true;
    }
  }
  if ((node_->now() - phase_start_).seconds() > params_.command_timeout_s) {
    return fail(command_failure_reason_);
  }
  if (!command_sent_) {
    if (!send_client_->service_is_ready()) {
      if (!command_waiting_service_logged_) {
        RCLCPP_WARN(node_->get_logger(),
                    "梅林预选赛等待机构命令服务可用：%s，service=%s",
                    command_label_.c_str(), params_.send_command_service.c_str());
        command_waiting_service_logged_ = true;
      }
      return BT::NodeStatus::RUNNING;
    }
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛已发送机构命令请求：%s(0x%02X)",
                command_label_.c_str(),
                static_cast<unsigned int>(command_id_));
    auto request = std::make_shared<SendCommandSrv::Request>();
    request->command_id = command_id_;
    request->payload.clear();
    const uint64_t token =
        command_generation_.load(std::memory_order_relaxed);
    send_client_->async_send_request(
        request, [this, token](rclcpp::Client<SendCommandSrv>::SharedFuture future) {
          if (token != command_generation_.load(std::memory_order_relaxed)) {
            return;
          }
          try {
            const auto response = future.get();
            command_accepted_.store(response && response->accepted,
                                    std::memory_order_relaxed);
            if (response) {
              command_seq_.store(static_cast<int>(response->seq),
                                 std::memory_order_relaxed);
            }
          } catch (...) {
            command_accepted_.store(false, std::memory_order_relaxed);
          }
          command_response_seen_.store(true, std::memory_order_relaxed);
        });
    command_sent_ = true;
  }
  return BT::NodeStatus::RUNNING;
}

void MfPreselectionFlowAction::beginCommandPair(
    uint8_t first_id, std::string first_label, uint8_t second_id,
    std::string second_label, Phase next_phase, std::string failure_reason) {
  command_generation_.fetch_add(1, std::memory_order_relaxed);
  command_pair_[0].command_id = first_id;
  command_pair_[0].label = std::move(first_label);
  command_pair_[0].sent = false;
  command_pair_[0].response_seen = false;
  command_pair_[0].accepted = false;
  command_pair_ack_logged_[0] = false;
  command_pair_[1].command_id = second_id;
  command_pair_[1].label = std::move(second_label);
  command_pair_[1].sent = false;
  command_pair_[1].response_seen = false;
  command_pair_[1].accepted = false;
  command_pair_ack_logged_[1] = false;
  command_pair_active_ = true;
  command_pair_waiting_service_logged_ = false;
  command_next_phase_ = next_phase;
  command_pair_failure_reason_ = std::move(failure_reason);
  if (node_) {
    phase_start_ = node_->now();
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛准备并发下发机构命令：%s(0x%02X) + %s(0x%02X)",
                command_pair_[0].label.c_str(),
                static_cast<unsigned int>(command_pair_[0].command_id),
                command_pair_[1].label.c_str(),
                static_cast<unsigned int>(command_pair_[1].command_id));
  }
  phase_ = Phase::MechanismCommand;
}

BT::NodeStatus MfPreselectionFlowAction::tickCommandPair() {
  publishStop();
  if (!node_ || !send_client_ || !command_pair_active_) {
    return fail(command_pair_failure_reason_);
  }
  for (const auto &slot : command_pair_) {
    if (slot.response_seen.load(std::memory_order_relaxed) &&
        !slot.accepted.load(std::memory_order_relaxed)) {
      return fail(command_pair_failure_reason_);
    }
  }
  const bool first_done =
      command_pair_[0].response_seen.load(std::memory_order_relaxed) &&
      command_pair_[0].accepted.load(std::memory_order_relaxed);
  const bool second_done =
      command_pair_[1].response_seen.load(std::memory_order_relaxed) &&
      command_pair_[1].accepted.load(std::memory_order_relaxed);
  for (std::size_t index = 0; index < 2; ++index) {
    if (!command_pair_ack_logged_[index] &&
        command_pair_[index].response_seen.load(std::memory_order_relaxed) &&
        command_pair_[index].accepted.load(std::memory_order_relaxed)) {
      RCLCPP_INFO(node_->get_logger(),
                  "梅林预选赛并发机构命令ACK成功：%s(0x%02X)",
                  command_pair_[index].label.c_str(),
                  static_cast<unsigned int>(command_pair_[index].command_id));
      command_pair_ack_logged_[index] = true;
    }
  }
  if (first_done && second_done) {
    command_pair_active_ = false;
    if (command_next_phase_ == Phase::StairPrimitive) {
      if (stair_phase_ == StairPhase::ClimbSendFrontRetractAndRearExtend) {
        stair_phase_ = StairPhase::ClimbHoldAfterFrontRetractAndRearExtend;
      } else if (stair_phase_ ==
                 StairPhase::DescendSendRearRetractAndFrontExtend) {
        stair_phase_ = StairPhase::DescendHoldAfterRearRetractAndFrontExtend;
      }
    }
    RCLCPP_INFO(node_->get_logger(), "梅林预选赛并发机构命令均ACK成功，进入下一阶段");
    phase_ = command_next_phase_;
    return BT::NodeStatus::RUNNING;
  }
  if ((node_->now() - phase_start_).seconds() > params_.command_timeout_s) {
    return fail(command_pair_failure_reason_);
  }
  if (!send_client_->service_is_ready()) {
    if (!command_pair_waiting_service_logged_) {
      RCLCPP_WARN(node_->get_logger(),
                  "梅林预选赛等待机构命令服务可用：并发命令，service=%s",
                  params_.send_command_service.c_str());
      command_pair_waiting_service_logged_ = true;
    }
    return BT::NodeStatus::RUNNING;
  }

  const uint64_t token =
      command_generation_.load(std::memory_order_relaxed);
  for (std::size_t index = 0; index < 2; ++index) {
    auto &slot = command_pair_[index];
    if (slot.sent) {
      continue;
    }
    auto request = std::make_shared<SendCommandSrv::Request>();
    request->command_id = slot.command_id;
    request->payload.clear();
    slot.sent = true;
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛已发送并发机构命令请求：%s(0x%02X)",
                slot.label.c_str(), static_cast<unsigned int>(slot.command_id));
    send_client_->async_send_request(
        request, [this, token, index](rclcpp::Client<SendCommandSrv>::SharedFuture future) {
          if (token != command_generation_.load(std::memory_order_relaxed)) {
            return;
          }
          bool accepted = false;
          try {
            const auto response = future.get();
            accepted = response && response->accepted;
          } catch (...) {
            accepted = false;
          }
          command_pair_[index].accepted.store(accepted, std::memory_order_relaxed);
          command_pair_[index].response_seen.store(true, std::memory_order_relaxed);
        });
  }
  return BT::NodeStatus::RUNNING;
}

void MfPreselectionFlowAction::beginMoveRelative(double vx, double vy,
                                                 double distance_m,
                                                 Phase next_phase,
                                                 std::string label) {
  move_vx_ = vx;
  move_vy_ = vy;
  move_distance_m_ = std::max(0.0, distance_m);
  move_next_phase_ = next_phase;
  move_label_ = std::move(label);
  move_start_captured_ = false;
  move_waiting_odom_logged_ = false;
  if (node_) {
    phase_start_ = node_->now();
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛开始相对移动：%s，vx=%.3f vy=%.3f distance=%.3fm",
                move_label_.c_str(), move_vx_, move_vy_, move_distance_m_);
  }
  phase_ = Phase::MoveRelative;
}

BT::NodeStatus MfPreselectionFlowAction::tickMoveRelative() {
  if (!node_) {
    return fail("move_runtime_missing");
  }
  if (guardPathObstacles()) {
    phase_start_ = node_->now();
    return BT::NodeStatus::RUNNING;
  }
  if (!odomReady()) {
    publishStop();
    if (!move_waiting_odom_logged_) {
      RCLCPP_WARN(node_->get_logger(),
                  "梅林预选赛相对移动等待odom新鲜：%s，odom_topic=%s",
                  move_label_.c_str(), params_.odom_topic.c_str());
      move_waiting_odom_logged_ = true;
    }
    return BT::NodeStatus::RUNNING;
  }
  if (!move_start_captured_) {
    move_start_x_ = odom_x_;
    move_start_y_ = odom_y_;
    move_start_yaw_ = odom_yaw_;
    move_start_captured_ = true;
    if (node_) {
      phase_start_ = node_->now();
      RCLCPP_INFO(node_->get_logger(),
                  "梅林预选赛相对移动已捕获起点：%s，x=%.3f y=%.3f yaw=%.3f",
                  move_label_.c_str(), move_start_x_, move_start_y_,
                  move_start_yaw_);
    }
  }
  const double dx = odom_x_ - move_start_x_;
  const double dy = odom_y_ - move_start_y_;
  const double traveled = std::hypot(dx, dy);
  if (traveled + params_.move_tolerance_m >= move_distance_m_) {
    publishStop();
    clearPathR1Wait();
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛相对移动完成：%s，已行驶=%.3fm 目标=%.3fm",
                move_label_.c_str(), traveled, move_distance_m_);
    phase_ = move_next_phase_;
    if (phase_ == Phase::EntryDetectStair1) {
      beginDetection(DetectMode::Stair1, params_.scan_detect_timeout_s);
    } else if (phase_ == Phase::EntryDetectStair3) {
      beginDetection(DetectMode::Stair3, params_.scan_detect_timeout_s);
    } else if (phase_ == Phase::FinalStop) {
      beginZeroHold(0.5, Phase::Done, "final_stop_hold");
    }
    return BT::NodeStatus::RUNNING;
  }
  if ((node_->now() - phase_start_).seconds() > params_.move_timeout_s) {
    return fail("move_timeout_" + move_label_);
  }
  publishTwist(move_vx_, move_vy_, headingAngularZ(move_start_yaw_));
  return BT::NodeStatus::RUNNING;
}

void MfPreselectionFlowAction::beginDirectExitDrive() {
  if (node_) {
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛进入专用直行离场兜底：distance=%.3fm speed=%.3fm/s",
                params_.direct_exit_drive_distance_m,
                params_.direct_exit_drive_speed_mps);
  }
  beginMoveRelative(params_.direct_exit_drive_speed_mps, 0.0,
                    params_.direct_exit_drive_distance_m, Phase::FinalStop,
                    "direct_exit");
  phase_ = Phase::DirectExitDrive;
  direct_exit_move_active_ = true;
}

BT::NodeStatus MfPreselectionFlowAction::tickDirectExitDrive() {
  if (!direct_exit_move_active_) {
    beginDirectExitDrive();
  }
  if (guardPathObstacles()) {
    phase_start_ = node_->now();
    return BT::NodeStatus::RUNNING;
  }
  if (canPickup() && findR2Target().has_value()) {
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛直行离场途中检测到R2 KFS，先夹取后继续离场");
    beginGrab(true, MfPreselectionPickupSource::None, Phase::DirectExitDrive);
    return BT::NodeStatus::RUNNING;
  }
  if (current_grid_ != 11 && findFakeTarget().has_value()) {
    RCLCPP_WARN(node_->get_logger(),
                "梅林预选赛直行离场途中检测到假KFS，切入假KFS避障");
    direct_exit_move_active_ = false;
    phase_ = Phase::FakeAvoidTurn;
    return BT::NodeStatus::RUNNING;
  }
  return tickMoveRelative();
}

bool MfPreselectionFlowAction::guardPathObstacles() {
  const auto r1 = findR1BlockingTarget();
  if (r1.has_value()) {
    publishStop();
    writeBlackboardState("waiting_r1_blocker");
    if (!path_r1_waiting_) {
      path_r1_waiting_ = true;
      path_r1_lost_count_ = 0;
      path_r1_last_sequence_ = r1->sequence;
      if (node_) {
        path_r1_wait_start_ = node_->now();
      }
      RCLCPP_WARN(node_->get_logger(),
                  "梅林预选赛路径前方遇到R1阻挡，原地等待人工机器人处理：label=%s distance=%.3fm",
                  r1->label.c_str(), r1->distance_m);
    } else if (r1->sequence != path_r1_last_sequence_) {
      path_r1_last_sequence_ = r1->sequence;
      path_r1_lost_count_ = 0;
    }
    return true;
  }
  if (!path_r1_waiting_) {
    return false;
  }
  if (params_.path_r1_lost_wait_timeout_s > 0.0 && node_ &&
      (node_->now() - path_r1_wait_start_).seconds() >
          params_.path_r1_lost_wait_timeout_s) {
    RCLCPP_WARN(node_->get_logger(),
                "梅林预选赛R1等待达到配置超时，按参数放行：timeout=%.2fs",
                params_.path_r1_lost_wait_timeout_s);
    clearPathR1Wait();
    return false;
  }
  const auto latest_sequence = latestVisionSequence();
  if (!latest_sequence.has_value() ||
      *latest_sequence == path_r1_last_sequence_) {
    publishStop();
    writeBlackboardState("waiting_r1_blocker");
    return true;
  }
  path_r1_last_sequence_ = *latest_sequence;
  ++path_r1_lost_count_;
  if (path_r1_lost_count_ >= params_.detect_lost_stable_frames) {
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛R1阻挡已连续丢失%d帧，解除等待继续流程",
                path_r1_lost_count_);
    clearPathR1Wait();
    return false;
  }
  publishStop();
  writeBlackboardState("waiting_r1_blocker");
  return true;
}

void MfPreselectionFlowAction::clearPathR1Wait() {
  path_r1_waiting_ = false;
  path_r1_lost_count_ = 0;
  path_r1_last_sequence_ = 0;
}

void MfPreselectionFlowAction::beginTurnYaw(double target_yaw_rad,
                                            Phase next_phase,
                                            std::string label) {
  turn_target_yaw_ = normalizeAngle(target_yaw_rad);
  turn_next_phase_ = next_phase;
  turn_label_ = std::move(label);
  turn_stable_ticks_ = 0;
  turn_waiting_odom_logged_ = false;
  if (node_) {
    phase_start_ = node_->now();
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛开始转向：%s，target_yaw=%.3f rad",
                turn_label_.c_str(), turn_target_yaw_);
  }
  phase_ = Phase::TurnYaw;
}

BT::NodeStatus MfPreselectionFlowAction::tickTurnYaw() {
  if (!node_) {
    return fail("turn_runtime_missing");
  }
  if ((node_->now() - phase_start_).seconds() > params_.turn_timeout_s) {
    return fail("turn_timeout_" + turn_label_);
  }
  if (!odomReady()) {
    publishStop();
    if (!turn_waiting_odom_logged_) {
      RCLCPP_WARN(node_->get_logger(),
                  "梅林预选赛转向等待odom新鲜：%s，odom_topic=%s",
                  turn_label_.c_str(), params_.odom_topic.c_str());
      turn_waiting_odom_logged_ = true;
    }
    return BT::NodeStatus::RUNNING;
  }
  const double error = normalizeAngle(turn_target_yaw_ - odom_yaw_);
  if (std::abs(error) <= params_.turn_tolerance_deg * kDeg2Rad) {
    ++turn_stable_ticks_;
    publishStop();
    if (turn_stable_ticks_ >= params_.turn_stable_ticks) {
      RCLCPP_INFO(node_->get_logger(),
                  "梅林预选赛转向完成：%s，当前yaw=%.3f target=%.3f error=%.3f rad",
                  turn_label_.c_str(), odom_yaw_, turn_target_yaw_, error);
      phase_ = turn_next_phase_;
    }
    return BT::NodeStatus::RUNNING;
  }
  turn_stable_ticks_ = 0;
  const double wz =
      std::clamp(params_.turn_kp * error, -params_.turn_max_speed_radps,
                 params_.turn_max_speed_radps);
  publishTwist(0.0, 0.0, wz);
  return BT::NodeStatus::RUNNING;
}

void MfPreselectionFlowAction::beginZeroHold(double duration_s,
                                             Phase next_phase,
                                             std::string label) {
  zero_hold_duration_s_ = std::max(0.0, duration_s);
  zero_hold_next_phase_ = next_phase;
  zero_hold_label_ = std::move(label);
  if (node_) {
    phase_start_ = node_->now();
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛开始零速等待：%s，duration=%.2fs",
                zero_hold_label_.c_str(), zero_hold_duration_s_);
  }
  phase_ = Phase::ZeroHold;
}

BT::NodeStatus MfPreselectionFlowAction::tickZeroHold() {
  publishStop();
  if (!node_ ||
      (node_->now() - phase_start_).seconds() >= zero_hold_duration_s_) {
    if (node_) {
      RCLCPP_INFO(node_->get_logger(),
                  "梅林预选赛零速等待完成：%s，进入下一阶段",
                  zero_hold_label_.c_str());
    }
    phase_ = zero_hold_next_phase_;
  }
  return BT::NodeStatus::RUNNING;
}

bool MfPreselectionFlowAction::prepareTransitionTo(int target_grid) {
  std::shared_ptr<MerlinMapManager> map;
  if (!config().blackboard->get("merlin_map", map) || !map) {
    return false;
  }
  const int from_depth = map->getDepth(current_grid_);
  const int to_depth = map->getDepth(target_grid);
  if (from_depth < 0 || to_depth < 0) {
    return false;
  }
  transition_from_grid_ = current_grid_;
  transition_target_grid_ = target_grid;
  transition_height_delta_ = to_depth - from_depth;
  if (std::abs(transition_height_delta_) != 1) {
    return false;
  }
  transition_high_side_ = transition_height_delta_ > 0;
  return true;
}

BT::NodeStatus MfPreselectionFlowAction::startTransitionTo(int target_grid) {
  if (!prepareTransitionTo(target_grid)) {
    return fail("invalid_transition");
  }
  const double yaw = transitionYaw(transition_from_grid_, transition_target_grid_,
                                   transition_height_delta_);
  RCLCPP_INFO(node_->get_logger(),
              "梅林预选赛准备格间转换：grid%d -> grid%d，高度差=%d，目标yaw=%.3f",
              transition_from_grid_, transition_target_grid_,
              transition_height_delta_, yaw);
  beginTurnYaw(yaw, Phase::TransitionArmAdjust, "transition_turn");
  return BT::NodeStatus::RUNNING;
}

void MfPreselectionFlowAction::continueAfterTransition() {
  current_grid_ = transition_target_grid_;
  config().blackboard->set("current_grid", current_grid_);
  writeBlackboardState("transition_done");
  RCLCPP_INFO(node_->get_logger(),
              "梅林预选赛格间转换完成：当前grid=%d，直出模式=%s",
              current_grid_, direct_exit_mode_ ? "是" : "否");
  if (direct_exit_mode_) {
    phase_ = Phase::AfterEntry;
  } else if (current_grid_ == 5 || current_grid_ == 8) {
    phase_ = Phase::RowScanTurnLeft;
  } else if (current_grid_ == 11) {
    phase_ = Phase::Row4ForcedTurn;
  } else if (current_grid_ == 12) {
    phase_ = Phase::Row4DirectDescendPrep;
  } else {
    phase_ = Phase::RowFrontDetect;
  }
}

double MfPreselectionFlowAction::transitionYaw(int from_grid, int target_grid,
                                               int height_delta) const {
  const int row_delta = gridRow(target_grid) - gridRow(from_grid);
  const int col_delta = gridCol(target_grid) - gridCol(from_grid);
  const double dx = static_cast<double>(row_delta);
  const double dy = static_cast<double>(-col_delta);
  const double edge_yaw = std::atan2(dy, dx);
  return normalizeAngle(height_delta > 0 ? edge_yaw : edge_yaw + kPi);
}

void MfPreselectionFlowAction::beginStair(StairMode mode, Phase next_phase,
                                          std::string label) {
  stair_mode_ = mode;
  stair_next_phase_ = next_phase;
  stair_label_ = std::move(label);
  stair_phase_ = (mode == StairMode::Climb) ? StairPhase::ClimbSendFrontExtend
                                            : StairPhase::DescendDriveUntilRearEvent;
  active_wheel_event_label_.clear();
  active_wheel_event_started_ = false;
  timed_drive_started_ = false;
  if (node_) {
    phase_start_ = node_->now();
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛开始%s动作：%s，完成后进入下一阶段",
                stairModeText(mode), stair_label_.c_str());
  }
  phase_ = Phase::StairPrimitive;
}

BT::NodeStatus MfPreselectionFlowAction::tickStair() {
  switch (stair_phase_) {
  case StairPhase::ClimbSendFrontExtend:
    beginMechanismCommand(static_cast<uint8_t>(rc26_serial::CommandID::FRONT_PUSHROD_EXTEND),
                          "FRONT_PUSHROD_EXTEND", -1, Phase::StairPrimitive,
                          "front_pushrod_extend_failed");
    break;
  case StairPhase::ClimbHoldAfterFrontExtend:
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛上阶梯：前推杆伸出ACK完成，零速等待 %.2fs",
                stair_params_.climb_front_extend_delay_s);
    stair_phase_ = StairPhase::ClimbDriveUntilFrontFirstEvent;
    beginZeroHold(stair_params_.climb_front_extend_delay_s,
                  Phase::StairPrimitive, "climb_front_extend_hold");
    break;
  case StairPhase::ClimbDriveUntilFrontFirstEvent:
    if (guardPathObstacles()) {
      phase_start_ = node_->now();
      break;
    }
    publishTwist(stair_params_.climb_drive_speed_mps, 0.0,
                 headingAngularZ(turn_target_yaw_));
    if (!active_wheel_event_started_) {
      beginWheelEvent(WheelEvent::FrontFirst, stair_params_.front_event_timeout_s,
                      "front_first");
    }
    if (const auto wheel_status = tickWheelEvent();
        wheel_status == BT::NodeStatus::SUCCESS) {
      publishStop();
      RCLCPP_INFO(node_->get_logger(),
                  "梅林预选赛上阶梯：前轮第一激光事件确认，准备前推杆收回+后推杆伸出");
      stair_phase_ = StairPhase::ClimbSendFrontRetractAndRearExtend;
      active_wheel_event_label_.clear();
      active_wheel_event_started_ = false;
    } else if (wheel_status == BT::NodeStatus::FAILURE) {
      return BT::NodeStatus::FAILURE;
    }
    break;
  case StairPhase::ClimbSendFrontRetractAndRearExtend:
    beginCommandPair(static_cast<uint8_t>(rc26_serial::CommandID::FRONT_PUSHROD_RETRACT),
                     "FRONT_PUSHROD_RETRACT",
                     static_cast<uint8_t>(rc26_serial::CommandID::REAR_PUSHROD_EXTEND),
                     "REAR_PUSHROD_EXTEND", Phase::StairPrimitive,
                     "climb_pair_command_failed");
    break;
  case StairPhase::ClimbHoldAfterFrontRetractAndRearExtend:
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛上阶梯：前推杆收回+后推杆伸出ACK完成，零速等待 %.2fs",
                stair_params_.climb_retract_rear_extend_delay_s);
    stair_phase_ = StairPhase::ClimbDriveUntilRearEvent;
    beginZeroHold(stair_params_.climb_retract_rear_extend_delay_s,
                  Phase::StairPrimitive, "climb_pair_hold");
    break;
  case StairPhase::ClimbDriveUntilRearEvent:
    if (guardPathObstacles()) {
      phase_start_ = node_->now();
      break;
    }
    publishTwist(climbRearProfileSpeed(), 0.0, headingAngularZ(turn_target_yaw_));
    if (!active_wheel_event_started_) {
      beginWheelEvent(WheelEvent::Rear, stair_params_.rear_event_timeout_s,
                      "rear");
    }
    if (const auto wheel_status = tickWheelEvent();
        wheel_status == BT::NodeStatus::SUCCESS) {
      publishStop();
      RCLCPP_INFO(node_->get_logger(),
                  "梅林预选赛上阶梯：后轮激光事件确认，准备收回后推杆");
      stair_phase_ = StairPhase::ClimbSendRearRetract;
      active_wheel_event_label_.clear();
      active_wheel_event_started_ = false;
    } else if (wheel_status == BT::NodeStatus::FAILURE) {
      return BT::NodeStatus::FAILURE;
    }
    break;
  case StairPhase::ClimbSendRearRetract:
    beginMechanismCommand(static_cast<uint8_t>(rc26_serial::CommandID::REAR_PUSHROD_RETRACT),
                          "REAR_PUSHROD_RETRACT", -1, Phase::StairPrimitive,
                          "rear_pushrod_retract_failed");
    break;
  case StairPhase::ClimbHoldAfterRearRetract:
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛上阶梯：后推杆收回ACK完成，零速等待 %.2fs 后完成上阶梯",
                stair_params_.climb_rear_retract_delay_s);
    stair_phase_ = StairPhase::Complete;
    beginZeroHold(stair_params_.climb_rear_retract_delay_s,
                  Phase::StairPrimitive, "climb_rear_retract_hold");
    break;
  case StairPhase::DescendDriveUntilRearEvent:
    if (guardPathObstacles()) {
      phase_start_ = node_->now();
      break;
    }
    publishTwist(-stair_params_.descend_drive_speed_mps, 0.0,
                 headingAngularZ(turn_target_yaw_));
    if (!active_wheel_event_started_) {
      beginWheelEvent(WheelEvent::Rear, stair_params_.rear_event_timeout_s,
                      "rear");
    }
    if (const auto wheel_status = tickWheelEvent();
        wheel_status == BT::NodeStatus::SUCCESS) {
      publishStop();
      RCLCPP_INFO(node_->get_logger(),
                  "梅林预选赛下阶梯：后轮激光事件确认，准备伸出后推杆");
      stair_phase_ = StairPhase::DescendSendRearExtend;
      active_wheel_event_label_.clear();
      active_wheel_event_started_ = false;
    } else if (wheel_status == BT::NodeStatus::FAILURE) {
      return BT::NodeStatus::FAILURE;
    }
    break;
  case StairPhase::DescendSendRearExtend:
    beginMechanismCommand(static_cast<uint8_t>(rc26_serial::CommandID::REAR_PUSHROD_EXTEND),
                          "REAR_PUSHROD_EXTEND", -1, Phase::StairPrimitive,
                          "rear_pushrod_extend_failed");
    break;
  case StairPhase::DescendHoldAfterRearExtend:
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛下阶梯：后推杆伸出ACK完成，零速等待 %.2fs",
                stair_params_.descend_rear_extend_delay_s);
    stair_phase_ = StairPhase::DescendDriveUntilFrontSecondEvent;
    beginZeroHold(stair_params_.descend_rear_extend_delay_s,
                  Phase::StairPrimitive, "descend_rear_extend_hold");
    break;
  case StairPhase::DescendDriveUntilFrontSecondEvent:
    if (guardPathObstacles()) {
      phase_start_ = node_->now();
      break;
    }
    publishTwist(-stair_params_.descend_drive_speed_mps, 0.0,
                 headingAngularZ(turn_target_yaw_));
    if (!active_wheel_event_started_) {
      beginWheelEvent(WheelEvent::FrontSecond,
                      stair_params_.front_event_timeout_s, "front_second");
    }
    if (const auto wheel_status = tickWheelEvent();
        wheel_status == BT::NodeStatus::SUCCESS) {
      publishStop();
      RCLCPP_INFO(node_->get_logger(),
                  "梅林预选赛下阶梯：前轮第二激光事件确认，准备后推杆收回+前推杆伸出");
      stair_phase_ = StairPhase::DescendSendRearRetractAndFrontExtend;
      active_wheel_event_label_.clear();
      active_wheel_event_started_ = false;
    } else if (wheel_status == BT::NodeStatus::FAILURE) {
      return BT::NodeStatus::FAILURE;
    }
    break;
  case StairPhase::DescendSendRearRetractAndFrontExtend:
    beginCommandPair(static_cast<uint8_t>(rc26_serial::CommandID::REAR_PUSHROD_RETRACT),
                     "REAR_PUSHROD_RETRACT",
                     static_cast<uint8_t>(rc26_serial::CommandID::FRONT_PUSHROD_EXTEND),
                     "FRONT_PUSHROD_EXTEND", Phase::StairPrimitive,
                     "descend_pair_command_failed");
    break;
  case StairPhase::DescendHoldAfterRearRetractAndFrontExtend:
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛下阶梯：后推杆收回+前推杆伸出ACK完成，零速等待 %.2fs",
                stair_params_.descend_retract_front_extend_delay_s);
    stair_phase_ = StairPhase::DescendTimedDriveBeforeFrontRetract;
    beginZeroHold(stair_params_.descend_retract_front_extend_delay_s,
                  Phase::StairPrimitive, "descend_pair_hold");
    break;
  case StairPhase::DescendTimedDriveBeforeFrontRetract:
    if (!timed_drive_started_) {
      phase_start_ = node_->now();
      timed_drive_started_ = true;
      RCLCPP_INFO(node_->get_logger(),
                  "梅林预选赛下阶梯：前推杆收回前定时后退，speed=%.3fm/s duration=%.2fs",
                  stair_params_.descend_front_retract_drive_speed_mps,
                  stair_params_.descend_front_retract_drive_duration_s);
    }
    if ((node_->now() - phase_start_).seconds() >=
        stair_params_.descend_front_retract_drive_duration_s) {
      publishStop();
      RCLCPP_INFO(node_->get_logger(),
                  "梅林预选赛下阶梯：前推杆收回前定时后退完成，准备收回前推杆");
      stair_phase_ = StairPhase::DescendSendFrontRetract;
    } else {
      publishTwist(-stair_params_.descend_front_retract_drive_speed_mps, 0.0,
                   headingAngularZ(turn_target_yaw_));
    }
    break;
  case StairPhase::DescendSendFrontRetract:
    beginMechanismCommand(static_cast<uint8_t>(rc26_serial::CommandID::FRONT_PUSHROD_RETRACT),
                          "FRONT_PUSHROD_RETRACT", -1, Phase::StairPrimitive,
                          "front_pushrod_retract_failed");
    break;
  case StairPhase::DescendHoldAfterFrontRetract:
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛下阶梯：前推杆收回ACK完成，零速等待 %.2fs 后完成下阶梯",
                stair_params_.descend_front_retract_delay_s);
    stair_phase_ = StairPhase::Complete;
    beginZeroHold(stair_params_.descend_front_retract_delay_s,
                  Phase::StairPrimitive, "descend_front_retract_hold");
    break;
  case StairPhase::Complete:
    RCLCPP_INFO(node_->get_logger(), "梅林预选赛台阶动作完成：%s",
                stair_label_.c_str());
    if (stair_next_phase_ == Phase::AfterEntry &&
        transition_target_grid_ != current_grid_) {
      continueAfterTransition();
    } else {
      phase_ = stair_next_phase_;
    }
    break;
  }
  return BT::NodeStatus::RUNNING;
}

void MfPreselectionFlowAction::beginWheelEvent(WheelEvent event,
                                               double timeout_s,
                                               std::string label) {
  active_wheel_event_ = event;
  active_wheel_event_timeout_s_ = std::max(kMinTimeoutS, timeout_s);
  active_wheel_event_label_ = std::move(label);
  active_wheel_event_started_ = true;
  front_first_event_baseline_ =
      front_first_event_count_.load(std::memory_order_relaxed);
  front_second_event_baseline_ =
      front_second_event_count_.load(std::memory_order_relaxed);
  rear_event_baseline_ = rear_event_count_.load(std::memory_order_relaxed);
  if (node_) {
    phase_start_ = node_->now();
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛开始等待激光事件：%s，阶段=%s，timeout=%.2fs",
                wheelEventText(event), active_wheel_event_label_.c_str(),
                active_wheel_event_timeout_s_);
  }
}

bool MfPreselectionFlowAction::wheelEventReceived() const {
  switch (active_wheel_event_) {
  case WheelEvent::FrontFirst:
    return front_first_event_count_.load(std::memory_order_relaxed) >
           front_first_event_baseline_;
  case WheelEvent::FrontSecond:
    return front_second_event_count_.load(std::memory_order_relaxed) >
           front_second_event_baseline_;
  case WheelEvent::Rear:
    return rear_event_count_.load(std::memory_order_relaxed) >
           rear_event_baseline_;
  }
  return false;
}

BT::NodeStatus MfPreselectionFlowAction::tickWheelEvent() {
  if (wheelEventReceived()) {
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛收到激光事件：%s，阶段=%s",
                wheelEventText(active_wheel_event_),
                active_wheel_event_label_.c_str());
    return BT::NodeStatus::SUCCESS;
  }
  if (node_ && (node_->now() - phase_start_).seconds() >
                   active_wheel_event_timeout_s_) {
    if (node_) {
      RCLCPP_WARN(node_->get_logger(),
                  "梅林预选赛等待激光事件超时：%s，阶段=%s",
                  wheelEventText(active_wheel_event_),
                  active_wheel_event_label_.c_str());
    }
    phase_ = Phase::Done;
    (void)fail("wheel_event_timeout");
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

double MfPreselectionFlowAction::climbRearProfileSpeed() {
  const double fast = stair_params_.climb_rear_drive_fast_speed_mps;
  const double slow = stair_params_.climb_rear_drive_slow_speed_mps;
  const double duration =
      std::max(0.0, stair_params_.climb_rear_drive_slowdown_duration_s);
  if (!node_) {
    return fast;
  }
  if (!climb_rear_profile_started_) {
    climb_rear_profile_start_ = node_->now();
    climb_rear_profile_started_ = true;
  }
  if (duration <= 0.0) {
    return slow;
  }
  const double ratio =
      std::clamp((node_->now() - climb_rear_profile_start_).seconds() / duration,
                 0.0, 1.0);
  return fast + (slow - fast) * ratio;
}

void MfPreselectionFlowAction::beginGrab(bool high_side,
                                         MfPreselectionPickupSource source,
                                         Phase next_phase) {
  if (!canPickup()) {
    if (node_) {
      RCLCPP_INFO(node_->get_logger(),
                  "梅林预选赛已达到R2 KFS夹取上限：%d/%d，跳过夹取继续流程",
                  pickup_count_, params_.max_pickup_count);
    }
    phase_ = next_phase;
    return;
  }
  grab_next_phase_ = next_phase;
  pending_grab_commit_ = true;
  pending_grab_source_ = source;
  const uint8_t command_id =
      MfPreselectionLogicResult::grabCommandForHighSide(high_side, params_);
  if (node_) {
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛开始夹取R2 KFS：command=%s(0x%02X)，高侧=%s，来源=%s，当前计数=%d/%d",
                high_side ? "GRAB_KFS_UP" : "GRAB_KFS_DOWN",
                static_cast<unsigned int>(command_id),
                high_side ? "是" : "否", sourceName(source), pickup_count_,
                params_.max_pickup_count);
  }
  beginMechanismCommand(command_id,
                        high_side ? "GRAB_KFS_UP" : "GRAB_KFS_DOWN", -1,
                        Phase::ZeroHold, "grab_kfs_failed");
  zero_hold_next_phase_ = next_phase;
  zero_hold_duration_s_ = params_.grab_settle_s;
  zero_hold_label_ = "grab_settle";
}

void MfPreselectionFlowAction::continueAfterGrab() { phase_ = grab_next_phase_; }

uint8_t MfPreselectionFlowAction::clampByte(int value) const {
  return static_cast<uint8_t>(std::clamp(value, 0, 255));
}

rclcpp::Duration MfPreselectionFlowAction::seconds(double value) const {
  return rclcpp::Duration::from_seconds(value);
}

void loadMfPreselectionParams(rclcpp::Node &node,
                              const BT::Blackboard::Ptr &blackboard) {
  MfPreselectionParams p;
  p.vision_config_file =
      node.declare_parameter<std::string>("mf_preselect_vision_config_file",
                                          p.vision_config_file);
  p.model_id =
      node.declare_parameter<std::string>("mf_preselect_model_id", p.model_id);
  p.r2_target_label_prefixes =
      node.declare_parameter<std::vector<std::string>>(
          "mf_preselect_r2_target_label_prefixes", p.r2_target_label_prefixes);
  p.r2_target_labels = node.declare_parameter<std::vector<std::string>>(
      "mf_preselect_r2_target_labels", p.r2_target_labels);
  p.r1_blocking_labels = node.declare_parameter<std::vector<std::string>>(
      "mf_preselect_r1_blocking_labels", p.r1_blocking_labels);
  p.r1_blocking_label_prefixes =
      node.declare_parameter<std::vector<std::string>>(
          "mf_preselect_r1_blocking_label_prefixes",
          p.r1_blocking_label_prefixes);
  p.fake_label_prefixes = node.declare_parameter<std::vector<std::string>>(
      "mf_preselect_fake_label_prefixes", p.fake_label_prefixes);
  p.fake_labels = node.declare_parameter<std::vector<std::string>>(
      "mf_preselect_fake_labels", p.fake_labels);
  p.depth_min_m =
      node.declare_parameter<double>("mf_preselect_depth_min_m", p.depth_min_m);
  p.depth_max_m =
      node.declare_parameter<double>("mf_preselect_depth_max_m", p.depth_max_m);
  p.detect_seen_stable_frames = node.declare_parameter<int>(
      "mf_preselect_detect_seen_stable_frames", p.detect_seen_stable_frames);
  p.detect_lost_stable_frames = node.declare_parameter<int>(
      "mf_preselect_detect_lost_stable_frames", p.detect_lost_stable_frames);
  p.entry_detect_timeout_s = node.declare_parameter<double>(
      "mf_preselect_entry_detect_timeout_s", p.entry_detect_timeout_s);
  p.scan_detect_timeout_s = node.declare_parameter<double>(
      "mf_preselect_scan_detect_timeout_s", p.scan_detect_timeout_s);

  p.max_pickup_count = node.declare_parameter<int>(
      "mf_preselect_max_pickup_count", p.max_pickup_count);
  p.grab_settle_s = node.declare_parameter<double>(
      "mf_preselect_grab_settle_s", p.grab_settle_s);
  p.grab_kfs_up_command_id = node.declare_parameter<int>(
      "mf_preselect_grab_kfs_up_command_id", p.grab_kfs_up_command_id);
  p.grab_kfs_down_command_id = node.declare_parameter<int>(
      "mf_preselect_grab_kfs_down_command_id", p.grab_kfs_down_command_id);

  p.cmd_vel_topic = node.declare_parameter<std::string>(
      "mf_preselect_cmd_vel_topic", p.cmd_vel_topic);
  p.odom_topic = node.declare_parameter<std::string>(
      "mf_preselect_odom_topic", p.odom_topic);
  p.send_command_service = node.declare_parameter<std::string>(
      "mf_preselect_send_command_service", p.send_command_service);
  p.feedback_topic = node.declare_parameter<std::string>(
      "mf_preselect_feedback_topic", p.feedback_topic);
  p.command_timeout_s = node.declare_parameter<double>(
      "mf_preselect_command_timeout_s", p.command_timeout_s);
  p.arm_high_raise_command_id = node.declare_parameter<int>(
      "mf_preselect_arm_high_raise_command_id", p.arm_high_raise_command_id);
  p.arm_high_raise_done_feedback_id = node.declare_parameter<int>(
      "mf_preselect_arm_high_raise_done_feedback_id",
      p.arm_high_raise_done_feedback_id);
  p.arm_raise_command_id = node.declare_parameter<int>(
      "mf_preselect_arm_raise_command_id", p.arm_raise_command_id);
  p.arm_lower_command_id = node.declare_parameter<int>(
      "mf_preselect_arm_lower_command_id", p.arm_lower_command_id);
  p.arm_raise_done_feedback_id = node.declare_parameter<int>(
      "mf_preselect_arm_raise_done_feedback_id", p.arm_raise_done_feedback_id);
  p.arm_lower_done_feedback_id = node.declare_parameter<int>(
      "mf_preselect_arm_lower_done_feedback_id", p.arm_lower_done_feedback_id);

  p.entry_probe_left_distance_m = node.declare_parameter<double>(
      "mf_preselect_entry_probe_left_distance_m",
      p.entry_probe_left_distance_m);
  p.entry_probe_right_sweep_distance_m = node.declare_parameter<double>(
      "mf_preselect_entry_probe_right_sweep_distance_m",
      p.entry_probe_right_sweep_distance_m);
  p.entry_probe_return_distance_m = node.declare_parameter<double>(
      "mf_preselect_entry_probe_return_distance_m",
      p.entry_probe_return_distance_m);
  p.lateral_probe_speed_mps = node.declare_parameter<double>(
      "mf_preselect_lateral_probe_speed_mps", p.lateral_probe_speed_mps);
  p.move_tolerance_m = node.declare_parameter<double>(
      "mf_preselect_move_tolerance_m", p.move_tolerance_m);
  p.move_timeout_s = node.declare_parameter<double>(
      "mf_preselect_move_timeout_s", p.move_timeout_s);
  p.direct_exit_drive_distance_m = node.declare_parameter<double>(
      "mf_preselect_direct_exit_drive_distance_m",
      p.direct_exit_drive_distance_m);
  p.direct_exit_drive_speed_mps = node.declare_parameter<double>(
      "mf_preselect_direct_exit_drive_speed_mps", p.direct_exit_drive_speed_mps);

  p.exit_yaw_rad =
      node.declare_parameter<double>("mf_preselect_exit_yaw_rad", p.exit_yaw_rad);
  p.stair1_direction_yaw_rad = node.declare_parameter<double>(
      "mf_preselect_stair1_direction_yaw_rad", p.stair1_direction_yaw_rad);
  p.stair3_direction_yaw_rad = node.declare_parameter<double>(
      "mf_preselect_stair3_direction_yaw_rad", p.stair3_direction_yaw_rad);
  p.row_scan_left_yaw_delta_rad = node.declare_parameter<double>(
      "mf_preselect_row_scan_left_yaw_delta_rad",
      p.row_scan_left_yaw_delta_rad);
  p.row_scan_back_yaw_delta_rad = node.declare_parameter<double>(
      "mf_preselect_row_scan_back_yaw_delta_rad",
      p.row_scan_back_yaw_delta_rad);
  p.row4_exit_turn_yaw_rad = node.declare_parameter<double>(
      "mf_preselect_row4_exit_turn_yaw_rad", p.row4_exit_turn_yaw_rad);
  p.turn_kp =
      node.declare_parameter<double>("mf_preselect_turn_kp", p.turn_kp);
  p.turn_max_speed_radps = node.declare_parameter<double>(
      "mf_preselect_turn_max_speed_radps", p.turn_max_speed_radps);
  p.turn_tolerance_deg = node.declare_parameter<double>(
      "mf_preselect_turn_tolerance_deg", p.turn_tolerance_deg);
  p.turn_stable_ticks = node.declare_parameter<int>(
      "mf_preselect_turn_stable_ticks", p.turn_stable_ticks);
  p.turn_timeout_s = node.declare_parameter<double>(
      "mf_preselect_turn_timeout_s", p.turn_timeout_s);
  p.odom_timeout_s = node.declare_parameter<double>(
      "mf_preselect_odom_timeout_s", p.odom_timeout_s);
  p.heading_kp = node.declare_parameter<double>(
      "mf_preselect_heading_kp", p.heading_kp);
  p.heading_max_speed_radps = node.declare_parameter<double>(
      "mf_preselect_heading_max_speed_radps", p.heading_max_speed_radps);
  p.path_r1_lost_wait_timeout_s = node.declare_parameter<double>(
      "mf_preselect_path_r1_lost_wait_timeout_s",
      p.path_r1_lost_wait_timeout_s);

  p.vision_config_file = resolveVisionConfig(p.vision_config_file);
  blackboard->set("mf_preselection_params", p);

  const bool entry_nav_enable = node.declare_parameter<bool>(
      "mf_preselect_entry2_nav_enable", false);
  blackboard->set("mf_preselect_entry2_nav_enable", entry_nav_enable);
  blackboard->set("mf_preselect_entry2_nav_x",
                  node.declare_parameter<double>("mf_preselect_entry2_nav_x",
                                                 0.0));
  blackboard->set("mf_preselect_entry2_nav_y",
                  node.declare_parameter<double>("mf_preselect_entry2_nav_y",
                                                 0.0));
  blackboard->set("mf_preselect_entry2_nav_yaw",
                  node.declare_parameter<double>("mf_preselect_entry2_nav_yaw",
                                                 0.0));
  blackboard->set(
      "mf_preselect_entry2_nav_frame_id",
      node.declare_parameter<std::string>("mf_preselect_entry2_nav_frame_id",
                                          "map"));
  blackboard->set(
      "mf_preselect_entry2_nav_timeout_sec",
      node.declare_parameter<double>("mf_preselect_entry2_nav_timeout_sec",
                                     180.0));
  blackboard->set(
      "mf_preselect_entry2_nav_behavior_tree_file",
      node.declare_parameter<std::string>(
          "mf_preselect_entry2_nav_behavior_tree_file", ""));

  RCLCPP_INFO(node.get_logger(),
              "梅林预选赛参数已加载: model=%s R2_prefixes=%zu fake_prefixes=%zu max_pickup=%d cmd_vel=%s odom=%s high_raise=0x%02X done=0x%02X",
              p.model_id.c_str(), p.r2_target_label_prefixes.size(),
              p.fake_label_prefixes.size(), p.max_pickup_count,
              p.cmd_vel_topic.c_str(), p.odom_topic.c_str(),
              static_cast<unsigned int>(std::clamp(p.arm_high_raise_command_id, 0, 255)),
              static_cast<unsigned int>(std::clamp(p.arm_high_raise_done_feedback_id, 0, 255)));
}

void registerMfPreselectionNodes(BT::BehaviorTreeFactory &factory) {
  factory.registerNodeType<MfPreselectionFlowAction>("MfPreselectionFlow");
}

} // namespace rc26_decision
