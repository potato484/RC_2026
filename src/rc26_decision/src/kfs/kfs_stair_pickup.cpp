#include "rc26_decision/kfs/kfs_stair_pickup.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <limits>
#include <stdexcept>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "rc26_vision/inference/config/model_profile_loader.hpp"
#include "rc26_vision/shared/sensors/depth_roi_sampler.hpp"

namespace rc26_decision {

namespace {

constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;
constexpr double kMinTimeoutS = 0.001;

std::string resolveVisionConfig(const std::string &configured) {
  namespace fs = std::filesystem;
  if (!configured.empty() && fs::exists(configured)) {
    return fs::path(configured).lexically_normal().string();
  }
  try {
    const fs::path share = ament_index_cpp::get_package_share_directory("rc26_vision");
    const fs::path candidate =
        configured.empty() ? (share / "config" / "vision_models.yaml") : (share / configured);
    if (fs::exists(candidate)) {
      return candidate.lexically_normal().string();
    }
  } catch (...) {
  }
  return configured;
}

uint8_t clampByte(int value) {
  return static_cast<uint8_t>(std::clamp(value, 0, 255));
}

bool containsLabel(const std::vector<std::string> &labels, const std::string &needle) {
  return std::find(labels.begin(), labels.end(), needle) != labels.end();
}

} // namespace

KfsStairPickupAction::KfsStairPickupAction(const std::string &name,
                                           const BT::NodeConfig &config)
    : BT::StatefulActionNode(name, config) {}

KfsStairPickupAction::~KfsStairPickupAction() { releaseRuntime(); }

BT::PortsList KfsStairPickupAction::providedPorts() {
  return {
      BT::InputPort<std::string>("direction", "climb|descend"),
      BT::InputPort<double>("target_yaw_rad", "可选 heading hold 目标 yaw"),
  };
}

BT::NodeStatus KfsStairPickupAction::onStart() {
  if (!setupRuntime()) {
    return BT::NodeStatus::FAILURE;
  }
  if (!parseDirection()) {
    return fail("invalid_direction");
  }

  setOutcome("running");
  start_tp_ = std::chrono::steady_clock::now();
  phase_ = Phase::SendingPrep;
  sendPrepCommand();
  RCLCPP_INFO(node_->get_logger(),
              "KFS 阶梯等待启动: direction=%s prep_cmd=0x%02X done=0x%02X labels=%zu model=%s",
              direction_text_.c_str(), prep_command_id_, prep_done_feedback_id_,
              params_.blocking_labels.size(), params_.model_id.c_str());
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus KfsStairPickupAction::onRunning() {
  if (phase_ == Phase::SendingPrep || phase_ == Phase::WaitingPrepDone) {
    return tickPrepCommand();
  }

  if (phase_ == Phase::DetectingInitial || phase_ == Phase::WaitingBlockingGone) {
    if (waitTimedOut()) {
      return fail("blocking_wait_timeout");
    }

    double distance_m = 0.0;
    std::string label;
    bool observation_valid = false;
    int64_t observation_sequence = 0;
    const bool blocking =
        isBlockingTargetPresent(distance_m, label, observation_valid, observation_sequence);
    const bool fresh_observation =
        observation_valid && observation_sequence > 0 &&
        observation_sequence != last_observation_sequence_;
    if (fresh_observation) {
      last_observation_sequence_ = observation_sequence;
    }

    if (blocking) {
      if (!fresh_observation) {
        publishStop();
        return BT::NodeStatus::RUNNING;
      }
      seen_stable_count_ = std::min(seen_stable_count_ + 1, params_.blocking_seen_stable_frames);
      lost_stable_count_ = 0;
      config().blackboard->set("kfs_last_label", label);
      config().blackboard->set("kfs_last_distance_m", distance_m);
      publishStop();
      if (seen_stable_count_ >= params_.blocking_seen_stable_frames) {
        if (!ever_blocking_seen_) {
          RCLCPP_INFO(node_->get_logger(), "KFS 占用目标稳定出现，进入等待: label=%s distance=%.3fm",
                      label.c_str(), distance_m);
        }
        ever_blocking_seen_ = true;
        phase_ = Phase::WaitingBlockingGone;
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "KFS 占用目标仍在等待: label=%s distance=%.3fm",
                             label.c_str(), distance_m);
      }
      return BT::NodeStatus::RUNNING;
    }

    if (!fresh_observation) {
      publishStop();
      return BT::NodeStatus::RUNNING;
    }

    seen_stable_count_ = 0;
    lost_stable_count_++;
    publishStop();
    if (!ever_blocking_seen_) {
      const bool vision_ready = vision_ && vision_->isReady();
      if (detectionTimedOut() ||
          (vision_ready && lost_stable_count_ >= params_.blocking_lost_stable_frames)) {
        setOutcome("no_target");
        phase_ = Phase::Done;
        releaseRuntime();
        return BT::NodeStatus::SUCCESS;
      }
      return BT::NodeStatus::RUNNING;
    }

    if (lost_stable_count_ >= params_.blocking_lost_stable_frames) {
      setOutcome("blocked_target_gone");
      phase_ = Phase::Done;
      releaseRuntime();
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  }

  return BT::NodeStatus::SUCCESS;
}

void KfsStairPickupAction::onHalted() { releaseRuntime(); }

bool KfsStairPickupAction::setupRuntime() {
  if (!config().blackboard->get("node", node_) || node_ == nullptr) {
    return false;
  }
  if (!config().blackboard->get("kfs_params", params_)) {
    RCLCPP_ERROR(node_->get_logger(), "KFS 阶梯等待: 黑板缺少 kfs_params");
    return false;
  }
  normalizeParams();

  cmd_pub_ = node_->create_publisher<TwistMsg>(params_.cmd_vel_topic, rclcpp::QoS(10));
  send_client_ = node_->create_client<SendCommandSrv>(params_.send_command_service);
  feedback_sub_ = node_->create_subscription<FeedbackMsg>(
      params_.feedback_topic, rclcpp::QoS(32).reliable(),
      [this](const FeedbackMsg::SharedPtr msg) {
        if (!msg) {
          return;
        }
        if (msg->feedback_id == prep_done_feedback_id_) {
          latest_prep_done_seq_.store(static_cast<int>(msg->seq), std::memory_order_relaxed);
          const int expected_seq = prep_seq_.load(std::memory_order_relaxed);
          if (expected_seq >= 0 && msg->seq == static_cast<uint8_t>(expected_seq)) {
            prep_done_seen_.store(true, std::memory_order_relaxed);
          }
        }
      });
  setupOdomSubscription();

  prep_response_seen_ = false;
  prep_accepted_ = false;
  prep_done_seen_ = false;
  command_generation_.fetch_add(1, std::memory_order_relaxed);
  prep_seq_.store(-1, std::memory_order_relaxed);
  latest_prep_done_seq_.store(-1, std::memory_order_relaxed);
  seen_stable_count_ = 0;
  lost_stable_count_ = 0;
  last_observation_sequence_ = 0;
  ever_blocking_seen_ = false;
  has_odom_yaw_ = false;
  heading_target_set_ = false;
  if (const auto input = getInput<double>("target_yaw_rad")) {
    heading_target_yaw_rad_ = normalizeAngle(input.value());
    heading_target_set_ = true;
  }
  return true;
}

bool KfsStairPickupAction::setupVision() {
  try {
    params_.vision_config_file = resolveVisionConfig(params_.vision_config_file);
    auto config = rc26_vision::ProfileLoader::loadFromYaml(params_.vision_config_file);
    rc26_vision::ProfileLoader::validate(config);
    auto profile_it = config.profiles.find(params_.model_id);
    if (profile_it == config.profiles.end()) {
      RCLCPP_ERROR(node_->get_logger(), "KFS 视觉 profile 不存在: %s", params_.model_id.c_str());
      return false;
    }
    blocking_class_ids_.clear();
    const auto &labels = profile_it->second.labels;
    for (std::size_t index = 0; index < labels.size(); ++index) {
      if (containsLabel(params_.blocking_labels, labels[index])) {
        blocking_class_ids_.push_back(static_cast<int>(index));
      }
    }
    if (blocking_class_ids_.empty()) {
      RCLCPP_ERROR(node_->get_logger(), "KFS 占用标签未匹配到模型类别");
      return false;
    }
    vision_ = std::make_shared<rc26_vision::VisionInferenceManager>(*node_);
    vision_->loadConfig(config);
    vision_->selectModel(params_.model_id);
    if (!vision_->start()) {
      RCLCPP_ERROR(node_->get_logger(), "KFS 视觉推理启动失败");
      vision_.reset();
      return false;
    }
    return true;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node_->get_logger(), "KFS 视觉初始化异常: %s", e.what());
    vision_.reset();
    return false;
  }
}

void KfsStairPickupAction::releaseRuntime() {
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
}

bool KfsStairPickupAction::parseDirection() {
  std::string direction;
  if (!getInput("direction", direction)) {
    RCLCPP_ERROR(node_->get_logger(), "KFS 阶梯等待: 缺少 direction 端口");
    return false;
  }
  if (direction == "climb") {
    direction_ = Direction::Climb;
    direction_text_ = "climb";
    prep_label_ = "ARM_RAISE";
    prep_command_id_ = clampByte(params_.arm_raise_command_id);
    prep_done_feedback_id_ = clampByte(params_.arm_raise_done_feedback_id);
    return true;
  }
  if (direction == "descend") {
    direction_ = Direction::Descend;
    direction_text_ = "descend";
    prep_label_ = "ARM_LOWER";
    prep_command_id_ = clampByte(params_.arm_lower_command_id);
    prep_done_feedback_id_ = clampByte(params_.arm_lower_done_feedback_id);
    return true;
  }
  RCLCPP_ERROR(node_->get_logger(), "KFS 阶梯等待: direction 非法: %s", direction.c_str());
  return false;
}

void KfsStairPickupAction::normalizeParams() {
  params_.depth_min_m = std::max(0.0, params_.depth_min_m);
  params_.depth_max_m = std::max(params_.depth_min_m, params_.depth_max_m);
  params_.blocking_seen_stable_frames = std::max(1, params_.blocking_seen_stable_frames);
  params_.blocking_lost_stable_frames = std::max(1, params_.blocking_lost_stable_frames);
  params_.blocking_initial_detection_timeout_s =
      std::max(kMinTimeoutS, params_.blocking_initial_detection_timeout_s);
  params_.blocking_wait_timeout_s = std::max(kMinTimeoutS, params_.blocking_wait_timeout_s);
  params_.command_timeout_s = std::max(kMinTimeoutS, params_.command_timeout_s);
  params_.heading_kp = std::max(0.0, params_.heading_kp);
  params_.heading_max_speed_radps = std::max(0.0, params_.heading_max_speed_radps);
  params_.heading_tolerance_deg = std::max(0.0, params_.heading_tolerance_deg);
  params_.heading_gate_deg = std::max(params_.heading_tolerance_deg, params_.heading_gate_deg);
  params_.heading_odom_timeout_s = std::max(kMinTimeoutS, params_.heading_odom_timeout_s);
  params_.vision_config_file = resolveVisionConfig(params_.vision_config_file);
}

void KfsStairPickupAction::publishStop() {
  if (cmd_pub_) {
    cmd_pub_->publish(TwistMsg{});
  }
}

void KfsStairPickupAction::setOutcome(const std::string &outcome, const std::string &error) {
  if (!config().blackboard) {
    return;
  }
  config().blackboard->set("kfs_last_outcome", outcome);
  config().blackboard->set("kfs_last_direction", direction_text_);
  config().blackboard->set("kfs_last_error", error);
}

BT::NodeStatus KfsStairPickupAction::fail(const std::string &error) {
  RCLCPP_ERROR(node_->get_logger(), "KFS 阶梯等待失败: %s", error.c_str());
  setOutcome("failed", error);
  releaseRuntime();
  return BT::NodeStatus::FAILURE;
}

void KfsStairPickupAction::sendPrepCommand() {
  if (!send_client_) {
    return;
  }
  prep_send_tp_ = std::chrono::steady_clock::now();
  prep_response_seen_ = false;
  prep_accepted_ = false;
  prep_done_seen_ = false;
  command_generation_.fetch_add(1, std::memory_order_relaxed);
  const uint64_t generation = command_generation_.load(std::memory_order_relaxed);

  if (!send_client_->service_is_ready()) {
    if (!send_client_->wait_for_service(std::chrono::milliseconds(1))) {
      RCLCPP_WARN(node_->get_logger(), "KFS 预调 service 暂不可用: %s",
                  params_.send_command_service.c_str());
      return;
    }
  }

  auto request = std::make_shared<SendCommandSrv::Request>();
  request->command_id = prep_command_id_;
  request->payload.clear();
  send_client_->async_send_request(
      request, [this, generation](rclcpp::Client<SendCommandSrv>::SharedFuture future) {
        if (generation != command_generation_.load(std::memory_order_relaxed)) {
          return;
        }
        try {
          const auto response = future.get();
          prep_accepted_.store(response && response->accepted, std::memory_order_relaxed);
          if (response) {
            prep_seq_.store(static_cast<int>(response->seq), std::memory_order_relaxed);
            if (latest_prep_done_seq_.load(std::memory_order_relaxed) ==
                static_cast<int>(response->seq)) {
              prep_done_seen_.store(true, std::memory_order_relaxed);
            }
          }
        } catch (...) {
          prep_accepted_.store(false, std::memory_order_relaxed);
        }
        prep_response_seen_.store(true, std::memory_order_relaxed);
      });
  phase_ = Phase::WaitingPrepDone;
  RCLCPP_INFO(node_->get_logger(), "KFS 下发预调命令: %s(0x%02X)", prep_label_.c_str(),
              prep_command_id_);
}

BT::NodeStatus KfsStairPickupAction::tickPrepCommand() {
  publishStop();
  if (!prep_response_seen_.load(std::memory_order_relaxed)) {
    if ((std::chrono::steady_clock::now() - prep_send_tp_) >
        std::chrono::duration<double>(params_.command_timeout_s)) {
      return fail("prep_command_ack_timeout");
    }
    return BT::NodeStatus::RUNNING;
  }

  if (!prep_accepted_.load(std::memory_order_relaxed)) {
    return fail("prep_command_rejected");
  }

  if (prepFeedbackReceived()) {
    RCLCPP_INFO(node_->get_logger(), "KFS 预调完成: %s seq=%u", prep_label_.c_str(),
                static_cast<unsigned int>(prep_seq_.load(std::memory_order_relaxed)));
    if (!setupVision()) {
      return fail("vision_start_failed");
    }
    start_tp_ = std::chrono::steady_clock::now();
    phase_ = Phase::DetectingInitial;
    lost_stable_count_ = 0;
    seen_stable_count_ = 0;
    return BT::NodeStatus::RUNNING;
  }

  if ((std::chrono::steady_clock::now() - prep_send_tp_) >
      std::chrono::duration<double>(params_.command_timeout_s)) {
    return fail("prep_done_feedback_timeout");
  }
  return BT::NodeStatus::RUNNING;
}

bool KfsStairPickupAction::prepFeedbackReceived() const {
  return prep_done_seen_.load(std::memory_order_relaxed);
}

bool KfsStairPickupAction::isBlockingTargetPresent(double &distance_m,
                                                   std::string &label,
                                                   bool &observation_valid,
                                                   int64_t &observation_sequence) {
  observation_valid = false;
  observation_sequence = 0;
  if (!vision_ || !vision_->isRunning()) {
    return false;
  }
  rc26_vision::VisionInferenceManager::FrameSnapshot snapshot;
  if (!vision_->getLatestFrameSnapshot(snapshot) || !snapshot.has_display || !snapshot.has_depth) {
    return false;
  }
  observation_valid = true;
  observation_sequence = snapshot.display_sequence;

  const rc26_vision::Detection *best = nullptr;
  for (const auto &det : snapshot.detections) {
    if (std::find(blocking_class_ids_.begin(), blocking_class_ids_.end(), det.class_id) ==
        blocking_class_ids_.end()) {
      continue;
    }
    if (!best || det.score > best->score) {
      best = &det;
    }
  }
  if (!best) {
    return false;
  }

  const int cx = static_cast<int>((best->x1 + best->x2) * 0.5F);
  const int cy = static_cast<int>((best->y1 + best->y2) * 0.5F);
  rc26_vision::DepthRoiSamplerConfig config;
  config.roi_size = 7;
  config.min_valid_count = 10;
  config.min_depth_m = params_.depth_min_m;
  config.max_depth_m = params_.depth_max_m;
  const auto sampled = rc26_vision::sampleMedianDepth(snapshot.depth, cx, cy, config);
  if (!sampled.has_value()) {
    return false;
  }

  distance_m = *sampled;
  label = best->class_name.empty() ? std::to_string(best->class_id) : best->class_name;
  return true;
}

bool KfsStairPickupAction::detectionTimedOut() const {
  return elapsedSinceStart() >= params_.blocking_initial_detection_timeout_s;
}

bool KfsStairPickupAction::waitTimedOut() const {
  return elapsedSinceStart() >= params_.blocking_wait_timeout_s;
}

double KfsStairPickupAction::elapsedSinceStart() const {
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - start_tp_).count();
}

void KfsStairPickupAction::setupOdomSubscription() {
  if (!params_.heading_hold_enable || params_.odom_topic.empty()) {
    return;
  }
  odom_sub_ = node_->create_subscription<OdomMsg>(
      params_.odom_topic, rclcpp::QoS(rclcpp::KeepLast(10)),
      [this](const OdomMsg::SharedPtr msg) { handleOdom(msg); });
}

void KfsStairPickupAction::handleOdom(const OdomMsg::SharedPtr msg) {
  if (!msg) {
    return;
  }
  current_yaw_rad_ = yawFromQuaternion(msg->pose.pose.orientation);
  has_odom_yaw_ = true;
  last_odom_tp_ = std::chrono::steady_clock::now();
  if (!heading_target_set_) {
    heading_target_yaw_rad_ = current_yaw_rad_;
    heading_target_set_ = true;
  }
}

double KfsStairPickupAction::headingAngularZ() const {
  if (!params_.heading_hold_enable || !has_odom_yaw_ || !heading_target_set_) {
    return 0.0;
  }
  const double age_s =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - last_odom_tp_).count();
  if (age_s > params_.heading_odom_timeout_s) {
    return 0.0;
  }
  const double error = normalizeAngle(heading_target_yaw_rad_ - current_yaw_rad_);
  const double tolerance = params_.heading_tolerance_deg * kDeg2Rad;
  if (std::abs(error) <= tolerance) {
    return 0.0;
  }
  const double speed =
      std::clamp(std::abs(error) * params_.heading_kp, 0.0, params_.heading_max_speed_radps);
  return error >= 0.0 ? speed : -speed;
}

double KfsStairPickupAction::normalizeAngle(double angle_rad) {
  while (angle_rad > M_PI) {
    angle_rad -= 2.0 * M_PI;
  }
  while (angle_rad < -M_PI) {
    angle_rad += 2.0 * M_PI;
  }
  return angle_rad;
}

double KfsStairPickupAction::yawFromQuaternion(const geometry_msgs::msg::Quaternion &q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

void loadKfsParams(rclcpp::Node &node, const BT::Blackboard::Ptr &blackboard) {
  KfsParams p;
  p.vision_config_file = node.declare_parameter<std::string>("kfs_vision_config_file", "");
  p.model_id = node.declare_parameter<std::string>("kfs_model_id", p.model_id);
  p.blocking_labels =
      node.declare_parameter<std::vector<std::string>>("kfs_blocking_labels", p.blocking_labels);
  p.depth_min_m = node.declare_parameter<double>("kfs_depth_min_m", p.depth_min_m);
  p.depth_max_m = node.declare_parameter<double>("kfs_depth_max_m", p.depth_max_m);
  p.blocking_seen_stable_frames =
      node.declare_parameter<int>("kfs_blocking_seen_stable_frames", p.blocking_seen_stable_frames);
  p.blocking_lost_stable_frames =
      node.declare_parameter<int>("kfs_blocking_lost_stable_frames", p.blocking_lost_stable_frames);
  p.blocking_initial_detection_timeout_s = node.declare_parameter<double>(
      "kfs_blocking_initial_detection_timeout_s", p.blocking_initial_detection_timeout_s);
  p.blocking_wait_timeout_s =
      node.declare_parameter<double>("kfs_blocking_wait_timeout_s", p.blocking_wait_timeout_s);

  p.cmd_vel_topic = node.declare_parameter<std::string>("kfs_cmd_vel_topic", p.cmd_vel_topic);
  p.send_command_service =
      node.declare_parameter<std::string>("kfs_send_command_service", p.send_command_service);
  p.feedback_topic = node.declare_parameter<std::string>("kfs_feedback_topic", p.feedback_topic);
  p.command_timeout_s =
      node.declare_parameter<double>("kfs_command_timeout_s", p.command_timeout_s);
  p.arm_raise_command_id =
      node.declare_parameter<int>("kfs_arm_raise_command_id", p.arm_raise_command_id);
  p.arm_lower_command_id =
      node.declare_parameter<int>("kfs_arm_lower_command_id", p.arm_lower_command_id);
  p.arm_raise_done_feedback_id =
      node.declare_parameter<int>("kfs_arm_raise_done_feedback_id", p.arm_raise_done_feedback_id);
  p.arm_lower_done_feedback_id =
      node.declare_parameter<int>("kfs_arm_lower_done_feedback_id", p.arm_lower_done_feedback_id);
  p.grab_kfs_up_command_id =
      node.declare_parameter<int>("kfs_grab_kfs_up_command_id", p.grab_kfs_up_command_id);
  p.grab_kfs_down_command_id =
      node.declare_parameter<int>("kfs_grab_kfs_down_command_id", p.grab_kfs_down_command_id);

  p.odom_topic = node.declare_parameter<std::string>("kfs_odom_topic", p.odom_topic);
  p.heading_hold_enable =
      node.declare_parameter<bool>("kfs_heading_hold_enable", p.heading_hold_enable);
  p.heading_kp = node.declare_parameter<double>("kfs_heading_kp", p.heading_kp);
  p.heading_max_speed_radps =
      node.declare_parameter<double>("kfs_heading_max_speed_radps", p.heading_max_speed_radps);
  p.heading_tolerance_deg =
      node.declare_parameter<double>("kfs_heading_tolerance_deg", p.heading_tolerance_deg);
  p.heading_gate_deg = node.declare_parameter<double>("kfs_heading_gate_deg", p.heading_gate_deg);
  p.heading_odom_timeout_s =
      node.declare_parameter<double>("kfs_heading_odom_timeout_s", p.heading_odom_timeout_s);

  p.vision_config_file = resolveVisionConfig(p.vision_config_file);
  blackboard->set("kfs_params", p);
  RCLCPP_INFO(node.get_logger(),
              "KFS 参数已加载: model=%s labels=%zu depth=[%.2f, %.2f] service=%s feedback=%s",
              p.model_id.c_str(), p.blocking_labels.size(), p.depth_min_m, p.depth_max_m,
              p.send_command_service.c_str(), p.feedback_topic.c_str());
}

void registerKfsNodes(BT::BehaviorTreeFactory &factory) {
  factory.registerNodeType<KfsStairPickupAction>("KfsStairPickup");
}

} // namespace rc26_decision
