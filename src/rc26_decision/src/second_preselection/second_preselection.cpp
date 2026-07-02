#include "rc26_decision/second_preselection/second_preselection.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <sstream>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/imgproc.hpp>

#include "rc26_decision/decision_failure.hpp"
#include "rc26_decision/team_color.hpp"
#include "rc26_serial/protocol.hpp"
#include "rc26_vision/inference/config/model_profile_loader.hpp"

namespace rc26_decision {

namespace {

double elapsedSec(const std::chrono::steady_clock::time_point &since) {
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - since)
      .count();
}

int clampByteParam(int value) { return std::clamp(value, 0, 255); }

int clampHueParam(int value) { return std::clamp(value, 0, 180); }

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

cv::Scalar hsvLower(const SecondPreselectionHsvRange &range) {
  return cv::Scalar(clampHueParam(range.hue_low),
                    clampByteParam(range.saturation_min),
                    clampByteParam(range.value_min));
}

cv::Scalar hsvUpper(const SecondPreselectionHsvRange &range) {
  return cv::Scalar(clampHueParam(range.hue_high), 255, 255);
}

void addRangeMask(const cv::Mat &hsv, const SecondPreselectionHsvRange &range,
                  cv::Mat &mask) {
  cv::Mat part;
  cv::inRange(hsv, hsvLower(range), hsvUpper(range), part);
  if (mask.empty()) {
    mask = part;
  } else {
    cv::bitwise_or(mask, part, mask);
  }
}

std::vector<uint8_t> emptyPayload() { return {}; }

} // namespace

SecondPreselectionHsvObservation evaluateSecondPreselectionOccupancy(
    const cv::Mat &frame_bgr, const SecondPreselectionParams &params,
    const std::string &team) {
  SecondPreselectionHsvObservation result;
  if (frame_bgr.empty()) {
    return result;
  }

  const int x = std::clamp(params.roi_x, 0, std::max(0, frame_bgr.cols - 1));
  const int y = std::clamp(params.roi_y, 0, std::max(0, frame_bgr.rows - 1));
  const int width = std::clamp(params.roi_width, 1, frame_bgr.cols - x);
  const int height = std::clamp(params.roi_height, 1, frame_bgr.rows - y);
  const cv::Mat roi = frame_bgr(cv::Rect(x, y, width, height));

  cv::Mat hsv;
  cv::cvtColor(roi, hsv, cv::COLOR_BGR2HSV);
  cv::Mat mask;
  const bool our_team_is_red = resolveTeamColorRuntime(team).normalized == "red";
  if (our_team_is_red) {
    addRangeMask(hsv, params.blue_hsv1, mask);
    addRangeMask(hsv, params.blue_hsv2, mask);
  } else {
    addRangeMask(hsv, params.red_hsv1, mask);
    addRangeMask(hsv, params.red_hsv2, mask);
  }

  const cv::Mat kernel =
      cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
  cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  for (const auto &contour : contours) {
    result.best_area_px = std::max(result.best_area_px, cv::contourArea(contour));
  }
  result.occupied =
      result.best_area_px >= static_cast<double>(params.occupied_min_area_px);
  return result;
}

BT::PortsList SecondPreselectionCommandAction::providedPorts() {
  return {BT::InputPort<int>("command_id"),
          BT::InputPort<int>("done_feedback_id", -1,
                             "Optional same-seq done feedback id; <0 means ACK-only"),
          BT::InputPort<double>("command_timeout_s", 5.0,
                                "Service ACK timeout in seconds"),
          BT::InputPort<double>("done_timeout_s", 5.0,
                                "Done feedback timeout in seconds"),
          BT::InputPort<std::string>("label", "second_preselection_command",
                                     "Human-readable command label")};
}

SecondPreselectionCommandAction::SecondPreselectionCommandAction(
    const std::string &name, const BT::NodeConfig &config)
    : BT::StatefulActionNode(name, config) {}

BT::NodeStatus SecondPreselectionCommandAction::onStart() {
  if (!config().blackboard || !config().blackboard->get("node", node_) ||
      !node_) {
    writeDecisionFailure(config().blackboard, "SecondPreselectionCommand",
                         "运行上下文缺失：blackboard 或 node 不可用");
    return BT::NodeStatus::FAILURE;
  }
  if (!config().blackboard->get("second_preselection_params", params_)) {
    writeDecisionFailure(config().blackboard, "SecondPreselectionCommand",
                         "黑板缺少 second_preselection_params");
    return BT::NodeStatus::FAILURE;
  }

  int command_id = 0;
  (void)getInput("command_id", command_id);
  (void)getInput("done_feedback_id", done_feedback_id_);
  (void)getInput("command_timeout_s", command_timeout_s_);
  (void)getInput("done_timeout_s", done_timeout_s_);
  (void)getInput("label", command_label_);

  command_id_ = static_cast<uint8_t>(command_id & 0xFF);
  command_timeout_s_ =
      (std::isfinite(command_timeout_s_) && command_timeout_s_ > 0.0)
          ? command_timeout_s_
          : params_.command_timeout_s;
  done_timeout_s_ = (std::isfinite(done_timeout_s_) && done_timeout_s_ > 0.0)
                        ? done_timeout_s_
                        : params_.done_timeout_s;
  if (command_label_.empty()) {
    command_label_ = "second_preselection_command";
  }

  command_response_seen_ = false;
  command_accepted_ = false;
  done_feedback_seen_ = false;
  command_seq_ = -1;
  generation_.fetch_add(1, std::memory_order_relaxed);
  phase_ = Phase::Sending;
  phase_tp_ = std::chrono::steady_clock::now();
  last_log_tp_ = phase_tp_;

  send_client_ = node_->create_client<SendCommandSrv>(params_.send_command_service);
  if (done_feedback_id_ >= 0) {
    feedback_sub_ = node_->create_subscription<FeedbackMsg>(
        params_.feedback_topic, rclcpp::QoS(32).reliable(),
        [this](const FeedbackMsg::SharedPtr msg) { handleFeedback(msg); });
  }

  RCLCPP_INFO(node_->get_logger(),
              "第二预选赛机构命令准备发送：%s command=%s done_feedback=%s service=%s",
              command_label_.c_str(), byteHex(command_id_).c_str(),
              done_feedback_id_ >= 0 ? byteHex(done_feedback_id_).c_str()
                                     : "ACK-only",
              params_.send_command_service.c_str());
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SecondPreselectionCommandAction::onRunning() {
  const auto now = std::chrono::steady_clock::now();
  if (phase_ == Phase::Sending) {
    if (!sendCommand()) {
      if (elapsedSec(phase_tp_) > command_timeout_s_) {
        return fail("等待机构命令服务可用超时：" + command_label_);
      }
      return BT::NodeStatus::RUNNING;
    }
    phase_ = Phase::WaitingAck;
    phase_tp_ = now;
    return BT::NodeStatus::RUNNING;
  }

  if (phase_ == Phase::WaitingAck) {
    if (command_response_seen_.load(std::memory_order_relaxed)) {
      const int seq = command_seq_.load(std::memory_order_relaxed);
      if (!command_accepted_.load(std::memory_order_relaxed)) {
        return fail("机构命令被拒绝：" + command_label_ +
                    " seq=" + std::to_string(seq));
      }
      RCLCPP_INFO(node_->get_logger(),
                  "第二预选赛机构命令 ACK 成功：%s command=%s seq=%d",
                  command_label_.c_str(), byteHex(command_id_).c_str(), seq);
      if (done_feedback_id_ < 0) {
        resetRuntimeHandles();
        return BT::NodeStatus::SUCCESS;
      }
      phase_ = Phase::WaitingDone;
      phase_tp_ = now;
      last_log_tp_ = now;
      return BT::NodeStatus::RUNNING;
    }
    if (elapsedSec(phase_tp_) > command_timeout_s_) {
      return fail("等待机构命令 ACK 超时：" + command_label_);
    }
    return BT::NodeStatus::RUNNING;
  }

  const int seq = command_seq_.load(std::memory_order_relaxed);
  if (done_feedback_seen_.load(std::memory_order_relaxed)) {
    RCLCPP_INFO(node_->get_logger(),
                "第二预选赛机构命令完成反馈已收到：%s feedback=%s seq=%d",
                command_label_.c_str(), byteHex(done_feedback_id_).c_str(), seq);
    resetRuntimeHandles();
    return BT::NodeStatus::SUCCESS;
  }
  if (elapsedSec(phase_tp_) > done_timeout_s_) {
    return fail("等待机构完成反馈超时：" + command_label_ +
                " seq=" + std::to_string(seq));
  }
  if (elapsedSec(last_log_tp_) >= params_.log_period_s) {
    RCLCPP_INFO(node_->get_logger(),
                "第二预选赛等待机构完成反馈：%s feedback=%s seq=%d elapsed=%.1fs",
                command_label_.c_str(), byteHex(done_feedback_id_).c_str(), seq,
                elapsedSec(phase_tp_));
    last_log_tp_ = now;
  }
  return BT::NodeStatus::RUNNING;
}

void SecondPreselectionCommandAction::onHalted() {
  generation_.fetch_add(1, std::memory_order_relaxed);
  resetRuntimeHandles();
}

void SecondPreselectionCommandAction::handleFeedback(
    const FeedbackMsg::SharedPtr msg) {
  if (!msg || done_feedback_id_ < 0) {
    return;
  }
  const int seq = command_seq_.load(std::memory_order_relaxed);
  if (seq >= 0 && msg->seq == static_cast<uint8_t>(seq & 0xFF) &&
      msg->feedback_id == static_cast<uint8_t>(done_feedback_id_ & 0xFF)) {
    done_feedback_seen_.store(true, std::memory_order_relaxed);
  }
}

bool SecondPreselectionCommandAction::sendCommand() {
  if (!send_client_ || !send_client_->service_is_ready()) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                         "第二预选赛等待机构命令服务：%s",
                         params_.send_command_service.c_str());
    return false;
  }

  auto request = std::make_shared<SendCommandSrv::Request>();
  request->command_id = command_id_;
  request->payload = emptyPayload();

  const uint64_t token = generation_.load(std::memory_order_relaxed);
  command_response_seen_ = false;
  command_accepted_ = false;
  command_seq_ = -1;
  try {
    send_client_->async_send_request(
        request, [this, token](rclcpp::Client<SendCommandSrv>::SharedFuture future) {
          if (token != generation_.load(std::memory_order_relaxed)) {
            return;
          }
          bool accepted = false;
          int seq = -1;
          try {
            const auto response = future.get();
            accepted = response && response->accepted;
            if (response) {
              seq = static_cast<int>(response->seq);
            }
          } catch (const std::exception &) {
            accepted = false;
          }
          command_seq_.store(seq, std::memory_order_relaxed);
          command_accepted_.store(accepted, std::memory_order_relaxed);
          command_response_seen_.store(true, std::memory_order_relaxed);
        });
  } catch (const std::exception &e) {
    writeDecisionFailure(config().blackboard, "SecondPreselectionCommand",
                         std::string("机构命令发送异常：") + e.what());
    command_response_seen_ = true;
    command_accepted_ = false;
    return true;
  }

  RCLCPP_INFO(node_->get_logger(),
              "第二预选赛已下发机构命令：%s command=%s",
              command_label_.c_str(), byteHex(command_id_).c_str());
  return true;
}

BT::NodeStatus SecondPreselectionCommandAction::fail(
    const std::string &reason) {
  RCLCPP_ERROR(node_->get_logger(), "第二预选赛机构命令失败：%s", reason.c_str());
  writeDecisionFailure(config().blackboard, "SecondPreselectionCommand", reason);
  resetRuntimeHandles();
  return BT::NodeStatus::FAILURE;
}

void SecondPreselectionCommandAction::resetRuntimeHandles() {
  feedback_sub_.reset();
  send_client_.reset();
}

std::string SecondPreselectionCommandAction::byteHex(int value) {
  char buf[8];
  std::snprintf(buf, sizeof(buf), "0x%02X",
                static_cast<unsigned int>(value & 0xFF));
  return std::string(buf);
}

SecondPreselectionObserveAction::SecondPreselectionObserveAction(
    const std::string &name, const BT::NodeConfig &config)
    : BT::StatefulActionNode(name, config) {}

SecondPreselectionObserveAction::~SecondPreselectionObserveAction() {
  releaseVision();
}

BT::NodeStatus SecondPreselectionObserveAction::onStart() {
  if (!config().blackboard || !config().blackboard->get("node", node_) ||
      !node_) {
    writeDecisionFailure(config().blackboard, "SecondPreselectionObserve",
                         "运行上下文缺失：blackboard 或 node 不可用");
    return BT::NodeStatus::FAILURE;
  }
  if (!config().blackboard->get("second_preselection_params", params_)) {
    config().blackboard->set("second_preselection_observe_error", true);
    return fail("黑板缺少 second_preselection_params");
  }
  (void)config().blackboard->get("team", team_);
  config().blackboard->set("second_preselection_observe_error", false);
  config().blackboard->set("second_preselection_middle_empty", false);
  config().blackboard->set("second_preselection_middle_occupied", false);
  config().blackboard->set("second_preselection_last_observe_area_px", 0.0);

  occupied_stable_count_ = 0;
  start_tp_ = std::chrono::steady_clock::now();
  last_log_tp_ = start_tp_;
  if (!setupVision()) {
    config().blackboard->set("second_preselection_observe_error", true);
    return fail("第二预选赛九宫格中层视觉启动失败");
  }

  RCLCPP_INFO(node_->get_logger(),
              "第二预选赛开始观察九宫格中层：team=%s ROI=[%d,%d,%d,%d] opponent=%s timeout=%.1fs area>=%d stable=%d",
              team_.c_str(), params_.roi_x, params_.roi_y, params_.roi_width,
              params_.roi_height,
              resolveTeamColorRuntime(team_).normalized == "red" ? "blue"
                                                                 : "red",
              params_.observe_timeout_s, params_.occupied_min_area_px,
              params_.occupied_stable_frames);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SecondPreselectionObserveAction::onRunning() {
  if (!vision_) {
    config().blackboard->set("second_preselection_observe_error", true);
    return fail("第二预选赛九宫格中层视觉运行时不可用");
  }

  rc26_vision::VisionInferenceManager::FrameSnapshot snapshot;
  const bool got_snapshot = vision_->getLatestFrameSnapshot(snapshot);
  if (got_snapshot && snapshot.has_color && !snapshot.color_bgr.empty()) {
    const auto observation = evaluateSecondPreselectionOccupancy(
        snapshot.color_bgr, params_, team_);
    config().blackboard->set("second_preselection_last_observe_area_px",
                             observation.best_area_px);
    if (observation.occupied) {
      ++occupied_stable_count_;
    } else {
      config().blackboard->set("second_preselection_middle_empty", true);
      config().blackboard->set("second_preselection_middle_occupied", false);
      RCLCPP_INFO(node_->get_logger(),
                  "第二预选赛九宫格中层判定为空：best_area=%.0fpx",
                  observation.best_area_px);
      releaseVision();
      return BT::NodeStatus::SUCCESS;
    }
    if (occupied_stable_count_ >= params_.occupied_stable_frames) {
      config().blackboard->set("second_preselection_middle_empty", false);
      config().blackboard->set("second_preselection_middle_occupied", true);
      RCLCPP_INFO(node_->get_logger(),
                  "第二预选赛九宫格中层被对方 KFS 占据：best_area=%.0fpx stable=%d/%d",
                  observation.best_area_px, occupied_stable_count_,
                  params_.occupied_stable_frames);
      releaseVision();
      return BT::NodeStatus::SUCCESS;
    }
    if (elapsedSec(last_log_tp_) >= params_.observe_log_period_s) {
      RCLCPP_INFO(node_->get_logger(),
                  "第二预选赛九宫格中层观察中：occupied=%s best_area=%.0fpx stable=%d/%d elapsed=%.1fs",
                  observation.occupied ? "true" : "false",
                  observation.best_area_px, occupied_stable_count_,
                  params_.occupied_stable_frames, elapsedSec(start_tp_));
      last_log_tp_ = std::chrono::steady_clock::now();
    }
  } else if (elapsedSec(last_log_tp_) >= params_.observe_log_period_s) {
    RCLCPP_WARN(node_->get_logger(),
                "第二预选赛九宫格中层观察等待彩色帧：snapshot=%s has_color=%s elapsed=%.1fs",
                got_snapshot ? "true" : "false",
                (got_snapshot && snapshot.has_color) ? "true" : "false",
                elapsedSec(start_tp_));
    last_log_tp_ = std::chrono::steady_clock::now();
  }

  if (elapsedSec(start_tp_) > params_.observe_timeout_s) {
    config().blackboard->set("second_preselection_middle_empty", false);
    config().blackboard->set("second_preselection_middle_occupied", true);
    RCLCPP_WARN(node_->get_logger(),
                "第二预选赛九宫格中层观察超时，保守判定为被占据");
    releaseVision();
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

void SecondPreselectionObserveAction::onHalted() { releaseVision(); }

BT::NodeStatus SecondPreselectionObserveAction::fail(
    const std::string &reason) {
  if (node_) {
    RCLCPP_ERROR(node_->get_logger(), "第二预选赛视觉观察失败：%s",
                 reason.c_str());
  }
  writeDecisionFailure(config().blackboard, "SecondPreselectionObserve", reason);
  releaseVision();
  return BT::NodeStatus::FAILURE;
}

bool SecondPreselectionObserveAction::setupVision() {
  try {
    params_.vision_config_file = resolveVisionConfig(params_.vision_config_file);
    auto config =
        rc26_vision::ProfileLoader::loadFromYaml(params_.vision_config_file);
    rc26_vision::ProfileLoader::validate(config);
    if (config.profiles.find(params_.model_id) == config.profiles.end()) {
      RCLCPP_ERROR(node_->get_logger(), "第二预选赛视觉 profile 不存在：%s",
                   params_.model_id.c_str());
      return false;
    }
    vision_ = std::make_shared<rc26_vision::VisionInferenceManager>(*node_);
    vision_->loadConfig(config);
    vision_->selectModel(params_.model_id);
    if (!vision_->start()) {
      vision_.reset();
      return false;
    }
    return true;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node_->get_logger(), "第二预选赛视觉初始化异常：%s",
                 e.what());
    vision_.reset();
    return false;
  }
}

void SecondPreselectionObserveAction::releaseVision() {
  if (vision_) {
    vision_->stop();
    vision_.reset();
  }
}

void loadSecondPreselectionParams(rclcpp::Node &node,
                                  const BT::Blackboard::Ptr &blackboard) {
  SecondPreselectionParams p;
  p.send_command_service = node.declare_parameter<std::string>(
      "second_preselect_send_command_service", p.send_command_service);
  p.feedback_topic = node.declare_parameter<std::string>(
      "second_preselect_feedback_topic", p.feedback_topic);
  p.command_timeout_s = node.declare_parameter<double>(
      "second_preselect_command_timeout_s", p.command_timeout_s);
  p.done_timeout_s = node.declare_parameter<double>(
      "second_preselect_done_timeout_s", p.done_timeout_s);
  p.log_period_s = node.declare_parameter<double>(
      "second_preselect_log_period_s", p.log_period_s);
  p.start_command_id = node.declare_parameter<int>(
      "second_preselect_start_command_id", p.start_command_id);
  p.start_done_feedback_id = node.declare_parameter<int>(
      "second_preselect_start_done_feedback_id", p.start_done_feedback_id);
  p.arm_high_raise_command_id = node.declare_parameter<int>(
      "second_preselect_arm_high_raise_command_id",
      p.arm_high_raise_command_id);
  p.arm_high_raise_done_feedback_id = node.declare_parameter<int>(
      "second_preselect_arm_high_raise_done_feedback_id",
      p.arm_high_raise_done_feedback_id);
  p.place_kfs_command_id = node.declare_parameter<int>(
      "second_preselect_place_kfs_command_id", p.place_kfs_command_id);

  p.nav_x1_m =
      node.declare_parameter<double>("second_preselect_nav_x1_m", p.nav_x1_m);
  p.nav_y1_m =
      node.declare_parameter<double>("second_preselect_nav_y1_m", p.nav_y1_m);
  p.nav_x2_m =
      node.declare_parameter<double>("second_preselect_nav_x2_m", p.nav_x2_m);
  p.search_y_positive_m = node.declare_parameter<double>(
      "second_preselect_search_y_positive_m", p.search_y_positive_m);
  p.search_y_negative_m = node.declare_parameter<double>(
      "second_preselect_search_y_negative_m", p.search_y_negative_m);
  p.place_forward_x_m = node.declare_parameter<double>(
      "second_preselect_place_forward_x_m", p.place_forward_x_m);
  p.retreat_x_m = node.declare_parameter<double>(
      "second_preselect_retreat_x_m", p.retreat_x_m);
  p.nav_timeout_s = node.declare_parameter<double>(
      "second_preselect_nav_timeout_s", p.nav_timeout_s);

  p.vision_config_file = node.declare_parameter<std::string>(
      "second_preselect_vision_config_file", p.vision_config_file);
  p.model_id = node.declare_parameter<std::string>(
      "second_preselect_model_id", p.model_id);
  p.roi_x = node.declare_parameter<int>("second_preselect_roi_x", p.roi_x);
  p.roi_y = node.declare_parameter<int>("second_preselect_roi_y", p.roi_y);
  p.roi_width =
      node.declare_parameter<int>("second_preselect_roi_width", p.roi_width);
  p.roi_height =
      node.declare_parameter<int>("second_preselect_roi_height", p.roi_height);
  p.occupied_min_area_px = node.declare_parameter<int>(
      "second_preselect_occupied_min_area_px", p.occupied_min_area_px);
  p.occupied_stable_frames = node.declare_parameter<int>(
      "second_preselect_occupied_stable_frames", p.occupied_stable_frames);
  p.observe_timeout_s = node.declare_parameter<double>(
      "second_preselect_observe_timeout_s", p.observe_timeout_s);
  p.observe_log_period_s = node.declare_parameter<double>(
      "second_preselect_observe_log_period_s", p.observe_log_period_s);

  p.red_hsv1.hue_low =
      node.declare_parameter<int>("second_preselect_red_hue_low1",
                                  p.red_hsv1.hue_low);
  p.red_hsv1.hue_high =
      node.declare_parameter<int>("second_preselect_red_hue_high1",
                                  p.red_hsv1.hue_high);
  p.red_hsv2.hue_low =
      node.declare_parameter<int>("second_preselect_red_hue_low2",
                                  p.red_hsv2.hue_low);
  p.red_hsv2.hue_high =
      node.declare_parameter<int>("second_preselect_red_hue_high2",
                                  p.red_hsv2.hue_high);
  p.red_hsv1.saturation_min = p.red_hsv2.saturation_min =
      node.declare_parameter<int>("second_preselect_red_saturation_min",
                                  p.red_hsv1.saturation_min);
  p.red_hsv1.value_min = p.red_hsv2.value_min =
      node.declare_parameter<int>("second_preselect_red_value_min",
                                  p.red_hsv1.value_min);

  p.blue_hsv1.hue_low =
      node.declare_parameter<int>("second_preselect_blue_hue_low1",
                                  p.blue_hsv1.hue_low);
  p.blue_hsv1.hue_high =
      node.declare_parameter<int>("second_preselect_blue_hue_high1",
                                  p.blue_hsv1.hue_high);
  p.blue_hsv2.hue_low =
      node.declare_parameter<int>("second_preselect_blue_hue_low2",
                                  p.blue_hsv2.hue_low);
  p.blue_hsv2.hue_high =
      node.declare_parameter<int>("second_preselect_blue_hue_high2",
                                  p.blue_hsv2.hue_high);
  p.blue_hsv1.saturation_min = p.blue_hsv2.saturation_min =
      node.declare_parameter<int>("second_preselect_blue_saturation_min",
                                  p.blue_hsv1.saturation_min);
  p.blue_hsv1.value_min = p.blue_hsv2.value_min =
      node.declare_parameter<int>("second_preselect_blue_value_min",
                                  p.blue_hsv1.value_min);

  p.command_timeout_s = std::max(0.001, p.command_timeout_s);
  p.done_timeout_s = std::max(0.001, p.done_timeout_s);
  p.log_period_s = std::max(0.1, p.log_period_s);
  p.nav_timeout_s = std::max(0.001, p.nav_timeout_s);
  p.roi_width = std::max(1, p.roi_width);
  p.roi_height = std::max(1, p.roi_height);
  p.occupied_min_area_px = std::max(1, p.occupied_min_area_px);
  p.occupied_stable_frames = std::max(1, p.occupied_stable_frames);
  p.observe_timeout_s = std::max(0.001, p.observe_timeout_s);
  p.observe_log_period_s = std::max(0.1, p.observe_log_period_s);
  p.vision_config_file = resolveVisionConfig(p.vision_config_file);

  blackboard->set("second_preselection_params", p);
  blackboard->set("second_preselect_command_timeout_s", p.command_timeout_s);
  blackboard->set("second_preselect_done_timeout_s", p.done_timeout_s);
  blackboard->set("second_preselect_start_command_id", p.start_command_id);
  blackboard->set("second_preselect_start_done_feedback_id",
                  p.start_done_feedback_id);
  blackboard->set("second_preselect_arm_high_raise_command_id",
                  p.arm_high_raise_command_id);
  blackboard->set("second_preselect_arm_high_raise_done_feedback_id",
                  p.arm_high_raise_done_feedback_id);
  blackboard->set("second_preselect_place_kfs_command_id",
                  p.place_kfs_command_id);
  blackboard->set("second_preselect_nav_x1_m", p.nav_x1_m);
  blackboard->set("second_preselect_nav_y1_m", p.nav_y1_m);
  blackboard->set("second_preselect_nav_x2_m", p.nav_x2_m);
  blackboard->set("second_preselect_search_y_positive_m",
                  p.search_y_positive_m);
  blackboard->set("second_preselect_search_y_negative_m",
                  p.search_y_negative_m);
  blackboard->set("second_preselect_place_forward_x_m", p.place_forward_x_m);
  blackboard->set("second_preselect_retreat_x_m", p.retreat_x_m);
  blackboard->set("second_preselect_nav_timeout_s", p.nav_timeout_s);
  blackboard->set("second_preselection_middle_empty", false);
  blackboard->set("second_preselection_middle_occupied", false);
  blackboard->set("second_preselection_observe_error", false);
  blackboard->set("second_preselection_last_observe_area_px", 0.0);

  RCLCPP_INFO(node.get_logger(),
              "第二预选赛参数已加载: start=0x%02X/done=0x%02X high=0x%02X/done=0x%02X place=0x%02X nav=[x1 %.2f, y1 %.2f, x2 %.2f, y+ %.2f, y- %.2f, place %.2f, retreat %.2f] ROI=[%d,%d,%d,%d] area>=%d stable=%d",
              p.start_command_id & 0xFF, p.start_done_feedback_id & 0xFF,
              p.arm_high_raise_command_id & 0xFF,
              p.arm_high_raise_done_feedback_id & 0xFF,
              p.place_kfs_command_id & 0xFF, p.nav_x1_m, p.nav_y1_m,
              p.nav_x2_m, p.search_y_positive_m, p.search_y_negative_m,
              p.place_forward_x_m, p.retreat_x_m, p.roi_x, p.roi_y,
              p.roi_width, p.roi_height, p.occupied_min_area_px,
              p.occupied_stable_frames);
}

void registerSecondPreselectionNodes(BT::BehaviorTreeFactory &factory) {
  factory.registerNodeType<SecondPreselectionCommandAction>(
      "SecondPreselectionCommand");
  factory.registerNodeType<SecondPreselectionObserveAction>(
      "SecondPreselectionObserve");
  factory.registerNodeType<SecondPreselectionNoEmptyFailureAction>(
      "SecondPreselectionNoEmptyFailure");
}

SecondPreselectionNoEmptyFailureAction::SecondPreselectionNoEmptyFailureAction(
    const std::string &name, const BT::NodeConfig &config)
    : BT::SyncActionNode(name, config) {}

BT::NodeStatus SecondPreselectionNoEmptyFailureAction::tick() {
  rclcpp::Node *node = nullptr;
  if (config().blackboard) {
    (void)config().blackboard->get("node", node);
  }
  const std::string reason =
      "第二预选赛三次观察九宫格中层均被对方 KFS 占据，停止放置";
  if (node) {
    RCLCPP_ERROR(node->get_logger(), "%s", reason.c_str());
  }
  writeDecisionFailure(config().blackboard, "SecondPreselection", reason);
  return BT::NodeStatus::FAILURE;
}

} // namespace rc26_decision
