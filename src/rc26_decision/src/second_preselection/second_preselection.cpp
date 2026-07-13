#include "rc26_decision/second_preselection/second_preselection.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <optional>
#include <sstream>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "rc26_decision/decision_failure.hpp"
#include "rc26_decision/mechanism_error_diagnostic.hpp"
#include "rc26_decision/mf_preselection/mf_preselection_flow.hpp"
#include "rc26_decision/team_color.hpp"
#include "rc26_serial/protocol.hpp"
#include "rc26_vision/inference/config/model_profile_loader.hpp"
#include "rc26_vision/shared/sensors/depth_roi_sampler.hpp"

namespace rc26_decision {

namespace {

constexpr double kDeg2Rad = M_PI / 180.0;

double elapsedSec(const std::chrono::steady_clock::time_point &since) {
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - since)
      .count();
}

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

std::vector<uint8_t> emptyPayload() { return {}; }

uint8_t clampByte(int value) {
  return static_cast<uint8_t>(value & 0xFF);
}

std::string byteHex(int value) {
  char buf[8];
  std::snprintf(buf, sizeof(buf), "0x%02X",
                static_cast<unsigned int>(value & 0xFF));
  return std::string(buf);
}

double normalizeAngle(double angle) {
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double yawFromQuaternion(double x, double y, double z, double w) {
  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(siny_cosp, cosy_cosp);
}

cv::Point2f detectionCenter(const rc26_vision::Detection &detection) {
  return cv::Point2f((detection.x1 + detection.x2) * 0.5F,
                     (detection.y1 + detection.y2) * 0.5F);
}

bool startsWith(const std::string &value, const std::string &prefix) {
  return value.size() >= prefix.size() &&
         value.compare(0, prefix.size(), prefix) == 0;
}

bool labelsMatch(const std::string &label,
                 const std::vector<std::string> &exact_labels,
                 const std::vector<std::string> &prefixes) {
  if (label.empty()) {
    return false;
  }
  if (std::find(exact_labels.begin(), exact_labels.end(), label) !=
      exact_labels.end()) {
    return true;
  }
  for (const auto &prefix : prefixes) {
    if (!prefix.empty() && startsWith(label, prefix)) {
      return true;
    }
  }
  return false;
}

bool isKfsLabel(const std::string &label) {
  return label.find("KFS") != std::string::npos;
}

std::optional<cv::Rect> clippedRect(const cv::Rect2f &rect,
                                    const cv::Size &frame_size) {
  if (frame_size.width <= 0 || frame_size.height <= 0 || rect.width <= 0.0F ||
      rect.height <= 0.0F) {
    return std::nullopt;
  }
  const int x1 = static_cast<int>(std::floor(rect.x));
  const int y1 = static_cast<int>(std::floor(rect.y));
  const int x2 = static_cast<int>(std::ceil(rect.x + rect.width));
  const int y2 = static_cast<int>(std::ceil(rect.y + rect.height));
  const cv::Rect bounds(0, 0, frame_size.width, frame_size.height);
  const cv::Rect clipped =
      cv::Rect(x1, y1, std::max(0, x2 - x1), std::max(0, y2 - y1)) & bounds;
  if (clipped.width <= 0 || clipped.height <= 0) {
    return std::nullopt;
  }
  return clipped;
}

void drawTextWithBackground(cv::Mat &image, const std::string &text,
                            cv::Point origin, const cv::Scalar &text_color,
                            const cv::Scalar &background_color,
                            double scale = 0.45, int thickness = 1) {
  int baseline = 0;
  const int font = cv::FONT_HERSHEY_SIMPLEX;
  const cv::Size text_size =
      cv::getTextSize(text, font, scale, thickness, &baseline);
  origin.x = std::clamp(origin.x, 0,
                        std::max(0, image.cols - text_size.width - 4));
  origin.y = std::clamp(origin.y, text_size.height + 4,
                        std::max(text_size.height + 4, image.rows - 4));
  const cv::Rect bg(origin.x - 2, origin.y - text_size.height - 3,
                    text_size.width + 4, text_size.height + baseline + 5);
  cv::rectangle(image, bg & cv::Rect(0, 0, image.cols, image.rows),
                background_color, cv::FILLED);
  cv::putText(image, text, origin, font, scale, text_color, thickness,
              cv::LINE_AA);
}

cv::Scalar detectionColor(const std::string &label) {
  if (startsWith(label, "T_")) {
    return cv::Scalar(60, 220, 80);
  }
  if (startsWith(label, "R_") || label == "R1_KFS") {
    return cv::Scalar(50, 50, 230);
  }
  if (startsWith(label, "B_")) {
    return cv::Scalar(230, 120, 40);
  }
  if (startsWith(label, "F_")) {
    return cv::Scalar(0, 170, 255);
  }
  return cv::Scalar(220, 220, 220);
}

void drawDetectionsOverlay(
    cv::Mat &canvas, const std::vector<rc26_vision::Detection> &detections,
    const cv::Scalar &text_bg,
    const std::optional<rc26_vision::VisualTargetSnapshot> &locked_target =
        std::nullopt) {
  const cv::Size frame_size = canvas.size();
  for (const auto &det : detections) {
    const std::string label = rc26_vision::visualTargetLabel(det);
    const cv::Scalar color = detectionColor(label);
    const cv::Rect2f raw_box(det.x1, det.y1, det.x2 - det.x1,
                             det.y2 - det.y1);
    if (const auto box = clippedRect(raw_box, frame_size)) {
      int thickness = 2;
      if (locked_target.has_value()) {
        const auto candidate =
            rc26_vision::makeVisualTargetSnapshot(det, locked_target->sequence);
        if (candidate.label == locked_target->label &&
            rc26_vision::bboxIou(*locked_target, candidate) >= 0.30) {
          thickness = 3;
        }
      }
      cv::rectangle(canvas, *box, color, thickness);
      const cv::Point2f center = detectionCenter(det);
      if (center.x >= 0.0F && center.x < frame_size.width &&
          center.y >= 0.0F && center.y < frame_size.height) {
        const cv::Point center_i(static_cast<int>(std::lround(center.x)),
                                 static_cast<int>(std::lround(center.y)));
        cv::circle(canvas, center_i, 3, color, cv::FILLED, cv::LINE_AA);
      }
      std::ostringstream det_text;
      det_text << (label.empty() ? "-" : label) << " " << std::fixed
               << std::setprecision(2) << det.score;
      drawTextWithBackground(canvas, det_text.str(), cv::Point(box->x, box->y - 4),
                             cv::Scalar(255, 255, 255), text_bg);
    }
  }
}

bool setupDebugWindowIfNeeded(rclcpp::Node *node,
                              const SecondPreselectionParams &params,
                              bool &window_active,
                              bool &disabled_after_error,
                              const char *log_prefix) {
  if (!params.dynamic_roi_ui_enable || disabled_after_error) {
    return false;
  }
  if (window_active) {
    return true;
  }
  try {
    cv::namedWindow(params.dynamic_roi_ui_window_name, cv::WINDOW_NORMAL);
    window_active = true;
    return true;
  } catch (const cv::Exception &e) {
    disabled_after_error = true;
    window_active = false;
    if (node) {
      RCLCPP_WARN(node->get_logger(), "%s UI 创建失败，自动关闭 UI：%s",
                  log_prefix, e.what());
    }
    return false;
  }
}

void releaseDebugWindow(rclcpp::Node *node,
                        const SecondPreselectionParams &params,
                        bool &window_active, const char *log_prefix) {
  if (!window_active) {
    return;
  }
  try {
    cv::destroyWindow(params.dynamic_roi_ui_window_name);
  } catch (const cv::Exception &e) {
    if (node) {
      RCLCPP_WARN(node->get_logger(), "%s UI 关闭异常：%s", log_prefix,
                  e.what());
    }
  }
  window_active = false;
}

std::optional<double> estimateMonocularDepth(
    const rc26_vision::Detection &detection, double locked_depth_m,
    const SecondPreselectionParams &params, double min_depth_m,
    double max_depth_m) {
  if (!params.kfs_mono_distance_fallback_enable ||
      !std::isfinite(locked_depth_m) || locked_depth_m <= 0.0) {
    return std::nullopt;
  }
  const double width_px =
      std::abs(static_cast<double>(detection.x2) -
               static_cast<double>(detection.x1));
  const double height_px =
      std::abs(static_cast<double>(detection.y2) -
               static_cast<double>(detection.y1));
  if (width_px < params.kfs_mono_min_bbox_px ||
      height_px < params.kfs_mono_min_bbox_px ||
      params.kfs_mono_fx_px <= 0.0 || params.kfs_mono_fy_px <= 0.0 ||
      params.kfs_mono_target_width_m <= 0.0 ||
      params.kfs_mono_target_height_m <= 0.0) {
    return std::nullopt;
  }
  const double z_width =
      params.kfs_mono_target_width_m * params.kfs_mono_fx_px / width_px;
  const double z_height =
      params.kfs_mono_target_height_m * params.kfs_mono_fy_px / height_px;
  const double depth_m = 0.5 * (z_width + z_height);
  if (!std::isfinite(depth_m) || depth_m < min_depth_m ||
      depth_m > max_depth_m ||
      std::abs(depth_m - locked_depth_m) >
          params.kfs_mono_max_delta_from_locked_m) {
    return std::nullopt;
  }
  return depth_m;
}

std::optional<double> sampleKfsDepthFromBbox(
    const cv::Mat &depth, const rc26_vision::Detection &detection,
    const rc26_vision::DepthRoiSamplerConfig &config,
    const std::vector<double> &sample_ratios, int min_success_count) {
  std::vector<double> values;
  const int required = std::max(1, min_success_count);
  for (const double ry : sample_ratios) {
    if (!std::isfinite(ry)) {
      continue;
    }
    for (const double rx : sample_ratios) {
      if (!std::isfinite(rx)) {
        continue;
      }
      const double clamped_rx = std::clamp(rx, 0.0, 1.0);
      const double clamped_ry = std::clamp(ry, 0.0, 1.0);
      const int cx = static_cast<int>(std::lround(
          static_cast<double>(detection.x1) +
          (static_cast<double>(detection.x2) -
           static_cast<double>(detection.x1)) *
              clamped_rx));
      const int cy = static_cast<int>(std::lround(
          static_cast<double>(detection.y1) +
          (static_cast<double>(detection.y2) -
           static_cast<double>(detection.y1)) *
              clamped_ry));
      const auto sampled = rc26_vision::sampleMedianDepth(depth, cx, cy, config);
      if (sampled.has_value()) {
        values.push_back(*sampled);
      }
    }
  }
  if (static_cast<int>(values.size()) < required) {
    return std::nullopt;
  }
  std::sort(values.begin(), values.end());
  return values[values.size() / 2];
}

} // namespace

double secondPreselectionKfsApproachDistance(
    double locked_depth_m, const SecondPreselectionParams &params) {
  const double sign = params.kfs_approach_x_sign < 0 ? -1.0 : 1.0;
  return sign * std::max(0.0, locked_depth_m - params.kfs_grab_distance_m);
}

double secondPreselectionProjectedX(double origin_x, double origin_y,
                                    double origin_yaw, double current_x,
                                    double current_y) {
  const double dx = current_x - origin_x;
  const double dy = current_y - origin_y;
  return dx * std::cos(origin_yaw) + dy * std::sin(origin_yaw);
}

double secondPreselectionTotalXRemainingToDrive(
    double projected_x_m, const SecondPreselectionParams &params) {
  return std::max(0.0, params.total_x_target_m - projected_x_m);
}

double secondPreselectionRampRemainingToDrive(
    double projected_x_m, const SecondPreselectionParams &params) {
  return std::max(0.0, params.ramp_forward_x_m - projected_x_m);
}

bool secondPreselectionRampTimedOut(double elapsed_s, double timeout_s) {
  return std::isfinite(timeout_s) && timeout_s > 0.0 && elapsed_s >= timeout_s;
}

double secondPreselectionPlaceAvoidanceDistance(
    int avoidance_stage, const SecondPreselectionParams &params) {
  const double mirror = params.team_mirror_sign < 0 ? -1.0 : 1.0;
  if (avoidance_stage == 1) {
    return mirror * params.place_occupied_first_lateral_m;
  }
  if (avoidance_stage == 2) {
    return -mirror * params.place_occupied_second_reverse_m;
  }
  return 0.0;
}

SecondPreselectionLayerObservation secondPreselectionFrameLayers(
    const std::vector<rc26_vision::Detection> &detections, int frame_width,
    int frame_height, const SecondPreselectionParams &params) {
  SecondPreselectionLayerObservation result;
  if (frame_width <= 0 || frame_height <= 0) {
    return result;
  }

  const double min_x =
      static_cast<double>(frame_width) * params.place_occupied_center_x_min_ratio;
  const double max_x =
      static_cast<double>(frame_width) * params.place_occupied_center_x_max_ratio;
  const double middle_min_y =
      static_cast<double>(frame_height) * params.place_occupied_middle_y_min_ratio;
  const double middle_max_y =
      static_cast<double>(frame_height) * params.place_occupied_middle_y_max_ratio;
  const double lower_min_y =
      static_cast<double>(frame_height) * params.place_occupied_lower_y_min_ratio;
  const double lower_max_y =
      static_cast<double>(frame_height) * params.place_occupied_lower_y_max_ratio;

  for (const auto &det : detections) {
    const std::string label = rc26_vision::visualTargetLabel(det);
    const bool is_r2 =
        labelsMatch(label, params.r2_target_labels,
                    params.r2_target_label_prefixes);
    const bool is_r1 =
        labelsMatch(label, params.r1_blocking_labels,
                    params.r1_blocking_label_prefixes);
    if (!(is_r2 || is_r1 || isKfsLabel(label))) {
      continue;
    }
    if (label == "R1_KFS" && det.score < params.r1_kfs_min_score) {
      continue;
    }

    const auto center = detectionCenter(det);
    if (center.x < min_x || center.x > max_x) {
      continue;
    }
    if (center.y >= middle_min_y && center.y < middle_max_y) {
      result.middle = true;
    }
    if (center.y >= lower_min_y && center.y <= lower_max_y) {
      result.lower = true;
    }
  }

  return result;
}

bool secondPreselectionPlaceApproachTimedOut(double elapsed_s,
                                             double timeout_s) {
  return std::isfinite(elapsed_s) && std::isfinite(timeout_s) &&
         timeout_s > 0.0 && elapsed_s > timeout_s;
}

bool secondPreselectionPlaceObserveTimedOut(double elapsed_s,
                                            double timeout_s) {
  return std::isfinite(elapsed_s) && std::isfinite(timeout_s) &&
         timeout_s > 0.0 && elapsed_s > timeout_s;
}

bool secondPreselectionConsumeNewFrameSequence(int64_t sequence,
                                               int64_t &last_sequence) {
  if (sequence <= 0 || sequence == last_sequence) {
    return false;
  }
  last_sequence = sequence;
  return true;
}

bool secondPreselectionFrameOccupied(
    const std::vector<rc26_vision::Detection> &detections, int frame_width,
    int frame_height, const SecondPreselectionParams &params) {
  return secondPreselectionFrameLayers(detections, frame_width, frame_height,
                                       params)
      .middle;
}

bool secondPreselectionFrameHasCenterKfs(
    const std::vector<rc26_vision::Detection> &detections, int frame_width,
    int frame_height, const SecondPreselectionParams &params) {
  if (frame_width <= 0 || frame_height <= 0) {
    return false;
  }
  const double min_x =
      static_cast<double>(frame_width) * params.place_occupied_center_x_min_ratio;
  const double max_x =
      static_cast<double>(frame_width) * params.place_occupied_center_x_max_ratio;
  const double min_y = static_cast<double>(frame_height) *
                       params.place_occupied_middle_y_min_ratio;
  const double max_y = static_cast<double>(frame_height);
  for (const auto &det : detections) {
    const std::string label = rc26_vision::visualTargetLabel(det);
    const bool is_r2 =
        labelsMatch(label, params.r2_target_labels,
                    params.r2_target_label_prefixes);
    const bool is_r1 =
        labelsMatch(label, params.r1_blocking_labels,
                    params.r1_blocking_label_prefixes);
    if (!(is_r2 || is_r1 || isKfsLabel(label))) {
      continue;
    }
    if (label == "R1_KFS" && det.score < params.r1_kfs_min_score) {
      continue;
    }
    const auto center = detectionCenter(det);
    if (center.x >= min_x && center.x <= max_x && center.y >= min_y &&
        center.y <= max_y) {
      return true;
    }
  }
  return false;
}

bool secondPreselectionClimbPlaceReadyForFinal(double elapsed_s,
                                               double required_delay_s,
                                               bool pickup_done) {
  const double normalized_delay =
      std::isfinite(required_delay_s) ? std::max(0.0, required_delay_s) : 0.0;
  return pickup_done && std::isfinite(elapsed_s) &&
         elapsed_s >= normalized_delay;
}

SecondPreselectionOccupancyDecision
secondPreselectionUpdateOccupancyStability(bool occupied, int stable_frames,
                                           int &occupied_count,
                                           int &clear_count) {
  const int required = std::max(1, stable_frames);
  if (occupied) {
    ++occupied_count;
    clear_count = 0;
  } else {
    ++clear_count;
    occupied_count = 0;
  }
  if (occupied_count >= required) {
    return SecondPreselectionOccupancyDecision::Occupied;
  }
  if (clear_count >= required) {
    return SecondPreselectionOccupancyDecision::Clear;
  }
  return SecondPreselectionOccupancyDecision::Pending;
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
  command_error_seen_ = false;
  command_busy_seen_ = false;
  command_seq_ = -1;
  command_error_detail_.clear();
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
  if (command_error_seen_.load(std::memory_order_relaxed)) {
    return fail(command_error_detail_.empty()
                    ? "机构命令收到 MCU 0xFE 最终错误：" + command_label_ +
                          " seq=" + std::to_string(seq)
                    : command_error_detail_);
  }
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
    const bool busy_seen = command_busy_seen_.load(std::memory_order_relaxed);
    RCLCPP_INFO(node_->get_logger(),
                "第二预选赛等待机构完成反馈：%s feedback=%s seq=%d elapsed=%.1fs busy=%s",
                command_label_.c_str(), byteHex(done_feedback_id_).c_str(), seq,
                elapsedSec(phase_tp_), busy_seen ? "是" : "否");
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
  std::optional<MechanismErrorDiagnostic> diagnostic;
  if (isSameSeqMechanismError(*msg, seq, diagnostic)) {
    const std::string detail = mechanismErrorDiagnosticText(*diagnostic);
    if (diagnostic->busy) {
      command_busy_seen_.store(true, std::memory_order_relaxed);
      if (node_) {
        RCLCPP_INFO(node_->get_logger(),
                    "第二预选赛机构命令仍在处理中：%s", detail.c_str());
      }
    } else {
      command_error_detail_ = detail;
      command_error_seen_.store(true, std::memory_order_relaxed);
      if (node_) {
        RCLCPP_ERROR(node_->get_logger(),
                     "第二预选赛机构命令收到 MCU 错误：%s", detail.c_str());
      }
    }
    return;
  }
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
  request->wait_ack = true;

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

SecondPreselectionKfsPickupAction::SecondPreselectionKfsPickupAction(
    const std::string &name, const BT::NodeConfig &config)
    : BT::StatefulActionNode(name, config) {}

SecondPreselectionKfsPickupAction::~SecondPreselectionKfsPickupAction() {
  clearRuntimeState();
}

BT::NodeStatus SecondPreselectionKfsPickupAction::onStart() {
  if (!config().blackboard || !config().blackboard->get("node", node_) ||
      !node_) {
    writeDecisionFailure(config().blackboard, "SecondPreselectionKfsPickup",
                         "运行上下文缺失：blackboard 或 node 不可用");
    return BT::NodeStatus::FAILURE;
  }
  if (!config().blackboard->get("second_preselection_params", params_)) {
    return fail("黑板缺少 second_preselection_params");
  }

  clearRuntimeState();
  config().blackboard->set("second_preselect_total_x_origin_valid", false);
  config().blackboard->set("second_preselect_total_x_progress_m", 0.0);
  if (!setupOdom()) {
    return fail("第二预选赛 KFS 夹取 odom 启动失败");
  }
  if (!setupVision()) {
    return fail("第二预选赛 KFS 夹取视觉启动失败");
  }
  if (!setupCommandIo()) {
    return fail("第二预选赛 KFS 夹取机构命令 IO 启动失败");
  }
  cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
      params_.cmd_vel_topic, rclcpp::QoS(10));
  if (!cmd_pub_) {
    return fail("第二预选赛 KFS 夹取 cmd_vel publisher 创建失败");
  }

  phase_ = Phase::Search;
  phase_tp_ = std::chrono::steady_clock::now();
  last_log_tp_ = phase_tp_;
  RCLCPP_INFO(node_->get_logger(),
              "第二预选赛开始前进搜索 KFS：speed=%.3fm/s timeout=%.1fs model=%s cmd_vel=%s odom=%s command=0x%02X",
              params_.search_forward_speed_mps, params_.search_timeout_s,
              params_.model_id.c_str(), params_.cmd_vel_topic.c_str(),
              params_.odom_topic.c_str(), params_.pickup_command_id & 0xFF);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SecondPreselectionKfsPickupAction::onRunning() {
  switch (phase_) {
  case Phase::Search:
    return tickSearch();
  case Phase::VisualAlign:
    return tickVisualAlign();
  case Phase::SendingPreApproachLower:
    return tickSendingPreApproachLower();
  case Phase::WaitingPreApproachLowerAck:
    return tickWaitingPreApproachLowerAck();
  case Phase::WaitingPreApproachLowerDone:
    return tickWaitingPreApproachLowerDone();
  case Phase::PreApproachLowerSettle:
    return tickPreApproachLowerSettle();
  case Phase::OdomApproach:
    return tickOdomApproach();
  case Phase::SendingPickup:
    return tickSendingPickup();
  case Phase::WaitingPickupAck:
    return tickWaitingPickupAck();
  case Phase::WaitingPickupDone:
    return tickWaitingPickupDone();
  case Phase::GrabVerify:
    return tickGrabVerify();
  case Phase::Settle:
    return tickSettle();
  }
  return fail("第二预选赛 KFS 夹取未知状态");
}

void SecondPreselectionKfsPickupAction::onHalted() {
  publishStop();
  clearRuntimeState();
}

BT::NodeStatus SecondPreselectionKfsPickupAction::fail(
    const std::string &reason) {
  if (node_) {
    RCLCPP_ERROR(node_->get_logger(), "第二预选赛 KFS 夹取失败：%s",
                 reason.c_str());
  }
  writeDecisionFailure(config().blackboard, "SecondPreselectionKfsPickup",
                       reason);
  publishStop();
  clearRuntimeState();
  return BT::NodeStatus::FAILURE;
}

bool SecondPreselectionKfsPickupAction::setupVision() {
  try {
    params_.vision_config_file = resolveVisionConfig(params_.vision_config_file);
    auto config =
        rc26_vision::ProfileLoader::loadFromYaml(params_.vision_config_file);
    rc26_vision::ProfileLoader::validate(config);
    if (config.profiles.find(params_.model_id) == config.profiles.end()) {
      RCLCPP_ERROR(node_->get_logger(), "第二预选赛 KFS 视觉 profile 不存在：%s",
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
    RCLCPP_ERROR(node_->get_logger(), "第二预选赛 KFS 视觉初始化异常：%s",
                 e.what());
    vision_.reset();
    return false;
  }
}

void SecondPreselectionKfsPickupAction::releaseVision() {
  if (vision_) {
    vision_->stop();
    vision_.reset();
  }
}

bool SecondPreselectionKfsPickupAction::setupOdom() {
  odom_sub_ = node_->create_subscription<OdomMsg>(
      params_.odom_topic, rclcpp::QoS(rclcpp::KeepLast(10)),
      [this](const OdomMsg::SharedPtr msg) {
        if (!msg) {
          return;
        }
        odom_x_ = msg->pose.pose.position.x;
        odom_y_ = msg->pose.pose.position.y;
        const auto &q = msg->pose.pose.orientation;
        odom_yaw_ = yawFromQuaternion(q.x, q.y, q.z, q.w);
        has_odom_ = true;
        last_odom_tp_ = std::chrono::steady_clock::now();
      });
  return static_cast<bool>(odom_sub_);
}

void SecondPreselectionKfsPickupAction::releaseOdom() { odom_sub_.reset(); }

bool SecondPreselectionKfsPickupAction::setupCommandIo() {
  send_client_ = node_->create_client<SendCommandSrv>(params_.send_command_service);
  feedback_sub_ = node_->create_subscription<FeedbackMsg>(
      params_.feedback_topic, rclcpp::QoS(32).reliable(),
      [this](const FeedbackMsg::SharedPtr msg) { handleFeedback(msg); });
  return static_cast<bool>(send_client_) && static_cast<bool>(feedback_sub_);
}

void SecondPreselectionKfsPickupAction::releaseCommandIo() {
  feedback_sub_.reset();
  send_client_.reset();
}

bool SecondPreselectionKfsPickupAction::setupUiIfNeeded() {
  return setupDebugWindowIfNeeded(node_, params_, ui_window_active_,
                                  ui_disabled_after_error_,
                                  "第二预选赛 KFS 识别");
}

void SecondPreselectionKfsPickupAction::releaseUi() {
  releaseDebugWindow(node_, params_, ui_window_active_,
                     "第二预选赛 KFS 识别");
}

void SecondPreselectionKfsPickupAction::renderKfsUi(
    const std::string &stage,
    const std::optional<KfsObservation> &observation,
    const std::string &detail) {
  if (!params_.dynamic_roi_ui_enable || ui_disabled_after_error_ || !vision_ ||
      !vision_->isRunning()) {
    return;
  }
  rc26_vision::VisionInferenceManager::FrameSnapshot snapshot;
  if (!vision_->getLatestFrameSnapshot(snapshot) || !snapshot.has_color ||
      snapshot.color_bgr.empty() || snapshot.display_sequence <= 0) {
    return;
  }
  if (!setupUiIfNeeded()) {
    return;
  }

  try {
    cv::Mat canvas = snapshot.color_bgr.clone();
    const cv::Scalar text_bg(20, 20, 20);
    std::optional<rc26_vision::VisualTargetSnapshot> locked_target;
    if (has_pickup_target_) {
      locked_target = pickup_target_;
    }
    drawDetectionsOverlay(canvas, snapshot.detections, text_bg, locked_target);

    const int target_line_x =
        std::max(0, canvas.cols / 2) + params_.kfs_align_target_line_offset_px;
    if (target_line_x >= 0 && target_line_x < canvas.cols) {
      cv::line(canvas, cv::Point(target_line_x, 0),
               cv::Point(target_line_x, canvas.rows - 1),
               cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
    }

    std::ostringstream status;
    status << "stage=" << stage << " det=" << snapshot.detections.size()
           << " seq=" << snapshot.display_sequence;
    if (observation.has_value()) {
      status << " target=" << observation->target.label << " depth="
             << std::fixed << std::setprecision(3)
             << observation->target.distance_m << "m offset="
             << observation->offset_px << " stable=" << align_stable_count_
             << "/" << params_.kfs_align_stable_frames;
    } else if (has_pickup_target_) {
      status << " target=" << pickup_target_.label;
    }
    drawTextWithBackground(canvas, status.str(), cv::Point(8, 20),
                           cv::Scalar(255, 255, 255), text_bg, 0.50, 1);
    if (!detail.empty()) {
      drawTextWithBackground(canvas, detail, cv::Point(8, 42),
                             cv::Scalar(255, 255, 255), text_bg, 0.50, 1);
    }

    cv::imshow(params_.dynamic_roi_ui_window_name, canvas);
    cv::waitKey(1);
  } catch (const cv::Exception &e) {
    ui_disabled_after_error_ = true;
    if (node_) {
      RCLCPP_WARN(node_->get_logger(),
                  "第二预选赛 KFS 识别 UI 渲染失败，自动关闭 UI：%s",
                  e.what());
    }
    releaseUi();
  }
}

void SecondPreselectionKfsPickupAction::publishStop() {
  if (!cmd_pub_) {
    return;
  }
  cmd_pub_->publish(geometry_msgs::msg::Twist{});
}

void SecondPreselectionKfsPickupAction::publishTwist(double vx, double vy,
                                                     double wz) {
  if (!cmd_pub_) {
    return;
  }
  geometry_msgs::msg::Twist twist;
  twist.linear.x = vx;
  twist.linear.y = vy;
  twist.angular.z = wz;
  cmd_pub_->publish(twist);
}

bool SecondPreselectionKfsPickupAction::odomReady() const {
  if (!has_odom_) {
    return false;
  }
  return elapsedSec(last_odom_tp_) <= params_.odom_timeout_s;
}

double
SecondPreselectionKfsPickupAction::headingAngularZ(double target_yaw_rad) const {
  const double yaw_error = normalizeAngle(target_yaw_rad - odom_yaw_);
  double out = params_.kfs_heading_kp * yaw_error;
  const double heading_max = std::max(0.0, params_.kfs_heading_max_speed_radps);
  if (std::abs(out) > heading_max) {
    out = std::copysign(heading_max, out);
  }
  return out;
}

rc26_vision::TipAlignmentConfig
SecondPreselectionKfsPickupAction::makeAlignmentConfig() const {
  rc26_vision::TipAlignmentConfig config;
  config.target_lock_enable = true;
  config.target_lock_max_jump_px = params_.kfs_align_max_jump_px;
  config.lost_stop_frames = params_.kfs_lost_stop_frames;
  config.tolerance_px = params_.kfs_align_tolerance_px;
  config.target_line_offset_px = params_.kfs_align_target_line_offset_px;
  config.kp = params_.kfs_align_kp;
  config.min_speed_mps = params_.kfs_align_min_speed_mps;
  config.max_speed_mps = params_.kfs_align_max_speed_mps;
  config.invert_direction = params_.kfs_invert_lateral_direction;
  config.heading_hold_enable = true;
  config.target_yaw_rad = align_yaw_;
  config.heading_kp = params_.kfs_heading_kp;
  config.heading_max_speed_radps = params_.kfs_heading_max_speed_radps;
  config.heading_tolerance_rad =
      std::abs(params_.kfs_odom_yaw_tolerance_deg) * kDeg2Rad;
  config.heading_gate_rad =
      std::max(config.heading_tolerance_rad,
               std::abs(params_.kfs_align_heading_gate_deg) * kDeg2Rad);
  return config;
}

SecondPreselectionKfsPickupAction::KfsObservation
SecondPreselectionKfsPickupAction::applyAlignmentObservationFilter(
    const KfsObservation &observation) {
  KfsObservation filtered = observation;
  const double raw_offset = static_cast<double>(observation.offset_px);
  const double alpha =
      std::clamp(params_.kfs_align_offset_filter_alpha, 0.05, 1.0);
  if (!align_filtered_offset_valid_) {
    align_filtered_offset_px_ = raw_offset;
    align_filtered_offset_valid_ = true;
  } else {
    align_filtered_offset_px_ =
        alpha * raw_offset + (1.0 - alpha) * align_filtered_offset_px_;
  }
  filtered.offset_px = static_cast<int>(std::lround(align_filtered_offset_px_));
  return filtered;
}

std::optional<rc26_vision::TipHeadingControl>
SecondPreselectionKfsPickupAction::alignHeadingControl() {
  if (!align_yaw_captured_) {
    if (!odomReady()) {
      if (node_ && !align_waiting_odom_logged_) {
        RCLCPP_WARN(node_->get_logger(),
                    "第二预选赛 KFS 视觉对齐等待 odom 捕获 yaw：topic=%s",
                    params_.odom_topic.c_str());
        align_waiting_odom_logged_ = true;
      }
      return std::nullopt;
    }
    align_yaw_ = odom_yaw_;
    align_yaw_captured_ = true;
    align_waiting_odom_logged_ = false;
    RCLCPP_INFO(node_->get_logger(),
                "第二预选赛 KFS 视觉对齐捕获 yaw=%.3f", align_yaw_);
  }
  if (!odomReady()) {
    if (node_ && !align_waiting_odom_logged_) {
      RCLCPP_WARN(node_->get_logger(),
                  "第二预选赛 KFS 视觉对齐等待 odom 新鲜：topic=%s",
                  params_.odom_topic.c_str());
      align_waiting_odom_logged_ = true;
    }
    return std::nullopt;
  }
  align_waiting_odom_logged_ = false;
  return rc26_vision::computeTipHeadingControl(odom_yaw_,
                                               makeAlignmentConfig());
}

std::optional<SecondPreselectionKfsPickupAction::KfsObservation>
SecondPreselectionKfsPickupAction::findNearestKfs(
    bool allow_depthless_r2_align) {
  if (!vision_ || !vision_->isRunning()) {
    return std::nullopt;
  }
  rc26_vision::VisionInferenceManager::FrameSnapshot snapshot;
  if (!vision_->getLatestFrameSnapshot(snapshot) || !snapshot.has_color ||
      snapshot.color_bgr.empty() || !snapshot.has_depth ||
      snapshot.depth.empty() || snapshot.display_sequence <= 0) {
    return std::nullopt;
  }

  rc26_vision::DepthRoiSamplerConfig depth_config;
  depth_config.roi_size = params_.kfs_depth_roi_size;
  depth_config.min_valid_count = params_.kfs_depth_min_valid_count;
  depth_config.min_depth_m = params_.depth_min_m;
  depth_config.max_depth_m = params_.depth_max_m;

  std::optional<KfsObservation> best;
  const int target_line_x =
      std::max(0, snapshot.color_bgr.cols / 2) +
      params_.kfs_align_target_line_offset_px;
  for (const auto &det : snapshot.detections) {
    const std::string label = rc26_vision::visualTargetLabel(det);
    const bool is_r2 =
        labelsMatch(label, params_.r2_target_labels,
                    params_.r2_target_label_prefixes);
    const bool is_r1 =
        labelsMatch(label, params_.r1_blocking_labels,
                    params_.r1_blocking_label_prefixes);
    const bool is_kfs = is_r2 || is_r1 || isKfsLabel(label);
    if (!is_kfs) {
      continue;
    }
    const auto center = detectionCenter(det);
    const auto sampled = sampleKfsDepthFromBbox(
        snapshot.depth, det, depth_config, params_.kfs_depth_bbox_sample_ratios,
        params_.kfs_depth_bbox_min_success_count);
    bool has_depth = sampled.has_value();
    bool real_depth = has_depth;
    double depth_m = sampled.value_or(0.0);
    if (!has_depth && allow_depthless_r2_align) {
      const auto mono = estimateMonocularDepth(det, last_real_depth_m_, params_,
                                               params_.depth_min_m,
                                               params_.depth_max_m);
      if (mono.has_value()) {
        has_depth = true;
        real_depth = false;
        depth_m = *mono;
      }
    }
    if (!has_depth) {
      continue;
    }

    KfsObservation observation;
    observation.kind = is_r2 ? KfsObservation::Kind::R2 : KfsObservation::Kind::R1;
    observation.target =
        rc26_vision::makeVisualTargetSnapshot(det, snapshot.display_sequence);
    observation.target.distance_m = depth_m;
    observation.detection = det;
    observation.offset_px =
        static_cast<int>(std::lround(center.x - target_line_x));
    observation.has_depth = true;
    observation.real_depth = real_depth;
    observation.depth_detail = real_depth ? "center_roi" : "monocular_bbox";

    if (!best.has_value()) {
      best = observation;
      continue;
    }
    const double depth_delta =
        observation.target.distance_m - best->target.distance_m;
    if (depth_delta < -1e-9 ||
        (std::abs(depth_delta) <= 1e-9 &&
         (std::abs(observation.offset_px) < std::abs(best->offset_px) ||
          (std::abs(observation.offset_px) == std::abs(best->offset_px) &&
           observation.target.score > best->target.score)))) {
      best = observation;
    }
  }

  if (!best.has_value()) {
    return best;
  }

  std::vector<rc26_vision::Detection> selected_detection{best->detection};
  std::vector<int> target_class_ids{best->detection.class_id};
  const auto selection = rc26_vision::updateTipAlignmentTarget(
      selected_detection, snapshot.color_bgr.cols, target_class_ids,
      align_lock_state_, makeAlignmentConfig());
  if (!selection.has_target || selection.target.source_index != 0) {
    return std::nullopt;
  }
  best->offset_px = selection.offset_px;
  align_target_lock_sequence_ = best->target.sequence;
  align_last_observation_ = best;
  if (best->real_depth) {
    last_real_depth_m_ = best->target.distance_m;
  }
  return best;
}

void SecondPreselectionKfsPickupAction::beginVisualAlign(
    const KfsObservation &observation) {
  publishStop();
  pickup_target_ = observation.target;
  has_pickup_target_ = true;
  align_lock_state_.reset();
  align_last_observation_ = observation;
  align_target_lock_sequence_ = 0;
  align_stable_count_ = 0;
  align_lost_count_ = 0;
  align_last_sequence_ = observation.target.sequence;
  align_waiting_verify_frame_ = false;
  align_yaw_captured_ = false;
  align_waiting_odom_logged_ = false;
  align_filtered_offset_valid_ = true;
  align_filtered_offset_px_ = static_cast<double>(observation.offset_px);
  if (observation.real_depth) {
    last_real_depth_m_ = observation.target.distance_m;
  }
  phase_ = Phase::VisualAlign;
  phase_tp_ = std::chrono::steady_clock::now();
  RCLCPP_INFO(node_->get_logger(),
              "第二预选赛发现 KFS，进入视觉对齐并夹取：label=%s seq=%ld depth=%.3fm offset=%d",
              observation.target.label.c_str(),
              static_cast<long>(observation.target.sequence),
              observation.target.distance_m, observation.offset_px);
}

BT::NodeStatus SecondPreselectionKfsPickupAction::tickSearch() {
  renderKfsUi("search", std::nullopt, "searching nearest KFS");
  if (elapsedSec(phase_tp_) > params_.search_timeout_s) {
    return fail("第二预选赛前进搜索 KFS 超时");
  }
  if (!odomReady()) {
    publishStop();
    if (elapsedSec(last_log_tp_) >= params_.log_period_s) {
      RCLCPP_WARN(node_->get_logger(),
                  "第二预选赛前进搜索等待 odom：topic=%s elapsed=%.1fs",
                  params_.odom_topic.c_str(), elapsedSec(phase_tp_));
      last_log_tp_ = std::chrono::steady_clock::now();
    }
    return BT::NodeStatus::RUNNING;
  }
  if (!search_yaw_captured_) {
    search_yaw_ = odom_yaw_;
    search_yaw_captured_ = true;
    search_origin_x_ = odom_x_;
    search_origin_y_ = odom_y_;
    search_origin_captured_ = true;
    config().blackboard->set("second_preselect_total_x_origin_x",
                             search_origin_x_);
    config().blackboard->set("second_preselect_total_x_origin_y",
                             search_origin_y_);
    config().blackboard->set("second_preselect_total_x_origin_yaw",
                             search_yaw_);
    config().blackboard->set("second_preselect_total_x_origin_valid", true);
    RCLCPP_INFO(node_->get_logger(),
                "第二预选赛总 X 原点已记录：x=%.3f y=%.3f yaw=%.3f",
                search_origin_x_, search_origin_y_, search_yaw_);
  }

  const auto observation = findNearestKfs(false);
  if (observation.has_value()) {
    renderKfsUi("search-hit", observation, "nearest KFS locked");
    beginVisualAlign(*observation);
    return BT::NodeStatus::RUNNING;
  }

  const double wz = headingAngularZ(search_yaw_);
  publishTwist(params_.search_forward_speed_mps, 0.0, wz);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SecondPreselectionKfsPickupAction::tickVisualAlign() {
  if (elapsedSec(phase_tp_) > params_.kfs_align_timeout_s) {
    if (align_last_observation_.has_value() &&
        align_last_observation_->has_depth &&
        std::abs(align_last_observation_->offset_px) <=
            params_.kfs_align_timeout_pickup_tolerance_px) {
      beginPreApproachLowerCommand(*align_last_observation_);
      return BT::NodeStatus::RUNNING;
    }
    return fail("第二预选赛 KFS 视觉对齐超时");
  }

  auto observation = findNearestKfs(true);
  if (!observation.has_value()) {
    ++align_lost_count_;
    if (align_lost_count_ > 2) {
      align_stable_count_ = 0;
    }
    std::ostringstream lost_detail;
    lost_detail << "target temporarily lost " << align_lost_count_ << "/"
                << params_.kfs_lost_stop_frames;
    renderKfsUi("align-latched", align_last_observation_, lost_detail.str());

    const auto heading = alignHeadingControl();
    if (!heading.has_value() || !align_last_observation_.has_value() ||
        align_lost_count_ >= params_.kfs_lost_stop_frames) {
      if (align_lost_count_ >= params_.kfs_lost_stop_frames) {
        align_filtered_offset_valid_ = false;
      }
      publishStop();
    } else {
      const int latched_offset = align_last_observation_->offset_px;
      const bool pixel_aligned =
          std::abs(latched_offset) <= params_.kfs_align_tolerance_px;
      if (pixel_aligned && heading->aligned) {
        publishStop();
      } else {
        double vy = 0.0;
        if (heading->allow_lateral) {
          vy = rc26_vision::computeTipAlignmentVy(latched_offset,
                                                  makeAlignmentConfig()) *
               params_.kfs_lost_servo_speed_scale;
        }
        publishTwist(0.0, vy, heading->angular_z_radps);
      }
    }
    return BT::NodeStatus::RUNNING;
  }
  const int recovered_lost_count = align_lost_count_;
  observation = applyAlignmentObservationFilter(*observation);
  renderKfsUi("align", observation,
              recovered_lost_count > 0 ? "relocked with filtered offset"
                                       : "pixel/yaw alignment");

  if (!align_waiting_verify_frame_) {
    align_waiting_verify_frame_ = true;
    RCLCPP_INFO(node_->get_logger(),
                "第二预选赛 KFS 使用 tip_alignment 口径等待稳定帧：target_offset=%d tolerance=%d stable=%d",
                params_.kfs_align_target_line_offset_px,
                params_.kfs_align_tolerance_px,
                params_.kfs_align_stable_frames);
  }

  const auto heading = alignHeadingControl();
  if (!heading.has_value()) {
    publishStop();
    align_stable_count_ = 0;
    return BT::NodeStatus::RUNNING;
  }

  align_lost_count_ = 0;
  align_last_observation_ = observation;
  if (observation->real_depth) {
    last_real_depth_m_ = observation->target.distance_m;
  }
  pickup_target_ = observation->target;
  has_pickup_target_ = true;

  const bool new_frame = observation->target.sequence != align_last_sequence_;
  if (new_frame) {
    align_last_sequence_ = observation->target.sequence;
  }
  const bool pixel_aligned =
      std::abs(observation->offset_px) <= params_.kfs_align_tolerance_px;
  const bool aligned = pixel_aligned && heading->aligned;
  if (aligned) {
    if (new_frame) {
      ++align_stable_count_;
    }
    publishStop();
    if (align_stable_count_ >= params_.kfs_align_stable_frames) {
      beginPreApproachLowerCommand(*observation);
      return BT::NodeStatus::RUNNING;
    }
    return BT::NodeStatus::RUNNING;
  }

  if (new_frame) {
    align_stable_count_ = 0;
  }
  const double vy = heading->allow_lateral
                        ? rc26_vision::computeTipAlignmentVy(
                              observation->offset_px, makeAlignmentConfig())
                        : 0.0;
  publishTwist(0.0, vy, heading->angular_z_radps);
  return BT::NodeStatus::RUNNING;
}

void SecondPreselectionKfsPickupAction::beginPreApproachLowerCommand(
    const KfsObservation &observation) {
  pre_approach_observation_ = observation;
  pickup_target_ = observation.target;
  has_pickup_target_ = true;
  beginMechanismCommand(params_.pre_approach_lower_command_id,
                        params_.pre_approach_lower_done_feedback_id,
                        "SECOND_PRESELECTION_ARM_LOWER");
  phase_ = Phase::SendingPreApproachLower;
  RCLCPP_INFO(node_->get_logger(),
              "第二预选赛视觉对齐完成，先发送机械臂放下命令：command=%s done_feedback=%s settle=%.2fs",
              byteHex(params_.pre_approach_lower_command_id).c_str(),
              byteHex(params_.pre_approach_lower_done_feedback_id).c_str(),
              params_.pre_approach_lower_settle_s);
}

BT::NodeStatus
SecondPreselectionKfsPickupAction::tickSendingPreApproachLower() {
  renderKfsUi("lower-send", align_last_observation_, "sending 0x14");
  publishStop();
  if (!sendActiveCommand()) {
    if (elapsedSec(phase_tp_) > params_.command_timeout_s) {
      return fail("第二预选赛等待机械臂放下命令服务超时");
    }
    return BT::NodeStatus::RUNNING;
  }
  phase_ = Phase::WaitingPreApproachLowerAck;
  phase_tp_ = std::chrono::steady_clock::now();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus
SecondPreselectionKfsPickupAction::tickWaitingPreApproachLowerAck() {
  renderKfsUi("lower-ack", align_last_observation_, "waiting 0x14 ACK");
  publishStop();
  if (command_error_seen_.load(std::memory_order_relaxed)) {
    return fail(command_error_detail_.empty()
                    ? "第二预选赛机械臂放下命令收到 MCU 错误"
                    : command_error_detail_);
  }
  if (command_response_seen_.load(std::memory_order_relaxed)) {
    if (!command_accepted_.load(std::memory_order_relaxed)) {
      return fail("第二预选赛机械臂放下命令 ACK 被拒绝");
    }
    const int seq = command_seq_.load(std::memory_order_relaxed);
    RCLCPP_INFO(node_->get_logger(),
                "第二预选赛机械臂放下命令 ACK 成功：command=%s seq=%d，等待完成反馈 %s",
                byteHex(active_command_id_).c_str(), seq,
                byteHex(active_done_feedback_id_).c_str());
    phase_ = Phase::WaitingPreApproachLowerDone;
    phase_tp_ = std::chrono::steady_clock::now();
    last_log_tp_ = phase_tp_;
    return BT::NodeStatus::RUNNING;
  }
  if (elapsedSec(phase_tp_) > params_.command_timeout_s) {
    return fail("第二预选赛等待机械臂放下命令 ACK 超时");
  }
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus
SecondPreselectionKfsPickupAction::tickWaitingPreApproachLowerDone() {
  renderKfsUi("lower-done", align_last_observation_,
              "waiting MCU 0x12 done");
  publishStop();
  const int seq = command_seq_.load(std::memory_order_relaxed);
  if (command_error_seen_.load(std::memory_order_relaxed)) {
    return fail(command_error_detail_.empty()
                    ? "第二预选赛机械臂放下命令收到 MCU 错误"
                    : command_error_detail_);
  }
  if (command_done_feedback_seen_.load(std::memory_order_relaxed)) {
    RCLCPP_INFO(node_->get_logger(),
                "第二预选赛机械臂放下完成反馈已收到：feedback=%s seq=%d，停车等待 %.2fs",
                byteHex(active_done_feedback_id_).c_str(), seq,
                params_.pre_approach_lower_settle_s);
    phase_ = Phase::PreApproachLowerSettle;
    phase_tp_ = std::chrono::steady_clock::now();
    return BT::NodeStatus::RUNNING;
  }
  if (elapsedSec(phase_tp_) > params_.done_timeout_s) {
    return fail("等待第二预选赛机械臂放下完成反馈超时：feedback=" +
                byteHex(active_done_feedback_id_) +
                " seq=" + std::to_string(seq));
  }
  if (elapsedSec(last_log_tp_) >= params_.log_period_s) {
    const bool busy_seen = command_busy_seen_.load(std::memory_order_relaxed);
    RCLCPP_INFO(node_->get_logger(),
                "第二预选赛等待机械臂放下完成反馈：feedback=%s seq=%d elapsed=%.1fs busy=%s",
                byteHex(active_done_feedback_id_).c_str(), seq,
                elapsedSec(phase_tp_), busy_seen ? "是" : "否");
    last_log_tp_ = std::chrono::steady_clock::now();
  }
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus
SecondPreselectionKfsPickupAction::tickPreApproachLowerSettle() {
  renderKfsUi("lower-settle", align_last_observation_,
              "waiting arm fully lowered");
  publishStop();
  if (elapsedSec(phase_tp_) < params_.pre_approach_lower_settle_s) {
    return BT::NodeStatus::RUNNING;
  }
  if (!pre_approach_observation_.has_value()) {
    return fail("第二预选赛机械臂放下后缺少前向趋近目标");
  }
  return beginOdomApproach(*pre_approach_observation_);
}

BT::NodeStatus SecondPreselectionKfsPickupAction::beginOdomApproach(
    const KfsObservation &observation) {
  publishStop();
  approach_distance_m_ =
      secondPreselectionKfsApproachDistance(observation.target.distance_m,
                                            params_);
  const double duration =
      params_.kfs_approach_speed_mps > 0.0
          ? std::abs(approach_distance_m_) / params_.kfs_approach_speed_mps
          : std::numeric_limits<double>::infinity();
  if (duration > params_.kfs_approach_timeout_s) {
    return fail("第二预选赛 KFS 前向趋近计划超过超时");
  }
  pickup_target_ = observation.target;
  has_pickup_target_ = true;
  approach_started_ = true;
  approach_start_captured_ = false;
  approach_stable_ticks_ = 0;
  approach_waiting_odom_logged_ = false;
  phase_ = Phase::OdomApproach;
  phase_tp_ = std::chrono::steady_clock::now();
  RCLCPP_INFO(node_->get_logger(),
              "第二预选赛 KFS 前向趋近启动：locked_depth=%.3fm distance=%.3fm",
              observation.target.distance_m, approach_distance_m_);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SecondPreselectionKfsPickupAction::tickOdomApproach() {
  renderKfsUi("approach", align_last_observation_, "odom forward approach");
  if (!approach_started_) {
    return fail("第二预选赛 KFS 前向趋近未启动");
  }
  if (elapsedSec(phase_tp_) > params_.kfs_approach_timeout_s) {
    return fail("第二预选赛 KFS 前向趋近超时");
  }
  if (!odomReady()) {
    publishStop();
    approach_stable_ticks_ = 0;
    if (!approach_waiting_odom_logged_) {
      RCLCPP_WARN(node_->get_logger(),
                  "第二预选赛 KFS 前向趋近等待 odom：topic=%s",
                  params_.odom_topic.c_str());
      approach_waiting_odom_logged_ = true;
    }
    return BT::NodeStatus::RUNNING;
  }
  if (!approach_start_captured_) {
    approach_start_x_ = odom_x_;
    approach_start_y_ = odom_y_;
    approach_start_yaw_ = odom_yaw_;
    approach_start_captured_ = true;
    approach_waiting_odom_logged_ = false;
  }

  const double c = std::cos(approach_start_yaw_);
  const double s = std::sin(approach_start_yaw_);
  const double dx = odom_x_ - approach_start_x_;
  const double dy = odom_y_ - approach_start_y_;
  const double progress = dx * c + dy * s;
  const double remaining = approach_distance_m_ - progress;
  const double yaw_error = normalizeAngle(approach_start_yaw_ - odom_yaw_);
  const double yaw_tol = std::abs(params_.kfs_odom_yaw_tolerance_deg) * kDeg2Rad;

  if (std::abs(remaining) <= params_.kfs_approach_odom_tolerance_m &&
      std::abs(yaw_error) <= yaw_tol) {
    ++approach_stable_ticks_;
    publishStop();
    if (approach_stable_ticks_ >= params_.kfs_odom_stable_ticks) {
      beginPickupCommand();
    }
    return BT::NodeStatus::RUNNING;
  }
  approach_stable_ticks_ = 0;
  if (params_.kfs_approach_speed_mps <= 0.0) {
    return fail("第二预选赛 KFS 前向趋近速度非正");
  }
  double vx = params_.kfs_odom_xy_kp * remaining;
  if (std::abs(vx) > params_.kfs_approach_speed_mps) {
    vx = std::copysign(params_.kfs_approach_speed_mps, vx);
  }
  if (std::abs(vx) < params_.kfs_approach_min_speed_mps &&
      std::abs(vx) > 1e-9) {
    vx = std::copysign(params_.kfs_approach_min_speed_mps, vx);
  }
  publishTwist(vx, 0.0, headingAngularZ(approach_start_yaw_));
  return BT::NodeStatus::RUNNING;
}

void SecondPreselectionKfsPickupAction::beginMechanismCommand(
    int command_id, int done_feedback_id, const std::string &label) {
  publishStop();
  command_generation_.fetch_add(1, std::memory_order_relaxed);
  command_response_seen_ = false;
  command_accepted_ = false;
  command_done_feedback_seen_ = false;
  command_error_seen_ = false;
  command_busy_seen_ = false;
  command_seq_ = -1;
  command_error_detail_.clear();
  active_command_id_ = clampByte(command_id);
  active_done_feedback_id_ = std::clamp(done_feedback_id, 0, 255);
  active_command_label_ = label.empty() ? "second_preselection_command" : label;
  phase_tp_ = std::chrono::steady_clock::now();
}

void SecondPreselectionKfsPickupAction::beginPickupCommand() {
  beginMechanismCommand(params_.pickup_command_id,
                        params_.pickup_done_feedback_id,
                        "SECOND_PRESELECTION_PICKUP_KFS");
  phase_ = Phase::SendingPickup;
  RCLCPP_INFO(node_->get_logger(),
              "第二预选赛准备发送 KFS 夹取触发命令：0x%02X，完成反馈=0x%02X",
              params_.pickup_command_id & 0xFF,
              params_.pickup_done_feedback_id & 0xFF);
}

BT::NodeStatus SecondPreselectionKfsPickupAction::tickSendingPickup() {
  renderKfsUi("pickup-send", align_last_observation_, "sending 0x12");
  publishStop();
  if (!sendActiveCommand()) {
    if (elapsedSec(phase_tp_) > params_.command_timeout_s) {
      return fail("第二预选赛等待 KFS 夹取命令服务超时");
    }
    return BT::NodeStatus::RUNNING;
  }
  phase_ = Phase::WaitingPickupAck;
  phase_tp_ = std::chrono::steady_clock::now();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SecondPreselectionKfsPickupAction::tickWaitingPickupAck() {
  renderKfsUi("pickup-ack", align_last_observation_, "waiting service ACK");
  publishStop();
  if (command_error_seen_.load(std::memory_order_relaxed)) {
    return fail(command_error_detail_.empty()
                    ? "第二预选赛 KFS 夹取命令收到 MCU 错误"
                    : command_error_detail_);
  }
  if (command_response_seen_.load(std::memory_order_relaxed)) {
    if (!command_accepted_.load(std::memory_order_relaxed)) {
      return fail("第二预选赛 KFS 夹取命令 ACK 被拒绝");
    }
    const int seq = command_seq_.load(std::memory_order_relaxed);
    RCLCPP_INFO(node_->get_logger(),
                "第二预选赛 KFS 夹取命令 ACK 成功：command=0x%02X seq=%d，等待完成反馈 0x%02X",
                active_command_id_, seq, active_done_feedback_id_ & 0xFF);
    phase_ = Phase::WaitingPickupDone;
    phase_tp_ = std::chrono::steady_clock::now();
    last_log_tp_ = phase_tp_;
    return BT::NodeStatus::RUNNING;
  }
  if (elapsedSec(phase_tp_) > params_.command_timeout_s) {
    return fail("第二预选赛等待 KFS 夹取命令 ACK 超时");
  }
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SecondPreselectionKfsPickupAction::tickWaitingPickupDone() {
  renderKfsUi("pickup-done", align_last_observation_, "waiting MCU 0x11 done");
  publishStop();
  const int seq = command_seq_.load(std::memory_order_relaxed);
  if (command_error_seen_.load(std::memory_order_relaxed)) {
    return fail(command_error_detail_.empty()
                    ? "第二预选赛 KFS 夹取命令收到 MCU 错误"
                    : command_error_detail_);
  }
  if (command_done_feedback_seen_.load(std::memory_order_relaxed)) {
    RCLCPP_INFO(node_->get_logger(),
                "第二预选赛 KFS 夹取完成反馈已收到：feedback=0x%02X seq=%d，开始视觉验证",
                active_done_feedback_id_ & 0xFF, seq);
    return beginGrabVerify();
  }
  if (elapsedSec(phase_tp_) > params_.done_timeout_s) {
    return fail("等待第二预选赛 KFS 夹取完成反馈超时：feedback=" +
                byteHex(params_.pickup_done_feedback_id) +
                " seq=" + std::to_string(seq));
  }
  if (elapsedSec(last_log_tp_) >= params_.log_period_s) {
    const bool busy_seen = command_busy_seen_.load(std::memory_order_relaxed);
    RCLCPP_INFO(node_->get_logger(),
                "第二预选赛等待 KFS 夹取完成反馈：feedback=0x%02X seq=%d elapsed=%.1fs busy=%s",
                active_done_feedback_id_ & 0xFF, seq,
                elapsedSec(phase_tp_), busy_seen ? "是" : "否");
    last_log_tp_ = std::chrono::steady_clock::now();
  }
  return BT::NodeStatus::RUNNING;
}

void SecondPreselectionKfsPickupAction::handleFeedback(
    const FeedbackMsg::SharedPtr msg) {
  if (!msg) {
    return;
  }
  const int seq = command_seq_.load(std::memory_order_relaxed);
  std::optional<MechanismErrorDiagnostic> diagnostic;
  if (isSameSeqMechanismError(*msg, seq, diagnostic) && diagnostic) {
    if (diagnostic->busy) {
      command_busy_seen_.store(true, std::memory_order_relaxed);
      return;
    }
    command_error_detail_ = mechanismErrorDiagnosticText(*diagnostic);
    command_error_seen_.store(true, std::memory_order_relaxed);
    return;
  }
  if (seq >= 0 && msg->seq == static_cast<uint8_t>(seq & 0xFF) &&
      msg->feedback_id ==
          static_cast<uint8_t>(active_done_feedback_id_ & 0xFF)) {
    command_done_feedback_seen_.store(true, std::memory_order_relaxed);
  }
}

bool SecondPreselectionKfsPickupAction::sendActiveCommand() {
  if (!send_client_ || !send_client_->service_is_ready()) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                         "第二预选赛等待机构命令服务：%s label=%s",
                         params_.send_command_service.c_str(),
                         active_command_label_.c_str());
    return false;
  }
  auto request = std::make_shared<SendCommandSrv::Request>();
  request->command_id = active_command_id_;
  request->payload = emptyPayload();
  request->wait_ack = true;
  const uint64_t token = command_generation_.load(std::memory_order_relaxed);
  try {
    send_client_->async_send_request(
        request, [this, token](rclcpp::Client<SendCommandSrv>::SharedFuture future) {
          if (token != command_generation_.load(std::memory_order_relaxed)) {
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
    command_error_detail_ = "第二预选赛机构命令发送异常：" +
                            active_command_label_ + " " + e.what();
    command_response_seen_.store(true, std::memory_order_relaxed);
    command_accepted_.store(false, std::memory_order_relaxed);
  }
  RCLCPP_INFO(node_->get_logger(), "第二预选赛已发送机构命令：%s command=%s",
              active_command_label_.c_str(), byteHex(active_command_id_).c_str());
  return true;
}

BT::NodeStatus SecondPreselectionKfsPickupAction::beginGrabVerify() {
  if (!has_pickup_target_) {
    return fail("第二预选赛 KFS 夹取验证目标缺失");
  }
  grab_verify_lost_count_ = 0;
  grab_verify_last_sequence_ = pickup_target_.sequence;
  grab_verify_seen_new_frame_ = false;
  grab_verify_visible_logged_ = false;
  grab_verify_last_logged_lost_count_ = 0;
  phase_ = Phase::GrabVerify;
  phase_tp_ = std::chrono::steady_clock::now();
  RCLCPP_INFO(node_->get_logger(),
              "第二预选赛开始 KFS 夹取视觉消失验证：target=%s seq=%ld timeout=%.1fs lost=%d",
              pickup_target_.label.c_str(),
              static_cast<long>(pickup_target_.sequence),
              params_.grab_verify_timeout_s,
              params_.grab_verify_lost_stable_frames);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SecondPreselectionKfsPickupAction::tickGrabVerify() {
  publishStop();
  if (!has_pickup_target_) {
    return fail("第二预选赛 KFS 夹取验证目标缺失");
  }
  const double elapsed = elapsedSec(phase_tp_);
  rc26_vision::VisionInferenceManager::FrameSnapshot snapshot;
  const bool has_snapshot =
      vision_ && vision_->isRunning() && vision_->getLatestFrameSnapshot(snapshot) &&
      snapshot.display_sequence > 0;
  if (!has_snapshot || snapshot.display_sequence <= grab_verify_last_sequence_) {
    renderKfsUi("verify", align_last_observation_, "waiting new frame");
    if (elapsed >= params_.grab_verify_timeout_s) {
      return fail("第二预选赛 KFS 夹取验证没有新视觉帧");
    }
    return BT::NodeStatus::RUNNING;
  }

  grab_verify_seen_new_frame_ = true;
  grab_verify_last_sequence_ = snapshot.display_sequence;
  bool still_visible = false;
  double best_iou = 0.0;
  for (const auto &det : snapshot.detections) {
    const auto candidate =
        rc26_vision::makeVisualTargetSnapshot(det, snapshot.display_sequence);
    if (candidate.label != pickup_target_.label) {
      continue;
    }
    const double iou = rc26_vision::bboxIou(pickup_target_, candidate);
    best_iou = std::max(best_iou, iou);
    if (iou >= params_.grab_verify_iou_threshold) {
      still_visible = true;
      break;
    }
  }
  std::ostringstream verify_detail;
  verify_detail << "visible=" << (still_visible ? "yes" : "no")
                << " lost=" << grab_verify_lost_count_ << "/"
                << params_.grab_verify_lost_stable_frames << " iou="
                << std::fixed << std::setprecision(3) << best_iou;
  renderKfsUi("verify", align_last_observation_, verify_detail.str());
  if (still_visible) {
    grab_verify_lost_count_ = 0;
    grab_verify_last_logged_lost_count_ = 0;
    if (!grab_verify_visible_logged_) {
      RCLCPP_INFO(node_->get_logger(),
                  "第二预选赛 KFS 夹取验证：原目标仍可见 iou=%.3f",
                  best_iou);
      grab_verify_visible_logged_ = true;
    }
  } else {
    ++grab_verify_lost_count_;
    grab_verify_visible_logged_ = false;
    if (grab_verify_lost_count_ != grab_verify_last_logged_lost_count_) {
      RCLCPP_INFO(node_->get_logger(),
                  "第二预选赛 KFS 夹取验证：原目标未匹配 stable=%d/%d best_iou=%.3f",
                  grab_verify_lost_count_,
                  params_.grab_verify_lost_stable_frames, best_iou);
      grab_verify_last_logged_lost_count_ = grab_verify_lost_count_;
    }
    if (grab_verify_lost_count_ >= params_.grab_verify_lost_stable_frames) {
      phase_ = Phase::Settle;
      phase_tp_ = std::chrono::steady_clock::now();
      RCLCPP_INFO(node_->get_logger(),
                  "第二预选赛 KFS 夹取确认成功，进入稳定等待 %.2fs",
                  params_.grab_settle_s);
      return BT::NodeStatus::RUNNING;
    }
  }
  if (elapsed >= params_.grab_verify_timeout_s) {
    if (!grab_verify_seen_new_frame_) {
      return fail("第二预选赛 KFS 夹取验证没有新视觉帧");
    }
    phase_ = Phase::Settle;
    phase_tp_ = std::chrono::steady_clock::now();
    RCLCPP_WARN(node_->get_logger(),
                "第二预选赛 KFS 夹取视觉验证超时兜底：原目标未稳定消失，继续后续流程 settle=%.2fs",
                params_.grab_settle_s);
    return BT::NodeStatus::RUNNING;
  }
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SecondPreselectionKfsPickupAction::tickSettle() {
  renderKfsUi("settle", align_last_observation_, "pickup verified");
  publishStop();
  if (elapsedSec(phase_tp_) >= params_.grab_settle_s) {
    clearRuntimeState();
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

void SecondPreselectionKfsPickupAction::clearRuntimeState() {
  publishStop();
  command_generation_.fetch_add(1, std::memory_order_relaxed);
  releaseUi();
  releaseVision();
  releaseOdom();
  releaseCommandIo();
  cmd_pub_.reset();
  has_odom_ = false;
  search_yaw_captured_ = false;
  search_origin_captured_ = false;
  search_origin_x_ = 0.0;
  search_origin_y_ = 0.0;
  align_yaw_captured_ = false;
  align_waiting_verify_frame_ = false;
  align_waiting_odom_logged_ = false;
  align_stable_count_ = 0;
  align_lost_count_ = 0;
  align_last_sequence_ = 0;
  align_target_lock_sequence_ = 0;
  align_lock_state_.reset();
  align_last_observation_.reset();
  align_filtered_offset_valid_ = false;
  align_filtered_offset_px_ = 0.0;
  has_pickup_target_ = false;
  pre_approach_observation_.reset();
  last_real_depth_m_ = 0.0;
  approach_distance_m_ = 0.0;
  approach_started_ = false;
  approach_start_captured_ = false;
  approach_stable_ticks_ = 0;
  approach_waiting_odom_logged_ = false;
  command_response_seen_ = false;
  command_accepted_ = false;
  command_done_feedback_seen_ = false;
  command_error_seen_ = false;
  command_busy_seen_ = false;
  command_seq_ = -1;
  command_error_detail_.clear();
  active_command_id_ = 0;
  active_done_feedback_id_ = -1;
  active_command_label_.clear();
  grab_verify_lost_count_ = 0;
  grab_verify_last_sequence_ = 0;
  grab_verify_seen_new_frame_ = false;
  grab_verify_visible_logged_ = false;
  grab_verify_last_logged_lost_count_ = 0;
}

SecondPreselectionRampForwardAction::SecondPreselectionRampForwardAction(
    const std::string &name, const BT::NodeConfig &config)
    : BT::StatefulActionNode(name, config) {}

SecondPreselectionRampForwardAction::~SecondPreselectionRampForwardAction() {
  clearRuntimeState();
}

BT::NodeStatus SecondPreselectionRampForwardAction::onStart() {
  if (!config().blackboard || !config().blackboard->get("node", node_) ||
      !node_) {
    writeDecisionFailure(config().blackboard, "SecondPreselectionRampForward",
                         "运行上下文缺失：blackboard 或 node 不可用");
    return BT::NodeStatus::FAILURE;
  }
  if (!config().blackboard->get("second_preselection_params", params_)) {
    writeDecisionFailure(config().blackboard, "SecondPreselectionRampForward",
                         "黑板缺少 second_preselection_params");
    return BT::NodeStatus::FAILURE;
  }

  rclcpp::Node *runtime_node = node_;
  clearRuntimeState();
  node_ = runtime_node;
  start_tp_ = std::chrono::steady_clock::now();
  odom_sub_ = node_->create_subscription<OdomMsg>(
      params_.odom_topic, rclcpp::QoS(rclcpp::KeepLast(10)),
      [this](const OdomMsg::SharedPtr msg) {
        if (!msg) {
          return;
        }
        odom_x_ = msg->pose.pose.position.x;
        odom_y_ = msg->pose.pose.position.y;
        const auto &q = msg->pose.pose.orientation;
        odom_yaw_ = yawFromQuaternion(q.x, q.y, q.z, q.w);
        has_odom_ = true;
        last_odom_tp_ = std::chrono::steady_clock::now();
      });
  cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
      params_.cmd_vel_topic, rclcpp::QoS(10));
  if (!odom_sub_ || !cmd_pub_) {
    writeDecisionFailure(config().blackboard, "SecondPreselectionRampForward",
                         "第二预选赛合并 ramp odom/cmd_vel 初始化失败");
    publishStop();
    clearRuntimeState();
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(node_->get_logger(),
              "第二预选赛合并 ramp 前进启动：target=%.3fm timeout=%.2fs speed=[%.3f, %.3f]m/s odom=%s cmd_vel=%s",
              params_.ramp_forward_x_m, params_.ramp_forward_timeout_s,
              params_.ramp_min_speed_mps, params_.ramp_max_speed_mps,
              params_.odom_topic.c_str(), params_.cmd_vel_topic.c_str());
  publishStop();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SecondPreselectionRampForwardAction::onRunning() {
  if (secondPreselectionRampTimedOut(elapsedSec(start_tp_),
                                     params_.ramp_forward_timeout_s)) {
    return finishSuccess("TIMEOUT_CONTINUE");
  }
  if (!odomReady()) {
    publishStop();
    return BT::NodeStatus::RUNNING;
  }
  if (!start_captured_) {
    start_x_ = odom_x_;
    start_y_ = odom_y_;
    start_yaw_ = odom_yaw_;
    start_captured_ = true;
    RCLCPP_INFO(node_->get_logger(),
                "第二预选赛合并 ramp 捕获起点：start=[%.3f %.3f %.3f]",
                start_x_, start_y_, start_yaw_);
  }

  const double progress = secondPreselectionProjectedX(
      start_x_, start_y_, start_yaw_, odom_x_, odom_y_);
  const double remaining =
      secondPreselectionRampRemainingToDrive(progress, params_);
  const double tolerance = std::max(0.0, params_.kfs_approach_odom_tolerance_m);
  if (progress >= params_.ramp_forward_x_m || remaining <= tolerance) {
    return finishSuccess(progress >= params_.ramp_forward_x_m
                             ? "TARGET_REACHED_OR_OVERSHOT"
                             : "TARGET_TOLERANCE");
  }

  double vx = params_.kfs_odom_xy_kp * remaining;
  vx = std::clamp(vx, 0.0, params_.ramp_max_speed_mps);
  if (vx > 1e-9 && vx < params_.ramp_min_speed_mps) {
    vx = params_.ramp_min_speed_mps;
  }
  publishTwist(vx, headingAngularZ());
  return BT::NodeStatus::RUNNING;
}

void SecondPreselectionRampForwardAction::onHalted() {
  publishStop();
  clearRuntimeState();
}

void SecondPreselectionRampForwardAction::publishStop() {
  if (cmd_pub_) {
    cmd_pub_->publish(geometry_msgs::msg::Twist{});
  }
}

void SecondPreselectionRampForwardAction::clearRuntimeState() {
  publishStop();
  odom_sub_.reset();
  cmd_pub_.reset();
  node_ = nullptr;
  has_odom_ = false;
  start_captured_ = false;
}

bool SecondPreselectionRampForwardAction::odomReady() const {
  return has_odom_ && elapsedSec(last_odom_tp_) <= params_.odom_timeout_s;
}

void SecondPreselectionRampForwardAction::publishTwist(double vx, double wz) {
  if (!cmd_pub_) {
    return;
  }
  geometry_msgs::msg::Twist twist;
  twist.linear.x = std::max(0.0, vx);
  twist.angular.z = wz;
  cmd_pub_->publish(twist);
}

double SecondPreselectionRampForwardAction::headingAngularZ() const {
  if (!start_captured_) {
    return 0.0;
  }
  double wz = params_.kfs_heading_kp * normalizeAngle(start_yaw_ - odom_yaw_);
  return std::clamp(wz, -params_.kfs_heading_max_speed_radps,
                    params_.kfs_heading_max_speed_radps);
}

BT::NodeStatus
SecondPreselectionRampForwardAction::finishSuccess(const std::string &reason) {
  publishStop();
  if (node_) {
    if (reason == "TIMEOUT_CONTINUE") {
      RCLCPP_WARN(node_->get_logger(),
                  "第二预选赛合并 ramp 超时，停车后继续后续逻辑：timeout=%.2fs",
                  params_.ramp_forward_timeout_s);
    } else {
      RCLCPP_INFO(node_->get_logger(),
                  "第二预选赛合并 ramp 完成，停车后继续：reason=%s",
                  reason.c_str());
    }
  }
  clearRuntimeState();
  return BT::NodeStatus::SUCCESS;
}

SecondPreselectionDriveToTotalXAction::SecondPreselectionDriveToTotalXAction(
    const std::string &name, const BT::NodeConfig &config)
    : BT::StatefulActionNode(name, config) {}

SecondPreselectionDriveToTotalXAction::~SecondPreselectionDriveToTotalXAction() {
  clearRuntimeState();
}

BT::NodeStatus SecondPreselectionDriveToTotalXAction::onStart() {
  if (!config().blackboard || !config().blackboard->get("node", node_) ||
      !node_) {
    writeDecisionFailure(config().blackboard, "SecondPreselectionDriveToTotalX",
                         "运行上下文缺失：blackboard 或 node 不可用");
    return BT::NodeStatus::FAILURE;
  }
  if (!config().blackboard->get("second_preselection_params", params_)) {
    return fail("黑板缺少 second_preselection_params");
  }
  bool origin_valid = false;
  if (!config().blackboard->get("second_preselect_total_x_origin_valid",
                                origin_valid) ||
      !origin_valid ||
      !config().blackboard->get("second_preselect_total_x_origin_x",
                                origin_x_) ||
      !config().blackboard->get("second_preselect_total_x_origin_y",
                                origin_y_) ||
      !config().blackboard->get("second_preselect_total_x_origin_yaw",
                                origin_yaw_)) {
    return fail("第二预选赛总 X 搜索原点缺失");
  }

  start_tp_ = std::chrono::steady_clock::now();
  odom_sub_ = node_->create_subscription<OdomMsg>(
      params_.odom_topic, rclcpp::QoS(rclcpp::KeepLast(10)),
      [this](const OdomMsg::SharedPtr msg) {
        if (!msg) {
          return;
        }
        odom_x_ = msg->pose.pose.position.x;
        odom_y_ = msg->pose.pose.position.y;
        const auto &q = msg->pose.pose.orientation;
        odom_yaw_ = yawFromQuaternion(q.x, q.y, q.z, q.w);
        has_odom_ = true;
        last_odom_tp_ = std::chrono::steady_clock::now();
      });
  cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
      params_.cmd_vel_topic, rclcpp::QoS(10));
  if (!odom_sub_ || !cmd_pub_) {
    return fail("第二预选赛总 X odom/cmd_vel 初始化失败");
  }
  stable_ticks_ = 0;
  RCLCPP_INFO(node_->get_logger(),
              "第二预选赛开始补齐总 X：target=%.3fm tolerance=%.3fm origin=[%.3f %.3f %.3f]",
              params_.total_x_target_m, params_.total_x_tolerance_m,
              origin_x_, origin_y_, origin_yaw_);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SecondPreselectionDriveToTotalXAction::onRunning() {
  if (elapsedSec(start_tp_) > params_.nav_timeout_s) {
    return fail("第二预选赛补齐总 X 超时");
  }
  if (!odomReady()) {
    publishStop();
    stable_ticks_ = 0;
    return BT::NodeStatus::RUNNING;
  }

  const double progress = secondPreselectionProjectedX(
      origin_x_, origin_y_, origin_yaw_, odom_x_, odom_y_);
  const double remaining = params_.total_x_target_m - progress;
  config().blackboard->set("second_preselect_total_x_progress_m", progress);
  config().blackboard->set("second_preselect_total_x_remaining_m", remaining);

  if (remaining < -params_.total_x_tolerance_m) {
    RCLCPP_WARN(node_->get_logger(),
                "第二预选赛总 X 已超调，不倒退补偿：progress=%.3fm target=%.3fm overshoot=%.3fm",
                progress, params_.total_x_target_m, -remaining);
    publishStop();
    clearRuntimeState();
    return BT::NodeStatus::SUCCESS;
  }

  const double yaw_error = normalizeAngle(origin_yaw_ - odom_yaw_);
  const double yaw_tolerance =
      std::abs(params_.kfs_odom_yaw_tolerance_deg) * kDeg2Rad;
  if (std::abs(remaining) <= params_.total_x_tolerance_m &&
      std::abs(yaw_error) <= yaw_tolerance) {
    ++stable_ticks_;
    publishStop();
    if (stable_ticks_ >= params_.kfs_odom_stable_ticks) {
      RCLCPP_INFO(node_->get_logger(),
                  "第二预选赛总 X 已到位：progress=%.3fm target=%.3fm",
                  progress, params_.total_x_target_m);
      clearRuntimeState();
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  }

  stable_ticks_ = 0;
  double vx = params_.kfs_odom_xy_kp *
              secondPreselectionTotalXRemainingToDrive(progress, params_);
  vx = std::min(vx, params_.nav_max_speed_mps);
  if (vx > 1e-9 && vx < params_.kfs_approach_min_speed_mps) {
    vx = params_.kfs_approach_min_speed_mps;
  }
  double wz = params_.kfs_heading_kp * yaw_error;
  wz = std::clamp(wz, -params_.kfs_heading_max_speed_radps,
                  params_.kfs_heading_max_speed_radps);
  geometry_msgs::msg::Twist twist;
  twist.linear.x = vx;
  twist.angular.z = wz;
  cmd_pub_->publish(twist);
  return BT::NodeStatus::RUNNING;
}

void SecondPreselectionDriveToTotalXAction::onHalted() {
  publishStop();
  clearRuntimeState();
}

BT::NodeStatus
SecondPreselectionDriveToTotalXAction::fail(const std::string &reason) {
  writeDecisionFailure(config().blackboard, "SecondPreselectionDriveToTotalX",
                       reason);
  if (node_) {
    RCLCPP_ERROR(node_->get_logger(), "第二预选赛总 X 失败：%s",
                 reason.c_str());
  }
  publishStop();
  clearRuntimeState();
  return BT::NodeStatus::FAILURE;
}

void SecondPreselectionDriveToTotalXAction::publishStop() {
  if (cmd_pub_) {
    cmd_pub_->publish(geometry_msgs::msg::Twist{});
  }
}

void SecondPreselectionDriveToTotalXAction::clearRuntimeState() {
  publishStop();
  odom_sub_.reset();
  cmd_pub_.reset();
  has_odom_ = false;
  stable_ticks_ = 0;
}

bool SecondPreselectionDriveToTotalXAction::odomReady() const {
  return has_odom_ && elapsedSec(last_odom_tp_) <= params_.odom_timeout_s;
}

SecondPreselectionKfsPlacePrepareAction::
    SecondPreselectionKfsPlacePrepareAction(const std::string &name,
                                            const BT::NodeConfig &config)
    : BT::StatefulActionNode(name, config) {}

SecondPreselectionKfsPlacePrepareAction::
    ~SecondPreselectionKfsPlacePrepareAction() {
  clearRuntimeState();
}

BT::NodeStatus SecondPreselectionKfsPlacePrepareAction::onStart() {
  if (!config().blackboard || !config().blackboard->get("node", node_) ||
      !node_) {
    writeDecisionFailure(config().blackboard,
                         "SecondPreselectionKfsPlacePrepare",
                         "运行上下文缺失：blackboard 或 node 不可用");
    return BT::NodeStatus::FAILURE;
  }
  if (!config().blackboard->get("second_preselection_params", params_)) {
    return fail("黑板缺少 second_preselection_params");
  }

  clearRuntimeState();
  config().blackboard->set("second_preselect_place_immediate", false);
  config().blackboard->set("second_preselect_place_avoidance_stage", 0);
  config().blackboard->set("second_preselect_place_target_depth_m", 0.0);
  config().blackboard->set("second_preselect_place_approach_distance_m", 0.0);
  config().blackboard->set("second_preselect_place_mode",
                           std::string("OCCUPANCY_CHECK"));
  config().blackboard->set("second_preselect_place_termination_reason",
                           std::string());
  config().blackboard->set("second_preselect_place_prepare_result",
                           std::string("PREPARING"));
  start_tp_ = std::chrono::steady_clock::now();
  phase_tp_ = start_tp_;
  last_log_tp_ = start_tp_;
  if (!setupOdom()) {
    return fail("第二预选赛放置准备 odom 启动失败");
  }
  if (!setupVision()) {
    return fail("第二预选赛放置准备视觉启动失败");
  }
  cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
      params_.cmd_vel_topic, rclcpp::QoS(10));
  if (!cmd_pub_) {
    return fail("第二预选赛放置准备 cmd_vel publisher 创建失败");
  }
  phase_ = Phase::ObserveInitial;
  latchLatestObservationSequence();
  RCLCPP_INFO(node_->get_logger(),
              "第二预选赛开始 KFS 放置准备：middle_y=%.2f~%.2f lower_y=%.2f~%.2f fixed_x=%.2fm timeout=%.1fs observe_timeout=%.1fs occupied_stable=%d first_y=%.2f second_y=%.2f mirror=%d",
              params_.place_occupied_middle_y_min_ratio,
              params_.place_occupied_middle_y_max_ratio,
              params_.place_occupied_lower_y_min_ratio,
              params_.place_occupied_lower_y_max_ratio,
              params_.place_fixed_forward_x_m,
              params_.place_fixed_forward_timeout_s,
              params_.place_observe_timeout_s,
              params_.place_occupied_stable_frames,
              params_.place_occupied_first_lateral_m,
              params_.place_occupied_second_reverse_m,
              params_.team_mirror_sign);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SecondPreselectionKfsPlacePrepareAction::onRunning() {
  if (!vision_ || !vision_->isRunning()) {
    return fail("第二预选赛放置准备视觉运行时不可用");
  }
  switch (phase_) {
  case Phase::ObserveInitial:
    return tickObserveCheckpoint(Phase::LateralFirst, Phase::AlignTarget, false);
  case Phase::LateralFirst:
  case Phase::LateralSecond:
    return tickLateralMove();
  case Phase::ObserveAfterFirst:
    return tickObserveCheckpoint(Phase::LateralSecond, Phase::AlignTarget,
                                 false);
  case Phase::ObserveAfterSecond:
    return tickObserveCheckpoint(Phase::AlignTarget, Phase::AlignTarget, true);
  case Phase::AlignTarget:
    return tickAlignTarget();
  }
  return fail("第二预选赛放置准备未知状态");
}

void SecondPreselectionKfsPlacePrepareAction::onHalted() {
  publishStop();
  clearRuntimeState();
}

BT::NodeStatus
SecondPreselectionKfsPlacePrepareAction::fail(const std::string &reason) {
  writeDecisionFailure(config().blackboard,
                       "SecondPreselectionKfsPlacePrepare", reason);
  if (node_) {
    RCLCPP_ERROR(node_->get_logger(), "第二预选赛放置准备失败：%s",
                 reason.c_str());
  }
  publishStop();
  clearRuntimeState();
  return BT::NodeStatus::FAILURE;
}

bool SecondPreselectionKfsPlacePrepareAction::setupVision() {
  try {
    params_.vision_config_file = resolveVisionConfig(params_.vision_config_file);
    auto config =
        rc26_vision::ProfileLoader::loadFromYaml(params_.vision_config_file);
    rc26_vision::ProfileLoader::validate(config);
    if (config.profiles.find(params_.model_id) == config.profiles.end()) {
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
    RCLCPP_ERROR(node_->get_logger(), "第二预选赛放置准备视觉初始化异常：%s",
                 e.what());
    vision_.reset();
    return false;
  }
}

void SecondPreselectionKfsPlacePrepareAction::releaseVision() {
  releaseUi();
  if (vision_) {
    vision_->stop();
    vision_.reset();
  }
}

bool SecondPreselectionKfsPlacePrepareAction::setupOdom() {
  odom_sub_ = node_->create_subscription<OdomMsg>(
      params_.odom_topic, rclcpp::QoS(rclcpp::KeepLast(10)),
      [this](const OdomMsg::SharedPtr msg) {
        if (!msg) {
          return;
        }
        odom_x_ = msg->pose.pose.position.x;
        odom_y_ = msg->pose.pose.position.y;
        const auto &q = msg->pose.pose.orientation;
        odom_yaw_ = yawFromQuaternion(q.x, q.y, q.z, q.w);
        has_odom_ = true;
        last_odom_tp_ = std::chrono::steady_clock::now();
      });
  return static_cast<bool>(odom_sub_);
}

void SecondPreselectionKfsPlacePrepareAction::releaseOdom() {
  odom_sub_.reset();
}

bool SecondPreselectionKfsPlacePrepareAction::odomReady() const {
  return has_odom_ && elapsedSec(last_odom_tp_) <= params_.odom_timeout_s;
}

bool SecondPreselectionKfsPlacePrepareAction::setupUiIfNeeded() {
  return setupDebugWindowIfNeeded(node_, params_, ui_window_active_,
                                  ui_disabled_after_error_,
                                  "第二预选赛 KFS 放置准备");
}

void SecondPreselectionKfsPlacePrepareAction::releaseUi() {
  releaseDebugWindow(node_, params_, ui_window_active_,
                     "第二预选赛 KFS 放置准备");
}

void SecondPreselectionKfsPlacePrepareAction::renderUi(
    const std::string &stage, const std::optional<KfsObservation> &observation,
    const std::string &detail) {
  if (!params_.dynamic_roi_ui_enable || ui_disabled_after_error_ || !vision_ ||
      !vision_->isRunning()) {
    return;
  }
  FrameSnapshot snapshot;
  if (!latestSnapshot(snapshot) || !setupUiIfNeeded()) {
    return;
  }
  try {
    cv::Mat canvas = snapshot.color_bgr.clone();
    const cv::Scalar text_bg(20, 20, 20);
    std::optional<rc26_vision::VisualTargetSnapshot> locked_target;
    if (observation) {
      locked_target = observation->target;
    }
    drawDetectionsOverlay(canvas, snapshot.detections, text_bg, locked_target);
    const int target_line_x =
        std::max(0, canvas.cols / 2) + params_.kfs_align_target_line_offset_px;
    cv::line(canvas, cv::Point(target_line_x, 0),
             cv::Point(target_line_x, std::max(0, canvas.rows - 1)),
             cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
    std::ostringstream status;
    status << "stage=" << stage << " occupied=" << occupied_stable_count_
           << " clear=" << clear_stable_count_ << " align="
           << align_stable_count_ << "/" << params_.kfs_align_stable_frames;
    if (observation) {
      status << " target=" << observation->target.label << " depth="
             << std::fixed << std::setprecision(3)
             << observation->target.distance_m << " offset="
             << observation->offset_px;
    }
    drawTextWithBackground(canvas, status.str(), cv::Point(8, 20),
                           cv::Scalar(255, 255, 255), text_bg, 0.50, 1);
    if (!detail.empty()) {
      drawTextWithBackground(canvas, detail, cv::Point(8, 42),
                             cv::Scalar(255, 255, 255), text_bg, 0.50, 1);
    }
    cv::imshow(params_.dynamic_roi_ui_window_name, canvas);
    cv::waitKey(1);
  } catch (const cv::Exception &e) {
    ui_disabled_after_error_ = true;
    RCLCPP_WARN(node_->get_logger(),
                "第二预选赛 KFS 放置准备 UI 失败，自动关闭：%s", e.what());
    releaseUi();
  }
}

void SecondPreselectionKfsPlacePrepareAction::publishStop() {
  if (cmd_pub_) {
    cmd_pub_->publish(geometry_msgs::msg::Twist{});
  }
}

void SecondPreselectionKfsPlacePrepareAction::publishTwist(double vx, double vy,
                                                           double wz) {
  if (!cmd_pub_) {
    return;
  }
  geometry_msgs::msg::Twist twist;
  twist.linear.x = vx;
  twist.linear.y = vy;
  twist.angular.z = wz;
  cmd_pub_->publish(twist);
}

rc26_vision::TipAlignmentConfig
SecondPreselectionKfsPlacePrepareAction::makeAlignmentConfig() const {
  rc26_vision::TipAlignmentConfig config;
  config.target_lock_enable = true;
  config.target_lock_max_jump_px = params_.kfs_align_max_jump_px;
  config.lost_stop_frames = params_.kfs_lost_stop_frames;
  config.tolerance_px = params_.kfs_align_tolerance_px;
  config.target_line_offset_px = params_.kfs_align_target_line_offset_px;
  config.kp = params_.kfs_align_kp;
  config.min_speed_mps = params_.kfs_align_min_speed_mps;
  config.max_speed_mps = params_.kfs_align_max_speed_mps;
  config.invert_direction = params_.kfs_invert_lateral_direction;
  config.heading_hold_enable = true;
  config.target_yaw_rad = align_yaw_;
  config.heading_kp = params_.kfs_heading_kp;
  config.heading_max_speed_radps = params_.kfs_heading_max_speed_radps;
  config.heading_tolerance_rad =
      std::abs(params_.kfs_odom_yaw_tolerance_deg) * kDeg2Rad;
  config.heading_gate_rad =
      std::max(config.heading_tolerance_rad,
               std::abs(params_.kfs_align_heading_gate_deg) * kDeg2Rad);
  return config;
}

std::optional<rc26_vision::TipHeadingControl>
SecondPreselectionKfsPlacePrepareAction::alignHeadingControl() {
  if (!align_yaw_captured_) {
    if (!odomReady()) {
      waiting_odom_logged_ = true;
      return std::nullopt;
    }
    align_yaw_ = odom_yaw_;
    align_yaw_captured_ = true;
    waiting_odom_logged_ = false;
  }
  if (!odomReady()) {
    waiting_odom_logged_ = true;
    return std::nullopt;
  }
  waiting_odom_logged_ = false;
  return rc26_vision::computeTipHeadingControl(odom_yaw_, makeAlignmentConfig());
}

bool SecondPreselectionKfsPlacePrepareAction::latestSnapshot(
    FrameSnapshot &snapshot) const {
  return vision_ && vision_->isRunning() &&
         vision_->getLatestFrameSnapshot(snapshot) && snapshot.has_color &&
         !snapshot.color_bgr.empty() && snapshot.display_sequence > 0;
}

std::optional<SecondPreselectionKfsPlacePrepareAction::KfsObservation>
SecondPreselectionKfsPlacePrepareAction::findNearestKfs(
    const FrameSnapshot &snapshot) {
  std::optional<KfsObservation> best;
  const int camera_center_x = std::max(0, snapshot.color_bgr.cols / 2);
  const int target_line_x =
      camera_center_x + params_.kfs_align_target_line_offset_px;
  const double min_x = static_cast<double>(snapshot.color_bgr.cols) *
                       params_.place_occupied_center_x_min_ratio;
  const double max_x = static_cast<double>(snapshot.color_bgr.cols) *
                       params_.place_occupied_center_x_max_ratio;
  const double lower_min_y = static_cast<double>(snapshot.color_bgr.rows) *
                             params_.place_occupied_lower_y_min_ratio;
  const double lower_max_y = static_cast<double>(snapshot.color_bgr.rows) *
                             params_.place_occupied_lower_y_max_ratio;
  for (const auto &det : snapshot.detections) {
    const std::string label = rc26_vision::visualTargetLabel(det);
    const bool is_r2 =
        labelsMatch(label, params_.r2_target_labels,
                    params_.r2_target_label_prefixes);
    const bool is_r1 =
        labelsMatch(label, params_.r1_blocking_labels,
                    params_.r1_blocking_label_prefixes);
    if (!(is_r2 || is_r1 || isKfsLabel(label))) {
      continue;
    }
    if (label == "R1_KFS" && det.score < params_.r1_kfs_min_score) {
      continue;
    }
    const auto center = detectionCenter(det);
    if (center.x < min_x || center.x > max_x || center.y < lower_min_y ||
        center.y > lower_max_y) {
      continue;
    }
    KfsObservation observation;
    observation.target =
        rc26_vision::makeVisualTargetSnapshot(det, snapshot.display_sequence);
    observation.target.distance_m = params_.place_fixed_forward_x_m;
    observation.detection = det;
    observation.offset_px =
        static_cast<int>(std::lround(center.x - target_line_x));
    observation.depth_detail = "lower_layer_visual";
    if (!best) {
      best = observation;
      continue;
    }
    const int offset =
        static_cast<int>(std::lround(center.x - camera_center_x));
    const int best_offset = static_cast<int>(std::lround(
        detectionCenter(best->detection).x - camera_center_x));
    if (std::abs(offset) < std::abs(best_offset) ||
        (std::abs(offset) == std::abs(best_offset) &&
         observation.target.score > best->target.score)) {
      best = observation;
    }
  }

  if (!best) {
    return std::nullopt;
  }
  std::vector<rc26_vision::Detection> selected{best->detection};
  std::vector<int> class_ids{best->detection.class_id};
  const auto selection = rc26_vision::updateTipAlignmentTarget(
      selected, snapshot.color_bgr.cols, class_ids, align_lock_state_,
      makeAlignmentConfig());
  if (!selection.has_target || selection.target.source_index != 0) {
    return std::nullopt;
  }
  best->offset_px = selection.offset_px;
  return best;
}

SecondPreselectionKfsPlacePrepareAction::KfsObservation
SecondPreselectionKfsPlacePrepareAction::applyAlignmentObservationFilter(
    const KfsObservation &observation) {
  KfsObservation filtered = observation;
  const double alpha =
      std::clamp(params_.kfs_align_offset_filter_alpha, 0.05, 1.0);
  if (!align_filtered_offset_valid_) {
    align_filtered_offset_px_ = observation.offset_px;
    align_filtered_offset_valid_ = true;
  } else {
    align_filtered_offset_px_ =
        alpha * observation.offset_px +
        (1.0 - alpha) * align_filtered_offset_px_;
  }
  filtered.offset_px = static_cast<int>(std::lround(align_filtered_offset_px_));
  return filtered;
}

BT::NodeStatus SecondPreselectionKfsPlacePrepareAction::tickObserveCheckpoint(
    Phase lateral_phase, Phase clear_phase, bool final_checkpoint) {
  if (secondPreselectionPlaceObserveTimedOut(
          elapsedSec(phase_tp_), params_.place_observe_timeout_s)) {
    return finishFixedForwardPlace("PLACE_OBSERVE_TIMEOUT_FIXED_FORWARD");
  }
  FrameSnapshot snapshot;
  if (!latestSnapshot(snapshot)) {
    publishStop();
    return BT::NodeStatus::RUNNING;
  }
  if (snapshot.display_sequence == occupancy_last_sequence_) {
    publishStop();
    return BT::NodeStatus::RUNNING;
  }
  occupancy_last_sequence_ = snapshot.display_sequence;
  const auto layers = secondPreselectionFrameLayers(
      snapshot.detections, snapshot.color_bgr.cols, snapshot.color_bgr.rows,
      params_);
  const bool occupied = layers.middle;
  const auto occupancy_decision = secondPreselectionUpdateOccupancyStability(
      occupied, params_.place_occupied_stable_frames, occupied_stable_count_,
      clear_stable_count_);
  renderUi("layer-check", std::nullopt,
           layers.middle ? (layers.lower ? "middle+lower KFS"
                                          : "middle KFS occupied")
                         : (layers.lower ? "lower KFS align target"
                                         : "front clear"));
  publishStop();

  if (occupancy_decision == SecondPreselectionOccupancyDecision::Clear) {
    resetObservationStability();
    resetAlignmentState();
    if (layers.lower) {
      phase_ = clear_phase;
      phase_tp_ = std::chrono::steady_clock::now();
      config().blackboard->set("second_preselect_place_mode",
                               std::string("ALIGNING_LOWER_KFS"));
      return BT::NodeStatus::RUNNING;
    }
    return finishFixedForwardPlace("CLEAR_NO_LOWER_FIXED_FORWARD");
  }
  if (occupancy_decision == SecondPreselectionOccupancyDecision::Pending) {
    return BT::NodeStatus::RUNNING;
  }
  if (final_checkpoint) {
    if (layers.lower) {
      phase_ = clear_phase;
      phase_tp_ = std::chrono::steady_clock::now();
      resetObservationStability();
      resetAlignmentState();
      config().blackboard->set("second_preselect_place_mode",
                               std::string("ALIGNING_LOWER_KFS"));
      return BT::NodeStatus::RUNNING;
    }
    return finishFixedForwardPlace("MIDDLE_ONLY_AFTER_SECOND_SHIFT");
  }

  const double distance = secondPreselectionPlaceAvoidanceDistance(
      lateral_phase == Phase::LateralFirst ? 1 : 2, params_);
  const Phase next_observe = lateral_phase == Phase::LateralFirst
                                 ? Phase::ObserveAfterFirst
                                 : Phase::ObserveAfterSecond;
  beginLateralMove(distance, lateral_phase, next_observe);
  return BT::NodeStatus::RUNNING;
}

void SecondPreselectionKfsPlacePrepareAction::beginLateralMove(
    double distance_m, Phase phase, Phase next_observe_phase) {
  lateral_distance_m_ = distance_m;
  lateral_start_captured_ = false;
  lateral_stable_ticks_ = 0;
  next_observe_phase_ = next_observe_phase;
  phase_ = phase;
  phase_tp_ = std::chrono::steady_clock::now();
  config().blackboard->set("second_preselect_place_avoidance_stage",
                           phase == Phase::LateralFirst ? 1 : 2);
  config().blackboard->set("second_preselect_place_mode",
                           std::string("OCCUPANCY_AVOIDANCE"));
  resetObservationStability();
  RCLCPP_INFO(node_->get_logger(),
              "第二预选赛占位避让横移启动：distance=%.3fm phase=%d",
              lateral_distance_m_, static_cast<int>(phase_));
}

BT::NodeStatus SecondPreselectionKfsPlacePrepareAction::tickLateralMove() {
  if (elapsedSec(phase_tp_) > params_.place_occupied_lateral_timeout_s) {
    return fail("第二预选赛占位避让横移超时");
  }
  if (!odomReady()) {
    publishStop();
    lateral_stable_ticks_ = 0;
    return BT::NodeStatus::RUNNING;
  }
  if (!lateral_start_captured_) {
    lateral_start_x_ = odom_x_;
    lateral_start_y_ = odom_y_;
    lateral_start_yaw_ = odom_yaw_;
    if (!align_yaw_captured_) {
      align_yaw_ = odom_yaw_;
      align_yaw_captured_ = true;
    }
    lateral_start_captured_ = true;
  }
  const double dx = odom_x_ - lateral_start_x_;
  const double dy = odom_y_ - lateral_start_y_;
  const double progress =
      -dx * std::sin(lateral_start_yaw_) + dy * std::cos(lateral_start_yaw_);
  const double remaining = lateral_distance_m_ - progress;
  const double yaw_error = normalizeAngle(align_yaw_ - odom_yaw_);
  const double yaw_tolerance =
      std::abs(params_.kfs_odom_yaw_tolerance_deg) * kDeg2Rad;
  if (std::abs(remaining) <= params_.kfs_approach_odom_tolerance_m &&
      std::abs(yaw_error) <= yaw_tolerance) {
    ++lateral_stable_ticks_;
    publishStop();
    if (lateral_stable_ticks_ >= params_.kfs_odom_stable_ticks) {
      phase_ = next_observe_phase_;
      phase_tp_ = std::chrono::steady_clock::now();
      resetObservationStability();
      latchLatestObservationSequence();
      return BT::NodeStatus::RUNNING;
    }
    return BT::NodeStatus::RUNNING;
  }
  lateral_stable_ticks_ = 0;
  double vy = params_.kfs_odom_xy_kp * remaining;
  vy = std::clamp(vy, -params_.place_occupied_lateral_max_speed_mps,
                  params_.place_occupied_lateral_max_speed_mps);
  if (std::abs(vy) > 1e-9 &&
      std::abs(vy) < params_.place_occupied_lateral_min_speed_mps) {
    vy = std::copysign(params_.place_occupied_lateral_min_speed_mps, vy);
  }
  publishTwist(0.0, vy, headingAngularZ(align_yaw_));
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SecondPreselectionKfsPlacePrepareAction::tickAlignTarget() {
  FrameSnapshot snapshot;
  if (!latestSnapshot(snapshot)) {
    publishStop();
    align_stable_count_ = 0;
    return BT::NodeStatus::RUNNING;
  }
  const bool new_visual_frame = secondPreselectionConsumeNewFrameSequence(
      snapshot.display_sequence, align_search_last_sequence_);
  auto observation = findNearestKfs(snapshot);
  if (!observation) {
    ++align_lost_count_;
    align_stable_count_ = 0;
    renderUi("place-align-search", align_last_observation_,
             "waiting lower-layer KFS");
    publishStop();
    if (!new_visual_frame) {
      return BT::NodeStatus::RUNNING;
    }
    config().blackboard->set("second_preselect_place_mode",
                             std::string("WAITING_LOWER_KFS"));
    if (elapsedSec(phase_tp_) > params_.kfs_align_timeout_s) {
      return finishFixedForwardPlace("LOWER_KFS_ALIGN_TARGET_TIMEOUT");
    }
    return BT::NodeStatus::RUNNING;
  }
  if (elapsedSec(phase_tp_) > params_.kfs_align_timeout_s) {
    return finishFixedForwardPlace("LOWER_KFS_ALIGN_TIMEOUT");
  }
  config().blackboard->set("second_preselect_place_mode",
                           std::string("ALIGNING_LOWER_KFS"));
  align_lost_count_ = 0;
  observation = applyAlignmentObservationFilter(*observation);
  align_last_observation_ = observation;
  renderUi("place-align", observation, "lower KFS pixel/yaw alignment");
  const auto heading = alignHeadingControl();
  if (!heading) {
    publishStop();
    align_stable_count_ = 0;
    return BT::NodeStatus::RUNNING;
  }
  const bool new_frame = observation->target.sequence != align_last_sequence_;
  if (new_frame) {
    align_last_sequence_ = observation->target.sequence;
  }
  const bool pixel_aligned =
      std::abs(observation->offset_px) <= params_.kfs_align_tolerance_px;
  if (pixel_aligned && heading->aligned) {
    publishStop();
    if (new_frame) {
      ++align_stable_count_;
    }
    if (align_stable_count_ >= params_.kfs_align_stable_frames) {
      return finishAlignedPlace(*observation);
    }
    return BT::NodeStatus::RUNNING;
  }
  if (new_frame) {
    align_stable_count_ = 0;
  }
  const double vy = heading->allow_lateral
                        ? rc26_vision::computeTipAlignmentVy(
                              observation->offset_px, makeAlignmentConfig())
                        : 0.0;
  publishTwist(0.0, vy, heading->angular_z_radps);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SecondPreselectionKfsPlacePrepareAction::finishAlignedPlace(
    const KfsObservation &observation) {
  config().blackboard->set("second_preselect_place_immediate", false);
  config().blackboard->set("second_preselect_place_target_label",
                           observation.target.label);
  config().blackboard->set("second_preselect_place_target_depth_m", 0.0);
  config().blackboard->set("second_preselect_place_approach_distance_m",
                           params_.place_fixed_forward_x_m);
  config().blackboard->set("second_preselect_place_prepare_result",
                           std::string("ALIGNED_LOWER_FIXED_FORWARD"));
  config().blackboard->set("second_preselect_place_mode",
                           std::string("ALIGNED_LOWER_FIXED_FORWARD"));
  config().blackboard->set("second_preselect_place_termination_reason",
                           std::string("ALIGNED_LOWER_FIXED_FORWARD"));
  RCLCPP_INFO(node_->get_logger(),
              "第二预选赛下层 KFS 放置对齐完成：label=%s fixed_forward=%.3fm",
              observation.target.label.c_str(), params_.place_fixed_forward_x_m);
  publishStop();
  clearRuntimeState();
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus SecondPreselectionKfsPlacePrepareAction::finishFixedForwardPlace(
    const std::string &reason) {
  config().blackboard->set("second_preselect_place_immediate", false);
  config().blackboard->set("second_preselect_place_target_depth_m", 0.0);
  config().blackboard->set("second_preselect_place_approach_distance_m",
                           params_.place_fixed_forward_x_m);
  config().blackboard->set("second_preselect_place_prepare_result", reason);
  config().blackboard->set("second_preselect_place_mode",
                           std::string("FIXED_FORWARD_PLACE"));
  config().blackboard->set("second_preselect_place_termination_reason", reason);
  RCLCPP_INFO(node_->get_logger(),
              "第二预选赛放置准备进入固定前进：reason=%s fixed_forward=%.3fm",
              reason.c_str(), params_.place_fixed_forward_x_m);
  publishStop();
  clearRuntimeState();
  return BT::NodeStatus::SUCCESS;
}

void SecondPreselectionKfsPlacePrepareAction::resetObservationStability() {
  occupied_stable_count_ = 0;
  clear_stable_count_ = 0;
  occupancy_last_sequence_ = 0;
}

void SecondPreselectionKfsPlacePrepareAction::latchLatestObservationSequence() {
  FrameSnapshot snapshot;
  if (latestSnapshot(snapshot)) {
    occupancy_last_sequence_ = snapshot.display_sequence;
  }
}

void SecondPreselectionKfsPlacePrepareAction::resetAlignmentState() {
  align_stable_count_ = 0;
  align_lost_count_ = 0;
  align_last_sequence_ = 0;
  align_search_last_sequence_ = 0;
  align_lock_state_.reset();
  align_last_observation_.reset();
  align_filtered_offset_valid_ = false;
  align_filtered_offset_px_ = 0.0;
}

double SecondPreselectionKfsPlacePrepareAction::headingAngularZ(
    double target_yaw_rad) const {
  const double error = normalizeAngle(target_yaw_rad - odom_yaw_);
  return std::clamp(params_.kfs_heading_kp * error,
                    -params_.kfs_heading_max_speed_radps,
                    params_.kfs_heading_max_speed_radps);
}

void SecondPreselectionKfsPlacePrepareAction::clearRuntimeState() {
  publishStop();
  releaseUi();
  releaseVision();
  releaseOdom();
  cmd_pub_.reset();
  has_odom_ = false;
  align_yaw_captured_ = false;
  align_yaw_ = 0.0;
  waiting_odom_logged_ = false;
  resetObservationStability();
  resetAlignmentState();
  lateral_start_captured_ = false;
  lateral_stable_ticks_ = 0;
  phase_ = Phase::ObserveInitial;
}

SecondPreselectionPlaceApproachAction::
    SecondPreselectionPlaceApproachAction(const std::string &name,
                                          const BT::NodeConfig &config)
    : BT::StatefulActionNode(name, config) {}

SecondPreselectionPlaceApproachAction::
    ~SecondPreselectionPlaceApproachAction() {
  clearRuntimeState();
}

BT::NodeStatus SecondPreselectionPlaceApproachAction::onStart() {
  if (!config().blackboard || !config().blackboard->get("node", node_) ||
      !node_) {
    writeDecisionFailure(config().blackboard,
                         "SecondPreselectionPlaceApproach",
                         "运行上下文缺失：blackboard 或 node 不可用");
    return BT::NodeStatus::FAILURE;
  }
  if (!config().blackboard->get("second_preselection_params", params_)) {
    return fail("黑板缺少 second_preselection_params");
  }
  bool immediate = false;
  (void)config().blackboard->get("second_preselect_place_immediate", immediate);
  if (immediate) {
    config().blackboard->set("second_preselect_place_approach_result",
                             std::string("SKIPPED_IMMEDIATE_PLACE"));
    config().blackboard->set("second_preselect_place_approach_timed_out",
                             false);
    return BT::NodeStatus::SUCCESS;
  }
  if (!config().blackboard->get("second_preselect_place_approach_distance_m",
                                distance_m_)) {
    return fail("黑板缺少第二预选赛放置趋近距离");
  }
  if (!std::isfinite(distance_m_) || distance_m_ < 0.0) {
    return fail("第二预选赛放置趋近距离非法");
  }
  if (distance_m_ <= params_.kfs_approach_odom_tolerance_m) {
    config().blackboard->set("second_preselect_place_approach_result",
                             std::string("NO_MOTION_REQUIRED"));
    config().blackboard->set("second_preselect_place_approach_timed_out",
                             false);
    return BT::NodeStatus::SUCCESS;
  }

  start_tp_ = std::chrono::steady_clock::now();
  odom_sub_ = node_->create_subscription<OdomMsg>(
      params_.odom_topic, rclcpp::QoS(rclcpp::KeepLast(10)),
      [this](const OdomMsg::SharedPtr msg) {
        if (!msg) {
          return;
        }
        odom_x_ = msg->pose.pose.position.x;
        odom_y_ = msg->pose.pose.position.y;
        const auto &q = msg->pose.pose.orientation;
        odom_yaw_ = yawFromQuaternion(q.x, q.y, q.z, q.w);
        has_odom_ = true;
        last_odom_tp_ = std::chrono::steady_clock::now();
      });
  cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
      params_.cmd_vel_topic, rclcpp::QoS(10));
  if (!odom_sub_ || !cmd_pub_) {
    return fail("第二预选赛放置趋近 odom/cmd_vel 初始化失败");
  }
  stable_ticks_ = 0;
  start_captured_ = false;
  RCLCPP_INFO(node_->get_logger(),
              "第二预选赛放置趋近启动：distance=%.3fm timeout=%.1fs",
              distance_m_, params_.place_fixed_forward_timeout_s);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SecondPreselectionPlaceApproachAction::onRunning() {
  if (secondPreselectionPlaceApproachTimedOut(
          elapsedSec(start_tp_), params_.place_fixed_forward_timeout_s)) {
    return finish("TIMED_OUT_CONTINUE_PLACE", true);
  }
  if (!odomReady()) {
    publishStop();
    stable_ticks_ = 0;
    return BT::NodeStatus::RUNNING;
  }
  if (!start_captured_) {
    start_x_ = odom_x_;
    start_y_ = odom_y_;
    start_yaw_ = odom_yaw_;
    start_captured_ = true;
  }
  const double progress = secondPreselectionProjectedX(
      start_x_, start_y_, start_yaw_, odom_x_, odom_y_);
  const double remaining = distance_m_ - progress;
  const double yaw_error = normalizeAngle(start_yaw_ - odom_yaw_);
  const double yaw_tolerance =
      std::abs(params_.kfs_odom_yaw_tolerance_deg) * kDeg2Rad;
  if (std::abs(remaining) <= params_.kfs_approach_odom_tolerance_m &&
      std::abs(yaw_error) <= yaw_tolerance) {
    ++stable_ticks_;
    publishStop();
    if (stable_ticks_ >= params_.kfs_odom_stable_ticks) {
      return finish("SUCCEEDED", false);
    }
    return BT::NodeStatus::RUNNING;
  }
  stable_ticks_ = 0;
  double vx = params_.kfs_odom_xy_kp * remaining;
  vx = std::clamp(vx, -params_.kfs_approach_speed_mps,
                  params_.kfs_approach_speed_mps);
  if (std::abs(vx) > 1e-9 &&
      std::abs(vx) < params_.kfs_approach_min_speed_mps) {
    vx = std::copysign(params_.kfs_approach_min_speed_mps, vx);
  }
  double wz = params_.kfs_heading_kp * yaw_error;
  wz = std::clamp(wz, -params_.kfs_heading_max_speed_radps,
                  params_.kfs_heading_max_speed_radps);
  geometry_msgs::msg::Twist twist;
  twist.linear.x = vx;
  twist.angular.z = wz;
  cmd_pub_->publish(twist);
  return BT::NodeStatus::RUNNING;
}

void SecondPreselectionPlaceApproachAction::onHalted() {
  publishStop();
  clearRuntimeState();
}

BT::NodeStatus
SecondPreselectionPlaceApproachAction::fail(const std::string &reason) {
  writeDecisionFailure(config().blackboard,
                       "SecondPreselectionPlaceApproach", reason);
  if (node_) {
    RCLCPP_ERROR(node_->get_logger(), "第二预选赛放置趋近失败：%s",
                 reason.c_str());
  }
  publishStop();
  clearRuntimeState();
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus SecondPreselectionPlaceApproachAction::finish(
    const std::string &result, bool timed_out) {
  publishStop();
  config().blackboard->set("second_preselect_place_approach_result", result);
  config().blackboard->set("second_preselect_place_approach_timed_out",
                           timed_out);
  if (timed_out) {
    RCLCPP_WARN(node_->get_logger(),
                "第二预选赛放置趋近超时，停车后继续下发 0x13：distance=%.3fm",
                distance_m_);
  } else {
    RCLCPP_INFO(node_->get_logger(),
                "第二预选赛放置趋近完成：distance=%.3fm", distance_m_);
  }
  clearRuntimeState();
  return BT::NodeStatus::SUCCESS;
}

void SecondPreselectionPlaceApproachAction::publishStop() {
  if (cmd_pub_) {
    cmd_pub_->publish(geometry_msgs::msg::Twist{});
  }
}

void SecondPreselectionPlaceApproachAction::clearRuntimeState() {
  publishStop();
  odom_sub_.reset();
  cmd_pub_.reset();
  has_odom_ = false;
  start_captured_ = false;
  stable_ticks_ = 0;
}

bool SecondPreselectionPlaceApproachAction::odomReady() const {
  return has_odom_ && elapsedSec(last_odom_tp_) <= params_.odom_timeout_s;
}

SecondPreselectionClimbFrontStageAction::
    SecondPreselectionClimbFrontStageAction(const std::string &name,
                                            const BT::NodeConfig &config)
    : StairActionBase(name, config) {}

BT::NodeStatus SecondPreselectionClimbFrontStageAction::onStart() {
  if (!setupRuntime("第二预选赛独立上阶前段")) {
    return BT::NodeStatus::FAILURE;
  }
  if (!config().blackboard ||
      !config().blackboard->get("second_preselection_params",
                                second_params_)) {
    return fail("黑板缺少 second_preselection_params");
  }

  clearManualFeedbackRuntime();
  const std::string feedback_topic =
      second_params_.feedback_topic.empty() ? params_.feedback_topic
                                            : second_params_.feedback_topic;
  manual_feedback_sub_ = node_->create_subscription<FeedbackMsg>(
      feedback_topic, rclcpp::QoS(32).reliable(),
      [this](const FeedbackMsg::SharedPtr msg) {
        if (!msg) {
          return;
        }
        if (msg->feedback_id ==
            static_cast<uint8_t>(
                second_params_.climb_place_manual_front_laser_feedback_id &
                0xFF)) {
          manual_front_laser_count_.fetch_add(1, std::memory_order_relaxed);
        }
      });
  if (!manual_feedback_sub_) {
    return fail("人工前激光反馈订阅创建失败");
  }

  publishStop();
  phase_ = Phase::HeadingAlign;
  beginHeadingAlignment();
  RCLCPP_INFO(
      node_->get_logger(),
      "第二预选赛独立上阶前段启动: front_extend=%s manual_front=%s front_retract=%s rear_extend=%s",
      byteHex(second_params_.climb_place_front_pushrod_extend_command_id)
          .c_str(),
      byteHex(second_params_.climb_place_manual_front_laser_feedback_id)
          .c_str(),
      byteHex(second_params_.climb_place_front_pushrod_retract_command_id)
          .c_str(),
      byteHex(second_params_.climb_place_rear_pushrod_extend_command_id)
          .c_str());
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SecondPreselectionClimbFrontStageAction::onRunning() {
  switch (phase_) {
  case Phase::HeadingAlign:
    switch (tickHeadingAlignment()) {
    case StepStatus::Success:
      phase_ = Phase::SendFrontExtend;
      beginCommand(
          static_cast<CommandID>(clampByte(
              second_params_.climb_place_front_pushrod_extend_command_id)),
          "FRONT_PUSHROD_EXTEND");
      break;
    case StepStatus::Failure:
      return fail("上阶前 yaw 对齐失败");
    case StepStatus::Running:
      break;
    }
    break;

  case Phase::SendFrontExtend:
    switch (tickCommand()) {
    case StepStatus::Success:
      phase_ = Phase::HoldAfterFrontExtend;
      beginZeroHold(params_.climb_front_extend_delay_s,
                    "climb_place_front_extend_settle");
      break;
    case StepStatus::Failure:
      return fail("FRONT_PUSHROD_EXTEND 命令失败");
    case StepStatus::Running:
      break;
    }
    break;

  case Phase::HoldAfterFrontExtend:
    switch (tickZeroHold()) {
    case StepStatus::Success:
      beginManualFrontLaserDrive();
      break;
    case StepStatus::Failure:
      return fail("前推杆伸出后零速等待失败");
    case StepStatus::Running:
      break;
    }
    break;

  case Phase::DriveUntilManualFrontLaser:
    if (tickDriveYawGate("climb_place_manual_front")) {
      manual_event_tp_ = std::chrono::steady_clock::now();
      break;
    }
    publishProfiledDrive(1.0);
    if (manual_front_laser_count_.load(std::memory_order_relaxed) >
        manual_front_laser_baseline_) {
      publishStop();
      phase_ = Phase::SendFrontRetractAndRearExtend;
      beginCommandPair(
          static_cast<CommandID>(clampByte(
              second_params_.climb_place_front_pushrod_retract_command_id)),
          "FRONT_PUSHROD_RETRACT",
          static_cast<CommandID>(clampByte(
              second_params_.climb_place_rear_pushrod_extend_command_id)),
          "REAR_PUSHROD_EXTEND");
      switch (tickCommandPair()) {
      case StepStatus::Success:
        phase_ = Phase::HoldAfterFrontRetractAndRearExtend;
        beginZeroHold(params_.climb_retract_rear_extend_delay_s,
                      "climb_place_front_retract_rear_extend_settle");
        break;
      case StepStatus::Failure:
        return fail("FRONT_PUSHROD_RETRACT + REAR_PUSHROD_EXTEND 并发命令失败");
      case StepStatus::Running:
        break;
      }
      break;
    }
    if (elapsedSec(manual_event_tp_) > params_.front_event_timeout_s) {
      return fail("等待第二预选赛人工前激光 0x15 超时");
    }
    break;

  case Phase::SendFrontRetractAndRearExtend:
    publishStop();
    switch (tickCommandPair()) {
    case StepStatus::Success:
      phase_ = Phase::HoldAfterFrontRetractAndRearExtend;
      beginZeroHold(params_.climb_retract_rear_extend_delay_s,
                    "climb_place_front_retract_rear_extend_settle");
      break;
    case StepStatus::Failure:
      return fail("FRONT_PUSHROD_RETRACT + REAR_PUSHROD_EXTEND 并发命令失败");
    case StepStatus::Running:
      break;
    }
    break;

  case Phase::HoldAfterFrontRetractAndRearExtend:
    switch (tickZeroHold()) {
    case StepStatus::Success:
      publishStop();
      clearManualFeedbackRuntime();
      phase_ = Phase::Done;
      releaseRuntime();
      return BT::NodeStatus::SUCCESS;
    case StepStatus::Failure:
      return fail("前推杆收回和后推杆伸出后零速等待失败");
    case StepStatus::Running:
      break;
    }
    break;

  case Phase::Done:
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

void SecondPreselectionClimbFrontStageAction::onHalted() {
  publishStop();
  clearManualFeedbackRuntime();
  releaseRuntime();
  phase_ = Phase::Done;
}

BT::NodeStatus
SecondPreselectionClimbFrontStageAction::fail(const char *reason) {
  clearManualFeedbackRuntime();
  phase_ = Phase::Done;
  return failWithStop(reason);
}

void SecondPreselectionClimbFrontStageAction::beginManualFrontLaserDrive() {
  manual_front_laser_baseline_ =
      manual_front_laser_count_.load(std::memory_order_relaxed);
  manual_event_tp_ = std::chrono::steady_clock::now();
  phase_ = Phase::DriveUntilManualFrontLaser;
  beginDriveProfile(params_.climb_front_drive_profile,
                    "climb_place_manual_front");
  RCLCPP_INFO(
      node_->get_logger(),
      "第二预选赛独立上阶前段开始行驶并等待人工前激光: feedback=%s timeout=%.1fs",
      byteHex(second_params_.climb_place_manual_front_laser_feedback_id)
          .c_str(),
      params_.front_event_timeout_s);
}

void SecondPreselectionClimbFrontStageAction::clearManualFeedbackRuntime() {
  manual_feedback_sub_.reset();
  manual_front_laser_count_ = 0;
  manual_front_laser_baseline_ = 0;
}

SecondPreselectionRearRetractPickupPlaceAction::
    SecondPreselectionRearRetractPickupPlaceAction(
        const std::string &name, const BT::NodeConfig &config)
    : BT::StatefulActionNode(name, config) {}

BT::NodeStatus SecondPreselectionRearRetractPickupPlaceAction::onStart() {
  if (!config().blackboard || !config().blackboard->get("node", node_) ||
      !node_) {
    writeDecisionFailure(config().blackboard,
                         "SecondPreselectionRearRetractPickupPlace",
                         "运行上下文缺失：blackboard 或 node 不可用");
    return BT::NodeStatus::FAILURE;
  }
  if (!config().blackboard->get("second_preselection_params", params_)) {
    writeDecisionFailure(config().blackboard,
                         "SecondPreselectionRearRetractPickupPlace",
                         "黑板缺少 second_preselection_params");
    return BT::NodeStatus::FAILURE;
  }

  clearRuntimeState();
  cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
      params_.cmd_vel_topic, rclcpp::QoS(10));
  send_client_ =
      node_->create_client<SendCommandSrv>(params_.send_command_service);
  feedback_sub_ = node_->create_subscription<FeedbackMsg>(
      params_.feedback_topic, rclcpp::QoS(32).reliable(),
      [this](const FeedbackMsg::SharedPtr msg) { handleFeedback(msg); });
  if (!cmd_pub_ || !send_client_ || !feedback_sub_) {
    return fail("第二预选赛独立收尾 ROS 资源创建失败");
  }

  command_generation_.fetch_add(1, std::memory_order_relaxed);
  resetCommand(rear_retract_command_,
               params_.climb_place_rear_pushrod_retract_command_id, -1,
               "REAR_PUSHROD_RETRACT");
  resetCommand(pickup_command_, params_.climb_place_preload_pickup_command_id,
               params_.climb_place_preload_pickup_done_feedback_id,
               "SECOND_PRESELECTION_PRELOAD_KFS_PICKUP");
  resetCommand(final_place_command_, params_.climb_place_final_command_id, -1,
               "SECOND_PRESELECTION_FINAL_PLACE_KFS");
  phase_ = Phase::SendRearRetractAndPickup;
  phase_tp_ = std::chrono::steady_clock::now();
  last_log_tp_ = phase_tp_;
  publishStop();
  RCLCPP_INFO(
      node_->get_logger(),
      "第二预选赛独立收尾启动: rear_retract=%s pickup=%s/done=%s delay=%.1fs final=%s",
      byteHex(params_.climb_place_rear_pushrod_retract_command_id).c_str(),
      byteHex(params_.climb_place_preload_pickup_command_id).c_str(),
      byteHex(params_.climb_place_preload_pickup_done_feedback_id).c_str(),
      params_.climb_place_final_delay_s,
      byteHex(params_.climb_place_final_command_id).c_str());
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus
SecondPreselectionRearRetractPickupPlaceAction::onRunning() {
  if (!node_) {
    return BT::NodeStatus::FAILURE;
  }
  if (command_error_seen_.load(std::memory_order_relaxed)) {
    return fail(command_error_detail_.empty()
                    ? "第二预选赛独立收尾收到机构错误"
                    : command_error_detail_);
  }

  switch (phase_) {
  case Phase::SendRearRetractAndPickup:
    publishStop();
    if (!rear_retract_command_.sent) {
      (void)sendCommand(rear_retract_command_);
    }
    if (!pickup_command_.sent) {
      (void)sendCommand(pickup_command_);
    }
    if (rear_retract_command_.sent && pickup_command_.sent) {
      phase_ = Phase::WaitRearRetractAndPickupAck;
      phase_tp_ = std::chrono::steady_clock::now();
    } else if (phaseElapsed() > params_.command_timeout_s) {
      return fail("等待后推杆收回和预装 KFS 夹取服务可用超时");
    }
    return BT::NodeStatus::RUNNING;

  case Phase::WaitRearRetractAndPickupAck:
    publishStop();
    if (commandRejected(rear_retract_command_) ||
        commandRejected(pickup_command_)) {
      return fail("后推杆收回或预装 KFS 夹取命令被拒绝");
    }
    if (commandAcked(rear_retract_command_) &&
        commandAcked(pickup_command_)) {
      final_gate_tp_ = std::chrono::steady_clock::now();
      phase_ = Phase::WaitDelayAndPickupDone;
      phase_tp_ = final_gate_tp_;
      last_log_tp_ = final_gate_tp_;
      RCLCPP_INFO(node_->get_logger(),
                  "第二预选赛独立收尾两条命令均 ACK，开始 %.1fs 计时并等待同 seq %s",
                  params_.climb_place_final_delay_s,
                  byteHex(params_.climb_place_preload_pickup_done_feedback_id)
                      .c_str());
      return BT::NodeStatus::RUNNING;
    }
    if (phaseElapsed() > params_.command_timeout_s) {
      return fail("等待后推杆收回和预装 KFS 夹取 ACK 超时");
    }
    return BT::NodeStatus::RUNNING;

  case Phase::WaitDelayAndPickupDone: {
    publishStop();
    const double gate_elapsed = elapsedSec(final_gate_tp_);
    if (secondPreselectionClimbPlaceReadyForFinal(
            gate_elapsed, params_.climb_place_final_delay_s,
            commandDone(pickup_command_))) {
      phase_ = Phase::SendFinalPlace;
      phase_tp_ = std::chrono::steady_clock::now();
      return BT::NodeStatus::RUNNING;
    }
    const double done_wait_timeout =
        std::max(params_.done_timeout_s,
                 params_.climb_place_final_delay_s +
                     params_.command_timeout_s);
    if (!commandDone(pickup_command_) && gate_elapsed > done_wait_timeout) {
      return fail("等待预装 KFS 夹取完成反馈超时");
    }
    if (elapsedSec(last_log_tp_) >= params_.log_period_s) {
      RCLCPP_INFO(
          node_->get_logger(),
          "第二预选赛独立收尾等待: delay=%.1f/%.1fs pickup_done=%s busy=%s",
          gate_elapsed, params_.climb_place_final_delay_s,
          commandDone(pickup_command_) ? "是" : "否",
          command_busy_seen_.load(std::memory_order_relaxed) ? "是" : "否");
      last_log_tp_ = std::chrono::steady_clock::now();
    }
    return BT::NodeStatus::RUNNING;
  }

  case Phase::SendFinalPlace:
    publishStop();
    if (!final_place_command_.sent) {
      (void)sendCommand(final_place_command_);
    }
    if (final_place_command_.sent) {
      phase_ = Phase::WaitFinalPlaceAck;
      phase_tp_ = std::chrono::steady_clock::now();
    } else if (phaseElapsed() > params_.command_timeout_s) {
      return fail("等待最终 0x13 服务可用超时");
    }
    return BT::NodeStatus::RUNNING;

  case Phase::WaitFinalPlaceAck:
    publishStop();
    if (commandRejected(final_place_command_)) {
      return fail("最终 0x13 命令被拒绝");
    }
    if (commandAcked(final_place_command_)) {
      phase_ = Phase::Done;
      clearRuntimeState();
      return BT::NodeStatus::SUCCESS;
    }
    if (phaseElapsed() > params_.command_timeout_s) {
      return fail("等待最终 0x13 ACK 超时");
    }
    return BT::NodeStatus::RUNNING;

  case Phase::Done:
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

void SecondPreselectionRearRetractPickupPlaceAction::onHalted() {
  publishStop();
  clearRuntimeState();
}

BT::NodeStatus SecondPreselectionRearRetractPickupPlaceAction::fail(
    const std::string &reason) {
  writeDecisionFailure(config().blackboard,
                       "SecondPreselectionRearRetractPickupPlace", reason);
  if (node_) {
    RCLCPP_WARN(node_->get_logger(), "第二预选赛独立收尾失败: %s",
                reason.c_str());
  }
  publishStop();
  clearRuntimeState();
  return BT::NodeStatus::FAILURE;
}

void SecondPreselectionRearRetractPickupPlaceAction::clearRuntimeState() {
  publishStop();
  command_generation_.fetch_add(1, std::memory_order_relaxed);
  cmd_pub_.reset();
  send_client_.reset();
  feedback_sub_.reset();
  resetCommand(rear_retract_command_, 0, -1, "");
  resetCommand(pickup_command_, 0, -1, "");
  resetCommand(final_place_command_, 0, -1, "");
  command_error_seen_ = false;
  command_busy_seen_ = false;
  command_error_detail_.clear();
  phase_ = Phase::Done;
}

void SecondPreselectionRearRetractPickupPlaceAction::resetCommand(
    CommandRuntime &command, int command_id, int done_feedback_id,
    const std::string &label) {
  command.command_id = clampByte(command_id);
  command.done_feedback_id = done_feedback_id;
  command.label = label;
  command.sent = false;
  command.response_seen = false;
  command.accepted = false;
  command.rejected = false;
  command.done_seen = done_feedback_id < 0;
  command.seq = -1;
}

bool SecondPreselectionRearRetractPickupPlaceAction::sendCommand(
    CommandRuntime &command) {
  if (!send_client_ || !send_client_->service_is_ready()) {
    if (node_) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "第二预选赛独立收尾等待机构命令服务");
    }
    return false;
  }

  auto request = std::make_shared<SendCommandSrv::Request>();
  request->command_id = command.command_id;
  request->payload = emptyPayload();
  request->wait_ack = true;
  const uint64_t token = command_generation_.load(std::memory_order_relaxed);
  CommandRuntime *slot = &command;
  try {
    send_client_->async_send_request(
        request,
        [this, token, slot](
            rclcpp::Client<SendCommandSrv>::SharedFuture future) {
          if (token != command_generation_.load(std::memory_order_relaxed)) {
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
          slot->seq.store(seq, std::memory_order_relaxed);
          slot->accepted.store(accepted, std::memory_order_relaxed);
          slot->rejected.store(!accepted, std::memory_order_relaxed);
          slot->response_seen.store(true, std::memory_order_relaxed);
        });
  } catch (const std::exception &e) {
    command_error_detail_ =
        std::string("第二预选赛独立收尾机构命令发送异常: ") + e.what();
    command_error_seen_.store(true, std::memory_order_relaxed);
    return true;
  }
  command.sent = true;
  if (node_) {
    RCLCPP_INFO(
        node_->get_logger(),
        "第二预选赛独立收尾已下发机构命令: %s command=%s done=%s",
        command.label.c_str(), byteHex(command.command_id).c_str(),
        command.done_feedback_id >= 0
            ? byteHex(command.done_feedback_id).c_str()
            : "ACK-only");
  }
  return true;
}

bool SecondPreselectionRearRetractPickupPlaceAction::commandAcked(
    const CommandRuntime &command) const {
  return command.response_seen.load(std::memory_order_relaxed) &&
         command.accepted.load(std::memory_order_relaxed);
}

bool SecondPreselectionRearRetractPickupPlaceAction::commandRejected(
    const CommandRuntime &command) const {
  return command.response_seen.load(std::memory_order_relaxed) &&
         command.rejected.load(std::memory_order_relaxed);
}

bool SecondPreselectionRearRetractPickupPlaceAction::commandDone(
    const CommandRuntime &command) const {
  return command.done_feedback_id < 0 ||
         command.done_seen.load(std::memory_order_relaxed);
}

void SecondPreselectionRearRetractPickupPlaceAction::handleFeedback(
    const FeedbackMsg::SharedPtr msg) {
  if (!msg) {
    return;
  }
  std::optional<MechanismErrorDiagnostic> diagnostic;
  for (auto *command : {&rear_retract_command_, &pickup_command_,
                        &final_place_command_}) {
    const int seq = command->seq.load(std::memory_order_relaxed);
    if (isSameSeqMechanismError(*msg, seq, diagnostic)) {
      const std::string detail = mechanismErrorDiagnosticText(*diagnostic);
      if (diagnostic->busy) {
        command_busy_seen_.store(true, std::memory_order_relaxed);
      } else {
        command_error_detail_ = detail;
        command_error_seen_.store(true, std::memory_order_relaxed);
      }
      return;
    }
    if (seq >= 0 && command->done_feedback_id >= 0 &&
        msg->seq == static_cast<uint8_t>(seq & 0xFF) &&
        msg->feedback_id ==
            static_cast<uint8_t>(command->done_feedback_id & 0xFF)) {
      command->done_seen.store(true, std::memory_order_relaxed);
      return;
    }
  }
}

void SecondPreselectionRearRetractPickupPlaceAction::publishStop() {
  if (cmd_pub_) {
    cmd_pub_->publish(geometry_msgs::msg::Twist{});
  }
}

double SecondPreselectionRearRetractPickupPlaceAction::phaseElapsed() const {
  return elapsedSec(phase_tp_);
}

SecondPreselectionPostPlaceClimbAction::SecondPreselectionPostPlaceClimbAction(
    const std::string &name, const BT::NodeConfig &config)
    : BT::StatefulActionNode(name, config) {}

BT::NodeStatus SecondPreselectionPostPlaceClimbAction::onStart() {
  if (!config().blackboard || !config().blackboard->get("node", node_) ||
      !node_) {
    writeDecisionFailure(config().blackboard,
                         "SecondPreselectionPostPlaceClimb",
                         "运行上下文缺失：blackboard 或 node 不可用");
    return BT::NodeStatus::FAILURE;
  }
  if (!config().blackboard->get("second_preselection_params", params_)) {
    writeDecisionFailure(config().blackboard,
                         "SecondPreselectionPostPlaceClimb",
                         "黑板缺少 second_preselection_params");
    return BT::NodeStatus::FAILURE;
  }
  if (!config().blackboard->get("stair_params", stair_params_)) {
    writeDecisionFailure(config().blackboard,
                         "SecondPreselectionPostPlaceClimb",
                         "黑板缺少 stair_params");
    return BT::NodeStatus::FAILURE;
  }
  normalizeStairParams(stair_params_);

  clearRuntimeState();
  cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
      params_.cmd_vel_topic.empty() ? stair_params_.cmd_vel_topic
                                    : params_.cmd_vel_topic,
      rclcpp::QoS(10));
  send_client_ = node_->create_client<SendCommandSrv>(
      params_.send_command_service.empty() ? stair_params_.send_command_service
                                           : params_.send_command_service);
  feedback_sub_ = node_->create_subscription<FeedbackMsg>(
      params_.feedback_topic.empty() ? stair_params_.feedback_topic
                                     : params_.feedback_topic,
      rclcpp::QoS(32).reliable(),
      [this](const FeedbackMsg::SharedPtr msg) { handleFeedback(msg); });
  if (stair_params_.heading_hold_enable && !stair_params_.odom_topic.empty()) {
    odom_sub_ = node_->create_subscription<OdomMsg>(
        stair_params_.odom_topic, rclcpp::QoS(rclcpp::KeepLast(10)),
        [this](const OdomMsg::SharedPtr msg) {
          current_yaw_rad_ = yawFromQuaternion(
              msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
              msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
          has_odom_ = true;
          last_odom_tp_ = std::chrono::steady_clock::now();
          if (!target_yaw_set_) {
            target_yaw_rad_ = current_yaw_rad_;
            target_yaw_set_ = true;
          }
        });
  } else {
    target_yaw_set_ = true;
  }

  command_generation_.fetch_add(1, std::memory_order_relaxed);
  resetCommand(command_a_, params_.post_place_front_pushrod_extend_command_id,
               -1, "FRONT_PUSHROD_EXTEND");
  resetCommand(command_b_, params_.post_place_preload_pickup_command_id,
               params_.post_place_preload_pickup_done_feedback_id,
               "SECOND_PRESELECTION_PRELOAD_KFS_PICKUP");
  phase_ = Phase::SendFrontExtendAndPreloadPickup;
  phase_tp_ = std::chrono::steady_clock::now();
  last_log_tp_ = phase_tp_;
  RCLCPP_INFO(
      node_->get_logger(),
      "第二预选赛放置后流程启动: front_extend=%s settle=%.2fs preload=%s/done=%s manual_laser=%s rear_event=%s final_delay=%.2fs final=%s",
      byteHex(params_.post_place_front_pushrod_extend_command_id).c_str(),
      params_.post_place_front_pushrod_extend_settle_s,
      byteHex(params_.post_place_preload_pickup_command_id).c_str(),
      byteHex(params_.post_place_preload_pickup_done_feedback_id).c_str(),
      byteHex(params_.post_place_manual_front_laser_feedback_id).c_str(),
      byteHex(params_.post_place_rear_laser_feedback_id).c_str(),
      params_.post_place_final_delay_s,
      byteHex(params_.post_place_final_command_id).c_str());
  publishStop();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SecondPreselectionPostPlaceClimbAction::onRunning() {
  if (!node_) {
    return BT::NodeStatus::FAILURE;
  }
  if (command_error_seen_.load(std::memory_order_relaxed)) {
    return fail(command_error_detail_.empty()
                    ? "第二预选赛放置后流程收到机构错误"
                    : command_error_detail_);
  }

  switch (phase_) {
  case Phase::SendFrontExtendAndPreloadPickup:
    publishStop();
    if (!command_a_.sent) {
      (void)sendCommand(command_a_);
    }
    if (!command_b_.sent) {
      (void)sendCommand(command_b_);
    }
    if (command_a_.sent && command_b_.sent) {
      phase_ = Phase::WaitFrontExtendSettleAndPreloadDone;
      phase_tp_ = std::chrono::steady_clock::now();
      last_log_tp_ = phase_tp_;
      return BT::NodeStatus::RUNNING;
    }
    if (phaseElapsed() > params_.command_timeout_s) {
      return fail("等待放置后并发机构命令服务可用超时");
    }
    return BT::NodeStatus::RUNNING;

  case Phase::WaitFrontExtendSettleAndPreloadDone: {
    publishStop();
    if (commandRejected(command_a_) || commandRejected(command_b_)) {
      return fail("放置后并发机构命令被拒绝");
    }
    if (phaseElapsed() > params_.command_timeout_s &&
        (!commandAcked(command_a_) || !commandAcked(command_b_))) {
      return fail("等待放置后并发机构命令 ACK 超时");
    }
    if (commandAcked(command_a_) && !front_extend_settle_started_) {
      front_extend_ack_tp_ = std::chrono::steady_clock::now();
      front_extend_settle_started_ = true;
    }
    const bool front_settled =
        front_extend_settle_started_ &&
        elapsedSec(front_extend_ack_tp_) >=
            params_.post_place_front_pushrod_extend_settle_s;
    if (commandDone(command_b_) && front_settled) {
      beginManualFrontLaserWait();
      return BT::NodeStatus::RUNNING;
    }
    const double post_place_wait_timeout_s =
        std::max(params_.done_timeout_s,
                 params_.command_timeout_s +
                     params_.post_place_front_pushrod_extend_settle_s);
    if (phaseElapsed() > post_place_wait_timeout_s) {
      return fail("等待预装 KFS 夹取完成 0x14 或前推杆延时超时");
    }
    if (elapsedSec(last_log_tp_) >= params_.log_period_s) {
      RCLCPP_INFO(
          node_->get_logger(),
          "第二预选赛放置后流程等待: front_ack=%s front_settle=%s preload_done=%s busy=%s",
          commandAcked(command_a_) ? "是" : "否", front_settled ? "是" : "否",
          commandDone(command_b_) ? "是" : "否",
          command_busy_seen_.load(std::memory_order_relaxed) ? "是" : "否");
      last_log_tp_ = std::chrono::steady_clock::now();
    }
    return BT::NodeStatus::RUNNING;
  }

  case Phase::WaitManualFrontLaser:
    publishStop();
    if (manual_front_laser_count_.load(std::memory_order_relaxed) >
        manual_front_laser_baseline_) {
      beginFrontRetractRearExtend();
      return BT::NodeStatus::RUNNING;
    }
    if (phaseElapsed() > params_.post_place_manual_front_laser_timeout_s) {
      return fail("等待人工触发前轮激光 0x15 超时");
    }
    if (elapsedSec(last_log_tp_) >= params_.log_period_s) {
      RCLCPP_INFO(node_->get_logger(),
                  "第二预选赛放置后流程等待人工前轮激光 %s elapsed=%.1fs",
                  byteHex(params_.post_place_manual_front_laser_feedback_id).c_str(),
                  phaseElapsed());
      last_log_tp_ = std::chrono::steady_clock::now();
    }
    return BT::NodeStatus::RUNNING;

  case Phase::SendFrontRetractAndRearExtend:
    publishStop();
    if (!command_a_.sent) {
      (void)sendCommand(command_a_);
    }
    if (!command_b_.sent) {
      (void)sendCommand(command_b_);
    }
    if (command_a_.sent && command_b_.sent) {
      phase_ = Phase::WaitFrontRetractAndRearExtendAck;
      phase_tp_ = std::chrono::steady_clock::now();
    } else if (phaseElapsed() > params_.command_timeout_s) {
      return fail("等待前推杆收回和后推杆伸出服务可用超时");
    }
    return BT::NodeStatus::RUNNING;

  case Phase::WaitFrontRetractAndRearExtendAck:
    publishStop();
    if (commandRejected(command_a_) || commandRejected(command_b_)) {
      return fail("前推杆收回或后推杆伸出命令被拒绝");
    }
    if (commandAcked(command_a_) && commandAcked(command_b_)) {
      phase_ = Phase::HoldAfterFrontRetractAndRearExtend;
      phase_tp_ = std::chrono::steady_clock::now();
      return BT::NodeStatus::RUNNING;
    }
    if (phaseElapsed() > params_.command_timeout_s) {
      return fail("等待前推杆收回和后推杆伸出 ACK 超时");
    }
    return BT::NodeStatus::RUNNING;

  case Phase::HoldAfterFrontRetractAndRearExtend:
    publishStop();
    if (phaseElapsed() >= stair_params_.climb_retract_rear_extend_delay_s) {
      beginRearDrive();
    }
    return BT::NodeStatus::RUNNING;

  case Phase::DriveUntilRearEvent:
    if (tickDriveYawGate("second_post_place_rear_drive")) {
      return BT::NodeStatus::RUNNING;
    }
    publishDrive(rearDriveSpeed());
    if (rear_laser_count_.load(std::memory_order_relaxed) > rear_laser_baseline_) {
      publishStop();
      beginRearRetract();
      return BT::NodeStatus::RUNNING;
    }
    if (phaseElapsed() > stair_params_.rear_event_timeout_s) {
      return fail("等待后轮激光高度突变超时");
    }
    return BT::NodeStatus::RUNNING;

  case Phase::SendRearRetract:
    publishStop();
    if (!command_a_.sent) {
      (void)sendCommand(command_a_);
    }
    if (command_a_.sent) {
      phase_ = Phase::WaitRearRetractAck;
      phase_tp_ = std::chrono::steady_clock::now();
    } else if (phaseElapsed() > params_.command_timeout_s) {
      return fail("等待后推杆收回服务可用超时");
    }
    return BT::NodeStatus::RUNNING;

  case Phase::WaitRearRetractAck:
    publishStop();
    if (commandRejected(command_a_)) {
      return fail("后推杆收回命令被拒绝");
    }
    if (commandAcked(command_a_)) {
      phase_ = Phase::HoldAfterRearRetract;
      phase_tp_ = std::chrono::steady_clock::now();
      return BT::NodeStatus::RUNNING;
    }
    if (phaseElapsed() > params_.command_timeout_s) {
      return fail("等待后推杆收回 ACK 超时");
    }
    return BT::NodeStatus::RUNNING;

  case Phase::HoldAfterRearRetract:
    publishStop();
    if (phaseElapsed() >= stair_params_.climb_rear_retract_delay_s) {
      beginFinalDelay();
    }
    return BT::NodeStatus::RUNNING;

  case Phase::FinalDelay:
    publishStop();
    if (phaseElapsed() >= params_.post_place_final_delay_s) {
      beginFinalPlace();
    }
    return BT::NodeStatus::RUNNING;

  case Phase::SendFinalPlace:
    publishStop();
    if (!command_a_.sent) {
      (void)sendCommand(command_a_);
    }
    if (command_a_.sent) {
      phase_ = Phase::WaitFinalPlaceAck;
      phase_tp_ = std::chrono::steady_clock::now();
    } else if (phaseElapsed() > params_.command_timeout_s) {
      return fail("等待最终放置命令服务可用超时");
    }
    return BT::NodeStatus::RUNNING;

  case Phase::WaitFinalPlaceAck:
    publishStop();
    if (commandRejected(command_a_)) {
      return fail("最终放置命令被拒绝");
    }
    if (commandAcked(command_a_)) {
      phase_ = Phase::Done;
      clearRuntimeState();
      return BT::NodeStatus::SUCCESS;
    }
    if (phaseElapsed() > params_.command_timeout_s) {
      return fail("等待最终放置命令 ACK 超时");
    }
    return BT::NodeStatus::RUNNING;

  case Phase::Done:
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

void SecondPreselectionPostPlaceClimbAction::onHalted() {
  publishStop();
  clearRuntimeState();
}

BT::NodeStatus SecondPreselectionPostPlaceClimbAction::fail(
    const std::string &reason) {
  writeDecisionFailure(config().blackboard, "SecondPreselectionPostPlaceClimb",
                       reason);
  if (node_) {
    RCLCPP_WARN(node_->get_logger(), "第二预选赛放置后流程失败: %s",
                reason.c_str());
  }
  publishStop();
  clearRuntimeState();
  return BT::NodeStatus::FAILURE;
}

void SecondPreselectionPostPlaceClimbAction::clearRuntimeState() {
  publishStop();
  command_generation_.fetch_add(1, std::memory_order_relaxed);
  cmd_pub_.reset();
  odom_sub_.reset();
  send_client_.reset();
  feedback_sub_.reset();
  resetCommand(command_a_, 0, -1, "");
  resetCommand(command_b_, 0, -1, "");
  command_error_seen_ = false;
  command_busy_seen_ = false;
  command_error_detail_.clear();
  manual_front_laser_count_ = 0;
  rear_laser_count_ = 0;
  manual_front_laser_baseline_ = 0;
  rear_laser_baseline_ = 0;
  has_odom_ = false;
  target_yaw_set_ = false;
  front_extend_settle_started_ = false;
  phase_ = Phase::Done;
}

void SecondPreselectionPostPlaceClimbAction::resetCommand(
    CommandRuntime &command, int command_id, int done_feedback_id,
    const std::string &label) {
  command.command_id = clampByte(command_id);
  command.done_feedback_id = done_feedback_id;
  command.label = label;
  command.sent = false;
  command.response_seen = false;
  command.accepted = false;
  command.rejected = false;
  command.done_seen = done_feedback_id < 0;
  command.seq = -1;
}

bool SecondPreselectionPostPlaceClimbAction::sendCommand(
    CommandRuntime &command) {
  if (!send_client_ || !send_client_->service_is_ready()) {
    if (node_) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "第二预选赛放置后流程等待机构命令服务");
    }
    return false;
  }

  auto request = std::make_shared<SendCommandSrv::Request>();
  request->command_id = command.command_id;
  request->payload = emptyPayload();
  request->wait_ack = true;
  const uint64_t token = command_generation_.load(std::memory_order_relaxed);
  CommandRuntime *slot = &command;
  try {
    send_client_->async_send_request(
        request, [this, token, slot](rclcpp::Client<SendCommandSrv>::SharedFuture future) {
          if (token != command_generation_.load(std::memory_order_relaxed)) {
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
          slot->seq.store(seq, std::memory_order_relaxed);
          slot->accepted.store(accepted, std::memory_order_relaxed);
          slot->rejected.store(!accepted, std::memory_order_relaxed);
          slot->response_seen.store(true, std::memory_order_relaxed);
        });
  } catch (const std::exception &e) {
    command_error_detail_ =
        std::string("第二预选赛放置后机构命令发送异常: ") + e.what();
    command_error_seen_.store(true, std::memory_order_relaxed);
    return true;
  }
  command.sent = true;
  if (node_) {
    RCLCPP_INFO(node_->get_logger(),
                "第二预选赛放置后流程已下发机构命令: %s command=%s done=%s",
                command.label.c_str(), byteHex(command.command_id).c_str(),
                command.done_feedback_id >= 0
                    ? byteHex(command.done_feedback_id).c_str()
                    : "ACK-only");
  }
  return true;
}

bool SecondPreselectionPostPlaceClimbAction::commandAcked(
    const CommandRuntime &command) const {
  return command.response_seen.load(std::memory_order_relaxed) &&
         command.accepted.load(std::memory_order_relaxed);
}

bool SecondPreselectionPostPlaceClimbAction::commandRejected(
    const CommandRuntime &command) const {
  return command.response_seen.load(std::memory_order_relaxed) &&
         command.rejected.load(std::memory_order_relaxed);
}

bool SecondPreselectionPostPlaceClimbAction::commandDone(
    const CommandRuntime &command) const {
  return command.done_feedback_id < 0 ||
         command.done_seen.load(std::memory_order_relaxed);
}

void SecondPreselectionPostPlaceClimbAction::handleFeedback(
    const FeedbackMsg::SharedPtr msg) {
  if (!msg) {
    return;
  }
  std::optional<MechanismErrorDiagnostic> diagnostic;
  for (auto *command : {&command_a_, &command_b_}) {
    const int seq = command->seq.load(std::memory_order_relaxed);
    if (isSameSeqMechanismError(*msg, seq, diagnostic)) {
      const std::string detail = mechanismErrorDiagnosticText(*diagnostic);
      if (diagnostic->busy) {
        command_busy_seen_.store(true, std::memory_order_relaxed);
      } else {
        command_error_detail_ = detail;
        command_error_seen_.store(true, std::memory_order_relaxed);
      }
      return;
    }
    if (seq >= 0 && command->done_feedback_id >= 0 &&
        msg->seq == static_cast<uint8_t>(seq & 0xFF) &&
        msg->feedback_id ==
            static_cast<uint8_t>(command->done_feedback_id & 0xFF)) {
      command->done_seen.store(true, std::memory_order_relaxed);
      return;
    }
  }
  if (msg->feedback_id ==
      static_cast<uint8_t>(params_.post_place_manual_front_laser_feedback_id &
                           0xFF)) {
    manual_front_laser_count_.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  if (msg->feedback_id ==
      static_cast<uint8_t>(params_.post_place_rear_laser_feedback_id & 0xFF)) {
    rear_laser_count_.fetch_add(1, std::memory_order_relaxed);
  }
}

void SecondPreselectionPostPlaceClimbAction::publishStop() {
  if (cmd_pub_) {
    cmd_pub_->publish(geometry_msgs::msg::Twist{});
  }
}

void SecondPreselectionPostPlaceClimbAction::publishDrive(double vx_mps) {
  if (!cmd_pub_) {
    return;
  }
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = vx_mps;
  if (stair_params_.heading_hold_enable && headingOdomReady()) {
    cmd.angular.z = headingAngularZ();
  }
  cmd_pub_->publish(cmd);
}

bool SecondPreselectionPostPlaceClimbAction::headingOdomReady() const {
  if (!stair_params_.heading_hold_enable) {
    return true;
  }
  if (!has_odom_ || !target_yaw_set_) {
    return false;
  }
  return elapsedSec(last_odom_tp_) <= stair_params_.heading_odom_timeout_s;
}

double SecondPreselectionPostPlaceClimbAction::headingError() const {
  return normalizeAngle(target_yaw_rad_ - current_yaw_rad_);
}

double SecondPreselectionPostPlaceClimbAction::headingAngularZ() const {
  return std::clamp(stair_params_.heading_kp * headingError(),
                    -stair_params_.heading_max_speed_radps,
                    stair_params_.heading_max_speed_radps);
}

bool SecondPreselectionPostPlaceClimbAction::tickDriveYawGate(
    const char *label) {
  if (!stair_params_.heading_hold_enable) {
    return false;
  }
  if (!headingOdomReady()) {
    publishStop();
    phase_tp_ = std::chrono::steady_clock::now();
    rear_drive_profile_tp_ = phase_tp_;
    return true;
  }
  const double gate_rad = stair_params_.heading_gate_deg * kDeg2Rad;
  if (std::abs(headingError()) <= gate_rad) {
    return false;
  }
  geometry_msgs::msg::Twist cmd;
  cmd.angular.z = headingAngularZ();
  cmd_pub_->publish(cmd);
  phase_tp_ = std::chrono::steady_clock::now();
  rear_drive_profile_tp_ = phase_tp_;
  if (node_) {
    RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 1000,
        "第二预选赛放置后流程 %s yaw超gate，暂停线速度 error=%.3frad",
        label ? label : "rear_drive", headingError());
  }
  return true;
}

double SecondPreselectionPostPlaceClimbAction::rearDriveSpeed() const {
  return sampleStairSpeedProfile(stair_params_.climb_rear_drive_profile,
                                 elapsedSec(rear_drive_profile_tp_));
}

void SecondPreselectionPostPlaceClimbAction::beginManualFrontLaserWait() {
  manual_front_laser_baseline_ =
      manual_front_laser_count_.load(std::memory_order_relaxed);
  phase_ = Phase::WaitManualFrontLaser;
  phase_tp_ = std::chrono::steady_clock::now();
  last_log_tp_ = phase_tp_;
  RCLCPP_INFO(node_->get_logger(),
              "第二预选赛放置后流程等待人工触发前轮激光: feedback=%s timeout=%.1fs",
              byteHex(params_.post_place_manual_front_laser_feedback_id).c_str(),
              params_.post_place_manual_front_laser_timeout_s);
}

void SecondPreselectionPostPlaceClimbAction::beginFrontRetractRearExtend() {
  resetCommand(command_a_, params_.post_place_front_pushrod_retract_command_id,
               -1, "FRONT_PUSHROD_RETRACT");
  resetCommand(command_b_, params_.post_place_rear_pushrod_extend_command_id,
               -1, "REAR_PUSHROD_EXTEND");
  command_generation_.fetch_add(1, std::memory_order_relaxed);
  phase_ = Phase::SendFrontRetractAndRearExtend;
  phase_tp_ = std::chrono::steady_clock::now();
}

void SecondPreselectionPostPlaceClimbAction::beginRearDrive() {
  rear_laser_baseline_ = rear_laser_count_.load(std::memory_order_relaxed);
  phase_ = Phase::DriveUntilRearEvent;
  phase_tp_ = std::chrono::steady_clock::now();
  rear_drive_profile_tp_ = phase_tp_;
  RCLCPP_INFO(node_->get_logger(),
              "第二预选赛放置后流程开始后轮上阶: rear_event=%s timeout=%.1fs",
              byteHex(params_.post_place_rear_laser_feedback_id).c_str(),
              stair_params_.rear_event_timeout_s);
}

void SecondPreselectionPostPlaceClimbAction::beginRearRetract() {
  resetCommand(command_a_, params_.post_place_rear_pushrod_retract_command_id,
               -1, "REAR_PUSHROD_RETRACT");
  command_generation_.fetch_add(1, std::memory_order_relaxed);
  phase_ = Phase::SendRearRetract;
  phase_tp_ = std::chrono::steady_clock::now();
}

void SecondPreselectionPostPlaceClimbAction::beginFinalDelay() {
  phase_ = Phase::FinalDelay;
  phase_tp_ = std::chrono::steady_clock::now();
  RCLCPP_INFO(node_->get_logger(),
              "第二预选赛放置后上阶完成，最终放置前等待 %.1fs",
              params_.post_place_final_delay_s);
}

void SecondPreselectionPostPlaceClimbAction::beginFinalPlace() {
  resetCommand(command_a_, params_.post_place_final_command_id, -1,
               "SECOND_PRESELECTION_FINAL_PLACE_KFS");
  command_generation_.fetch_add(1, std::memory_order_relaxed);
  phase_ = Phase::SendFinalPlace;
  phase_tp_ = std::chrono::steady_clock::now();
}

double SecondPreselectionPostPlaceClimbAction::phaseElapsed() const {
  return elapsedSec(phase_tp_);
}

std::string SecondPreselectionPostPlaceClimbAction::byteHex(int value) {
  return rc26_decision::byteHex(value);
}

void loadSecondPreselectionParams(rclcpp::Node &node,
                                  const BT::Blackboard::Ptr &blackboard) {
  SecondPreselectionParams p;
  int mirror_sign = 1;
  if (blackboard) {
    (void)blackboard->get("team_mirror_sign", mirror_sign);
  }
  mirror_sign = normalizedMirrorSign(mirror_sign);
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
  p.pickup_command_id = node.declare_parameter<int>(
      "second_preselect_pickup_command_id", p.pickup_command_id);
  p.pickup_done_feedback_id = node.declare_parameter<int>(
      "second_preselect_pickup_done_feedback_id",
      p.pickup_done_feedback_id);
  p.pre_approach_lower_command_id = node.declare_parameter<int>(
      "second_preselect_pre_approach_lower_command_id",
      p.pre_approach_lower_command_id);
  p.pre_approach_lower_done_feedback_id = node.declare_parameter<int>(
      "second_preselect_pre_approach_lower_done_feedback_id",
      p.pre_approach_lower_done_feedback_id);
  p.pre_approach_lower_settle_s = node.declare_parameter<double>(
      "second_preselect_pre_approach_lower_settle_s",
      p.pre_approach_lower_settle_s);
  p.place_kfs_command_id = node.declare_parameter<int>(
      "second_preselect_place_kfs_command_id", p.place_kfs_command_id);
  p.post_place_retreat_x_m = node.declare_parameter<double>(
      "second_preselect_post_place_retreat_x_m", p.post_place_retreat_x_m);
  p.post_place_front_pushrod_extend_command_id =
      node.declare_parameter<int>(
          "second_preselect_post_place_front_pushrod_extend_command_id",
          p.post_place_front_pushrod_extend_command_id);
  p.post_place_front_pushrod_extend_settle_s =
      node.declare_parameter<double>(
          "second_preselect_post_place_front_pushrod_extend_settle_s",
          p.post_place_front_pushrod_extend_settle_s);
  p.post_place_preload_pickup_command_id = node.declare_parameter<int>(
      "second_preselect_post_place_preload_pickup_command_id",
      p.post_place_preload_pickup_command_id);
  p.post_place_preload_pickup_done_feedback_id =
      node.declare_parameter<int>(
          "second_preselect_post_place_preload_pickup_done_feedback_id",
          p.post_place_preload_pickup_done_feedback_id);
  p.post_place_manual_front_laser_feedback_id =
      node.declare_parameter<int>(
          "second_preselect_post_place_manual_front_laser_feedback_id",
          p.post_place_manual_front_laser_feedback_id);
  p.post_place_manual_front_laser_timeout_s =
      node.declare_parameter<double>(
          "second_preselect_post_place_manual_front_laser_timeout_s",
          p.post_place_manual_front_laser_timeout_s);
  p.post_place_front_pushrod_retract_command_id =
      node.declare_parameter<int>(
          "second_preselect_post_place_front_pushrod_retract_command_id",
          p.post_place_front_pushrod_retract_command_id);
  p.post_place_rear_pushrod_extend_command_id =
      node.declare_parameter<int>(
          "second_preselect_post_place_rear_pushrod_extend_command_id",
          p.post_place_rear_pushrod_extend_command_id);
  p.post_place_rear_laser_feedback_id = node.declare_parameter<int>(
      "second_preselect_post_place_rear_laser_feedback_id",
      p.post_place_rear_laser_feedback_id);
  p.post_place_rear_pushrod_retract_command_id =
      node.declare_parameter<int>(
          "second_preselect_post_place_rear_pushrod_retract_command_id",
          p.post_place_rear_pushrod_retract_command_id);
  p.post_place_final_delay_s = node.declare_parameter<double>(
      "second_preselect_post_place_final_delay_s",
      p.post_place_final_delay_s);
  p.post_place_final_command_id = node.declare_parameter<int>(
      "second_preselect_post_place_final_command_id",
      p.post_place_final_command_id);
  p.climb_place_forward_x_m = node.declare_parameter<double>(
      "second_preselect_climb_place_forward_x_m",
      p.climb_place_forward_x_m);
  p.climb_place_lateral_y_m = node.declare_parameter<double>(
      "second_preselect_climb_place_lateral_y_m",
      p.climb_place_lateral_y_m);
  p.climb_place_pre_climb_delay_msec = node.declare_parameter<int>(
      "second_preselect_climb_place_pre_climb_delay_msec",
      p.climb_place_pre_climb_delay_msec);
  p.climb_place_front_pushrod_extend_command_id =
      node.declare_parameter<int>(
          "second_preselect_climb_place_front_pushrod_extend_command_id",
          p.climb_place_front_pushrod_extend_command_id);
  p.climb_place_manual_front_laser_feedback_id =
      node.declare_parameter<int>(
          "second_preselect_climb_place_manual_front_laser_feedback_id",
          p.climb_place_manual_front_laser_feedback_id);
  p.climb_place_front_pushrod_retract_command_id =
      node.declare_parameter<int>(
          "second_preselect_climb_place_front_pushrod_retract_command_id",
          p.climb_place_front_pushrod_retract_command_id);
  p.climb_place_rear_pushrod_extend_command_id = node.declare_parameter<int>(
      "second_preselect_climb_place_rear_pushrod_extend_command_id",
      p.climb_place_rear_pushrod_extend_command_id);
  p.climb_place_rear_forward_x_m = node.declare_parameter<double>(
      "second_preselect_climb_place_rear_forward_x_m",
      p.climb_place_rear_forward_x_m);
  p.climb_place_rear_max_speed_mps = node.declare_parameter<double>(
      "second_preselect_climb_place_rear_max_speed_mps",
      p.climb_place_rear_max_speed_mps);
  p.climb_place_rear_min_speed_mps = node.declare_parameter<double>(
      "second_preselect_climb_place_rear_min_speed_mps",
      p.climb_place_rear_min_speed_mps);
  p.climb_place_rear_timeout_s = node.declare_parameter<double>(
      "second_preselect_climb_place_rear_timeout_s",
      p.climb_place_rear_timeout_s);
  p.climb_place_rear_pushrod_retract_command_id =
      node.declare_parameter<int>(
          "second_preselect_climb_place_rear_pushrod_retract_command_id",
          p.climb_place_rear_pushrod_retract_command_id);
  p.climb_place_preload_pickup_command_id = node.declare_parameter<int>(
      "second_preselect_climb_place_preload_pickup_command_id",
      p.climb_place_preload_pickup_command_id);
  p.climb_place_preload_pickup_done_feedback_id =
      node.declare_parameter<int>(
          "second_preselect_climb_place_preload_pickup_done_feedback_id",
          p.climb_place_preload_pickup_done_feedback_id);
  p.climb_place_final_delay_s = node.declare_parameter<double>(
      "second_preselect_climb_place_final_delay_s",
      p.climb_place_final_delay_s);
  p.climb_place_final_command_id = node.declare_parameter<int>(
      "second_preselect_climb_place_final_command_id",
      p.climb_place_final_command_id);
  p.cmd_vel_topic = node.declare_parameter<std::string>(
      "second_preselect_cmd_vel_topic", p.cmd_vel_topic);

  p.team_mirror_sign = mirror_sign;
  p.nav_y1_m =
      node.declare_parameter<double>("second_preselect_nav_y1_m", p.nav_y1_m);
  p.post_pickup_forward_x_m = node.declare_parameter<double>(
      "second_preselect_post_pickup_forward_x_m",
      p.post_pickup_forward_x_m);
  p.nav_max_speed_mps = node.declare_parameter<double>(
      "second_preselect_nav_max_speed_mps", p.nav_max_speed_mps);
  p.nav_min_speed_mps = node.declare_parameter<double>(
      "second_preselect_nav_min_speed_mps", p.nav_min_speed_mps);
  p.total_x_target_m = node.declare_parameter<double>(
      "second_preselect_total_x_target_m", p.total_x_target_m);
  p.total_x_tolerance_m = node.declare_parameter<double>(
      "second_preselect_total_x_tolerance_m", p.total_x_tolerance_m);
  p.nav_timeout_s = node.declare_parameter<double>(
      "second_preselect_nav_timeout_s", p.nav_timeout_s);
  p.place_fixed_forward_x_m = node.declare_parameter<double>(
      "second_preselect_place_fixed_forward_x_m",
      p.place_fixed_forward_x_m);
  p.place_fixed_forward_timeout_s = node.declare_parameter<double>(
      "second_preselect_place_fixed_forward_timeout_s",
      p.place_fixed_forward_timeout_s);
  p.place_observe_timeout_s = node.declare_parameter<double>(
      "second_preselect_place_observe_timeout_s", p.place_observe_timeout_s);
  p.place_occupied_center_x_min_ratio = node.declare_parameter<double>(
      "second_preselect_place_occupied_center_x_min_ratio",
      p.place_occupied_center_x_min_ratio);
  p.place_occupied_center_x_max_ratio = node.declare_parameter<double>(
      "second_preselect_place_occupied_center_x_max_ratio",
      p.place_occupied_center_x_max_ratio);
  p.place_occupied_middle_y_min_ratio = node.declare_parameter<double>(
      "second_preselect_place_occupied_middle_y_min_ratio",
      p.place_occupied_middle_y_min_ratio);
  p.place_occupied_middle_y_max_ratio = node.declare_parameter<double>(
      "second_preselect_place_occupied_middle_y_max_ratio",
      p.place_occupied_middle_y_max_ratio);
  p.place_occupied_lower_y_min_ratio = node.declare_parameter<double>(
      "second_preselect_place_occupied_lower_y_min_ratio",
      p.place_occupied_lower_y_min_ratio);
  p.place_occupied_lower_y_max_ratio = node.declare_parameter<double>(
      "second_preselect_place_occupied_lower_y_max_ratio",
      p.place_occupied_lower_y_max_ratio);
  p.place_occupied_stable_frames = node.declare_parameter<int>(
      "second_preselect_place_occupied_stable_frames",
      p.place_occupied_stable_frames);
  p.place_occupied_first_lateral_m = node.declare_parameter<double>(
      "second_preselect_place_occupied_first_lateral_m",
      p.place_occupied_first_lateral_m);
  p.place_occupied_second_reverse_m = node.declare_parameter<double>(
      "second_preselect_place_occupied_second_reverse_m",
      p.place_occupied_second_reverse_m);
  p.place_occupied_lateral_max_speed_mps = node.declare_parameter<double>(
      "second_preselect_place_occupied_lateral_max_speed_mps",
      p.place_occupied_lateral_max_speed_mps);
  p.place_occupied_lateral_min_speed_mps = node.declare_parameter<double>(
      "second_preselect_place_occupied_lateral_min_speed_mps",
      p.place_occupied_lateral_min_speed_mps);
  p.place_occupied_lateral_timeout_s = node.declare_parameter<double>(
      "second_preselect_place_occupied_lateral_timeout_s",
      p.place_occupied_lateral_timeout_s);
  p.ramp_forward_x_m = node.declare_parameter<double>(
      "preselection_ramp_forward_x_m", p.ramp_forward_x_m);
  p.ramp_max_speed_mps = node.declare_parameter<double>(
      "preselection_ramp_max_speed_mps", p.ramp_max_speed_mps);
  p.ramp_min_speed_mps = node.declare_parameter<double>(
      "preselection_ramp_min_speed_mps", p.ramp_min_speed_mps);
  p.ramp_forward_timeout_s = node.declare_parameter<double>(
      "preselection_ramp_forward_timeout_s", p.ramp_forward_timeout_s);
  const double configured_after_ramp_turn_delta_rad =
      node.declare_parameter<double>("second_preselect_after_ramp_turn_delta_rad",
                                     p.after_ramp_turn_delta_rad);
  p.after_ramp_turn_delta_rad =
      configured_after_ramp_turn_delta_rad * static_cast<double>(mirror_sign);
  p.after_ramp_turn_timeout_s = node.declare_parameter<double>(
      "second_preselect_after_ramp_turn_timeout_s",
      p.after_ramp_turn_timeout_s);

  p.vision_config_file = node.declare_parameter<std::string>(
      "second_preselect_vision_config_file", p.vision_config_file);
  p.model_id = node.declare_parameter<std::string>(
      "second_preselect_model_id", p.model_id);
  p.search_forward_speed_mps = node.declare_parameter<double>(
      "second_preselect_search_forward_speed_mps",
      p.search_forward_speed_mps);
  p.search_timeout_s = node.declare_parameter<double>(
      "second_preselect_search_timeout_s", p.search_timeout_s);
  p.r2_target_label_prefixes =
      node.declare_parameter<std::vector<std::string>>(
          "second_preselect_r2_target_label_prefixes",
          p.r2_target_label_prefixes);
  p.r2_target_labels = node.declare_parameter<std::vector<std::string>>(
      "second_preselect_r2_target_labels", p.r2_target_labels);
  p.r1_blocking_labels = node.declare_parameter<std::vector<std::string>>(
      "second_preselect_r1_blocking_labels", p.r1_blocking_labels);
  p.r1_blocking_label_prefixes =
      node.declare_parameter<std::vector<std::string>>(
          "second_preselect_r1_blocking_label_prefixes",
          p.r1_blocking_label_prefixes);
  p.r1_kfs_min_score = node.declare_parameter<double>(
      "second_preselect_r1_kfs_min_score", p.r1_kfs_min_score);
  p.depth_min_m = node.declare_parameter<double>(
      "second_preselect_kfs_depth_min_m", p.depth_min_m);
  p.depth_max_m = node.declare_parameter<double>(
      "second_preselect_kfs_depth_max_m", p.depth_max_m);
  p.kfs_align_tolerance_px = node.declare_parameter<int>(
      "second_preselect_kfs_align_tolerance_px",
      p.kfs_align_tolerance_px);
  p.kfs_align_target_line_offset_px = node.declare_parameter<int>(
      "second_preselect_kfs_align_target_line_offset_px",
      p.kfs_align_target_line_offset_px);
  p.kfs_align_stable_frames = node.declare_parameter<int>(
      "second_preselect_kfs_align_stable_frames",
      p.kfs_align_stable_frames);
  p.kfs_align_max_jump_px = node.declare_parameter<int>(
      "second_preselect_kfs_align_max_jump_px", p.kfs_align_max_jump_px);
  p.kfs_align_kp = node.declare_parameter<double>(
      "second_preselect_kfs_align_kp", p.kfs_align_kp);
  p.kfs_align_min_speed_mps = node.declare_parameter<double>(
      "second_preselect_kfs_align_min_speed_mps",
      p.kfs_align_min_speed_mps);
  p.kfs_align_max_speed_mps = node.declare_parameter<double>(
      "second_preselect_kfs_align_max_speed_mps",
      p.kfs_align_max_speed_mps);
  p.kfs_align_timeout_s = node.declare_parameter<double>(
      "second_preselect_kfs_align_timeout_s", p.kfs_align_timeout_s);
  p.kfs_align_timeout_pickup_tolerance_px = node.declare_parameter<int>(
      "second_preselect_kfs_align_timeout_pickup_tolerance_px",
      p.kfs_align_timeout_pickup_tolerance_px);
  p.kfs_align_heading_gate_deg = node.declare_parameter<double>(
      "second_preselect_kfs_align_heading_gate_deg",
      p.kfs_align_heading_gate_deg);
  p.kfs_lost_stop_frames = node.declare_parameter<int>(
      "second_preselect_kfs_lost_stop_frames", p.kfs_lost_stop_frames);
  p.kfs_lost_servo_speed_scale = node.declare_parameter<double>(
      "second_preselect_kfs_lost_servo_speed_scale",
      p.kfs_lost_servo_speed_scale);
  p.kfs_align_offset_filter_alpha = node.declare_parameter<double>(
      "second_preselect_kfs_align_offset_filter_alpha",
      p.kfs_align_offset_filter_alpha);
  p.kfs_invert_lateral_direction = node.declare_parameter<bool>(
      "second_preselect_kfs_invert_lateral_direction",
      p.kfs_invert_lateral_direction);
  p.kfs_odom_xy_kp = node.declare_parameter<double>(
      "second_preselect_kfs_odom_xy_kp", p.kfs_odom_xy_kp);
  p.kfs_approach_odom_tolerance_m = node.declare_parameter<double>(
      "second_preselect_kfs_approach_odom_tolerance_m",
      p.kfs_approach_odom_tolerance_m);
  p.kfs_odom_yaw_tolerance_deg = node.declare_parameter<double>(
      "second_preselect_kfs_odom_yaw_tolerance_deg",
      p.kfs_odom_yaw_tolerance_deg);
  p.kfs_odom_stable_ticks = node.declare_parameter<int>(
      "second_preselect_kfs_odom_stable_ticks", p.kfs_odom_stable_ticks);
  p.kfs_approach_speed_mps = node.declare_parameter<double>(
      "second_preselect_kfs_approach_speed_mps",
      p.kfs_approach_speed_mps);
  p.kfs_approach_min_speed_mps = node.declare_parameter<double>(
      "second_preselect_kfs_approach_min_speed_mps",
      p.kfs_approach_min_speed_mps);
  p.kfs_approach_x_sign = node.declare_parameter<int>(
      "second_preselect_kfs_approach_x_sign", p.kfs_approach_x_sign);
  p.kfs_approach_timeout_s = node.declare_parameter<double>(
      "second_preselect_kfs_approach_timeout_s",
      p.kfs_approach_timeout_s);
  p.kfs_grab_distance_m = node.declare_parameter<double>(
      "second_preselect_kfs_grab_distance_m", p.kfs_grab_distance_m);
  p.kfs_heading_kp = node.declare_parameter<double>(
      "second_preselect_kfs_heading_kp", p.kfs_heading_kp);
  p.kfs_heading_max_speed_radps = node.declare_parameter<double>(
      "second_preselect_kfs_heading_max_speed_radps",
      p.kfs_heading_max_speed_radps);
  p.kfs_mono_distance_fallback_enable = node.declare_parameter<bool>(
      "second_preselect_kfs_mono_distance_fallback_enable",
      p.kfs_mono_distance_fallback_enable);
  p.kfs_mono_target_width_m = node.declare_parameter<double>(
      "second_preselect_kfs_mono_target_width_m",
      p.kfs_mono_target_width_m);
  p.kfs_mono_target_height_m = node.declare_parameter<double>(
      "second_preselect_kfs_mono_target_height_m",
      p.kfs_mono_target_height_m);
  p.kfs_mono_fx_px = node.declare_parameter<double>(
      "second_preselect_kfs_mono_fx_px", p.kfs_mono_fx_px);
  p.kfs_mono_fy_px = node.declare_parameter<double>(
      "second_preselect_kfs_mono_fy_px", p.kfs_mono_fy_px);
  p.kfs_mono_min_bbox_px = node.declare_parameter<int>(
      "second_preselect_kfs_mono_min_bbox_px", p.kfs_mono_min_bbox_px);
  p.kfs_mono_max_delta_from_locked_m = node.declare_parameter<double>(
      "second_preselect_kfs_mono_max_delta_from_locked_m",
      p.kfs_mono_max_delta_from_locked_m);
  p.kfs_depth_roi_size = node.declare_parameter<int>(
      "second_preselect_kfs_depth_roi_size", p.kfs_depth_roi_size);
  p.kfs_depth_min_valid_count = node.declare_parameter<int>(
      "second_preselect_kfs_depth_min_valid_count",
      p.kfs_depth_min_valid_count);
  p.kfs_depth_bbox_sample_ratios =
      node.declare_parameter<std::vector<double>>(
          "second_preselect_kfs_depth_bbox_sample_ratios",
          p.kfs_depth_bbox_sample_ratios);
  p.kfs_depth_bbox_min_success_count = node.declare_parameter<int>(
      "second_preselect_kfs_depth_bbox_min_success_count",
      p.kfs_depth_bbox_min_success_count);
  p.grab_verify_timeout_s = node.declare_parameter<double>(
      "second_preselect_grab_verify_timeout_s", p.grab_verify_timeout_s);
  p.grab_verify_lost_stable_frames = node.declare_parameter<int>(
      "second_preselect_grab_verify_lost_stable_frames",
      p.grab_verify_lost_stable_frames);
  p.grab_verify_iou_threshold = node.declare_parameter<double>(
      "second_preselect_grab_verify_iou_threshold",
      p.grab_verify_iou_threshold);
  p.grab_settle_s = node.declare_parameter<double>(
      "second_preselect_grab_settle_s", p.grab_settle_s);
  p.dynamic_roi_ui_enable = node.declare_parameter<bool>(
      "second_preselect_dynamic_roi_ui_enable",
      p.dynamic_roi_ui_enable);
  p.dynamic_roi_ui_window_name = node.declare_parameter<std::string>(
      "second_preselect_dynamic_roi_ui_window_name",
      p.dynamic_roi_ui_window_name);
  if (p.dynamic_roi_ui_window_name.empty()) {
    p.dynamic_roi_ui_window_name =
        SecondPreselectionParams{}.dynamic_roi_ui_window_name;
  }
  p.odom_topic = node.declare_parameter<std::string>(
      "second_preselect_odom_topic", p.odom_topic);
  p.odom_timeout_s = node.declare_parameter<double>(
      "second_preselect_odom_timeout_s", p.odom_timeout_s);

  p.command_timeout_s = std::max(0.001, p.command_timeout_s);
  p.done_timeout_s = std::max(0.001, p.done_timeout_s);
  p.log_period_s = std::max(0.1, p.log_period_s);
  p.nav_y1_m = std::isfinite(p.nav_y1_m)
                   ? p.nav_y1_m
                   : SecondPreselectionParams{}.nav_y1_m;
  p.post_pickup_forward_x_m =
      (std::isfinite(p.post_pickup_forward_x_m) &&
       p.post_pickup_forward_x_m >= 0.0)
          ? p.post_pickup_forward_x_m
          : SecondPreselectionParams{}.post_pickup_forward_x_m;
  p.total_x_target_m =
      (std::isfinite(p.total_x_target_m) && p.total_x_target_m > 0.0)
          ? p.total_x_target_m
          : SecondPreselectionParams{}.total_x_target_m;
  p.total_x_tolerance_m =
      (std::isfinite(p.total_x_tolerance_m) &&
       p.total_x_tolerance_m > 0.0)
          ? p.total_x_tolerance_m
          : SecondPreselectionParams{}.total_x_tolerance_m;
  p.nav_timeout_s = std::max(0.001, p.nav_timeout_s);
  p.place_fixed_forward_x_m =
      (std::isfinite(p.place_fixed_forward_x_m) &&
       p.place_fixed_forward_x_m >= 0.0)
          ? p.place_fixed_forward_x_m
          : SecondPreselectionParams{}.place_fixed_forward_x_m;
  p.place_fixed_forward_timeout_s =
      (std::isfinite(p.place_fixed_forward_timeout_s) &&
       p.place_fixed_forward_timeout_s > 0.0)
          ? p.place_fixed_forward_timeout_s
          : SecondPreselectionParams{}.place_fixed_forward_timeout_s;
  p.place_observe_timeout_s =
      (std::isfinite(p.place_observe_timeout_s) &&
       p.place_observe_timeout_s > 0.0)
          ? p.place_observe_timeout_s
          : SecondPreselectionParams{}.place_observe_timeout_s;
  p.place_occupied_center_x_min_ratio =
      std::isfinite(p.place_occupied_center_x_min_ratio)
          ? std::clamp(p.place_occupied_center_x_min_ratio, 0.0, 1.0)
          : SecondPreselectionParams{}.place_occupied_center_x_min_ratio;
  p.place_occupied_center_x_max_ratio =
      std::isfinite(p.place_occupied_center_x_max_ratio)
          ? std::clamp(p.place_occupied_center_x_max_ratio, 0.0, 1.0)
          : SecondPreselectionParams{}.place_occupied_center_x_max_ratio;
  if (p.place_occupied_center_x_min_ratio >
      p.place_occupied_center_x_max_ratio) {
    p.place_occupied_center_x_min_ratio =
        SecondPreselectionParams{}.place_occupied_center_x_min_ratio;
    p.place_occupied_center_x_max_ratio =
        SecondPreselectionParams{}.place_occupied_center_x_max_ratio;
  }
  p.place_occupied_middle_y_min_ratio =
      std::isfinite(p.place_occupied_middle_y_min_ratio)
          ? std::clamp(p.place_occupied_middle_y_min_ratio, 0.0, 1.0)
          : SecondPreselectionParams{}.place_occupied_middle_y_min_ratio;
  p.place_occupied_middle_y_max_ratio =
      std::isfinite(p.place_occupied_middle_y_max_ratio)
          ? std::clamp(p.place_occupied_middle_y_max_ratio, 0.0, 1.0)
          : SecondPreselectionParams{}.place_occupied_middle_y_max_ratio;
  if (p.place_occupied_middle_y_min_ratio >
      p.place_occupied_middle_y_max_ratio) {
    p.place_occupied_middle_y_min_ratio =
        SecondPreselectionParams{}.place_occupied_middle_y_min_ratio;
    p.place_occupied_middle_y_max_ratio =
        SecondPreselectionParams{}.place_occupied_middle_y_max_ratio;
  }
  p.place_occupied_lower_y_min_ratio =
      std::isfinite(p.place_occupied_lower_y_min_ratio)
          ? std::clamp(p.place_occupied_lower_y_min_ratio, 0.0, 1.0)
          : SecondPreselectionParams{}.place_occupied_lower_y_min_ratio;
  p.place_occupied_lower_y_max_ratio =
      std::isfinite(p.place_occupied_lower_y_max_ratio)
          ? std::clamp(p.place_occupied_lower_y_max_ratio, 0.0, 1.0)
          : SecondPreselectionParams{}.place_occupied_lower_y_max_ratio;
  if (p.place_occupied_lower_y_min_ratio >
      p.place_occupied_lower_y_max_ratio) {
    p.place_occupied_lower_y_min_ratio =
        SecondPreselectionParams{}.place_occupied_lower_y_min_ratio;
    p.place_occupied_lower_y_max_ratio =
        SecondPreselectionParams{}.place_occupied_lower_y_max_ratio;
  }
  p.place_occupied_stable_frames =
      std::max(1, p.place_occupied_stable_frames);
  p.place_occupied_first_lateral_m =
      (std::isfinite(p.place_occupied_first_lateral_m) &&
       p.place_occupied_first_lateral_m >= 0.0)
          ? p.place_occupied_first_lateral_m
          : SecondPreselectionParams{}.place_occupied_first_lateral_m;
  p.place_occupied_second_reverse_m =
      (std::isfinite(p.place_occupied_second_reverse_m) &&
       p.place_occupied_second_reverse_m >= 0.0)
          ? p.place_occupied_second_reverse_m
          : SecondPreselectionParams{}.place_occupied_second_reverse_m;
  p.place_occupied_lateral_max_speed_mps =
      (std::isfinite(p.place_occupied_lateral_max_speed_mps) &&
       p.place_occupied_lateral_max_speed_mps > 0.0)
          ? std::abs(p.place_occupied_lateral_max_speed_mps)
          : SecondPreselectionParams{}.place_occupied_lateral_max_speed_mps;
  p.place_occupied_lateral_min_speed_mps =
      (std::isfinite(p.place_occupied_lateral_min_speed_mps) &&
       p.place_occupied_lateral_min_speed_mps >= 0.0)
          ? std::min(std::abs(p.place_occupied_lateral_min_speed_mps),
                     p.place_occupied_lateral_max_speed_mps)
          : SecondPreselectionParams{}.place_occupied_lateral_min_speed_mps;
  p.place_occupied_lateral_timeout_s =
      (std::isfinite(p.place_occupied_lateral_timeout_s) &&
       p.place_occupied_lateral_timeout_s > 0.0)
          ? p.place_occupied_lateral_timeout_s
          : SecondPreselectionParams{}.place_occupied_lateral_timeout_s;
  p.ramp_max_speed_mps =
      (std::isfinite(p.ramp_max_speed_mps) && p.ramp_max_speed_mps > 0.0)
          ? std::abs(p.ramp_max_speed_mps)
          : SecondPreselectionParams{}.ramp_max_speed_mps;
  p.ramp_min_speed_mps =
      (std::isfinite(p.ramp_min_speed_mps) && p.ramp_min_speed_mps >= 0.0)
          ? std::min(std::abs(p.ramp_min_speed_mps), p.ramp_max_speed_mps)
          : SecondPreselectionParams{}.ramp_min_speed_mps;
  p.ramp_forward_x_m =
      (std::isfinite(p.ramp_forward_x_m) && p.ramp_forward_x_m >= 0.0)
          ? p.ramp_forward_x_m
          : SecondPreselectionParams{}.ramp_forward_x_m;
  p.ramp_forward_timeout_s =
      (std::isfinite(p.ramp_forward_timeout_s) &&
       p.ramp_forward_timeout_s > 0.0)
          ? p.ramp_forward_timeout_s
          : SecondPreselectionParams{}.ramp_forward_timeout_s;
  p.after_ramp_turn_delta_rad =
      std::isfinite(p.after_ramp_turn_delta_rad)
          ? p.after_ramp_turn_delta_rad
          : SecondPreselectionParams{}.after_ramp_turn_delta_rad;
  p.after_ramp_turn_timeout_s =
      std::max(0.001, p.after_ramp_turn_timeout_s);
  p.search_forward_speed_mps =
      (std::isfinite(p.search_forward_speed_mps) &&
       p.search_forward_speed_mps > 0.0)
          ? std::abs(p.search_forward_speed_mps)
          : SecondPreselectionParams{}.search_forward_speed_mps;
  p.pre_approach_lower_done_feedback_id =
      std::clamp(p.pre_approach_lower_done_feedback_id, 0, 255);
  p.pre_approach_lower_settle_s =
      std::isfinite(p.pre_approach_lower_settle_s)
          ? std::max(0.0, p.pre_approach_lower_settle_s)
          : SecondPreselectionParams{}.pre_approach_lower_settle_s;
  p.pickup_done_feedback_id = std::clamp(p.pickup_done_feedback_id, 0, 255);
  p.post_place_front_pushrod_extend_settle_s =
      std::isfinite(p.post_place_front_pushrod_extend_settle_s)
          ? std::max(0.0, p.post_place_front_pushrod_extend_settle_s)
          : SecondPreselectionParams{}.post_place_front_pushrod_extend_settle_s;
  p.post_place_preload_pickup_done_feedback_id =
      std::clamp(p.post_place_preload_pickup_done_feedback_id, 0, 255);
  p.post_place_manual_front_laser_feedback_id =
      std::clamp(p.post_place_manual_front_laser_feedback_id, 0, 255);
  p.post_place_manual_front_laser_timeout_s =
      std::isfinite(p.post_place_manual_front_laser_timeout_s)
          ? std::max(0.001, p.post_place_manual_front_laser_timeout_s)
          : SecondPreselectionParams{}.post_place_manual_front_laser_timeout_s;
  p.post_place_rear_laser_feedback_id =
      std::clamp(p.post_place_rear_laser_feedback_id, 0, 255);
  p.post_place_final_delay_s =
      std::isfinite(p.post_place_final_delay_s)
          ? std::max(0.0, p.post_place_final_delay_s)
          : SecondPreselectionParams{}.post_place_final_delay_s;
  p.climb_place_forward_x_m =
      std::isfinite(p.climb_place_forward_x_m)
          ? std::max(0.0, p.climb_place_forward_x_m)
          : SecondPreselectionParams{}.climb_place_forward_x_m;
  p.climb_place_lateral_y_m =
      std::isfinite(p.climb_place_lateral_y_m)
          ? p.climb_place_lateral_y_m
          : SecondPreselectionParams{}.climb_place_lateral_y_m;
  p.climb_place_pre_climb_delay_msec =
      std::max(0, p.climb_place_pre_climb_delay_msec);
  p.climb_place_front_pushrod_extend_command_id =
      std::clamp(p.climb_place_front_pushrod_extend_command_id, 0, 255);
  p.climb_place_manual_front_laser_feedback_id =
      std::clamp(p.climb_place_manual_front_laser_feedback_id, 0, 255);
  p.climb_place_front_pushrod_retract_command_id =
      std::clamp(p.climb_place_front_pushrod_retract_command_id, 0, 255);
  p.climb_place_rear_pushrod_extend_command_id =
      std::clamp(p.climb_place_rear_pushrod_extend_command_id, 0, 255);
  p.climb_place_rear_forward_x_m =
      std::isfinite(p.climb_place_rear_forward_x_m)
          ? std::max(0.0, p.climb_place_rear_forward_x_m)
          : SecondPreselectionParams{}.climb_place_rear_forward_x_m;
  p.climb_place_rear_max_speed_mps =
      (std::isfinite(p.climb_place_rear_max_speed_mps) &&
       p.climb_place_rear_max_speed_mps > 0.0)
          ? std::abs(p.climb_place_rear_max_speed_mps)
          : SecondPreselectionParams{}.climb_place_rear_max_speed_mps;
  p.climb_place_rear_min_speed_mps =
      (std::isfinite(p.climb_place_rear_min_speed_mps) &&
       p.climb_place_rear_min_speed_mps >= 0.0)
          ? std::min(std::abs(p.climb_place_rear_min_speed_mps),
                     p.climb_place_rear_max_speed_mps)
          : SecondPreselectionParams{}.climb_place_rear_min_speed_mps;
  p.climb_place_rear_timeout_s =
      (std::isfinite(p.climb_place_rear_timeout_s) &&
       p.climb_place_rear_timeout_s > 0.0)
          ? p.climb_place_rear_timeout_s
          : SecondPreselectionParams{}.climb_place_rear_timeout_s;
  p.climb_place_rear_pushrod_retract_command_id =
      std::clamp(p.climb_place_rear_pushrod_retract_command_id, 0, 255);
  p.climb_place_preload_pickup_command_id =
      std::clamp(p.climb_place_preload_pickup_command_id, 0, 255);
  p.climb_place_preload_pickup_done_feedback_id =
      std::clamp(p.climb_place_preload_pickup_done_feedback_id, 0, 255);
  p.climb_place_final_delay_s =
      std::isfinite(p.climb_place_final_delay_s)
          ? std::max(0.0, p.climb_place_final_delay_s)
          : SecondPreselectionParams{}.climb_place_final_delay_s;
  p.climb_place_final_command_id =
      std::clamp(p.climb_place_final_command_id, 0, 255);
  p.search_timeout_s = std::max(0.001, p.search_timeout_s);
  p.r1_kfs_min_score =
      std::isfinite(p.r1_kfs_min_score) ? p.r1_kfs_min_score
                                        : SecondPreselectionParams{}.r1_kfs_min_score;
  p.depth_min_m =
      std::isfinite(p.depth_min_m) ? std::max(0.0, p.depth_min_m)
                                   : SecondPreselectionParams{}.depth_min_m;
  p.depth_max_m =
      (std::isfinite(p.depth_max_m) && p.depth_max_m >= p.depth_min_m)
          ? p.depth_max_m
          : SecondPreselectionParams{}.depth_max_m;
  p.kfs_align_tolerance_px = std::max(0, p.kfs_align_tolerance_px);
  p.kfs_align_target_line_offset_px =
      std::clamp(p.kfs_align_target_line_offset_px, -10000, 10000);
  p.kfs_align_stable_frames = std::max(1, p.kfs_align_stable_frames);
  p.kfs_align_max_jump_px = std::max(0, p.kfs_align_max_jump_px);
  p.kfs_align_kp =
      (std::isfinite(p.kfs_align_kp) && p.kfs_align_kp > 0.0)
          ? p.kfs_align_kp
          : SecondPreselectionParams{}.kfs_align_kp;
  p.kfs_align_max_speed_mps =
      (std::isfinite(p.kfs_align_max_speed_mps) &&
       p.kfs_align_max_speed_mps > 0.0)
          ? std::abs(p.kfs_align_max_speed_mps)
          : SecondPreselectionParams{}.kfs_align_max_speed_mps;
  p.kfs_align_min_speed_mps =
      (std::isfinite(p.kfs_align_min_speed_mps) &&
       p.kfs_align_min_speed_mps >= 0.0)
          ? std::min(std::abs(p.kfs_align_min_speed_mps),
                     p.kfs_align_max_speed_mps)
          : SecondPreselectionParams{}.kfs_align_min_speed_mps;
  p.kfs_align_timeout_s = std::max(0.001, p.kfs_align_timeout_s);
  p.kfs_align_timeout_pickup_tolerance_px =
      std::max(0, p.kfs_align_timeout_pickup_tolerance_px);
  p.kfs_align_heading_gate_deg =
      std::isfinite(p.kfs_align_heading_gate_deg)
          ? std::abs(p.kfs_align_heading_gate_deg)
          : SecondPreselectionParams{}.kfs_align_heading_gate_deg;
  p.kfs_lost_stop_frames = std::max(1, p.kfs_lost_stop_frames);
  p.kfs_lost_servo_speed_scale =
      std::isfinite(p.kfs_lost_servo_speed_scale)
          ? std::clamp(p.kfs_lost_servo_speed_scale, 0.0, 1.0)
          : SecondPreselectionParams{}.kfs_lost_servo_speed_scale;
  p.kfs_align_offset_filter_alpha =
      std::isfinite(p.kfs_align_offset_filter_alpha)
          ? std::clamp(p.kfs_align_offset_filter_alpha, 0.05, 1.0)
          : SecondPreselectionParams{}.kfs_align_offset_filter_alpha;
  p.kfs_odom_xy_kp =
      (std::isfinite(p.kfs_odom_xy_kp) && p.kfs_odom_xy_kp > 0.0)
          ? p.kfs_odom_xy_kp
          : SecondPreselectionParams{}.kfs_odom_xy_kp;
  p.kfs_approach_odom_tolerance_m =
      (std::isfinite(p.kfs_approach_odom_tolerance_m) &&
       p.kfs_approach_odom_tolerance_m > 0.0)
          ? std::abs(p.kfs_approach_odom_tolerance_m)
          : SecondPreselectionParams{}.kfs_approach_odom_tolerance_m;
  p.kfs_odom_yaw_tolerance_deg =
      std::isfinite(p.kfs_odom_yaw_tolerance_deg)
          ? std::abs(p.kfs_odom_yaw_tolerance_deg)
          : SecondPreselectionParams{}.kfs_odom_yaw_tolerance_deg;
  p.kfs_odom_stable_ticks = std::max(1, p.kfs_odom_stable_ticks);
  p.kfs_approach_speed_mps =
      (std::isfinite(p.kfs_approach_speed_mps) &&
       p.kfs_approach_speed_mps > 0.0)
          ? std::abs(p.kfs_approach_speed_mps)
          : SecondPreselectionParams{}.kfs_approach_speed_mps;
  p.kfs_approach_min_speed_mps =
      (std::isfinite(p.kfs_approach_min_speed_mps) &&
       p.kfs_approach_min_speed_mps >= 0.0)
          ? std::min(std::abs(p.kfs_approach_min_speed_mps),
                     p.kfs_approach_speed_mps)
          : SecondPreselectionParams{}.kfs_approach_min_speed_mps;
  p.kfs_approach_x_sign = p.kfs_approach_x_sign < 0 ? -1 : 1;
  p.kfs_approach_timeout_s = std::max(0.001, p.kfs_approach_timeout_s);
  p.kfs_grab_distance_m =
      (std::isfinite(p.kfs_grab_distance_m) && p.kfs_grab_distance_m >= 0.0)
          ? p.kfs_grab_distance_m
          : SecondPreselectionParams{}.kfs_grab_distance_m;
  p.kfs_heading_kp =
      (std::isfinite(p.kfs_heading_kp) && p.kfs_heading_kp > 0.0)
          ? p.kfs_heading_kp
          : SecondPreselectionParams{}.kfs_heading_kp;
  p.kfs_heading_max_speed_radps =
      (std::isfinite(p.kfs_heading_max_speed_radps) &&
       p.kfs_heading_max_speed_radps > 0.0)
          ? std::abs(p.kfs_heading_max_speed_radps)
          : SecondPreselectionParams{}.kfs_heading_max_speed_radps;
  p.kfs_mono_target_width_m =
      (std::isfinite(p.kfs_mono_target_width_m) &&
       p.kfs_mono_target_width_m > 0.0)
          ? p.kfs_mono_target_width_m
          : SecondPreselectionParams{}.kfs_mono_target_width_m;
  p.kfs_mono_target_height_m =
      (std::isfinite(p.kfs_mono_target_height_m) &&
       p.kfs_mono_target_height_m > 0.0)
          ? p.kfs_mono_target_height_m
          : SecondPreselectionParams{}.kfs_mono_target_height_m;
  p.kfs_mono_fx_px =
      (std::isfinite(p.kfs_mono_fx_px) && p.kfs_mono_fx_px > 0.0)
          ? p.kfs_mono_fx_px
          : SecondPreselectionParams{}.kfs_mono_fx_px;
  p.kfs_mono_fy_px =
      (std::isfinite(p.kfs_mono_fy_px) && p.kfs_mono_fy_px > 0.0)
          ? p.kfs_mono_fy_px
          : SecondPreselectionParams{}.kfs_mono_fy_px;
  p.kfs_mono_min_bbox_px = std::max(1, p.kfs_mono_min_bbox_px);
  p.kfs_mono_max_delta_from_locked_m =
      (std::isfinite(p.kfs_mono_max_delta_from_locked_m) &&
       p.kfs_mono_max_delta_from_locked_m >= 0.0)
          ? p.kfs_mono_max_delta_from_locked_m
          : SecondPreselectionParams{}.kfs_mono_max_delta_from_locked_m;
  p.kfs_depth_roi_size = std::max(1, p.kfs_depth_roi_size);
  p.kfs_depth_min_valid_count = std::max(1, p.kfs_depth_min_valid_count);
  if (p.kfs_depth_bbox_sample_ratios.empty()) {
    p.kfs_depth_bbox_sample_ratios =
        SecondPreselectionParams{}.kfs_depth_bbox_sample_ratios;
  }
  p.kfs_depth_bbox_min_success_count =
      std::max(1, p.kfs_depth_bbox_min_success_count);
  p.grab_verify_timeout_s = std::max(0.001, p.grab_verify_timeout_s);
  p.grab_verify_lost_stable_frames =
      std::max(1, p.grab_verify_lost_stable_frames);
  p.grab_verify_iou_threshold =
      std::clamp(p.grab_verify_iou_threshold, 0.0, 1.0);
  p.grab_settle_s = std::max(0.0, p.grab_settle_s);
  p.nav_y1_m *= static_cast<double>(mirror_sign);
  p.climb_place_lateral_y_m *= static_cast<double>(mirror_sign);
  p.nav_max_speed_mps = std::max(0.001, std::abs(p.nav_max_speed_mps));
  p.nav_min_speed_mps =
      std::min(std::max(0.0, std::abs(p.nav_min_speed_mps)),
               p.nav_max_speed_mps);
  p.odom_timeout_s = std::max(0.001, p.odom_timeout_s);
  p.vision_config_file = resolveVisionConfig(p.vision_config_file);

  blackboard->set("second_preselection_params", p);
  blackboard->set("second_preselect_command_timeout_s", p.command_timeout_s);
  blackboard->set("second_preselect_done_timeout_s", p.done_timeout_s);
  blackboard->set("second_preselect_start_command_id", p.start_command_id);
  blackboard->set("second_preselect_start_done_feedback_id",
                  p.start_done_feedback_id);
  blackboard->set("second_preselect_pickup_command_id",
                  p.pickup_command_id);
  blackboard->set("second_preselect_pickup_done_feedback_id",
                  p.pickup_done_feedback_id);
  blackboard->set("second_preselect_pre_approach_lower_command_id",
                  p.pre_approach_lower_command_id);
  blackboard->set("second_preselect_pre_approach_lower_done_feedback_id",
                  p.pre_approach_lower_done_feedback_id);
  blackboard->set("second_preselect_pre_approach_lower_settle_s",
                  p.pre_approach_lower_settle_s);
  blackboard->set("second_preselect_place_kfs_command_id",
                  p.place_kfs_command_id);
  blackboard->set("second_preselect_post_place_retreat_x_m",
                  p.post_place_retreat_x_m);
  blackboard->set("second_preselect_post_place_front_pushrod_extend_command_id",
                  p.post_place_front_pushrod_extend_command_id);
  blackboard->set("second_preselect_post_place_front_pushrod_extend_settle_s",
                  p.post_place_front_pushrod_extend_settle_s);
  blackboard->set("second_preselect_post_place_preload_pickup_command_id",
                  p.post_place_preload_pickup_command_id);
  blackboard->set("second_preselect_post_place_preload_pickup_done_feedback_id",
                  p.post_place_preload_pickup_done_feedback_id);
  blackboard->set("second_preselect_post_place_manual_front_laser_feedback_id",
                  p.post_place_manual_front_laser_feedback_id);
  blackboard->set("second_preselect_post_place_manual_front_laser_timeout_s",
                  p.post_place_manual_front_laser_timeout_s);
  blackboard->set("second_preselect_post_place_front_pushrod_retract_command_id",
                  p.post_place_front_pushrod_retract_command_id);
  blackboard->set("second_preselect_post_place_rear_pushrod_extend_command_id",
                  p.post_place_rear_pushrod_extend_command_id);
  blackboard->set("second_preselect_post_place_rear_laser_feedback_id",
                  p.post_place_rear_laser_feedback_id);
  blackboard->set("second_preselect_post_place_rear_pushrod_retract_command_id",
                  p.post_place_rear_pushrod_retract_command_id);
  blackboard->set("second_preselect_post_place_final_delay_s",
                  p.post_place_final_delay_s);
  blackboard->set("second_preselect_post_place_final_command_id",
                  p.post_place_final_command_id);
  blackboard->set("second_preselect_climb_place_forward_x_m",
                  p.climb_place_forward_x_m);
  blackboard->set("second_preselect_climb_place_lateral_y_m",
                  p.climb_place_lateral_y_m);
  blackboard->set(
      "second_preselect_climb_place_pre_climb_delay_msec",
      static_cast<unsigned int>(p.climb_place_pre_climb_delay_msec));
  blackboard->set(
      "second_preselect_climb_place_front_pushrod_extend_command_id",
      p.climb_place_front_pushrod_extend_command_id);
  blackboard->set(
      "second_preselect_climb_place_manual_front_laser_feedback_id",
      p.climb_place_manual_front_laser_feedback_id);
  blackboard->set(
      "second_preselect_climb_place_front_pushrod_retract_command_id",
      p.climb_place_front_pushrod_retract_command_id);
  blackboard->set(
      "second_preselect_climb_place_rear_pushrod_extend_command_id",
      p.climb_place_rear_pushrod_extend_command_id);
  blackboard->set("second_preselect_climb_place_rear_forward_x_m",
                  p.climb_place_rear_forward_x_m);
  blackboard->set("second_preselect_climb_place_rear_max_speed_mps",
                  p.climb_place_rear_max_speed_mps);
  blackboard->set("second_preselect_climb_place_rear_min_speed_mps",
                  p.climb_place_rear_min_speed_mps);
  blackboard->set("second_preselect_climb_place_rear_timeout_s",
                  p.climb_place_rear_timeout_s);
  blackboard->set(
      "second_preselect_climb_place_rear_pushrod_retract_command_id",
      p.climb_place_rear_pushrod_retract_command_id);
  blackboard->set("second_preselect_climb_place_preload_pickup_command_id",
                  p.climb_place_preload_pickup_command_id);
  blackboard->set(
      "second_preselect_climb_place_preload_pickup_done_feedback_id",
      p.climb_place_preload_pickup_done_feedback_id);
  blackboard->set("second_preselect_climb_place_final_delay_s",
                  p.climb_place_final_delay_s);
  blackboard->set("second_preselect_climb_place_final_command_id",
                  p.climb_place_final_command_id);
  blackboard->set("second_preselect_cmd_vel_topic", p.cmd_vel_topic);
  blackboard->set("second_preselect_nav_y1_m", p.nav_y1_m);
  blackboard->set("second_preselect_post_pickup_forward_x_m",
                  p.post_pickup_forward_x_m);
  blackboard->set("second_preselect_nav_max_speed_mps",
                  p.nav_max_speed_mps);
  blackboard->set("second_preselect_nav_min_speed_mps",
                  p.nav_min_speed_mps);
  blackboard->set("second_preselect_total_x_target_m", p.total_x_target_m);
  blackboard->set("second_preselect_total_x_tolerance_m",
                  p.total_x_tolerance_m);
  blackboard->set("second_preselect_place_fixed_forward_x_m",
                  p.place_fixed_forward_x_m);
  blackboard->set("second_preselect_place_fixed_forward_timeout_s",
                  p.place_fixed_forward_timeout_s);
  blackboard->set("second_preselect_place_observe_timeout_s",
                  p.place_observe_timeout_s);
  blackboard->set("second_preselect_place_occupied_middle_y_min_ratio",
                  p.place_occupied_middle_y_min_ratio);
  blackboard->set("second_preselect_place_occupied_middle_y_max_ratio",
                  p.place_occupied_middle_y_max_ratio);
  blackboard->set("second_preselect_place_occupied_lower_y_min_ratio",
                  p.place_occupied_lower_y_min_ratio);
  blackboard->set("second_preselect_place_occupied_lower_y_max_ratio",
                  p.place_occupied_lower_y_max_ratio);
  blackboard->set("second_preselect_nav_timeout_s", p.nav_timeout_s);
  blackboard->set("preselection_ramp_forward_x_m", p.ramp_forward_x_m);
  blackboard->set("preselection_ramp_max_speed_mps", p.ramp_max_speed_mps);
  blackboard->set("preselection_ramp_min_speed_mps", p.ramp_min_speed_mps);
  blackboard->set("preselection_ramp_forward_timeout_s",
                  p.ramp_forward_timeout_s);
  blackboard->set("second_preselect_after_ramp_turn_delta_rad",
                  p.after_ramp_turn_delta_rad);
  blackboard->set("second_preselect_after_ramp_turn_timeout_s",
                  p.after_ramp_turn_timeout_s);
  blackboard->set("second_preselect_after_ramp_turn_target_yaw", 0.0);
  blackboard->set("second_preselect_odom_topic", p.odom_topic);
  blackboard->set("second_preselect_odom_timeout_s", p.odom_timeout_s);
  blackboard->set("second_preselect_search_forward_speed_mps",
                  p.search_forward_speed_mps);
  blackboard->set("second_preselect_search_timeout_s", p.search_timeout_s);

  RCLCPP_INFO(node.get_logger(),
              "第二预选赛参数已加载: mirror_sign=%d start=0x%02X/done=0x%02X lower=0x%02X/done=0x%02X/settle=%.2fs pickup=0x%02X/done=0x%02X place=0x%02X search=[speed %.2f timeout %.1f] nav=[post_pickup_x %.2f, y1 %.2f, total_x %.2f tol %.2f] place=[fixed_x %.2f timeout %.1f observe_timeout %.1f middle_y %.2f~%.2f lower_y %.2f~%.2f occupied_shift %.2f/%.2f] ramp=[forward %.2f max %.2f min %.2f timeout %.1f turn %.2f timeout %.1f] align=[timeout %.1f tolerance %d stable %d] odom=%s",
              mirror_sign,
              p.start_command_id & 0xFF, p.start_done_feedback_id & 0xFF,
              p.pre_approach_lower_command_id & 0xFF,
              p.pre_approach_lower_done_feedback_id & 0xFF,
              p.pre_approach_lower_settle_s,
              p.pickup_command_id & 0xFF,
              p.pickup_done_feedback_id & 0xFF,
              p.place_kfs_command_id & 0xFF,
              p.search_forward_speed_mps, p.search_timeout_s,
              p.post_pickup_forward_x_m, p.nav_y1_m, p.total_x_target_m,
              p.total_x_tolerance_m, p.place_fixed_forward_x_m,
              p.place_fixed_forward_timeout_s, p.place_observe_timeout_s,
              p.place_occupied_middle_y_min_ratio,
              p.place_occupied_middle_y_max_ratio,
              p.place_occupied_lower_y_min_ratio,
              p.place_occupied_lower_y_max_ratio,
              p.place_occupied_first_lateral_m,
              p.place_occupied_second_reverse_m,
              p.ramp_forward_x_m, p.ramp_max_speed_mps,
              p.ramp_min_speed_mps, p.ramp_forward_timeout_s,
              p.after_ramp_turn_delta_rad,
              p.after_ramp_turn_timeout_s,
              p.kfs_align_timeout_s, p.kfs_align_tolerance_px,
              p.kfs_align_stable_frames, p.odom_topic.c_str());
  RCLCPP_INFO(
      node.get_logger(),
      "第二预选赛独立上阶放置参数: route=[x %.2f y %.2f delay %dms] front=[extend=0x%02X laser=0x%02X retract=0x%02X rear_extend=0x%02X] rear=[x %.2f speed %.2f~%.2f timeout %.1f] finish=[rear_retract=0x%02X pickup=0x%02X/done=0x%02X delay %.1f final=0x%02X]",
      p.climb_place_forward_x_m, p.climb_place_lateral_y_m,
      p.climb_place_pre_climb_delay_msec,
      p.climb_place_front_pushrod_extend_command_id & 0xFF,
      p.climb_place_manual_front_laser_feedback_id & 0xFF,
      p.climb_place_front_pushrod_retract_command_id & 0xFF,
      p.climb_place_rear_pushrod_extend_command_id & 0xFF,
      p.climb_place_rear_forward_x_m, p.climb_place_rear_min_speed_mps,
      p.climb_place_rear_max_speed_mps, p.climb_place_rear_timeout_s,
      p.climb_place_rear_pushrod_retract_command_id & 0xFF,
      p.climb_place_preload_pickup_command_id & 0xFF,
      p.climb_place_preload_pickup_done_feedback_id & 0xFF,
      p.climb_place_final_delay_s, p.climb_place_final_command_id & 0xFF);
}

void registerSecondPreselectionNodes(BT::BehaviorTreeFactory &factory) {
  factory.registerNodeType<SecondPreselectionCommandAction>(
      "SecondPreselectionCommand");
  factory.registerNodeType<SecondPreselectionKfsPickupAction>(
      "SecondPreselectionKfsPickup");
  factory.registerNodeType<SecondPreselectionRampForwardAction>(
      "SecondPreselectionRampForward");
  factory.registerNodeType<SecondPreselectionDriveToTotalXAction>(
      "SecondPreselectionDriveToTotalX");
  factory.registerNodeType<SecondPreselectionKfsPlacePrepareAction>(
      "SecondPreselectionKfsPlacePrepare");
  factory.registerNodeType<SecondPreselectionPlaceApproachAction>(
      "SecondPreselectionPlaceApproach");
  factory.registerNodeType<SecondPreselectionClimbFrontStageAction>(
      "SecondPreselectionClimbFrontStage");
  factory.registerNodeType<SecondPreselectionRearRetractPickupPlaceAction>(
      "SecondPreselectionRearRetractPickupPlace");
  factory.registerNodeType<SecondPreselectionPostPlaceClimbAction>(
      "SecondPreselectionPostPlaceClimb");
}

} // namespace rc26_decision
