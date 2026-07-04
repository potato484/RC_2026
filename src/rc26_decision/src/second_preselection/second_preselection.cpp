#include "rc26_decision/second_preselection/second_preselection.hpp"

#include <algorithm>
#include <array>
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
#include "rc26_decision/team_color.hpp"
#include "rc26_serial/protocol.hpp"
#include "rc26_vision/inference/config/model_profile_loader.hpp"
#include "rc26_vision/shared/sensors/depth_roi_sampler.hpp"

namespace rc26_decision {

namespace {

constexpr std::array<int, 3> kGridCols{-1, 0, 1};
constexpr std::array<int, 3> kGridRows{-1, 0, 1};
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

bool pointInRoi(const cv::Point2f &point, const cv::Rect2f &roi) {
  return point.x >= roi.x && point.x <= roi.x + roi.width &&
         point.y >= roi.y && point.y <= roi.y + roi.height;
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

bool labelAllowed(const std::string &label,
                  const SecondPreselectionParams &params) {
  if (label.empty()) {
    return false;
  }
  if (params.grid_label_exact_names.empty() &&
      params.grid_label_prefixes.empty()) {
    return true;
  }
  if (std::find(params.grid_label_exact_names.begin(),
                params.grid_label_exact_names.end(),
                label) != params.grid_label_exact_names.end()) {
    return true;
  }
  for (const auto &prefix : params.grid_label_prefixes) {
    if (!prefix.empty() && startsWith(label, prefix)) {
      return true;
    }
  }
  return false;
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

int gridCellIndex(int col, int row) {
  const auto col_it = std::find(kGridCols.begin(), kGridCols.end(), col);
  const auto row_it = std::find(kGridRows.begin(), kGridRows.end(), row);
  if (col_it == kGridCols.end() || row_it == kGridRows.end()) {
    return -1;
  }
  return static_cast<int>(std::distance(kGridRows.begin(), row_it) * 3 +
                          std::distance(kGridCols.begin(), col_it));
}

double currentGridDistance(const SecondPreselectionParams &params,
                           double odom_delta_x_m) {
  return std::max(0.05, params.grid_initial_distance_m - odom_delta_x_m);
}

double currentGridLateralOffset(const SecondPreselectionParams &params,
                                double odom_delta_y_m) {
  return params.grid_initial_lateral_offset_m +
         params.grid_base_y_to_grid_x_sign * odom_delta_y_m;
}

double gridColumnCenterX(const SecondPreselectionParams &params, int col) {
  if (col < 0) {
    return -(params.grid_center_col_width_m + params.grid_left_col_width_m) *
           0.5;
  }
  if (col > 0) {
    return (params.grid_center_col_width_m + params.grid_right_col_width_m) *
           0.5;
  }
  return 0.0;
}

SecondPreselectionGridCellProjection projectGridCell(
    int col, int row, const SecondPreselectionParams &params, double distance_m,
    double lateral_offset_m) {
  SecondPreselectionGridCellProjection cell;
  cell.col = col;
  cell.row = row;

  const double center_grid_x_m = gridColumnCenterX(params, col);
  const double center_grid_height_m =
      params.grid_middle_center_height_m +
      static_cast<double>(row) * params.grid_row_pitch_m;
  const double u = params.grid_camera_ppx_px +
                   params.grid_camera_fx_px *
                       (center_grid_x_m - lateral_offset_m) / distance_m;
  const double v = params.grid_camera_ppy_px +
                   params.grid_camera_fy_px *
                       (params.grid_camera_height_m - center_grid_height_m) /
                       distance_m;
  const double width_px =
      params.grid_camera_fx_px * params.grid_safe_width_m / distance_m;
  const double height_px =
      params.grid_camera_fy_px * params.grid_safe_height_m / distance_m;

  cell.center =
      cv::Point2f(static_cast<float>(u), static_cast<float>(v));
  cell.roi = cv::Rect2f(static_cast<float>(u - width_px * 0.5),
                        static_cast<float>(v - height_px * 0.5),
                        static_cast<float>(width_px),
                        static_cast<float>(height_px));
  return cell;
}

std::optional<int> selectMiddleColumn(
    uint16_t mask, const SecondPreselectionParams &params) {
  (void)params;
  for (const int col : std::array<int, 3>{0, -1, 1}) {
    const int index = gridCellIndex(col, 0);
    if (index >= 0 && (mask & (static_cast<uint16_t>(1U) << index)) == 0U) {
      return col;
    }
  }
  return std::nullopt;
}

double selectedLateralMotion(const SecondPreselectionParams &params,
                             int selected_col, double odom_delta_y_m) {
  const double current_offset_m = currentGridLateralOffset(params, odom_delta_y_m);
  const double target_offset_m =
      gridColumnCenterX(params, selected_col) + params.grid_place_lateral_bias_m;
  return params.grid_base_y_to_grid_x_sign *
         (target_offset_m - current_offset_m);
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

SecondPreselectionOccupancyObservation evaluateSecondPreselectionGridOccupancy(
    const std::vector<rc26_vision::Detection> &detections,
    const SecondPreselectionParams &params, double odom_delta_x_m,
    double odom_delta_y_m) {
  SecondPreselectionOccupancyObservation result;
  const double distance_m = currentGridDistance(params, odom_delta_x_m);
  const double lateral_offset_m = currentGridLateralOffset(params, odom_delta_y_m);

  for (const int row : kGridRows) {
    for (const int col : kGridCols) {
      const int index = gridCellIndex(col, row);
      if (index < 0) {
        continue;
      }
      result.grid_cells[static_cast<std::size_t>(index)] =
          projectGridCell(col, row, params, distance_m, lateral_offset_m);
    }
  }

  for (const auto &detection : detections) {
    if (!labelAllowed(detection.class_name, params)) {
      continue;
    }
    const cv::Point2f center = detectionCenter(detection);
    for (std::size_t i = 0; i < result.grid_cells.size(); ++i) {
      if (!pointInRoi(center, result.grid_cells[i].roi)) {
        continue;
      }
      ++result.grid_detection_counts[i];
      result.grid_occupied_mask |= static_cast<uint16_t>(1U << i);
      ++result.matched_detections;
      if (result.first_label.empty()) {
        result.first_label = detection.class_name;
      }
      break;
    }
  }

  result.selected_middle_col =
      selectMiddleColumn(result.grid_occupied_mask, params);
  if (result.selected_middle_col.has_value()) {
    result.selected_lateral_m =
        selectedLateralMotion(params, *result.selected_middle_col, odom_delta_y_m);
    result.occupied = false;
  } else {
    result.selected_lateral_m = 0.0;
    result.occupied = true;
  }
  return result;
}

double secondPreselectionKfsApproachDistance(
    double locked_depth_m, const SecondPreselectionParams &params) {
  const double sign = params.kfs_approach_x_sign < 0 ? -1.0 : 1.0;
  return sign * std::max(0.0, locked_depth_m - params.kfs_grab_distance_m);
}

bool secondPreselectionHasFrontKfs(
    const SecondPreselectionOccupancyObservation &observation) {
  return observation.matched_detections > 0;
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
      return beginOdomApproach(*align_last_observation_);
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
      return beginOdomApproach(*observation);
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

void SecondPreselectionKfsPickupAction::beginPickupCommand() {
  publishStop();
  command_generation_.fetch_add(1, std::memory_order_relaxed);
  command_response_seen_ = false;
  command_accepted_ = false;
  pickup_done_feedback_seen_ = false;
  command_error_seen_ = false;
  command_busy_seen_ = false;
  command_seq_ = -1;
  command_error_detail_.clear();
  phase_ = Phase::SendingPickup;
  phase_tp_ = std::chrono::steady_clock::now();
  RCLCPP_INFO(node_->get_logger(),
              "第二预选赛准备发送 KFS 夹取触发命令：0x%02X，完成反馈=0x%02X",
              params_.pickup_command_id & 0xFF,
              params_.pickup_done_feedback_id & 0xFF);
}

BT::NodeStatus SecondPreselectionKfsPickupAction::tickSendingPickup() {
  renderKfsUi("pickup-send", align_last_observation_, "sending 0x12");
  publishStop();
  if (!sendPickupCommand()) {
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
                params_.pickup_command_id & 0xFF, seq,
                params_.pickup_done_feedback_id & 0xFF);
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
  if (pickup_done_feedback_seen_.load(std::memory_order_relaxed)) {
    RCLCPP_INFO(node_->get_logger(),
                "第二预选赛 KFS 夹取完成反馈已收到：feedback=0x%02X seq=%d，开始视觉验证",
                params_.pickup_done_feedback_id & 0xFF, seq);
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
                params_.pickup_done_feedback_id & 0xFF, seq,
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
          static_cast<uint8_t>(params_.pickup_done_feedback_id & 0xFF)) {
    pickup_done_feedback_seen_.store(true, std::memory_order_relaxed);
  }
}

bool SecondPreselectionKfsPickupAction::sendPickupCommand() {
  if (!send_client_ || !send_client_->service_is_ready()) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                         "第二预选赛等待 KFS 夹取命令服务：%s",
                         params_.send_command_service.c_str());
    return false;
  }
  auto request = std::make_shared<SendCommandSrv::Request>();
  request->command_id = clampByte(params_.pickup_command_id);
  request->payload = emptyPayload();
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
    command_error_detail_ = std::string("第二预选赛 KFS 夹取命令发送异常：") + e.what();
    command_response_seen_.store(true, std::memory_order_relaxed);
    command_accepted_.store(false, std::memory_order_relaxed);
  }
  RCLCPP_INFO(node_->get_logger(), "第二预选赛已发送 KFS 夹取触发命令：0x%02X",
              params_.pickup_command_id & 0xFF);
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
    return fail(grab_verify_seen_new_frame_
                    ? "第二预选赛 KFS 夹取验证未稳定消失"
                    : "第二预选赛 KFS 夹取验证没有新视觉帧");
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
  last_real_depth_m_ = 0.0;
  approach_distance_m_ = 0.0;
  approach_started_ = false;
  approach_start_captured_ = false;
  approach_stable_ticks_ = 0;
  approach_waiting_odom_logged_ = false;
  command_response_seen_ = false;
  command_accepted_ = false;
  pickup_done_feedback_seen_ = false;
  command_error_seen_ = false;
  command_busy_seen_ = false;
  command_seq_ = -1;
  command_error_detail_.clear();
  grab_verify_lost_count_ = 0;
  grab_verify_last_sequence_ = 0;
  grab_verify_seen_new_frame_ = false;
  grab_verify_visible_logged_ = false;
  grab_verify_last_logged_lost_count_ = 0;
}

SecondPreselectionObserveAction::SecondPreselectionObserveAction(
    const std::string &name, const BT::NodeConfig &config)
    : BT::StatefulActionNode(name, config) {}

SecondPreselectionObserveAction::~SecondPreselectionObserveAction() {
  releaseUi();
  releaseVision();
  releaseOdom();
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
  config().blackboard->set("second_preselection_last_observe_detection_count", 0);
  config().blackboard->set("second_preselection_grid_occupied_mask", 0);
  config().blackboard->set("second_preselection_selected_middle_col", 0);
  config().blackboard->set("second_preselection_selected_lateral_m", 0.0);

  occupied_stable_count_ = 0;
  ui_disabled_after_error_ = false;
  has_odom_ = false;
  odom_reference_ready_ = false;
  start_tp_ = std::chrono::steady_clock::now();
  last_log_tp_ = start_tp_;
  if (!setupOdom()) {
    config().blackboard->set("second_preselection_observe_error", true);
    return fail("第二预选赛动态 ROI odom 启动失败");
  }
  if (!setupVision()) {
    config().blackboard->set("second_preselection_observe_error", true);
    return fail("第二预选赛九宫格中层视觉启动失败");
  }

  RCLCPP_INFO(node_->get_logger(),
              "第二预选赛开始动态九宫格观察：team=%s D0=%.2f S0=%.2f cols=[%.2f,%.2f,%.2f] row_pitch=%.2f camera=[fx %.1f fy %.1f ppx %.1f ppy %.1f] odom=%s timeout=%.1fs label_stable=%d",
              team_.c_str(), params_.grid_initial_distance_m,
              params_.grid_initial_lateral_offset_m,
              params_.grid_left_col_width_m, params_.grid_center_col_width_m,
              params_.grid_right_col_width_m, params_.grid_row_pitch_m,
              params_.grid_camera_fx_px, params_.grid_camera_fy_px,
              params_.grid_camera_ppx_px, params_.grid_camera_ppy_px,
              params_.odom_topic.c_str(), params_.observe_timeout_s,
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
    if (!odomReady()) {
      if (odom_reference_ready_) {
        config().blackboard->set("second_preselection_observe_error", true);
        return fail("第二预选赛动态 ROI odom 超时");
      }
      if (elapsedSec(last_log_tp_) >= params_.observe_log_period_s) {
        RCLCPP_WARN(node_->get_logger(),
                    "第二预选赛动态 ROI 等待 odom：topic=%s elapsed=%.1fs",
                    params_.odom_topic.c_str(), elapsedSec(start_tp_));
        last_log_tp_ = std::chrono::steady_clock::now();
      }
      if (elapsedSec(start_tp_) > params_.observe_timeout_s) {
        config().blackboard->set("second_preselection_observe_error", true);
        return fail("第二预选赛动态 ROI 等待 odom 超时");
      }
      return BT::NodeStatus::RUNNING;
    }

    const double odom_delta_x_m = current_odom_x_ - start_odom_x_;
    const double odom_delta_y_m = current_odom_y_ - start_odom_y_;
    SecondPreselectionOccupancyObservation observation =
        evaluateSecondPreselectionGridOccupancy(
            snapshot.detections, params_, odom_delta_x_m, odom_delta_y_m);
    writeObservationToBlackboard(observation);
    renderObservationUi(snapshot, observation, odom_delta_x_m, odom_delta_y_m);

    if (observation.selected_middle_col) {
      config().blackboard->set("second_preselection_middle_empty", true);
      config().blackboard->set("second_preselection_middle_occupied", false);
      RCLCPP_INFO(node_->get_logger(),
                  "第二预选赛中层选位成功：col=%d lateral=%.3fm mask=0x%03X detections=%d",
                  *observation.selected_middle_col, observation.selected_lateral_m,
                  observation.grid_occupied_mask, observation.matched_detections);
      releaseVision();
      releaseOdom();
      return BT::NodeStatus::SUCCESS;
    }

    if (observation.occupied) {
      ++occupied_stable_count_;
    } else {
      config().blackboard->set("second_preselection_middle_empty", true);
      config().blackboard->set("second_preselection_middle_occupied", false);
      RCLCPP_INFO(node_->get_logger(),
                  "第二预选赛九宫格中层判定为空：ROI内无目标标签");
      releaseVision();
      releaseOdom();
      return BT::NodeStatus::SUCCESS;
    }
    if (occupied_stable_count_ >= params_.occupied_stable_frames) {
      config().blackboard->set("second_preselection_middle_empty", false);
      config().blackboard->set("second_preselection_middle_occupied", true);
      RCLCPP_INFO(node_->get_logger(),
                  "第二预选赛九宫格中层被目标标签占据：label=%s count=%d stable=%d/%d",
                  observation.first_label.c_str(), observation.matched_detections,
                  occupied_stable_count_,
                  params_.occupied_stable_frames);
      releaseVision();
      releaseOdom();
      return BT::NodeStatus::SUCCESS;
    }
    if (elapsedSec(last_log_tp_) >= params_.observe_log_period_s) {
      RCLCPP_INFO(node_->get_logger(),
                  "第二预选赛九宫格中层观察中：occupied=%s label=%s count=%d stable=%d/%d mask=0x%03X elapsed=%.1fs",
                  observation.occupied ? "true" : "false",
                  observation.first_label.empty() ? "-" : observation.first_label.c_str(),
                  observation.matched_detections, occupied_stable_count_,
                  params_.occupied_stable_frames, observation.grid_occupied_mask,
                  elapsedSec(start_tp_));
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
    releaseOdom();
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

void SecondPreselectionObserveAction::onHalted() {
  releaseUi();
  releaseVision();
  releaseOdom();
}

BT::NodeStatus SecondPreselectionObserveAction::fail(
    const std::string &reason) {
  if (node_) {
    RCLCPP_ERROR(node_->get_logger(), "第二预选赛视觉观察失败：%s",
                 reason.c_str());
  }
  writeDecisionFailure(config().blackboard, "SecondPreselectionObserve", reason);
  releaseUi();
  releaseVision();
  releaseOdom();
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
  releaseUi();
  if (vision_) {
    vision_->stop();
    vision_.reset();
  }
}

bool SecondPreselectionObserveAction::setupOdom() {
  odom_sub_ = node_->create_subscription<OdomMsg>(
      params_.odom_topic, rclcpp::QoS(rclcpp::KeepLast(10)),
      [this](const OdomMsg::SharedPtr msg) {
        if (!msg) {
          return;
        }
        current_odom_x_ = msg->pose.pose.position.x;
        current_odom_y_ = msg->pose.pose.position.y;
        has_odom_ = true;
        last_odom_tp_ = std::chrono::steady_clock::now();
        if (!odom_reference_ready_) {
          start_odom_x_ = current_odom_x_;
          start_odom_y_ = current_odom_y_;
          odom_reference_ready_ = true;
        }
      });
  return static_cast<bool>(odom_sub_);
}

void SecondPreselectionObserveAction::releaseOdom() {
  odom_sub_.reset();
}

bool SecondPreselectionObserveAction::odomReady() const {
  if (!has_odom_ || !odom_reference_ready_) {
    return false;
  }
  return elapsedSec(last_odom_tp_) <= params_.odom_timeout_s;
}

bool SecondPreselectionObserveAction::setupUiIfNeeded() {
  if (!params_.dynamic_roi_ui_enable || ui_disabled_after_error_) {
    return false;
  }
  if (ui_window_active_) {
    return true;
  }
  try {
    cv::namedWindow(params_.dynamic_roi_ui_window_name, cv::WINDOW_NORMAL);
    ui_window_active_ = true;
    return true;
  } catch (const cv::Exception &e) {
    ui_disabled_after_error_ = true;
    ui_window_active_ = false;
    if (node_) {
      RCLCPP_WARN(node_->get_logger(),
                  "第二预选赛动态 ROI UI 创建失败，自动关闭 UI：%s",
                  e.what());
    }
    return false;
  }
}

void SecondPreselectionObserveAction::releaseUi() {
  if (!ui_window_active_) {
    return;
  }
  try {
    cv::destroyWindow(params_.dynamic_roi_ui_window_name);
  } catch (const cv::Exception &e) {
    if (node_) {
      RCLCPP_WARN(node_->get_logger(),
                  "第二预选赛动态 ROI UI 关闭异常：%s", e.what());
    }
  }
  ui_window_active_ = false;
}

void SecondPreselectionObserveAction::renderObservationUi(
    const rc26_vision::VisionInferenceManager::FrameSnapshot &snapshot,
    const SecondPreselectionOccupancyObservation &observation,
    double odom_delta_x_m, double odom_delta_y_m) {
  if (!params_.dynamic_roi_ui_enable || ui_disabled_after_error_ ||
      !snapshot.has_color || snapshot.color_bgr.empty()) {
    return;
  }
  if (!setupUiIfNeeded()) {
    return;
  }

  try {
    cv::Mat canvas = snapshot.color_bgr.clone();
    const cv::Size frame_size = canvas.size();
    const cv::Scalar occupied_color(40, 40, 230);
    const cv::Scalar empty_color(170, 170, 170);
    const cv::Scalar selected_color(0, 220, 255);
    const cv::Scalar text_bg(20, 20, 20);

    for (int i = 0; i < static_cast<int>(observation.grid_cells.size()); ++i) {
      const auto &cell = observation.grid_cells[static_cast<size_t>(i)];
      const bool occupied =
          observation.grid_detection_counts[static_cast<size_t>(i)] > 0;
      const bool selected =
          observation.selected_middle_col.has_value() && cell.row == 0 &&
          *observation.selected_middle_col == cell.col;
      const cv::Scalar color =
          selected ? selected_color : (occupied ? occupied_color : empty_color);
      const int thickness = selected ? 3 : 2;
      if (const auto rect = clippedRect(cell.roi, frame_size)) {
        cv::rectangle(canvas, *rect, color, thickness);
        std::ostringstream label;
        label << "c" << cell.col << " r" << cell.row << " n"
              << observation.grid_detection_counts[static_cast<size_t>(i)];
        drawTextWithBackground(canvas, label.str(),
                               cv::Point(rect->x + 3, rect->y + 15),
                               cv::Scalar(255, 255, 255), text_bg);
      }
      if (cell.center.x >= 0.0F && cell.center.x < frame_size.width &&
          cell.center.y >= 0.0F && cell.center.y < frame_size.height) {
        const cv::Point center_i(static_cast<int>(std::lround(cell.center.x)),
                                 static_cast<int>(std::lround(cell.center.y)));
        cv::drawMarker(canvas, center_i, color, cv::MARKER_CROSS, 10, 1,
                       cv::LINE_AA);
      }
    }

    for (const auto &det : snapshot.detections) {
      const std::string label = rc26_vision::visualTargetLabel(det);
      const cv::Scalar color = detectionColor(label);
      const cv::Rect2f raw_box(det.x1, det.y1, det.x2 - det.x1,
                               det.y2 - det.y1);
      if (const auto box = clippedRect(raw_box, frame_size)) {
        cv::rectangle(canvas, *box, color, 2);
        const cv::Point2f center = detectionCenter(det);
        if (center.x >= 0.0F && center.x < frame_size.width &&
            center.y >= 0.0F && center.y < frame_size.height) {
          const cv::Point center_i(static_cast<int>(std::lround(center.x)),
                                   static_cast<int>(std::lround(center.y)));
          cv::circle(canvas, center_i, 3, color, cv::FILLED, cv::LINE_AA);
        }
        std::ostringstream det_text;
        det_text << (label.empty() ? "-" : label) << " "
                 << std::fixed << std::setprecision(2) << det.score;
        drawTextWithBackground(canvas, det_text.str(),
                               cv::Point(box->x, box->y - 4),
                               cv::Scalar(255, 255, 255), text_bg);
      }
    }

    std::ostringstream status1;
    status1 << "mask=0x" << std::uppercase << std::hex << std::setw(3)
            << std::setfill('0') << observation.grid_occupied_mask
            << std::dec << " det=" << observation.matched_detections;
    if (observation.selected_middle_col) {
      status1 << " selected_col=" << *observation.selected_middle_col
              << " lateral=" << std::fixed << std::setprecision(3)
              << observation.selected_lateral_m << "m";
    } else {
      status1 << " selected_col=none";
    }
    std::ostringstream status2;
    status2 << "odom_dx=" << std::fixed << std::setprecision(3)
            << odom_delta_x_m << "m odom_dy=" << odom_delta_y_m << "m";
    drawTextWithBackground(canvas, status1.str(), cv::Point(8, 20),
                           cv::Scalar(255, 255, 255), text_bg, 0.50, 1);
    drawTextWithBackground(canvas, status2.str(), cv::Point(8, 42),
                           cv::Scalar(255, 255, 255), text_bg, 0.50, 1);

    cv::imshow(params_.dynamic_roi_ui_window_name, canvas);
    cv::waitKey(1);
  } catch (const cv::Exception &e) {
    ui_disabled_after_error_ = true;
    if (node_) {
      RCLCPP_WARN(node_->get_logger(),
                  "第二预选赛动态 ROI UI 渲染失败，自动关闭 UI：%s",
                  e.what());
    }
    releaseUi();
  }
}

void SecondPreselectionObserveAction::writeObservationToBlackboard(
    const SecondPreselectionOccupancyObservation &observation) {
  if (!config().blackboard) {
    return;
  }
  config().blackboard->set("second_preselection_last_observe_detection_count",
                           observation.matched_detections);
  config().blackboard->set(
      "second_preselection_grid_occupied_mask",
      static_cast<int>(observation.grid_occupied_mask));
  config().blackboard->set(
      "second_preselection_selected_middle_col",
      observation.selected_middle_col.value_or(0));
  config().blackboard->set("second_preselection_selected_lateral_m",
                           observation.selected_lateral_m);
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
  p.place_kfs_command_id = node.declare_parameter<int>(
      "second_preselect_place_kfs_command_id", p.place_kfs_command_id);
  p.cmd_vel_topic = node.declare_parameter<std::string>(
      "second_preselect_cmd_vel_topic", p.cmd_vel_topic);

  p.nav_y1_m =
      node.declare_parameter<double>("second_preselect_nav_y1_m", p.nav_y1_m);
  p.nav_x2_m =
      node.declare_parameter<double>("second_preselect_nav_x2_m", p.nav_x2_m);
  p.place_forward_x_m = node.declare_parameter<double>(
      "second_preselect_place_forward_x_m", p.place_forward_x_m);
  p.retreat_x_m = node.declare_parameter<double>(
      "second_preselect_retreat_x_m", p.retreat_x_m);
  p.nav_timeout_s = node.declare_parameter<double>(
      "second_preselect_nav_timeout_s", p.nav_timeout_s);
  p.ramp_approach_x_m = node.declare_parameter<double>(
      "preselection_ramp_approach_x_m", p.ramp_approach_x_m);
  p.ramp_climb_x_m = node.declare_parameter<double>(
      "preselection_ramp_climb_x_m", p.ramp_climb_x_m);
  p.ramp_max_speed_mps = node.declare_parameter<double>(
      "preselection_ramp_max_speed_mps", p.ramp_max_speed_mps);
  p.ramp_min_speed_mps = node.declare_parameter<double>(
      "preselection_ramp_min_speed_mps", p.ramp_min_speed_mps);
  p.ramp_timeout_s = node.declare_parameter<double>(
      "preselection_ramp_timeout_s", p.ramp_timeout_s);
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
  p.grid_camera_fx_px = node.declare_parameter<double>(
      "second_preselect_grid_camera_fx_px", p.grid_camera_fx_px);
  p.grid_camera_fy_px = node.declare_parameter<double>(
      "second_preselect_grid_camera_fy_px", p.grid_camera_fy_px);
  p.grid_camera_ppx_px = node.declare_parameter<double>(
      "second_preselect_grid_camera_ppx_px", p.grid_camera_ppx_px);
  p.grid_camera_ppy_px = node.declare_parameter<double>(
      "second_preselect_grid_camera_ppy_px", p.grid_camera_ppy_px);
  p.grid_left_col_width_m = node.declare_parameter<double>(
      "second_preselect_grid_left_col_width_m", p.grid_left_col_width_m);
  p.grid_center_col_width_m = node.declare_parameter<double>(
      "second_preselect_grid_center_col_width_m", p.grid_center_col_width_m);
  p.grid_right_col_width_m = node.declare_parameter<double>(
      "second_preselect_grid_right_col_width_m", p.grid_right_col_width_m);
  p.grid_row_pitch_m = node.declare_parameter<double>(
      "second_preselect_grid_row_pitch_m", p.grid_row_pitch_m);
  p.grid_middle_center_height_m = node.declare_parameter<double>(
      "second_preselect_grid_middle_center_height_m",
      p.grid_middle_center_height_m);
  p.grid_safe_width_m = node.declare_parameter<double>(
      "second_preselect_grid_safe_width_m", p.grid_safe_width_m);
  p.grid_safe_height_m = node.declare_parameter<double>(
      "second_preselect_grid_safe_height_m", p.grid_safe_height_m);
  p.grid_camera_height_m = node.declare_parameter<double>(
      "second_preselect_grid_camera_height_m", p.grid_camera_height_m);
  p.grid_initial_distance_m = node.declare_parameter<double>(
      "second_preselect_grid_initial_distance_m",
      p.grid_initial_distance_m);
  p.grid_initial_lateral_offset_m = node.declare_parameter<double>(
      "second_preselect_grid_initial_lateral_offset_m",
      p.grid_initial_lateral_offset_m);
  p.grid_base_y_to_grid_x_sign = node.declare_parameter<double>(
      "second_preselect_grid_base_y_to_grid_x_sign",
      p.grid_base_y_to_grid_x_sign);
  p.grid_place_lateral_bias_m = node.declare_parameter<double>(
      "second_preselect_grid_place_lateral_bias_m",
      p.grid_place_lateral_bias_m);
  p.grid_label_prefixes = node.declare_parameter<std::vector<std::string>>(
      "second_preselect_grid_label_prefixes", p.grid_label_prefixes);
  p.grid_label_exact_names = node.declare_parameter<std::vector<std::string>>(
      "second_preselect_grid_label_exact_names", p.grid_label_exact_names);
  p.occupied_stable_frames = node.declare_parameter<int>(
      "second_preselect_occupied_stable_frames", p.occupied_stable_frames);
  p.observe_timeout_s = node.declare_parameter<double>(
      "second_preselect_observe_timeout_s", p.observe_timeout_s);
  p.observe_log_period_s = node.declare_parameter<double>(
      "second_preselect_observe_log_period_s", p.observe_log_period_s);
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
  p.nav_timeout_s = std::max(0.001, p.nav_timeout_s);
  p.ramp_max_speed_mps =
      (std::isfinite(p.ramp_max_speed_mps) && p.ramp_max_speed_mps > 0.0)
          ? std::abs(p.ramp_max_speed_mps)
          : SecondPreselectionParams{}.ramp_max_speed_mps;
  p.ramp_min_speed_mps =
      (std::isfinite(p.ramp_min_speed_mps) && p.ramp_min_speed_mps >= 0.0)
          ? std::min(std::abs(p.ramp_min_speed_mps), p.ramp_max_speed_mps)
          : SecondPreselectionParams{}.ramp_min_speed_mps;
  p.ramp_timeout_s = std::max(0.001, p.ramp_timeout_s);
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
  p.pickup_done_feedback_id = std::clamp(p.pickup_done_feedback_id, 0, 255);
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
  p.occupied_stable_frames = std::max(1, p.occupied_stable_frames);
  p.observe_timeout_s = std::max(0.001, p.observe_timeout_s);
  p.observe_log_period_s = std::max(0.1, p.observe_log_period_s);
  p.grid_camera_fx_px =
      (std::isfinite(p.grid_camera_fx_px) && p.grid_camera_fx_px > 0.0)
          ? p.grid_camera_fx_px
          : SecondPreselectionParams{}.grid_camera_fx_px;
  p.grid_camera_fy_px =
      (std::isfinite(p.grid_camera_fy_px) && p.grid_camera_fy_px > 0.0)
          ? p.grid_camera_fy_px
          : SecondPreselectionParams{}.grid_camera_fy_px;
  p.grid_camera_ppx_px =
      std::isfinite(p.grid_camera_ppx_px)
          ? p.grid_camera_ppx_px
          : SecondPreselectionParams{}.grid_camera_ppx_px;
  p.grid_camera_ppy_px =
      std::isfinite(p.grid_camera_ppy_px)
          ? p.grid_camera_ppy_px
          : SecondPreselectionParams{}.grid_camera_ppy_px;
  p.grid_left_col_width_m =
      (std::isfinite(p.grid_left_col_width_m) &&
       p.grid_left_col_width_m > 0.0)
          ? p.grid_left_col_width_m
          : SecondPreselectionParams{}.grid_left_col_width_m;
  p.grid_center_col_width_m =
      (std::isfinite(p.grid_center_col_width_m) &&
       p.grid_center_col_width_m > 0.0)
          ? p.grid_center_col_width_m
          : SecondPreselectionParams{}.grid_center_col_width_m;
  p.grid_right_col_width_m =
      (std::isfinite(p.grid_right_col_width_m) &&
       p.grid_right_col_width_m > 0.0)
          ? p.grid_right_col_width_m
          : SecondPreselectionParams{}.grid_right_col_width_m;
  p.grid_row_pitch_m =
      (std::isfinite(p.grid_row_pitch_m) && p.grid_row_pitch_m > 0.0)
          ? p.grid_row_pitch_m
          : SecondPreselectionParams{}.grid_row_pitch_m;
  p.grid_middle_center_height_m =
      std::isfinite(p.grid_middle_center_height_m)
          ? p.grid_middle_center_height_m
          : SecondPreselectionParams{}.grid_middle_center_height_m;
  p.grid_safe_width_m =
      (std::isfinite(p.grid_safe_width_m) && p.grid_safe_width_m > 0.0)
          ? p.grid_safe_width_m
          : SecondPreselectionParams{}.grid_safe_width_m;
  p.grid_safe_height_m =
      (std::isfinite(p.grid_safe_height_m) && p.grid_safe_height_m > 0.0)
          ? p.grid_safe_height_m
          : SecondPreselectionParams{}.grid_safe_height_m;
  p.grid_camera_height_m =
      std::isfinite(p.grid_camera_height_m)
          ? p.grid_camera_height_m
          : SecondPreselectionParams{}.grid_camera_height_m;
  p.grid_initial_distance_m =
      (std::isfinite(p.grid_initial_distance_m) &&
       p.grid_initial_distance_m > 0.05)
          ? p.grid_initial_distance_m
          : SecondPreselectionParams{}.grid_initial_distance_m;
  p.grid_initial_lateral_offset_m =
      std::isfinite(p.grid_initial_lateral_offset_m)
          ? p.grid_initial_lateral_offset_m
          : SecondPreselectionParams{}.grid_initial_lateral_offset_m;
  p.grid_base_y_to_grid_x_sign =
      (std::isfinite(p.grid_base_y_to_grid_x_sign) &&
       p.grid_base_y_to_grid_x_sign >= 0.0)
          ? 1.0
          : -1.0;
  p.nav_y1_m *= static_cast<double>(mirror_sign);
  p.grid_initial_lateral_offset_m *= static_cast<double>(mirror_sign);
  p.grid_base_y_to_grid_x_sign *= static_cast<double>(mirror_sign);
  p.grid_place_lateral_bias_m *= static_cast<double>(mirror_sign);
  p.grid_place_lateral_bias_m =
      std::isfinite(p.grid_place_lateral_bias_m)
          ? p.grid_place_lateral_bias_m
          : SecondPreselectionParams{}.grid_place_lateral_bias_m;
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
  blackboard->set("second_preselect_place_kfs_command_id",
                  p.place_kfs_command_id);
  blackboard->set("second_preselect_cmd_vel_topic", p.cmd_vel_topic);
  blackboard->set("second_preselect_nav_y1_m", p.nav_y1_m);
  blackboard->set("second_preselect_nav_x2_m", p.nav_x2_m);
  blackboard->set("second_preselect_place_forward_x_m", p.place_forward_x_m);
  blackboard->set("second_preselect_retreat_x_m", p.retreat_x_m);
  blackboard->set("second_preselect_nav_timeout_s", p.nav_timeout_s);
  blackboard->set("preselection_ramp_approach_x_m", p.ramp_approach_x_m);
  blackboard->set("preselection_ramp_climb_x_m", p.ramp_climb_x_m);
  blackboard->set("preselection_ramp_max_speed_mps", p.ramp_max_speed_mps);
  blackboard->set("preselection_ramp_min_speed_mps", p.ramp_min_speed_mps);
  blackboard->set("preselection_ramp_timeout_s", p.ramp_timeout_s);
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
  blackboard->set("second_preselection_middle_empty", false);
  blackboard->set("second_preselection_middle_occupied", false);
  blackboard->set("second_preselection_observe_error", false);
  blackboard->set("second_preselection_last_observe_detection_count", 0);
  blackboard->set("second_preselection_grid_occupied_mask", 0);
  blackboard->set("second_preselection_selected_middle_col", 0);
  blackboard->set("second_preselection_selected_lateral_m", 0.0);

  RCLCPP_INFO(node.get_logger(),
              "第二预选赛参数已加载: mirror_sign=%d start=0x%02X/done=0x%02X pickup=0x%02X/done=0x%02X place=0x%02X search=[speed %.2f timeout %.1f] nav=[y1 %.2f, x2 %.2f, place %.2f, retreat %.2f] ramp=[approach %.2f climb %.2f max %.2f min %.2f timeout %.1f turn %.2f timeout %.1f] D0=%.2f S0=%.2f base_y_to_grid_x=%.1f camera=[fx %.1f fy %.1f ppx %.1f ppy %.1f] cols=[%.2f,%.2f,%.2f] row_pitch=%.2f safe=[%.2f,%.2f] odom=%s label_stable=%d",
              mirror_sign,
              p.start_command_id & 0xFF, p.start_done_feedback_id & 0xFF,
              p.pickup_command_id & 0xFF,
              p.pickup_done_feedback_id & 0xFF,
              p.place_kfs_command_id & 0xFF,
              p.search_forward_speed_mps, p.search_timeout_s,
              p.nav_y1_m, p.nav_x2_m, p.place_forward_x_m, p.retreat_x_m,
              p.ramp_approach_x_m, p.ramp_climb_x_m,
              p.ramp_max_speed_mps, p.ramp_min_speed_mps,
              p.ramp_timeout_s, p.after_ramp_turn_delta_rad,
              p.after_ramp_turn_timeout_s,
              p.grid_initial_distance_m, p.grid_initial_lateral_offset_m,
              p.grid_base_y_to_grid_x_sign, p.grid_camera_fx_px,
              p.grid_camera_fy_px, p.grid_camera_ppx_px,
              p.grid_camera_ppy_px, p.grid_left_col_width_m,
              p.grid_center_col_width_m, p.grid_right_col_width_m,
              p.grid_row_pitch_m, p.grid_safe_width_m, p.grid_safe_height_m,
              p.odom_topic.c_str(), p.occupied_stable_frames);
}

void registerSecondPreselectionNodes(BT::BehaviorTreeFactory &factory) {
  factory.registerNodeType<SecondPreselectionCommandAction>(
      "SecondPreselectionCommand");
  factory.registerNodeType<SecondPreselectionKfsPickupAction>(
      "SecondPreselectionKfsPickup");
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
      "第二预选赛动态九宫格观察未找到中层空位，停止放置";
  if (node) {
    RCLCPP_ERROR(node->get_logger(), "%s", reason.c_str());
  }
  writeDecisionFailure(config().blackboard, "SecondPreselection", reason);
  return BT::NodeStatus::FAILURE;
}

} // namespace rc26_decision
