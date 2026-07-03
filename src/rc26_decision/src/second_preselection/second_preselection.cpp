#include "rc26_decision/second_preselection/second_preselection.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <optional>
#include <sstream>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/core.hpp>

#include "rc26_decision/decision_failure.hpp"
#include "rc26_decision/team_color.hpp"
#include "rc26_serial/protocol.hpp"
#include "rc26_vision/inference/config/model_profile_loader.hpp"

namespace rc26_decision {

namespace {

constexpr std::array<int, 3> kGridCols{-1, 0, 1};
constexpr std::array<int, 3> kGridRows{-1, 0, 1};

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

    SecondPreselectionOccupancyObservation observation =
        evaluateSecondPreselectionGridOccupancy(
            snapshot.detections, params_, current_odom_x_ - start_odom_x_,
            current_odom_y_ - start_odom_y_);
    writeObservationToBlackboard(observation);

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
  p.odom_topic = node.declare_parameter<std::string>(
      "second_preselect_odom_topic", p.odom_topic);
  p.odom_timeout_s = node.declare_parameter<double>(
      "second_preselect_odom_timeout_s", p.odom_timeout_s);

  p.command_timeout_s = std::max(0.001, p.command_timeout_s);
  p.done_timeout_s = std::max(0.001, p.done_timeout_s);
  p.log_period_s = std::max(0.1, p.log_period_s);
  p.nav_timeout_s = std::max(0.001, p.nav_timeout_s);
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
  blackboard->set("second_preselect_arm_high_raise_command_id",
                  p.arm_high_raise_command_id);
  blackboard->set("second_preselect_arm_high_raise_done_feedback_id",
                  p.arm_high_raise_done_feedback_id);
  blackboard->set("second_preselect_place_kfs_command_id",
                  p.place_kfs_command_id);
  blackboard->set("second_preselect_nav_x1_m", p.nav_x1_m);
  blackboard->set("second_preselect_nav_y1_m", p.nav_y1_m);
  blackboard->set("second_preselect_nav_x2_m", p.nav_x2_m);
  blackboard->set("second_preselect_place_forward_x_m", p.place_forward_x_m);
  blackboard->set("second_preselect_retreat_x_m", p.retreat_x_m);
  blackboard->set("second_preselect_nav_timeout_s", p.nav_timeout_s);
  blackboard->set("second_preselect_odom_topic", p.odom_topic);
  blackboard->set("second_preselect_odom_timeout_s", p.odom_timeout_s);
  blackboard->set("second_preselection_middle_empty", false);
  blackboard->set("second_preselection_middle_occupied", false);
  blackboard->set("second_preselection_observe_error", false);
  blackboard->set("second_preselection_last_observe_detection_count", 0);
  blackboard->set("second_preselection_grid_occupied_mask", 0);
  blackboard->set("second_preselection_selected_middle_col", 0);
  blackboard->set("second_preselection_selected_lateral_m", 0.0);

  RCLCPP_INFO(node.get_logger(),
              "第二预选赛参数已加载: mirror_sign=%d start=0x%02X/done=0x%02X high=0x%02X/done=0x%02X place=0x%02X nav=[x1 %.2f, y1 %.2f, x2 %.2f, place %.2f, retreat %.2f] D0=%.2f S0=%.2f base_y_to_grid_x=%.1f camera=[fx %.1f fy %.1f ppx %.1f ppy %.1f] cols=[%.2f,%.2f,%.2f] row_pitch=%.2f safe=[%.2f,%.2f] odom=%s label_stable=%d",
              mirror_sign,
              p.start_command_id & 0xFF, p.start_done_feedback_id & 0xFF,
              p.arm_high_raise_command_id & 0xFF,
              p.arm_high_raise_done_feedback_id & 0xFF,
              p.place_kfs_command_id & 0xFF, p.nav_x1_m, p.nav_y1_m,
              p.nav_x2_m, p.place_forward_x_m, p.retreat_x_m,
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
