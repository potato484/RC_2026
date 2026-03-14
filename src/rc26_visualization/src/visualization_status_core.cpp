#include "rc26_visualization/visualization_status_core.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <initializer_list>
#include <limits>
#include <sstream>
#include <utility>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

namespace rc26_visualization {

namespace {

std::string formatDouble(double value, int precision = 2) {
  if (!std::isfinite(value)) {
    return "nan";
  }
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(precision) << value;
  return stream.str();
}

std::string formatSec(double value) {
  return formatDouble(value, 2) + " s";
}

std::string formatMs(double value, int precision = 1) {
  return formatDouble(value, precision) + " ms";
}

std::string formatMeters(double value, int precision = 2) {
  return formatDouble(value, precision) + " m";
}

uint8_t maxLevel(uint8_t left, uint8_t right) {
  return std::max(left, right);
}

uint8_t severityForHighValue(double value, double yellow, double orange, double red) {
  if (!std::isfinite(value)) {
    return kLevelGreen;
  }
  if (value > red) {
    return kLevelRed;
  }
  if (value > orange) {
    return kLevelOrange;
  }
  if (value > yellow) {
    return kLevelYellow;
  }
  return kLevelGreen;
}

uint8_t severityForLowValue(double value, double yellow, double orange, double red) {
  if (!std::isfinite(value)) {
    return kLevelGreen;
  }
  if (value < red) {
    return kLevelRed;
  }
  if (value < orange) {
    return kLevelOrange;
  }
  if (value < yellow) {
    return kLevelYellow;
  }
  return kLevelGreen;
}

uint8_t diagLevel(uint8_t level) {
  if (level >= kLevelRed) {
    return diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  }
  if (level >= kLevelYellow) {
    return diagnostic_msgs::msg::DiagnosticStatus::WARN;
  }
  return diagnostic_msgs::msg::DiagnosticStatus::OK;
}

std::string joinStrings(const std::vector<std::string>& values) {
  if (values.empty()) {
    return "";
  }
  std::ostringstream stream;
  for (size_t index = 0; index < values.size(); ++index) {
    if (index > 0) {
      stream << ", ";
    }
    stream << values[index];
  }
  return stream.str();
}

std::string joinCsv(const std::vector<std::string>& values) {
  if (values.empty()) {
    return "";
  }
  std::ostringstream stream;
  for (size_t index = 0; index < values.size(); ++index) {
    if (index > 0) {
      stream << ",";
    }
    stream << values[index];
  }
  return stream.str();
}

std::string boolString(bool value) {
  return value ? "true" : "false";
}

std::string trimCopy(std::string_view value) {
  const auto first = std::find_if_not(value.begin(), value.end(), [](unsigned char ch) {
    return std::isspace(ch) != 0;
  });
  if (first == value.end()) {
    return "";
  }
  const auto last = std::find_if_not(value.rbegin(), value.rend(), [](unsigned char ch) {
    return std::isspace(ch) != 0;
  }).base();
  return std::string(first, last);
}

std::string lowerCopy(std::string_view value) {
  std::string normalized;
  normalized.reserve(value.size());
  for (const unsigned char ch : value) {
    normalized.push_back(static_cast<char>(std::tolower(ch)));
  }
  return normalized;
}

bool isUnknownSource(std::string_view value) {
  const std::string normalized = lowerCopy(trimCopy(value));
  return normalized == "unknown" || normalized == "unknown source";
}

std::string normalizeSourceSignal(const std::string& source_signal) {
  const std::string trimmed = trimCopy(source_signal);
  if (trimmed.empty()) {
    return "system_internal";
  }
  if (isUnknownSource(trimmed)) {
    return "unnamed_module";
  }
  return trimmed;
}

int64_t headerTimeMs(const std_msgs::msg::Header& header) {
  return static_cast<int64_t>(header.stamp.sec) * 1000LL + static_cast<int64_t>(header.stamp.nanosec / 1000000U);
}

double latestAgeSec(std::initializer_list<double> ages) {
  double best = kInf;
  for (double age : ages) {
    if (std::isfinite(age)) {
      best = std::min(best, age);
    }
  }
  return best;
}

std::string formatLastUpdateMs(const std_msgs::msg::Header& header, double age_sec) {
  if (!std::isfinite(age_sec)) {
    return "";
  }
  const int64_t delta_ms = static_cast<int64_t>(std::llround(age_sec * 1000.0));
  return std::to_string(std::max<int64_t>(0, headerTimeMs(header) - delta_ms));
}

struct StatusMetadata {
  bool present{true};
  bool enabled{true};
  bool received{false};
  std::string last_update_ms;
  std::vector<std::string> source_topics;
};

void addKv(diagnostic_msgs::msg::DiagnosticStatus& status, const std::string& key, const std::string& value) {
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  status.values.push_back(std::move(item));
}

diagnostic_msgs::msg::DiagnosticStatus makeStatus(
    const std::string& name, uint8_t level, const std::string& message,
    const StatusMetadata& metadata,
    const std::vector<std::pair<std::string, std::string>>& values) {
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = name;
  status.hardware_id = "R2";
  status.level = diagLevel(level);
  status.message = message;
  addKv(status, "present", boolString(metadata.present));
  addKv(status, "enabled", boolString(metadata.enabled));
  addKv(status, "received", boolString(metadata.received));
  addKv(status, "last_update_ms", metadata.last_update_ms);
  addKv(status, "source_topics", joinCsv(metadata.source_topics));
  for (const auto& [key, value] : values) {
    addKv(status, key, value);
  }
  return status;
}

rc26_interfaces::msg::VisualizationEvent makeEvent(
    const std::string& code, uint8_t severity, const std::string& title, const std::string& detail,
    const std::string& source_signal, const std::string& recommendation, bool latched) {
  rc26_interfaces::msg::VisualizationEvent event;
  event.code = code;
  event.severity = severity;
  event.title = title;
  event.detail = detail;
  event.source_signal = normalizeSourceSignal(source_signal);
  event.recommendation = recommendation;
  event.active = true;
  event.latched = latched;
  return event;
}

std::string staleReason(const std::string& topic_name, double age_sec, double max_age_sec) {
  if (!std::isfinite(age_sec)) {
    return "waiting " + topic_name;
  }
  return topic_name + " stale: age=" + formatSec(age_sec) + ", limit=" + formatSec(max_age_sec);
}

}  // namespace

VisualizationStatusCore::VisualizationStatusCore(VisualizationStatusConfig config)
    : config_(std::move(config)) {}

void VisualizationStatusCore::setConfig(const VisualizationStatusConfig& config) {
  config_ = config;
}

uint32_t TopicTimeoutTracker::observe(const std::vector<TopicWatchInput>& monitored_topics) {
  for (const auto& topic : monitored_topics) {
    const std::string key = topicKey(topic);
    const bool stale = isStale(topic);
    const bool previous = stale_states_[key];
    if (stale && !previous) {
      ++total_;
    }
    stale_states_[key] = stale;
  }
  return total_;
}

void TopicTimeoutTracker::resetBaseline(const std::vector<TopicWatchInput>& monitored_topics) {
  total_ = 0U;
  stale_states_.clear();
  for (const auto& topic : monitored_topics) {
    stale_states_[topicKey(topic)] = isStale(topic);
  }
}

bool TopicTimeoutTracker::isStale(const TopicWatchInput& topic) {
  return topic.required && (!topic.received || topic.age_sec > topic.max_age_sec);
}

std::string TopicTimeoutTracker::topicKey(const TopicWatchInput& topic) {
  return topic.code_suffix.empty() ? topic.topic_name : topic.code_suffix;
}

VisualizationStatusCore::Output VisualizationStatusCore::evaluate(
    const EvaluationInput& input, const std_msgs::msg::Header& header) const {
  const std::string disabledReason = "module disabled in current mode";
  Output output;
  output.summary.header = header;
  output.operator_status.header = header;
  output.events.header = header;

  auto& status = output.operator_status;
  status.overall_level = kLevelGreen;
  status.overall_reason = "nominal";
  status.localization_level =
      config_.localization_present
          ? (input.localization.received ? std::min<uint8_t>(input.localization.level, kLevelRed) : kLevelRed)
          : kLevelGreen;
  status.localization_reason = !config_.localization_present
                                   ? disabledReason
                                   : (input.localization.received ? input.localization.reason : "waiting localization health");
  status.localization_state =
      !config_.localization_present ? "DISABLED" : (input.localization.received ? input.localization.state : "UNKNOWN");
  status.controller_level = kLevelGreen;
  status.controller_mode =
      !config_.controller_present ? "disabled" : (input.controller_mode.received ? input.controller_mode.value : "unknown");
  status.control_degraded = config_.controller_present &&
                            (input.control_degraded.received ? input.control_degraded.value : input.localization.control_degraded);
  status.control_degenerate_score = config_.controller_present && input.control_degenerate_score.received
                                        ? input.control_degenerate_score.value
                                        : std::numeric_limits<double>::quiet_NaN();
  status.compute_time_ms = config_.controller_present && input.compute_time_ms.received
                               ? input.compute_time_ms.value
                               : std::numeric_limits<double>::quiet_NaN();
  status.pose_age_ms = config_.controller_present && input.pose_age_ms.received ? input.pose_age_ms.value
                                                                                : std::numeric_limits<double>::quiet_NaN();
  status.collision_d_min = config_.controller_present && input.collision_d_min.received
                               ? input.collision_d_min.value
                               : std::numeric_limits<double>::quiet_NaN();
  status.keepout_level = kLevelGreen;
  status.keepout_ready = false;
  status.keepout_reason = config_.keepout_present ? "waiting keepout inputs" : disabledReason;
  status.terrain_level = kLevelGreen;
  status.terrain_online = false;
  status.terrain_obstacle_active = config_.terrain_present && input.terrain.obstacles_active;
  status.terrain_drop_active = config_.terrain_present && input.terrain.drop_active;
  status.terrain_traversability_min = config_.terrain_present
                                          ? static_cast<float>(input.terrain.traversability_min)
                                          : std::numeric_limits<float>::quiet_NaN();
  status.terrain_speed_limited = config_.terrain_present && input.terrain.speed_limited;
  status.terrain_climbable_active = config_.terrain_present && input.terrain.climbable_active;
  status.terrain_step_edge_active = config_.terrain_present && input.terrain.step_edge_active;
  status.nav_safety_level = kLevelGreen;
  status.nav_profile =
      !config_.nav_safety_present ? "disabled" : (input.nav_safety.received ? input.nav_safety.current_profile : "unknown");
  status.nav_stop_required = config_.nav_safety_present && input.nav_safety.stop_required;
  status.nav_timed_out = config_.nav_safety_present && input.nav_safety.timed_out;
  status.mechanism_level = kLevelGreen;
  status.mechanism_comm_health = config_.mechanism_present ? input.mechanism.comm_health_level : 0U;
  status.topic_timeout_count = 0U;

  std::vector<rc26_interfaces::msg::VisualizationEvent> events;
  std::vector<std::pair<uint8_t, std::string>> reasons;
  if (config_.localization_present) {
    reasons.emplace_back(status.localization_level, status.localization_reason);
  }

  const double pose_age_red_ms = config_.loc_timeout_sec * 1000.0;
  const double pose_age_orange_ms = pose_age_red_ms * 0.8;
  const double pose_age_yellow_ms = pose_age_red_ms * 0.5;
  const uint8_t pose_age_level = input.pose_age_ms.received
                                     ? severityForHighValue(input.pose_age_ms.value, pose_age_yellow_ms,
                                                            pose_age_orange_ms, pose_age_red_ms)
                                     : kLevelGreen;

  const double control_time_red_ms = config_.controller_period_ms;
  const double control_time_orange_ms = control_time_red_ms * 0.8;
  const double control_time_yellow_ms = control_time_red_ms * 0.5;
  const uint8_t control_time_level = input.compute_time_ms.received
                                         ? severityForHighValue(input.compute_time_ms.value, control_time_yellow_ms,
                                                                control_time_orange_ms, control_time_red_ms)
                                         : kLevelGreen;

  const double obstacle_yellow = config_.brake_margin_m * 3.0;
  const double obstacle_orange = config_.brake_margin_m * 2.0;
  const double obstacle_red = config_.brake_margin_m;
  const uint8_t obstacle_level = input.collision_d_min.received
                                     ? severityForLowValue(input.collision_d_min.value, obstacle_yellow,
                                                           obstacle_orange, obstacle_red)
                                     : kLevelGreen;

  const uint8_t control_degraded_level = status.control_degraded ? kLevelOrange : kLevelGreen;
  if (config_.controller_present) {
    status.controller_level = std::max({pose_age_level, control_time_level, obstacle_level, control_degraded_level});
    reasons.emplace_back(status.controller_level,
                         status.controller_level == kLevelGreen ? "controller nominal"
                                                                : "controller attention required");
  } else {
    status.controller_level = kLevelGreen;
  }

  uint8_t backend_level = kLevelGreen;
  std::string backend_reason = config_.localization_present ? "backend nominal" : disabledReason;
  if (config_.localization_present && input.backend.received) {
    if (input.backend.age_sec > (config_.backend_status_max_age_ms / 1000.0)) {
      backend_level = kLevelYellow;
      backend_reason = staleReason(config_.localization_backend_topic, input.backend.age_sec,
                                   config_.backend_status_max_age_ms / 1000.0);
    }
    if (!input.backend.optimizer_ready) {
      backend_level = maxLevel(backend_level, kLevelOrange);
      backend_reason = "optimizer not ready";
    }
    if (input.backend.imu_spike) {
      backend_level = maxLevel(backend_level, kLevelOrange);
      backend_reason = "imu spike active";
    }
    if (std::isfinite(input.backend.graph_health)) {
      if (input.backend.graph_health < config_.backend_graph_health_error) {
        backend_level = maxLevel(backend_level, kLevelOrange);
        backend_reason = "graph health low";
      } else if (input.backend.graph_health < config_.backend_graph_health_warn) {
        backend_level = maxLevel(backend_level, kLevelYellow);
        backend_reason = "graph health warning";
      }
    }
    if (std::isfinite(input.backend.last_local_reg_age_sec)) {
      if (input.backend.last_local_reg_age_sec > config_.backend_local_reg_error_sec) {
        backend_level = maxLevel(backend_level, kLevelOrange);
        backend_reason = "local registration age high";
      } else if (input.backend.last_local_reg_age_sec > config_.backend_local_reg_warn_sec) {
        backend_level = maxLevel(backend_level, kLevelYellow);
        backend_reason = "local registration age warning";
      }
    }
  }

  if (config_.localization_present && status.localization_level >= kLevelYellow) {
    std::ostringstream detail;
    detail << "localization_state=" << status.localization_state
           << "; reason=" << status.localization_reason
           << "; sigma_xy=" << formatDouble(input.localization.sigma_xy, 3)
           << "; sigma_yaw=" << formatDouble(input.localization.sigma_yaw, 3)
           << "; h_min_eig=" << formatDouble(input.localization.h_min_eig, 2)
           << "; degenerate_score=" << formatDouble(input.localization.degenerate_score, 3);
    events.push_back(makeEvent(
        "LOCALIZATION_DEGRADED", status.localization_level, "定位退化", detail.str(),
        config_.localization_health_topic, "确认定位输入、重定位状态和环境可观测性，必要时切人工接管。", true));
  }

  if (config_.localization_present && backend_level >= kLevelYellow) {
    std::ostringstream detail;
    detail << "backend_reason=" << backend_reason
           << "; optimizer_state=" << input.backend.optimizer_state
           << "; graph_health=" << formatDouble(input.backend.graph_health, 2)
           << " (warn<" << formatDouble(config_.backend_graph_health_warn, 2)
           << ", error<" << formatDouble(config_.backend_graph_health_error, 2) << ")"
           << "; last_local_reg_age_sec=" << formatSec(input.backend.last_local_reg_age_sec)
           << " (warn>" << formatSec(config_.backend_local_reg_warn_sec)
           << ", error>" << formatSec(config_.backend_local_reg_error_sec) << ")"
           << "; last_loop_age_sec=" << formatSec(input.backend.last_loop_age_sec)
           << "; last_anchor_age_sec=" << formatSec(input.backend.last_anchor_age_sec)
           << "; status_age_sec=" << formatSec(input.backend.age_sec)
           << " (limit " << formatSec(config_.backend_status_max_age_ms / 1000.0) << ")"
           << "; imu_spike=" << (input.backend.imu_spike ? "true" : "false");
    events.push_back(makeEvent(
        "LOCALIZATION_BACKEND_WARN", backend_level, "定位后端告警", detail.str(),
        config_.localization_backend_topic, "检查图优化器、IMU 异常与回环/锚点输入，避免在异常状态下持续自动运行。", false));
    reasons.emplace_back(backend_level, backend_reason);
  }

  if (config_.controller_present && status.control_degraded) {
    std::ostringstream detail;
    detail << "control_degraded=true"
           << "; degenerate_score=" << formatDouble(status.control_degenerate_score, 3)
           << "; control_degraded_age_sec=" << formatSec(input.control_degraded.age_sec)
           << "; degenerate_score_age_sec=" << formatSec(input.control_degenerate_score.age_sec);
    events.push_back(makeEvent(
        "CONTROL_DEGRADED", kLevelOrange, "控制退化", detail.str(), config_.control_degraded_topic,
        "降低速度并检查预测里程计与退化分数来源；必要时切换保底视图确认轨迹。", true));
  }

  if (config_.controller_present && pose_age_level >= kLevelYellow) {
    std::ostringstream detail;
    detail << "pose_age_ms=" << formatMs(status.pose_age_ms, 1)
           << "; yellow=" << formatMs(pose_age_yellow_ms, 1)
           << "; orange=" << formatMs(pose_age_orange_ms, 1)
           << "; red=" << formatMs(pose_age_red_ms, 1);
    events.push_back(makeEvent(
        "POSE_STALE", pose_age_level, "位姿时效下降", detail.str(), config_.pose_age_ms_topic,
        "检查定位链路刷新率与控制输入时间戳，避免继续高速运动。", false));
  }

  if (config_.controller_present && control_time_level >= kLevelYellow) {
    std::ostringstream detail;
    detail << "compute_time_ms=" << formatMs(status.compute_time_ms, 1)
           << "; yellow=" << formatMs(control_time_yellow_ms, 1)
           << "; orange=" << formatMs(control_time_orange_ms, 1)
           << "; red=" << formatMs(control_time_red_ms, 1);
    events.push_back(makeEvent(
        "CONTROL_OVERRUN", control_time_level, "控制周期超限", detail.str(), config_.compute_time_ms_topic,
        "降低任务负载并检查控制器实时性，必要时暂停自动导航。", false));
  }

  if (config_.controller_present && obstacle_level >= kLevelYellow) {
    std::ostringstream detail;
    detail << "collision_d_min=" << formatMeters(status.collision_d_min, 2)
           << "; yellow_limit=" << formatMeters(obstacle_yellow, 2)
           << "; orange_limit=" << formatMeters(obstacle_orange, 2)
           << "; red_limit=" << formatMeters(obstacle_red, 2);
    events.push_back(makeEvent(
        "OBSTACLE_NEAR", obstacle_level, "近障碍风险", detail.str(), config_.collision_d_min_topic,
        "确认局部代价图和障碍物来源，必要时减速或人工接管。", false));
  }

  const double keepout_limit_sec = config_.keepout_max_age_ms / 1000.0;
  const bool filter_fresh = input.keepout.filter_info_received && input.keepout.filter_info_age_sec <= keepout_limit_sec;
  const bool mask_fresh = input.keepout.mask_received && input.keepout.mask_age_sec <= keepout_limit_sec;
  const bool heartbeat_fresh =
      input.keepout.heartbeat_received && input.keepout.heartbeat_age_sec <= keepout_limit_sec;
  const double keepout_ratio = std::max(
      input.keepout.filter_info_received && keepout_limit_sec > 0.0 ? input.keepout.filter_info_age_sec / keepout_limit_sec : 0.0,
      std::max(input.keepout.mask_received && keepout_limit_sec > 0.0 ? input.keepout.mask_age_sec / keepout_limit_sec : 0.0,
               input.keepout.heartbeat_received && keepout_limit_sec > 0.0 ? input.keepout.heartbeat_age_sec / keepout_limit_sec : 0.0));
  if (!config_.keepout_present) {
    status.keepout_level = kLevelGreen;
    status.keepout_reason = disabledReason;
    status.keepout_ready = false;
  } else {
    if (!input.keepout.filter_info_received) {
      status.keepout_level = kLevelRed;
      status.keepout_reason = "waiting /costmap_filter_info";
    } else if (!input.keepout.mask_received && !input.keepout.heartbeat_received) {
      status.keepout_level = kLevelRed;
      status.keepout_reason = "waiting keepout mask or heartbeat";
    } else if (input.keepout.heartbeat_received && !input.keepout.heartbeat_enabled) {
      status.keepout_level = kLevelRed;
      status.keepout_reason = "keepout heartbeat disabled";
    } else if (!filter_fresh || (!mask_fresh && !heartbeat_fresh)) {
      status.keepout_level = kLevelRed;
      status.keepout_reason = "keepout inputs stale";
    } else if (keepout_ratio > 0.8) {
      status.keepout_level = kLevelOrange;
      status.keepout_reason = "keepout inputs approaching stale threshold";
    } else if (keepout_ratio > 0.5) {
      status.keepout_level = kLevelYellow;
      status.keepout_reason = "keepout inputs aging";
    } else {
      status.keepout_level = kLevelGreen;
      status.keepout_reason = "keepout ready";
    }
    status.keepout_ready = status.keepout_level != kLevelRed;
  }
  if (config_.keepout_present) {
    reasons.emplace_back(status.keepout_level, status.keepout_reason);
  }
  if (config_.keepout_present && status.keepout_level >= kLevelYellow) {
    std::ostringstream detail;
    detail << "keepout_reason=" << status.keepout_reason
           << "; filter_age_sec=" << formatSec(input.keepout.filter_info_age_sec)
           << "; mask_age_sec=" << formatSec(input.keepout.mask_age_sec)
           << "; heartbeat_age_sec=" << formatSec(input.keepout.heartbeat_age_sec)
           << "; max_age_sec=" << formatSec(keepout_limit_sec)
           << "; heartbeat_enabled=" << (input.keepout.heartbeat_enabled ? "true" : "false");
    events.push_back(makeEvent(
        "KEEPOUT_STALE", status.keepout_level, "Keepout 失效风险", detail.str(),
        config_.costmap_filter_info_topic + "," + config_.kfs_filter_mask_topic + "," + config_.kfs_heartbeat_topic,
        "检查 keepout 掩码、元数据和心跳链路，确认防区仍在更新。", true));
  }

  const double terrain_limit_sec = config_.terrain_max_age_ms / 1000.0;
  const bool obstacles_fresh = input.terrain.obstacles_received && input.terrain.obstacles_age_sec <= terrain_limit_sec;
  const bool drop_fresh = input.terrain.drop_received && input.terrain.drop_age_sec <= terrain_limit_sec;
  const bool grid_fresh = input.terrain.grid_received && input.terrain.grid_age_sec <= terrain_limit_sec;
  const bool speed_limit_fresh =
      input.terrain.speed_limit_received && input.terrain.speed_limit_age_sec <= terrain_limit_sec;
  status.terrain_online = config_.terrain_present && (obstacles_fresh || drop_fresh || grid_fresh || speed_limit_fresh);
  const bool traversability_low =
      std::isfinite(input.terrain.traversability_min) && input.terrain.traversability_min <= 0.6;
  const bool terrain_hazard_active =
      input.terrain.obstacles_active || input.terrain.drop_active ||
      input.terrain.climbable_active || input.terrain.step_edge_active ||
      input.terrain.speed_limited || traversability_low;
  if (!config_.terrain_present) {
    status.terrain_level = kLevelGreen;
  } else if (!input.terrain.obstacles_received && !input.terrain.drop_received &&
             !input.terrain.grid_received && !input.terrain.speed_limit_received) {
    status.terrain_level = kLevelOrange;
  } else if (!status.terrain_online) {
    status.terrain_level = kLevelOrange;
  } else if (terrain_hazard_active) {
    status.terrain_level = kLevelYellow;
  } else {
    status.terrain_level = kLevelGreen;
  }
  if (config_.terrain_present) {
    reasons.emplace_back(status.terrain_level,
                         status.terrain_level == kLevelGreen ? "terrain nominal"
                                                            : (!status.terrain_online ? "terrain offline/stale"
                                                                                      : "terrain hazard detected"));
  }

  const double nav_limit_sec = config_.nav_safety_max_age_ms / 1000.0;
  if (!config_.nav_safety_present) {
    status.nav_safety_level = kLevelGreen;
  } else if (!input.nav_safety.received) {
    status.nav_safety_level = kLevelOrange;
    reasons.emplace_back(status.nav_safety_level, "waiting nav safety state");
  } else if (input.nav_safety.age_sec > nav_limit_sec) {
    status.nav_safety_level = kLevelOrange;
    reasons.emplace_back(status.nav_safety_level, staleReason(config_.nav_safety_topic, input.nav_safety.age_sec, nav_limit_sec));
  } else if (input.nav_safety.timed_out) {
    status.nav_safety_level = kLevelRed;
    reasons.emplace_back(status.nav_safety_level, input.nav_safety.reason.empty() ? "navigation timed out" : input.nav_safety.reason);
  } else if (input.nav_safety.stop_required) {
    status.nav_safety_level = kLevelRed;
    reasons.emplace_back(status.nav_safety_level, input.nav_safety.reason.empty() ? "navigation stop required" : input.nav_safety.reason);
  } else {
    status.nav_safety_level = kLevelGreen;
    reasons.emplace_back(status.nav_safety_level, "navigation safety nominal");
  }

  if (config_.nav_safety_present && input.nav_safety.stop_required) {
    std::ostringstream detail;
    detail << "current_profile=" << input.nav_safety.current_profile
           << "; stop_required=true"
           << "; timed_out=" << (input.nav_safety.timed_out ? "true" : "false")
           << "; reason=" << (input.nav_safety.reason.empty() ? "nav_safety_state.stop_required=true" : input.nav_safety.reason);
    events.push_back(makeEvent(
        "NAV_STOP_REQUIRED", kLevelRed, "导航要求停车", detail.str(),
        config_.nav_safety_topic, "立即确认障碍物、超时或策略切换原因，必要时人工接管。", true));
  }
  if (config_.nav_safety_present && input.nav_safety.timed_out) {
    std::ostringstream detail;
    detail << "current_profile=" << input.nav_safety.current_profile
           << "; stop_required=" << (input.nav_safety.stop_required ? "true" : "false")
           << "; timed_out=true"
           << "; reason=" << (input.nav_safety.reason.empty() ? "nav_safety_state.timed_out=true" : input.nav_safety.reason);
    events.push_back(makeEvent(
        "NAV_TIMED_OUT", kLevelRed, "导航超时", detail.str(),
        config_.nav_safety_topic, "检查 profile watchdog、地形策略和上层任务状态，避免继续自动推进。", true));
  }

  const double mechanism_limit_sec = config_.mechanism_max_age_ms / 1000.0;
  if (!config_.mechanism_present) {
    status.mechanism_level = kLevelGreen;
  } else if (!input.mechanism.received) {
    status.mechanism_level = kLevelOrange;
    reasons.emplace_back(status.mechanism_level, "waiting mechanism state");
  } else if (input.mechanism.age_sec > mechanism_limit_sec) {
    status.mechanism_level = kLevelOrange;
    reasons.emplace_back(status.mechanism_level,
                         staleReason(config_.mechanism_state_topic, input.mechanism.age_sec, mechanism_limit_sec));
  } else if (input.mechanism.comm_health_level >= config_.mechanism_error_level) {
    status.mechanism_level = kLevelRed;
    reasons.emplace_back(status.mechanism_level, "mechanism communication unhealthy");
  } else if (input.mechanism.comm_health_level >= config_.mechanism_warn_level) {
    status.mechanism_level = kLevelYellow;
    reasons.emplace_back(status.mechanism_level, "mechanism communication warning");
  } else {
    status.mechanism_level = kLevelGreen;
    reasons.emplace_back(status.mechanism_level, "mechanism nominal");
  }

  if (config_.mechanism_present && status.mechanism_level >= kLevelYellow && input.mechanism.received) {
    std::ostringstream detail;
    detail << "comm_health_level=" << static_cast<int>(input.mechanism.comm_health_level)
           << "; age_sec=" << formatSec(input.mechanism.age_sec)
           << "; warn_level=" << static_cast<int>(config_.mechanism_warn_level)
           << "; error_level=" << static_cast<int>(config_.mechanism_error_level)
           << "; max_age_sec=" << formatSec(mechanism_limit_sec);
    events.push_back(makeEvent(
        "MECHANISM_COMM_WARN", status.mechanism_level, "机构通信异常", detail.str(),
        config_.mechanism_state_topic, "检查机构串口链路、下位机状态和动作执行反馈。", true));
  }

  std::vector<std::string> stale_topic_names;
  for (const auto& monitored_topic : input.monitored_topics) {
    const bool stale = monitored_topic.required &&
                       (!monitored_topic.received || monitored_topic.age_sec > monitored_topic.max_age_sec);
    if (!stale) {
      continue;
    }
    stale_topic_names.push_back(monitored_topic.topic_name);
    ++status.topic_timeout_count;
    events.push_back(makeEvent(
        "TOPIC_STALE_" + monitored_topic.code_suffix, kLevelYellow, "关键话题超时",
        staleReason(monitored_topic.topic_name, monitored_topic.age_sec, monitored_topic.max_age_sec),
        monitored_topic.topic_name, "检查发布节点、命名空间和 QoS 是否匹配。", false));
  }

  const uint8_t topics_level = status.topic_timeout_count == 0U
                                   ? kLevelGreen
                                   : (status.topic_timeout_count >= config_.topics_orange_count ? kLevelOrange : kLevelYellow);
  reasons.emplace_back(topics_level,
                       status.topic_timeout_count == 0U ? "topic freshness nominal"
                                                        : ("stale topics: " + joinStrings(stale_topic_names)));

  status.overall_level = std::max({status.localization_level, status.controller_level, status.keepout_level,
                                   status.terrain_level, status.nav_safety_level, status.mechanism_level,
                                   topics_level, backend_level});

  std::stable_sort(reasons.begin(), reasons.end(), [](const auto& left, const auto& right) {
    return left.first > right.first;
  });
  for (const auto& [level, reason] : reasons) {
    if (level == status.overall_level && !reason.empty()) {
      status.overall_reason = reason;
      break;
    }
  }

  std::sort(events.begin(), events.end(), [](const auto& left, const auto& right) {
    if (left.severity != right.severity) {
      return left.severity > right.severity;
    }
    return left.code < right.code;
  });
  output.events.events = events;
  for (const auto& event : events) {
    status.active_event_codes.push_back(event.code);
  }

  const double localization_last_age = latestAgeSec(
      {input.localization.received ? input.localization.age_sec : kInf,
       input.backend.received ? input.backend.age_sec : kInf});
  const double controller_last_age = latestAgeSec(
      {input.control_degraded.received ? input.control_degraded.age_sec : kInf,
       input.control_degenerate_score.received ? input.control_degenerate_score.age_sec : kInf,
       input.compute_time_ms.received ? input.compute_time_ms.age_sec : kInf,
       input.pose_age_ms.received ? input.pose_age_ms.age_sec : kInf,
       input.collision_d_min.received ? input.collision_d_min.age_sec : kInf,
       input.controller_mode.received ? input.controller_mode.age_sec : kInf});
  const double keepout_last_age = latestAgeSec(
      {input.keepout.filter_info_received ? input.keepout.filter_info_age_sec : kInf,
       input.keepout.mask_received ? input.keepout.mask_age_sec : kInf,
       input.keepout.heartbeat_received ? input.keepout.heartbeat_age_sec : kInf});
  const double terrain_last_age = latestAgeSec(
      {input.terrain.obstacles_received ? input.terrain.obstacles_age_sec : kInf,
       input.terrain.drop_received ? input.terrain.drop_age_sec : kInf,
       input.terrain.grid_received ? input.terrain.grid_age_sec : kInf,
       input.terrain.speed_limit_received ? input.terrain.speed_limit_age_sec : kInf});
  const double topics_last_age = [&input]() {
    double best = kInf;
    for (const auto& monitored_topic : input.monitored_topics) {
      if (std::isfinite(monitored_topic.age_sec)) {
        best = std::min(best, monitored_topic.age_sec);
      }
    }
    return best;
  }();

  output.summary.status.push_back(makeStatus(
      "r2/localization", std::max(status.localization_level, backend_level), status.localization_reason,
      {
          config_.localization_present,
          config_.localization_present,
          input.localization.received || input.backend.received,
          formatLastUpdateMs(header, localization_last_age),
          {config_.localization_health_topic, config_.localization_backend_topic},
      },
      {
          {"state", status.localization_state},
          {"reason", status.localization_reason},
          {"sigma_xy", formatDouble(input.localization.sigma_xy, 3)},
          {"sigma_yaw", formatDouble(input.localization.sigma_yaw, 3)},
          {"h_min_eig", formatDouble(input.localization.h_min_eig, 2)},
          {"degenerate_score", formatDouble(input.localization.degenerate_score, 3)},
          {"backend_reason", backend_reason},
          {"backend_optimizer_state", input.backend.optimizer_state},
          {"graph_health", formatDouble(input.backend.graph_health, 2)},
      }));

  output.summary.status.push_back(makeStatus(
      "r2/controller", status.controller_level,
      config_.controller_present
          ? (status.controller_level == kLevelGreen ? "controller nominal" : "controller attention required")
          : disabledReason,
      {
          config_.controller_present,
          config_.controller_present,
          input.control_degraded.received || input.control_degenerate_score.received || input.compute_time_ms.received ||
              input.pose_age_ms.received || input.collision_d_min.received || input.controller_mode.received,
          formatLastUpdateMs(header, controller_last_age),
          {config_.control_degraded_topic, config_.control_degenerate_score_topic, config_.compute_time_ms_topic,
           config_.pose_age_ms_topic, config_.collision_d_min_topic, config_.controller_mode_topic},
      },
      {
          {"mode", status.controller_mode},
          {"control_degraded", boolString(status.control_degraded)},
          {"control_degenerate_score", formatDouble(status.control_degenerate_score, 3)},
          {"compute_time_ms", formatDouble(status.compute_time_ms, 1)},
          {"pose_age_ms", formatDouble(status.pose_age_ms, 1)},
          {"collision_d_min", formatDouble(status.collision_d_min, 2)},
      }));

  output.summary.status.push_back(makeStatus(
      "r2/keepout", status.keepout_level, status.keepout_reason,
      {
          config_.keepout_present,
          config_.keepout_present,
          input.keepout.filter_info_received || input.keepout.mask_received || input.keepout.heartbeat_received,
          formatLastUpdateMs(header, keepout_last_age),
          {config_.costmap_filter_info_topic, config_.kfs_filter_mask_topic, config_.kfs_heartbeat_topic},
      },
      {
          {"ready", boolString(status.keepout_ready)},
          {"filter_age_sec", formatDouble(input.keepout.filter_info_age_sec, 2)},
          {"mask_age_sec", formatDouble(input.keepout.mask_age_sec, 2)},
          {"heartbeat_age_sec", formatDouble(input.keepout.heartbeat_age_sec, 2)},
          {"heartbeat_enabled", boolString(input.keepout.heartbeat_enabled)},
      }));

  output.summary.status.push_back(makeStatus(
      "r2/terrain", status.terrain_level,
      !config_.terrain_present ? disabledReason
                               : (status.terrain_level == kLevelGreen ? "terrain nominal"
                                                                      : (!status.terrain_online ? "terrain offline/stale"
                                                                                                : "terrain hazard active")),
      {
          config_.terrain_present,
          config_.terrain_present,
          input.terrain.obstacles_received || input.terrain.drop_received ||
              input.terrain.grid_received || input.terrain.speed_limit_received,
          formatLastUpdateMs(header, terrain_last_age),
          {config_.terrain_obstacles_topic, config_.terrain_drop_topic,
           config_.terrain_grid_topic, config_.terrain_speed_limit_topic},
      },
      {
          {"online", boolString(status.terrain_online)},
          {"terrain_obstacle_active", boolString(status.terrain_obstacle_active)},
          {"terrain_drop_active", boolString(status.terrain_drop_active)},
          {"terrain_climbable_active", boolString(status.terrain_climbable_active)},
          {"terrain_step_edge_active", boolString(status.terrain_step_edge_active)},
          {"terrain_speed_limited", boolString(status.terrain_speed_limited)},
          {"terrain_traversability_min", formatDouble(status.terrain_traversability_min, 3)},
          {"obstacles_age_sec", formatDouble(input.terrain.obstacles_age_sec, 2)},
          {"drop_age_sec", formatDouble(input.terrain.drop_age_sec, 2)},
          {"grid_age_sec", formatDouble(input.terrain.grid_age_sec, 2)},
          {"speed_limit_age_sec", formatDouble(input.terrain.speed_limit_age_sec, 2)},
      }));

  output.summary.status.push_back(makeStatus(
      "r2/nav_safety", status.nav_safety_level,
      !config_.nav_safety_present ? disabledReason
                                  : (status.nav_safety_level == kLevelGreen ? "nav safety nominal"
                                                                            : (input.nav_safety.reason.empty() ? "nav safety attention required"
                                                                                                               : input.nav_safety.reason)),
      {
          config_.nav_safety_present,
          config_.nav_safety_present,
          input.nav_safety.received,
          formatLastUpdateMs(header, input.nav_safety.age_sec),
          {config_.nav_safety_topic},
      },
      {
          {"profile", status.nav_profile},
          {"stop_required", boolString(status.nav_stop_required)},
          {"timed_out", boolString(status.nav_timed_out)},
          {"reason", input.nav_safety.reason},
      }));

  output.summary.status.push_back(makeStatus(
      "r2/mechanism", status.mechanism_level,
      !config_.mechanism_present ? disabledReason
                                 : (status.mechanism_level == kLevelGreen ? "mechanism nominal"
                                                                         : (status.mechanism_level == kLevelRed ? "mechanism communication unhealthy"
                                                                                                                : "mechanism communication warning")),
      {
          config_.mechanism_present,
          config_.mechanism_present,
          input.mechanism.received,
          formatLastUpdateMs(header, input.mechanism.age_sec),
          {config_.mechanism_state_topic},
      },
      {
          {"comm_health_level", std::to_string(static_cast<int>(input.mechanism.comm_health_level))},
          {"age_sec", formatDouble(input.mechanism.age_sec, 2)},
      }));

  output.summary.status.push_back(makeStatus(
      "r2/topics", topics_level,
      status.topic_timeout_count == 0U ? "topic freshness nominal" : ("stale_count=" + std::to_string(status.topic_timeout_count)),
      {
          true,
          true,
          topics_last_age != kInf,
          formatLastUpdateMs(header, topics_last_age),
          [&input]() {
            std::vector<std::string> topics;
            topics.reserve(input.monitored_topics.size());
            for (const auto& monitored_topic : input.monitored_topics) {
              topics.push_back(monitored_topic.topic_name);
            }
            return topics;
          }(),
      },
      {
          {"stale_count", std::to_string(status.topic_timeout_count)},
          {"stale_topics", joinStrings(stale_topic_names)},
      }));

  return output;
}

}  // namespace rc26_visualization
