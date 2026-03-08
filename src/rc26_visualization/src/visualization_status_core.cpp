#include "rc26_visualization/visualization_status_core.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
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

std::string formatMs(double value) {
  return formatDouble(value, 1) + " ms";
}

std::string formatSec(double value) {
  return formatDouble(value, 2) + " s";
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

void addKv(diagnostic_msgs::msg::DiagnosticStatus& status, const std::string& key, const std::string& value) {
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  status.values.push_back(std::move(item));
}

diagnostic_msgs::msg::DiagnosticStatus makeStatus(
    const std::string& name, uint8_t level, const std::string& message,
    const std::vector<std::pair<std::string, std::string>>& values) {
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = name;
  status.hardware_id = "R2";
  status.level = diagLevel(level);
  status.message = message;
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
  event.source_signal = source_signal;
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

VisualizationStatusCore::Output VisualizationStatusCore::evaluate(
    const EvaluationInput& input, const std_msgs::msg::Header& header) const {
  Output output;
  output.summary.header = header;
  output.operator_status.header = header;
  output.events.header = header;

  auto& status = output.operator_status;
  status.overall_level = kLevelGreen;
  status.overall_reason = "nominal";
  status.localization_level = input.localization.received ? std::min<uint8_t>(input.localization.level, kLevelRed) : kLevelRed;
  status.localization_reason = input.localization.received ? input.localization.reason : "waiting localization health";
  status.localization_state = input.localization.received ? input.localization.state : "UNKNOWN";
  status.controller_level = kLevelGreen;
  status.controller_mode = input.controller_mode.received ? input.controller_mode.value : "unknown";
  status.control_degraded = input.control_degraded.received ? input.control_degraded.value : input.localization.control_degraded;
  status.control_degenerate_score = input.control_degenerate_score.received ? input.control_degenerate_score.value
                                                                           : std::numeric_limits<double>::quiet_NaN();
  status.compute_time_ms = input.compute_time_ms.received ? input.compute_time_ms.value
                                                          : std::numeric_limits<double>::quiet_NaN();
  status.pose_age_ms = input.pose_age_ms.received ? input.pose_age_ms.value : std::numeric_limits<double>::quiet_NaN();
  status.collision_d_min = input.collision_d_min.received ? input.collision_d_min.value
                                                           : std::numeric_limits<double>::quiet_NaN();
  status.keepout_level = kLevelGreen;
  status.keepout_ready = false;
  status.keepout_reason = "waiting keepout inputs";
  status.terrain_level = kLevelGreen;
  status.terrain_online = false;
  status.terrain_obstacle_active = input.terrain.obstacles_active;
  status.terrain_drop_active = input.terrain.drop_active;
  status.nav_safety_level = kLevelGreen;
  status.nav_profile = input.nav_safety.received ? input.nav_safety.current_profile : "unknown";
  status.nav_stop_required = input.nav_safety.stop_required;
  status.nav_timed_out = input.nav_safety.timed_out;
  status.mechanism_level = kLevelGreen;
  status.mechanism_comm_health = input.mechanism.comm_health_level;
  status.topic_timeout_count = 0U;

  std::vector<rc26_interfaces::msg::VisualizationEvent> events;
  std::vector<std::pair<uint8_t, std::string>> reasons;
  reasons.emplace_back(status.localization_level, status.localization_reason);

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
  status.controller_level = std::max({pose_age_level, control_time_level, obstacle_level, control_degraded_level});
  reasons.emplace_back(status.controller_level,
                       status.controller_level == kLevelGreen ? "controller nominal"
                                                              : "controller attention required");

  uint8_t backend_level = kLevelGreen;
  std::string backend_reason = "backend nominal";
  if (input.backend.received) {
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

  if (status.localization_level >= kLevelYellow) {
    events.push_back(makeEvent(
        "LOCALIZATION_DEGRADED", status.localization_level, "定位退化", status.localization_reason,
        config_.localization_health_topic, "确认定位输入、重定位状态和环境可观测性，必要时切人工接管。", true));
  }

  if (backend_level >= kLevelYellow) {
    std::ostringstream detail;
    detail << backend_reason << "; optimizer_state=" << input.backend.optimizer_state
           << "; graph_health=" << formatDouble(input.backend.graph_health, 2)
           << "; last_local_reg_age_sec=" << formatDouble(input.backend.last_local_reg_age_sec, 2)
           << "; imu_spike=" << (input.backend.imu_spike ? "true" : "false");
    events.push_back(makeEvent(
        "LOCALIZATION_BACKEND_WARN", backend_level, "定位后端告警", detail.str(),
        config_.localization_backend_topic, "检查图优化器、IMU 异常与回环/锚点输入，避免在异常状态下持续自动运行。", false));
    reasons.emplace_back(backend_level, backend_reason);
  }

  if (status.control_degraded) {
    std::ostringstream detail;
    detail << "control_degraded=true, degenerate_score=" << formatDouble(status.control_degenerate_score, 3);
    events.push_back(makeEvent(
        "CONTROL_DEGRADED", kLevelOrange, "控制退化", detail.str(), config_.control_degraded_topic,
        "降低速度并检查预测里程计与退化分数来源；必要时切换保底视图确认轨迹。", true));
  }

  if (pose_age_level >= kLevelYellow) {
    std::ostringstream detail;
    detail << "pose_age_ms=" << formatDouble(status.pose_age_ms, 1)
           << ", thresholds=[" << formatDouble(pose_age_yellow_ms, 1) << ", "
           << formatDouble(pose_age_orange_ms, 1) << ", " << formatDouble(pose_age_red_ms, 1) << "]";
    events.push_back(makeEvent(
        "POSE_STALE", pose_age_level, "位姿时效下降", detail.str(), config_.pose_age_ms_topic,
        "检查定位链路刷新率与控制输入时间戳，避免继续高速运动。", false));
  }

  if (control_time_level >= kLevelYellow) {
    std::ostringstream detail;
    detail << "compute_time_ms=" << formatDouble(status.compute_time_ms, 1)
           << ", thresholds=[" << formatDouble(control_time_yellow_ms, 1) << ", "
           << formatDouble(control_time_orange_ms, 1) << ", " << formatDouble(control_time_red_ms, 1) << "]";
    events.push_back(makeEvent(
        "CONTROL_OVERRUN", control_time_level, "控制周期超限", detail.str(), config_.compute_time_ms_topic,
        "降低任务负载并检查控制器实时性，必要时暂停自动导航。", false));
  }

  if (obstacle_level >= kLevelYellow) {
    std::ostringstream detail;
    detail << "collision_d_min=" << formatDouble(status.collision_d_min, 2)
           << " m, brake_margin=" << formatDouble(config_.brake_margin_m, 2) << " m";
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
  reasons.emplace_back(status.keepout_level, status.keepout_reason);
  if (status.keepout_level >= kLevelYellow) {
    std::ostringstream detail;
    detail << status.keepout_reason << "; filter_age=" << formatSec(input.keepout.filter_info_age_sec)
           << ", mask_age=" << formatSec(input.keepout.mask_age_sec)
           << ", heartbeat_age=" << formatSec(input.keepout.heartbeat_age_sec)
           << ", heartbeat_enabled=" << (input.keepout.heartbeat_enabled ? "true" : "false");
    events.push_back(makeEvent(
        "KEEPOUT_STALE", status.keepout_level, "Keepout 失效风险", detail.str(),
        config_.costmap_filter_info_topic + "," + config_.kfs_filter_mask_topic + "," + config_.kfs_heartbeat_topic,
        "检查 keepout 掩码、元数据和心跳链路，确认防区仍在更新。", true));
  }

  const double terrain_limit_sec = config_.terrain_max_age_ms / 1000.0;
  const bool obstacles_fresh = input.terrain.obstacles_received && input.terrain.obstacles_age_sec <= terrain_limit_sec;
  const bool drop_fresh = input.terrain.drop_received && input.terrain.drop_age_sec <= terrain_limit_sec;
  status.terrain_online = obstacles_fresh || drop_fresh;
  if (!input.terrain.obstacles_received && !input.terrain.drop_received) {
    status.terrain_level = kLevelOrange;
  } else if (!status.terrain_online) {
    status.terrain_level = kLevelOrange;
  } else if (input.terrain.obstacles_active || input.terrain.drop_active) {
    status.terrain_level = kLevelYellow;
  } else {
    status.terrain_level = kLevelGreen;
  }
  reasons.emplace_back(status.terrain_level,
                       status.terrain_level == kLevelGreen ? "terrain nominal"
                                                          : (!status.terrain_online ? "terrain offline/stale"
                                                                                    : "terrain hazard detected"));

  const double nav_limit_sec = config_.nav_safety_max_age_ms / 1000.0;
  if (!input.nav_safety.received) {
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

  if (input.nav_safety.stop_required) {
    events.push_back(makeEvent(
        "NAV_STOP_REQUIRED", kLevelRed, "导航要求停车",
        input.nav_safety.reason.empty() ? "nav_safety_state.stop_required=true" : input.nav_safety.reason,
        config_.nav_safety_topic, "立即确认障碍物、超时或策略切换原因，必要时人工接管。", true));
  }
  if (input.nav_safety.timed_out) {
    events.push_back(makeEvent(
        "NAV_TIMED_OUT", kLevelRed, "导航超时",
        input.nav_safety.reason.empty() ? "nav_safety_state.timed_out=true" : input.nav_safety.reason,
        config_.nav_safety_topic, "检查 profile watchdog、地形策略和上层任务状态，避免继续自动推进。", true));
  }

  const double mechanism_limit_sec = config_.mechanism_max_age_ms / 1000.0;
  if (!input.mechanism.received) {
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

  if (status.mechanism_level >= kLevelYellow && input.mechanism.received) {
    std::ostringstream detail;
    detail << "comm_health_level=" << static_cast<int>(input.mechanism.comm_health_level)
           << ", age=" << formatSec(input.mechanism.age_sec);
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

  output.summary.status.push_back(makeStatus(
      "r2/localization", std::max(status.localization_level, backend_level), status.localization_reason,
      {
          {"state", status.localization_state},
          {"reason", status.localization_reason},
          {"backend_reason", backend_reason},
          {"backend_optimizer_state", input.backend.optimizer_state},
          {"graph_health", formatDouble(input.backend.graph_health, 2)},
      }));

  output.summary.status.push_back(makeStatus(
      "r2/controller", status.controller_level,
      status.controller_level == kLevelGreen ? "controller nominal" : "controller attention required",
      {
          {"mode", status.controller_mode},
          {"control_degraded", status.control_degraded ? "true" : "false"},
          {"control_degenerate_score", formatDouble(status.control_degenerate_score, 3)},
          {"compute_time_ms", formatDouble(status.compute_time_ms, 1)},
          {"pose_age_ms", formatDouble(status.pose_age_ms, 1)},
          {"collision_d_min", formatDouble(status.collision_d_min, 2)},
      }));

  output.summary.status.push_back(makeStatus(
      "r2/keepout", status.keepout_level, status.keepout_reason,
      {
          {"ready", status.keepout_ready ? "true" : "false"},
          {"filter_age_sec", formatDouble(input.keepout.filter_info_age_sec, 2)},
          {"mask_age_sec", formatDouble(input.keepout.mask_age_sec, 2)},
          {"heartbeat_age_sec", formatDouble(input.keepout.heartbeat_age_sec, 2)},
          {"heartbeat_enabled", input.keepout.heartbeat_enabled ? "true" : "false"},
      }));

  output.summary.status.push_back(makeStatus(
      "r2/terrain", status.terrain_level,
      status.terrain_level == kLevelGreen ? "terrain nominal"
                                          : (!status.terrain_online ? "terrain offline/stale" : "terrain hazard active"),
      {
          {"online", status.terrain_online ? "true" : "false"},
          {"terrain_obstacle_active", status.terrain_obstacle_active ? "true" : "false"},
          {"terrain_drop_active", status.terrain_drop_active ? "true" : "false"},
          {"obstacles_age_sec", formatDouble(input.terrain.obstacles_age_sec, 2)},
          {"drop_age_sec", formatDouble(input.terrain.drop_age_sec, 2)},
      }));

  output.summary.status.push_back(makeStatus(
      "r2/nav_safety", status.nav_safety_level,
      status.nav_safety_level == kLevelGreen ? "nav safety nominal"
                                             : (input.nav_safety.reason.empty() ? "nav safety attention required"
                                                                                : input.nav_safety.reason),
      {
          {"profile", status.nav_profile},
          {"stop_required", status.nav_stop_required ? "true" : "false"},
          {"timed_out", status.nav_timed_out ? "true" : "false"},
          {"reason", input.nav_safety.reason},
      }));

  output.summary.status.push_back(makeStatus(
      "r2/mechanism", status.mechanism_level,
      status.mechanism_level == kLevelGreen ? "mechanism nominal"
                                            : (status.mechanism_level == kLevelRed ? "mechanism communication unhealthy"
                                                                                   : "mechanism communication warning"),
      {
          {"comm_health_level", std::to_string(static_cast<int>(input.mechanism.comm_health_level))},
          {"age_sec", formatDouble(input.mechanism.age_sec, 2)},
      }));

  output.summary.status.push_back(makeStatus(
      "r2/topics", topics_level,
      status.topic_timeout_count == 0U ? "topic freshness nominal" : ("stale_count=" + std::to_string(status.topic_timeout_count)),
      {
          {"stale_count", std::to_string(status.topic_timeout_count)},
          {"stale_topics", joinStrings(stale_topic_names)},
      }));

  return output;
}

}  // namespace rc26_visualization
