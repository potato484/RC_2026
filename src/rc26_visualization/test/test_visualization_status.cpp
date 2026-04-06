#include <algorithm>
#include <string_view>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include "gtest/gtest.h"

#include "rc26_visualization/visualization_status_core.hpp"

namespace rc26_visualization {
namespace {

EvaluationInput makeNominalInput() {
  EvaluationInput input;
  input.localization.received = true;
  input.localization.age_sec = 0.05;
  input.localization.level = kLevelGreen;
  input.localization.reason = "ok";
  input.localization.state = "TRACKING";
  input.localization.control_degraded = false;
  input.localization.sigma_xy = 0.03;
  input.localization.sigma_yaw = 0.02;
  input.localization.degenerate_score = 0.85;
  input.localization.h_min_eig = 92.0;

  input.backend.received = true;
  input.backend.age_sec = 0.05;
  input.backend.optimizer_ready = true;
  input.backend.optimizer_state = "running";
  input.backend.graph_health = 0.95;
  input.backend.last_local_reg_age_sec = 0.05;
  input.backend.last_loop_age_sec = 0.2;
  input.backend.last_anchor_age_sec = 0.3;
  input.backend.imu_spike = false;

  input.control_degraded.received = true;
  input.control_degraded.age_sec = 0.05;
  input.control_degraded.value = false;

  input.control_degenerate_score.received = true;
  input.control_degenerate_score.age_sec = 0.05;
  input.control_degenerate_score.value = 0.85;

  input.compute_time_ms.received = true;
  input.compute_time_ms.age_sec = 0.05;
  input.compute_time_ms.value = 10.0;

  input.pose_age_ms.received = true;
  input.pose_age_ms.age_sec = 0.05;
  input.pose_age_ms.value = 20.0;

  input.collision_d_min.received = true;
  input.collision_d_min.age_sec = 0.05;
  input.collision_d_min.value = 0.80;

  input.controller_mode.received = true;
  input.controller_mode.age_sec = 0.05;
  input.controller_mode.value = "nmpc";

  input.nav_safety.received = true;
  input.nav_safety.age_sec = 0.10;
  input.nav_safety.current_profile = "normal";
  input.nav_safety.reason = "nominal";
  input.nav_safety.stop_required = false;
  input.nav_safety.timed_out = false;

  input.local_planner.received = true;
  input.local_planner.age_sec = 0.10;
  input.local_planner.status = "PASS";
  input.local_planner.terminal = false;
  input.local_planner.semantic_revision = 7.0;
  input.local_planner.best_score = 0.11;
  input.local_planner.clearance_margin_m = 0.24;
  input.local_planner.reason = "tracking";

  input.recovery_runtime.received = true;
  input.recovery_runtime.age_sec = 0.10;
  input.recovery_runtime.recovery_name = "none";
  input.recovery_runtime.status = "IDLE";
  input.recovery_runtime.terminal = false;
  input.recovery_runtime.elapsed_sec = 0.0;
  input.recovery_runtime.reason = "tracking";

  input.semantic_runtime.received = true;
  input.semantic_runtime.age_sec = 0.10;
  input.semantic_runtime.revision = 7U;
  input.semantic_runtime.terrain_available = true;
  input.semantic_runtime.keepout_available = true;
  input.semantic_runtime.blocked_cells = 0U;
  input.semantic_runtime.slow_cells = 0U;
  input.semantic_runtime.max_obstacle_probability = 0.05;
  input.semantic_runtime.max_drop_probability = 0.03;

  input.mechanism.received = true;
  input.mechanism.age_sec = 0.10;
  input.mechanism.comm_health_level = 0U;

  input.keepout.overlay_received = true;
  input.keepout.overlay_age_sec = 0.05;
  input.keepout.mask_received = true;
  input.keepout.mask_age_sec = 0.05;
  input.keepout.heartbeat_received = true;
  input.keepout.heartbeat_age_sec = 0.05;
  input.keepout.heartbeat_enabled = true;

  input.terrain.obstacles_received = true;
  input.terrain.obstacles_age_sec = 0.10;
  input.terrain.obstacles_active = false;
  input.terrain.drop_received = true;
  input.terrain.drop_age_sec = 0.10;
  input.terrain.drop_active = false;
  input.terrain.grid_received = true;
  input.terrain.grid_age_sec = 0.10;
  input.terrain.traversability_min = 0.92;
  input.terrain.climbable_active = false;
  input.terrain.step_edge_active = false;

  input.monitored_topics = {
      {"LOCALIZATION_HEALTH", "/localization/health", 1.0, true, true, 0.05},
      {"LOCALIZATION_BACKEND_STATUS", "/localization/backend_status", 1.5, true, true, 0.05},
      {"CONTROL_DEGRADED", "/control_degraded", 0.5, true, true, 0.05},
      {"CONTROL_DEGENERATE_SCORE", "control_degenerate_score", 0.5, true, true, 0.05},
      {"COMPUTE_TIME_MS", "compute_time_ms", 0.5, true, true, 0.05},
      {"POSE_AGE_MS", "pose_age_ms", 0.5, true, true, 0.05},
      {"COLLISION_D_MIN", "collision_d_min", 0.5, true, true, 0.05},
      {"MOTION_MODE_STATE", "/xhu_nav/motion_mode_state", 2.5, true, true, 0.10},
      {"TRACKING_STATE", "/xhu_nav/tracking_state", 1.5, true, true, 0.10},
      {"LOCAL_PLANNER_STATE", "/xhu_nav/local_planner_state", 1.5, true, true, 0.10},
      {"RECOVERY_STATE", "/xhu_nav/recovery_state", 1.5, true, true, 0.10},
      {"SEMANTIC_LAYER_SUMMARY", "/xhu_nav/semantic_layer_summary", 1.5, true, true, 0.10},
      {"MECHANISM_STATE", "/mechanism/state", 1.0, true, true, 0.10},
      {"BLOCK_OVERLAY", "/mf_block_overlay", 0.3, true, true, 0.05},
      {"KFS_FILTER_MASK", "/kfs_filter_mask", 0.3, true, true, 0.05},
      {"KFS_KEEPOUT_HEARTBEAT", "/kfs_keepout_heartbeat", 0.3, true, true, 0.05},
      {"TERRAIN_OBSTACLES", "terrain_obstacles", 1.0, true, true, 0.10},
      {"TERRAIN_DROP", "terrain_drop", 1.0, true, true, 0.10},
      {"TERRAIN_GRID_MAP_LOCAL", "/terrain_grid_map_local", 1.0, true, true, 0.10},
      {"ODOM", "odom", 0.5, true, true, 0.05},
      {"CONTROL_STATE", "control_state", 0.5, true, true, 0.05},
  };
  return input;
}

std::vector<std::string> eventCodes(const rc26_interfaces::msg::VisualizationEventArray& events) {
  std::vector<std::string> codes;
  for (const auto& event : events.events) {
    codes.push_back(event.code);
  }
  return codes;
}

const diagnostic_msgs::msg::DiagnosticStatus& findStatus(
    const diagnostic_msgs::msg::DiagnosticArray& summary, std::string_view name) {
  const auto iter = std::find_if(summary.status.begin(), summary.status.end(), [name](const auto& status) {
    return status.name == name;
  });
  EXPECT_NE(iter, summary.status.end());
  if (iter == summary.status.end()) {
    return summary.status.front();
  }
  return *iter;
}

std::string valueForKey(const diagnostic_msgs::msg::DiagnosticStatus& status, std::string_view key) {
  const auto iter = std::find_if(status.values.begin(), status.values.end(), [key](const auto& item) {
    return item.key == key;
  });
  EXPECT_NE(iter, status.values.end());
  if (iter == status.values.end()) {
    return {};
  }
  return iter->value;
}

const rc26_interfaces::msg::VisualizationEvent& findEvent(
    const rc26_interfaces::msg::VisualizationEventArray& events, std::string_view code) {
  const auto iter = std::find_if(events.events.begin(), events.events.end(), [code](const auto& event) {
    return event.code == code;
  });
  EXPECT_NE(iter, events.events.end());
  if (iter == events.events.end()) {
    return events.events.front();
  }
  return *iter;
}

void expectContainsAll(std::string_view value, std::initializer_list<std::string_view> parts) {
  for (const auto part : parts) {
    EXPECT_NE(value.find(part), std::string_view::npos) << "missing substring: " << part;
  }
}

TEST(VisualizationStatusCoreTest, NominalCruiseStaysGreen) {
  VisualizationStatusCore core;
  std_msgs::msg::Header header;
  header.stamp.sec = 123;
  header.stamp.nanosec = 400000000U;
  header.frame_id = "map";
  auto output = core.evaluate(makeNominalInput(), header);
  const auto& localization = findStatus(output.summary, "r2/localization");
  const auto& controller = findStatus(output.summary, "r2/controller");
  const auto& keepout = findStatus(output.summary, "r2/keepout");

  EXPECT_EQ(output.operator_status.overall_level, kLevelGreen);
  EXPECT_EQ(output.operator_status.controller_level, kLevelGreen);
  EXPECT_TRUE(output.operator_status.keepout_ready);
  EXPECT_FALSE(output.operator_status.terrain_climbable_active);
  EXPECT_FALSE(output.operator_status.terrain_step_edge_active);
  EXPECT_NEAR(output.operator_status.terrain_traversability_min, 0.92f, 1e-5f);
  EXPECT_EQ(output.operator_status.topic_timeout_count, 0U);
  EXPECT_TRUE(output.events.events.empty());
  EXPECT_EQ(output.summary.status.size(), 7U);
  EXPECT_EQ(valueForKey(localization, "present"), "true");
  EXPECT_EQ(valueForKey(localization, "enabled"), "true");
  EXPECT_EQ(valueForKey(localization, "received"), "true");
  EXPECT_EQ(valueForKey(localization, "source_topics"), "/localization/health,/localization/backend_status");
  EXPECT_EQ(valueForKey(localization, "last_update_ms"), "123350");
  EXPECT_EQ(valueForKey(controller, "source_topics"),
            "/control_degraded,control_degenerate_score,compute_time_ms,pose_age_ms,collision_d_min,/xhu_nav/semantic_gate,/xhu_nav/tracking_state");
  EXPECT_EQ(valueForKey(keepout, "source_topics"), "/mf_block_overlay,/kfs_filter_mask,/kfs_keepout_heartbeat");
}

TEST(VisualizationStatusCoreTest, NearObstacleRaisesControllerAlert) {
  VisualizationStatusCore core;
  std_msgs::msg::Header header;
  auto input = makeNominalInput();
  input.collision_d_min.value = 0.25;

  auto output = core.evaluate(input, header);
  const auto codes = eventCodes(output.events);

  EXPECT_EQ(output.operator_status.controller_level, kLevelOrange);
  EXPECT_EQ(output.operator_status.overall_level, kLevelOrange);
  EXPECT_NE(std::find(codes.begin(), codes.end(), "OBSTACLE_NEAR"), codes.end());
  expectContainsAll(findEvent(output.events, "OBSTACLE_NEAR").detail,
                    {"collision_d_min=0.25 m", "yellow_limit=0.45 m", "orange_limit=0.30 m", "red_limit=0.15 m"});
}

TEST(VisualizationStatusCoreTest, LocalPlannerRuntimeSignalsBecomeEvents) {
  VisualizationStatusCore core;
  std_msgs::msg::Header header;
  auto input = makeNominalInput();
  input.local_planner.status = "WAITING_ON_BLOCK";
  input.local_planner.reason = "keepout blocked";
  input.local_planner.clearance_margin_m = 0.05;
  input.recovery_runtime.recovery_name = "rotate_in_place";
  input.recovery_runtime.status = "RUNNING";
  input.recovery_runtime.elapsed_sec = 1.2;
  input.recovery_runtime.reason = "heading mismatch";
  input.semantic_runtime.blocked_cells = 12U;
  input.semantic_runtime.slow_cells = 4U;
  input.semantic_runtime.active_sources = {"keepout", "terrain"};
  input.semantic_runtime.active_reasons = {"blocked_zone"};

  auto output = core.evaluate(input, header);
  const auto codes = eventCodes(output.events);
  const auto& nav_safety = findStatus(output.summary, "r2/nav_safety");

  EXPECT_NE(std::find(codes.begin(), codes.end(), "LOCAL_PLANNER_WAITING"), codes.end());
  EXPECT_NE(std::find(codes.begin(), codes.end(), "LOCAL_RECOVERY_RUNNING"), codes.end());
  EXPECT_NE(std::find(codes.begin(), codes.end(), "SEMANTIC_LAYER_BLOCKED"), codes.end());
  EXPECT_EQ(valueForKey(nav_safety, "local_planner_status"), "WAITING_ON_BLOCK");
  EXPECT_EQ(valueForKey(nav_safety, "recovery_status"), "RUNNING");
  EXPECT_EQ(valueForKey(nav_safety, "semantic_blocked_cells"), "12");
  expectContainsAll(
      findEvent(output.events, "SEMANTIC_LAYER_BLOCKED").detail,
      {"blocked_cells=12", "slow_cells=4", "active_sources=keepout, terrain"});
}

TEST(VisualizationStatusCoreTest, KeepoutStaleTriggersRedEvent) {
  VisualizationStatusCore core;
  std_msgs::msg::Header header;
  auto input = makeNominalInput();
  input.keepout.overlay_age_sec = 0.45;
  input.keepout.mask_age_sec = 0.45;
  input.keepout.heartbeat_age_sec = 0.45;
  for (auto& topic : input.monitored_topics) {
    if (topic.code_suffix == "BLOCK_OVERLAY" || topic.code_suffix == "KFS_FILTER_MASK" ||
        topic.code_suffix == "KFS_KEEPOUT_HEARTBEAT") {
      topic.age_sec = 0.45;
    }
  }

  auto output = core.evaluate(input, header);
  const auto codes = eventCodes(output.events);

  EXPECT_EQ(output.operator_status.keepout_level, kLevelRed);
  EXPECT_EQ(output.operator_status.overall_level, kLevelRed);
  EXPECT_NE(std::find(codes.begin(), codes.end(), "KEEPOUT_STALE"), codes.end());
  expectContainsAll(findEvent(output.events, "KEEPOUT_STALE").detail,
                    {"overlay_age_sec=0.45 s", "mask_age_sec=0.45 s", "heartbeat_age_sec=0.45 s", "max_age_sec=0.30 s"});
}

TEST(VisualizationStatusCoreTest, LocalizationOrControlDegradeIsDetected) {
  VisualizationStatusCore core;
  std_msgs::msg::Header header;
  auto input = makeNominalInput();
  input.localization.level = kLevelRed;
  input.localization.reason = "RELOC_FAILED";
  input.localization.state = "RELOCALIZING";
  input.localization.sigma_xy = 0.42;
  input.localization.sigma_yaw = 0.31;
  input.localization.h_min_eig = 1.23;
  input.localization.degenerate_score = 0.12;
  input.backend.graph_health = 0.45;
  input.backend.last_local_reg_age_sec = 0.90;
  input.control_degraded.value = true;
  input.control_degenerate_score.value = 0.01;

  auto output = core.evaluate(input, header);
  const auto codes = eventCodes(output.events);

  EXPECT_EQ(output.operator_status.localization_level, kLevelRed);
  EXPECT_EQ(output.operator_status.overall_level, kLevelRed);
  EXPECT_NE(std::find(codes.begin(), codes.end(), "LOCALIZATION_DEGRADED"), codes.end());
  EXPECT_NE(std::find(codes.begin(), codes.end(), "CONTROL_DEGRADED"), codes.end());
  EXPECT_NE(std::find(codes.begin(), codes.end(), "LOCALIZATION_BACKEND_WARN"), codes.end());
  expectContainsAll(findEvent(output.events, "LOCALIZATION_DEGRADED").detail,
                    {"localization_state=RELOCALIZING", "reason=RELOC_FAILED", "sigma_xy=0.420",
                     "sigma_yaw=0.310", "h_min_eig=1.23", "degenerate_score=0.120"});
  expectContainsAll(findEvent(output.events, "CONTROL_DEGRADED").detail,
                    {"control_degraded=true", "degenerate_score=0.010", "control_degraded_age_sec=0.05 s",
                     "degenerate_score_age_sec=0.05 s"});
  expectContainsAll(findEvent(output.events, "LOCALIZATION_BACKEND_WARN").detail,
                    {"graph_health=0.45", "warn<0.60", "error<0.30", "last_local_reg_age_sec=0.90 s",
                     "warn>0.75 s", "error>1.50 s", "imu_spike=false"});
}

TEST(VisualizationStatusCoreTest, TimingNavMechanismAndTopicDetailsExposeEvidence) {
  VisualizationStatusCore core;
  std_msgs::msg::Header header;
  auto input = makeNominalInput();
  input.pose_age_ms.value = 240.0;
  input.compute_time_ms.value = 40.0;
  input.nav_safety.stop_required = true;
  input.nav_safety.timed_out = true;
  input.nav_safety.reason = "watchdog timeout";
  input.mechanism.comm_health_level = 1U;
  for (auto& topic : input.monitored_topics) {
    if (topic.code_suffix == "ODOM") {
      topic.age_sec = 0.80;
    }
  }

  auto output = core.evaluate(input, header);

  expectContainsAll(findEvent(output.events, "POSE_STALE").detail,
                    {"pose_age_ms=240.0 ms", "yellow=100.0 ms", "orange=160.0 ms", "red=200.0 ms"});
  expectContainsAll(findEvent(output.events, "CONTROL_OVERRUN").detail,
                    {"compute_time_ms=40.0 ms", "yellow=16.7 ms", "orange=26.7 ms", "red=33.3 ms"});
  expectContainsAll(findEvent(output.events, "NAV_STOP_REQUIRED").detail,
                    {"current_profile=normal", "stop_required=true", "timed_out=true", "reason=watchdog timeout"});
  expectContainsAll(findEvent(output.events, "NAV_TIMED_OUT").detail,
                    {"current_profile=normal", "stop_required=true", "timed_out=true", "reason=watchdog timeout"});
  expectContainsAll(findEvent(output.events, "MECHANISM_COMM_WARN").detail,
                    {"comm_health_level=1", "age_sec=0.10 s", "warn_level=1", "error_level=2", "max_age_sec=1.00 s"});
  expectContainsAll(findEvent(output.events, "TOPIC_STALE_ODOM").detail,
                    {"age=0.80 s", "limit=0.50 s"});
}

TEST(VisualizationStatusCoreTest, DisabledSubsystemsExposeMetadataWithoutTriggeringFailures) {
  VisualizationStatusConfig config;
  config.controller_present = false;
  config.keepout_present = false;
  config.nav_safety_present = false;
  config.terrain_present = false;

  VisualizationStatusCore core(config);
  std_msgs::msg::Header header;
  header.stamp.sec = 55;
  auto input = makeNominalInput();
  input.control_degraded.received = false;
  input.control_degenerate_score.received = false;
  input.compute_time_ms.received = false;
  input.pose_age_ms.received = false;
  input.collision_d_min.received = false;
  input.controller_mode.received = false;
  input.keepout.overlay_received = false;
  input.keepout.mask_received = false;
  input.keepout.heartbeat_received = false;
  input.nav_safety.received = false;
  input.terrain.obstacles_received = false;
  input.terrain.drop_received = false;
  input.terrain.grid_received = false;
  for (auto& topic : input.monitored_topics) {
    if (topic.code_suffix == "CONTROL_DEGRADED" || topic.code_suffix == "CONTROL_DEGENERATE_SCORE" ||
        topic.code_suffix == "COMPUTE_TIME_MS" || topic.code_suffix == "POSE_AGE_MS" ||
        topic.code_suffix == "COLLISION_D_MIN" || topic.code_suffix == "MOTION_MODE_STATE" ||
        topic.code_suffix == "TRACKING_STATE" || topic.code_suffix == "BLOCK_OVERLAY" ||
        topic.code_suffix == "KFS_FILTER_MASK" ||
        topic.code_suffix == "KFS_KEEPOUT_HEARTBEAT" || topic.code_suffix == "TERRAIN_OBSTACLES" ||
        topic.code_suffix == "TERRAIN_DROP" || topic.code_suffix == "TERRAIN_GRID_MAP_LOCAL") {
      topic.required = false;
      topic.received = false;
      topic.age_sec = kInf;
    }
  }

  auto output = core.evaluate(input, header);
  const auto codes = eventCodes(output.events);
  const auto& controller = findStatus(output.summary, "r2/controller");
  const auto& keepout = findStatus(output.summary, "r2/keepout");
  const auto& terrain = findStatus(output.summary, "r2/terrain");
  const auto& nav = findStatus(output.summary, "r2/nav_safety");

  EXPECT_EQ(output.operator_status.overall_level, kLevelGreen);
  EXPECT_EQ(output.operator_status.controller_level, kLevelGreen);
  EXPECT_EQ(output.operator_status.keepout_level, kLevelGreen);
  EXPECT_EQ(output.operator_status.nav_safety_level, kLevelGreen);
  EXPECT_EQ(output.operator_status.terrain_level, kLevelGreen);
  EXPECT_EQ(output.operator_status.topic_timeout_count, 0U);
  EXPECT_EQ(controller.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
  EXPECT_EQ(controller.message, "module disabled in current mode");
  EXPECT_EQ(valueForKey(controller, "present"), "false");
  EXPECT_EQ(valueForKey(controller, "enabled"), "false");
  EXPECT_EQ(valueForKey(controller, "received"), "false");
  EXPECT_TRUE(valueForKey(controller, "last_update_ms").empty());
  EXPECT_EQ(valueForKey(keepout, "present"), "false");
  EXPECT_EQ(valueForKey(terrain, "present"), "false");
  EXPECT_EQ(valueForKey(nav, "present"), "false");
  EXPECT_EQ(std::find(codes.begin(), codes.end(), "CONTROL_DEGRADED"), codes.end());
  EXPECT_EQ(std::find(codes.begin(), codes.end(), "KEEPOUT_STALE"), codes.end());
  EXPECT_EQ(std::find(codes.begin(), codes.end(), "NAV_STOP_REQUIRED"), codes.end());
}

TEST(TopicTimeoutTrackerTest, CountsOnlyFreshToStaleTransitions) {
  TopicTimeoutTracker tracker;
  auto topics = makeNominalInput().monitored_topics;

  EXPECT_EQ(tracker.observe(topics), 0U);

  for (auto& topic : topics) {
    if (topic.code_suffix == "ODOM") {
      topic.age_sec = 0.80;
    }
  }
  EXPECT_EQ(tracker.observe(topics), 1U);

  for (auto& topic : topics) {
    if (topic.code_suffix == "ODOM") {
      topic.age_sec = 0.95;
    }
  }
  EXPECT_EQ(tracker.observe(topics), 1U);

  for (auto& topic : topics) {
    if (topic.code_suffix == "ODOM") {
      topic.age_sec = 0.05;
    }
  }
  EXPECT_EQ(tracker.observe(topics), 1U);

  for (auto& topic : topics) {
    if (topic.code_suffix == "ODOM") {
      topic.age_sec = 0.75;
    }
  }
  EXPECT_EQ(tracker.observe(topics), 2U);
}

TEST(TopicTimeoutTrackerTest, ResetBaselinePreventsImmediateRecountOfCurrentStaleTopics) {
  TopicTimeoutTracker tracker;
  auto topics = makeNominalInput().monitored_topics;
  for (auto& topic : topics) {
    if (topic.code_suffix == "ODOM") {
      topic.age_sec = 0.80;
    }
  }

  EXPECT_EQ(tracker.observe(topics), 1U);
  tracker.resetBaseline(topics);
  EXPECT_EQ(tracker.total(), 0U);
  EXPECT_EQ(tracker.observe(topics), 0U);

  for (auto& topic : topics) {
    if (topic.code_suffix == "ODOM") {
      topic.age_sec = 0.05;
    }
  }
  EXPECT_EQ(tracker.observe(topics), 0U);

  for (auto& topic : topics) {
    if (topic.code_suffix == "ODOM") {
      topic.age_sec = 0.80;
    }
  }
  EXPECT_EQ(tracker.observe(topics), 1U);
}

TEST(VisualizationStatusCoreTest, EmptySourceSignalFallsBackToSystemInternal) {
  VisualizationStatusConfig config;
  config.control_degraded_topic = "";
  VisualizationStatusCore core(config);
  std_msgs::msg::Header header;
  auto input = makeNominalInput();
  input.control_degraded.value = true;

  auto output = core.evaluate(input, header);
  EXPECT_EQ(findEvent(output.events, "CONTROL_DEGRADED").source_signal, "system_internal");
}

TEST(VisualizationStatusCoreTest, ExplicitUnknownSourceFallsBackToUnnamedModule) {
  VisualizationStatusConfig config;
  config.control_degraded_topic = "Unknown Source";
  VisualizationStatusCore core(config);
  std_msgs::msg::Header header;
  auto input = makeNominalInput();
  input.control_degraded.value = true;

  auto output = core.evaluate(input, header);
  EXPECT_EQ(findEvent(output.events, "CONTROL_DEGRADED").source_signal, "unnamed_module");
}

}  // namespace
}  // namespace rc26_visualization
