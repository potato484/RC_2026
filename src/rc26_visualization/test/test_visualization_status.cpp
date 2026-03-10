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
  input.localization.degenerate_score = 0.85;

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

  input.mechanism.received = true;
  input.mechanism.age_sec = 0.10;
  input.mechanism.comm_health_level = 0U;

  input.keepout.filter_info_received = true;
  input.keepout.filter_info_age_sec = 0.05;
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

  input.monitored_topics = {
      {"LOCALIZATION_HEALTH", "/localization/health", 1.0, true, true, 0.05},
      {"LOCALIZATION_BACKEND_STATUS", "/localization/backend_status", 1.5, true, true, 0.05},
      {"CONTROL_DEGRADED", "/control_degraded", 0.5, true, true, 0.05},
      {"CONTROL_DEGENERATE_SCORE", "control_degenerate_score", 0.5, true, true, 0.05},
      {"COMPUTE_TIME_MS", "compute_time_ms", 0.5, true, true, 0.05},
      {"POSE_AGE_MS", "pose_age_ms", 0.5, true, true, 0.05},
      {"COLLISION_D_MIN", "collision_d_min", 0.5, true, true, 0.05},
      {"NAV_SAFETY_STATE", "nav_safety_state", 2.5, true, true, 0.10},
      {"MECHANISM_STATE", "/mechanism/state", 1.0, true, true, 0.10},
      {"COSTMAP_FILTER_INFO", "/costmap_filter_info", 0.3, true, true, 0.05},
      {"KFS_FILTER_MASK", "/kfs_filter_mask", 0.3, true, true, 0.05},
      {"KFS_KEEPOUT_HEARTBEAT", "/kfs_keepout_heartbeat", 0.3, true, true, 0.05},
      {"TERRAIN_OBSTACLES", "terrain_obstacles", 1.0, true, true, 0.10},
      {"TERRAIN_DROP", "terrain_drop", 1.0, true, true, 0.10},
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
  EXPECT_EQ(output.operator_status.topic_timeout_count, 0U);
  EXPECT_TRUE(output.events.events.empty());
  EXPECT_EQ(output.summary.status.size(), 7U);
  EXPECT_EQ(valueForKey(localization, "present"), "true");
  EXPECT_EQ(valueForKey(localization, "enabled"), "true");
  EXPECT_EQ(valueForKey(localization, "received"), "true");
  EXPECT_EQ(valueForKey(localization, "source_topics"), "/localization/health,/localization/backend_status");
  EXPECT_EQ(valueForKey(localization, "last_update_ms"), "123350");
  EXPECT_EQ(valueForKey(controller, "source_topics"),
            "/control_degraded,control_degenerate_score,compute_time_ms,pose_age_ms,collision_d_min,controller_server/NMPCFollowPath/mode");
  EXPECT_EQ(valueForKey(keepout, "source_topics"), "/costmap_filter_info,/kfs_filter_mask,/kfs_keepout_heartbeat");
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
}

TEST(VisualizationStatusCoreTest, KeepoutStaleTriggersRedEvent) {
  VisualizationStatusCore core;
  std_msgs::msg::Header header;
  auto input = makeNominalInput();
  input.keepout.filter_info_age_sec = 0.45;
  input.keepout.mask_age_sec = 0.45;
  input.keepout.heartbeat_age_sec = 0.45;
  for (auto& topic : input.monitored_topics) {
    if (topic.code_suffix == "COSTMAP_FILTER_INFO" || topic.code_suffix == "KFS_FILTER_MASK" ||
        topic.code_suffix == "KFS_KEEPOUT_HEARTBEAT") {
      topic.age_sec = 0.45;
    }
  }

  auto output = core.evaluate(input, header);
  const auto codes = eventCodes(output.events);

  EXPECT_EQ(output.operator_status.keepout_level, kLevelRed);
  EXPECT_EQ(output.operator_status.overall_level, kLevelRed);
  EXPECT_NE(std::find(codes.begin(), codes.end(), "KEEPOUT_STALE"), codes.end());
}

TEST(VisualizationStatusCoreTest, LocalizationOrControlDegradeIsDetected) {
  VisualizationStatusCore core;
  std_msgs::msg::Header header;
  auto input = makeNominalInput();
  input.localization.level = kLevelRed;
  input.localization.reason = "RELOC_FAILED";
  input.localization.state = "RELOCALIZING";
  input.control_degraded.value = true;

  auto output = core.evaluate(input, header);
  const auto codes = eventCodes(output.events);

  EXPECT_EQ(output.operator_status.localization_level, kLevelRed);
  EXPECT_EQ(output.operator_status.overall_level, kLevelRed);
  EXPECT_NE(std::find(codes.begin(), codes.end(), "LOCALIZATION_DEGRADED"), codes.end());
  EXPECT_NE(std::find(codes.begin(), codes.end(), "CONTROL_DEGRADED"), codes.end());
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
  input.keepout.filter_info_received = false;
  input.keepout.mask_received = false;
  input.keepout.heartbeat_received = false;
  input.nav_safety.received = false;
  input.terrain.obstacles_received = false;
  input.terrain.drop_received = false;
  for (auto& topic : input.monitored_topics) {
    if (topic.code_suffix == "CONTROL_DEGRADED" || topic.code_suffix == "CONTROL_DEGENERATE_SCORE" ||
        topic.code_suffix == "COMPUTE_TIME_MS" || topic.code_suffix == "POSE_AGE_MS" ||
        topic.code_suffix == "COLLISION_D_MIN" || topic.code_suffix == "NAV_SAFETY_STATE" ||
        topic.code_suffix == "COSTMAP_FILTER_INFO" || topic.code_suffix == "KFS_FILTER_MASK" ||
        topic.code_suffix == "KFS_KEEPOUT_HEARTBEAT" || topic.code_suffix == "TERRAIN_OBSTACLES" ||
        topic.code_suffix == "TERRAIN_DROP") {
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

}  // namespace
}  // namespace rc26_visualization
