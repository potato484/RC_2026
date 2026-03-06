#include <limits>

#include "gtest/gtest.h"

#include "rc26_terrain/safety_guard.hpp"

namespace rc26_terrain {
namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

TEST(SafetyGuardTest, StartupGraceSuppressesFailSafeUntilGraceEnds) {
    SafetyGuardConfig config;
    config.startup_grace_sec = 1.0;
    config.cloud_timeout_sec = 0.5;
    config.odom_timeout_sec = 0.5;
    config.tf_health_timeout_sec = 0.5;

    SafetyGuard guard(config);

    SafetyGuardInput input;
    input.since_start_sec = 0.2;
    input.received_cloud = false;
    input.cloud_age_sec = kInf;
    input.received_odom = false;
    input.odom_age_sec = kInf;
    input.need_tf = false;
    input.tf_age_sec = kInf;
    input.latency_ms = 0.0;
    input.thermal_throttle_requested = false;
    input.thermal_throttle_last_true_age_sec = kInf;

    auto decision = guard.evaluate(input);
    EXPECT_TRUE(decision.in_startup_grace);
    EXPECT_FALSE(decision.fail_safe_active);

    input.since_start_sec = 1.5;
    decision = guard.evaluate(input);
    EXPECT_FALSE(decision.in_startup_grace);
    EXPECT_TRUE(decision.fail_safe_active);
    EXPECT_EQ(decision.fail_safe_reason, "点云输入超时/中断");
    EXPECT_EQ(decision.required_intervention, SafetyIntervention::kVirtualFence);
}

TEST(SafetyGuardTest, TfTimeoutTriggersFailSafeWhenCloudIsFresh) {
    SafetyGuard guard;

    SafetyGuardInput input;
    input.since_start_sec = 2.0;
    input.received_cloud = true;
    input.cloud_age_sec = 0.1;
    input.received_odom = true;
    input.odom_age_sec = 0.1;
    input.need_tf = true;
    input.tf_age_sec = 1.2;
    input.latency_ms = 3.0;
    input.thermal_throttle_requested = false;
    input.thermal_throttle_last_true_age_sec = kInf;

    const auto decision = guard.evaluate(input);
    EXPECT_TRUE(decision.fail_safe_active);
    EXPECT_EQ(decision.fail_safe_reason, "TF 变换链异常/超时");
    EXPECT_EQ(decision.required_intervention, SafetyIntervention::kVirtualFence);
    EXPECT_EQ(decision.diagnostic_level, 2);
}

TEST(SafetyGuardTest, LatencyInterventionNeedsConsecutiveOverrunsAndRecovery) {
    SafetyGuardConfig config;
    config.latency_trigger_frames = 2;
    config.latency_recover_frames = 2;
    config.latency_warn_ms = 10.0;
    config.latency_error_ms = 20.0;
    config.enable_fail_safe = false;

    SafetyGuard guard(config);

    SafetyGuardInput input;
    input.since_start_sec = 3.0;
    input.received_cloud = true;
    input.cloud_age_sec = 0.1;
    input.received_odom = true;
    input.odom_age_sec = 0.1;
    input.need_tf = true;
    input.tf_age_sec = 0.1;
    input.thermal_throttle_requested = false;
    input.thermal_throttle_last_true_age_sec = kInf;

    input.latency_ms = 25.0;
    auto decision = guard.evaluate(input);
    EXPECT_FALSE(decision.latency_intervention_active);
    EXPECT_EQ(decision.latency_overrun_count, 1);

    decision = guard.evaluate(input);
    EXPECT_TRUE(decision.latency_intervention_active);
    EXPECT_TRUE(decision.latency_intervention_entered);
    EXPECT_EQ(decision.required_intervention, SafetyIntervention::kVirtualFence);

    input.latency_ms = 5.0;
    decision = guard.evaluate(input);
    EXPECT_TRUE(decision.latency_intervention_active);
    EXPECT_EQ(decision.latency_recover_count, 1);

    decision = guard.evaluate(input);
    EXPECT_FALSE(decision.latency_intervention_active);
    EXPECT_TRUE(decision.latency_intervention_cleared);
}

TEST(SafetyGuardTest, ThermalThrottleKeepsInterventionAliveWithinReleaseWindow) {
    SafetyGuardConfig config;
    config.enable_fail_safe = false;
    config.latency_trigger_frames = 1;
    config.latency_recover_frames = 1;
    config.thermal_throttle_release_sec = 0.5;

    SafetyGuard guard(config);

    SafetyGuardInput input;
    input.since_start_sec = 2.0;
    input.received_cloud = true;
    input.cloud_age_sec = 0.1;
    input.received_odom = true;
    input.odom_age_sec = 0.1;
    input.need_tf = true;
    input.tf_age_sec = 0.1;
    input.latency_ms = 0.0;
    input.thermal_throttle_requested = true;
    input.thermal_throttle_last_true_age_sec = 0.0;

    auto decision = guard.evaluate(input);
    EXPECT_TRUE(decision.thermal_throttle_active);
    EXPECT_TRUE(decision.latency_intervention_active);
    EXPECT_EQ(decision.required_intervention, SafetyIntervention::kVirtualFence);

    input.thermal_throttle_requested = false;
    input.thermal_throttle_last_true_age_sec = 0.3;
    decision = guard.evaluate(input);
    EXPECT_TRUE(decision.thermal_throttle_active);
    EXPECT_TRUE(decision.latency_intervention_active);

    input.thermal_throttle_last_true_age_sec = 0.8;
    decision = guard.evaluate(input);
    EXPECT_FALSE(decision.thermal_throttle_active);
    EXPECT_FALSE(decision.latency_intervention_active);
}

TEST(SafetyGuardTest, ForcedFailSafeCanBeClearedAfterRecovery) {
    SafetyGuard guard;

    auto decision = guard.forceFailSafe("TF(base) 查询失败");
    EXPECT_TRUE(decision.fail_safe_active);
    EXPECT_EQ(decision.fail_safe_reason, "default: TF(base) 查询失败");

    guard.clearForcedFailSafe();
    decision = guard.decision();
    EXPECT_FALSE(decision.fail_safe_active);

    SafetyGuardInput input;
    input.since_start_sec = 2.0;
    input.received_cloud = true;
    input.cloud_age_sec = 0.1;
    input.received_odom = true;
    input.odom_age_sec = 0.1;
    input.need_tf = true;
    input.tf_age_sec = 0.1;
    input.latency_ms = 0.0;
    input.thermal_throttle_requested = false;
    input.thermal_throttle_last_true_age_sec = kInf;

    decision = guard.evaluate(input);
    EXPECT_FALSE(decision.fail_safe_active);
    EXPECT_EQ(decision.diagnostic_level, 0);
}


TEST(SafetyGuardTest, ForcedFailSafeTracksMultipleIndependentSources) {
    SafetyGuard guard;

    auto decision = guard.forceFailSafe("tf_chain", "TF 链路验证失败");
    EXPECT_TRUE(decision.fail_safe_active);
    EXPECT_NE(decision.fail_safe_reason.find("tf_chain: TF 链路验证失败"), std::string::npos);

    decision = guard.forceFailSafe("cloud_preprocess", "点云预处理失败");
    EXPECT_TRUE(decision.fail_safe_active);
    EXPECT_NE(decision.fail_safe_reason.find("tf_chain: TF 链路验证失败"), std::string::npos);
    EXPECT_NE(decision.fail_safe_reason.find("cloud_preprocess: 点云预处理失败"), std::string::npos);

    guard.clearForcedFailSafe("tf_chain");
    decision = guard.decision();
    EXPECT_TRUE(decision.fail_safe_active);
    EXPECT_EQ(decision.fail_safe_reason, "cloud_preprocess: 点云预处理失败");

    guard.clearForcedFailSafe("cloud_preprocess");
    decision = guard.decision();
    EXPECT_FALSE(decision.fail_safe_active);
}

}  // namespace
}  // namespace rc26_terrain
