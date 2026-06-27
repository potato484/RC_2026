#include "rc26_mcu_transport/chassis_cmd_vel_logic.hpp"

#include <chrono>
#include <cmath>
#include <limits>

#include <gtest/gtest.h>

namespace rc26_mcu_transport {
namespace {

using Clock = ChassisCmdVelState::Clock;

TEST(ChassisCmdVelLogic, DefaultsEnableConsumerAndUseTwoUnitLimits) {
    const ChassisCmdVelConfig config = normalizeChassisCmdVelConfig({});

    EXPECT_TRUE(config.enabled);
    EXPECT_EQ(config.topic, "cmd_vel");
    EXPECT_EQ(config.target_send_rate_hz, 50);
    EXPECT_EQ(config.cmd_vel_timeout_ms, 200);
    EXPECT_DOUBLE_EQ(config.v_max_mps, 2.0);
    EXPECT_DOUBLE_EQ(config.w_max_radps, 2.0);
    EXPECT_EQ(config.stop_repeat_n, 10);
}

TEST(ChassisCmdVelLogic, ClampsPlanarLinearVelocityToCircle) {
    ChassisCmdVelConfig config;
    config.v_max_mps = 2.0;
    config.w_max_radps = 2.0;

    const auto command = clampChassisCommand(3.0, 4.0, 3.5, config);

    EXPECT_NEAR(std::hypot(command.vx, command.vy), 2.0, 1e-6);
    EXPECT_NEAR(command.vx, 1.2, 1e-6);
    EXPECT_NEAR(command.vy, 1.6, 1e-6);
    EXPECT_NEAR(command.wz, 2.0, 1e-6);
}

TEST(ChassisCmdVelLogic, NonFiniteInputFallsBackToZero) {
    ChassisCmdVelConfig config;

    const auto command = clampChassisCommand(
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(),
        config);

    EXPECT_FLOAT_EQ(command.vx, 0.0f);
    EXPECT_FLOAT_EQ(command.vy, 0.0f);
    EXPECT_FLOAT_EQ(command.wz, 0.0f);
}

TEST(ChassisCmdVelLogic, PublishesFreshCommandUntilTimeoutThenFiniteZeros) {
    ChassisCmdVelConfig config;
    config.cmd_vel_timeout_ms = 100;
    config.stop_repeat_n = 3;
    ChassisCmdVelState state(config);
    const auto t0 = Clock::now();

    state.receive(1.0, -0.5, 0.25, t0);

    auto command = state.nextCommand(t0 + std::chrono::milliseconds(50));
    ASSERT_TRUE(command.has_value());
    EXPECT_FLOAT_EQ(command->vx, 1.0f);
    EXPECT_FLOAT_EQ(command->vy, -0.5f);
    EXPECT_FLOAT_EQ(command->wz, 0.25f);

    for (int i = 0; i < 3; ++i) {
        command = state.nextCommand(t0 + std::chrono::milliseconds(101 + i));
        ASSERT_TRUE(command.has_value());
        EXPECT_FLOAT_EQ(command->vx, 0.0f);
        EXPECT_FLOAT_EQ(command->vy, 0.0f);
        EXPECT_FLOAT_EQ(command->wz, 0.0f);
    }

    EXPECT_FALSE(state.nextCommand(t0 + std::chrono::milliseconds(110)).has_value());
}

TEST(ChassisCmdVelLogic, DisabledConfigStaysDisabledAfterNormalization) {
    ChassisCmdVelConfig config;
    config.enabled = false;

    const auto normalized = normalizeChassisCmdVelConfig(config);

    EXPECT_FALSE(normalized.enabled);
}

}  // namespace
}  // namespace rc26_mcu_transport
