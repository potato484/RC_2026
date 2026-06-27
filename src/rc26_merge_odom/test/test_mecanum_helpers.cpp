#include <array>
#include <cstdint>
#include <cstring>
#include <vector>

#include <gtest/gtest.h>

#include "rc26_merge_odom/mecanum_kinematics.hpp"
#include "rc26_merge_odom/odom_payload.hpp"

namespace rc26_merge_odom {
namespace {

std::vector<uint8_t> encodePayload(const std::array<float, 4>& wheel_speeds) {
  std::vector<uint8_t> payload(sizeof(float) * wheel_speeds.size(), 0U);
  std::memcpy(payload.data(), wheel_speeds.data(), payload.size());
  return payload;
}

TEST(MecanumHelpersTest, ParsesFourWheelPayloadInProtocolOrder) {
  const auto payload = encodePayload({1.0F, 2.0F, 3.0F, 4.0F});

  WheelSpeedPayload parsed;
  ASSERT_TRUE(parseWheelSpeedPayload(payload, parsed));
  EXPECT_DOUBLE_EQ(parsed.v_fl, 1.0);
  EXPECT_DOUBLE_EQ(parsed.v_rl, 2.0);
  EXPECT_DOUBLE_EQ(parsed.v_rr, 3.0);
  EXPECT_DOUBLE_EQ(parsed.v_fr, 4.0);
}

TEST(MecanumHelpersTest, RejectsShortPayload) {
  WheelSpeedPayload parsed;
  EXPECT_FALSE(parseWheelSpeedPayload(std::vector<uint8_t>(8U, 0U), parsed));
}

TEST(MecanumHelpersTest, ComputesHolonomicBodyVelocity) {
  const auto velocity = bodyVelocityFromWheelSpeeds(0.62326, 0.7, -1.0, 1.0, -1.0, 1.0);

  EXPECT_NEAR(velocity.vx, 0.0, 1e-9);
  EXPECT_NEAR(velocity.vy, 1.0, 1e-9);
  EXPECT_NEAR(velocity.omega, 0.0, 1e-9);
}

TEST(MecanumHelpersTest, IntegratesLateralMotionWithoutDiscardingVy) {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;

  integrateHolonomicBodyVelocity(x, y, yaw, 0.0, 1.0, 0.0, 0.5);

  EXPECT_NEAR(x, 0.0, 1e-9);
  EXPECT_NEAR(y, 0.5, 1e-9);
  EXPECT_NEAR(yaw, 0.0, 1e-9);
}

}  // namespace
}  // namespace rc26_merge_odom
