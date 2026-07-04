#include <cstdint>

#include <gtest/gtest.h>

#include "rc26_serial/protocol.hpp"

namespace {

TEST(ProtocolIDs, CommandIDsAreContinuousAfterLegacyMechanismCleanup) {
    using CID = rc26_serial::CommandID;

    EXPECT_EQ(static_cast<uint8_t>(CID::STOP), 0x00U);
    EXPECT_EQ(static_cast<uint8_t>(CID::GRAB_TIP), 0x01U);
    EXPECT_EQ(static_cast<uint8_t>(CID::GRAB_KFS_DOWN), 0x02U);
    EXPECT_EQ(static_cast<uint8_t>(CID::GRAB_KFS_UP), 0x03U);
    EXPECT_EQ(static_cast<uint8_t>(CID::ARM_RAISE), 0x04U);
    EXPECT_EQ(static_cast<uint8_t>(CID::ARM_LOWER), 0x05U);
    EXPECT_EQ(static_cast<uint8_t>(CID::PLACE_KFS_GRID), 0x06U);
    EXPECT_EQ(static_cast<uint8_t>(CID::HEARTBEAT), 0x07U);
    EXPECT_EQ(static_cast<uint8_t>(CID::FRONT_PUSHROD_EXTEND), 0x08U);
    EXPECT_EQ(static_cast<uint8_t>(CID::FRONT_PUSHROD_RETRACT), 0x09U);
    EXPECT_EQ(static_cast<uint8_t>(CID::REAR_PUSHROD_EXTEND), 0x0AU);
    EXPECT_EQ(static_cast<uint8_t>(CID::REAR_PUSHROD_RETRACT), 0x0BU);
    EXPECT_EQ(static_cast<uint8_t>(CID::POSE_TARGET), 0x0CU);
    EXPECT_EQ(static_cast<uint8_t>(CID::ARM_HIGH_RAISE), 0x0DU);
    EXPECT_EQ(static_cast<uint8_t>(CID::ARM_SECOND_LOWER), 0x0EU);
    EXPECT_EQ(static_cast<uint8_t>(CID::ENTRY_GRAB_KFS_UP), 0x0FU);
    EXPECT_EQ(static_cast<uint8_t>(CID::COMPETITION_START), 0x10U);
    EXPECT_EQ(static_cast<uint8_t>(CID::SECOND_PRESELECTION_START), 0x11U);
    EXPECT_EQ(static_cast<uint8_t>(CID::SECOND_PRESELECTION_ARM_HIGH_RAISE), 0x12U);
    EXPECT_EQ(static_cast<uint8_t>(CID::SECOND_PRESELECTION_PLACE_KFS), 0x13U);
    EXPECT_EQ(static_cast<uint8_t>(CID::SECOND_PRESELECTION_ARM_LOWER), 0x14U);
}

TEST(ProtocolIDs, FeedbackIDsAreContinuousAfterLegacyMechanismCleanup) {
    using FID = rc26_serial::FeedbackID;

    EXPECT_EQ(static_cast<uint8_t>(FID::ACK), 0x00U);
    EXPECT_EQ(static_cast<uint8_t>(FID::HEARTBEAT_ACK), 0x01U);
    EXPECT_EQ(static_cast<uint8_t>(FID::ARM_RAISE_DONE), 0x02U);
    EXPECT_EQ(static_cast<uint8_t>(FID::ARM_LOWER_DONE), 0x03U);
    EXPECT_EQ(static_cast<uint8_t>(FID::FRONT_LASER_HEIGHT_JUMP), 0x04U);
    EXPECT_EQ(static_cast<uint8_t>(FID::REAR_LASER_HEIGHT_JUMP), 0x05U);
    EXPECT_EQ(static_cast<uint8_t>(FID::FRONT_LIMIT_SWITCH_TRIGGERED), 0x06U);
    EXPECT_EQ(static_cast<uint8_t>(FID::FRONT_SECOND_LASER_HEIGHT_JUMP), 0x07U);
    EXPECT_EQ(static_cast<uint8_t>(FID::ODOM_DATA), 0x08U);
    EXPECT_EQ(static_cast<uint8_t>(FID::ARM_HIGH_RAISE_DONE), 0x09U);
    EXPECT_EQ(static_cast<uint8_t>(FID::ARM_SECOND_LOWER_DONE), 0x0AU);
    EXPECT_EQ(static_cast<uint8_t>(FID::ENTRY_GRAB_KFS_UP_DONE), 0x0BU);
    EXPECT_EQ(static_cast<uint8_t>(FID::COMPETITION_START_DONE), 0x0CU);
    EXPECT_EQ(static_cast<uint8_t>(FID::SECOND_PRESELECTION_START_DONE), 0x0DU);
    EXPECT_EQ(static_cast<uint8_t>(FID::SECOND_PRESELECTION_ARM_HIGH_RAISE_DONE), 0x0FU);
    EXPECT_EQ(static_cast<uint8_t>(FID::MF_PRESELECTION_TRIGGER), 0x10U);
    EXPECT_EQ(static_cast<uint8_t>(FID::SECOND_PRESELECTION_PICKUP_KFS_DONE), 0x11U);
    EXPECT_EQ(static_cast<uint8_t>(FID::SECOND_PRESELECTION_ARM_LOWER_DONE), 0x12U);
    EXPECT_EQ(static_cast<uint8_t>(FID::MCU_ERROR), 0xFEU);
}

TEST(ProtocolIDs, PlanarArmErrorHelpersDescribeKnownCodes) {
    using Code = rc26_serial::PlanarArmFailCode;

    EXPECT_TRUE(rc26_serial::isPlanarArmErrorPayloadSize(2));
    EXPECT_FALSE(rc26_serial::isPlanarArmErrorPayloadSize(0));
    EXPECT_TRUE(rc26_serial::isPlanarArmBusy(static_cast<uint8_t>(Code::BUSY)));
    EXPECT_FALSE(rc26_serial::isPlanarArmBusy(static_cast<uint8_t>(Code::HAL_ERROR)));

    EXPECT_STREQ(rc26_serial::commandName(static_cast<uint8_t>(
                     rc26_serial::CommandID::ARM_LOWER)),
                 "ARM_LOWER");
    EXPECT_STREQ(rc26_serial::commandName(static_cast<uint8_t>(
                     rc26_serial::CommandID::ARM_SECOND_LOWER)),
                 "GRAB_KFS_DOWN_EXTEND");
    EXPECT_STREQ(rc26_serial::commandName(static_cast<uint8_t>(
                     rc26_serial::CommandID::SECOND_PRESELECTION_ARM_LOWER)),
                 "ARM_BOTTOM_LOWER");
    EXPECT_STREQ(rc26_serial::commandName(0xEE), "UNKNOWN_COMMAND");

    EXPECT_STREQ(rc26_serial::planarArmFailCodeName(static_cast<uint8_t>(
                     Code::BUSY)),
                 "PLANAR_ARM_FAIL_BUSY");
    EXPECT_STREQ(rc26_serial::planarArmFailCodeName(static_cast<uint8_t>(
                     Code::INVALID_PAYLOAD)),
                 "PLANAR_ARM_FAIL_INVALID_PAYLOAD");
    EXPECT_STREQ(rc26_serial::planarArmFailCodeName(static_cast<uint8_t>(
                     Code::NOT_INIT)),
                 "PLANAR_ARM_FAIL_NOT_INIT");
    EXPECT_STREQ(rc26_serial::planarArmFailCodeName(static_cast<uint8_t>(
                     Code::HAL_ERROR)),
                 "PLANAR_ARM_FAIL_HAL_ERROR");
    EXPECT_STREQ(rc26_serial::planarArmFailCodeName(static_cast<uint8_t>(
                     Code::INVALID_STATE)),
                 "PLANAR_ARM_FAIL_INVALID_STATE");
    EXPECT_STREQ(rc26_serial::planarArmFailCodeName(0xEE),
                 "PLANAR_ARM_FAIL_UNKNOWN");
}

}  // namespace
