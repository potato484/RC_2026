#include <gtest/gtest.h>

#include "rc26_mcu_transport/mcu_transport_node.hpp"
#include "rc26_serial/protocol.hpp"

namespace {

TEST(McuErrorFeedbackFilter, PublishesOnlyTwoByteMcuErrorPayload) {
    using FID = rc26_serial::FeedbackID;

    EXPECT_FALSE(rc26_mcu_transport::shouldPublishTransportFeedback(
        static_cast<uint8_t>(FID::ACK), 0));
    EXPECT_FALSE(rc26_mcu_transport::shouldPublishTransportFeedback(
        static_cast<uint8_t>(FID::HEARTBEAT_ACK), 0));
    EXPECT_FALSE(rc26_mcu_transport::shouldPublishTransportFeedback(
        static_cast<uint8_t>(FID::ODOM_DATA), 16));

    EXPECT_TRUE(rc26_mcu_transport::shouldPublishTransportFeedback(
        static_cast<uint8_t>(FID::MCU_ERROR), 2));
    EXPECT_FALSE(rc26_mcu_transport::shouldPublishTransportFeedback(
        static_cast<uint8_t>(FID::MCU_ERROR), 0));
    EXPECT_FALSE(rc26_mcu_transport::shouldPublishTransportFeedback(
        static_cast<uint8_t>(FID::MCU_ERROR), 1));
    EXPECT_FALSE(rc26_mcu_transport::shouldPublishTransportFeedback(
        static_cast<uint8_t>(FID::MCU_ERROR), 3));

    EXPECT_TRUE(rc26_mcu_transport::shouldPublishTransportFeedback(
        static_cast<uint8_t>(FID::ARM_RAISE_DONE), 0));
}

TEST(McuTransportSendMode, RequestWaitAckSelectsReliableOrNoAckPath) {
    EXPECT_EQ(rc26_mcu_transport::sendModeForRequest(true),
              rc26_mcu_transport::MechanismTransportSendMode::ReliableAck);
    EXPECT_EQ(rc26_mcu_transport::sendModeForRequest(false),
              rc26_mcu_transport::MechanismTransportSendMode::NoAck);
}

}  // namespace
