#include <chrono>
#include <future>
#include <memory>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "rc26_mechanism/catalog/mechanism_command_catalog.hpp"
#include "rc26_mechanism/nodes/mechanism_lifecycle_server.hpp"
#include "rc26_serial/protocol.hpp"

namespace rc26_mechanism
{
namespace
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class MechanismCommandCatalogTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      int argc = 0;
      rclcpp::init(argc, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

TEST_F(MechanismCommandCatalogTest, CatalogDrivesExecuteSupportTerminalFeedbackAndTimeout)
{
  using CID = rc26_serial::CommandID;
  using FID = rc26_serial::FeedbackID;

  const uint8_t grab_kfs = static_cast<uint8_t>(CID::GRAB_KFS);
  const auto * entry = findMechanismCommandCatalogEntry(grab_kfs);
  ASSERT_NE(entry, nullptr);
  EXPECT_TRUE(entry->execute_supported);
  ASSERT_EQ(entry->terminal_success_feedback_ids.size(), 1U);
  EXPECT_EQ(entry->terminal_success_feedback_ids.front(), static_cast<uint8_t>(FID::GRAB_KFS_DONE));
  EXPECT_TRUE(isExecuteSupportedMechanismCommand(grab_kfs));
  EXPECT_TRUE(isTerminalSuccessFeedbackForMechanismCommand(
    grab_kfs, static_cast<uint8_t>(FID::GRAB_KFS_DONE)));
  EXPECT_FALSE(isTerminalSuccessFeedbackForMechanismCommand(
    grab_kfs, static_cast<uint8_t>(FID::MECH_UP_DUEL_DONE)));
  EXPECT_TRUE(isTerminalMechanismFeedback(static_cast<uint8_t>(FID::GRAB_KFS_DONE)));
  EXPECT_EQ(defaultTimeoutForMechanismCommand(grab_kfs), std::chrono::seconds(8));
  ASSERT_TRUE(defaultSimulatedSuccessFeedbackForMechanismCommand(grab_kfs).has_value());
  EXPECT_EQ(
    defaultSimulatedSuccessFeedbackForMechanismCommand(grab_kfs).value(),
    static_cast<uint8_t>(FID::GRAB_KFS_DONE));

  const auto * place_entry = findMechanismCommandCatalogEntry(
    static_cast<uint8_t>(CID::PLACE_KFS_GRID));
  ASSERT_NE(place_entry, nullptr);
  EXPECT_TRUE(place_entry->execute_supported);
  ASSERT_EQ(place_entry->terminal_success_feedback_ids.size(), 1U);
  EXPECT_EQ(
    place_entry->terminal_success_feedback_ids.front(),
    static_cast<uint8_t>(FID::PLACE_KFS_GRID_DONE));

  const auto * grab_tip_entry = findMechanismCommandCatalogEntry(
    static_cast<uint8_t>(CID::GRAB_TIP));
  ASSERT_NE(grab_tip_entry, nullptr);
  EXPECT_FALSE(grab_tip_entry->execute_supported);

  EXPECT_EQ(
    findMechanismCommandCatalogEntry(static_cast<uint8_t>(CID::ROTATE_POS_90)),
    nullptr);
}

TEST_F(MechanismCommandCatalogTest, NonSharedHalTypesAreRejected)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("hal_type", "sim");

  auto mechanism_server = std::make_shared<MechanismLifecycleServer>(options);
  EXPECT_EQ(
    mechanism_server->on_configure(rclcpp_lifecycle::State()),
    CallbackReturn::FAILURE);
}

}  // namespace
}  // namespace rc26_mechanism
