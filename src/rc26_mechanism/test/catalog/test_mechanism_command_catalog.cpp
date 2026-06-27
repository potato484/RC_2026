#include <chrono>
#include <memory>

#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "rc26_mechanism/catalog/mechanism_command_catalog.hpp"
#include "rc26_mechanism/nodes/mechanism_lifecycle_server.hpp"

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

TEST_F(MechanismCommandCatalogTest, CatalogIsEmptyAfterRawTransportMigration)
{
  EXPECT_TRUE(mechanismCommandCatalog().empty());
  EXPECT_EQ(findMechanismCommandCatalogEntry(0x01), nullptr);
  EXPECT_FALSE(isExecuteSupportedMechanismCommand(0x06));
  EXPECT_FALSE(isTerminalSuccessFeedbackForMechanismCommand(0x06, 0x00));
  EXPECT_FALSE(isTerminalMechanismFeedback(0x00));
  EXPECT_EQ(defaultTimeoutForMechanismCommand(0x06), std::chrono::seconds(8));
  EXPECT_FALSE(defaultSimulatedSuccessFeedbackForMechanismCommand(0x06).has_value());
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
