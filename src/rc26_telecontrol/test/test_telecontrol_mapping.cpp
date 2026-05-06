#include <memory>

#include <gtest/gtest.h>

#include "rc26_telecontrol/telecontrol_nodes.hpp"

namespace rc26_telecontrol {
namespace {

class TestableStickTelecontrolNode : public StickTelecontrolNode
{
public:
  geometry_msgs::msg::Twist compute(const JoyMsgConstSharedPtr & joy_msg)
  {
    return compute_target_twist(joy_msg);
  }
};

class TestableDpadTelecontrolNode : public DpadTelecontrolNode
{
public:
  geometry_msgs::msg::Twist compute(const JoyMsgConstSharedPtr & joy_msg)
  {
    return compute_target_twist(joy_msg);
  }
};

class TelecontrolMappingTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

TEST_F(TelecontrolMappingTest, StickModeMapsLeftStickToVxVyAndRightStickToYaw)
{
  auto joy = std::make_shared<sensor_msgs::msg::Joy>();
  joy->axes.resize(4U, 0.0F);
  joy->axes[0] = -1.0F;
  joy->axes[1] = 1.0F;
  joy->axes[3] = 1.0F;

  TestableStickTelecontrolNode node;
  const auto twist = node.compute(joy);

  EXPECT_DOUBLE_EQ(twist.linear.x, 0.2);
  EXPECT_DOUBLE_EQ(twist.linear.y, -0.2);
  EXPECT_DOUBLE_EQ(twist.angular.z, -0.5);
}

TEST_F(TelecontrolMappingTest, DpadModeMapsAxesToVxVyAndButtonsToYaw)
{
  auto joy = std::make_shared<sensor_msgs::msg::Joy>();
  joy->axes.resize(8U, 0.0F);
  joy->buttons.resize(3U, 0);
  joy->axes[6] = -1.0F;
  joy->axes[7] = 1.0F;
  joy->buttons[2] = 1;

  TestableDpadTelecontrolNode node;
  const auto twist = node.compute(joy);

  EXPECT_DOUBLE_EQ(twist.linear.x, 0.2);
  EXPECT_DOUBLE_EQ(twist.linear.y, -0.2);
  EXPECT_DOUBLE_EQ(twist.angular.z, -0.5);
}

}  // namespace
}  // namespace rc26_telecontrol
