/*
 * Copyright (c) 2017, Bosch Software Innovations GmbH.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <gmock/gmock.h>

#include <csignal>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "rviz_common/ros_integration/ros_client_abstraction.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction.hpp"

using namespace ::testing;  // NOLINT

class RosNodeAbstractionTestFixture : public Test
{
protected:
  void SetUp()
  {
    rclcpp::init(0, nullptr);
  }

  void TearDown()
  {
    rclcpp::shutdown();
  }

  std::string node_name_ = "node_name";
};

TEST_F(RosNodeAbstractionTestFixture, get_node_name_returns_the_name_of_the_internal_node) {
  auto node = rviz_common::ros_integration::RosNodeAbstraction(node_name_);

  ASSERT_THAT(node.get_node_name(), Eq(node_name_));
}

TEST(RosClientAbstraction, shutdown_destroys_created_node_before_context_teardown) {
  rviz_common::ros_integration::RosClientAbstraction client;
  auto weak_node = client.init(0, nullptr, "rviz_test_node", false);

  ASSERT_FALSE(weak_node.expired());

  client.shutdown();

  EXPECT_TRUE(weak_node.expired());
  EXPECT_FALSE(rclcpp::ok());
}

TEST(RosClientAbstraction, ok_turns_false_after_sigint_without_async_rclcpp_shutdown) {
  rviz_common::ros_integration::RosClientAbstraction client;
  auto weak_node = client.init(0, nullptr, "rviz_sigint_test_node", false);

  ASSERT_FALSE(weak_node.expired());
  ASSERT_TRUE(client.ok());

  std::raise(SIGINT);

  EXPECT_FALSE(client.ok());
  EXPECT_TRUE(rclcpp::ok());

  client.shutdown();
  EXPECT_FALSE(rclcpp::ok());
  EXPECT_FALSE(weak_node.expired());
}
