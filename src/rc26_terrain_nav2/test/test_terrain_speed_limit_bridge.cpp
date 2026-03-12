#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <limits>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_msgs/msg/speed_limit.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

#include "rc26_terrain_nav2/terrain_speed_limit_bridge.hpp"

namespace {

using namespace std::chrono_literals;

class TerrainSpeedLimitBridgeTest : public ::testing::Test {
protected:
    static void SetUpTestSuite() {
        if (!rclcpp::ok()) {
            int argc = 0;
            rclcpp::init(argc, nullptr);
        }
    }

    static void TearDownTestSuite() {
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
    }

    void SetUp() override {
        executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
        harness_node_ = std::make_shared<rclcpp::Node>("terrain_speed_limit_bridge_test_harness");

        rclcpp::NodeOptions options;
        options.parameter_overrides({
            rclcpp::Parameter("input_topic", "terrain_speed_limit_in"),
            rclcpp::Parameter("output_topic", "speed_limit_out"),
            rclcpp::Parameter("output_topic_compat", ""),
            rclcpp::Parameter("min_speed_limit", 0.0),
            rclcpp::Parameter("max_speed_limit", 2.0),
            rclcpp::Parameter("publish_no_limit_on_nan", true),
        });
        bridge_node_ = std::make_shared<rc26_terrain_nav2::TerrainSpeedLimitBridge>(options);

        executor_->add_node(harness_node_);
        executor_->add_node(bridge_node_);

        input_pub_ = harness_node_->create_publisher<std_msgs::msg::Float32>("terrain_speed_limit_in", 10);
        output_sub_ = harness_node_->create_subscription<nav2_msgs::msg::SpeedLimit>(
            "speed_limit_out", 10,
            [this](const nav2_msgs::msg::SpeedLimit::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(output_mutex_);
                output_msgs_.push_back(*msg);
            });

        spinFor(120ms);
    }

    void TearDown() override {
        output_sub_.reset();
        input_pub_.reset();

        if (executor_ && harness_node_) {
            executor_->remove_node(harness_node_);
        }
        if (executor_ && bridge_node_) {
            executor_->remove_node(bridge_node_);
        }

        bridge_node_.reset();
        harness_node_.reset();
        executor_.reset();

        std::lock_guard<std::mutex> lock(output_mutex_);
        output_msgs_.clear();
    }

    void spinFor(std::chrono::milliseconds duration) {
        const auto deadline = std::chrono::steady_clock::now() + duration;
        while (std::chrono::steady_clock::now() < deadline) {
            executor_->spin_some();
            std::this_thread::sleep_for(5ms);
        }
    }

    size_t outputCount() const {
        std::lock_guard<std::mutex> lock(output_mutex_);
        return output_msgs_.size();
    }

    bool waitForNewOutput(size_t before, std::chrono::milliseconds timeout) {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (std::chrono::steady_clock::now() < deadline) {
            executor_->spin_some();
            {
                std::lock_guard<std::mutex> lock(output_mutex_);
                if (output_msgs_.size() > before) {
                    return true;
                }
            }
            std::this_thread::sleep_for(5ms);
        }
        return false;
    }

    nav2_msgs::msg::SpeedLimit latestOutput() const {
        std::lock_guard<std::mutex> lock(output_mutex_);
        return output_msgs_.back();
    }

    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    rclcpp::Node::SharedPtr harness_node_;
    std::shared_ptr<rc26_terrain_nav2::TerrainSpeedLimitBridge> bridge_node_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr input_pub_;
    rclcpp::Subscription<nav2_msgs::msg::SpeedLimit>::SharedPtr output_sub_;

    mutable std::mutex output_mutex_;
    std::vector<nav2_msgs::msg::SpeedLimit> output_msgs_;
};

TEST_F(TerrainSpeedLimitBridgeTest, ConvertsFloat32ToAbsoluteSpeedLimit) {
    std_msgs::msg::Float32 input;
    input.data = 1.25f;

    const size_t before = outputCount();
    input_pub_->publish(input);
    ASSERT_TRUE(waitForNewOutput(before, 2s));

    const auto output = latestOutput();
    EXPECT_FALSE(output.percentage);
    EXPECT_NEAR(output.speed_limit, 1.25, 1e-6);
}

TEST_F(TerrainSpeedLimitBridgeTest, ClampsAndHandlesNaNAsNoLimit) {
    std_msgs::msg::Float32 high;
    high.data = 5.0f;

    const size_t before_high = outputCount();
    input_pub_->publish(high);
    ASSERT_TRUE(waitForNewOutput(before_high, 2s));
    EXPECT_NEAR(latestOutput().speed_limit, 2.0, 1e-6);

    std_msgs::msg::Float32 nan_input;
    nan_input.data = std::numeric_limits<float>::quiet_NaN();

    const size_t before_nan = outputCount();
    input_pub_->publish(nan_input);
    ASSERT_TRUE(waitForNewOutput(before_nan, 2s));
    EXPECT_NEAR(latestOutput().speed_limit, nav2_costmap_2d::NO_SPEED_LIMIT, 1e-9);
}

}  // namespace
