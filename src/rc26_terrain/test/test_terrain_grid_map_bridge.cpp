#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "grid_map_core/grid_map_core.hpp"
#include "grid_map_msgs/msg/grid_map.hpp"
#include "grid_map_ros/GridMapRosConverter.hpp"
#include "gtest/gtest.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/qos.hpp"
#include "rc26_interfaces/msg/terrain_feature_grid.hpp"
#include "rc26_terrain/terrain_grid_map_bridge.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

namespace {

using namespace std::chrono_literals;

constexpr const char* kLayerElevationAbs = "elevation_abs";
constexpr const char* kLayerElevationTopAbs = "elevation_top_abs";
constexpr const char* kLayerKfsKeepout = "kfs_keepout";
constexpr const char* kLayerBlockId = "block_id";
constexpr const char* kLayerExpectedHeight = "expected_height";
constexpr const char* kLayerHeightError = "height_error";
constexpr const char* kLayerTraversability = "traversability";

class TerrainGridMapBridgeTest : public ::testing::Test {
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
        harness_node_ = std::make_shared<rclcpp::Node>("terrain_grid_map_bridge_test_harness");

        const auto mf_layout_file =
            ament_index_cpp::get_package_share_directory("rc26_kfs_keepout") +
            "/config/mf_grid_layout.yaml";

        rclcpp::NodeOptions bridge_options;
        bridge_options.parameter_overrides({
            rclcpp::Parameter("terrain_features_topic", "terrain_features"),
            rclcpp::Parameter("kfs_mask_topic", "/kfs_filter_mask"),
            rclcpp::Parameter("output_topic", "/terrain_grid_map"),
            rclcpp::Parameter("map_frame", "map"),
            rclcpp::Parameter("base_frame", "base_link"),
            rclcpp::Parameter("tf_timeout_sec", 0.2),
            rclcpp::Parameter("keepout_stale_timeout_sec", 0.15),
            rclcpp::Parameter("enable_mf_semantics", true),
            rclcpp::Parameter("mf_grid_layout_file", mf_layout_file),
            rclcpp::Parameter("step_edge_height_thresh_m", 0.10),
            rclcpp::Parameter("slope_norm_limit", 0.35),
            rclcpp::Parameter("roughness_norm_limit", 0.08),
            rclcpp::Parameter("diagnostics_topic", "diagnostics"),
        });
        bridge_node_ = std::make_shared<rc26_terrain::TerrainGridMapBridge>(bridge_options);

        executor_->add_node(harness_node_);
        executor_->add_node(bridge_node_);

        feature_pub_ = harness_node_->create_publisher<rc26_interfaces::msg::TerrainFeatureGrid>(
            "terrain_features",
            rclcpp::QoS(rclcpp::KeepLast(5))
                .best_effort()
                .durability(rclcpp::DurabilityPolicy::Volatile));
        keepout_pub_ = harness_node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "/kfs_filter_mask",
            rclcpp::QoS(rclcpp::KeepLast(1))
                .reliable()
                .durability(rclcpp::DurabilityPolicy::TransientLocal));
        static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(harness_node_);

        grid_map_sub_ = harness_node_->create_subscription<grid_map_msgs::msg::GridMap>(
            "/terrain_grid_map",
            rclcpp::QoS(rclcpp::KeepLast(1))
                .reliable()
                .durability(rclcpp::DurabilityPolicy::TransientLocal),
            [this](const grid_map_msgs::msg::GridMap::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(map_mutex_);
                last_grid_map_msg_ = msg;
                ++grid_map_msg_count_;
            });

        spinFor(150ms);
    }

    void TearDown() override {
        grid_map_sub_.reset();
        static_tf_broadcaster_.reset();
        keepout_pub_.reset();
        feature_pub_.reset();

        if (executor_ && harness_node_) {
            executor_->remove_node(harness_node_);
        }
        if (executor_ && bridge_node_) {
            executor_->remove_node(bridge_node_);
        }

        bridge_node_.reset();
        harness_node_.reset();
        executor_.reset();

        {
            std::lock_guard<std::mutex> lock(map_mutex_);
            last_grid_map_msg_.reset();
            grid_map_msg_count_ = 0;
        }
    }

    builtin_interfaces::msg::Time nowAsMsg() const {
        const rclcpp::Time now = harness_node_->get_clock()->now();
        builtin_interfaces::msg::Time stamp;
        const int64_t ns = now.nanoseconds();
        stamp.sec = static_cast<int32_t>(ns / 1000000000LL);
        stamp.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
        return stamp;
    }

    rc26_interfaces::msg::TerrainFeatureGrid makeFeatureGrid(int width, float resolution) const {
        rc26_interfaces::msg::TerrainFeatureGrid msg;
        msg.header.stamp = nowAsMsg();
        msg.header.frame_id = "base_link";
        msg.resolution_m = resolution;
        msg.width = static_cast<uint32_t>(width);
        msg.height = static_cast<uint32_t>(width);
        msg.origin.position.x = -static_cast<double>(width / 2) * static_cast<double>(resolution);
        msg.origin.position.y = -static_cast<double>(width / 2) * static_cast<double>(resolution);
        msg.origin.orientation.w = 1.0;

        const size_t size = static_cast<size_t>(width * width);
        msg.in_radius.assign(size, 1U);
        msg.fresh.assign(size, 1U);
        msg.density.assign(size, 10U);
        msg.h_ground.assign(size, 0.0f);
        msg.sigma_h.assign(size, 0.02f);
        msg.h_top.assign(size, 0.2f);
        msg.slope_x.assign(size, 0.05f);
        msg.slope_y.assign(size, 0.03f);
        msg.roughness.assign(size, 0.01f);
        msg.p_obstacle.assign(size, 0.2f);
        msg.p_drop.assign(size, 0.1f);
        return msg;
    }

    nav_msgs::msg::OccupancyGrid makeKeepoutMask() const {
        nav_msgs::msg::OccupancyGrid grid;
        grid.header.stamp = nowAsMsg();
        grid.header.frame_id = "map";
        grid.info.resolution = 1.0f;
        grid.info.width = 3U;
        grid.info.height = 3U;
        grid.info.origin.position.x = -1.5;
        grid.info.origin.position.y = -1.5;
        grid.info.origin.orientation.w = 1.0;
        grid.data = {
            100, 0, -1,
            0, 100, 0,
            -1, 0, 100,
        };
        return grid;
    }

    void publishMapToBaseStaticTransform(double x, double y, double z, double yaw_rad) {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = nowAsMsg();
        tf.header.frame_id = "map";
        tf.child_frame_id = "base_link";
        tf.transform.translation.x = x;
        tf.transform.translation.y = y;
        tf.transform.translation.z = z;
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw_rad);
        tf.transform.rotation = tf2::toMsg(q);
        static_tf_broadcaster_->sendTransform(tf);
        spinFor(120ms);
    }

    void spinFor(std::chrono::milliseconds duration) {
        const auto deadline = std::chrono::steady_clock::now() + duration;
        while (std::chrono::steady_clock::now() < deadline) {
            executor_->spin_some();
            std::this_thread::sleep_for(5ms);
        }
    }

    size_t currentGridMapCount() const {
        std::lock_guard<std::mutex> lock(map_mutex_);
        return grid_map_msg_count_;
    }

    bool waitForNewGridMap(size_t previous_count, std::chrono::milliseconds timeout) {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (std::chrono::steady_clock::now() < deadline) {
            executor_->spin_some();
            {
                std::lock_guard<std::mutex> lock(map_mutex_);
                if (grid_map_msg_count_ > previous_count) {
                    return true;
                }
            }
            std::this_thread::sleep_for(5ms);
        }
        return false;
    }

    grid_map_msgs::msg::GridMap::SharedPtr takeLastGridMap() const {
        std::lock_guard<std::mutex> lock(map_mutex_);
        return last_grid_map_msg_;
    }

    bool convertToGridMap(const grid_map_msgs::msg::GridMap& msg, grid_map::GridMap& map) const {
        return grid_map::GridMapRosConverter::fromMessage(msg, map);
    }

    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    rclcpp::Node::SharedPtr harness_node_;
    std::shared_ptr<rc26_terrain::TerrainGridMapBridge> bridge_node_;
    rclcpp::Publisher<rc26_interfaces::msg::TerrainFeatureGrid>::SharedPtr feature_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr keepout_pub_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_sub_;

    mutable std::mutex map_mutex_;
    grid_map_msgs::msg::GridMap::SharedPtr last_grid_map_msg_;
    size_t grid_map_msg_count_{0};
};

TEST_F(TerrainGridMapBridgeTest, RecoversAbsoluteElevationLayers) {
    publishMapToBaseStaticTransform(0.0, 0.0, 1.0, 0.0);

    auto feature = makeFeatureGrid(5, 1.0f);
    const size_t center = static_cast<size_t>(2 * 5 + 2);
    feature.h_ground[center] = 0.2f;
    feature.h_top[center] = 0.5f;

    const size_t before = currentGridMapCount();
    feature_pub_->publish(feature);
    ASSERT_TRUE(waitForNewGridMap(before, 2s));

    const auto msg = takeLastGridMap();
    ASSERT_NE(msg, nullptr);
    EXPECT_EQ(msg->header.frame_id, "map");

    grid_map::GridMap map;
    ASSERT_TRUE(convertToGridMap(*msg, map));

    grid_map::Index index;
    ASSERT_TRUE(map.getIndex(grid_map::Position(0.0, 0.0), index));
    ASSERT_TRUE(map.isValid(index, kLayerElevationAbs));
    ASSERT_TRUE(map.isValid(index, kLayerElevationTopAbs));
    EXPECT_NEAR(map.at(kLayerElevationAbs, index), 1.2, 1e-4);
    EXPECT_NEAR(map.at(kLayerElevationTopAbs, index), 1.5, 1e-4);
}

TEST_F(TerrainGridMapBridgeTest, YawResamplingKeepsMostCellsFilled) {
    constexpr double kYaw45 = 0.7853981633974483;
    publishMapToBaseStaticTransform(0.0, 0.0, 0.0, kYaw45);

    auto feature = makeFeatureGrid(11, 0.4f);
    const size_t size = feature.h_ground.size();
    for (size_t i = 0; i < size; ++i) {
        feature.h_ground[i] = 0.1f;
        feature.h_top[i] = 0.3f;
    }

    const size_t before = currentGridMapCount();
    feature_pub_->publish(feature);
    ASSERT_TRUE(waitForNewGridMap(before, 2s));

    const auto msg = takeLastGridMap();
    ASSERT_NE(msg, nullptr);

    grid_map::GridMap map;
    ASSERT_TRUE(convertToGridMap(*msg, map));
    int valid_cells = 0;
    int total_cells = 0;
    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
        ++total_cells;
        if (map.isValid(*it, kLayerElevationAbs)) {
            ++valid_cells;
        }
    }

    ASSERT_GT(total_cells, 0);
    const double valid_ratio = static_cast<double>(valid_cells) / static_cast<double>(total_cells);
    EXPECT_GT(valid_ratio, 0.70);
}

TEST_F(TerrainGridMapBridgeTest, SamplesKeepoutMaskValuesCorrectly) {
    publishMapToBaseStaticTransform(0.0, 0.0, 0.0, 0.0);

    keepout_pub_->publish(makeKeepoutMask());
    spinFor(120ms);

    auto feature = makeFeatureGrid(5, 1.0f);
    const size_t before = currentGridMapCount();
    feature_pub_->publish(feature);
    ASSERT_TRUE(waitForNewGridMap(before, 2s));

    const auto msg = takeLastGridMap();
    ASSERT_NE(msg, nullptr);

    grid_map::GridMap map;
    ASSERT_TRUE(convertToGridMap(*msg, map));

    grid_map::Index idx_100;
    grid_map::Index idx_0;
    grid_map::Index idx_unknown;
    ASSERT_TRUE(map.getIndex(grid_map::Position(-1.0, -1.0), idx_100));
    ASSERT_TRUE(map.getIndex(grid_map::Position(0.0, -1.0), idx_0));
    ASSERT_TRUE(map.getIndex(grid_map::Position(1.0, -1.0), idx_unknown));

    EXPECT_NEAR(map.at(kLayerKfsKeepout, idx_100), 1.0, 1e-6);
    EXPECT_NEAR(map.at(kLayerKfsKeepout, idx_0), 0.0, 1e-6);
    EXPECT_TRUE(std::isnan(map.at(kLayerKfsKeepout, idx_unknown)));
}

TEST_F(TerrainGridMapBridgeTest, ProducesMfSemanticLayersFromLayout) {
    publishMapToBaseStaticTransform(1.2, 0.0, 0.0, 0.0);

    auto feature = makeFeatureGrid(5, 1.0f);
    const size_t center = static_cast<size_t>(2 * 5 + 2);
    feature.h_ground[center] = 0.25f;
    feature.h_top[center] = 0.45f;

    const size_t before = currentGridMapCount();
    feature_pub_->publish(feature);
    ASSERT_TRUE(waitForNewGridMap(before, 2s));

    const auto msg = takeLastGridMap();
    ASSERT_NE(msg, nullptr);

    grid_map::GridMap map;
    ASSERT_TRUE(convertToGridMap(*msg, map));

    grid_map::Index index;
    ASSERT_TRUE(map.getIndex(grid_map::Position(1.2, 0.0), index));
    ASSERT_TRUE(map.isValid(index, kLayerBlockId));
    ASSERT_TRUE(map.isValid(index, kLayerExpectedHeight));
    ASSERT_TRUE(map.isValid(index, kLayerHeightError));
    ASSERT_TRUE(map.isValid(index, kLayerTraversability));

    EXPECT_NEAR(map.at(kLayerBlockId, index), 2.0, 1e-4);
    EXPECT_NEAR(map.at(kLayerExpectedHeight, index), 0.2, 1e-4);
    EXPECT_NEAR(map.at(kLayerHeightError, index), 0.05, 1e-3);
}

TEST_F(TerrainGridMapBridgeTest, RejectsMalformedFeatureGridMessages) {
    publishMapToBaseStaticTransform(0.0, 0.0, 0.0, 0.0);

    auto malformed = makeFeatureGrid(5, 1.0f);
    malformed.h_ground.pop_back();

    const size_t before = currentGridMapCount();
    feature_pub_->publish(malformed);
    EXPECT_FALSE(waitForNewGridMap(before, 500ms));
}

TEST_F(TerrainGridMapBridgeTest, HandlesTfAndFeatureErrorsAndKeepoutStale) {
    auto feature = makeFeatureGrid(5, 1.0f);
    const size_t before_no_tf = currentGridMapCount();
    feature_pub_->publish(feature);
    EXPECT_FALSE(waitForNewGridMap(before_no_tf, 500ms));

    publishMapToBaseStaticTransform(0.0, 0.0, 0.0, 0.0);

    auto wrong_frame_feature = makeFeatureGrid(5, 1.0f);
    wrong_frame_feature.header.frame_id = "odom";
    const size_t before_wrong_frame = currentGridMapCount();
    feature_pub_->publish(wrong_frame_feature);
    EXPECT_FALSE(waitForNewGridMap(before_wrong_frame, 500ms));

    keepout_pub_->publish(makeKeepoutMask());
    spinFor(80ms);
    std::this_thread::sleep_for(250ms);

    const size_t before_stale = currentGridMapCount();
    feature_pub_->publish(makeFeatureGrid(5, 1.0f));
    ASSERT_TRUE(waitForNewGridMap(before_stale, 2s));

    const auto msg = takeLastGridMap();
    ASSERT_NE(msg, nullptr);

    grid_map::GridMap map;
    ASSERT_TRUE(convertToGridMap(*msg, map));
    grid_map::Index center;
    ASSERT_TRUE(map.getIndex(grid_map::Position(0.0, 0.0), center));
    EXPECT_TRUE(std::isnan(map.at(kLayerKfsKeepout, center)));
}

}  // namespace
