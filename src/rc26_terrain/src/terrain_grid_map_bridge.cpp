#include "rc26_terrain/terrain_grid_map_bridge.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "grid_map_core/grid_map_core.hpp"
#include "grid_map_ros/GridMapRosConverter.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "yaml-cpp/yaml.h"

namespace {

constexpr float kNaNf = std::numeric_limits<float>::quiet_NaN();
constexpr const char* kLayerElevationAbs = "elevation_abs";
constexpr const char* kLayerElevationTopAbs = "elevation_top_abs";
constexpr const char* kLayerSigmaH = "sigma_h";
constexpr const char* kLayerFresh = "fresh";
constexpr const char* kLayerDensity = "density";
constexpr const char* kLayerSlope = "slope";
constexpr const char* kLayerSlopeX = "slope_x";
constexpr const char* kLayerSlopeY = "slope_y";
constexpr const char* kLayerRoughness = "roughness";
constexpr const char* kLayerObstacleProb = "obstacle_prob";
constexpr const char* kLayerDropProb = "drop_prob";
constexpr const char* kLayerStepUp = "step_up";
constexpr const char* kLayerClimbableProb = "climbable_prob";
constexpr const char* kLayerKfsKeepout = "kfs_keepout";

constexpr const char* kLayerBlockId = "block_id";
constexpr const char* kLayerExpectedHeight = "expected_height";
constexpr const char* kLayerHeightError = "height_error";
constexpr const char* kLayerEdgeStrength = "edge_strength";
constexpr const char* kLayerStepEdgeMask = "step_edge_mask";
constexpr const char* kLayerTraversability = "traversability";
constexpr const char* kLayerAgeSec = "age_sec";
constexpr const char* kLayerHitCount = "hit_count";
constexpr const char* kLayerElevationFused = "elevation_fused";
constexpr const char* kLayerTraversabilityFused = "traversability_fused";
constexpr const char* kLayerStepEdgeConfidence = "step_edge_confidence";

inline rclcpp::QoS makeFeatureQos() {
    return rclcpp::QoS(rclcpp::KeepLast(5))
        .best_effort()
        .durability(rclcpp::DurabilityPolicy::Volatile);
}

inline rclcpp::QoS makeKeepoutQos() {
    return rclcpp::QoS(rclcpp::KeepLast(1))
        .reliable()
        .durability(rclcpp::DurabilityPolicy::TransientLocal);
}

inline rclcpp::QoS makeOutputQos() {
    return rclcpp::QoS(rclcpp::KeepLast(1))
        .reliable()
        .durability(rclcpp::DurabilityPolicy::TransientLocal);
}

inline rclcpp::QoS makeDiagnosticsQos() {
    return rclcpp::QoS(rclcpp::KeepLast(10))
        .reliable()
        .durability(rclcpp::DurabilityPolicy::Volatile);
}

inline float clamp01(float value) {
    if (!std::isfinite(static_cast<double>(value))) {
        return 0.0f;
    }
    return std::clamp(value, 0.0f, 1.0f);
}

inline bool isFinite(float value) {
    return std::isfinite(static_cast<double>(value));
}

}  // namespace

namespace rc26_terrain {

TerrainGridMapBridge::TerrainGridMapBridge(const rclcpp::NodeOptions& options)
    : Node("terrain_grid_map_bridge", options) {
    this->declare_parameter<std::string>("terrain_features_topic", terrain_features_topic_);
    this->declare_parameter<std::string>("kfs_mask_topic", kfs_mask_topic_);
    this->declare_parameter<std::string>("output_topic", output_topic_);
    this->declare_parameter<bool>("publish_local_map", publish_local_map_);
    this->declare_parameter<std::string>("output_topic_local", output_topic_local_);
    this->declare_parameter<bool>("fusion_enable", fusion_enable_);
    this->declare_parameter<bool>("fusion_publish_raw", fusion_publish_raw_);
    this->declare_parameter<std::string>("output_topic_raw", output_topic_raw_);
    this->declare_parameter<std::string>("output_topic_local_raw", output_topic_local_raw_);
    this->declare_parameter<double>("fusion_time_constant_sec", fusion_time_constant_sec_);
    this->declare_parameter<double>("fusion_unknown_decay_sec", fusion_unknown_decay_sec_);
    this->declare_parameter<std::string>("map_frame", map_frame_);
    this->declare_parameter<std::string>("local_frame", local_frame_);
    this->declare_parameter<std::string>("base_frame", base_frame_);
    this->declare_parameter<double>("tf_timeout_sec", tf_timeout_sec_);
    this->declare_parameter<double>("keepout_stale_timeout_sec", keepout_stale_timeout_sec_);
    this->declare_parameter<bool>("enable_mf_semantics", enable_mf_semantics_);
    this->declare_parameter<std::string>("mf_grid_layout_file", mf_grid_layout_file_);
    this->declare_parameter<double>("step_edge_height_thresh_m", step_edge_height_thresh_m_);
    this->declare_parameter<double>("slope_norm_limit", slope_norm_limit_);
    this->declare_parameter<double>("roughness_norm_limit", roughness_norm_limit_);
    this->declare_parameter<double>("height_error_limit_m", height_error_limit_m_);
    this->declare_parameter<double>("traversability_height_error_weight",
                                    traversability_height_error_weight_);
    this->declare_parameter<double>("traversability_step_edge_weight",
                                    traversability_step_edge_weight_);
    this->declare_parameter<std::string>("diagnostics_topic", diagnostics_topic_);

    this->get_parameter("terrain_features_topic", terrain_features_topic_);
    this->get_parameter("kfs_mask_topic", kfs_mask_topic_);
    this->get_parameter("output_topic", output_topic_);
    this->get_parameter("publish_local_map", publish_local_map_);
    this->get_parameter("output_topic_local", output_topic_local_);
    this->get_parameter("fusion_enable", fusion_enable_);
    this->get_parameter("fusion_publish_raw", fusion_publish_raw_);
    this->get_parameter("output_topic_raw", output_topic_raw_);
    this->get_parameter("output_topic_local_raw", output_topic_local_raw_);
    this->get_parameter("fusion_time_constant_sec", fusion_time_constant_sec_);
    this->get_parameter("fusion_unknown_decay_sec", fusion_unknown_decay_sec_);
    this->get_parameter("map_frame", map_frame_);
    this->get_parameter("local_frame", local_frame_);
    this->get_parameter("base_frame", base_frame_);
    this->get_parameter("tf_timeout_sec", tf_timeout_sec_);
    this->get_parameter("keepout_stale_timeout_sec", keepout_stale_timeout_sec_);
    this->get_parameter("enable_mf_semantics", enable_mf_semantics_);
    this->get_parameter("mf_grid_layout_file", mf_grid_layout_file_);
    this->get_parameter("step_edge_height_thresh_m", step_edge_height_thresh_m_);
    this->get_parameter("slope_norm_limit", slope_norm_limit_);
    this->get_parameter("roughness_norm_limit", roughness_norm_limit_);
    this->get_parameter("height_error_limit_m", height_error_limit_m_);
    this->get_parameter("traversability_height_error_weight",
                        traversability_height_error_weight_);
    this->get_parameter("traversability_step_edge_weight",
                        traversability_step_edge_weight_);
    this->get_parameter("diagnostics_topic", diagnostics_topic_);

    tf_timeout_sec_ = std::max(0.01, tf_timeout_sec_);
    keepout_stale_timeout_sec_ = std::max(0.01, keepout_stale_timeout_sec_);
    fusion_time_constant_sec_ = std::max(1e-3, fusion_time_constant_sec_);
    fusion_unknown_decay_sec_ = std::max(0.0, fusion_unknown_decay_sec_);
    step_edge_height_thresh_m_ = std::max(0.0, step_edge_height_thresh_m_);
    slope_norm_limit_ = std::max(1e-6, slope_norm_limit_);
    roughness_norm_limit_ = std::max(1e-6, roughness_norm_limit_);
    height_error_limit_m_ = std::max(1e-6, height_error_limit_m_);
    traversability_height_error_weight_ =
        std::clamp(traversability_height_error_weight_, 0.0, 1.0);
    traversability_step_edge_weight_ =
        std::clamp(traversability_step_edge_weight_, 0.0, 1.0);

    if (enable_mf_semantics_) {
        if (mf_grid_layout_file_.empty()) {
            try {
                const auto keepout_share =
                    ament_index_cpp::get_package_share_directory("rc26_kfs_keepout");
                mf_grid_layout_file_ = keepout_share + "/config/mf_grid_layout.yaml";
            } catch (const std::exception& ex) {
                mf_layout_status_ = std::string("cannot resolve default mf layout: ") + ex.what();
                mf_layout_ready_ = false;
            }
        }
        if (!mf_grid_layout_file_.empty()) {
            mf_layout_ready_ = loadMfGridLayout(mf_grid_layout_file_);
        }
        if (!mf_layout_ready_) {
            RCLCPP_WARN(this->get_logger(),
                        "MF semantic layout unavailable, semantic layers will remain NaN: %s",
                        mf_layout_status_.empty() ? "unknown reason" : mf_layout_status_.c_str());
        }
    } else {
        mf_layout_ready_ = false;
        mf_layout_status_ = "MF semantics disabled by parameter";
    }

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    pub_grid_map_ = this->create_publisher<grid_map_msgs::msg::GridMap>(output_topic_, makeOutputQos());
    if (publish_local_map_) {
        pub_grid_map_local_ =
            this->create_publisher<grid_map_msgs::msg::GridMap>(output_topic_local_, makeOutputQos());
    }
    if (fusion_publish_raw_) {
        pub_grid_map_raw_ =
            this->create_publisher<grid_map_msgs::msg::GridMap>(output_topic_raw_, makeOutputQos());
        if (publish_local_map_) {
            pub_grid_map_local_raw_ =
                this->create_publisher<grid_map_msgs::msg::GridMap>(output_topic_local_raw_, makeOutputQos());
        }
    }
    pub_diagnostics_ =
        this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(diagnostics_topic_, makeDiagnosticsQos());

    sub_features_ = this->create_subscription<rc26_interfaces::msg::TerrainFeatureGrid>(
        terrain_features_topic_, makeFeatureQos(),
        std::bind(&TerrainGridMapBridge::featureCallback, this, std::placeholders::_1));

    sub_keepout_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        kfs_mask_topic_, makeKeepoutQos(),
        std::bind(&TerrainGridMapBridge::keepoutCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
        this->get_logger(),
        "terrain_grid_map_bridge started: feature_topic=%s keepout_topic=%s output_topic=%s map_frame=%s "
        "publish_local_map=%s output_topic_local=%s local_frame=%s fusion_enable=%s "
        "fusion_publish_raw=%s output_topic_raw=%s output_topic_local_raw=%s "
        "fusion_tau=%.2f fusion_decay=%.2f",
        terrain_features_topic_.c_str(),
        kfs_mask_topic_.c_str(),
        output_topic_.c_str(),
        map_frame_.c_str(),
        publish_local_map_ ? "true" : "false",
        output_topic_local_.c_str(),
        local_frame_.c_str(),
        fusion_enable_ ? "true" : "false",
        fusion_publish_raw_ ? "true" : "false",
        output_topic_raw_.c_str(),
        output_topic_local_raw_.c_str(),
        fusion_time_constant_sec_,
        fusion_unknown_decay_sec_);
}

void TerrainGridMapBridge::keepoutCallback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr& msg) {
    if (!msg) {
        return;
    }
    std::lock_guard<std::mutex> lock(keepout_mutex_);
    keepout_mask_ = msg;
    keepout_receive_time_ = this->get_clock()->now();
    keepout_received_ = true;
}

void TerrainGridMapBridge::featureCallback(
    const rc26_interfaces::msg::TerrainFeatureGrid::ConstSharedPtr& msg) {
    if (!msg) {
        return;
    }

    const bool stamp_is_zero = msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0U;
    const rclcpp::Time feature_stamp = stamp_is_zero ? this->get_clock()->now()
                                                     : rclcpp::Time(msg->header.stamp);

    DiagnosticsState diagnostics;
    std::string invalid_reason;
    if (!validateFeatureMessage(*msg, invalid_reason)) {
        diagnostics.feature_valid = false;
        diagnostics.detail = invalid_reason;
        publishDiagnostics(feature_stamp, diagnostics);
        return;
    }
    if (msg->header.frame_id != base_frame_) {
        diagnostics.feature_valid = false;
        diagnostics.detail = "feature frame_id mismatch, expected " + base_frame_ +
                             ", got " + msg->header.frame_id;
        publishDiagnostics(feature_stamp, diagnostics);
        return;
    }

    geometry_msgs::msg::TransformStamped tf_map_base;
    try {
        tf_map_base = tf_buffer_->lookupTransform(
            map_frame_, base_frame_, feature_stamp,
            rclcpp::Duration::from_seconds(tf_timeout_sec_));
    } catch (const tf2::TransformException& ex) {
        diagnostics.tf_ok = false;
        diagnostics.detail = std::string("lookup ") + map_frame_ + "<-" + base_frame_ + " failed: " + ex.what();
        publishDiagnostics(feature_stamp, diagnostics);
        return;
    }

    const auto toPose2D = [](const geometry_msgs::msg::TransformStamped& tf_msg,
                             double& out_x,
                             double& out_y,
                             double& out_z,
                             double& out_cos_yaw,
                             double& out_sin_yaw) {
        out_x = tf_msg.transform.translation.x;
        out_y = tf_msg.transform.translation.y;
        out_z = tf_msg.transform.translation.z;

        tf2::Quaternion q;
        tf2::fromMsg(tf_msg.transform.rotation, q);
        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 0.0;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        (void)roll;
        (void)pitch;
        out_cos_yaw = std::cos(yaw);
        out_sin_yaw = std::sin(yaw);
    };

    double base_x_map = 0.0;
    double base_y_map = 0.0;
    double base_z_map = 0.0;
    double cos_yaw_map = 1.0;
    double sin_yaw_map = 0.0;
    toPose2D(tf_map_base, base_x_map, base_y_map, base_z_map, cos_yaw_map, sin_yaw_map);

    double base_x_local = base_x_map;
    double base_y_local = base_y_map;
    double base_z_local = base_z_map;
    double cos_yaw_local = cos_yaw_map;
    double sin_yaw_local = sin_yaw_map;

    if (publish_local_map_) {
        if (local_frame_ == map_frame_) {
            base_x_local = base_x_map;
            base_y_local = base_y_map;
            base_z_local = base_z_map;
            cos_yaw_local = cos_yaw_map;
            sin_yaw_local = sin_yaw_map;
        } else {
            geometry_msgs::msg::TransformStamped tf_local_base;
            try {
                tf_local_base = tf_buffer_->lookupTransform(
                    local_frame_, base_frame_, feature_stamp,
                    rclcpp::Duration::from_seconds(tf_timeout_sec_));
            } catch (const tf2::TransformException& ex) {
                diagnostics.tf_ok = false;
                diagnostics.detail =
                    std::string("lookup ") + local_frame_ + "<-" + base_frame_ + " failed: " + ex.what();
                publishDiagnostics(feature_stamp, diagnostics);
                return;
            }
            toPose2D(tf_local_base, base_x_local, base_y_local, base_z_local, cos_yaw_local, sin_yaw_local);
        }
    }

    const int width = static_cast<int>(msg->width);
    const int height = static_cast<int>(msg->height);
    const int half_width = width / 2;
    const double resolution = static_cast<double>(msg->resolution_m);

    std::vector<std::string> layers = {
        kLayerElevationAbs,
        kLayerElevationTopAbs,
        kLayerSigmaH,
        kLayerFresh,
        kLayerDensity,
        kLayerSlope,
        kLayerSlopeX,
        kLayerSlopeY,
        kLayerRoughness,
        kLayerObstacleProb,
        kLayerDropProb,
        kLayerStepUp,
        kLayerClimbableProb,
        kLayerKfsKeepout,
        kLayerTraversability,
    };
    if (fusion_enable_) {
        layers.push_back(kLayerAgeSec);
        layers.push_back(kLayerHitCount);
        layers.push_back(kLayerElevationFused);
        layers.push_back(kLayerTraversabilityFused);
        layers.push_back(kLayerStepEdgeConfidence);
    }
    if (enable_mf_semantics_) {
        layers.push_back(kLayerBlockId);
        layers.push_back(kLayerExpectedHeight);
        layers.push_back(kLayerHeightError);
        layers.push_back(kLayerEdgeStrength);
        layers.push_back(kLayerStepEdgeMask);
    }

    nav_msgs::msg::OccupancyGrid::ConstSharedPtr keepout_mask;
    double keepout_age_sec = std::numeric_limits<double>::infinity();
    {
        std::lock_guard<std::mutex> lock(keepout_mutex_);
        keepout_mask = keepout_mask_;
        if (keepout_received_) {
            keepout_age_sec = (this->get_clock()->now() - keepout_receive_time_).seconds();
        }
    }

    const bool keepout_available = keepout_received_ && keepout_mask &&
                                   (keepout_age_sec <= keepout_stale_timeout_sec_);
    if (!keepout_received_ || !keepout_mask) {
        diagnostics.keepout_available = false;
    } else if (!keepout_available) {
        diagnostics.keepout_available = false;
        diagnostics.keepout_stale = true;
    }

    const bool semantics_active = enable_mf_semantics_ && mf_layout_ready_;
    if (enable_mf_semantics_) {
        if (!mf_layout_ready_) {
            diagnostics.mf_layout_ready = false;
        } else {
            const bool team_valid = (mf_layout_team_ == "blue" || mf_layout_team_ == "red");
            if (!team_valid) {
                diagnostics.team_valid = false;
            }
        }
    }

    const auto srcIndex = [width](int ix, int iy) -> size_t {
        return static_cast<size_t>(ix * width + iy);
    };

    const auto buildOutputMap = [&](const std::string& frame_id,
                                    double base_x,
                                    double base_y,
                                    double base_z,
                                    double cos_yaw,
                                    double sin_yaw,
                                    bool sample_keepout) {
        grid_map::GridMap output_map(layers);
        output_map.setFrameId(frame_id);
        const int64_t stamp_ns = feature_stamp.nanoseconds();
        output_map.setTimestamp(static_cast<grid_map::Time>(std::max<int64_t>(0, stamp_ns)));
        output_map.setGeometry(grid_map::Length(width * resolution, height * resolution),
                               resolution,
                               grid_map::Position(base_x, base_y));
        output_map.setBasicLayers({kLayerElevationAbs});

        for (const auto& layer : layers) {
            output_map[layer].setConstant(kNaNf);
        }
        output_map[kLayerFresh].setZero();

        for (grid_map::GridMapIterator it(output_map); !it.isPastEnd(); ++it) {
            const grid_map::Index index(*it);
            grid_map::Position position;
            if (!output_map.getPosition(index, position)) {
                continue;
            }

            const double dx = position.x() - base_x;
            const double dy = position.y() - base_y;
            const double x_bl = cos_yaw * dx + sin_yaw * dy;
            const double y_bl = -sin_yaw * dx + cos_yaw * dy;

            const int ix = static_cast<int>(std::llround(x_bl / resolution)) + half_width;
            const int iy = static_cast<int>(std::llround(y_bl / resolution)) + half_width;
            if (ix < 0 || ix >= width || iy < 0 || iy >= height) {
                continue;
            }

            const size_t source_idx = srcIndex(ix, iy);
            if (source_idx >= msg->in_radius.size() || msg->in_radius[source_idx] == 0U) {
                continue;
            }

            const bool fresh = msg->fresh[source_idx] != 0U;
            output_map.at(kLayerFresh, index) = fresh ? 1.0f : 0.0f;
            output_map.at(kLayerObstacleProb, index) = msg->p_obstacle[source_idx];
            output_map.at(kLayerDropProb, index) = msg->p_drop[source_idx];
            output_map.at(kLayerStepUp, index) = msg->step_up[source_idx];
            output_map.at(kLayerClimbableProb, index) = msg->p_climbable[source_idx];

            if (sample_keepout && keepout_available) {
                const auto keepout_value = sampleKeepoutValue(*keepout_mask, position.x(), position.y());
                if (keepout_value.has_value()) {
                    output_map.at(kLayerKfsKeepout, index) = *keepout_value;
                }
            }

            if (!fresh) {
                continue;
            }

            output_map.at(kLayerElevationAbs, index) =
                static_cast<float>(base_z) + msg->h_ground[source_idx];
            output_map.at(kLayerElevationTopAbs, index) =
                static_cast<float>(base_z) + msg->h_top[source_idx];
            output_map.at(kLayerSigmaH, index) = msg->sigma_h[source_idx];
            output_map.at(kLayerDensity, index) = static_cast<float>(msg->density[source_idx]);
            output_map.at(kLayerSlopeX, index) = msg->slope_x[source_idx];
            output_map.at(kLayerSlopeY, index) = msg->slope_y[source_idx];
            output_map.at(kLayerSlope, index) =
                std::max(std::abs(msg->slope_x[source_idx]), std::abs(msg->slope_y[source_idx]));
            output_map.at(kLayerRoughness, index) = msg->roughness[source_idx];
        }

        if (semantics_active) {
            for (grid_map::GridMapIterator it(output_map); !it.isPastEnd(); ++it) {
                const grid_map::Index index(*it);
                grid_map::Position position;
                if (!output_map.getPosition(index, position)) {
                    continue;
                }

                const int block_id = resolveBlockId(position.x(), position.y());
                if (block_id <= 0) {
                    continue;
                }

                output_map.at(kLayerBlockId, index) = static_cast<float>(block_id);
                const auto expected_height = expectedHeightForGridId(block_id);
                if (!expected_height.has_value()) {
                    continue;
                }

                output_map.at(kLayerExpectedHeight, index) = static_cast<float>(*expected_height);
                if (output_map.isValid(index, kLayerElevationAbs)) {
                    output_map.at(kLayerHeightError, index) =
                        output_map.at(kLayerElevationAbs, index) -
                        static_cast<float>(*expected_height);
                }
            }

            const grid_map::Size map_size = output_map.getSize();
            const std::array<grid_map::Index, 4> kNeighbors = {
                grid_map::Index(1, 0),
                grid_map::Index(-1, 0),
                grid_map::Index(0, 1),
                grid_map::Index(0, -1),
            };

            for (grid_map::GridMapIterator it(output_map); !it.isPastEnd(); ++it) {
                const grid_map::Index index(*it);
                if (!output_map.isValid(index, kLayerElevationAbs)) {
                    continue;
                }

                const float center_elevation = output_map.at(kLayerElevationAbs, index);
                float edge_strength = 0.0f;
                bool has_neighbor = false;
                for (const auto& offset : kNeighbors) {
                    const grid_map::Index neighbor = index + offset;
                    if (neighbor(0) < 0 || neighbor(0) >= map_size(0) ||
                        neighbor(1) < 0 || neighbor(1) >= map_size(1)) {
                        continue;
                    }
                    if (!output_map.isValid(neighbor, kLayerElevationAbs)) {
                        continue;
                    }
                    const float neighbor_elevation = output_map.at(kLayerElevationAbs, neighbor);
                    edge_strength = std::max(edge_strength, std::abs(neighbor_elevation - center_elevation));
                    has_neighbor = true;
                }
                if (has_neighbor) {
                    output_map.at(kLayerEdgeStrength, index) = edge_strength;
                    output_map.at(kLayerStepEdgeMask, index) =
                        edge_strength >= static_cast<float>(step_edge_height_thresh_m_) ? 1.0f : 0.0f;
                }
            }
        }

        for (grid_map::GridMapIterator it(output_map); !it.isPastEnd(); ++it) {
            const grid_map::Index index(*it);
            if (output_map.at(kLayerFresh, index) <= 0.5f) {
                continue;
            }

            const float slope = output_map.at(kLayerSlope, index);
            const float roughness = output_map.at(kLayerRoughness, index);
            const float obstacle_prob = output_map.at(kLayerObstacleProb, index);
            const float drop_prob = output_map.at(kLayerDropProb, index);
            if (!isFinite(slope) || !isFinite(roughness) ||
                !isFinite(obstacle_prob) || !isFinite(drop_prob)) {
                continue;
            }

            const float slope_norm = clamp01(slope / static_cast<float>(slope_norm_limit_));
            const float roughness_norm = clamp01(roughness / static_cast<float>(roughness_norm_limit_));
            float max_term = 0.0f;
            max_term = std::max(max_term, 0.35f * slope_norm);
            max_term = std::max(max_term, 0.20f * roughness_norm);
            max_term = std::max(max_term, 0.25f * clamp01(obstacle_prob));
            max_term = std::max(max_term, 0.35f * clamp01(drop_prob));
            if (semantics_active) {
                const float height_error = output_map.at(kLayerHeightError, index);
                if (isFinite(height_error)) {
                    const float err_norm = clamp01(
                        std::abs(height_error) / static_cast<float>(height_error_limit_m_));
                    max_term = std::max(
                        max_term,
                        static_cast<float>(traversability_height_error_weight_) * err_norm);
                }
                const float step_edge = output_map.at(kLayerStepEdgeMask, index);
                if (isFinite(step_edge)) {
                    max_term = std::max(
                        max_term,
                        static_cast<float>(traversability_step_edge_weight_) * clamp01(step_edge));
                }
            }

            output_map.at(kLayerTraversability, index) = clamp01(1.0f - max_term);
        }

        return output_map;
    };

    auto output_map_raw = buildOutputMap(map_frame_,
                                         base_x_map,
                                         base_y_map,
                                         base_z_map,
                                         cos_yaw_map,
                                         sin_yaw_map,
                                         true);
    auto output_map = output_map_raw;

    if (fusion_enable_) {
        const double dt_sec =
            (last_fusion_stamp_.nanoseconds() > 0 &&
             feature_stamp.nanoseconds() > last_fusion_stamp_.nanoseconds())
                ? (feature_stamp - last_fusion_stamp_).seconds()
                : fusion_time_constant_sec_;
        const float alpha = static_cast<float>(std::clamp(
            dt_sec / std::max(1e-3, fusion_time_constant_sec_), 0.0, 1.0));

        const std::array<const char*, 14> kBlendLayers = {
            kLayerElevationAbs,     kLayerElevationTopAbs, kLayerSigmaH,      kLayerDensity,
            kLayerSlope,            kLayerSlopeX,          kLayerSlopeY,      kLayerRoughness,
            kLayerObstacleProb,     kLayerDropProb,        kLayerStepUp,      kLayerClimbableProb,
            kLayerTraversability,   kLayerEdgeStrength,
        };
        const std::array<const char*, 4> kCarryOnlyLayers = {
            kLayerBlockId, kLayerExpectedHeight, kLayerHeightError, kLayerStepEdgeMask,
        };

        const auto readPrevValue = [&](const char* layer,
                                       const grid_map::Position& position,
                                       float& value) -> bool {
            if (!fused_map_.has_value() || !fused_map_->exists(layer)) {
                return false;
            }
            grid_map::Index prev_index;
            if (!fused_map_->getIndex(position, prev_index)) {
                return false;
            }
            if (!fused_map_->isValid(prev_index, layer)) {
                return false;
            }
            value = fused_map_->at(layer, prev_index);
            return isFinite(value);
        };

        for (grid_map::GridMapIterator it(output_map); !it.isPastEnd(); ++it) {
            const grid_map::Index index(*it);
            grid_map::Position position;
            if (!output_map.getPosition(index, position)) {
                continue;
            }

            const bool raw_fresh = output_map.at(kLayerFresh, index) > 0.5f;
            float prev_age = 0.0f;
            float prev_hit_count = 0.0f;
            const bool has_prev_age = readPrevValue(kLayerAgeSec, position, prev_age);
            const bool has_prev_hit = readPrevValue(kLayerHitCount, position, prev_hit_count);
            const float updated_hit = has_prev_hit ? prev_hit_count + 1.0f : 1.0f;

            auto blendLayerIfPossible = [&](const char* layer) {
                if (!output_map.exists(layer)) {
                    return;
                }
                if (!output_map.isValid(index, layer)) {
                    return;
                }
                float prev_value = 0.0f;
                if (!readPrevValue(layer, position, prev_value)) {
                    return;
                }
                const float raw_value = output_map.at(layer, index);
                output_map.at(layer, index) = (1.0f - alpha) * prev_value + alpha * raw_value;
            };

            auto copyPreviousIfExists = [&](const char* layer) {
                if (!output_map.exists(layer)) {
                    return;
                }
                float prev_value = 0.0f;
                if (readPrevValue(layer, position, prev_value)) {
                    output_map.at(layer, index) = prev_value;
                } else {
                    output_map.at(layer, index) = kNaNf;
                }
            };

            if (raw_fresh) {
                for (const auto* layer : kBlendLayers) {
                    blendLayerIfPossible(layer);
                }
                for (const auto* layer : kCarryOnlyLayers) {
                    if (!output_map.exists(layer) || !output_map.isValid(index, layer)) {
                        copyPreviousIfExists(layer);
                    }
                }

                float prev_conf = 0.0f;
                const bool has_prev_conf = readPrevValue(kLayerStepEdgeConfidence, position, prev_conf);
                float edge_mask = 0.0f;
                if (output_map.exists(kLayerStepEdgeMask) && output_map.isValid(index, kLayerStepEdgeMask)) {
                    edge_mask = clamp01(output_map.at(kLayerStepEdgeMask, index));
                }
                output_map.at(kLayerStepEdgeConfidence, index) =
                    has_prev_conf ? ((1.0f - alpha) * prev_conf + alpha * edge_mask) : edge_mask;
                output_map.at(kLayerAgeSec, index) = 0.0f;
                output_map.at(kLayerHitCount, index) = updated_hit;
            } else {
                const float age_next = has_prev_age ? (prev_age + static_cast<float>(dt_sec))
                                                    : std::numeric_limits<float>::infinity();
                const bool keep_memory =
                    has_prev_age && std::isfinite(age_next) &&
                    age_next <= static_cast<float>(fusion_unknown_decay_sec_);
                if (keep_memory) {
                    for (const auto* layer : kBlendLayers) {
                        copyPreviousIfExists(layer);
                    }
                    for (const auto* layer : kCarryOnlyLayers) {
                        copyPreviousIfExists(layer);
                    }
                    float prev_conf = 0.0f;
                    if (readPrevValue(kLayerStepEdgeConfidence, position, prev_conf)) {
                        const float decay =
                            static_cast<float>(std::exp(-dt_sec / fusion_time_constant_sec_));
                        output_map.at(kLayerStepEdgeConfidence, index) = prev_conf * decay;
                    } else {
                        output_map.at(kLayerStepEdgeConfidence, index) = kNaNf;
                    }
                    output_map.at(kLayerFresh, index) = 1.0f;
                    output_map.at(kLayerAgeSec, index) = age_next;
                    output_map.at(kLayerHitCount, index) = has_prev_hit ? prev_hit_count : 0.0f;
                } else {
                    for (const auto* layer : kBlendLayers) {
                        if (output_map.exists(layer)) {
                            output_map.at(layer, index) = kNaNf;
                        }
                    }
                    for (const auto* layer : kCarryOnlyLayers) {
                        if (output_map.exists(layer)) {
                            output_map.at(layer, index) = kNaNf;
                        }
                    }
                    if (output_map.exists(kLayerStepEdgeConfidence)) {
                        output_map.at(kLayerStepEdgeConfidence, index) = kNaNf;
                    }
                    output_map.at(kLayerFresh, index) = 0.0f;
                    output_map.at(kLayerAgeSec, index) = age_next;
                    output_map.at(kLayerHitCount, index) = has_prev_hit ? prev_hit_count : 0.0f;
                }
            }

            if (output_map.exists(kLayerElevationFused)) {
                if (output_map.isValid(index, kLayerElevationAbs)) {
                    output_map.at(kLayerElevationFused, index) =
                        output_map.at(kLayerElevationAbs, index);
                } else {
                    output_map.at(kLayerElevationFused, index) = kNaNf;
                }
            }
            if (output_map.exists(kLayerTraversabilityFused)) {
                if (output_map.isValid(index, kLayerTraversability)) {
                    output_map.at(kLayerTraversabilityFused, index) =
                        output_map.at(kLayerTraversability, index);
                } else {
                    output_map.at(kLayerTraversabilityFused, index) = kNaNf;
                }
            }
        }

        fused_map_ = output_map;
        last_fusion_stamp_ = feature_stamp;
    } else {
        fused_map_.reset();
        last_fusion_stamp_ = feature_stamp;
    }

    if (fusion_publish_raw_ && pub_grid_map_raw_) {
        auto output_msg_raw = grid_map::GridMapRosConverter::toMessage(output_map_raw);
        if (output_msg_raw) {
            pub_grid_map_raw_->publish(std::move(*output_msg_raw));
        }
    }

    auto output_msg = grid_map::GridMapRosConverter::toMessage(output_map);
    if (output_msg) {
        pub_grid_map_->publish(std::move(*output_msg));
    }

    if (publish_local_map_ && pub_grid_map_local_) {
        const bool sample_local_keepout = keepout_available && (local_frame_ == map_frame_);
        auto output_map_local_raw = buildOutputMap(local_frame_,
                                                   base_x_local,
                                                   base_y_local,
                                                   base_z_local,
                                                   cos_yaw_local,
                                                   sin_yaw_local,
                                                   sample_local_keepout);
        if (fusion_publish_raw_ && pub_grid_map_local_raw_) {
            auto output_msg_local_raw = grid_map::GridMapRosConverter::toMessage(output_map_local_raw);
            if (output_msg_local_raw) {
                pub_grid_map_local_raw_->publish(std::move(*output_msg_local_raw));
            }
        }
        auto output_msg_local = grid_map::GridMapRosConverter::toMessage(output_map_local_raw);
        if (output_msg_local) {
            pub_grid_map_local_->publish(std::move(*output_msg_local));
        }
    }

    publishDiagnostics(feature_stamp, diagnostics);
}

bool TerrainGridMapBridge::validateFeatureMessage(
    const rc26_interfaces::msg::TerrainFeatureGrid& msg,
    std::string& reason) const {
    if (msg.width == 0U || msg.height == 0U) {
        reason = "feature grid width/height must be > 0";
        return false;
    }
    if (msg.width != msg.height) {
        reason = "feature grid must be square";
        return false;
    }
    if ((msg.width % 2U) == 0U) {
        reason = "feature grid width must be odd";
        return false;
    }
    if (!std::isfinite(static_cast<double>(msg.resolution_m)) || msg.resolution_m <= 0.0f) {
        reason = "feature grid resolution_m must be finite and > 0";
        return false;
    }
    if (msg.header.frame_id.empty()) {
        reason = "feature header.frame_id is empty";
        return false;
    }

    const size_t expected_size = static_cast<size_t>(msg.width) * static_cast<size_t>(msg.height);
    const auto check_size = [&reason, expected_size](size_t actual, const std::string& field) -> bool {
        if (actual == expected_size) {
            return true;
        }
        std::ostringstream oss;
        oss << "feature field '" << field << "' size mismatch, expected "
            << expected_size << " got " << actual;
        reason = oss.str();
        return false;
    };

    if (!check_size(msg.in_radius.size(), "in_radius")) return false;
    if (!check_size(msg.fresh.size(), "fresh")) return false;
    if (!check_size(msg.density.size(), "density")) return false;
    if (!check_size(msg.h_ground.size(), "h_ground")) return false;
    if (!check_size(msg.sigma_h.size(), "sigma_h")) return false;
    if (!check_size(msg.h_top.size(), "h_top")) return false;
    if (!check_size(msg.slope_x.size(), "slope_x")) return false;
    if (!check_size(msg.slope_y.size(), "slope_y")) return false;
    if (!check_size(msg.roughness.size(), "roughness")) return false;
    if (!check_size(msg.p_obstacle.size(), "p_obstacle")) return false;
    if (!check_size(msg.p_drop.size(), "p_drop")) return false;
    if (!check_size(msg.step_up.size(), "step_up")) return false;
    if (!check_size(msg.p_climbable.size(), "p_climbable")) return false;

    return true;
}

bool TerrainGridMapBridge::loadMfGridLayout(const std::string& path) {
    try {
        YAML::Node root = YAML::LoadFile(path);
        if (!root["meta"] || !root["grids"]) {
            mf_layout_status_ = "missing meta/grids in layout file";
            return false;
        }

        const YAML::Node meta = root["meta"];
        if (!meta["team"] || !meta["grid_spacing_m"]) {
            mf_layout_status_ = "meta.team/meta.grid_spacing_m required";
            return false;
        }

        mf_layout_team_ = toLower(meta["team"].as<std::string>());
        mf_grid_spacing_m_ = meta["grid_spacing_m"].as<double>();
        if (!std::isfinite(mf_grid_spacing_m_) || mf_grid_spacing_m_ <= 0.0) {
            mf_layout_status_ = "meta.grid_spacing_m must be finite and > 0";
            return false;
        }
        mf_block_half_extent_m_ = mf_grid_spacing_m_ * 0.5;

        for (auto& cell : mf_cells_) {
            cell = MfCell{};
        }

        std::array<bool, 13> seen{};
        for (const auto& grid : root["grids"]) {
            const int id = grid["id"].as<int>();
            if (id < 1 || id > 12) {
                mf_layout_status_ = "grid id out of range";
                return false;
            }
            const double x = grid["x"].as<double>();
            const double y = grid["y"].as<double>();
            if (!std::isfinite(x) || !std::isfinite(y)) {
                mf_layout_status_ = "grid x/y must be finite";
                return false;
            }
            mf_cells_[static_cast<size_t>(id)] = MfCell{x, y, true};
            seen[static_cast<size_t>(id)] = true;
        }

        for (int id = 1; id <= 12; ++id) {
            if (!seen[static_cast<size_t>(id)]) {
                mf_layout_status_ = "grid ids 1..12 must all exist";
                return false;
            }
        }

        mf_layout_status_ = "ok";
        return true;
    } catch (const std::exception& ex) {
        mf_layout_status_ = std::string("yaml parse error: ") + ex.what();
        return false;
    }
}

std::optional<double> TerrainGridMapBridge::expectedHeightForGridId(int grid_id) const {
    if (grid_id < 1 || grid_id > 12) {
        return std::nullopt;
    }

    static const std::array<int, 13> kBlueDepth = {0, 2, 1, 2, 1, 2, 3, 2, 3, 2, 1, 2, 1};
    static const std::array<int, 13> kRedDepth = {0, 2, 1, 2, 3, 2, 1, 2, 3, 2, 1, 2, 1};

    int depth = 0;
    if (mf_layout_team_ == "blue") {
        depth = kBlueDepth[static_cast<size_t>(grid_id)];
    } else if (mf_layout_team_ == "red") {
        depth = kRedDepth[static_cast<size_t>(grid_id)];
    } else {
        return std::nullopt;
    }
    if (depth <= 0) {
        return std::nullopt;
    }
    return static_cast<double>(depth) * 0.2;
}

int TerrainGridMapBridge::resolveBlockId(double x_map, double y_map) const {
    for (int id = 1; id <= 12; ++id) {
        const auto& cell = mf_cells_[static_cast<size_t>(id)];
        if (!cell.valid) {
            continue;
        }
        if (std::abs(x_map - cell.x) <= mf_block_half_extent_m_ &&
            std::abs(y_map - cell.y) <= mf_block_half_extent_m_) {
            return id;
        }
    }
    return -1;
}

std::optional<float> TerrainGridMapBridge::sampleKeepoutValue(
    const nav_msgs::msg::OccupancyGrid& grid, double x_map, double y_map) const {
    if (grid.info.width == 0U || grid.info.height == 0U || grid.info.resolution <= 0.0f) {
        return std::nullopt;
    }
    if (grid.data.size() < static_cast<size_t>(grid.info.width) * static_cast<size_t>(grid.info.height)) {
        return std::nullopt;
    }

    const double origin_x = grid.info.origin.position.x;
    const double origin_y = grid.info.origin.position.y;
    const double resolution = static_cast<double>(grid.info.resolution);
    const int px = static_cast<int>(std::floor((x_map - origin_x) / resolution));
    const int py = static_cast<int>(std::floor((y_map - origin_y) / resolution));
    if (px < 0 || py < 0 ||
        px >= static_cast<int>(grid.info.width) ||
        py >= static_cast<int>(grid.info.height)) {
        return std::nullopt;
    }

    const size_t index =
        static_cast<size_t>(py) * static_cast<size_t>(grid.info.width) + static_cast<size_t>(px);
    const int8_t value = grid.data[index];
    if (value == 100) {
        return 1.0f;
    }
    if (value == 0) {
        return 0.0f;
    }
    return std::nullopt;
}

void TerrainGridMapBridge::publishDiagnostics(const rclcpp::Time& stamp,
                                              const DiagnosticsState& state) const {
    if (!pub_diagnostics_) {
        return;
    }

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = this->get_fully_qualified_name();
    status.hardware_id = "R2";

    if (!state.feature_valid || !state.tf_ok) {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        status.message = state.detail.empty() ? "grid_map_bridge error" : state.detail;
    } else {
        std::vector<std::string> warnings;
        if (!state.keepout_available) {
            warnings.emplace_back(state.keepout_stale ? "keepout stale" : "keepout unavailable");
        }
        if (!state.mf_layout_ready) {
            warnings.emplace_back("mf layout unavailable");
        }
        if (!state.team_valid) {
            warnings.emplace_back("mf team invalid");
        }

        if (!warnings.empty()) {
            status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            std::ostringstream oss;
            for (size_t i = 0; i < warnings.size(); ++i) {
                if (i != 0U) {
                    oss << "; ";
                }
                oss << warnings[i];
            }
            status.message = oss.str();
        } else {
            status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
            status.message = "ok";
        }
    }

    auto add_key_value = [&](const std::string& key, const std::string& value) {
        diagnostic_msgs::msg::KeyValue kv;
        kv.key = key;
        kv.value = value;
        status.values.push_back(kv);
    };

    add_key_value("terrain_features_topic", terrain_features_topic_);
    add_key_value("kfs_mask_topic", kfs_mask_topic_);
    add_key_value("output_topic", output_topic_);
    add_key_value("publish_local_map", publish_local_map_ ? "true" : "false");
    add_key_value("output_topic_local", output_topic_local_);
    add_key_value("fusion_enable", fusion_enable_ ? "true" : "false");
    add_key_value("fusion_publish_raw", fusion_publish_raw_ ? "true" : "false");
    add_key_value("output_topic_raw", output_topic_raw_);
    add_key_value("output_topic_local_raw", output_topic_local_raw_);
    add_key_value("fusion_time_constant_sec", std::to_string(fusion_time_constant_sec_));
    add_key_value("fusion_unknown_decay_sec", std::to_string(fusion_unknown_decay_sec_));
    add_key_value("map_frame", map_frame_);
    add_key_value("local_frame", local_frame_);
    add_key_value("base_frame", base_frame_);
    add_key_value("keepout_available", state.keepout_available ? "true" : "false");
    add_key_value("keepout_stale", state.keepout_stale ? "true" : "false");
    add_key_value("enable_mf_semantics", enable_mf_semantics_ ? "true" : "false");
    add_key_value("mf_layout_ready", mf_layout_ready_ ? "true" : "false");
    add_key_value("mf_layout_team", mf_layout_team_);
    add_key_value("mf_layout_file", mf_grid_layout_file_);
    add_key_value("mf_layout_status", mf_layout_status_);
    add_key_value("height_error_limit_m", std::to_string(height_error_limit_m_));
    add_key_value("traversability_height_error_weight",
                  std::to_string(traversability_height_error_weight_));
    add_key_value("traversability_step_edge_weight",
                  std::to_string(traversability_step_edge_weight_));

    diagnostic_msgs::msg::DiagnosticArray diagnostics_msg;
    diagnostics_msg.header.stamp = stamp;
    diagnostics_msg.status.push_back(status);
    pub_diagnostics_->publish(diagnostics_msg);
}

std::string TerrainGridMapBridge::toLower(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return value;
}

}  // namespace rc26_terrain

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rc26_terrain::TerrainGridMapBridge)
