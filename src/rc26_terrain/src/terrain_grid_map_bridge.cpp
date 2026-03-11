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
constexpr const char* kLayerRoughness = "roughness";
constexpr const char* kLayerObstacleProb = "obstacle_prob";
constexpr const char* kLayerDropProb = "drop_prob";
constexpr const char* kLayerKfsKeepout = "kfs_keepout";

constexpr const char* kLayerBlockId = "block_id";
constexpr const char* kLayerExpectedHeight = "expected_height";
constexpr const char* kLayerHeightError = "height_error";
constexpr const char* kLayerEdgeStrength = "edge_strength";
constexpr const char* kLayerStepEdgeMask = "step_edge_mask";
constexpr const char* kLayerTraversability = "traversability";

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
    this->declare_parameter<std::string>("map_frame", map_frame_);
    this->declare_parameter<std::string>("base_frame", base_frame_);
    this->declare_parameter<double>("tf_timeout_sec", tf_timeout_sec_);
    this->declare_parameter<double>("keepout_stale_timeout_sec", keepout_stale_timeout_sec_);
    this->declare_parameter<bool>("enable_mf_semantics", enable_mf_semantics_);
    this->declare_parameter<std::string>("mf_grid_layout_file", mf_grid_layout_file_);
    this->declare_parameter<double>("step_edge_height_thresh_m", step_edge_height_thresh_m_);
    this->declare_parameter<double>("slope_norm_limit", slope_norm_limit_);
    this->declare_parameter<double>("roughness_norm_limit", roughness_norm_limit_);
    this->declare_parameter<std::string>("diagnostics_topic", diagnostics_topic_);

    this->get_parameter("terrain_features_topic", terrain_features_topic_);
    this->get_parameter("kfs_mask_topic", kfs_mask_topic_);
    this->get_parameter("output_topic", output_topic_);
    this->get_parameter("map_frame", map_frame_);
    this->get_parameter("base_frame", base_frame_);
    this->get_parameter("tf_timeout_sec", tf_timeout_sec_);
    this->get_parameter("keepout_stale_timeout_sec", keepout_stale_timeout_sec_);
    this->get_parameter("enable_mf_semantics", enable_mf_semantics_);
    this->get_parameter("mf_grid_layout_file", mf_grid_layout_file_);
    this->get_parameter("step_edge_height_thresh_m", step_edge_height_thresh_m_);
    this->get_parameter("slope_norm_limit", slope_norm_limit_);
    this->get_parameter("roughness_norm_limit", roughness_norm_limit_);
    this->get_parameter("diagnostics_topic", diagnostics_topic_);

    tf_timeout_sec_ = std::max(0.01, tf_timeout_sec_);
    keepout_stale_timeout_sec_ = std::max(0.01, keepout_stale_timeout_sec_);
    step_edge_height_thresh_m_ = std::max(0.0, step_edge_height_thresh_m_);
    slope_norm_limit_ = std::max(1e-6, slope_norm_limit_);
    roughness_norm_limit_ = std::max(1e-6, roughness_norm_limit_);

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
    pub_diagnostics_ =
        this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(diagnostics_topic_, makeDiagnosticsQos());

    sub_features_ = this->create_subscription<rc26_interfaces::msg::TerrainFeatureGrid>(
        terrain_features_topic_, makeFeatureQos(),
        std::bind(&TerrainGridMapBridge::featureCallback, this, std::placeholders::_1));

    sub_keepout_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        kfs_mask_topic_, makeKeepoutQos(),
        std::bind(&TerrainGridMapBridge::keepoutCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
                "terrain_grid_map_bridge started: feature_topic=%s keepout_topic=%s output_topic=%s map_frame=%s",
                terrain_features_topic_.c_str(),
                kfs_mask_topic_.c_str(),
                output_topic_.c_str(),
                map_frame_.c_str());
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
        diagnostics.detail = std::string("lookup map<-base_link failed: ") + ex.what();
        publishDiagnostics(feature_stamp, diagnostics);
        return;
    }

    const auto& translation = tf_map_base.transform.translation;
    const double base_x_map = translation.x;
    const double base_y_map = translation.y;
    const double base_z_map = translation.z;

    tf2::Quaternion q;
    tf2::fromMsg(tf_map_base.transform.rotation, q);
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    (void)roll;
    (void)pitch;
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);

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
        kLayerRoughness,
        kLayerObstacleProb,
        kLayerDropProb,
        kLayerKfsKeepout,
    };
    if (enable_mf_semantics_) {
        layers.push_back(kLayerBlockId);
        layers.push_back(kLayerExpectedHeight);
        layers.push_back(kLayerHeightError);
        layers.push_back(kLayerEdgeStrength);
        layers.push_back(kLayerStepEdgeMask);
        layers.push_back(kLayerTraversability);
    }

    grid_map::GridMap output_map(layers);
    output_map.setFrameId(map_frame_);
    const int64_t stamp_ns = feature_stamp.nanoseconds();
    output_map.setTimestamp(static_cast<grid_map::Time>(std::max<int64_t>(0, stamp_ns)));
    output_map.setGeometry(grid_map::Length(width * resolution, height * resolution),
                           resolution,
                           grid_map::Position(base_x_map, base_y_map));
    output_map.setBasicLayers({kLayerElevationAbs});

    for (const auto& layer : layers) {
        output_map[layer].setConstant(kNaNf);
    }
    output_map[kLayerFresh].setZero();

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

    const auto srcIndex = [width](int ix, int iy) -> size_t {
        return static_cast<size_t>(ix * width + iy);
    };

    for (grid_map::GridMapIterator it(output_map); !it.isPastEnd(); ++it) {
        const grid_map::Index index(*it);
        grid_map::Position position;
        if (!output_map.getPosition(index, position)) {
            continue;
        }

        const double dx = position.x() - base_x_map;
        const double dy = position.y() - base_y_map;
        const double x_bl = cos_yaw * dx + sin_yaw * dy;
        const double y_bl = -sin_yaw * dx + cos_yaw * dy;

        const int ix = static_cast<int>(std::llround(x_bl / resolution)) + half_width;
        const int iy = static_cast<int>(std::llround(y_bl / resolution)) + half_width;
        if (ix < 0 || ix >= width || iy < 0 || iy >= width) {
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

        if (keepout_available) {
            const auto keepout_value = sampleKeepoutValue(*keepout_mask, position.x(), position.y());
            if (keepout_value.has_value()) {
                output_map.at(kLayerKfsKeepout, index) = *keepout_value;
            }
        }

        if (!fresh) {
            continue;
        }

        output_map.at(kLayerElevationAbs, index) =
            static_cast<float>(base_z_map) + msg->h_ground[source_idx];
        output_map.at(kLayerElevationTopAbs, index) =
            static_cast<float>(base_z_map) + msg->h_top[source_idx];
        output_map.at(kLayerSigmaH, index) = msg->sigma_h[source_idx];
        output_map.at(kLayerDensity, index) = static_cast<float>(msg->density[source_idx]);
        output_map.at(kLayerSlope, index) =
            std::max(std::abs(msg->slope_x[source_idx]), std::abs(msg->slope_y[source_idx]));
        output_map.at(kLayerRoughness, index) = msg->roughness[source_idx];
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

            const float slope = output_map.at(kLayerSlope, index);
            const float roughness = output_map.at(kLayerRoughness, index);
            const float obstacle_prob = output_map.at(kLayerObstacleProb, index);
            const float drop_prob = output_map.at(kLayerDropProb, index);
            if (!isFinite(slope) || !isFinite(roughness) ||
                !isFinite(obstacle_prob) || !isFinite(drop_prob)) {
                continue;
            }

            const float slope_norm =
                clamp01(slope / static_cast<float>(slope_norm_limit_));
            const float roughness_norm =
                clamp01(roughness / static_cast<float>(roughness_norm_limit_));
            float max_term = 0.0f;
            max_term = std::max(max_term, 0.35f * slope_norm);
            max_term = std::max(max_term, 0.20f * roughness_norm);
            max_term = std::max(max_term, 0.25f * clamp01(obstacle_prob));
            max_term = std::max(max_term, 0.35f * clamp01(drop_prob));

            const float kfs_keepout = output_map.at(kLayerKfsKeepout, index);
            if (isFinite(kfs_keepout)) {
                max_term = std::max(max_term, clamp01(kfs_keepout));
            }

            output_map.at(kLayerTraversability, index) = clamp01(1.0f - max_term);
        }
    }

    auto output_msg = grid_map::GridMapRosConverter::toMessage(output_map);
    if (output_msg) {
        pub_grid_map_->publish(std::move(*output_msg));
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
    add_key_value("map_frame", map_frame_);
    add_key_value("base_frame", base_frame_);
    add_key_value("keepout_available", state.keepout_available ? "true" : "false");
    add_key_value("keepout_stale", state.keepout_stale ? "true" : "false");
    add_key_value("enable_mf_semantics", enable_mf_semantics_ ? "true" : "false");
    add_key_value("mf_layout_ready", mf_layout_ready_ ? "true" : "false");
    add_key_value("mf_layout_team", mf_layout_team_);
    add_key_value("mf_layout_file", mf_grid_layout_file_);
    add_key_value("mf_layout_status", mf_layout_status_);

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
