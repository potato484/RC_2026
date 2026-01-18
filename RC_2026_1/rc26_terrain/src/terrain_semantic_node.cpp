#include "rc26_terrain/terrain_semantic_node.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "pcl/filters/voxel_grid.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace {

float quantileInplace(std::vector<float>& values, double q) {
    if (values.empty()) return 0.0f;
    q = std::clamp(q, 0.0, 1.0);
    const size_t n = values.size();
    const size_t idx = static_cast<size_t>(std::floor(q * static_cast<double>(n - 1)));
    std::nth_element(values.begin(),
                     values.begin() + static_cast<std::vector<float>::difference_type>(idx),
                     values.end());
    return values[idx];
}

}  // namespace

namespace rc26_terrain {

TerrainSemanticNode::TerrainSemanticNode(const rclcpp::NodeOptions& options)
    : Node("terrain_semantic", options) {
    // Declare parameters
    this->declare_parameter<std::string>("input_cloud_topic", "registered_scan");
    this->declare_parameter<std::string>("output_obstacles_topic", "terrain_obstacles");
    this->declare_parameter<std::string>("output_drop_topic", "terrain_drop");
    this->declare_parameter<std::string>("target_frame", "odom");
    this->declare_parameter<std::string>("base_frame", "base_link");
    this->declare_parameter<double>("tf_timeout_sec", tf_timeout_sec_);
    this->declare_parameter<double>("perception_radius_m", perception_radius_m_);
    this->declare_parameter<double>("grid_resolution_m", grid_resolution_m_);
    this->declare_parameter<double>("voxel_leaf_size_m", voxel_leaf_size_m_);
    this->declare_parameter<double>("min_rel_z_m", min_rel_z_m_);
    this->declare_parameter<double>("max_rel_z_m", max_rel_z_m_);
    this->declare_parameter<double>("dis_ratio_z", dis_ratio_z_);
    this->declare_parameter<int>("min_points_per_cell", min_points_per_cell_);
    this->declare_parameter<double>("ground_quantile", ground_quantile_);
    this->declare_parameter<double>("top_quantile", top_quantile_);
    this->declare_parameter<double>("ground_ema_alpha", ground_ema_alpha_);
    this->declare_parameter<double>("h_climb_m", h_climb_m_);
    this->declare_parameter<double>("h_obstacle_m", h_obstacle_m_);
    this->declare_parameter<double>("h_drop_m", h_drop_m_);
    this->declare_parameter<bool>("enable_hysteresis", enable_hysteresis_);
    this->declare_parameter<int>("score_max", score_max_);
    this->declare_parameter<int>("score_inc", score_inc_);
    this->declare_parameter<int>("score_dec", score_dec_);
    this->declare_parameter<int>("obstacle_on_score", obstacle_on_score_);
    this->declare_parameter<int>("obstacle_off_score", obstacle_off_score_);
    this->declare_parameter<int>("drop_on_score", drop_on_score_);
    this->declare_parameter<int>("drop_off_score", drop_off_score_);
    this->declare_parameter<double>("stale_time_sec", stale_time_sec_);

    // Get parameters
    this->get_parameter("input_cloud_topic", input_cloud_topic_);
    this->get_parameter("output_obstacles_topic", output_obstacles_topic_);
    this->get_parameter("output_drop_topic", output_drop_topic_);
    this->get_parameter("target_frame", target_frame_);
    this->get_parameter("base_frame", base_frame_);
    this->get_parameter("tf_timeout_sec", tf_timeout_sec_);
    this->get_parameter("perception_radius_m", perception_radius_m_);
    this->get_parameter("grid_resolution_m", grid_resolution_m_);
    this->get_parameter("voxel_leaf_size_m", voxel_leaf_size_m_);
    this->get_parameter("min_rel_z_m", min_rel_z_m_);
    this->get_parameter("max_rel_z_m", max_rel_z_m_);
    this->get_parameter("dis_ratio_z", dis_ratio_z_);
    this->get_parameter("min_points_per_cell", min_points_per_cell_);
    this->get_parameter("ground_quantile", ground_quantile_);
    this->get_parameter("top_quantile", top_quantile_);
    this->get_parameter("ground_ema_alpha", ground_ema_alpha_);
    this->get_parameter("h_climb_m", h_climb_m_);
    this->get_parameter("h_obstacle_m", h_obstacle_m_);
    this->get_parameter("h_drop_m", h_drop_m_);
    this->get_parameter("enable_hysteresis", enable_hysteresis_);
    this->get_parameter("score_max", score_max_);
    this->get_parameter("score_inc", score_inc_);
    this->get_parameter("score_dec", score_dec_);
    this->get_parameter("obstacle_on_score", obstacle_on_score_);
    this->get_parameter("obstacle_off_score", obstacle_off_score_);
    this->get_parameter("drop_on_score", drop_on_score_);
    this->get_parameter("drop_off_score", drop_off_score_);
    this->get_parameter("stale_time_sec", stale_time_sec_);

    // Parameter validation
    if (perception_radius_m_ <= 0.0) {
        throw std::invalid_argument("perception_radius_m must be > 0");
    }
    if (grid_resolution_m_ <= 0.0) {
        throw std::invalid_argument("grid_resolution_m must be > 0");
    }
    if (voxel_leaf_size_m_ <= 0.0) {
        throw std::invalid_argument("voxel_leaf_size_m must be > 0");
    }
    if (stale_time_sec_ <= 0.0) {
        throw std::invalid_argument("stale_time_sec must be > 0");
    }
    ground_ema_alpha_ = std::clamp(ground_ema_alpha_, 0.01, 1.0);
    min_points_per_cell_ = std::max(1, min_points_per_cell_);
    score_max_ = std::max(1, score_max_);
    score_inc_ = std::max(0, score_inc_);
    score_dec_ = std::max(0, score_dec_);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    initGrid();

    pub_obstacles_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        output_obstacles_topic_, rclcpp::SensorDataQoS());
    pub_drop_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        output_drop_topic_, rclcpp::SensorDataQoS());

    sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_cloud_topic_, rclcpp::SensorDataQoS(),
        std::bind(&TerrainSemanticNode::cloudCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "TerrainSemanticNode initialized: grid %dx%d, radius %.1fm",
                width_, width_, perception_radius_m_);
}

std::optional<tf2::Transform> TerrainSemanticNode::getTransform(
    const std::string& target_frame, const std::string& source_frame,
    const rclcpp::Time& time) {
    try {
        auto ts = tf_buffer_->lookupTransform(
            target_frame, source_frame, time,
            rclcpp::Duration::from_seconds(tf_timeout_sec_));
        tf2::Transform transform;
        tf2::fromMsg(ts.transform, transform);
        return transform;
    } catch (tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "TF lookup failed (%s -> %s): %s",
                             source_frame.c_str(), target_frame.c_str(), ex.what());
        return std::nullopt;
    }
}

void TerrainSemanticNode::initGrid() {
    half_width_ = static_cast<int>(std::ceil(perception_radius_m_ / grid_resolution_m_));
    width_ = 2 * half_width_ + 1;
    num_cells_ = width_ * width_;

    cell_in_radius_.assign(static_cast<size_t>(num_cells_), 0);
    ground_z_filtered_.assign(static_cast<size_t>(num_cells_), 0.0f);
    top_z_.assign(static_cast<size_t>(num_cells_), 0.0f);
    last_seen_sec_.assign(static_cast<size_t>(num_cells_), -1.0);
    obstacle_score_.assign(static_cast<size_t>(num_cells_), 0);
    drop_score_.assign(static_cast<size_t>(num_cells_), 0);
    obstacle_state_.assign(static_cast<size_t>(num_cells_), 0);
    drop_state_.assign(static_cast<size_t>(num_cells_), 0);
    cell_z_samples_.assign(static_cast<size_t>(num_cells_), {});
    touched_cells_.reserve(static_cast<size_t>(num_cells_));

    const double r2 = perception_radius_m_ * perception_radius_m_;
    for (int ix = 0; ix < width_; ix++) {
        for (int iy = 0; iy < width_; iy++) {
            const double x = (static_cast<double>(ix - half_width_) + 0.5) * grid_resolution_m_;
            const double y = (static_cast<double>(iy - half_width_) + 0.5) * grid_resolution_m_;
            if (x * x + y * y <= r2) {
                cell_in_radius_[static_cast<size_t>(ix * width_ + iy)] = 1;
            }
        }
    }
}

void TerrainSemanticNode::estimateCellHeights(double stamp_sec) {
    for (const int cell : touched_cells_) {
        auto& samples = cell_z_samples_[static_cast<size_t>(cell)];
        if (static_cast<int>(samples.size()) < min_points_per_cell_) continue;

        const float ground_z = quantileInplace(samples, ground_quantile_);
        float top_z = quantileInplace(samples, top_quantile_);
        if (top_z < ground_z) top_z = ground_z;

        const size_t idx = static_cast<size_t>(cell);
        if (last_seen_sec_[idx] < 0.0) {
            ground_z_filtered_[idx] = ground_z;
        } else {
            ground_z_filtered_[idx] = static_cast<float>(ground_ema_alpha_) * ground_z +
                                      static_cast<float>(1.0 - ground_ema_alpha_) * ground_z_filtered_[idx];
        }
        top_z_[idx] = top_z;
        last_seen_sec_[idx] = stamp_sec;
    }
}

void TerrainSemanticNode::classifyAndUpdate(double stamp_sec) {
    for (int cell = 0; cell < num_cells_; cell++) {
        const size_t idx = static_cast<size_t>(cell);

        if (!cell_in_radius_[idx]) {
            if (enable_hysteresis_) {
                obstacle_score_[idx] = std::max(0, obstacle_score_[idx] - score_dec_);
                drop_score_[idx] = std::max(0, drop_score_[idx] - score_dec_);
                if (obstacle_state_[idx] && obstacle_score_[idx] <= obstacle_off_score_)
                    obstacle_state_[idx] = 0;
                if (drop_state_[idx] && drop_score_[idx] <= drop_off_score_)
                    drop_state_[idx] = 0;
            } else {
                obstacle_state_[idx] = 0;
                drop_state_[idx] = 0;
            }
            continue;
        }

        const double last = last_seen_sec_[idx];
        const bool fresh = (last >= 0.0) && ((stamp_sec - last) <= stale_time_sec_);

        if (!fresh) {
            if (enable_hysteresis_) {
                obstacle_score_[idx] = std::max(0, obstacle_score_[idx] - score_dec_);
                drop_score_[idx] = std::max(0, drop_score_[idx] - score_dec_);
                if (obstacle_state_[idx] && obstacle_score_[idx] <= obstacle_off_score_)
                    obstacle_state_[idx] = 0;
                if (drop_state_[idx] && drop_score_[idx] <= drop_off_score_)
                    drop_state_[idx] = 0;
            } else {
                obstacle_state_[idx] = 0;
                drop_state_[idx] = 0;
            }
            continue;
        }

        const int ix = cell / width_;
        const int iy = cell % width_;
        float dz_up = 0.0f, dz_down = 0.0f;

        for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++) {
                if (dx == 0 && dy == 0) continue;
                const int nx = ix + dx, ny = iy + dy;
                if (nx < 0 || nx >= width_ || ny < 0 || ny >= width_) continue;

                const size_t nidx = static_cast<size_t>(nx * width_ + ny);
                if (last_seen_sec_[nidx] < 0.0 || (stamp_sec - last_seen_sec_[nidx]) > stale_time_sec_)
                    continue;

                const float diff = ground_z_filtered_[nidx] - ground_z_filtered_[idx];
                dz_up = std::max(dz_up, diff);
                dz_down = std::min(dz_down, diff);
            }
        }

        const float h_above = top_z_[idx] - ground_z_filtered_[idx];
        const bool obstacle_candidate =
            (dz_up > static_cast<float>(h_obstacle_m_)) ||
            (h_above > static_cast<float>(h_obstacle_m_));
        const bool drop_candidate = (dz_down < static_cast<float>(-h_drop_m_));

        if (!enable_hysteresis_) {
            obstacle_state_[idx] = obstacle_candidate ? 1 : 0;
            drop_state_[idx] = drop_candidate ? 1 : 0;
            continue;
        }

        // Score-based hysteresis
        if (obstacle_candidate)
            obstacle_score_[idx] = std::min(score_max_, obstacle_score_[idx] + score_inc_);
        else
            obstacle_score_[idx] = std::max(0, obstacle_score_[idx] - score_dec_);

        if (drop_candidate)
            drop_score_[idx] = std::min(score_max_, drop_score_[idx] + score_inc_);
        else
            drop_score_[idx] = std::max(0, drop_score_[idx] - score_dec_);

        if (!obstacle_state_[idx] && obstacle_score_[idx] >= obstacle_on_score_)
            obstacle_state_[idx] = 1;
        if (obstacle_state_[idx] && obstacle_score_[idx] <= obstacle_off_score_)
            obstacle_state_[idx] = 0;

        if (!drop_state_[idx] && drop_score_[idx] >= drop_on_score_)
            drop_state_[idx] = 1;
        if (drop_state_[idx] && drop_score_[idx] <= drop_off_score_)
            drop_state_[idx] = 0;
    }
}

void TerrainSemanticNode::publishOutputs(const rclcpp::Time& stamp, double base_x,
                                         double base_y, double base_z,
                                         double cos_yaw, double sin_yaw) {
    pcl::PointCloud<pcl::PointXYZI> obs_cloud, drop_cloud;
    const double stamp_sec = stamp.seconds();

    for (int cell = 0; cell < num_cells_; cell++) {
        const size_t idx = static_cast<size_t>(cell);
        if (!cell_in_radius_[idx]) continue;

        const int ix = cell / width_;
        const int iy = cell % width_;
        const double x_rel = (static_cast<double>(ix - half_width_) + 0.5) * grid_resolution_m_;
        const double y_rel = (static_cast<double>(iy - half_width_) + 0.5) * grid_resolution_m_;
        const double x = base_x + cos_yaw * x_rel - sin_yaw * y_rel;
        const double y = base_y + sin_yaw * x_rel + cos_yaw * y_rel;

        float z = static_cast<float>(base_z);
        if (last_seen_sec_[idx] >= 0.0 && (stamp_sec - last_seen_sec_[idx]) <= stale_time_sec_ * 2.0)
            z = ground_z_filtered_[idx];

        if (obstacle_state_[idx]) {
            pcl::PointXYZI p;
            p.x = static_cast<float>(x);
            p.y = static_cast<float>(y);
            p.z = z;
            p.intensity = static_cast<float>(obstacle_score_[idx]);
            obs_cloud.push_back(p);
        }
        if (drop_state_[idx]) {
            pcl::PointXYZI p;
            p.x = static_cast<float>(x);
            p.y = static_cast<float>(y);
            p.z = z;
            p.intensity = static_cast<float>(drop_score_[idx]);
            drop_cloud.push_back(p);
        }
    }

    sensor_msgs::msg::PointCloud2 obs_msg, drop_msg;
    pcl::toROSMsg(obs_cloud, obs_msg);
    obs_msg.header.stamp = stamp;
    obs_msg.header.frame_id = target_frame_;
    pub_obstacles_->publish(obs_msg);

    pcl::toROSMsg(drop_cloud, drop_msg);
    drop_msg.header.stamp = stamp;
    drop_msg.header.frame_id = target_frame_;
    pub_drop_->publish(drop_msg);
}

void TerrainSemanticNode::cloudCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
    if (!msg) return;

    const rclcpp::Time stamp(msg->header.stamp);
    const double stamp_sec = stamp.seconds();

    // Clear per-frame buffers first (even if we early-return)
    for (int cell : touched_cells_) {
        cell_z_samples_[static_cast<size_t>(cell)].clear();
    }
    touched_cells_.clear();

    const auto tf_base = getTransform(target_frame_, base_frame_, stamp);
    if (!tf_base) {
        // Still run decay even without TF
        classifyAndUpdate(stamp_sec);
        sensor_msgs::msg::PointCloud2 empty;
        empty.header.stamp = stamp;
        empty.header.frame_id = target_frame_;
        pub_obstacles_->publish(empty);
        pub_drop_->publish(empty);
        return;
    }

    // Get robot pose
    const auto& origin = tf_base->getOrigin();
    const double base_x = origin.x();
    const double base_y = origin.y();
    const double base_z = origin.z();

    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_base->getRotation()).getRPY(roll, pitch, yaw);
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);

    if (msg->data.empty() || msg->header.frame_id.empty()) {
        classifyAndUpdate(stamp_sec);
        publishOutputs(stamp, base_x, base_y, base_z, cos_yaw, sin_yaw);
        return;
    }

    const auto tf_cloud = getTransform(target_frame_, msg->header.frame_id, stamp);
    if (!tf_cloud) {
        classifyAndUpdate(stamp_sec);
        publishOutputs(stamp, base_x, base_y, base_z, cos_yaw, sin_yaw);
        return;
    }

    // Transform cloud to target frame
    sensor_msgs::msg::PointCloud2 cloud_transformed;
    pcl_ros::transformPointCloud(target_frame_, *tf_cloud, *msg, cloud_transformed);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(cloud_transformed, *cloud);

    // Downsample
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ds(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setLeafSize(static_cast<float>(voxel_leaf_size_m_),
                      static_cast<float>(voxel_leaf_size_m_),
                      static_cast<float>(voxel_leaf_size_m_));
    voxel.setInputCloud(cloud);
    voxel.filter(*cloud_ds);

    const double r2 = perception_radius_m_ * perception_radius_m_;

    // Accumulate points into grid
    for (const auto& p : cloud_ds->points) {
        // Skip non-finite points to avoid UB
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;

        const double dx = static_cast<double>(p.x) - base_x;
        const double dy = static_cast<double>(p.y) - base_y;
        const double rel_z = static_cast<double>(p.z) - base_z;

        // Robot-yaw aligned coordinates
        const double x_rel = cos_yaw * dx + sin_yaw * dy;
        const double y_rel = -sin_yaw * dx + cos_yaw * dy;
        const double d2 = x_rel * x_rel + y_rel * y_rel;
        if (d2 > r2) continue;

        const double r = std::sqrt(d2);
        if (rel_z < (min_rel_z_m_ - dis_ratio_z_ * r) ||
            rel_z > (max_rel_z_m_ + dis_ratio_z_ * r))
            continue;

        int ix = static_cast<int>(std::floor(x_rel / grid_resolution_m_)) + half_width_;
        int iy = static_cast<int>(std::floor(y_rel / grid_resolution_m_)) + half_width_;
        ix = std::clamp(ix, 0, width_ - 1);
        iy = std::clamp(iy, 0, width_ - 1);

        const int cell = ix * width_ + iy;
        auto& bucket = cell_z_samples_[static_cast<size_t>(cell)];
        if (bucket.empty()) touched_cells_.push_back(cell);
        bucket.push_back(p.z);
    }

    estimateCellHeights(stamp_sec);
    classifyAndUpdate(stamp_sec);
    publishOutputs(stamp, base_x, base_y, base_z, cos_yaw, sin_yaw);
}

}  // namespace rc26_terrain

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rc26_terrain::TerrainSemanticNode)
