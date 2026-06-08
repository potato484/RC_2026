// Maintained by DongXuan Chen <2220362462@qq.com>
// #include <so3_math.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <malloc.h>
#include <memory>
#include <sstream>
#include <thread>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "experimental_loop_closure.hpp"
#include "li_initialization.hpp"

using namespace std;

#define PUBFRAME_PERIOD (20)

const float MOV_THRESHOLD = 1.5f;

string root_dir = ROOT_DIR;

int time_log_counter = 0;

bool init_map = false, flg_first_scan = true;

// Time Log Variables
double match_time = 0, solve_time = 0, propag_time = 0, update_time = 0;

bool flg_reset = false;

// surf feature in map
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body_space(new PointCloudXYZI());
PointCloudXYZI::Ptr init_feats_world(new PointCloudXYZI());
std::deque<PointCloudXYZI::Ptr> depth_feats_world;
PointCloudXYZI::Ptr full_map_cloud_for_viz(new PointCloudXYZI());
pcl::VoxelGrid<PointType> downSizeFilterSurf;

V3D euler_cur;

nav_msgs::msg::Path path;
nav_msgs::msg::Odometry odomAftMapped;
geometry_msgs::msg::PoseStamped msg_body_pose;
double last_full_map_publish_time = -1.0;

auto LOGGER = rclcpp::get_logger("laserMapping");
std::mutex runtime_param_mutex;
std::shared_ptr<rc26_point_lio::ExperimentalLoopClosureBackend> experimental_loop_backend;

constexpr char kBodyFilterBaseFrame[] = "base_link";
constexpr char kBodyFilterLidarFrame[] = "livox_frame";

bool isBodyFilterParameter(const std::string& name) {
    return name == "filter_car_body" || name == "body_x_min" || name == "body_x_max" || name == "body_y_min" ||
           name == "body_y_max" || name == "body_z_min" || name == "body_z_max";
}

std::string validateBodyFilterConfig(const Preprocess::BodyFilterConfig& config) {
    const double values[] = {config.x_min, config.x_max, config.y_min, config.y_max, config.z_min, config.z_max};
    for (const double value : values) {
        if (!std::isfinite(value)) {
            return "车身 ROI 边界必须是有限数值";
        }
    }

    if (config.x_min > config.x_max) {
        return "body_x_min 必须小于或等于 body_x_max";
    }
    if (config.y_min > config.y_max) {
        return "body_y_min 必须小于或等于 body_y_max";
    }
    if (config.z_min > config.z_max) {
        return "body_z_min 必须小于或等于 body_z_max";
    }
    return {};
}

std::string pcdFlushReasonText(const char* reason) {
    if (reason == nullptr) {
        return "未说明原因";
    }
    const std::string reason_text(reason);
    if (reason_text == "interval flush") {
        return "达到 pcd_save.interval 分段保存条件";
    }
    if (reason_text == "shutdown") {
        return "节点正常退出";
    }
    return reason_text;
}

Eigen::Isometry3d transformStampedToIsometry(const geometry_msgs::msg::TransformStamped& transform) {
    const auto& rotation = transform.transform.rotation;
    const auto& translation = transform.transform.translation;

    Eigen::Quaterniond quaternion(rotation.w, rotation.x, rotation.y, rotation.z);
    quaternion.normalize();

    Eigen::Isometry3d isometry = Eigen::Isometry3d::Identity();
    isometry.linear() = quaternion.toRotationMatrix();
    isometry.translation() = Eigen::Vector3d(translation.x, translation.y, translation.z);
    return isometry;
}

bool lookupBodyFilterTransform(tf2_ros::Buffer& tf_buffer, Eigen::Isometry3d& lidar_to_base, std::string& error) {
    try {
        const auto transform =
            tf_buffer.lookupTransform(kBodyFilterBaseFrame, kBodyFilterLidarFrame, tf2::TimePointZero);
        lidar_to_base = transformStampedToIsometry(transform);
        error.clear();
        return true;
    } catch (const tf2::TransformException& ex) {
        error = ex.what();
        return false;
    }
}

bool waitForBodyFilterTransform(const std::shared_ptr<tf2_ros::Buffer>& tf_buffer,
                                rclcpp::executors::MultiThreadedExecutor& executor,
                                Eigen::Isometry3d& lidar_to_base,
                                std::string& error) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
        executor.spin_some();
        if (lookupBodyFilterTransform(*tf_buffer, lidar_to_base, error)) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    executor.spin_some();
    return lookupBodyFilterTransform(*tf_buffer, lidar_to_base, error);
}

void applyBodyFilterRuntimeConfig(const Preprocess::BodyFilterConfig& config) {
    filter_car_body = config.enabled;
    body_x_min = config.x_min;
    body_x_max = config.x_max;
    body_y_min = config.y_min;
    body_y_max = config.y_max;
    body_z_min = config.z_min;
    body_z_max = config.z_max;
    p_pre->setBodyFilterConfig(
        filter_car_body, body_x_min, body_x_max, body_y_min, body_y_max, body_z_min, body_z_max);
    if (config.transform_ready) {
        p_pre->setBodyFilterTransform(config.lidar_to_base);
    }
}

void finalizeCloudMetadata(PointCloudXYZI& cloud) {
    cloud.width = static_cast<uint32_t>(cloud.points.size());
    cloud.height = 1;
    cloud.is_dense = false;
}

void resetFullMapForViz() {
    if (full_map_cloud_for_viz) {
        full_map_cloud_for_viz->clear();
        finalizeCloudMetadata(*full_map_cloud_for_viz);
    }
    last_full_map_publish_time = -1.0;
}

rc26_point_lio::ExperimentalLoopClosureConfig buildExperimentalLoopClosureConfig() {
    rc26_point_lio::ExperimentalLoopClosureConfig config;
    config.enable = experimental_loop_closure_enable;
    config.frequency_hz = experimental_loop_closure_frequency_hz;
    config.keyframe_dist_threshold_m = experimental_loop_closure_keyframe_dist_threshold_m;
    config.keyframe_angle_threshold_rad = experimental_loop_closure_keyframe_angle_threshold_rad;
    config.search_radius_m = experimental_loop_closure_search_radius_m;
    config.time_diff_threshold_sec = experimental_loop_closure_time_diff_threshold_sec;
    config.exclude_recent_keyframes = experimental_loop_closure_exclude_recent_keyframes;
    config.sc_dist_threshold = experimental_loop_closure_sc_dist_threshold;
    config.icp_fitness_threshold = experimental_loop_closure_icp_fitness_threshold;
    config.icp_max_correspondence_dist_m = experimental_loop_closure_icp_max_correspondence_dist_m;
    config.icp_max_iterations = experimental_loop_closure_icp_max_iterations;
    config.submap_size = experimental_loop_closure_submap_size;
    config.keyframe_cloud_voxel_size_m = experimental_loop_closure_keyframe_cloud_voxel_size_m;
    config.max_keyframes_with_cloud = experimental_loop_closure_max_keyframes_with_cloud;
    return config;
}

M3D currentLioRotation() {
    return use_imu_as_input ? M3D(kf_input.x_.rot) : M3D(kf_output.x_.rot);
}

V3D currentLioPosition() {
    return use_imu_as_input ? kf_input.x_.pos : kf_output.x_.pos;
}

void appendFullMapForViz(const PointVector& points_to_add) {
    if (!full_map_publish_en || points_to_add.empty()) {
        return;
    }

    if (!full_map_cloud_for_viz) {
        full_map_cloud_for_viz.reset(new PointCloudXYZI());
    }
    full_map_cloud_for_viz->points.insert(
        full_map_cloud_for_viz->points.end(), points_to_add.begin(), points_to_add.end());
    finalizeCloudMetadata(*full_map_cloud_for_viz);
}

void appendFullMapForViz(const PointCloudXYZI& cloud) {
    if (!full_map_publish_en || cloud.empty()) {
        return;
    }

    if (!full_map_cloud_for_viz) {
        full_map_cloud_for_viz.reset(new PointCloudXYZI());
    }
    full_map_cloud_for_viz->points.insert(
        full_map_cloud_for_viz->points.end(), cloud.points.begin(), cloud.points.end());
    finalizeCloudMetadata(*full_map_cloud_for_viz);
}

PointCloudXYZI::Ptr buildFullMapCloudForPublish(const std::shared_ptr<rclcpp::Node>& nh) {
    if (!full_map_cloud_for_viz || full_map_cloud_for_viz->empty()) {
        return {};
    }

    PointCloudXYZI::Ptr filtered_cloud(new PointCloudXYZI());
    pcl::VoxelGrid<PointType> voxel_filter;
    const float leaf_size = static_cast<float>(full_map_voxel_size);
    voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_filter.setInputCloud(full_map_cloud_for_viz);
    voxel_filter.filter(*filtered_cloud);
    finalizeCloudMetadata(*filtered_cloud);

    const size_t max_points = static_cast<size_t>(full_map_max_points);
    if (filtered_cloud->points.size() <= max_points) {
        return filtered_cloud;
    }

    PointCloudXYZI::Ptr capped_cloud(new PointCloudXYZI());
    capped_cloud->points.reserve(max_points);
    const size_t stride = std::max<size_t>(
        1, static_cast<size_t>(std::ceil(static_cast<double>(filtered_cloud->points.size()) /
                                         static_cast<double>(max_points))));
    for (size_t i = 0; i < filtered_cloud->points.size() && capped_cloud->points.size() < max_points; i += stride) {
        capped_cloud->points.emplace_back(filtered_cloud->points[i]);
    }
    finalizeCloudMetadata(*capped_cloud);

    RCLCPP_WARN_THROTTLE(LOGGER, *nh->get_clock(), 5000,
                         "完整累计地图可视化点数过多，已从 %zu 点抽稀到 %zu 点，抽样步长=%zu",
                         filtered_cloud->points.size(), capped_cloud->points.size(), stride);
    return capped_cloud;
}

void publish_full_map_if_due(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pubFullMap,
                             const std::shared_ptr<rclcpp::Node>& nh) {
    if (!full_map_publish_en || !pubFullMap || !full_map_cloud_for_viz || full_map_cloud_for_viz->empty()) {
        return;
    }

    if (last_full_map_publish_time >= 0.0 &&
        (lidar_end_time - last_full_map_publish_time) < full_map_interval_sec) {
        return;
    }

    PointCloudXYZI::Ptr cloud_to_publish = buildFullMapCloudForPublish(nh);
    if (!cloud_to_publish || cloud_to_publish->empty()) {
        return;
    }

    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud_to_publish, cloud_msg);
    cloud_msg.header.stamp = get_ros_time(lidar_end_time);
    cloud_msg.header.frame_id = odom_frame;
    pubFullMap->publish(cloud_msg);
    last_full_map_publish_time = lidar_end_time;
}

inline void dump_lio_state_to_log(FILE* fp) {
    // Check for null file pointer to prevent crash
    if (fp == nullptr) {
        return;
    }

    V3D rot_ang;
    if (!use_imu_as_input) {
        rot_ang = SO3ToEuler(kf_output.x_.rot);
    } else {
        rot_ang = SO3ToEuler(kf_input.x_.rot);
    }

    fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
    fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));  // Angle
    if (use_imu_as_input) {
        fprintf(fp, "%lf %lf %lf ", kf_input.x_.pos(0), kf_input.x_.pos(1), kf_input.x_.pos(2));  // Pos
        fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                               // omega
        fprintf(fp, "%lf %lf %lf ", kf_input.x_.vel(0), kf_input.x_.vel(1), kf_input.x_.vel(2));  // Vel
        fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                               // Acc
        fprintf(fp, "%lf %lf %lf ", kf_input.x_.bg(0), kf_input.x_.bg(1), kf_input.x_.bg(2));     // Bias_g
        fprintf(fp, "%lf %lf %lf ", kf_input.x_.ba(0), kf_input.x_.ba(1), kf_input.x_.ba(2));     // Bias_a
        fprintf(fp, "%lf %lf %lf ", kf_input.x_.gravity(0), kf_input.x_.gravity(1),
                kf_input.x_.gravity(2));  // Bias_a
    } else {
        fprintf(fp, "%lf %lf %lf ", kf_output.x_.pos(0), kf_output.x_.pos(1), kf_output.x_.pos(2));  // Pos
        fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                                  // omega
        fprintf(fp, "%lf %lf %lf ", kf_output.x_.vel(0), kf_output.x_.vel(1), kf_output.x_.vel(2));  // Vel
        fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                                  // Acc
        fprintf(fp, "%lf %lf %lf ", kf_output.x_.bg(0), kf_output.x_.bg(1), kf_output.x_.bg(2));     // Bias_g
        fprintf(fp, "%lf %lf %lf ", kf_output.x_.ba(0), kf_output.x_.ba(1), kf_output.x_.ba(2));     // Bias_a
        fprintf(fp, "%lf %lf %lf ", kf_output.x_.gravity(0), kf_output.x_.gravity(1),
                kf_output.x_.gravity(2));  // Bias_a
    }
    fprintf(fp, "\r\n");
    fflush(fp);
}

void pointBodyLidarToIMU(PointType const* const pi, PointType* const po) {
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu;
    if (extrinsic_est_en) {
        if (!use_imu_as_input) {
            p_body_imu = kf_output.x_.offset_R_L_I * p_body_lidar + kf_output.x_.offset_T_L_I;
        } else {
            p_body_imu = kf_input.x_.offset_R_L_I * p_body_lidar + kf_input.x_.offset_T_L_I;
        }
    } else {
        p_body_imu = Lidar_R_wrt_IMU * p_body_lidar + Lidar_T_wrt_IMU;
    }
    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

PointCloudXYZI::Ptr makeExperimentalLoopBodyCloud() {
    if (!feats_down_body || feats_down_body->empty()) {
        return {};
    }

    PointCloudXYZI::Ptr body_cloud(new PointCloudXYZI());
    body_cloud->points.resize(feats_down_body->points.size());
    for (size_t i = 0; i < feats_down_body->points.size(); ++i) {
        pointBodyLidarToIMU(&feats_down_body->points[i], &body_cloud->points[i]);
    }
    finalizeCloudMetadata(*body_cloud);
    return body_cloud;
}

void rebuildPathFromExperimentalLoop(const std::vector<rc26_point_lio::ExperimentalOptimizedPose>& poses) {
    path.poses.clear();
    path.header.frame_id = odom_frame;
    path.header.stamp = get_ros_time(lidar_end_time);
    for (const auto& optimized_pose : poses) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = odom_frame;
        pose.header.stamp = get_ros_time(optimized_pose.timestamp);
        pose.pose.position.x = optimized_pose.position.x();
        pose.pose.position.y = optimized_pose.position.y();
        pose.pose.position.z = optimized_pose.position.z();
        Eigen::Quaterniond q(optimized_pose.rotation);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();
        path.poses.emplace_back(std::move(pose));
    }
}

void refreshDownsampledWorldCloudFromCurrentState() {
    if (!feats_down_body || feats_down_body->empty()) {
        return;
    }

    if (!feats_down_world) {
        feats_down_world.reset(new PointCloudXYZI());
    }
    feats_down_world->resize(feats_down_body->points.size());
    for (size_t i = 0; i < feats_down_body->points.size(); ++i) {
        pointBodyToWorld(&feats_down_body->points[i], &feats_down_world->points[i]);
    }
    finalizeCloudMetadata(*feats_down_world);
}

void applyExperimentalLoopCorrectionIfReady() {
    if (!experimental_loop_backend || !experimental_loop_backend->enabled()) {
        return;
    }

    rc26_point_lio::ExperimentalLoopCorrection correction;
    if (!experimental_loop_backend->consumePendingCorrection(correction)) {
        return;
    }

    const M3D previous_rotation = currentLioRotation();
    const V3D previous_position = currentLioPosition();
    const M3D delta_rotation = correction.rotation * correction.original_rotation.transpose();
    const V3D delta_position = correction.position - delta_rotation * correction.original_position;
    const M3D corrected_rotation = delta_rotation * previous_rotation;
    const V3D corrected_position = delta_rotation * previous_position + delta_position;

    if (use_imu_as_input) {
        state_input updated = kf_input.get_x();
        updated.pos = corrected_position;
        updated.rot = SO3(corrected_rotation);
        state_in = updated;
        kf_input.change_x(updated);
    } else {
        state_output updated = kf_output.get_x();
        updated.pos = corrected_position;
        updated.rot = SO3(corrected_rotation);
        state_out = updated;
        kf_output.change_x(updated);
    }
    refreshDownsampledWorldCloudFromCurrentState();

    PointCloudXYZI::Ptr optimized_map = experimental_loop_backend->buildOptimizedMap();
    if (optimized_map && !optimized_map->empty()) {
        ivox_.reset(new IVoxType(ivox_options_));
        ivox_->AddPoints(optimized_map->points);
        if (full_map_publish_en) {
            full_map_cloud_for_viz = optimized_map;
            finalizeCloudMetadata(*full_map_cloud_for_viz);
            last_full_map_publish_time = -1.0;
        }
    }
    rebuildPathFromExperimentalLoop(experimental_loop_backend->optimizedPoses());

    RCLCPP_WARN(LOGGER,
                "实验性全局闭环已回写 Point-LIO 状态: keyframe=%zu, pos=[%.3f, %.3f, %.3f], "
                "iVox 已按优化关键帧重建",
                correction.keyframe_id, corrected_position.x(), corrected_position.y(), corrected_position.z());
}

void MapIncremental() {
    PointVector points_to_add;
    int cur_pts = feats_down_world->size();
    points_to_add.reserve(cur_pts);

    for (size_t i = 0; i < cur_pts; ++i) {
        /* decide if need add to map */
        PointType& point_world = feats_down_world->points[i];
        if (!Nearest_Points[i].empty()) {
            const PointVector& points_near = Nearest_Points[i];

            Eigen::Vector3f center =
                ((point_world.getVector3fMap() / filter_size_map_min).array().floor() + 0.5) * filter_size_map_min;
            bool need_add = true;
            for (int readd_i = 0; readd_i < points_near.size(); readd_i++) {
                Eigen::Vector3f dis_2_center = points_near[readd_i].getVector3fMap() - center;
                if (fabs(dis_2_center.x()) < 0.5 * filter_size_map_min &&
                    fabs(dis_2_center.y()) < 0.5 * filter_size_map_min &&
                    fabs(dis_2_center.z()) < 0.5 * filter_size_map_min) {
                    need_add = false;
                    break;
                }
            }
            if (need_add) {
                points_to_add.emplace_back(point_world);
            }
        } else {
            points_to_add.emplace_back(point_world);
        }
    }
    ivox_->AddPoints(points_to_add);
    appendFullMapForViz(points_to_add);
}

void publish_init_map(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pubLaserCloudFullRes) {
    if (!init_feats_world || init_feats_world->empty()) {
        return;
    }

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    finalizeCloudMetadata(*init_feats_world);
    pcl::toROSMsg(*init_feats_world, laserCloudmsg);

    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = odom_frame;
    pubLaserCloudFullRes->publish(laserCloudmsg);
}

PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());
int pending_pcd_scan_count = 0;

std::filesystem::path ensurePcdDirectory() {
    const auto pcd_dir = std::filesystem::path(ROOT_DIR) / "PCD";
    std::error_code ec;

    const bool exists = std::filesystem::exists(pcd_dir, ec);
    if (ec) {
        RCLCPP_ERROR(LOGGER, "无法检查 PCD 目录 %s: %s", pcd_dir.string().c_str(), ec.message().c_str());
        return {};
    }

    if (!exists && !std::filesystem::create_directories(pcd_dir, ec) && ec) {
        RCLCPP_ERROR(LOGGER, "无法创建 PCD 目录 %s: %s", pcd_dir.string().c_str(), ec.message().c_str());
        return {};
    }

    return pcd_dir;
}

std::string nextPcdOutputPath() {
    const auto pcd_dir = ensurePcdDirectory();
    if (pcd_dir.empty()) {
        return {};
    }

    if (pcd_save_interval > 0) {
        ++pcd_index;
        return (pcd_dir / ("scans_" + std::to_string(pcd_index) + ".pcd")).string();
    }

    return (pcd_dir / "scans.pcd").string();
}

bool flushPendingPcd(const char* reason) {
    if (!pcd_save_en || !pcl_wait_save || pcl_wait_save->empty()) {
        return true;
    }

    const auto output_path = nextPcdOutputPath();
    if (output_path.empty()) {
        return false;
    }

    pcl::PCDWriter pcd_writer;
    const int rc = pcd_writer.writeBinary(output_path, *pcl_wait_save);
    const std::string reason_text = pcdFlushReasonText(reason);
    if (rc != 0) {
        RCLCPP_ERROR(LOGGER, "累计点云保存失败: path=%s, 原因=%s, 点数=%zu, 返回码=%d",
                     output_path.c_str(), reason_text.c_str(), pcl_wait_save->size(), rc);
        return false;
    }

    RCLCPP_INFO(LOGGER, "累计点云已保存: path=%s, 原因=%s, 点数=%zu", output_path.c_str(), reason_text.c_str(),
                pcl_wait_save->size());
    pcl_wait_save->clear();
    pending_pcd_scan_count = 0;
    return true;
}

void accumulatePcdForSave() {
    if (!pcd_save_en || !feats_down_world || feats_down_world->empty()) {
        return;
    }

    *pcl_wait_save += *feats_down_world;
    finalizeCloudMetadata(*pcl_wait_save);
    ++pending_pcd_scan_count;

    if (pcd_save_interval > 0 && pending_pcd_scan_count >= pcd_save_interval) {
        flushPendingPcd("interval flush");
    }
}

void publish_frame_world(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pubLaserCloudFullRes) {
    if (scan_pub_en) {
        if (feats_down_world && !feats_down_world->empty()) {
            sensor_msgs::msg::PointCloud2 laserCloudmsg;
            finalizeCloudMetadata(*feats_down_world);
            pcl::toROSMsg(*feats_down_world, laserCloudmsg);

            laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
            laserCloudmsg.header.frame_id = odom_frame;
            pubLaserCloudFullRes->publish(laserCloudmsg);
        }
    }

    // PCD 保存不应依赖 scan_publish_en；即使关闭实时点云输出，只要启用 pcd_save，也应持续累计并在退出时落盘。
    accumulatePcdForSave();
}

void publish_frame_body(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pubLaserCloudFull_body) {
    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++) {
        pointBodyLidarToIMU(&feats_undistort->points[i], &laserCloudIMUBody->points[i]);
    }

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = body_frame;
    pubLaserCloudFull_body->publish(laserCloudmsg);
}

template <typename T>
void set_posestamp(T& out) {
    auto set_output_from_kf = [&](const auto& kf) {
        out.position.x = kf.x_.pos(0);
        out.position.y = kf.x_.pos(1);
        out.position.z = kf.x_.pos(2);
        Eigen::Quaterniond q(kf.x_.rot);
        out.orientation.x = q.coeffs()[0];
        out.orientation.y = q.coeffs()[1];
        out.orientation.z = q.coeffs()[2];
        out.orientation.w = q.coeffs()[3];
    };

    if (!use_imu_as_input) {
        set_output_from_kf(kf_output);
    } else {
        set_output_from_kf(kf_input);
    }
}

void publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr& pubOdomAftMapped,
                      std::shared_ptr<tf2_ros::TransformBroadcaster>& tf_br) {
    odomAftMapped.header.frame_id = odom_frame;
    odomAftMapped.child_frame_id = body_frame;
    if (publish_odometry_without_downsample) {
        odomAftMapped.header.stamp = get_ros_time(time_current);
    } else {
        odomAftMapped.header.stamp = get_ros_time(lidar_end_time);
    }
    set_posestamp(odomAftMapped.pose.pose);

    // Twist follows child_frame_id (body_frame) semantics.
    const Eigen::Quaterniond q_world_from_body(odomAftMapped.pose.pose.orientation.w, odomAftMapped.pose.pose.orientation.x,
                                               odomAftMapped.pose.pose.orientation.y, odomAftMapped.pose.pose.orientation.z);
    const V3D v_world = kf_output.x_.vel;
    const V3D v_body = q_world_from_body.inverse() * v_world;
    odomAftMapped.twist.twist.linear.x = v_body(0);
    odomAftMapped.twist.twist.linear.y = v_body(1);
    odomAftMapped.twist.twist.linear.z = v_body(2);

    if (!use_imu_as_input) {
        odomAftMapped.twist.twist.angular.x = kf_output.x_.omg(0);
        odomAftMapped.twist.twist.angular.y = kf_output.x_.omg(1);
        odomAftMapped.twist.twist.angular.z = kf_output.x_.omg(2);
    } else {
        const V3D omega_body = angvel_avr - kf_input.x_.bg;
        odomAftMapped.twist.twist.angular.x = omega_body(0);
        odomAftMapped.twist.twist.angular.y = omega_body(1);
        odomAftMapped.twist.twist.angular.z = omega_body(2);
    }

    {
        auto& pc = odomAftMapped.pose.covariance;
        auto& tc = odomAftMapped.twist.covariance;
        std::fill(pc.begin(), pc.end(), 0.0);
        std::fill(tc.begin(), tc.end(), 0.0);
        if (!use_imu_as_input) {
            const auto& P = kf_output.get_P();
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    pc[i * 6 + j] = P(i, j);
                    pc[i * 6 + (j + 3)] = P(i, j + 3);
                    pc[(i + 3) * 6 + j] = P(i + 3, j);
                    pc[(i + 3) * 6 + (j + 3)] = P(i + 3, j + 3);
                    tc[i * 6 + j] = P(12 + i, 12 + j);
                    tc[i * 6 + (j + 3)] = P(12 + i, 15 + j);
                    tc[(i + 3) * 6 + j] = P(15 + i, 12 + j);
                    tc[(i + 3) * 6 + (j + 3)] = P(15 + i, 15 + j);
                }
            }
        }
    }

    pubOdomAftMapped->publish(odomAftMapped);

    if (tf_send_en) {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.frame_id = odom_frame;
        transform.child_frame_id = body_frame;
        transform.transform.translation.x = odomAftMapped.pose.pose.position.x;
        transform.transform.translation.y = odomAftMapped.pose.pose.position.y;
        transform.transform.translation.z = odomAftMapped.pose.pose.position.z;
        transform.transform.rotation.w = odomAftMapped.pose.pose.orientation.w;
        transform.transform.rotation.x = odomAftMapped.pose.pose.orientation.x;
        transform.transform.rotation.y = odomAftMapped.pose.pose.orientation.y;
        transform.transform.rotation.z = odomAftMapped.pose.pose.orientation.z;
        transform.header.stamp = odomAftMapped.header.stamp;
        tf_br->sendTransform(transform);
    }
}

void publish_path(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath) {
    set_posestamp(msg_body_pose.pose);
    // msg_body_pose.header.stamp = ros::Time::now();
    msg_body_pose.header.stamp = get_ros_time(lidar_end_time);
    msg_body_pose.header.frame_id = odom_frame;
    static int jjj = 0;
    jjj++;
    // if (jjj % 2 == 0) // if path is too large, the rvis will crash
    {
        path.poses.emplace_back(msg_body_pose);
        pubPath->publish(path);
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto nh = std::make_shared<rclcpp::Node>("laserMapping");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(nh);

    try {
        readParameters(nh);
    } catch (const std::exception& ex) {
        RCLCPP_FATAL(LOGGER, "Point-LIO 参数加载失败: %s", ex.what());
        rclcpp::shutdown();
        return 1;
    }
    RCLCPP_INFO(LOGGER, "LiDAR 类型配置已加载: lidar_type=%d", lidar_type);
    ivox_ = std::make_shared<IVoxType>(ivox_options_);
    if (experimental_loop_closure_enable) {
        experimental_loop_backend =
            std::make_shared<rc26_point_lio::ExperimentalLoopClosureBackend>(buildExperimentalLoopClosureConfig());
        experimental_loop_backend->start();
        RCLCPP_WARN(LOGGER,
                    "实验性全局闭环已开启: 这是非默认实验功能，会在闭环成功后回写 Point-LIO 状态并重建 iVox");
    } else {
        experimental_loop_backend.reset();
        RCLCPP_INFO(LOGGER, "实验性全局闭环未开启: experimental_loop_closure.enable=false");
    }

    auto tf_buffer = std::make_shared<tf2_ros::Buffer>(nh->get_clock());
    auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, nh, false);
    (void)tf_listener;

    if (filter_car_body) {
        Eigen::Isometry3d lidar_to_base = Eigen::Isometry3d::Identity();
        std::string tf_error;
        if (!waitForBodyFilterTransform(tf_buffer, executor, lidar_to_base, tf_error)) {
            RCLCPP_FATAL(LOGGER,
                         "filter_car_body 已开启，但 5 秒内没有等到 TF %s <- %s: %s",
                         kBodyFilterBaseFrame, kBodyFilterLidarFrame, tf_error.c_str());
            rclcpp::shutdown();
            return 1;
        }
        p_pre->setBodyFilterTransform(lidar_to_base);
        RCLCPP_INFO(LOGGER,
                    "车身 ROI 裁剪已开启，使用 TF %s <- %s，范围 x=[%.3f, %.3f], y=[%.3f, %.3f], z=[%.3f, %.3f]",
                    kBodyFilterBaseFrame, kBodyFilterLidarFrame, body_x_min, body_x_max, body_y_min, body_y_max,
                    body_z_min, body_z_max);
    } else {
        RCLCPP_INFO(LOGGER, "车身 ROI 裁剪未开启: filter_car_body=false");
    }

    path.header.stamp = get_ros_time(lidar_end_time);
    path.header.frame_id = odom_frame;

    /*** variables definition for counting ***/
    int frame_num = 0;
    double aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0,
           aver_time_propag = 0;

    memset(point_selected_surf, true, sizeof(point_selected_surf));
    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);

    auto dyn_params_handler = nh->add_on_set_parameters_callback(
        [tf_buffer](const std::vector<rclcpp::Parameter>& params) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            result.reason = "参数已接受";

            auto reject = [&](const std::string& reason) {
                result.successful = false;
                result.reason = reason;
            };

            Preprocess::BodyFilterConfig body_candidate = p_pre->getBodyFilterConfigSnapshot();
            bool body_filter_touched = false;
            bool looked_up_body_transform = false;

            for (const auto& p : params) {
                const std::string& name = p.get_name();
                if (!isBodyFilterParameter(name)) {
                    continue;
                }

                body_filter_touched = true;
                if (name == "filter_car_body") {
                    if (p.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
                        reject("filter_car_body 需要 bool 类型");
                        return result;
                    }
                    body_candidate.enabled = p.as_bool();
                    continue;
                }

                if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
                    reject(name + " 需要 double 类型");
                    return result;
                }
                const double value = p.as_double();
                if (!std::isfinite(value)) {
                    reject(name + " 必须是有限数值");
                    return result;
                }

                if (name == "body_x_min") {
                    body_candidate.x_min = value;
                } else if (name == "body_x_max") {
                    body_candidate.x_max = value;
                } else if (name == "body_y_min") {
                    body_candidate.y_min = value;
                } else if (name == "body_y_max") {
                    body_candidate.y_max = value;
                } else if (name == "body_z_min") {
                    body_candidate.z_min = value;
                } else if (name == "body_z_max") {
                    body_candidate.z_max = value;
                }
            }

            if (body_filter_touched) {
                const std::string range_error = validateBodyFilterConfig(body_candidate);
                if (!range_error.empty()) {
                    reject(range_error);
                    return result;
                }

                if (body_candidate.enabled && !body_candidate.transform_ready) {
                    Eigen::Isometry3d lidar_to_base = Eigen::Isometry3d::Identity();
                    std::string tf_error;
                    if (!lookupBodyFilterTransform(*tf_buffer, lidar_to_base, tf_error)) {
                        reject(std::string("filter_car_body 需要先拿到 TF ") + kBodyFilterBaseFrame + " <- " +
                               kBodyFilterLidarFrame + ": " + tf_error);
                        return result;
                    }
                    body_candidate.lidar_to_base = lidar_to_base;
                    body_candidate.transform_ready = true;
                    looked_up_body_transform = true;
                }
            }

            for (const auto& p : params) {
                const std::string& name = p.get_name();

                if (name.rfind("qos_overrides.", 0) == 0) {
                    continue;
                }
                if (isBodyFilterParameter(name)) {
                    continue;
                }

                reject("参数不支持运行时热更新: " + name);
                return result;
            }

            if (result.successful && body_filter_touched) {
                const auto old_body_config = p_pre->getBodyFilterConfigSnapshot();
                {
                    std::lock_guard<std::mutex> lk(runtime_param_mutex);
                    applyBodyFilterRuntimeConfig(body_candidate);
                }
                RCLCPP_INFO(LOGGER,
                            "车身 ROI 参数已更新: node=laserMapping,param=body_roi,enabled=%s->%s,x=[%.6f,%.6f]->[%.6f,%.6f],y=[%.6f,%.6f]->[%.6f,%.6f],z=[%.6f,%.6f]->[%.6f,%.6f]",
                            old_body_config.enabled ? "true" : "false", body_candidate.enabled ? "true" : "false",
                            old_body_config.x_min, old_body_config.x_max, body_candidate.x_min, body_candidate.x_max,
                            old_body_config.y_min, old_body_config.y_max, body_candidate.y_min, body_candidate.y_max,
                            old_body_config.z_min, old_body_config.z_max, body_candidate.z_min, body_candidate.z_max);
                if (looked_up_body_transform) {
                    RCLCPP_INFO(LOGGER, "车身 ROI 裁剪所需 TF 已缓存: node=laserMapping,param=filter_car_body,tf=%s<-%s",
                                kBodyFilterBaseFrame, kBodyFilterLidarFrame);
                }
            }

            return result;
        });

    Lidar_T_wrt_IMU << VEC_FROM_ARRAY(extrinT);
    Lidar_R_wrt_IMU << MAT_FROM_ARRAY(extrinR);

    if (extrinsic_est_en) {
        if (!use_imu_as_input) {
            kf_output.x_.offset_R_L_I = Lidar_R_wrt_IMU;
            kf_output.x_.offset_T_L_I = Lidar_T_wrt_IMU;
        } else {
            kf_input.x_.offset_R_L_I = Lidar_R_wrt_IMU;
            kf_input.x_.offset_T_L_I = Lidar_T_wrt_IMU;
        }
    }

    p_imu->lidar_type = p_pre->lidar_type = lidar_type;
    p_imu->imu_en = imu_en;

    kf_input.init_dyn_share_modified_2h(get_f_input, df_dx_input, h_model_input);
    kf_output.init_dyn_share_modified_3h(get_f_output, df_dx_output, h_model_output, h_model_IMU_output);
    Eigen::Matrix<double, 24, 24> P_init;  // = MD(18, 18)::Identity() * 0.1;
    reset_cov(P_init);
    kf_input.change_P(P_init);
    Eigen::Matrix<double, 30, 30> P_init_output;  // = MD(24, 24)::Identity() * 0.01;
    reset_cov_output(P_init_output);
    kf_output.change_P(P_init_output);
    Eigen::Matrix<double, 24, 24> Q_input = process_noise_cov_input();
    Eigen::Matrix<double, 30, 30> Q_output = process_noise_cov_output();
    /*** debug record ***/
    open_file();

    using FileHandle = std::unique_ptr<FILE, int (*)(FILE*)>;
    FileHandle fp(nullptr, &fclose);
    string pos_log_dir = root_dir + "/Log/pos_log.txt";
    fp.reset(fopen(pos_log_dir.c_str(), "w"));
    if (!fp) {
        RCLCPP_WARN(LOGGER, "无法打开位姿日志文件: %s", pos_log_dir.c_str());
    }

    /*** ROS subscribe initialization ***/
    auto sub_pcl_pc = nh->create_subscription<sensor_msgs::msg::PointCloud2>(
        lid_topic, rclcpp::SensorDataQoS(),
        [](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { standard_pcl_cbk(msg); });
    auto sub_imu = nh->create_subscription<sensor_msgs::msg::Imu>(imu_topic, rclcpp::SensorDataQoS(), imu_cbk);
    auto pub_laser_cloud_full_res = nh->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_registered", 20);
    auto pub_laser_cloud_full_res_body =
        nh->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_registered_body", 20);
    auto pub_laser_cloud_map = nh->create_publisher<sensor_msgs::msg::PointCloud2>("Laser_map", 20);
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_full_map_cloud;
    if (full_map_publish_en) {
        pub_full_map_cloud = nh->create_publisher<sensor_msgs::msg::PointCloud2>(full_map_topic, 1);
        RCLCPP_INFO(LOGGER,
                    "完整累计地图可视化已开启: topic=%s, 发布间隔=%.3fs, 体素=%.3fm, 最大点数=%d",
                    full_map_topic.c_str(), full_map_interval_sec, full_map_voxel_size, full_map_max_points);
    }
    auto pub_odom_aft_mapped = nh->create_publisher<nav_msgs::msg::Odometry>("state_estimation", 20);
    auto pub_path = nh->create_publisher<nav_msgs::msg::Path>("path", 20);
    auto tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(nh);

    //------------------------------------------------------------------------------------------------------
    (void)dyn_params_handler;
    rclcpp::Rate rate(500);
    while (rclcpp::ok()) {
        executor.spin_some();
        if (sync_packages(Measures)) {
            if (flg_reset) {
                RCLCPP_WARN(LOGGER, "检测到 rosbag 时间回退，重置 Point-LIO 状态");
                p_imu->Reset();
                feats_undistort.reset(new PointCloudXYZI());
                if (use_imu_as_input) {
                    // state_in = kf_input.get_x();
                    state_in = state_input();
                    kf_input.change_P(P_init);
                } else {
                    // state_out = kf_output.get_x();
                    state_out = state_output();
                    kf_output.change_P(P_init_output);
                }
                flg_first_scan = true;
                is_first_frame = true;
                flg_reset = false;
                init_map = false;

                { ivox_.reset(new IVoxType(ivox_options_)); }
                if (experimental_loop_backend) {
                    experimental_loop_backend->reset();
                }
                resetFullMapForViz();
            }

            if (flg_first_scan) {
                first_lidar_time = Measures.lidar_beg_time;
                flg_first_scan = false;
                if (first_imu_time < 1) {
                    first_imu_time = get_time_sec(imu_next.header.stamp);
                    RCLCPP_INFO(LOGGER, "收到首帧 IMU 时间戳: %.6f", first_imu_time);
                }
                time_current = 0.0;
                if (imu_en) {
                    // imu_next = *(imu_deque.front());
                    kf_input.x_.gravity << VEC_FROM_ARRAY(gravity);
                    kf_output.x_.gravity << VEC_FROM_ARRAY(gravity);
                    // kf_output.x_.acc << VEC_FROM_ARRAY(gravity);
                    // kf_output.x_.acc *= -1;

                    {
                        while (Measures.lidar_beg_time >
                               get_time_sec(imu_next.header.stamp))  // if it is needed for the new map?
                        {
                            imu_deque.pop_front();
                            if (imu_deque.empty()) {
                                break;
                            }
                            imu_last = imu_next;
                            imu_next = *(imu_deque.front());
                            // imu_deque.pop();
                        }
                    }
                } else {
                    kf_input.x_.gravity << VEC_FROM_ARRAY(gravity);   // _init);
                    kf_output.x_.gravity << VEC_FROM_ARRAY(gravity);  //_init);
                    kf_output.x_.acc << VEC_FROM_ARRAY(gravity);      //_init);
                    kf_output.x_.acc *= -1;
                    p_imu->imu_need_init_ = false;
                    // p_imu->after_imu_init_ = true;
                }
                G_m_s2 = std::sqrt(gravity[0] * gravity[0] + gravity[1] * gravity[1] + gravity[2] * gravity[2]);
            }

            double t0, t1, t2, t3, t4, t5, match_start, solve_start;
            match_time = 0;
            solve_time = 0;
            propag_time = 0;
            update_time = 0;
            t0 = omp_get_wtime();

            /*** downsample the feature points in a scan ***/
            t1 = omp_get_wtime();
            p_imu->Process(Measures, feats_undistort);
            if (space_down_sample) {
                downSizeFilterSurf.setInputCloud(feats_undistort);
                downSizeFilterSurf.filter(*feats_down_body);
                sort(feats_down_body->points.begin(), feats_down_body->points.end(), time_list);
            } else {
                feats_down_body = Measures.lidar;
                sort(feats_down_body->points.begin(), feats_down_body->points.end(), time_list);
            }
            {
                time_seq = time_compressing<int>(feats_down_body);
                feats_down_size = feats_down_body->points.size();
            }

            if (!p_imu->after_imu_init_)  // !p_imu->UseLIInit &&
            {
                if (!p_imu->imu_need_init_) {
                    V3D tmp_gravity;
                    if (imu_en) {
                        tmp_gravity = -p_imu->mean_acc / p_imu->mean_acc.norm() * G_m_s2;
                    } else {
                        tmp_gravity << VEC_FROM_ARRAY(gravity_init);
                        p_imu->after_imu_init_ = true;
                    }
                    // V3D tmp_gravity << VEC_FROM_ARRAY(gravity_init);
                    M3D rot_init;
                    p_imu->Set_init(tmp_gravity, rot_init);
                    kf_input.x_.rot = rot_init;
                    kf_output.x_.rot = rot_init;
                    // kf_input.x_.rot; //.normalize();
                    // kf_output.x_.rot; //.normalize();
                    kf_output.x_.acc = -rot_init.transpose() * kf_output.x_.gravity;
                } else {
                    continue;
                }
            }
            /*** initialize the map ***/
            if (!init_map) {
                feats_down_world->resize(feats_undistort->size());
                for (int i = 0; i < feats_undistort->size(); i++) {
                    { pointBodyToWorld(&(feats_undistort->points[i]), &(feats_down_world->points[i])); }
                }
                for (const auto& point : *feats_down_world) {
                    init_feats_world->points.emplace_back(point);
                }

                if (init_feats_world->size() >= init_map_size) {
                    ivox_->AddPoints(init_feats_world->points);
                    appendFullMapForViz(*init_feats_world);
                    publish_init_map(pub_laser_cloud_map);
                    publish_full_map_if_due(pub_full_map_cloud, nh);
                    init_feats_world.reset(new PointCloudXYZI());
                    init_map = true;
                } else {
                    init_map = false;
                }
                continue;
            }

            /*** ICP and Kalman filter update ***/
            normvec->resize(feats_down_size);
            feats_down_world->resize(feats_down_size);

            Nearest_Points.resize(feats_down_size);

            t2 = omp_get_wtime();

            /*** iterated state estimation ***/
            crossmat_list.resize(feats_down_size);
            pbody_list.resize(feats_down_size);
            // pbody_ext_list.reserve(feats_down_size);

            for (size_t i = 0; i < feats_down_body->size(); i++) {
                V3D point_this(feats_down_body->points[i].x, feats_down_body->points[i].y,
                               feats_down_body->points[i].z);
                pbody_list[i] = point_this;
                if (!extrinsic_est_en)
                // {
                //     if (!use_imu_as_input)
                //     {
                //         point_this = kf_output.x_.offset_R_L_I * point_this + kf_output.x_.offset_T_L_I;
                //     }
                //     else
                //     {
                //         point_this = kf_input.x_.offset_R_L_I * point_this + kf_input.x_.offset_T_L_I;
                //     }
                // }
                // else
                {
                    point_this = Lidar_R_wrt_IMU * point_this + Lidar_T_wrt_IMU;
                    M3D point_crossmat;
                    point_crossmat << SKEW_SYM_MATRX(point_this);
                    crossmat_list[i] = point_crossmat;
                }
            }
            if (!use_imu_as_input) {
                bool imu_upda_cov = false;
                effct_feat_num = 0;
                /**** point by point update ****/
                if (!time_seq.empty()) {
                    double pcl_beg_time = Measures.lidar_beg_time;
                    idx = -1;
                    for (k = 0; k < time_seq.size(); k++) {
                        PointType& point_body = feats_down_body->points[idx + time_seq[k]];

                        time_current = point_body.curvature / 1000.0 + pcl_beg_time;

                        if (is_first_frame) {
                            if (imu_en) {
                                while (time_current > get_time_sec(imu_next.header.stamp)) {
                                    imu_deque.pop_front();
                                    if (imu_deque.empty())
                                        break;
                                    imu_last = imu_next;
                                    imu_next = *(imu_deque.front());
                                }
                                angvel_avr << imu_last.angular_velocity.x, imu_last.angular_velocity.y,
                                    imu_last.angular_velocity.z;
                                acc_avr << imu_last.linear_acceleration.x, imu_last.linear_acceleration.y,
                                    imu_last.linear_acceleration.z;
                            }
                            is_first_frame = false;
                            imu_upda_cov = true;
                            time_update_last = time_current;
                            time_predict_last_const = time_current;
                        }
                        if (imu_en && !imu_deque.empty()) {
                            bool last_imu =
                                get_time_sec(imu_next.header.stamp) == get_time_sec(imu_deque.front()->header.stamp);
                            while (get_time_sec(imu_next.header.stamp) < time_predict_last_const &&
                                   !imu_deque.empty()) {
                                if (!last_imu) {
                                    imu_last = imu_next;
                                    imu_next = *(imu_deque.front());
                                    break;
                                } else {
                                    imu_deque.pop_front();
                                    if (imu_deque.empty())
                                        break;
                                    imu_last = imu_next;
                                    imu_next = *(imu_deque.front());
                                }
                            }
                            bool imu_comes = time_current > get_time_sec(imu_next.header.stamp);
                            while (imu_comes) {
                                imu_upda_cov = true;
                                angvel_avr << imu_next.angular_velocity.x, imu_next.angular_velocity.y,
                                    imu_next.angular_velocity.z;
                                acc_avr << imu_next.linear_acceleration.x, imu_next.linear_acceleration.y,
                                    imu_next.linear_acceleration.z;

                                /*** covariance update ***/
                                double dt = get_time_sec(imu_next.header.stamp) - time_predict_last_const;
                                kf_output.predict(dt, Q_output, input_in, true, false);
                                time_predict_last_const = get_time_sec(imu_next.header.stamp);  // big problem

                                {
                                    double dt_cov = get_time_sec(imu_next.header.stamp) - time_update_last;

                                    if (dt_cov > 0.0) {
                                        time_update_last = get_time_sec(imu_next.header.stamp);
                                        double propag_imu_start = omp_get_wtime();

                                        kf_output.predict(dt_cov, Q_output, input_in, false, true);

                                        propag_time += omp_get_wtime() - propag_imu_start;
                                        double solve_imu_start = omp_get_wtime();
                                        kf_output.update_iterated_dyn_share_IMU();
                                        solve_time += omp_get_wtime() - solve_imu_start;
                                    }
                                }
                                imu_deque.pop_front();
                                if (imu_deque.empty())
                                    break;
                                imu_last = imu_next;
                                imu_next = *(imu_deque.front());
                                imu_comes = time_current > get_time_sec(imu_next.header.stamp);
                            }
                        }
                        if (flg_reset) {
                            break;
                        }

                        double dt = time_current - time_predict_last_const;
                        double propag_state_start = omp_get_wtime();
                        if (!prop_at_freq_of_imu) {
                            double dt_cov = time_current - time_update_last;
                            if (dt_cov > 0.0) {
                                kf_output.predict(dt_cov, Q_output, input_in, false, true);
                                time_update_last = time_current;
                            }
                        }
                        kf_output.predict(dt, Q_output, input_in, true, false);
                        propag_time += omp_get_wtime() - propag_state_start;
                        time_predict_last_const = time_current;
                        double t_update_start = omp_get_wtime();

                        if (feats_down_size < 1) {
                            RCLCPP_WARN(LOGGER, "当前帧没有可用点云，本帧跳过");
                            idx += time_seq[k];
                            continue;
                        }
                        if (!kf_output.update_iterated_dyn_share_modified()) {
                            idx = idx + time_seq[k];
                            continue;
                        }
                        solve_start = omp_get_wtime();

                        if (publish_odometry_without_downsample) {
                            /******* Publish odometry *******/

                            publish_odometry(pub_odom_aft_mapped, tf_broadcaster);
                            if (runtime_pos_log) {
                                euler_cur = SO3ToEuler(kf_output.x_.rot);
                                fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " "
                                         << euler_cur.transpose() << " " << kf_output.x_.pos.transpose() << " "
                                         << kf_output.x_.vel.transpose() << " " << kf_output.x_.omg.transpose() << " "
                                         << kf_output.x_.acc.transpose() << " " << kf_output.x_.gravity.transpose()
                                         << " " << kf_output.x_.bg.transpose() << " " << kf_output.x_.ba.transpose()
                                         << " " << feats_undistort->points.size() << '\n';
                            }
                        }

                        for (int j = 0; j < time_seq[k]; j++) {
                            PointType& point_body_j = feats_down_body->points[idx + j + 1];
                            PointType& point_world_j = feats_down_world->points[idx + j + 1];
                            pointBodyToWorld(&point_body_j, &point_world_j);
                        }

                        solve_time += omp_get_wtime() - solve_start;

                        update_time += omp_get_wtime() - t_update_start;
                        idx += time_seq[k];
                        // std::cout << "pbp output effect feat num:" << effct_feat_num << '\n';
                    }
                } else {
                    if (!imu_deque.empty()) {
                        imu_last = imu_next;
                        imu_next = *(imu_deque.front());

                        while (get_time_sec(imu_next.header.stamp) > time_current &&
                               ((get_time_sec(imu_next.header.stamp) <
                                 Measures.lidar_beg_time + lidar_time_inte))) {  // >= ?
                            if (is_first_frame) {
                                {
                                    {
                                        while (get_time_sec(imu_next.header.stamp) <
                                               Measures.lidar_beg_time + lidar_time_inte) {
                                            // meas.imu.emplace_back(imu_deque.front()); should add to initialization
                                            imu_deque.pop_front();
                                            if (imu_deque.empty())
                                                break;
                                            imu_last = imu_next;
                                            imu_next = *(imu_deque.front());
                                        }
                                    }
                                    break;
                                }
                                angvel_avr << imu_last.angular_velocity.x, imu_last.angular_velocity.y,
                                    imu_last.angular_velocity.z;

                                acc_avr << imu_last.linear_acceleration.x, imu_last.linear_acceleration.y,
                                    imu_last.linear_acceleration.z;

                                imu_upda_cov = true;
                                time_update_last = time_current;
                                time_predict_last_const = time_current;

                                is_first_frame = false;
                            }
                            time_current = get_time_sec(imu_next.header.stamp);

                            if (!is_first_frame) {
                                double dt = time_current - time_predict_last_const;
                                {
                                    double dt_cov = time_current - time_update_last;
                                    if (dt_cov > 0.0) {
                                        kf_output.predict(dt_cov, Q_output, input_in, false, true);
                                        time_update_last = time_current;
                                    }
                                    kf_output.predict(dt, Q_output, input_in, true, false);
                                }

                                time_predict_last_const = time_current;

                                angvel_avr << imu_next.angular_velocity.x, imu_next.angular_velocity.y,
                                    imu_next.angular_velocity.z;
                                acc_avr << imu_next.linear_acceleration.x, imu_next.linear_acceleration.y,
                                    imu_next.linear_acceleration.z;
                                // acc_avr_norm = acc_avr * G_m_s2 / acc_norm;
                                kf_output.update_iterated_dyn_share_IMU();
                                imu_deque.pop_front();
                                if (imu_deque.empty())
                                    break;
                                imu_last = imu_next;
                                imu_next = *(imu_deque.front());
                            } else {
                                imu_deque.pop_front();
                                if (imu_deque.empty())
                                    break;
                                imu_last = imu_next;
                                imu_next = *(imu_deque.front());
                            }
                        }
                    }
                }
            } else {
                bool imu_prop_cov = false;
                effct_feat_num = 0;
                if (!time_seq.empty()) {
                    double pcl_beg_time = Measures.lidar_beg_time;
                    idx = -1;
                    for (k = 0; k < time_seq.size(); k++) {
                        PointType& point_body = feats_down_body->points[idx + time_seq[k]];
                        time_current = point_body.curvature / 1000.0 + pcl_beg_time;
                        if (is_first_frame) {
                            while (time_current > get_time_sec(imu_next.header.stamp)) {
                                imu_deque.pop_front();
                                if (imu_deque.empty())
                                    break;
                                imu_last = imu_next;
                                imu_next = *(imu_deque.front());
                            }
                            imu_prop_cov = true;

                            is_first_frame = false;
                            t_last = time_current;
                            time_update_last = time_current;
                            {
                                input_in.gyro << imu_last.angular_velocity.x, imu_last.angular_velocity.y,
                                    imu_last.angular_velocity.z;
                                input_in.acc << imu_last.linear_acceleration.x, imu_last.linear_acceleration.y,
                                    imu_last.linear_acceleration.z;
                                input_in.acc = input_in.acc * G_m_s2 / acc_norm;
                            }
                        }

                        while (time_current > get_time_sec(imu_next.header.stamp))  // && !imu_deque.empty())
                        {
                            imu_deque.pop_front();

                            input_in.gyro << imu_last.angular_velocity.x, imu_last.angular_velocity.y,
                                imu_last.angular_velocity.z;
                            input_in.acc << imu_last.linear_acceleration.x, imu_last.linear_acceleration.y,
                                imu_last.linear_acceleration.z;
                            input_in.acc = input_in.acc * G_m_s2 / acc_norm;
                            double dt = get_time_sec(imu_last.header.stamp) - t_last;

                            double dt_cov = get_time_sec(imu_last.header.stamp) - time_update_last;
                            if (dt_cov > 0.0) {
                                kf_input.predict(dt_cov, Q_input, input_in, false, true);
                                time_update_last = get_time_sec(imu_last.header.stamp);  // time_current;
                            }
                            kf_input.predict(dt, Q_input, input_in, true, false);
                            t_last = get_time_sec(imu_last.header.stamp);
                            imu_prop_cov = true;

                            if (imu_deque.empty())
                                break;
                            imu_last = imu_next;
                            imu_next = *(imu_deque.front());
                            // imu_upda_cov = true;
                        }
                        if (flg_reset) {
                            break;
                        }
                        double dt = time_current - t_last;
                        t_last = time_current;
                        double propag_start = omp_get_wtime();

                        if (!prop_at_freq_of_imu) {
                            double dt_cov = time_current - time_update_last;
                            if (dt_cov > 0.0) {
                                kf_input.predict(dt_cov, Q_input, input_in, false, true);
                                time_update_last = time_current;
                            }
                        }
                        kf_input.predict(dt, Q_input, input_in, true, false);

                        propag_time += omp_get_wtime() - propag_start;

                        double t_update_start = omp_get_wtime();

                        if (feats_down_size < 1) {
                            RCLCPP_WARN(LOGGER, "当前帧没有可用点云，本帧跳过");

                            idx += time_seq[k];
                            continue;
                        }
                        if (!kf_input.update_iterated_dyn_share_modified()) {
                            idx = idx + time_seq[k];
                            continue;
                        }

                        solve_start = omp_get_wtime();

                        if (publish_odometry_without_downsample) {
                            /******* Publish odometry *******/

                            publish_odometry(pub_odom_aft_mapped, tf_broadcaster);
                            if (runtime_pos_log) {
                                euler_cur = SO3ToEuler(kf_input.x_.rot);
                                fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " "
                                         << euler_cur.transpose() << " " << kf_input.x_.pos.transpose() << " "
                                         << kf_input.x_.vel.transpose() << " " << kf_input.x_.bg.transpose() << " "
                                         << kf_input.x_.ba.transpose() << " " << kf_input.x_.gravity.transpose() << " "
                                         << feats_undistort->points.size() << '\n';
                            }
                        }

                        for (int j = 0; j < time_seq[k]; j++) {
                            PointType& point_body_j = feats_down_body->points[idx + j + 1];
                            PointType& point_world_j = feats_down_world->points[idx + j + 1];
                            pointBodyToWorld(&point_body_j, &point_world_j);
                        }
                        solve_time += omp_get_wtime() - solve_start;

                        update_time += omp_get_wtime() - t_update_start;
                        idx = idx + time_seq[k];
                    }
                } else {
                    if (!imu_deque.empty()) {
                        imu_last = imu_next;
                        imu_next = *(imu_deque.front());
                        while (get_time_sec(imu_next.header.stamp) > time_current &&
                               ((get_time_sec(imu_next.header.stamp) <
                                 Measures.lidar_beg_time + lidar_time_inte))) {  // >= ?
                            if (is_first_frame) {
                                while (get_time_sec(imu_next.header.stamp) <
                                       Measures.lidar_beg_time + lidar_time_inte) {
                                    imu_deque.pop_front();
                                    if (imu_deque.empty())
                                        break;
                                    imu_last = imu_next;
                                    imu_next = *(imu_deque.front());
                                }

                                imu_prop_cov = true;
                                t_last = time_current;
                                time_update_last = time_current;
                                input_in.gyro << imu_last.angular_velocity.x, imu_last.angular_velocity.y,
                                    imu_last.angular_velocity.z;
                                input_in.acc << imu_last.linear_acceleration.x, imu_last.linear_acceleration.y,
                                    imu_last.linear_acceleration.z;
                                input_in.acc = input_in.acc * G_m_s2 / acc_norm;

                                is_first_frame = false;
                                break;
                            }
                            time_current = get_time_sec(imu_next.header.stamp);

                            if (!is_first_frame) {
                                double dt = time_current - t_last;

                                double dt_cov = time_current - time_update_last;
                                if (dt_cov > 0.0) {
                                    // kf_input.predict(dt_cov, Q_input, input_in, false, true);
                                    time_update_last = get_time_sec(imu_next.header.stamp);  // time_current;
                                }
                                // kf_input.predict(dt, Q_input, input_in, true, false);

                                t_last = get_time_sec(imu_next.header.stamp);

                                input_in.gyro << imu_next.angular_velocity.x, imu_next.angular_velocity.y,
                                    imu_next.angular_velocity.z;
                                input_in.acc << imu_next.linear_acceleration.x, imu_next.linear_acceleration.y,
                                    imu_next.linear_acceleration.z;
                                input_in.acc = input_in.acc * G_m_s2 / acc_norm;
                                imu_deque.pop_front();
                                if (imu_deque.empty())
                                    break;
                                imu_last = imu_next;
                                imu_next = *(imu_deque.front());
                            } else {
                                imu_deque.pop_front();
                                if (imu_deque.empty())
                                    break;
                                imu_last = imu_next;
                                imu_next = *(imu_deque.front());
                            }
                        }
                    }
                }
            }
            // M3D rot_cur_lidar;
            // {
            //     rot_cur_lidar = state.rot_end;
            // }
            // euler_cur = RotMtoEuler(rot_cur_lidar);
            // geoQuat = tf::createQuaternionMsgFromRollPitchYaw
            //                     (euler_cur(0), euler_cur(1), euler_cur(2));
            applyExperimentalLoopCorrectionIfReady();
            /******* Publish odometry downsample *******/
            if (!publish_odometry_without_downsample) {
                publish_odometry(pub_odom_aft_mapped, tf_broadcaster);
            }
            if (extrinsic_est_en) {
                const auto& R = use_imu_as_input ? kf_input.x_.offset_R_L_I : kf_output.x_.offset_R_L_I;
                const auto& T = use_imu_as_input ? kf_input.x_.offset_T_L_I : kf_output.x_.offset_T_L_I;
                RCLCPP_INFO_THROTTLE(
                    LOGGER, *nh->get_clock(), 5000,
                    "LiDAR->IMU 外参估计: R=[%.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f] T=[%.6f %.6f %.6f]",
                    R(0, 0), R(0, 1), R(0, 2), R(1, 0), R(1, 1), R(1, 2), R(2, 0), R(2, 1), R(2, 2), T(0), T(1),
                    T(2));
            }

            /*** add the feature points to map ***/
            t3 = omp_get_wtime();
            if (feats_down_size > 4) {
                MapIncremental();
                if (experimental_loop_backend && experimental_loop_backend->enabled()) {
                    experimental_loop_backend->maybeAddKeyFrame(lidar_end_time, currentLioRotation(),
                                                                currentLioPosition(),
                                                                makeExperimentalLoopBodyCloud());
                }
            }
            publish_full_map_if_due(pub_full_map_cloud, nh);
            t5 = omp_get_wtime();
            /******* Publish points *******/
            if (path_en)
                publish_path(pub_path);
            if (scan_pub_en || pcd_save_en)
                publish_frame_world(pub_laser_cloud_full_res);
            if (scan_pub_en && scan_body_pub_en)
                publish_frame_body(pub_laser_cloud_full_res_body);

            /*** Debug variables Logging ***/
            if (runtime_pos_log) {
                frame_num++;
                aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
                { aver_time_icp = aver_time_icp * (frame_num - 1) / frame_num + update_time / frame_num; }
                aver_time_match = aver_time_match * (frame_num - 1) / frame_num + (match_time) / frame_num;
                aver_time_solve = aver_time_solve * (frame_num - 1) / frame_num + solve_time / frame_num;
                aver_time_propag = aver_time_propag * (frame_num - 1) / frame_num + propag_time / frame_num;
                if (time_log_counter < MAXN) {
                    T1[time_log_counter] = Measures.lidar_beg_time;
                    s_plot[time_log_counter] = t5 - t0;
                    s_plot2[time_log_counter] = feats_undistort->points.size();
                    s_plot3[time_log_counter] = aver_time_consu;
                    time_log_counter++;
                }
                RCLCPP_INFO(LOGGER,
                            "建图耗时统计: IMU+建图+输入降采样=%.6fs, 平均匹配=%.6fs, 平均求解=%.6fs, "
                            "本帧 ICP=%.6fs, 地图增量=%.6fs, 平均总耗时=%.6fs, 平均 ICP=%.6fs, 平均传播=%.6fs",
                            t1 - t0, aver_time_match, aver_time_solve, t3 - t1, t5 - t3, aver_time_consu,
                            aver_time_icp, aver_time_propag);
                if (!publish_odometry_without_downsample) {
                    if (!use_imu_as_input) {
                        euler_cur = SO3ToEuler(kf_output.x_.rot);
                        fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " "
                                 << euler_cur.transpose() << " " << kf_output.x_.pos.transpose() << " "
                                 << kf_output.x_.vel.transpose() << " " << kf_output.x_.omg.transpose() << " "
                                 << kf_output.x_.acc.transpose() << " " << kf_output.x_.gravity.transpose() << " "
                                 << kf_output.x_.bg.transpose() << " " << kf_output.x_.ba.transpose() << " "
                                 << feats_undistort->points.size() << '\n';
                    } else {
                        euler_cur = SO3ToEuler(kf_input.x_.rot);
                        fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " "
                                 << euler_cur.transpose() << " " << kf_input.x_.pos.transpose() << " "
                                 << kf_input.x_.vel.transpose() << " " << kf_input.x_.bg.transpose() << " "
                                 << kf_input.x_.ba.transpose() << " " << kf_input.x_.gravity.transpose() << " "
                                 << feats_undistort->points.size() << '\n';
                    }
                }
                dump_lio_state_to_log(fp.get());
            }
        }
        rate.sleep();
    }
    //--------------------------save map-----------------------------------
    // 1. make sure you have enough memories
    // 2. noted that pcd save will influence the real-time performances
    if (experimental_loop_backend) {
        experimental_loop_backend->stop();
    }
    flushPendingPcd("shutdown");
    fout_out.close();
    fout_imu_pbp.close();
    rclcpp::shutdown();
    return 0;
}
