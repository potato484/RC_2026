// Copyright 2025 RC2026
// 基于 small_gicp_relocalization 移植
// 原作者: Lihan Chen
//
// Licensed under the Apache License, Version 2.0

#include "rc26_localization/localization.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

#include "pcl/common/transforms.h"
#include "pcl/features/fpfh_omp.h"
#include "pcl/features/normal_3d_omp.h"
#include "pcl/filters/filter.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/registration/gicp.h"
#include "pcl/registration/ia_ransac.h"
#include "pcl/registration/icp.h"
#include "pcl/search/kdtree.h"
#include "pcl_conversions/pcl_conversions.h"
#include "small_gicp/pcl/pcl_registration.hpp"
#include "small_gicp/util/downsampling_omp.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

namespace rc26_localization {

namespace {
constexpr double kNearZero = 1e-6;
constexpr double kGrayZoneMin = 0.9;
constexpr double kGrayZoneMax = 1.2;
constexpr double kCostWf = 0.5;
constexpr double kCostWxy = 0.3;
constexpr double kCostWyaw = 0.2;
}  // namespace

LocalizationNode::LocalizationNode(const rclcpp::NodeOptions& options)
    : Node("localization", options), result_t_(Eigen::Isometry3d::Identity()),
      previous_result_t_(Eigen::Isometry3d::Identity()) {
    // 声明参数：全部在 YAML 中集中配置，避免魔法数散落在代码里
    this->declare_parameter("num_threads", 4);
    this->declare_parameter("num_neighbors", 20);
    this->declare_parameter("global_leaf_size", 0.25);
    this->declare_parameter("registered_leaf_size", 0.25);
    this->declare_parameter("max_dist_sq", 1.0);
    this->declare_parameter("min_points_for_registration", 20);
    this->declare_parameter("gicp_max_iterations", 20);
    this->declare_parameter("max_delta_translation", 0.5);
    this->declare_parameter("max_delta_rotation", 0.3);
    this->declare_parameter("map_frame", "map");
    this->declare_parameter("odom_frame", "odom");
    this->declare_parameter("base_frame", "");
    this->declare_parameter("robot_base_frame", "");
    this->declare_parameter("lidar_frame", "");
    this->declare_parameter("prior_pcd_file", "");
    this->declare_parameter("init_pose", std::vector<double>{0., 0., 0., 0., 0., 0.});
    this->declare_parameter("input_cloud_topic", "registered_scan");
    this->declare_parameter("tf_timeout_sec", 1.0);

    // 绑架检测参数
    this->declare_parameter("kidnap_threshold_count", 5);
    this->declare_parameter("kidnap_fitness_threshold", 0.5);

    // 全局重定位参数
    this->declare_parameter("sac_ia_num_samples", 5);
    this->declare_parameter("sac_ia_min_sample_distance", 0.1);
    this->declare_parameter("sac_ia_correspondence_randomness", 50);
    this->declare_parameter("global_icp_max_iterations", 100);
    this->declare_parameter("global_icp_max_correspondence_distance", 1.0);
    this->declare_parameter("global_fitness_threshold", 0.1);
    this->declare_parameter("min_points_for_relocalization", 50);
    this->declare_parameter("global_downsample_leaf_size", 0.5);
    this->declare_parameter("sac_ia_normal_ksearch", 20);
    this->declare_parameter("sac_ia_fpfh_ksearch", 50);

    // ISS关键点参数
    this->declare_parameter("use_iss_keypoints", true);
    this->declare_parameter("iss_salient_radius", 0.3);
    this->declare_parameter("iss_non_max_radius", 0.15);
    this->declare_parameter("iss_threshold21", 0.975);
    this->declare_parameter("iss_threshold32", 0.975);
    this->declare_parameter("iss_min_neighbors", 5);

    // NDT条件触发参数
    this->declare_parameter("use_ndt_refinement", true);
    this->declare_parameter("ndt_resolution", 1.0);
    this->declare_parameter("ndt_max_iterations", 50);
    this->declare_parameter("ndt_step_size", 0.1);
    this->declare_parameter("ndt_transformation_epsilon", 1e-6);
    this->declare_parameter("ndt_trigger_threshold", 0.5);

    // 多假设初值参数
    this->declare_parameter("use_multi_hypothesis", true);
    this->declare_parameter("num_yaw_hypotheses", 4);

    // 配准失败超时参数
    this->declare_parameter("registration_timeout_sec", 10.0);

    // I1: 冻结门控参数
    this->declare_parameter("freeze_update_err", 0.3);
    this->declare_parameter("min_inliers", 200);

    // I2: 重试区先验参数
    this->declare_parameter("retry_zone_enable", false);
    this->declare_parameter("retry_zone_x", 0.0);
    this->declare_parameter("retry_zone_y", 0.0);
    this->declare_parameter("retry_zone_yaw_candidates_deg", std::vector<double>{0.0, 90.0, 180.0, 270.0});
    this->declare_parameter("retry_zone_fast_accept_th", 0.15);
    this->declare_parameter("retry_zone_max_xy_offset", 1.5);
    this->declare_parameter("retry_zone_max_yaw_offset_deg", 60.0);
    this->declare_parameter("competition_mode", true);

    // I3: 亚克力过滤参数
    this->declare_parameter("acrylic_filter_enable", false);
    this->declare_parameter("acrylic_roi_boxes", std::vector<double>{});
    this->declare_parameter("acrylic_filter_max_stale_sec", 1.0);

    // L2: Scan Context 参数
    this->declare_parameter("enable_scan_context", true);
    this->declare_parameter("enable_fpfh_fallback", false);
    this->declare_parameter("sc_num_rings", 20);
    this->declare_parameter("sc_num_sectors", 60);
    this->declare_parameter("sc_max_radius", 8.0);
    this->declare_parameter("sc_submap_radius", 5.0);
    this->declare_parameter("sc_grid_resolution", 1.0);
    this->declare_parameter("sc_topk", 5);
    this->declare_parameter("sc_sim_threshold", 0.18);
    this->declare_parameter("sc_min_points_per_submap", 80);

    // 获取参数
    this->get_parameter("num_threads", num_threads_);
    this->get_parameter("num_neighbors", num_neighbors_);
    this->get_parameter("global_leaf_size", global_leaf_size_);
    this->get_parameter("registered_leaf_size", registered_leaf_size_);
    this->get_parameter("max_dist_sq", max_dist_sq_);
    this->get_parameter("min_points_for_registration", min_points_for_registration_);
    this->get_parameter("gicp_max_iterations", gicp_max_iterations_);
    this->get_parameter("max_delta_translation", max_delta_translation_);
    this->get_parameter("max_delta_rotation", max_delta_rotation_);
    this->get_parameter("map_frame", map_frame_);
    this->get_parameter("odom_frame", odom_frame_);
    this->get_parameter("base_frame", base_frame_);
    this->get_parameter("robot_base_frame", robot_base_frame_);
    this->get_parameter("lidar_frame", lidar_frame_);
    this->get_parameter("prior_pcd_file", prior_pcd_file_);
    this->get_parameter("init_pose", init_pose_);
    this->get_parameter("input_cloud_topic", input_cloud_topic_);
    this->get_parameter("tf_timeout_sec", tf_timeout_sec_);

    this->get_parameter("kidnap_threshold_count", kidnap_threshold_count_);
    this->get_parameter("kidnap_fitness_threshold", kidnap_fitness_threshold_);

    this->get_parameter("sac_ia_num_samples", sac_ia_num_samples_);
    this->get_parameter("sac_ia_min_sample_distance", sac_ia_min_sample_distance_);
    this->get_parameter("sac_ia_correspondence_randomness", sac_ia_correspondence_randomness_);
    this->get_parameter("global_icp_max_iterations", global_icp_max_iterations_);
    this->get_parameter("global_icp_max_correspondence_distance", global_icp_max_correspondence_distance_);
    this->get_parameter("global_fitness_threshold", global_fitness_threshold_);
    this->get_parameter("min_points_for_relocalization", min_points_for_relocalization_);
    this->get_parameter("global_downsample_leaf_size", global_downsample_leaf_size_);
    this->get_parameter("sac_ia_normal_ksearch", sac_ia_normal_ksearch_);
    this->get_parameter("sac_ia_fpfh_ksearch", sac_ia_fpfh_ksearch_);

    this->get_parameter("use_iss_keypoints", use_iss_keypoints_);
    this->get_parameter("iss_salient_radius", iss_salient_radius_);
    this->get_parameter("iss_non_max_radius", iss_non_max_radius_);
    this->get_parameter("iss_threshold21", iss_threshold21_);
    this->get_parameter("iss_threshold32", iss_threshold32_);
    this->get_parameter("iss_min_neighbors", iss_min_neighbors_);

    this->get_parameter("use_ndt_refinement", use_ndt_refinement_);
    this->get_parameter("ndt_resolution", ndt_resolution_);
    this->get_parameter("ndt_max_iterations", ndt_max_iterations_);
    this->get_parameter("ndt_step_size", ndt_step_size_);
    this->get_parameter("ndt_transformation_epsilon", ndt_transformation_epsilon_);
    this->get_parameter("ndt_trigger_threshold", ndt_trigger_threshold_);

    this->get_parameter("use_multi_hypothesis", use_multi_hypothesis_);
    this->get_parameter("num_yaw_hypotheses", num_yaw_hypotheses_);

    this->get_parameter("registration_timeout_sec", registration_timeout_sec_);

    this->get_parameter("freeze_update_err", freeze_update_err_);
    this->get_parameter("min_inliers", min_inliers_);

    this->get_parameter("retry_zone_enable", retry_zone_enable_);
    this->get_parameter("retry_zone_x", retry_zone_x_);
    this->get_parameter("retry_zone_y", retry_zone_y_);
    this->get_parameter("retry_zone_yaw_candidates_deg", retry_zone_yaw_candidates_deg_);
    this->get_parameter("retry_zone_fast_accept_th", retry_zone_fast_accept_th_);
    this->get_parameter("retry_zone_max_xy_offset", retry_zone_max_xy_offset_);
    this->get_parameter("retry_zone_max_yaw_offset_deg", retry_zone_max_yaw_offset_deg_);
    this->get_parameter("competition_mode", competition_mode_);

    this->get_parameter("acrylic_filter_enable", acrylic_filter_enable_);
    this->get_parameter("acrylic_roi_boxes", acrylic_roi_boxes_);
    this->get_parameter("acrylic_filter_max_stale_sec", acrylic_filter_max_stale_sec_);

    this->get_parameter("enable_scan_context", enable_scan_context_);
    this->get_parameter("enable_fpfh_fallback", enable_fpfh_fallback_);
    this->get_parameter("sc_num_rings", sc_num_rings_);
    this->get_parameter("sc_num_sectors", sc_num_sectors_);
    this->get_parameter("sc_max_radius", sc_max_radius_);
    this->get_parameter("sc_submap_radius", sc_submap_radius_);
    this->get_parameter("sc_grid_resolution", sc_grid_resolution_);
    this->get_parameter("sc_topk", sc_topk_);
    this->get_parameter("sc_sim_threshold", sc_sim_threshold_);
    this->get_parameter("sc_min_points_per_submap", sc_min_points_per_submap_);

    validateAndNormalizeParams();
    setLocalizationState(LocalizationState::TRACKING, "startup");

    // 初始位姿 [x, y, z, roll, pitch, yaw]
    if (!init_pose_.empty() && init_pose_.size() >= 6) {
        std::lock_guard<std::mutex> lock(result_mutex_);
        result_t_.translation() << init_pose_[0], init_pose_[1], init_pose_[2];
        result_t_.linear() = Eigen::AngleAxisd(init_pose_[5], Eigen::Vector3d::UnitZ()) *
                             Eigen::AngleAxisd(init_pose_[4], Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(init_pose_[3], Eigen::Vector3d::UnitX()).toRotationMatrix();
        previous_result_t_ = result_t_;
    } else {
        std::lock_guard<std::mutex> lock(result_mutex_);
        previous_result_t_ = result_t_;
    }

    // 初始化配准成功时间（线程安全）
    {
        std::lock_guard<std::mutex> lock(registration_time_mutex_);
        last_successful_registration_time_ = this->now();
    }

    // 初始化点云
    accumulated_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    global_map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    reloc_pending_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    register_ = std::make_shared<small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>();

    // TF
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    // 加载先验地图（转换和降采样在后续按需完成）
    loadGlobalMap(prior_pcd_file_);

    // 订阅点云
    pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_cloud_topic_, 10, std::bind(&LocalizationNode::registeredPcdCallback, this, std::placeholders::_1));

    // 订阅初始位姿
    initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "initialpose", 10, std::bind(&LocalizationNode::initialPoseCallback, this, std::placeholders::_1));

    // 配准定时器 (2 Hz)
    register_timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
                                              std::bind(&LocalizationNode::performRegistration, this));

    // TF 发布定时器 (20 Hz)
    transform_timer_ =
        this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&LocalizationNode::publishTransform, this));

    // 后台重定位工作线程（single-flight）
    reloc_worker_thread_ = std::thread(&LocalizationNode::relocWorkerLoop, this);

    RCLCPP_INFO(this->get_logger(), "rc26_localization 节点已启动");
}

LocalizationNode::~LocalizationNode() {
    shutdown_requested_.store(true);
    reloc_request_cv_.notify_all();
    if (reloc_worker_thread_.joinable()) {
        reloc_worker_thread_.join();
    }
}

void LocalizationNode::setLocalizationState(LocalizationState next, const char* reason) {
    const LocalizationState prev = localization_state_.exchange(next);
    if (prev != next) {
        RCLCPP_INFO(this->get_logger(), "定位状态切换: %s -> %s (%s)", toString(prev), toString(next),
                    reason ? reason : "n/a");
    }
}

LocalizationNode::LocalizationState LocalizationNode::getLocalizationState() const {
    return localization_state_.load();
}

bool LocalizationNode::isRelocatingState(LocalizationState state) const {
    return state == LocalizationState::FAST_RECOVERY || state == LocalizationState::GLOBAL_RECOVERY;
}

bool LocalizationNode::isMapToOdomReliableState(LocalizationState state) const {
    return state == LocalizationState::TRACKING;
}

const char* LocalizationNode::toString(LocalizationState state) {
    switch (state) {
        case LocalizationState::TRACKING:
            return "TRACKING";
        case LocalizationState::SUSPECT:
            return "SUSPECT";
        case LocalizationState::FAST_RECOVERY:
            return "FAST_RECOVERY";
        case LocalizationState::GLOBAL_RECOVERY:
            return "GLOBAL_RECOVERY";
        case LocalizationState::RELOC_FAILED:
            return "RELOC_FAILED";
        default:
            return "UNKNOWN";
    }
}

const char* LocalizationNode::toString(RelocTriggerReason reason) {
    switch (reason) {
        case RelocTriggerReason::KIDNAP:
            return "kidnap";
        case RelocTriggerReason::TIMEOUT:
            return "timeout";
        case RelocTriggerReason::MANUAL:
            return "manual";
        default:
            return "unknown";
    }
}

void LocalizationNode::publishRelocMetrics(const RelocMetrics& metrics) const {
    RCLCPP_INFO(this->get_logger(),
                "RELOC_METRIC,trigger_reason=%s,path_used=%s,t_total_ms=%.2f,t_l1_ms=%.2f,t_l2_ms=%.2f,"
                "candidate_count=%d,best_fitness=%.6f,best_J=%.6f,accepted=%d",
                toString(metrics.trigger_reason), metrics.path_used.c_str(), metrics.t_total_ms, metrics.t_l1_ms,
                metrics.t_l2_ms, metrics.candidate_count, metrics.best_fitness, metrics.best_j, metrics.accepted ? 1 : 0);
}

void LocalizationNode::requestRelocalization(RelocTriggerReason reason, pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud) {
    if (shutdown_requested_.load()) {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(reloc_request_mutex_);
        reloc_pending_reason_ = reason;
        if (source_cloud && !source_cloud->empty()) {
            reloc_pending_cloud_ = source_cloud;
        }
        reloc_request_pending_ = true;
    }

    setLocalizationState(LocalizationState::SUSPECT, "reloc_requested");
    reloc_request_cv_.notify_one();
}

void LocalizationNode::relocWorkerLoop() {
    while (!shutdown_requested_.load()) {
        RelocTriggerReason reason = RelocTriggerReason::TIMEOUT;
        pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud;

        {
            std::unique_lock<std::mutex> lock(reloc_request_mutex_);
            reloc_request_cv_.wait(lock, [this] {
                return shutdown_requested_.load() || reloc_request_pending_;
            });

            if (shutdown_requested_.load()) {
                return;
            }

            reason = reloc_pending_reason_;
            source_cloud = reloc_pending_cloud_;
            reloc_pending_cloud_.reset();
            reloc_request_pending_ = false;
        }

        performGlobalRelocalization(reason, source_cloud);
    }
}

void LocalizationNode::validateAndNormalizeParams() {
    if (kidnap_threshold_count_ < 1) {
        RCLCPP_WARN(this->get_logger(), "kidnap_threshold_count=%d 非法，已钳制为 1", kidnap_threshold_count_);
        kidnap_threshold_count_ = 1;
    }
    if (kidnap_fitness_threshold_ <= 0.0) {
        RCLCPP_WARN(this->get_logger(), "kidnap_fitness_threshold=%.4f 非法，已回退为 0.3", kidnap_fitness_threshold_);
        kidnap_fitness_threshold_ = 0.3;
    }
    if (registration_timeout_sec_ <= 0.0) {
        RCLCPP_WARN(this->get_logger(), "registration_timeout_sec=%.4f 非法，已回退为 3.0", registration_timeout_sec_);
        registration_timeout_sec_ = 3.0;
    }
    if (global_fitness_threshold_ <= 0.0) {
        RCLCPP_WARN(this->get_logger(), "global_fitness_threshold=%.4f 非法，已回退为 0.1", global_fitness_threshold_);
        global_fitness_threshold_ = 0.1;
    }
    if (freeze_update_err_ <= 0.0) {
        RCLCPP_WARN(this->get_logger(), "freeze_update_err=%.4f 非法，已回退为 0.3", freeze_update_err_);
        freeze_update_err_ = 0.3;
    }
    if (min_inliers_ < 0) {
        RCLCPP_WARN(this->get_logger(), "min_inliers=%d 非法，已钳制为 0", min_inliers_);
        min_inliers_ = 0;
    }
    if (retry_zone_fast_accept_th_ <= 0.0) {
        RCLCPP_WARN(this->get_logger(), "retry_zone_fast_accept_th=%.4f 非法，已回退为 global_fitness_threshold=%.4f",
                    retry_zone_fast_accept_th_, global_fitness_threshold_);
        retry_zone_fast_accept_th_ = global_fitness_threshold_;
    }
    if (retry_zone_max_xy_offset_ <= 0.0) {
        RCLCPP_WARN(this->get_logger(), "retry_zone_max_xy_offset=%.4f 非法，已回退为 1.5", retry_zone_max_xy_offset_);
        retry_zone_max_xy_offset_ = 1.5;
    }
    if (retry_zone_max_yaw_offset_deg_ <= 0.0 || retry_zone_max_yaw_offset_deg_ > 180.0) {
        RCLCPP_WARN(this->get_logger(), "retry_zone_max_yaw_offset_deg=%.4f 非法，已回退为 60.0",
                    retry_zone_max_yaw_offset_deg_);
        retry_zone_max_yaw_offset_deg_ = 60.0;
    }
    if (acrylic_filter_max_stale_sec_ <= 0.0) {
        RCLCPP_WARN(this->get_logger(), "acrylic_filter_max_stale_sec=%.4f 非法，已回退为 1.0",
                    acrylic_filter_max_stale_sec_);
        acrylic_filter_max_stale_sec_ = 1.0;
    }
    if (acrylic_filter_enable_ && acrylic_roi_boxes_.empty()) {
        RCLCPP_WARN(this->get_logger(), "acrylic_filter_enable=true 但 acrylic_roi_boxes 为空，已自动关闭过滤");
        acrylic_filter_enable_ = false;
    }
    if (!acrylic_roi_boxes_.empty() && (acrylic_roi_boxes_.size() % 6 != 0)) {
        const size_t raw_size = acrylic_roi_boxes_.size();
        const size_t normalized_size = (raw_size / 6) * 6;
        acrylic_roi_boxes_.resize(normalized_size);
        RCLCPP_WARN(this->get_logger(), "acrylic_roi_boxes 数量=%zu 不是 6 的倍数，已截断为 %zu", raw_size,
                    normalized_size);
    }
    for (size_t b = 0; b + 5 < acrylic_roi_boxes_.size(); b += 6) {
        if (acrylic_roi_boxes_[b] > acrylic_roi_boxes_[b + 3]) {
            std::swap(acrylic_roi_boxes_[b], acrylic_roi_boxes_[b + 3]);
            RCLCPP_WARN(this->get_logger(), "acrylic_roi_boxes[%zu] 的 xmin>xmax，已自动交换", b / 6);
        }
        if (acrylic_roi_boxes_[b + 1] > acrylic_roi_boxes_[b + 4]) {
            std::swap(acrylic_roi_boxes_[b + 1], acrylic_roi_boxes_[b + 4]);
            RCLCPP_WARN(this->get_logger(), "acrylic_roi_boxes[%zu] 的 ymin>ymax，已自动交换", b / 6);
        }
        if (acrylic_roi_boxes_[b + 2] > acrylic_roi_boxes_[b + 5]) {
            std::swap(acrylic_roi_boxes_[b + 2], acrylic_roi_boxes_[b + 5]);
            RCLCPP_WARN(this->get_logger(), "acrylic_roi_boxes[%zu] 的 zmin>zmax，已自动交换", b / 6);
        }
    }
    if (retry_zone_enable_ && retry_zone_yaw_candidates_deg_.empty()) {
        RCLCPP_WARN(this->get_logger(), "retry_zone_enable=true 但 retry_zone_yaw_candidates_deg 为空，已自动关闭快速通道");
        retry_zone_enable_ = false;
    }
    if (ndt_trigger_threshold_ <= 0.0) {
        ndt_trigger_threshold_ = 0.5;
    }

    sc_num_rings_ = std::max(sc_num_rings_, 4);
    sc_num_sectors_ = std::max(sc_num_sectors_, 12);
    sc_max_radius_ = std::max(sc_max_radius_, 1.0);
    sc_submap_radius_ = std::max(sc_submap_radius_, 1.0);
    sc_grid_resolution_ = std::max(sc_grid_resolution_, 0.2);
    sc_topk_ = std::max(sc_topk_, 1);
    sc_sim_threshold_ = std::clamp(sc_sim_threshold_, 0.01, 1.0);
    sc_min_points_per_submap_ = std::max(sc_min_points_per_submap_, 20);

    if (competition_mode_) {
        if (!retry_zone_enable_) {
            throw std::runtime_error("competition_mode=true 但 retry_zone_enable=false，拒绝启动");
        }
        if (std::abs(retry_zone_x_) <= kNearZero && std::abs(retry_zone_y_) <= kNearZero) {
            throw std::runtime_error("competition_mode=true 但 retry_zone_x/y 仍为默认占位值(0,0)，拒绝启动");
        }
    }
}

void LocalizationNode::loadGlobalMap(const std::string& file_name) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (file_name.empty()) {
        RCLCPP_ERROR(this->get_logger(), "PCD 文件路径为空，定位将不可用");
        map_loaded_ = false;
        target_ready_ = false;
        return;
    }

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *global_map_) == -1) {
        RCLCPP_ERROR(this->get_logger(), "无法读取 PCD 文件: %s，定位将不可用", file_name.c_str());
        map_loaded_ = false;
        target_ready_ = false;
        return;
    }
    RCLCPP_INFO(this->get_logger(), "加载先验地图，共 %zu 个点", global_map_->points.size());
    map_loaded_ = true;
    target_ready_ = false;
    map_needs_transform_ = false;
}

bool LocalizationNode::prepareTargetMap() {
    if (!map_loaded_) {
        return false;
    }
    if (target_ready_ && target_tree_) {
        return true;
    }

    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        target_ =
            small_gicp::voxelgrid_sampling_omp<pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
                *global_map_, global_leaf_size_);
    }

    size_t min_points_for_map =
        static_cast<size_t>(min_points_for_registration_ > 0 ? min_points_for_registration_ : 1);
    if (!target_ || target_->size() < min_points_for_map) {
        target_ready_ = false;
        target_tree_.reset();
        RCLCPP_ERROR(this->get_logger(), "先验地图点数不足: %zu < %zu，无法准备目标地图",
                     target_ ? target_->size() : static_cast<size_t>(0), min_points_for_map);
        return false;
    }
    small_gicp::estimate_covariances_omp(*target_, num_neighbors_, num_threads_);
    target_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
        target_, small_gicp::KdTreeBuilderOMP(num_threads_));
    target_ready_ = true;
    RCLCPP_INFO(this->get_logger(), "先验地图准备完成，目标点数: %zu", target_->points.size());

    if (enable_scan_context_ && !sc_db_ready_) {
        if (!buildScanContextDatabase()) {
            RCLCPP_WARN(this->get_logger(), "Scan Context 数据库构建失败，L2 将不可用");
        }
    }

    return true;
}

void LocalizationNode::registeredPcdCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (!map_loaded_) {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        last_scan_time_ = msg->header.stamp;
        current_scan_frame_id_ = msg->header.frame_id;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr scan(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *scan);

    std::lock_guard<std::mutex> lock(cloud_mutex_);
    const size_t current_size = accumulated_cloud_->size();
    if (current_size >= max_accumulated_points_) {
        return;
    }
    const size_t remaining = max_accumulated_points_ - current_size;
    const size_t scan_size = scan->size();
    if (scan_size > remaining) {
        accumulated_cloud_->points.insert(accumulated_cloud_->points.end(), scan->points.begin(),
                                          scan->points.begin() + static_cast<std::ptrdiff_t>(remaining));
        accumulated_cloud_->width = static_cast<decltype(accumulated_cloud_->width)>(accumulated_cloud_->points.size());
        accumulated_cloud_->height = 1;
    } else {
        *accumulated_cloud_ += *scan;
    }
}

void LocalizationNode::performRegistration() {
    if (!map_loaded_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "地图未加载，跳过配准");
        return;
    }

    if (!prepareTargetMap()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "地图坐标转换未完成");
        return;
    }

    const LocalizationState state = getLocalizationState();
    if (isRelocatingState(state)) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "重定位进行中，跳过常规配准");
        return;
    }
    {
        std::lock_guard<std::mutex> lock(reloc_request_mutex_);
        if (reloc_request_pending_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "重定位请求待处理，跳过常规配准");
            return;
        }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_register;
    {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        if (accumulated_cloud_->empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "无累积点云数据");
            return;
        }
        cloud_to_register = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        std::swap(cloud_to_register, accumulated_cloud_);
    }

    pcl::Indices nan_indices;
    pcl::removeNaNFromPointCloud(*cloud_to_register, *cloud_to_register, nan_indices);

    if (acrylic_filter_enable_) {
        applyAcrylicROIFilter(cloud_to_register);
    }

    source_ = small_gicp::voxelgrid_sampling_omp<pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
        *cloud_to_register, registered_leaf_size_);

    size_t min_points_for_reg =
        static_cast<size_t>(min_points_for_registration_ > 0 ? min_points_for_registration_ : 1);
    if (!source_ || source_->size() < min_points_for_reg) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "降采样后点云不足: %zu < %zu，跳过配准",
                             source_ ? source_->size() : 0, min_points_for_reg);
        return;
    }

    small_gicp::estimate_covariances_omp(*source_, num_neighbors_, num_threads_);

    register_->reduction.num_threads = num_threads_;
    register_->rejector.max_dist_sq = max_dist_sq_;
    register_->optimizer.max_iterations = gicp_max_iterations_;

    Eigen::Isometry3d initial_guess;
    {
        std::lock_guard<std::mutex> lock(result_mutex_);
        initial_guess = previous_result_t_;
    }

    auto result = register_->align(*target_, *source_, *target_tree_, initial_guess);

    const double normalized_error =
        (result.num_inliers > 0) ? (result.error / static_cast<double>(result.num_inliers))
                                 : std::numeric_limits<double>::max();

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "配准: converged=%d, inliers=%zu, normalized_error=%.4f (阈值=%.4f)", result.converged,
                         result.num_inliers, normalized_error, kidnap_fitness_threshold_);

    if (detectKidnapping(normalized_error)) {
        RCLCPP_WARN(this->get_logger(), "检测到绑架，提交重定位请求...");
        requestRelocalization(RelocTriggerReason::KIDNAP, cloud_to_register);
        return;
    }

    const bool bad_quality =
        (normalized_error > freeze_update_err_) || (result.num_inliers < static_cast<size_t>(min_inliers_));

    if (!bad_quality) {
        std::lock_guard<std::mutex> lock(result_mutex_);
        if (result.converged) {
            result_t_ = previous_result_t_ = result.T_target_source;
        } else {
            Eigen::Vector3d delta_translation = result.T_target_source.translation() - previous_result_t_.translation();

            Eigen::Quaterniond q_result(result.T_target_source.rotation());
            Eigen::Quaterniond q_prev(previous_result_t_.rotation());
            Eigen::Quaterniond q_diff = q_result * q_prev.inverse();
            if (q_diff.w() < 0) {
                q_diff.coeffs() = -q_diff.coeffs();
            }
            double delta_rotation = 2.0 * std::acos(std::min(1.0, std::abs(q_diff.w())));

            if (delta_translation.norm() < max_delta_translation_ && delta_rotation < max_delta_rotation_) {
                previous_result_t_ = result.T_target_source;
                RCLCPP_WARN(this->get_logger(), "GICP 配准未收敛，偏移量 %.3fm / %.2f° 可接受，更新初值",
                            delta_translation.norm(), delta_rotation * 180.0 / M_PI);
            } else {
                RCLCPP_WARN(this->get_logger(), "GICP 配准未收敛，偏移量 %.3fm / %.2f° 过大，保持原初值",
                            delta_translation.norm(), delta_rotation * 180.0 / M_PI);
            }
        }
    } else {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "配准质量低 (error=%.4f, inliers=%zu < %d), 冻结 TF 更新", normalized_error,
                             result.num_inliers, min_inliers_);
        setLocalizationState(LocalizationState::SUSPECT, "local_quality_bad");
    }

    if (result.converged && !bad_quality) {
        {
            std::lock_guard<std::mutex> time_lock(registration_time_mutex_);
            last_successful_registration_time_ = this->now();
        }
        setLocalizationState(LocalizationState::TRACKING, "local_registration_ok");
    }

    rclcpp::Time last_success_snapshot;
    {
        std::lock_guard<std::mutex> time_lock(registration_time_mutex_);
        last_success_snapshot = last_successful_registration_time_;
    }
    const double time_since_last_success = (this->now() - last_success_snapshot).seconds();
    if (time_since_last_success > registration_timeout_sec_) {
        RCLCPP_ERROR(this->get_logger(), "配准失败超时 %.1f秒，提交重定位请求...", time_since_last_success);
        requestRelocalization(RelocTriggerReason::TIMEOUT, cloud_to_register);
    }
}

void LocalizationNode::publishTransform() {
    if (!map_loaded_) {
        return;
    }

    Eigen::Isometry3d result_snapshot;
    {
        std::lock_guard<std::mutex> lock(result_mutex_);
        result_snapshot = result_t_;
    }

    if (result_snapshot.matrix().isZero()) {
        return;
    }

    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = this->now();
    transform_stamped.header.frame_id = map_frame_;
    transform_stamped.child_frame_id = odom_frame_;

    const Eigen::Vector3d translation = result_snapshot.translation();
    const Eigen::Quaterniond rotation(result_snapshot.rotation());

    transform_stamped.transform.translation.x = translation.x();
    transform_stamped.transform.translation.y = translation.y();
    transform_stamped.transform.translation.z = translation.z();
    transform_stamped.transform.rotation.x = rotation.x();
    transform_stamped.transform.rotation.y = rotation.y();
    transform_stamped.transform.rotation.z = rotation.z();
    transform_stamped.transform.rotation.w = rotation.w();

    tf_broadcaster_->sendTransform(transform_stamped);
}

bool LocalizationNode::detectKidnapping(double fitness_score) {
    if (isRelocatingState(getLocalizationState())) {
        return false;
    }

    if (fitness_score > kidnap_fitness_threshold_) {
        const int count = consecutive_high_error_count_.fetch_add(1) + 1;
        setLocalizationState(LocalizationState::SUSPECT, "high_error_counting");
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "配准误差过高: %.4f (连续 %d/%d 帧)",
                             fitness_score, count, kidnap_threshold_count_);
        if (count >= kidnap_threshold_count_) {
            consecutive_high_error_count_.store(0);
            return true;
        }
    } else {
        consecutive_high_error_count_.store(0);
    }
    return false;
}

std::vector<Eigen::Matrix4f> LocalizationNode::generateCandidateTransforms(const Eigen::Matrix4f& sac_transform) {
    std::vector<Eigen::Matrix4f> candidates;
    candidates.push_back(sac_transform);

    int yaw_hypotheses = num_yaw_hypotheses_;
    if (yaw_hypotheses < 1) {
        yaw_hypotheses = 1;
    }

    if (!use_multi_hypothesis_) {
        return candidates;
    }

    const double yaw_step = 2.0 * M_PI / static_cast<double>(yaw_hypotheses);
    for (int i = 1; i < yaw_hypotheses; ++i) {
        const double yaw_offset = i * yaw_step;
        Eigen::Matrix4f perturbed = sac_transform;
        Eigen::AngleAxisf rot(static_cast<float>(yaw_offset), Eigen::Vector3f::UnitZ());
        perturbed.block<3, 3>(0, 0) = perturbed.block<3, 3>(0, 0) * rot.matrix();
        candidates.push_back(perturbed);
    }

    return candidates;
}

double LocalizationNode::computeCandidateCost(double fitness, const Eigen::Matrix4f& seed,
                                              const Eigen::Matrix4f& refined) const {
    const double fitness_th = std::max(global_fitness_threshold_, kNearZero);
    const double xy_th = std::max(retry_zone_max_xy_offset_, kNearZero);
    const double yaw_th = std::max(retry_zone_max_yaw_offset_deg_, kNearZero);

    const Eigen::Vector2d seed_xy(seed(0, 3), seed(1, 3));
    const Eigen::Vector2d refined_xy(refined(0, 3), refined(1, 3));
    const double dxy = (refined_xy - seed_xy).norm();

    const double seed_yaw = std::atan2(static_cast<double>(seed(1, 0)), static_cast<double>(seed(0, 0)));
    const double refined_yaw = std::atan2(static_cast<double>(refined(1, 0)), static_cast<double>(refined(0, 0)));
    const double dyaw = std::atan2(std::sin(refined_yaw - seed_yaw), std::cos(refined_yaw - seed_yaw));
    const double dyaw_deg = std::abs(dyaw) * 180.0 / M_PI;

    return kCostWf * (fitness / fitness_th) + kCostWxy * std::pow(dxy / xy_th, 2.0) +
           kCostWyaw * std::pow(dyaw_deg / yaw_th, 2.0);
}

bool LocalizationNode::maybeConditionalNdtRefine(const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_down,
                                                 const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_down,
                                                 Eigen::Matrix4f& io_guess, double& io_fitness) const {
    if (!use_ndt_refinement_ || io_fitness <= ndt_trigger_threshold_) {
        return false;
    }

    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
    ndt.setInputSource(source_down);
    ndt.setInputTarget(target_down);
    ndt.setResolution(ndt_resolution_);
    ndt.setMaximumIterations(ndt_max_iterations_);
    ndt.setStepSize(ndt_step_size_);
    ndt.setTransformationEpsilon(ndt_transformation_epsilon_);

    pcl::PointCloud<pcl::PointXYZ> ndt_result;
    ndt.align(ndt_result, io_guess);
    if (!ndt.hasConverged()) {
        return false;
    }

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(source_down);
    icp.setInputTarget(target_down);
    icp.setMaxCorrespondenceDistance(global_icp_max_correspondence_distance_);
    icp.setMaximumIterations(global_icp_max_iterations_);
    icp.setTransformationEpsilon(1e-6);

    pcl::PointCloud<pcl::PointXYZ> icp_result;
    icp.align(icp_result, ndt.getFinalTransformation());
    if (!icp.hasConverged()) {
        return false;
    }

    const double refined_fitness = icp.getFitnessScore();
    if (refined_fitness >= io_fitness) {
        return false;
    }

    io_fitness = refined_fitness;
    io_guess = icp.getFinalTransformation();
    return true;
}

bool LocalizationNode::buildScanContextDatabase() {
    if (!enable_scan_context_) {
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(sc_mutex_);
        if (sc_db_ready_) {
            return true;
        }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr map_copy;
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        if (!global_map_ || global_map_->empty()) {
            return false;
        }
        map_copy = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*global_map_);
    }

    pcl::Indices map_indices;
    pcl::removeNaNFromPointCloud(*map_copy, *map_copy, map_indices);
    if (map_copy->empty()) {
        return false;
    }

    double min_x = std::numeric_limits<double>::max();
    double min_y = std::numeric_limits<double>::max();
    double max_x = -std::numeric_limits<double>::max();
    double max_y = -std::numeric_limits<double>::max();

    for (const auto& pt : map_copy->points) {
        min_x = std::min(min_x, static_cast<double>(pt.x));
        min_y = std::min(min_y, static_cast<double>(pt.y));
        max_x = std::max(max_x, static_cast<double>(pt.x));
        max_y = std::max(max_y, static_cast<double>(pt.y));
    }

    if (!(min_x < max_x && min_y < max_y)) {
        return false;
    }

    std::vector<ScanContextEntry> database;
    const double radius_sq = sc_submap_radius_ * sc_submap_radius_;

    for (double cx = min_x; cx <= max_x; cx += sc_grid_resolution_) {
        for (double cy = min_y; cy <= max_y; cy += sc_grid_resolution_) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr submap(new pcl::PointCloud<pcl::PointXYZ>());
            submap->points.reserve(map_copy->points.size() / 16 + 1);
            for (const auto& pt : map_copy->points) {
                const double dx = static_cast<double>(pt.x) - cx;
                const double dy = static_cast<double>(pt.y) - cy;
                if (dx * dx + dy * dy <= radius_sq) {
                    submap->points.push_back(pt);
                }
            }

            if (submap->size() < static_cast<size_t>(sc_min_points_per_submap_)) {
                continue;
            }

            ScanContextEntry entry;
            entry.center_xy = Eigen::Vector2d(cx, cy);
            entry.descriptor = makeScanContextDescriptor(submap, entry.center_xy);
            entry.ring_key = makeRingKey(entry.descriptor);
            if (entry.ring_key.size() == 0) {
                continue;
            }
            database.push_back(std::move(entry));
        }
    }

    if (database.empty()) {
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(sc_mutex_);
        sc_database_ = std::move(database);
        sc_db_ready_ = true;
    }

    RCLCPP_INFO(this->get_logger(), "Scan Context 数据库构建完成: entries=%zu", sc_database_.size());
    return true;
}

Eigen::MatrixXf LocalizationNode::makeScanContextDescriptor(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                                            const Eigen::Vector2d& center_xy) const {
    Eigen::MatrixXf descriptor = Eigen::MatrixXf::Zero(sc_num_rings_, sc_num_sectors_);
    if (!cloud || cloud->empty()) {
        return descriptor;
    }

    for (const auto& pt : cloud->points) {
        const double dx = static_cast<double>(pt.x) - center_xy.x();
        const double dy = static_cast<double>(pt.y) - center_xy.y();
        const double r = std::hypot(dx, dy);
        if (r < kNearZero || r > sc_max_radius_) {
            continue;
        }

        double theta = std::atan2(dy, dx);
        if (theta < 0.0) {
            theta += 2.0 * M_PI;
        }

        const int ring_idx = std::min(sc_num_rings_ - 1, static_cast<int>((r / sc_max_radius_) * sc_num_rings_));
        const int sector_idx =
            std::min(sc_num_sectors_ - 1, static_cast<int>((theta / (2.0 * M_PI)) * sc_num_sectors_));

        descriptor(ring_idx, sector_idx) = std::max(descriptor(ring_idx, sector_idx), static_cast<float>(pt.z));
    }

    const float max_abs = descriptor.cwiseAbs().maxCoeff();
    if (max_abs > static_cast<float>(kNearZero)) {
        descriptor /= max_abs;
    }

    return descriptor;
}

Eigen::VectorXf LocalizationNode::makeRingKey(const Eigen::MatrixXf& descriptor) const {
    if (descriptor.rows() == 0 || descriptor.cols() == 0) {
        return Eigen::VectorXf();
    }
    return descriptor.rowwise().mean();
}

double LocalizationNode::bestSectorSimilarity(const Eigen::MatrixXf& query_desc, const Eigen::MatrixXf& target_desc,
                                              int& best_shift) const {
    best_shift = 0;
    if (query_desc.rows() != target_desc.rows() || query_desc.cols() != target_desc.cols() || query_desc.size() == 0) {
        return -1.0;
    }

    const int rows = query_desc.rows();
    const int cols = query_desc.cols();

    double query_sq = 0.0;
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            const double q = query_desc(r, c);
            query_sq += q * q;
        }
    }
    if (query_sq <= kNearZero) {
        return -1.0;
    }

    double best_sim = -1.0;
    for (int shift = 0; shift < cols; ++shift) {
        double dot = 0.0;
        double target_sq = 0.0;
        for (int r = 0; r < rows; ++r) {
            for (int c = 0; c < cols; ++c) {
                const double q = query_desc(r, c);
                const double t = target_desc(r, (c + shift) % cols);
                dot += q * t;
                target_sq += t * t;
            }
        }

        if (target_sq <= kNearZero) {
            continue;
        }

        const double sim = dot / (std::sqrt(query_sq) * std::sqrt(target_sq));
        if (sim > best_sim) {
            best_sim = sim;
            best_shift = shift;
        }
    }

    return best_sim;
}

bool LocalizationNode::tryRetryZoneFastChannel(const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_down,
                                               const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_down,
                                               Eigen::Isometry3d& best_pose, double& best_fitness,
                                               double& best_cost, int& candidate_count) {
    const double accept_threshold = std::min(retry_zone_fast_accept_th_, global_fitness_threshold_);
    RCLCPP_INFO(this->get_logger(), "尝试重试区快速通道: 坐标(%.2f, %.2f), %zu 个朝向候选", retry_zone_x_, retry_zone_y_,
                retry_zone_yaw_candidates_deg_.size());

    auto source_cov =
        small_gicp::voxelgrid_sampling_omp<pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
            *source_down, registered_leaf_size_);
    if (!source_cov || source_cov->empty()) {
        return false;
    }
    small_gicp::estimate_covariances_omp(*source_cov, num_neighbors_, num_threads_);

    register_->reduction.num_threads = num_threads_;
    register_->rejector.max_dist_sq = global_icp_max_correspondence_distance_ * global_icp_max_correspondence_distance_;
    register_->optimizer.max_iterations = global_icp_max_iterations_;

    bool found = false;
    best_fitness = std::numeric_limits<double>::max();
    best_cost = std::numeric_limits<double>::max();
    candidate_count = 0;

    for (double yaw_deg : retry_zone_yaw_candidates_deg_) {
        const double yaw = yaw_deg * M_PI / 180.0;
        const float cy = static_cast<float>(std::cos(yaw));
        const float sy = static_cast<float>(std::sin(yaw));

        Eigen::Matrix4f seed = Eigen::Matrix4f::Identity();
        seed(0, 0) = cy;
        seed(0, 1) = -sy;
        seed(1, 0) = sy;
        seed(1, 1) = cy;
        seed(0, 3) = static_cast<float>(retry_zone_x_);
        seed(1, 3) = static_cast<float>(retry_zone_y_);

        Eigen::Isometry3d seed_iso = Eigen::Isometry3d::Identity();
        seed_iso.matrix() = seed.cast<double>();
        auto result = register_->align(*target_, *source_cov, *target_tree_, seed_iso);

        if (!result.converged || result.num_inliers == 0) {
            continue;
        }

        ++candidate_count;
        const Eigen::Matrix4f refined = result.T_target_source.matrix().cast<float>();
        double fitness = result.error / static_cast<double>(result.num_inliers);
        double cost = computeCandidateCost(fitness, seed, refined);

        Eigen::Matrix4f refined_mutable = refined;
        if (cost >= kGrayZoneMin && cost <= kGrayZoneMax && fitness > ndt_trigger_threshold_) {
            if (maybeConditionalNdtRefine(source_down, target_down, refined_mutable, fitness)) {
                cost = computeCandidateCost(fitness, seed, refined_mutable);
            }
        }

        RCLCPP_INFO(this->get_logger(), "RZ候选 yaw=%.1f°: fitness=%.4f, J=%.4f", yaw_deg, fitness, cost);

        if (!found || cost < best_cost || (std::abs(cost - best_cost) < 1e-6 && fitness < best_fitness)) {
            found = true;
            best_fitness = fitness;
            best_cost = cost;
            best_pose = Eigen::Isometry3d::Identity();
            best_pose.matrix() = refined_mutable.cast<double>();
        }

        if (found && best_fitness < accept_threshold && best_cost < 1.0) {
            break;
        }
    }

    return found && best_fitness < accept_threshold && best_cost < 1.0;
}

bool LocalizationNode::tryScanContextGlobalChannel(const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_down,
                                                   const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_down,
                                                   Eigen::Isometry3d& best_pose, double& best_fitness,
                                                   double& best_cost, int& candidate_count) {
    if (!enable_scan_context_) {
        return false;
    }

    if (!sc_db_ready_ && !buildScanContextDatabase()) {
        RCLCPP_WARN(this->get_logger(), "Scan Context 数据库不可用");
        return false;
    }

    if (!source_down || source_down->empty()) {
        return false;
    }

    Eigen::Vector2d query_center = Eigen::Vector2d::Zero();
    for (const auto& pt : source_down->points) {
        query_center.x() += static_cast<double>(pt.x);
        query_center.y() += static_cast<double>(pt.y);
    }
    query_center /= static_cast<double>(source_down->size());

    const Eigen::MatrixXf query_desc = makeScanContextDescriptor(source_down, query_center);
    const Eigen::VectorXf query_ring = makeRingKey(query_desc);
    if (query_ring.size() == 0) {
        return false;
    }

    std::vector<std::pair<double, size_t>> ranked;
    {
        std::lock_guard<std::mutex> lock(sc_mutex_);
        ranked.reserve(sc_database_.size());
        for (size_t i = 0; i < sc_database_.size(); ++i) {
            if (sc_database_[i].ring_key.size() != query_ring.size()) {
                continue;
            }
            const double dist = (sc_database_[i].ring_key - query_ring).norm();
            ranked.emplace_back(dist, i);
        }
    }

    if (ranked.empty()) {
        return false;
    }

    std::sort(ranked.begin(), ranked.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
    if (static_cast<int>(ranked.size()) > sc_topk_) {
        ranked.resize(static_cast<size_t>(sc_topk_));
    }

    auto source_cov =
        small_gicp::voxelgrid_sampling_omp<pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
            *source_down, registered_leaf_size_);
    if (!source_cov || source_cov->empty()) {
        return false;
    }
    small_gicp::estimate_covariances_omp(*source_cov, num_neighbors_, num_threads_);

    register_->reduction.num_threads = num_threads_;
    register_->rejector.max_dist_sq = global_icp_max_correspondence_distance_ * global_icp_max_correspondence_distance_;
    register_->optimizer.max_iterations = global_icp_max_iterations_;

    bool found = false;
    best_fitness = std::numeric_limits<double>::max();
    best_cost = std::numeric_limits<double>::max();
    candidate_count = 0;

    const double yaw_step = 2.0 * M_PI / static_cast<double>(sc_num_sectors_);

    for (const auto& [ring_dist, idx] : ranked) {
        (void)ring_dist;

        ScanContextEntry entry;
        {
            std::lock_guard<std::mutex> lock(sc_mutex_);
            if (idx >= sc_database_.size()) {
                continue;
            }
            entry = sc_database_[idx];
        }

        int best_shift = 0;
        const double similarity = bestSectorSimilarity(query_desc, entry.descriptor, best_shift);
        if (similarity < 0.0) {
            continue;
        }

        const double sim_cost = 1.0 - similarity;
        const double yaw = -static_cast<double>(best_shift) * yaw_step;

        const float cy = static_cast<float>(std::cos(yaw));
        const float sy = static_cast<float>(std::sin(yaw));

        Eigen::Matrix4f seed = Eigen::Matrix4f::Identity();
        seed(0, 0) = cy;
        seed(0, 1) = -sy;
        seed(1, 0) = sy;
        seed(1, 1) = cy;

        const Eigen::Vector2d rotated_query_center(cy * query_center.x() - sy * query_center.y(),
                                                   sy * query_center.x() + cy * query_center.y());
        const Eigen::Vector2d trans = entry.center_xy - rotated_query_center;
        seed(0, 3) = static_cast<float>(trans.x());
        seed(1, 3) = static_cast<float>(trans.y());

        Eigen::Isometry3d seed_iso = Eigen::Isometry3d::Identity();
        seed_iso.matrix() = seed.cast<double>();
        auto result = register_->align(*target_, *source_cov, *target_tree_, seed_iso);

        if (!result.converged || result.num_inliers == 0) {
            continue;
        }

        ++candidate_count;
        Eigen::Matrix4f refined = result.T_target_source.matrix().cast<float>();
        double fitness = result.error / static_cast<double>(result.num_inliers);
        double cost = computeCandidateCost(fitness, seed, refined) + std::max(0.0, sim_cost - sc_sim_threshold_);

        if (cost >= kGrayZoneMin && cost <= kGrayZoneMax && fitness > ndt_trigger_threshold_) {
            if (maybeConditionalNdtRefine(source_down, target_down, refined, fitness)) {
                cost = computeCandidateCost(fitness, seed, refined) + std::max(0.0, sim_cost - sc_sim_threshold_);
            }
        }

        RCLCPP_INFO(this->get_logger(),
                    "SC候选 idx=%zu shift=%d sim=%.3f fitness=%.4f J=%.4f center=(%.2f,%.2f)", idx, best_shift,
                    similarity, fitness, cost, entry.center_xy.x(), entry.center_xy.y());

        if (!found || cost < best_cost || (std::abs(cost - best_cost) < 1e-6 && fitness < best_fitness)) {
            found = true;
            best_fitness = fitness;
            best_cost = cost;
            best_pose = Eigen::Isometry3d::Identity();
            best_pose.matrix() = refined.cast<double>();
        }
    }

    return found && best_fitness < global_fitness_threshold_ && best_cost < 1.0;
}

bool LocalizationNode::tryLegacyFpfhGlobalChannel(const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_down,
                                                  const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_down,
                                                  Eigen::Isometry3d& best_pose, double& best_fitness,
                                                  double& best_cost, int& candidate_count) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_keypoints = source_down;
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_keypoints = target_down;

    if (use_iss_keypoints_) {
        pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> iss_src;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_iss_src(new pcl::search::KdTree<pcl::PointXYZ>);
        iss_src.setSearchMethod(tree_iss_src);
        iss_src.setInputCloud(source_down);
        iss_src.setSalientRadius(iss_salient_radius_);
        iss_src.setNonMaxRadius(iss_non_max_radius_);
        iss_src.setThreshold21(iss_threshold21_);
        iss_src.setThreshold32(iss_threshold32_);
        iss_src.setMinNeighbors(iss_min_neighbors_);
        iss_src.setNumberOfThreads(num_threads_);
        source_keypoints = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        iss_src.compute(*source_keypoints);

        pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> iss_tgt;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_iss_tgt(new pcl::search::KdTree<pcl::PointXYZ>);
        iss_tgt.setSearchMethod(tree_iss_tgt);
        iss_tgt.setInputCloud(target_down);
        iss_tgt.setSalientRadius(iss_salient_radius_);
        iss_tgt.setNonMaxRadius(iss_non_max_radius_);
        iss_tgt.setThreshold21(iss_threshold21_);
        iss_tgt.setThreshold32(iss_threshold32_);
        iss_tgt.setMinNeighbors(iss_min_neighbors_);
        iss_tgt.setNumberOfThreads(num_threads_);
        target_keypoints = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        iss_tgt.compute(*target_keypoints);

        if (source_keypoints->size() < 30 || target_keypoints->size() < 30) {
            source_keypoints = source_down;
            target_keypoints = target_down;
        }
    }

    pcl::PointCloud<pcl::Normal>::Ptr source_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Normal>::Ptr target_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setNumberOfThreads(num_threads_);
    ne.setKSearch(sac_ia_normal_ksearch_);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_src(new pcl::search::KdTree<pcl::PointXYZ>);
    ne.setSearchMethod(tree_src);
    ne.setInputCloud(source_keypoints);
    ne.compute(*source_normals);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_tgt(new pcl::search::KdTree<pcl::PointXYZ>);
    ne.setSearchMethod(tree_tgt);
    ne.setInputCloud(target_keypoints);
    ne.compute(*target_normals);

    pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_fpfh(new pcl::PointCloud<pcl::FPFHSignature33>);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_fpfh(new pcl::PointCloud<pcl::FPFHSignature33>);
    pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
    fpfh.setNumberOfThreads(num_threads_);
    fpfh.setKSearch(sac_ia_fpfh_ksearch_);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_fpfh_src(new pcl::search::KdTree<pcl::PointXYZ>);
    fpfh.setInputCloud(source_keypoints);
    fpfh.setInputNormals(source_normals);
    fpfh.setSearchMethod(tree_fpfh_src);
    fpfh.compute(*source_fpfh);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_fpfh_tgt(new pcl::search::KdTree<pcl::PointXYZ>);
    fpfh.setInputCloud(target_keypoints);
    fpfh.setInputNormals(target_normals);
    fpfh.setSearchMethod(tree_fpfh_tgt);
    fpfh.compute(*target_fpfh);

    if (source_fpfh->empty() || target_fpfh->empty()) {
        return false;
    }

    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia;
    sac_ia.setInputSource(source_keypoints);
    sac_ia.setSourceFeatures(source_fpfh);
    sac_ia.setInputTarget(target_keypoints);
    sac_ia.setTargetFeatures(target_fpfh);
    sac_ia.setMinSampleDistance(sac_ia_min_sample_distance_);
    sac_ia.setCorrespondenceRandomness(sac_ia_correspondence_randomness_);
    sac_ia.setNumberOfSamples(sac_ia_num_samples_);

    pcl::PointCloud<pcl::PointXYZ> sac_result;
    sac_ia.align(sac_result);
    const Eigen::Matrix4f sac_transform = sac_ia.getFinalTransformation();

    std::vector<Eigen::Matrix4f> candidates = generateCandidateTransforms(sac_transform);
    bool found = false;
    best_fitness = std::numeric_limits<double>::max();
    best_cost = std::numeric_limits<double>::max();
    candidate_count = 0;

    for (const auto& seed : candidates) {
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(source_down);
        icp.setInputTarget(target_down);
        icp.setMaxCorrespondenceDistance(global_icp_max_correspondence_distance_);
        icp.setMaximumIterations(global_icp_max_iterations_);
        icp.setTransformationEpsilon(1e-6);
        icp.setRANSACIterations(100);
        icp.setRANSACOutlierRejectionThreshold(0.05);

        pcl::PointCloud<pcl::PointXYZ> icp_result;
        icp.align(icp_result, seed);
        if (!icp.hasConverged()) {
            continue;
        }

        ++candidate_count;
        Eigen::Matrix4f refined = icp.getFinalTransformation();
        double fitness = icp.getFitnessScore();
        double cost = computeCandidateCost(fitness, seed, refined);

        if (cost >= kGrayZoneMin && cost <= kGrayZoneMax && fitness > ndt_trigger_threshold_) {
            if (maybeConditionalNdtRefine(source_down, target_down, refined, fitness)) {
                cost = computeCandidateCost(fitness, seed, refined);
            }
        }

        if (!found || cost < best_cost || (std::abs(cost - best_cost) < 1e-6 && fitness < best_fitness)) {
            found = true;
            best_fitness = fitness;
            best_cost = cost;
            best_pose = Eigen::Isometry3d::Identity();
            best_pose.matrix() = refined.cast<double>();
        }

        if (found && best_fitness < global_fitness_threshold_ && best_cost < 1.0) {
            break;
        }
    }

    return found && best_fitness < global_fitness_threshold_ && best_cost < 1.0;
}

void LocalizationNode::markRelocalizationSuccess(const Eigen::Isometry3d& map_to_odom) {
    {
        std::lock_guard<std::mutex> lock(result_mutex_);
        result_t_ = previous_result_t_ = map_to_odom;
    }
    {
        std::lock_guard<std::mutex> time_lock(registration_time_mutex_);
        last_successful_registration_time_ = this->now();
    }
    consecutive_high_error_count_.store(0);
    setLocalizationState(LocalizationState::TRACKING, "relocalization_success");
}

void LocalizationNode::performGlobalRelocalization(RelocTriggerReason reason,
                                                   pcl::PointCloud<pcl::PointXYZ>::Ptr passed_cloud) {
    RelocMetrics metrics;
    metrics.trigger_reason = reason;

    const auto t0 = std::chrono::steady_clock::now();

    if (shutdown_requested_.load()) {
        return;
    }

    try {
        pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud;
        if (passed_cloud && !passed_cloud->empty()) {
            source_cloud = passed_cloud;
        } else {
            std::lock_guard<std::mutex> lock(cloud_mutex_);
            if (!accumulated_cloud_->empty()) {
                source_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*accumulated_cloud_);
                accumulated_cloud_->clear();
            }
        }

        if (!source_cloud || source_cloud->empty()) {
            RCLCPP_WARN(this->get_logger(), "重定位失败：无点云数据");
            setLocalizationState(LocalizationState::RELOC_FAILED, "no_source_cloud");
            metrics.path_used = "no_cloud";
            metrics.accepted = false;
            metrics.t_total_ms =
                std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
            publishRelocMetrics(metrics);
            return;
        }

        pcl::Indices indices;
        pcl::removeNaNFromPointCloud(*source_cloud, *source_cloud, indices);

        if (acrylic_filter_enable_) {
            applyAcrylicROIFilter(source_cloud);
        }
        if (source_cloud->empty()) {
            RCLCPP_WARN(this->get_logger(), "重定位失败：过滤后无有效点云");
            setLocalizationState(LocalizationState::RELOC_FAILED, "empty_after_filter");
            metrics.path_used = "empty_after_filter";
            metrics.accepted = false;
            metrics.t_total_ms =
                std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
            publishRelocMetrics(metrics);
            return;
        }

        pcl::VoxelGrid<pcl::PointXYZ> vg;
        const float leaf_size = static_cast<float>(global_downsample_leaf_size_);
        vg.setLeafSize(leaf_size, leaf_size, leaf_size);
        vg.setInputCloud(source_cloud);
        pcl::PointCloud<pcl::PointXYZ>::Ptr source_down(new pcl::PointCloud<pcl::PointXYZ>);
        vg.filter(*source_down);

        pcl::PointCloud<pcl::PointXYZ>::Ptr target_down(new pcl::PointCloud<pcl::PointXYZ>);
        {
            std::lock_guard<std::mutex> lock(map_mutex_);
            pcl::PointCloud<pcl::PointXYZ>::Ptr map_copy(new pcl::PointCloud<pcl::PointXYZ>(*global_map_));
            pcl::Indices map_indices;
            pcl::removeNaNFromPointCloud(*map_copy, *map_copy, map_indices);
            vg.setInputCloud(map_copy);
            vg.filter(*target_down);
        }

        const size_t min_points =
            static_cast<size_t>(min_points_for_relocalization_ > 0 ? min_points_for_relocalization_ : 1);
        if (source_down->size() < min_points || target_down->size() < min_points) {
            RCLCPP_WARN(this->get_logger(), "重定位失败：点云数量不足 (source=%zu, target=%zu, min=%zu)",
                        source_down->size(), target_down->size(), min_points);
            setLocalizationState(LocalizationState::RELOC_FAILED, "insufficient_points");
            metrics.path_used = "insufficient_points";
            metrics.accepted = false;
            metrics.t_total_ms =
                std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
            publishRelocMetrics(metrics);
            return;
        }

        bool accepted = false;
        Eigen::Isometry3d best_pose = Eigen::Isometry3d::Identity();

        if (retry_zone_enable_ && !retry_zone_yaw_candidates_deg_.empty()) {
            setLocalizationState(LocalizationState::FAST_RECOVERY, "run_l1_retry_zone");
            const auto t_l1_start = std::chrono::steady_clock::now();
            double best_fitness = std::numeric_limits<double>::max();
            double best_cost = std::numeric_limits<double>::max();
            int candidate_count = 0;

            accepted = tryRetryZoneFastChannel(source_down, target_down, best_pose, best_fitness, best_cost, candidate_count);
            metrics.t_l1_ms =
                std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_l1_start).count();
            metrics.candidate_count += candidate_count;
            metrics.best_fitness = best_fitness;
            metrics.best_j = best_cost;

            if (accepted) {
                metrics.path_used = "L1";
                metrics.accepted = true;
                markRelocalizationSuccess(best_pose);
            }
        }

        if (!accepted) {
            setLocalizationState(LocalizationState::GLOBAL_RECOVERY, "run_l2_scan_context");
            const auto t_l2_start = std::chrono::steady_clock::now();

            double best_fitness = std::numeric_limits<double>::max();
            double best_cost = std::numeric_limits<double>::max();
            int candidate_count = 0;

            accepted = tryScanContextGlobalChannel(source_down, target_down, best_pose, best_fitness, best_cost,
                                                   candidate_count);

            metrics.t_l2_ms =
                std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_l2_start).count();
            metrics.candidate_count += candidate_count;
            metrics.best_fitness = std::min(metrics.best_fitness, best_fitness);
            metrics.best_j = std::min(metrics.best_j, best_cost);

            if (accepted) {
                metrics.path_used = "L2";
                metrics.accepted = true;
                markRelocalizationSuccess(best_pose);
            } else {
                metrics.path_used = "L2_failed";
            }
        }

        if (!accepted) {
            if (enable_fpfh_fallback_) {
                setLocalizationState(LocalizationState::GLOBAL_RECOVERY, "run_fallback_fpfh");
                const auto t_fb_start = std::chrono::steady_clock::now();
                double best_fitness = std::numeric_limits<double>::max();
                double best_cost = std::numeric_limits<double>::max();
                int candidate_count = 0;
                accepted = tryLegacyFpfhGlobalChannel(source_down, target_down, best_pose, best_fitness, best_cost,
                                                      candidate_count);

                metrics.t_l2_ms +=
                    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_fb_start).count();
                metrics.candidate_count += candidate_count;
                metrics.best_fitness = std::min(metrics.best_fitness, best_fitness);
                metrics.best_j = std::min(metrics.best_j, best_cost);

                if (accepted) {
                    metrics.path_used = "fallback";
                    metrics.accepted = true;
                    markRelocalizationSuccess(best_pose);
                } else {
                    metrics.path_used = "fallback_failed";
                }
            }
        }

        if (!accepted) {
            setLocalizationState(LocalizationState::RELOC_FAILED, "all_recovery_failed");
            metrics.accepted = false;
        }

    } catch (const std::exception& ex) {
        RCLCPP_ERROR(this->get_logger(), "重定位异常: %s", ex.what());
        setLocalizationState(LocalizationState::RELOC_FAILED, "exception");
        metrics.path_used = "exception";
        metrics.accepted = false;
    } catch (...) {
        RCLCPP_ERROR(this->get_logger(), "重定位异常: unknown");
        setLocalizationState(LocalizationState::RELOC_FAILED, "unknown_exception");
        metrics.path_used = "unknown_exception";
        metrics.accepted = false;
    }

    metrics.t_total_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    publishRelocMetrics(metrics);
}

void LocalizationNode::initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    consecutive_high_error_count_.store(0);

    RCLCPP_INFO(this->get_logger(), "收到初始位姿: [x: %f, y: %f, z: %f]", msg->pose.pose.position.x,
                msg->pose.pose.position.y, msg->pose.pose.position.z);

    Eigen::Isometry3d map_to_robot_base = Eigen::Isometry3d::Identity();
    map_to_robot_base.translation() << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    map_to_robot_base.linear() = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                                                    msg->pose.pose.orientation.y, msg->pose.pose.orientation.z)
                                     .toRotationMatrix();

    try {
        auto transform = tf_buffer_->lookupTransform(odom_frame_, robot_base_frame_, rclcpp::Time(msg->header.stamp),
                                                     rclcpp::Duration::from_seconds(tf_timeout_sec_));
        Eigen::Isometry3d odom_to_robot_base = tf2::transformToEigen(transform.transform);
        Eigen::Isometry3d robot_to_odom = odom_to_robot_base.inverse();
        Eigen::Isometry3d map_to_odom = map_to_robot_base * robot_to_odom;

        markRelocalizationSuccess(map_to_odom);
    } catch (tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "无法查询 %s -> %s: %s", odom_frame_.c_str(), robot_base_frame_.c_str(),
                    ex.what());
        setLocalizationState(LocalizationState::SUSPECT, "initial_pose_tf_lookup_failed");
    }
}

bool LocalizationNode::tryGetReliableMapToOdom(Eigen::Isometry3d& map_to_odom) {
    if (!isMapToOdomReliableState(getLocalizationState())) {
        return false;
    }

    rclcpp::Time last_success_snapshot;
    {
        std::lock_guard<std::mutex> time_lock(registration_time_mutex_);
        last_success_snapshot = last_successful_registration_time_;
    }
    const double stale_sec = (this->now() - last_success_snapshot).seconds();
    if (stale_sec > acrylic_filter_max_stale_sec_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "亚克力过滤跳过: map->odom 过期 %.2fs > %.2fs", stale_sec, acrylic_filter_max_stale_sec_);
        setLocalizationState(LocalizationState::SUSPECT, "map_to_odom_stale");
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(result_mutex_);
        map_to_odom = result_t_;
    }
    return true;
}

void LocalizationNode::applyAcrylicROIFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    if (acrylic_roi_boxes_.size() < 6) {
        return;
    }

    Eigen::Isometry3d map_to_odom;
    if (!tryGetReliableMapToOdom(map_to_odom)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "亚克力过滤跳过: map->odom 当前不可靠");
        return;
    }

    const size_t n_boxes = acrylic_roi_boxes_.size() / 6;
    pcl::PointCloud<pcl::PointXYZ> filtered;
    filtered.reserve(cloud->size());

    for (const auto& pt : cloud->points) {
        const Eigen::Vector3d p_map = map_to_odom * Eigen::Vector3d(pt.x, pt.y, pt.z);
        bool in_box = false;
        for (size_t i = 0; i < n_boxes && !in_box; ++i) {
            const size_t b = i * 6;
            if (p_map.x() >= acrylic_roi_boxes_[b] && p_map.x() <= acrylic_roi_boxes_[b + 3] &&
                p_map.y() >= acrylic_roi_boxes_[b + 1] && p_map.y() <= acrylic_roi_boxes_[b + 4] &&
                p_map.z() >= acrylic_roi_boxes_[b + 2] && p_map.z() <= acrylic_roi_boxes_[b + 5]) {
                in_box = true;
            }
        }
        if (!in_box) {
            filtered.push_back(pt);
        }
    }

    cloud->points = std::move(filtered.points);
    cloud->width = static_cast<uint32_t>(cloud->points.size());
    cloud->height = 1;
}

}  // namespace rc26_localization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rc26_localization::LocalizationNode)
