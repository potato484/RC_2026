// Copyright 2025 RC2026
// 基于 small_gicp_relocalization 移植
// 原作者: Lihan Chen
//
// Licensed under the Apache License, Version 2.0

#include "rc26_localization/localization.hpp"

#include <cstddef>
#include <limits>
#include <thread>
#include <utility>

#include "pcl/common/transforms.h"
#include "pcl/features/fpfh_omp.h"
#include "pcl/features/normal_3d_omp.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/registration/gicp.h"
#include "pcl/registration/ia_ransac.h"
#include "pcl/search/kdtree.h"
#include "pcl_conversions/pcl_conversions.h"
#include "small_gicp/pcl/pcl_registration.hpp"
#include "small_gicp/util/downsampling_omp.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "pcl/filters/filter.h"

namespace rc26_localization
{

LocalizationNode::LocalizationNode(const rclcpp::NodeOptions & options)
: Node("localization", options),
  result_t_(Eigen::Isometry3d::Identity()),
  previous_result_t_(Eigen::Isometry3d::Identity())
{
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
  
  // 全局重定位参数 (SAC-IA + ICP)
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
  
  // NDT中间层参数
  this->declare_parameter("use_ndt_refinement", true);
  this->declare_parameter("ndt_resolution", 1.0);
  this->declare_parameter("ndt_max_iterations", 50);
  this->declare_parameter("ndt_step_size", 0.1);
  this->declare_parameter("ndt_transformation_epsilon", 1e-6);
  
  // 多假设初值参数
  this->declare_parameter("use_multi_hypothesis", true);
  this->declare_parameter("num_yaw_hypotheses", 4);
  
  // [修复] 配准失败超时参数
  this->declare_parameter("registration_timeout_sec", 10.0);

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
  
  // 获取绑架检测参数
  this->get_parameter("kidnap_threshold_count", kidnap_threshold_count_);
  this->get_parameter("kidnap_fitness_threshold", kidnap_fitness_threshold_);
  
  // 获取全局重定位参数
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
  
  // 获取ISS关键点参数
  this->get_parameter("use_iss_keypoints", use_iss_keypoints_);
  this->get_parameter("iss_salient_radius", iss_salient_radius_);
  this->get_parameter("iss_non_max_radius", iss_non_max_radius_);
  this->get_parameter("iss_threshold21", iss_threshold21_);
  this->get_parameter("iss_threshold32", iss_threshold32_);
  this->get_parameter("iss_min_neighbors", iss_min_neighbors_);
  
  // 获取NDT中间层参数
  this->get_parameter("use_ndt_refinement", use_ndt_refinement_);
  this->get_parameter("ndt_resolution", ndt_resolution_);
  this->get_parameter("ndt_max_iterations", ndt_max_iterations_);
  this->get_parameter("ndt_step_size", ndt_step_size_);
  this->get_parameter("ndt_transformation_epsilon", ndt_transformation_epsilon_);
  
  // 获取多假设初值参数
  this->get_parameter("use_multi_hypothesis", use_multi_hypothesis_);
  this->get_parameter("num_yaw_hypotheses", num_yaw_hypotheses_);
  
  // [修复] 获取配准失败超时参数
  this->get_parameter("registration_timeout_sec", registration_timeout_sec_);

  // 初始位姿 [x, y, z, roll, pitch, yaw]
  if (!init_pose_.empty() && init_pose_.size() >= 6) {
    std::lock_guard<std::mutex> lock(result_mutex_);
    result_t_.translation() << init_pose_[0], init_pose_[1], init_pose_[2];
    result_t_.linear() =
      Eigen::AngleAxisd(init_pose_[5], Eigen::Vector3d::UnitZ()) *
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
  register_ = std::make_shared<
    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>();

  // TF
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  // 加载先验地图（转换和降采样在后续按需完成）
  loadGlobalMap(prior_pcd_file_);

  // 订阅点云
  pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_cloud_topic_, 10,
    std::bind(&LocalizationNode::registeredPcdCallback, this, std::placeholders::_1));

  // 订阅初始位姿
  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 10,
    std::bind(&LocalizationNode::initialPoseCallback, this, std::placeholders::_1));

  // 配准定时器 (2 Hz)
  register_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&LocalizationNode::performRegistration, this));

  // TF 发布定时器 (20 Hz)
  transform_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&LocalizationNode::publishTransform, this));

  RCLCPP_INFO(this->get_logger(), "rc26_localization 节点已启动");
}

LocalizationNode::~LocalizationNode()
{
  // 设置关闭标志，等待全局重定位线程完成
  shutdown_requested_.store(true);
  if (global_reloc_thread_.joinable()) {
    global_reloc_thread_.join();
  }
}

void LocalizationNode::loadGlobalMap(const std::string & file_name)
{
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

bool LocalizationNode::prepareTargetMap()
{
  if (!map_loaded_) {
    return false;
  }
  if (target_ready_ && target_tree_) {
    return true;
  }

  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    target_ = small_gicp::voxelgrid_sampling_omp<
      pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
      *global_map_, global_leaf_size_);
  }

  // 目标地图空检查
  size_t min_points_for_map = static_cast<size_t>(
    min_points_for_registration_ > 0 ? min_points_for_registration_ : 1);
  if (!target_ || target_->size() < min_points_for_map) {
    target_ready_ = false;
    target_tree_.reset();
    RCLCPP_ERROR(this->get_logger(),
                 "先验地图点数不足: %zu < %zu，无法准备目标地图",
                 target_ ? target_->size() : static_cast<size_t>(0), min_points_for_map);
    return false;
  }
  small_gicp::estimate_covariances_omp(*target_, num_neighbors_, num_threads_);
  target_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    target_, small_gicp::KdTreeBuilderOMP(num_threads_));
  target_ready_ = true;
  RCLCPP_INFO(this->get_logger(), "先验地图准备完成，目标点数: %zu", target_->points.size());
  return true;
}

void LocalizationNode::registeredPcdCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (!map_loaded_) {
    return;  // 地图未加载，不处理点云
  }

  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    last_scan_time_ = msg->header.stamp;
    current_scan_frame_id_ = msg->header.frame_id;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr scan(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*msg, *scan);

  // 线程安全地追加点云，并限制单帧追加规模防止OOM
  std::lock_guard<std::mutex> lock(cloud_mutex_);
  const size_t current_size = accumulated_cloud_->size();
  if (current_size >= max_accumulated_points_) {
    return;
  }
  const size_t remaining = max_accumulated_points_ - current_size;
  const size_t scan_size = scan->size();
  if (scan_size > remaining) {
    accumulated_cloud_->points.insert(
      accumulated_cloud_->points.end(),
      scan->points.begin(),
      scan->points.begin() + static_cast<std::ptrdiff_t>(remaining));
    accumulated_cloud_->width = static_cast<decltype(accumulated_cloud_->width)>(
      accumulated_cloud_->points.size());
    accumulated_cloud_->height = 1;
  } else {
    *accumulated_cloud_ += *scan;
  }
}

// 执行点云配准：将最新累积的点云与先验地图对齐，更新 map->odom 估计
void LocalizationNode::performRegistration()
{
  if (!map_loaded_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "地图未加载，跳过配准");
    return;
  }

  if (!prepareTargetMap()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "地图坐标转换未完成");
    return;
  }

  // 如果正在进行全局重定位，跳过常规配准（提前检查，避免清空点云）
  if (global_reloc_running_.load()) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                         "全局重定位进行中，跳过常规配准");
    return;
  }

  // 线程安全地获取并交换累积点云（避免拷贝）
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

  // 移除 NaN 点，防止配准异常
  pcl::Indices nan_indices;
  pcl::removeNaNFromPointCloud(*cloud_to_register, *cloud_to_register, nan_indices);

  // 对累积点云降采样（降低计算量，同时保持协方差估计质量）
  source_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *cloud_to_register, registered_leaf_size_);

  // 检查降采样后点云是否有足够的点进行配准
  size_t min_points_for_reg = static_cast<size_t>(min_points_for_registration_ > 0 ? min_points_for_registration_ : 1);
  if (!source_ || source_->size() < min_points_for_reg) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "降采样后点云不足: %zu < %zu，跳过配准",
                         source_ ? source_->size() : 0, min_points_for_reg);
    return;
  }

  small_gicp::estimate_covariances_omp(*source_, num_neighbors_, num_threads_);

  // 配准参数：线程数 / 离群点阈值 / 最大迭代次数
  register_->reduction.num_threads = num_threads_;
  register_->rejector.max_dist_sq = max_dist_sq_;
  register_->optimizer.max_iterations = gicp_max_iterations_;

  // 线程安全地获取初值
  Eigen::Isometry3d initial_guess;
  {
    std::lock_guard<std::mutex> lock(result_mutex_);
    initial_guess = previous_result_t_;
  }

  // 执行配准
  auto result = register_->align(*target_, *source_, *target_tree_, initial_guess);
  
  // 绑架检测：使用归一化误差（平均每内点误差）
  // result.error 是总误差累加和，需要除以内点数得到可比较的指标
  // [BUG修复] 当 num_inliers=0 时，返回极大值确保触发绑架检测
  const double normalized_error = (result.num_inliers > 0) 
      ? (result.error / static_cast<double>(result.num_inliers)) 
      : std::numeric_limits<double>::max();
  
  // 定期打印配准质量，用于调参（每2秒一次）
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "配准: converged=%d, inliers=%zu, normalized_error=%.4f (阈值=%.4f)",
      result.converged, result.num_inliers, normalized_error, kidnap_fitness_threshold_);
  
  if (detectKidnapping(normalized_error)) {
    RCLCPP_WARN(this->get_logger(), "检测到绑架，触发全局重定位...");
    // 在后台线程执行全局重定位（管理线程生命周期，避免 detach）
    if (global_reloc_thread_.joinable()) {
      global_reloc_thread_.join();  // 等待上一次完成
    }
    // [BUG修复] 将当前点云传递给重定位线程，避免点云丢失
    global_reloc_thread_ = std::thread(
      &LocalizationNode::performGlobalRelocalization, this, cloud_to_register);
    return;
  }

  {
    std::lock_guard<std::mutex> lock(result_mutex_);
    if (result.converged) {
      result_t_ = previous_result_t_ = result.T_target_source;
    } else {
      // [C1 修复] 未收敛时仍更新初值，但限制最大偏移量和旋转角度，防止配准发散
      // 计算平移差
      Eigen::Vector3d delta_translation =
        result.T_target_source.translation() - previous_result_t_.translation();

      // 计算旋转差 (四元数差的角度)
      Eigen::Quaterniond q_result(result.T_target_source.rotation());
      Eigen::Quaterniond q_prev(previous_result_t_.rotation());
      Eigen::Quaterniond q_diff = q_result * q_prev.inverse();
      // 确保取最短路径：w < 0 时翻转
      if (q_diff.w() < 0) {
        q_diff.coeffs() = -q_diff.coeffs();
      }
      double delta_rotation = 2.0 * std::acos(std::min(1.0, std::abs(q_diff.w())));

      const double kMaxDeltaTranslation = max_delta_translation_;
      const double kMaxDeltaRotation = max_delta_rotation_;

      if (delta_translation.norm() < kMaxDeltaTranslation &&
          delta_rotation < kMaxDeltaRotation) {
        // 偏移在合理范围内，接受结果作为下次初值
        previous_result_t_ = result.T_target_source;
        RCLCPP_WARN(this->get_logger(),
                    "GICP 配准未收敛，偏移量 %.3fm / %.2f° 可接受，更新初值",
                    delta_translation.norm(), delta_rotation * 180.0 / M_PI);
      } else {
        // 偏移过大，保持原初值不变
        RCLCPP_WARN(this->get_logger(),
                    "GICP 配准未收敛，偏移量 %.3fm / %.2f° 过大，保持原初值",
                    delta_translation.norm(), delta_rotation * 180.0 / M_PI);
      }
    }
  }

  // 配准成功时更新时间戳（线程安全）
  if (result.converged) {
    std::lock_guard<std::mutex> time_lock(registration_time_mutex_);
    last_successful_registration_time_ = this->now();
  }

  // 检查配准失败超时
  rclcpp::Time last_success_snapshot;
  {
    std::lock_guard<std::mutex> time_lock(registration_time_mutex_);
    last_success_snapshot = last_successful_registration_time_;
  }
  double time_since_last_success = (this->now() - last_success_snapshot).seconds();
  if (time_since_last_success > registration_timeout_sec_) {
    RCLCPP_ERROR(this->get_logger(), 
                 "配准失败超时 %.1f秒，触发全局重定位...",
                 time_since_last_success);

    // 触发全局重定位
    if (global_reloc_thread_.joinable()) {
      global_reloc_thread_.join();
    }
    global_reloc_thread_ = std::thread(
      &LocalizationNode::performGlobalRelocalization, this, cloud_to_register);
  }
}

void LocalizationNode::publishTransform()
{
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

  // [BUG修复] 使用当前时间发布TF，避免重复时间戳问题
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

bool LocalizationNode::detectKidnapping(double fitness_score)
{
  // 如果已经在重定位，不重复检测
  if (is_kidnapped_.load() || global_reloc_running_.load()) {
    return false;
  }
  
  if (fitness_score > kidnap_fitness_threshold_) {
    int count = consecutive_high_error_count_.fetch_add(1) + 1;
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "配准误差过高: %.4f (连续 %d/%d 帧)",
                         fitness_score, count, kidnap_threshold_count_);
    if (count >= kidnap_threshold_count_) {
      is_kidnapped_.store(true);
      consecutive_high_error_count_.store(0);
      return true;
    }
  } else {
    // 误差恢复正常，重置计数器
    consecutive_high_error_count_.store(0);
  }
  return false;
}

std::vector<Eigen::Matrix4f> LocalizationNode::generateCandidateTransforms(
  const Eigen::Matrix4f& sac_transform)
{
  std::vector<Eigen::Matrix4f> candidates;
  candidates.push_back(sac_transform);

  // 参数校验：clamp yaw假设数量，避免除零
  int yaw_hypotheses = num_yaw_hypotheses_;
  if (yaw_hypotheses < 1) {
    RCLCPP_WARN(this->get_logger(),
                "num_yaw_hypotheses_=%d 无效，已钳制为 1", yaw_hypotheses);
    yaw_hypotheses = 1;
  }

  if (!use_multi_hypothesis_) {
    return candidates;
  }

  // 基于SAC-IA结果生成多个yaw方向的假设初值
  // 绑架时previous_result_t_是错误的，应基于SAC-IA结果扰动
  // 生成不同yaw角度的候选初值（从 i=1 开始避免与 sac_transform 重复）
  const double yaw_step = 2.0 * M_PI / static_cast<double>(yaw_hypotheses);
  for (int i = 1; i < yaw_hypotheses; ++i) {
    double yaw_offset = i * yaw_step;
    Eigen::Matrix4f perturbed = sac_transform;
    Eigen::AngleAxisf rot(static_cast<float>(yaw_offset), Eigen::Vector3f::UnitZ());
    // 在机器人自身坐标系下旋转：R_new = R_old * R_yaw
    perturbed.block<3,3>(0,0) = perturbed.block<3,3>(0,0) * rot.matrix();
    candidates.push_back(perturbed);
  }
  
  RCLCPP_INFO(this->get_logger(), "生成 %zu 个候选初值用于多假设配准", candidates.size());
  return candidates;
}

void LocalizationNode::performGlobalRelocalization(
  pcl::PointCloud<pcl::PointXYZ>::Ptr passed_cloud)
{
  if (global_reloc_running_.exchange(true)) {
    return;
  }

  if (shutdown_requested_.load()) {
    global_reloc_running_.store(false);
    return;
  }

  try {
    // 异常安全保护：确保异常时标志位被正确复位
    RCLCPP_INFO(this->get_logger(), "开始 ISS + FPFH + SAC-IA + NDT + ICP 全局重定位...");
  
  // [BUG修复] 优先使用传入的点云，避免点云丢失问题
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud;
  if (passed_cloud && !passed_cloud->empty()) {
    source_cloud = passed_cloud;
  } else {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    if (accumulated_cloud_->empty()) {
      RCLCPP_WARN(this->get_logger(), "全局重定位失败：无点云数据");
      is_kidnapped_.store(false);
      global_reloc_running_.store(false);
      return;
    }
    source_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*accumulated_cloud_);
    accumulated_cloud_->clear();
  }
  
  // 移除 NaN 点
  pcl::Indices indices;
  pcl::removeNaNFromPointCloud(*source_cloud, *source_cloud, indices);
  
  // 降采样
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  float leaf_size = static_cast<float>(global_downsample_leaf_size_);
  vg.setLeafSize(leaf_size, leaf_size, leaf_size);
  vg.setInputCloud(source_cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_down(new pcl::PointCloud<pcl::PointXYZ>);
  vg.filter(*source_down);
  
  // 获取先验地图的降采样版本
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_down(new pcl::PointCloud<pcl::PointXYZ>);
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_copy(new pcl::PointCloud<pcl::PointXYZ>(*global_map_));
    pcl::Indices map_indices;
    pcl::removeNaNFromPointCloud(*map_copy, *map_copy, map_indices);
    vg.setInputCloud(map_copy);
    vg.filter(*target_down);
  }
  
  RCLCPP_INFO(this->get_logger(), "全局重定位点云: source=%zu, target=%zu",
              source_down->size(), target_down->size());
  
  // 检查点云数量
  size_t min_points = static_cast<size_t>(min_points_for_relocalization_ > 0 ? min_points_for_relocalization_ : 1);
  if (source_down->size() < min_points || target_down->size() < min_points) {
    RCLCPP_WARN(this->get_logger(), 
                "全局重定位失败：点云数量不足 (source=%zu, target=%zu, min=%zu)",
                source_down->size(), target_down->size(), min_points);
    is_kidnapped_.store(false);
    global_reloc_running_.store(false);
    return;
  }
  
  // ==================== ISS关键点提取（可选）====================
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_keypoints = source_down;
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_keypoints = target_down;
  
  if (use_iss_keypoints_) {
    RCLCPP_INFO(this->get_logger(), "使用ISS关键点提取...");
    
    // 源点云ISS关键点
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
    
    // 目标点云ISS关键点
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
    
    RCLCPP_INFO(this->get_logger(), "ISS关键点: source=%zu, target=%zu",
                source_keypoints->size(), target_keypoints->size());
    
    // 如果关键点太少，回退到使用全部点云
    if (source_keypoints->size() < 30 || target_keypoints->size() < 30) {
      RCLCPP_WARN(this->get_logger(), "ISS关键点不足，回退到全点云FPFH");
      source_keypoints = source_down;
      target_keypoints = target_down;
    }
  }
  
  // ==================== 计算法向量和FPFH特征 ====================
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
  
  // 计算 FPFH 特征
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
    RCLCPP_WARN(this->get_logger(), "FPFH 特征计算失败");
    is_kidnapped_.store(false);
    global_reloc_running_.store(false);
    return;
  }
  RCLCPP_INFO(this->get_logger(), "FPFH 特征: source=%zu, target=%zu",
              source_fpfh->size(), target_fpfh->size());
  
  // ==================== SAC-IA 粗对齐 ====================
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
  
  Eigen::Matrix4f sac_transform = sac_ia.getFinalTransformation();
  RCLCPP_INFO(this->get_logger(), "SAC-IA 完成，score=%.4f", sac_ia.getFitnessScore());
  
  // ==================== 生成多假设候选初值 ====================
  std::vector<Eigen::Matrix4f> candidates = generateCandidateTransforms(sac_transform);
  
  double best_fitness = std::numeric_limits<double>::max();
  Eigen::Matrix4f best_transform = sac_transform;
  bool any_converged = false;
  
  for (size_t ci = 0; ci < candidates.size(); ++ci) {
    Eigen::Matrix4f current_guess = candidates[ci];
    
    // ==================== NDT 中间层（可选）====================
    if (use_ndt_refinement_) {
      pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
      ndt.setInputSource(source_down);
      ndt.setInputTarget(target_down);
      ndt.setResolution(ndt_resolution_);
      ndt.setMaximumIterations(ndt_max_iterations_);
      ndt.setStepSize(ndt_step_size_);
      ndt.setTransformationEpsilon(ndt_transformation_epsilon_);
      
      pcl::PointCloud<pcl::PointXYZ> ndt_result;
      ndt.align(ndt_result, current_guess);
      
      if (ndt.hasConverged()) {
        current_guess = ndt.getFinalTransformation();
        RCLCPP_DEBUG(this->get_logger(), "候选%zu NDT收敛, score=%.4f", 
                     ci, ndt.getFitnessScore());
      }
    }
    
    // ==================== ICP 精细对齐 ====================
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(source_down);
    icp.setInputTarget(target_down);
    icp.setMaxCorrespondenceDistance(global_icp_max_correspondence_distance_);
    icp.setMaximumIterations(global_icp_max_iterations_);
    icp.setTransformationEpsilon(1e-6);
    icp.setRANSACIterations(100);
    icp.setRANSACOutlierRejectionThreshold(0.05);
    
    pcl::PointCloud<pcl::PointXYZ> icp_result;
    icp.align(icp_result, current_guess);
    
    double fitness = icp.getFitnessScore();
    bool converged = icp.hasConverged();
    
    RCLCPP_INFO(this->get_logger(), "候选%zu ICP: converged=%d, score=%.4f",
                ci, converged, fitness);
    
    if (converged && fitness < best_fitness) {
      best_fitness = fitness;
      best_transform = icp.getFinalTransformation();
      any_converged = true;
    }
    
    // 如果已经找到足够好的结果，提前退出
    if (any_converged && best_fitness < global_fitness_threshold_) {
      RCLCPP_INFO(this->get_logger(), "找到满足阈值的结果，跳过剩余候选");
      break;
    }
  }
  
  // ==================== 处理结果 ====================
  // 输入点云已在 odom 坐标系（由 odom_interface 预先转换），配准结果直接是 map->odom
  if (any_converged && best_fitness < global_fitness_threshold_) {
    Eigen::Isometry3d map_to_odom = Eigen::Isometry3d::Identity();
    map_to_odom.matrix() = best_transform.cast<double>();

    {
      std::lock_guard<std::mutex> lock(result_mutex_);
      result_t_ = previous_result_t_ = map_to_odom;
    }

    // 全局重定位成功后更新配准成功时间戳（线程安全）
    {
      std::lock_guard<std::mutex> time_lock(registration_time_mutex_);
      last_successful_registration_time_ = this->now();
    }

    RCLCPP_INFO(this->get_logger(),
                "全局重定位成功! score=%.4f, 新位置: [%.2f, %.2f, %.2f]",
                best_fitness,
                map_to_odom.translation().x(),
                map_to_odom.translation().y(),
                map_to_odom.translation().z());
  } else {
    RCLCPP_WARN(this->get_logger(),
                "全局重定位失败: best_score=%.4f > threshold=%.4f",
                best_fitness, global_fitness_threshold_);
  }

  is_kidnapped_.store(false);
  global_reloc_running_.store(false);

  } catch (const std::exception & ex) {
    RCLCPP_ERROR(this->get_logger(), "全局重定位异常: %s", ex.what());
    is_kidnapped_.store(false);
    global_reloc_running_.store(false);
  } catch (...) {
    RCLCPP_ERROR(this->get_logger(), "全局重定位异常: unknown");
    is_kidnapped_.store(false);
    global_reloc_running_.store(false);
  }
}

void LocalizationNode::initialPoseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  // 通过 RViz / 外部系统提供的初始位姿，快速对齐 map->odom
  // 收到外部初始位姿时，取消绑架状态
  is_kidnapped_.store(false);
  consecutive_high_error_count_.store(0);
  
  RCLCPP_INFO(
    this->get_logger(), "收到初始位姿: [x: %f, y: %f, z: %f]",
    msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

  Eigen::Isometry3d map_to_robot_base = Eigen::Isometry3d::Identity();
  map_to_robot_base.translation() << msg->pose.pose.position.x,
    msg->pose.pose.position.y, msg->pose.pose.position.z;
  map_to_robot_base.linear() = Eigen::Quaterniond(
    msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y, msg->pose.pose.orientation.z).toRotationMatrix();

  try {
    auto transform = tf_buffer_->lookupTransform(
      odom_frame_, robot_base_frame_, rclcpp::Time(msg->header.stamp),
      rclcpp::Duration::from_seconds(tf_timeout_sec_));
    Eigen::Isometry3d odom_to_robot_base = tf2::transformToEigen(transform.transform);
    Eigen::Isometry3d robot_to_odom = odom_to_robot_base.inverse();
    Eigen::Isometry3d map_to_odom = map_to_robot_base * robot_to_odom;

    {
      std::lock_guard<std::mutex> lock(result_mutex_);
      previous_result_t_ = result_t_ = map_to_odom;
    }
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(
      this->get_logger(), "无法查询 %s -> %s: %s",
      odom_frame_.c_str(), robot_base_frame_.c_str(), ex.what());
  }
}

}  // namespace rc26_localization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rc26_localization::LocalizationNode)
