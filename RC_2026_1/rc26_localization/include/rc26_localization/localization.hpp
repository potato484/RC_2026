// Copyright 2025 RC2026
// 基于 small_gicp_relocalization 移植
// 原作者: Lihan Chen
//
// Licensed under the Apache License, Version 2.0

#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "pcl/io/pcd_io.h"
#include "pcl/keypoints/iss_3d.h"
#include "pcl/registration/ndt.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "small_gicp/ann/kdtree_omp.hpp"
#include "small_gicp/factors/gicp_factor.hpp"
#include "small_gicp/pcl/pcl_point.hpp"
#include "small_gicp/registration/reduction_omp.hpp"
#include "small_gicp/registration/registration.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace rc26_localization {

/**
 * @brief 基于 small_gicp 的点云重定位节点
 *
 * 功能:
 * - 订阅里程计输出的点云
 * - 与先验点云地图进行配准
 * - 发布 map -> odom 变换
 */
class LocalizationNode : public rclcpp::Node {
public:
    explicit LocalizationNode(const rclcpp::NodeOptions& options);
    ~LocalizationNode();

private:
    // 回调函数
    void registeredPcdCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

    // 核心功能
    void loadGlobalMap(const std::string& file_name);
    void performRegistration();
    void publishTransform();
    bool prepareTargetMap();

    // 全局重定位 (ISS + FPFH + SAC-IA + NDT + ICP)
    void performGlobalRelocalization(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud = nullptr);
    bool detectKidnapping(double fitness_score);

    // 多假设初值生成
    std::vector<Eigen::Matrix4f> generateCandidateTransforms(const Eigen::Matrix4f& sac_transform);

    // 订阅者
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;

    // 参数
    int num_threads_;
    int num_neighbors_;
    float global_leaf_size_;
    float registered_leaf_size_;
    float max_dist_sq_;
    std::vector<double> init_pose_;

    // 坐标系
    std::string map_frame_;
    std::string odom_frame_;
    std::string prior_pcd_file_;
    std::string base_frame_;
    std::string robot_base_frame_;
    std::string lidar_frame_;
    std::string current_scan_frame_id_;
    std::string input_cloud_topic_;

    // 状态
    rclcpp::Time last_scan_time_;
    Eigen::Isometry3d result_t_;
    Eigen::Isometry3d previous_result_t_;
    bool map_loaded_{false};  // 地图是否成功加载
    bool target_ready_{false};
    bool map_needs_transform_{false};
    double tf_timeout_sec_{1.0};
    std::mutex cloud_mutex_;              // 保护 accumulated_cloud_ 的互斥锁
    std::mutex result_mutex_;             // 保护 result_t_ 和 previous_result_t_
    std::mutex registration_time_mutex_;  // 保护 last_successful_registration_time_
    std::mutex map_mutex_;

    // 绑架检测状态
    std::atomic<int> consecutive_high_error_count_{0};
    int kidnap_threshold_count_{5};          // 连续N帧高误差判定为绑架
    double kidnap_fitness_threshold_{0.5};   // fitness_score超过此值视为异常
    int min_points_for_registration_{20};    // 局部配准最少点数
    int min_points_for_relocalization_{50};  // 全局重定位最少点数
    int gicp_max_iterations_{20};            // small_gicp 最大迭代次数
    double max_delta_translation_{0.5};      // 未收敛时允许的最大平移（米）
    double max_delta_rotation_{0.3};         // 未收敛时允许的最大旋转（弧度）
    std::atomic<bool> is_kidnapped_{false};
    std::atomic<bool> global_reloc_running_{false};
    std::atomic<bool> shutdown_requested_{false};
    std::thread global_reloc_thread_;  // 全局重定位线程，避免 detach

    // [修复] 配准失败超时机制
    rclcpp::Time last_successful_registration_time_;
    double registration_timeout_sec_{10.0};  // 配准失败超时时间（秒）

    // 全局重定位参数
    int sac_ia_num_samples_{5};
    double sac_ia_min_sample_distance_{0.1};
    int sac_ia_correspondence_randomness_{50};
    int global_icp_max_iterations_{100};
    double global_icp_max_correspondence_distance_{1.0};
    double global_fitness_threshold_{0.1};     // 全局重定位成功阈值
    double global_downsample_leaf_size_{0.5};  // 全局重定位体素滤波尺寸
    int sac_ia_normal_ksearch_{20};            // 法向量估计近邻数
    int sac_ia_fpfh_ksearch_{50};              // FPFH 特征近邻数

    // ISS关键点参数
    bool use_iss_keypoints_{true};           // 是否使用ISS关键点
    double iss_salient_radius_{0.3};         // ISS显著性半径
    double iss_non_max_radius_{0.15};        // ISS非极大值抑制半径
    double iss_threshold21_{0.975};          // ISS特征值比阈值
    double iss_threshold32_{0.975};          // ISS特征值比阈值
    int iss_min_neighbors_{5};               // ISS最小邻居数
    size_t max_accumulated_points_{100000};  // 累积点云上限，防止OOM

    // NDT中间层参数
    bool use_ndt_refinement_{true};            // 是否使用NDT中间层
    double ndt_resolution_{1.0};               // NDT体素分辨率
    int ndt_max_iterations_{50};               // NDT最大迭代次数
    double ndt_step_size_{0.1};                // NDT步长
    double ndt_transformation_epsilon_{1e-6};  // NDT收敛阈值

    // 多假设初值参数
    bool use_multi_hypothesis_{true};  // 是否使用多假设初值
    int num_yaw_hypotheses_{4};        // yaw方向假设数量

    // 点云数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud_;
    pcl::PointCloud<pcl::PointCovariance>::Ptr target_;
    pcl::PointCloud<pcl::PointCovariance>::Ptr source_;

    // small_gicp 配准
    std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> target_tree_;
    std::shared_ptr<small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>> register_;

    // 定时器
    rclcpp::TimerBase::SharedPtr transform_timer_;
    rclcpp::TimerBase::SharedPtr register_timer_;

    // TF
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace rc26_localization
