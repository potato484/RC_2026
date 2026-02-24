// Copyright 2025 RC2026
// 基于 small_gicp_relocalization 移植
// 原作者: Lihan Chen
//
// Licensed under the Apache License, Version 2.0

#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Dense>
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "pcl/io/pcd_io.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/imu.hpp"
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
    enum class LocalizationState : uint8_t {
        TRACKING,
        SUSPECT,
        FAST_RECOVERY,
        GLOBAL_RECOVERY,
        RELOC_FAILED,
    };

    enum class RelocTriggerReason : uint8_t {
        KIDNAP,
        TIMEOUT,
        MANUAL,
    };

    struct RelocMetrics {
        RelocTriggerReason trigger_reason{RelocTriggerReason::TIMEOUT};
        std::string path_used{"none"};
        double t_total_ms{0.0};
        double t_l1_ms{0.0};
        double t_l2_ms{0.0};
        int candidate_count{0};
        double best_fitness{std::numeric_limits<double>::max()};
        double best_j{std::numeric_limits<double>::max()};
        bool accepted{false};
    };

    struct ScanContextEntry {
        Eigen::Vector2d center_xy{Eigen::Vector2d::Zero()};
        Eigen::MatrixXf descriptor;
        Eigen::VectorXf ring_key;
    };

    // 回调函数
    void registeredPcdCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

    // 核心功能
    void loadGlobalMap(const std::string& file_name);
    void performRegistration();
    void publishTransform();
    bool prepareTargetMap();

    // 全局重定位（后台线程）
    void performGlobalRelocalization(RelocTriggerReason reason, pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud = nullptr);
    void requestRelocalization(RelocTriggerReason reason, pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud);
    void relocWorkerLoop();

    // 状态机与重定位结果管理
    void setLocalizationState(LocalizationState next, const char* reason);
    LocalizationState getLocalizationState() const;
    bool isRelocatingState(LocalizationState state) const;
    bool isMapToOdomReliableState(LocalizationState state) const;
    void markRelocalizationSuccess(const Eigen::Isometry3d& map_to_odom);
    void publishRelocMetrics(const RelocMetrics& metrics) const;
    static const char* toString(LocalizationState state);
    static const char* toString(RelocTriggerReason reason);

    // 全局候选评估
    double computeCandidateCost(double fitness, const Eigen::Matrix4f& seed, const Eigen::Matrix4f& refined) const;

    // L2: Scan Context 全局检索
    bool buildScanContextDatabase();
    Eigen::MatrixXf makeScanContextDescriptor(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                              const Eigen::Vector2d& center_xy) const;
    Eigen::VectorXf makeRingKey(const Eigen::MatrixXf& descriptor) const;
    double bestSectorSimilarity(const Eigen::MatrixXf& query_desc, const Eigen::MatrixXf& target_desc,
                                int& best_shift) const;
    bool tryScanContextGlobalChannel(const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_down,
                                     const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_down,
                                     Eigen::Isometry3d& best_pose, double& best_fitness, double& best_cost,
                                     int& candidate_count);

    bool detectKidnapping(double fitness_score);

    // 多假设初值生成
    std::vector<Eigen::Matrix4f> generateCandidateTransforms(const Eigen::Matrix4f& sac_transform);

    // 参数校验与归一化（避免非法配置导致异常行为）
    void validateAndNormalizeParams();
    rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(
        const std::vector<rclcpp::Parameter>& parameters);

    // QCS8550 线程亲和配置
    void configureThreadAffinityQcs8550();

    // 获取可用于 ROI 过滤的可靠 map->odom 快照
    bool tryGetReliableMapToOdom(Eigen::Isometry3d& map_to_odom);

    // I3: 亚克力幽灵点 ROI 过滤
    void applyAcrylicROIFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

    // I2: 重试区先验快速通道
    bool tryRetryZoneFastChannel(const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_down,
                                 const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_down,
                                 Eigen::Isometry3d& best_pose, double& best_fitness, double& best_cost,
                                 int& candidate_count);

    // 订阅者
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

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
    std::atomic<LocalizationState> localization_state_{LocalizationState::TRACKING};
    std::atomic<bool> shutdown_requested_{false};
    std::thread reloc_worker_thread_;
    std::mutex reloc_request_mutex_;
    std::condition_variable reloc_request_cv_;
    bool reloc_request_pending_{false};
    RelocTriggerReason reloc_pending_reason_{RelocTriggerReason::TIMEOUT};
    pcl::PointCloud<pcl::PointXYZ>::Ptr reloc_pending_cloud_;

    // [修复] 配准失败超时机制
    rclcpp::Time last_successful_registration_time_;
    double registration_timeout_sec_{10.0};  // 配准失败超时时间（秒）

    // I1: 冻结门控参数
    double freeze_update_err_{0.3};  // normalized_error 超过此值时冻结 TF 更新
    int min_inliers_{200};           // 内点数低于此值时冻结 TF 更新

    // I2: 重试区先验
    bool retry_zone_enable_{false};
    double retry_zone_x_{0.0};
    double retry_zone_y_{0.0};
    std::vector<double> retry_zone_yaw_candidates_deg_;
    double retry_zone_fast_accept_th_{0.15};
    double retry_zone_max_xy_offset_{1.5};
    double retry_zone_max_yaw_offset_deg_{60.0};
    bool competition_mode_{true};

    // I3: 亚克力过滤
    bool acrylic_filter_enable_{false};
    std::vector<double> acrylic_roi_boxes_;  // 平铺: [xmin,ymin,zmin,xmax,ymax,zmax, ...]
    double acrylic_filter_max_stale_sec_{1.0};

    // 全局重定位参数
    int global_icp_max_iterations_{100};
    double global_icp_max_correspondence_distance_{1.0};
    double global_fitness_threshold_{0.1};     // 全局重定位成功阈值
    double global_downsample_leaf_size_{0.5};  // 全局重定位体素滤波尺寸
    size_t max_accumulated_points_{100000};  // 累积点云上限，防止OOM

    // 多假设初值参数
    bool use_multi_hypothesis_{true};  // 是否使用多假设初值
    int num_yaw_hypotheses_{4};        // yaw方向假设数量

    // L2: Scan Context 参数
    bool enable_scan_context_{true};
    int sc_num_rings_{20};
    int sc_num_sectors_{60};
    double sc_max_radius_{8.0};
    double sc_submap_radius_{5.0};
    double sc_grid_resolution_{1.0};
    int sc_topk_{5};
    double sc_sim_threshold_{0.18};  // 1-cos 相似度代价上限
    int sc_min_points_per_submap_{80};
    bool sc_db_ready_{false};
    std::vector<ScanContextEntry> sc_database_;
    std::mutex sc_mutex_;

    // T2: QCS8550 线程亲和
    bool qcs8550_affinity_enable_{false};
    bool qcs8550_realtime_enable_{false};
    std::vector<int> prime_cpus_, gold_cpus_, silver_cpus_;
    int gicp_omp_threads_{4};

    // S1: IMU Spike 门控
    bool s1_enable_{false};
    std::string s1_imu_topic_{"/livox/imu"};
    double s1_accel_threshold_{20.0};
    double s1_gyro_threshold_{6.0};
    int s1_freeze_duration_ms_{300};
    std::atomic<bool> imu_spike_active_{false};
    rclcpp::Time imu_spike_deadline_;
    std::mutex imu_spike_mutex_;

    // S2: Hessian 退化轴拒绝
    bool s2_enable_{false};
    double s2_hessian_min_eigenvalue_{100.0};
    int s2_max_continuous_frames_{10};
    std::atomic<int> consecutive_s2_count_{0};

    // S3: SC 对称歧义拒绝
    bool s3_enable_{false};
    double s3_min_score_gap_{0.03};

    // T8: 丘陵工况坡道约束
    bool slope_roll_pitch_from_imu_{false};
    double slope_z_weight_{1.0};
    double slope_normal_consistency_deg_{25.0};
    std::string gicp_kernel_mode_{"scalar"};
    double imu_roll_{0.0};
    double imu_pitch_{0.0};
    bool imu_attitude_valid_{false};
    std::mutex imu_attitude_mutex_;

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
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

    // TF
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace rc26_localization
