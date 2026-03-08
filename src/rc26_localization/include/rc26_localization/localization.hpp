// Copyright 2025 RC2026
// 基于 small_gicp_relocalization 移植
// 原作者: Lihan Chen
//
// Licensed under the Apache License, Version 2.0
// Maintained by DongXuan Chen <2220362462@qq.com>

#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <Eigen/Dense>
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pcl/io/pcd_io.h"
#include "rc26_interfaces/msg/localization_backend_status.hpp"
#include "rc26_interfaces/msg/localization_health.hpp"
#include "rc26_interfaces/msg/route_observability.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "rc26_localization/constraint_validator.hpp"
#include "rc26_localization/esikf.hpp"
#include "rc26_localization/keyframe_manager.hpp"
#include "rc26_localization/map_to_odom_smoother.hpp"
#include "rc26_localization/online_scan_context_db.hpp"
#include "rc26_localization/pose_graph_backend.hpp"
#include "rc26_localization/route_observability_evaluator.hpp"
#include "rc26_localization/static_voxel_filter.hpp"
#include "small_gicp/ann/kdtree_omp.hpp"
#include "small_gicp/factors/gicp_factor.hpp"
#include "small_gicp/factors/robust_kernel.hpp"
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
        double t_l0_ms{0.0};
        double t_l1_ms{0.0};
        double t_l2_ms{0.0};
        int candidate_count{0};
        double best_fitness{std::numeric_limits<double>::max()};
        double best_j{std::numeric_limits<double>::max()};
        std::string winner_channel{"none"};
        std::string cancel_reason{"none"};
        bool accepted{false};
    };

    struct ScanContextEntry {
        Eigen::Vector2d center_xy{Eigen::Vector2d::Zero()};
        Eigen::MatrixXf descriptor;
        Eigen::VectorXf ring_key;
    };

    struct DegenAnalysis {
        Eigen::Vector3d degen_risk{Eigen::Vector3d::Zero()};
        Eigen::Matrix3d P_obs{Eigen::Matrix3d::Identity()};
        bool is_fully_degenerate{false};
    };

    struct ImuSample {
        Eigen::Vector3d accel{Eigen::Vector3d::Zero()};
        Eigen::Vector3d gyro{Eigen::Vector3d::Zero()};
        rclcpp::Time stamp;
    };

    struct ExternalCandidate {
        Eigen::Isometry3d pose_map{Eigen::Isometry3d::Identity()};
        rclcpp::Time stamp;
        std::string source{"unknown"};
    };

    // 回调函数
    void registeredPcdCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void uwbCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void controlDegradedCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void planCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void externalDynamicCandidatesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
    void externalVisualCandidatesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
    void externalLearnedCandidatesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
    void ingestExternalCandidates(const geometry_msgs::msg::PoseArray::SharedPtr& msg, const std::string& source);

    // 核心功能
    void loadGlobalMap(const std::string& file_name);
    void performRegistration();
    void publishTransform();
    bool prepareTargetMap();
    DegenAnalysis analyzeObservability(const pcl::PointCloud<pcl::PointCovariance>::Ptr& source) const;
    Eigen::Isometry3d constrainUpdate(const Eigen::Isometry3d& aligned_pose, const Eigen::Isometry3d& initial_guess,
                                      const DegenAnalysis& degen) const;
    Eigen::Matrix<double, 6, 6> computeObsCov(const small_gicp::RegistrationResult& result) const;
    void computeHessianStats(const Eigen::Matrix<double, 6, 6>& H,
                             double& min_eig, double& max_eig, double& cond) const;
    Eigen::Matrix<double, 6, 6> buildObsCovariance(const small_gicp::RegistrationResult& result) const;
    void publishPoseWithCov(const Eigen::Isometry3d& pose, const Eigen::Matrix<double, 6, 6>& cov) const;
    void publishDiagnostics(double normalized_error, size_t inliers, bool bad_quality,
                            const small_gicp::RegistrationResult& result) const;
    uint8_t computeHealthLevel(double sigma_xy, double sigma_yaw_deg, double h_min_eig, bool optimizer_ready,
                               double last_local_reg_age_sec, uint32_t candidate_conflict_count,
                               const std::string& fallback_reason, std::string& out_reason) const;
    void publishLocalizationHealth(const std::string& fallback_reason);
    void publishBackendStatus();
    void publishRouteObservability();
    void initializeGraphBackend();
    bool processGraphBackendOnLocalRegistration(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                                const Eigen::Isometry3d& map_to_odom,
                                                const rclcpp::Time& stamp);
    bool processGraphBackendAnchor(const Eigen::Isometry3d& map_to_odom, const rclcpp::Time& stamp);
    bool tryLookupOdomToBase(const rclcpp::Time& stamp, Eigen::Isometry3d& odom_to_base) const;
    std::vector<ExternalCandidate> consumeExternalCandidates(const rclcpp::Time& now, size_t max_count);
    pcl::PointCloud<pcl::PointXYZ>::Ptr buildMapPatchAround(const Eigen::Vector2d& center_map, double radius_m);
    void applyExternalAnchorCandidates(const KeyframeData& inserted, bool& graph_changed, const rclcpp::Time& stamp);

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
    bool tryGlobalChannel(const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_down,
                          const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_down, Eigen::Isometry3d& best_pose,
                          double& best_fitness, double& best_cost, int& candidate_count,
                          const std::atomic<bool>* stop_flag = nullptr);
    bool tryScanContextGlobalChannel(const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_down,
                                     const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_down,
                                     Eigen::Isometry3d& best_pose, double& best_fitness, double& best_cost,
                                     int& candidate_count, const std::atomic<bool>* stop_flag = nullptr);
    bool tryL0FastChannel(const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_down,
                          const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_down, Eigen::Isometry3d& best_pose,
                          double& best_fitness, double& best_cost, int& candidate_count,
                          const std::atomic<bool>* stop_flag = nullptr);

    bool detectKidnapping(double fitness_score, size_t num_inliers, const DegenAnalysis& degen);

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
                                 int& candidate_count, const std::atomic<bool>* stop_flag = nullptr);

    // 订阅者
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr uwb_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr control_degraded_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr dynamic_candidates_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr visual_candidates_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr learned_candidates_sub_;

    // 参数
    int num_threads_;
    int num_neighbors_;
    float global_leaf_size_;
    float registered_leaf_size_;
    float max_dist_sq_;
    bool robust_enable_{true};
    double huber_c_{1.0};
    bool cov_from_hessian_enable_{true};
    double cov_eig_floor_{1.0};
    bool cov_scale_enable_{true};
    double cov_scale_min_{1e-4};
    double cov_scale_max_{10.0};
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
    std::string pose_cov_topic_{"/localization/pose_with_cov"};
    std::string diagnostics_topic_{"/localization/diagnostics"};
    std::string health_topic_{"/localization/health"};
    std::string backend_status_topic_{"/localization/backend_status"};
    std::string route_observability_topic_{"/localization/route_observability"};
    std::string control_degraded_topic_{"/control_degraded"};
    std::string plan_topic_{"local_plan"};

    // 状态
    rclcpp::Time last_scan_time_;
    Eigen::Isometry3d result_t_;
    Eigen::Isometry3d previous_result_t_;
    bool map_loaded_{false};  // 地图是否成功加载
    bool target_ready_{false};
    bool map_needs_transform_{false};
    double tf_timeout_sec_{1.0};
    std::mutex cloud_mutex_;              // 保护 accumulated_cloud_ 的互斥锁
    mutable std::mutex result_mutex_;     // 保护 result_t_、previous_result_t_ 和协方差快照
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
    rclcpp::Time last_local_registration_time_;
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
    bool parallel_reloc_enable_{true};

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
    bool bevplace_enable_{false};
    std::string bevplace_model_path_;
    std::string bevplace_index_path_;
    std::string bevplace_infer_backend_{"onnxruntime"};
    double bevplace_bev_resolution_{0.2};
    int bevplace_bev_size_{128};
    int bevplace_topk_{5};
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

    // 退化分析（替代 S2 二值门控）
    bool degen_enable_{true};
    double degen_eigenvalue_ratio_threshold_{0.01};
    DegenAnalysis last_degen_;

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
    std::atomic<bool> imu_spike_recent_{false};
    rclcpp::Time imu_spike_deadline_;
    rclcpp::Time imu_spike_last_stamp_;
    rclcpp::Time last_imu_stamp_;
    bool last_imu_stamp_valid_{false};
    std::mutex imu_spike_mutex_;
    std::deque<ImuSample> imu_buffer_;
    std::mutex imu_buffer_mutex_;

    // S2: Hessian 退化轴拒绝
    bool s2_enable_{false};
    double s2_hessian_min_eigenvalue_{100.0};
    int s2_max_continuous_frames_{10};
    std::atomic<int> consecutive_s2_count_{0};
    bool hessian_degen_enable_{true};
    double hessian_lambda_hard_{10.0};

    // S3: SC 对称歧义拒绝
    bool s3_enable_{false};
    double s3_min_score_gap_{0.03};

    // ESIKF
    bool esikf_enable_{false};
    double esikf_accel_noise_{0.1};
    double esikf_gyro_noise_{0.01};
    double esikf_accel_bias_noise_{0.001};
    double esikf_gyro_bias_noise_{0.0001};
    mutable std::mutex esikf_mutex_;
    ESIKF esikf_;
    Eigen::Matrix<double, 6, 6> last_pose_cov_;

    // 动态物体过滤
    bool dynamic_filter_enable_{false};
    double dynamic_filter_voxel_size_{0.3};
    int dynamic_filter_window_size_{10};
    int dynamic_filter_stable_threshold_{5};
    StaticVoxelFilter static_voxel_filter_;

    // L0 快速通道
    bool l0_enable_{true};
    int l0_max_imu_gap_ms_{1000};

    // P0: LHI/后端状态参数与缓存
    bool enable_graph_backend_{false};
    bool legacy_hard_reloc_enable_{false};
    double graph_keyframe_translation_thresh_m_{0.4};
    double graph_keyframe_yaw_thresh_deg_{6.0};
    double graph_keyframe_time_thresh_sec_{1.0};
    bool graph_keyframe_trigger_on_degraded_rising_{true};
    bool graph_keyframe_trigger_on_hessian_drop_{true};
    bool graph_keyframe_trigger_on_sigma_cross_{true};
    int graph_loop_topk_{5};
    int graph_loop_min_keyframe_gap_{5};
    double graph_loop_similarity_min_{0.2};
    double graph_odom_sigma_translation_m_{0.05};
    double graph_odom_sigma_yaw_deg_{2.0};
    double graph_loop_sigma_translation_m_{0.08};
    double graph_loop_sigma_yaw_deg_{3.0};
    double graph_anchor_sigma_translation_m_{0.12};
    double graph_anchor_sigma_yaw_deg_{5.0};
    double graph_jump_detect_translation_m_{0.3};
    double graph_jump_detect_yaw_deg_{10.0};
    double graph_smoother_max_translation_speed_mps_{0.25};
    double graph_smoother_max_yaw_speed_degps_{10.0};
    double graph_validator_accept_fitness_threshold_{0.1};
    double graph_validator_conflict_fitness_threshold_{0.25};
    double graph_validator_max_corr_dist_m_{1.0};
    int graph_validator_max_iterations_{50};
    double lhi_green_sigma_xy_max_{0.12};
    double lhi_green_sigma_yaw_deg_max_{4.0};
    double lhi_green_h_min_eig_min_{50.0};
    double lhi_yellow_sigma_xy_min_{0.12};
    double lhi_yellow_sigma_yaw_deg_min_{4.0};
    double lhi_yellow_h_min_eig_max_{50.0};
    double lhi_orange_sigma_xy_min_{0.25};
    double lhi_orange_sigma_yaw_deg_min_{8.0};
    double lhi_orange_h_min_eig_max_{20.0};
    double lhi_red_sigma_xy_min_{1.0};
    double lhi_red_last_local_reg_age_sec_{2.0};
    int lhi_red_conflict_count_threshold_{3};
    std::string backend_status_optimizer_state_{"legacy_localization_only"};
    double backend_status_graph_health_placeholder_{0.0};
    double backend_status_loop_age_placeholder_sec_{-1.0};
    double backend_status_anchor_age_placeholder_sec_{-1.0};
    std::atomic<bool> control_degraded_{false};
    std::atomic<uint32_t> backend_candidate_conflict_count_{0};
    std::atomic<bool> backend_map_to_odom_jump_suppressed_{false};
    std::mutex plan_mutex_;
    nav_msgs::msg::Path latest_plan_;
    bool latest_plan_valid_{false};
    std::unique_ptr<RouteObservabilityEvaluator> route_observability_evaluator_;
    double route_window_min_m_{2.0};
    double route_window_max_m_{5.0};
    double route_sample_step_m_{0.5};
    double route_map_near_dist_m_{0.7};
    double route_risk_medium_threshold_{0.45};
    double route_risk_high_threshold_{0.7};
    double route_anchor_effective_dist_m_{2.0};
    double route_loop_recent_age_sec_{8.0};
    bool p4_candidate_enable_{false};
    std::string p4_dynamic_candidates_topic_{"/localization/p4/dynamic_candidates"};
    std::string p4_visual_candidates_topic_{"/localization/p4/visual_candidates"};
    std::string p4_learned_candidates_topic_{"/localization/p4/learned_candidates"};
    double p4_candidate_patch_radius_m_{8.0};
    double p4_candidate_max_stale_sec_{2.0};
    int p4_candidate_max_queue_size_{64};
    int p4_candidate_max_per_cycle_{2};
    double p4_candidate_sigma_translation_m_{0.12};
    double p4_candidate_sigma_yaw_deg_{5.0};
    mutable std::mutex health_mutex_;
    mutable std::mutex graph_mutex_;
    mutable std::mutex external_candidates_mutex_;
    bool graph_backend_initialized_{false};
    PoseGraphStatus graph_status_cache_;
    std::deque<ExternalCandidate> pending_external_candidates_;
    std::unique_ptr<KeyframeManager> keyframe_manager_;
    std::unique_ptr<OnlineScanContextDB> online_sc_db_;
    std::unique_ptr<ConstraintValidator> constraint_validator_;
    std::unique_ptr<PoseGraphBackend> pose_graph_backend_;
    std::unique_ptr<MapToOdomSmoother> map_to_odom_smoother_;
    double last_h_min_eig_{0.0};
    double last_h_cond_{1e12};

    // T8: 丘陵工况坡道约束
    bool slope_roll_pitch_from_imu_{false};
    double slope_z_weight_{1.0};
    double slope_normal_consistency_deg_{25.0};
    std::string gicp_kernel_mode_{"scalar"};
    double imu_roll_{0.0};
    double imu_pitch_{0.0};
    bool imu_attitude_valid_{false};
    std::mutex imu_attitude_mutex_;

    // UWB 绝对锚点
    bool uwb_enable_{false};
    std::string uwb_topic_{"/uwb/position"};
    double uwb_max_stale_sec_{2.0};
    double uwb_yaw_spread_deg_{30.0};
    Eigen::Vector2d uwb_position_{Eigen::Vector2d::Zero()};
    rclcpp::Time uwb_last_stamp_;
    bool uwb_available_{false};
    std::mutex uwb_mutex_;

    // 点云数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud_;
    pcl::PointCloud<pcl::PointCovariance>::Ptr target_;
    pcl::PointCloud<pcl::PointCovariance>::Ptr source_;

    // small_gicp 配准
    using RobustGICP = small_gicp::RobustFactor<small_gicp::Huber, small_gicp::GICPFactor>;
    using RegLM = small_gicp::Registration<RobustGICP, small_gicp::ParallelReductionOMP>;
    using RegGN = small_gicp::Registration<RobustGICP, small_gicp::ParallelReductionOMP,
                                           small_gicp::NullFactor, small_gicp::DistanceRejector,
                                           small_gicp::GaussNewtonOptimizer>;

    std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> target_tree_;
    std::shared_ptr<RegLM> register_lm_;
    std::shared_ptr<RegGN> register_gn_;
    std::string gicp_optimizer_mode_{"gn_auto"};
    double gn_auto_trans_threshold_m_{0.05};
    Eigen::Vector3d last_t_init_{Eigen::Vector3d::Zero()};

    // 定时器
    rclcpp::TimerBase::SharedPtr transform_timer_;
    rclcpp::TimerBase::SharedPtr register_timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_cov_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
    rclcpp::Publisher<rc26_interfaces::msg::LocalizationHealth>::SharedPtr health_pub_;
    rclcpp::Publisher<rc26_interfaces::msg::LocalizationBackendStatus>::SharedPtr backend_status_pub_;
    rclcpp::Publisher<rc26_interfaces::msg::RouteObservability>::SharedPtr route_observability_pub_;

    // TF
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace rc26_localization
