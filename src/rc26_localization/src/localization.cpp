// Copyright 2025 RC2026
// 基于 small_gicp_relocalization 移植
// 原作者: Lihan Chen
//
// Licensed under the Apache License, Version 2.0

#include "rc26_localization/localization.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <future>
#include <limits>
#include <pthread.h>
#include <sched.h>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <unordered_set>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "pcl/common/transforms.h"
#include "pcl/filters/filter.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/registration/gicp.h"
#include "pcl/search/kdtree.h"
#include "pcl_conversions/pcl_conversions.h"
#include "small_gicp/pcl/pcl_registration.hpp"
#include "small_gicp/util/downsampling_omp.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

#include "localization_internal.hpp"

namespace rc26_localization {

namespace {
struct ThreadBindResult {
    bool affinity_ok{false};
    bool sched_ok{false};
    bool sched_downgraded{false};
    int affinity_err{0};
    int sched_err{0};
};

static std::string formatCpuList(const std::vector<int>& cpus) {
    std::ostringstream oss;
    for (size_t i = 0; i < cpus.size(); ++i) {
        if (i > 0) {
            oss << ",";
        }
        oss << cpus[i];
    }
    return oss.str();
}

// 绑定当前线程到指定 CPU 集合，失败仅告警不终止
static ThreadBindResult bindCurrentThread(const std::vector<int>& cpus, int policy, int priority) {
    ThreadBindResult result;
    if (cpus.empty()) {
        result.affinity_err = EINVAL;
        return result;
    }
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    for (int cpu : cpus) {
        if (cpu >= 0) CPU_SET(cpu, &cpuset);
    }
    const int affinity_err = pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
    if (affinity_err != 0) {
        result.affinity_err = affinity_err;
        return result;
    }
    result.affinity_ok = true;
    result.sched_ok = true;

    if (policy == SCHED_FIFO || policy == SCHED_RR) {
        struct sched_param param{};
        param.sched_priority = priority;
        const int sched_err = pthread_setschedparam(pthread_self(), policy, &param);
        if (sched_err != 0) {
            result.sched_err = sched_err;
            result.sched_downgraded = true;
            result.sched_ok = false;
            param.sched_priority = 0;
            if (pthread_setschedparam(pthread_self(), SCHED_OTHER, &param) == 0) {
                result.sched_ok = true;
            }
        }
    }

    return result;
}

// YAML 显式配置优先；未配置时读 /sys/.../cpuinfo_max_freq 自动分桶
static void detectQcs8550Buckets(
    std::vector<int>& prime, std::vector<int>& gold, std::vector<int>& silver) {
    struct CpuFreq { int cpu; long freq; };
    std::vector<CpuFreq> freqs;
    for (int i = 0; i < 8; ++i) {
        std::ifstream ifs("/sys/devices/system/cpu/cpu" + std::to_string(i) +
                          "/cpufreq/cpuinfo_max_freq");
        long freq = 0;
        if (ifs >> freq) freqs.push_back({i, freq});
    }
    if (freqs.empty()) {
        prime = {0}; gold = {1, 2, 3, 4}; silver = {5, 6, 7};
        return;
    }
    std::sort(freqs.begin(), freqs.end(),
              [](const CpuFreq& a, const CpuFreq& b) { return a.freq > b.freq; });
    const long max_f = freqs.front().freq, min_f = freqs.back().freq, r = max_f - min_f;
    for (const auto& cf : freqs) {
        if (r <= 0 || cf.freq >= max_f - r / 4) prime.push_back(cf.cpu);
        else if (cf.freq >= min_f + r / 3) gold.push_back(cf.cpu);
        else silver.push_back(cf.cpu);
    }
    if (prime.size() > 1) {
        gold.insert(gold.begin(), prime.begin() + 1, prime.end());
        prime.resize(1);
    }
}
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
    this->declare_parameter("robust_enable", true);
    this->declare_parameter("huber_c", 1.0);
    this->declare_parameter("cov_from_hessian_enable", true);
    this->declare_parameter("cov_eig_floor", 1.0);
    this->declare_parameter("cov_scale_enable", true);
    this->declare_parameter("cov_scale_min", 1e-4);
    this->declare_parameter("cov_scale_max", 10.0);
    this->declare_parameter("gicp_optimizer_mode", std::string("gn_auto"));
    this->declare_parameter("gn_auto_trans_threshold_m", 0.05);
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
    this->declare_parameter("pose_cov_topic", std::string("/localization/pose_with_cov"));
    this->declare_parameter("diagnostics_topic", std::string("/localization/diagnostics"));

    // 绑架检测参数
    this->declare_parameter("kidnap_threshold_count", 5);
    this->declare_parameter("kidnap_fitness_threshold", 0.5);

    // 全局重定位参数
    this->declare_parameter("global_icp_max_iterations", 100);
    this->declare_parameter("global_icp_max_correspondence_distance", 1.0);
    this->declare_parameter("global_fitness_threshold", 0.1);
    this->declare_parameter("min_points_for_relocalization", 50);
    this->declare_parameter("global_downsample_leaf_size", 0.5);

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
    this->declare_parameter("parallel_reloc_enable", true);

    // I3: 亚克力过滤参数
    this->declare_parameter("acrylic_filter_enable", false);
    this->declare_parameter("acrylic_roi_boxes", std::vector<double>{});
    this->declare_parameter("acrylic_filter_max_stale_sec", 1.0);

    // L2: Scan Context 参数
    this->declare_parameter("bevplace_enable", false);
    this->declare_parameter("bevplace_model_path", std::string(""));
    this->declare_parameter("bevplace_index_path", std::string(""));
    this->declare_parameter("bevplace_infer_backend", std::string("onnxruntime"));
    this->declare_parameter("bevplace_bev_resolution", 0.2);
    this->declare_parameter("bevplace_bev_size", 128);
    this->declare_parameter("bevplace_topk", 5);
    this->declare_parameter("enable_scan_context", true);
    this->declare_parameter("sc_num_rings", 20);
    this->declare_parameter("sc_num_sectors", 60);
    this->declare_parameter("sc_max_radius", 8.0);
    this->declare_parameter("sc_submap_radius", 5.0);
    this->declare_parameter("sc_grid_resolution", 1.0);
    this->declare_parameter("sc_topk", 5);
    this->declare_parameter("sc_sim_threshold", 0.18);
    this->declare_parameter("sc_min_points_per_submap", 80);

    // 退化子空间约束
    this->declare_parameter("degen_enable", true);
    this->declare_parameter("degen_eigenvalue_ratio_threshold", 0.01);

    // T2: QCS8550 线程亲和
    this->declare_parameter("qcs8550_affinity_enable", false);
    this->declare_parameter("qcs8550_realtime_enable", false);
    this->declare_parameter("qcs8550_prime_cpu_ids", std::vector<int64_t>{});
    this->declare_parameter("qcs8550_gold_cpu_ids", std::vector<int64_t>{});
    this->declare_parameter("qcs8550_silver_cpu_ids", std::vector<int64_t>{});
    this->declare_parameter("gicp_omp_threads", 4);

    // S1: IMU Spike 门控
    this->declare_parameter("s1_enable", false);
    this->declare_parameter("s1_imu_topic", std::string("/livox/imu"));
    this->declare_parameter("s1_accel_threshold", 20.0);
    this->declare_parameter("s1_gyro_threshold", 6.0);
    this->declare_parameter("s1_freeze_duration_ms", 300);

    // S2: Hessian 退化轴拒绝
    this->declare_parameter("s2_enable", false);
    this->declare_parameter("s2_hessian_min_eigenvalue", 100.0);
    this->declare_parameter("s2_max_continuous_frames", 10);
    this->declare_parameter("hessian_degen_enable", true);
    this->declare_parameter("hessian_lambda_hard", 10.0);

    // S3: SC 对称歧义拒绝
    this->declare_parameter("s3_enable", false);
    this->declare_parameter("s3_min_score_gap", 0.03);

    // ESIKF
    this->declare_parameter("esikf_enable", false);
    this->declare_parameter("esikf_accel_noise", 0.1);
    this->declare_parameter("esikf_gyro_noise", 0.01);
    this->declare_parameter("esikf_accel_bias_noise", 0.001);
    this->declare_parameter("esikf_gyro_bias_noise", 0.0001);

    // 动态物体过滤
    this->declare_parameter("dynamic_filter_enable", false);
    this->declare_parameter("dynamic_filter_voxel_size", 0.3);
    this->declare_parameter("dynamic_filter_window_size", 10);
    this->declare_parameter("dynamic_filter_stable_threshold", 5);

    // L0 通道
    this->declare_parameter("l0_enable", true);
    this->declare_parameter("l0_max_imu_gap_ms", 1000);

    // UWB 绝对锚点
    this->declare_parameter("uwb_enable", false);
    this->declare_parameter("uwb_topic", std::string("/uwb/position"));
    this->declare_parameter("uwb_max_stale_sec", 2.0);
    this->declare_parameter("uwb_yaw_spread_deg", 30.0);

    // T8: 丘陵工况坡道约束
    this->declare_parameter("slope_roll_pitch_from_imu", false);
    this->declare_parameter("slope_z_weight", 1.0);
    this->declare_parameter("slope_normal_consistency_deg", 25.0);

    // NEON 内核模式（占位，待 A/B 验证）
    this->declare_parameter("gicp_kernel_mode", std::string("scalar"));

    // 获取参数
    this->get_parameter("num_threads", num_threads_);
    this->get_parameter("num_neighbors", num_neighbors_);
    this->get_parameter("global_leaf_size", global_leaf_size_);
    this->get_parameter("registered_leaf_size", registered_leaf_size_);
    this->get_parameter("max_dist_sq", max_dist_sq_);
    this->get_parameter("robust_enable", robust_enable_);
    this->get_parameter("huber_c", huber_c_);
    this->get_parameter("cov_from_hessian_enable", cov_from_hessian_enable_);
    this->get_parameter("cov_eig_floor", cov_eig_floor_);
    this->get_parameter("cov_scale_enable", cov_scale_enable_);
    this->get_parameter("cov_scale_min", cov_scale_min_);
    this->get_parameter("cov_scale_max", cov_scale_max_);
    this->get_parameter("gicp_optimizer_mode", gicp_optimizer_mode_);
    this->get_parameter("gn_auto_trans_threshold_m", gn_auto_trans_threshold_m_);
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
    this->get_parameter("pose_cov_topic", pose_cov_topic_);
    this->get_parameter("diagnostics_topic", diagnostics_topic_);

    this->get_parameter("kidnap_threshold_count", kidnap_threshold_count_);
    this->get_parameter("kidnap_fitness_threshold", kidnap_fitness_threshold_);

    this->get_parameter("global_icp_max_iterations", global_icp_max_iterations_);
    this->get_parameter("global_icp_max_correspondence_distance", global_icp_max_correspondence_distance_);
    this->get_parameter("global_fitness_threshold", global_fitness_threshold_);
    this->get_parameter("min_points_for_relocalization", min_points_for_relocalization_);
    this->get_parameter("global_downsample_leaf_size", global_downsample_leaf_size_);

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
    this->get_parameter("parallel_reloc_enable", parallel_reloc_enable_);

    this->get_parameter("acrylic_filter_enable", acrylic_filter_enable_);
    this->get_parameter("acrylic_roi_boxes", acrylic_roi_boxes_);
    this->get_parameter("acrylic_filter_max_stale_sec", acrylic_filter_max_stale_sec_);

    this->get_parameter("bevplace_enable", bevplace_enable_);
    this->get_parameter("bevplace_model_path", bevplace_model_path_);
    this->get_parameter("bevplace_index_path", bevplace_index_path_);
    this->get_parameter("bevplace_infer_backend", bevplace_infer_backend_);
    this->get_parameter("bevplace_bev_resolution", bevplace_bev_resolution_);
    this->get_parameter("bevplace_bev_size", bevplace_bev_size_);
    this->get_parameter("bevplace_topk", bevplace_topk_);
    this->get_parameter("enable_scan_context", enable_scan_context_);
    this->get_parameter("sc_num_rings", sc_num_rings_);
    this->get_parameter("sc_num_sectors", sc_num_sectors_);
    this->get_parameter("sc_max_radius", sc_max_radius_);
    this->get_parameter("sc_submap_radius", sc_submap_radius_);
    this->get_parameter("sc_grid_resolution", sc_grid_resolution_);
    this->get_parameter("sc_topk", sc_topk_);
    this->get_parameter("sc_sim_threshold", sc_sim_threshold_);
    this->get_parameter("sc_min_points_per_submap", sc_min_points_per_submap_);
    this->get_parameter("degen_enable", degen_enable_);
    this->get_parameter("degen_eigenvalue_ratio_threshold", degen_eigenvalue_ratio_threshold_);

    // T2: QCS8550 线程亲和
    this->get_parameter("qcs8550_affinity_enable", qcs8550_affinity_enable_);
    this->get_parameter("qcs8550_realtime_enable", qcs8550_realtime_enable_);
    this->get_parameter("gicp_omp_threads", gicp_omp_threads_);
    {
        std::vector<int64_t> p, g, s;
        this->get_parameter("qcs8550_prime_cpu_ids", p);
        this->get_parameter("qcs8550_gold_cpu_ids", g);
        this->get_parameter("qcs8550_silver_cpu_ids", s);
        const bool yaml_cpu_override = !p.empty() || !g.empty() || !s.empty();
        if (yaml_cpu_override) {
            prime_cpus_.assign(p.begin(), p.end());
            gold_cpus_.assign(g.begin(), g.end());
            silver_cpus_.assign(s.begin(), s.end());
        } else {
            detectQcs8550Buckets(prime_cpus_, gold_cpus_, silver_cpus_);
        }

        if (gold_cpus_.empty() && !prime_cpus_.empty()) {
            gold_cpus_ = prime_cpus_;
        }
        if (prime_cpus_.empty() && !gold_cpus_.empty()) {
            prime_cpus_.push_back(gold_cpus_.front());
        }
        if (silver_cpus_.empty()) {
            auto exists = [this](int cpu) {
                return std::find(prime_cpus_.begin(), prime_cpus_.end(), cpu) != prime_cpus_.end() ||
                       std::find(gold_cpus_.begin(), gold_cpus_.end(), cpu) != gold_cpus_.end();
            };
            for (int cpu = 0; cpu < 8; ++cpu) {
                if (!exists(cpu)) {
                    silver_cpus_.push_back(cpu);
                }
            }
        }
    }

    // S1
    this->get_parameter("s1_enable", s1_enable_);
    this->get_parameter("s1_imu_topic", s1_imu_topic_);
    this->get_parameter("s1_accel_threshold", s1_accel_threshold_);
    this->get_parameter("s1_gyro_threshold", s1_gyro_threshold_);
    this->get_parameter("s1_freeze_duration_ms", s1_freeze_duration_ms_);

    // S2
    this->get_parameter("s2_enable", s2_enable_);
    this->get_parameter("s2_hessian_min_eigenvalue", s2_hessian_min_eigenvalue_);
    this->get_parameter("s2_max_continuous_frames", s2_max_continuous_frames_);
    this->get_parameter("hessian_degen_enable", hessian_degen_enable_);
    this->get_parameter("hessian_lambda_hard", hessian_lambda_hard_);

    // S3
    this->get_parameter("s3_enable", s3_enable_);
    this->get_parameter("s3_min_score_gap", s3_min_score_gap_);

    // ESIKF
    this->get_parameter("esikf_enable", esikf_enable_);
    this->get_parameter("esikf_accel_noise", esikf_accel_noise_);
    this->get_parameter("esikf_gyro_noise", esikf_gyro_noise_);
    this->get_parameter("esikf_accel_bias_noise", esikf_accel_bias_noise_);
    this->get_parameter("esikf_gyro_bias_noise", esikf_gyro_bias_noise_);

    // 动态物体过滤
    this->get_parameter("dynamic_filter_enable", dynamic_filter_enable_);
    this->get_parameter("dynamic_filter_voxel_size", dynamic_filter_voxel_size_);
    this->get_parameter("dynamic_filter_window_size", dynamic_filter_window_size_);
    this->get_parameter("dynamic_filter_stable_threshold", dynamic_filter_stable_threshold_);

    // L0
    this->get_parameter("l0_enable", l0_enable_);
    this->get_parameter("l0_max_imu_gap_ms", l0_max_imu_gap_ms_);

    // UWB
    this->get_parameter("uwb_enable", uwb_enable_);
    this->get_parameter("uwb_topic", uwb_topic_);
    this->get_parameter("uwb_max_stale_sec", uwb_max_stale_sec_);
    this->get_parameter("uwb_yaw_spread_deg", uwb_yaw_spread_deg_);

    // T8
    this->get_parameter("slope_roll_pitch_from_imu", slope_roll_pitch_from_imu_);
    this->get_parameter("slope_z_weight", slope_z_weight_);
    this->get_parameter("slope_normal_consistency_deg", slope_normal_consistency_deg_);
    this->get_parameter("gicp_kernel_mode", gicp_kernel_mode_);

    std::transform(gicp_kernel_mode_.begin(), gicp_kernel_mode_.end(), gicp_kernel_mode_.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (gicp_kernel_mode_ != "scalar" && gicp_kernel_mode_ != "neon") {
        RCLCPP_WARN(this->get_logger(),
                    "gicp_kernel_mode=%s 非法，已回退为 scalar", gicp_kernel_mode_.c_str());
        gicp_kernel_mode_ = "scalar";
    }
    if (gicp_kernel_mode_ != "scalar") {
        RCLCPP_WARN(this->get_logger(),
                    "gicp_kernel_mode=%s 目前尚未在主链启用，运行时仍使用 scalar 路径",
                    gicp_kernel_mode_.c_str());
    }

    std::transform(gicp_optimizer_mode_.begin(), gicp_optimizer_mode_.end(), gicp_optimizer_mode_.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (gicp_optimizer_mode_ != "gn_auto" && gicp_optimizer_mode_ != "gn" && gicp_optimizer_mode_ != "lm") {
        RCLCPP_WARN(this->get_logger(),
                    "gicp_optimizer_mode=%s 非法，已回退为 gn_auto", gicp_optimizer_mode_.c_str());
        gicp_optimizer_mode_ = "gn_auto";
    }

    StaticVoxelFilter::Config dynamic_filter_cfg;
    dynamic_filter_cfg.voxel_size = dynamic_filter_voxel_size_;
    dynamic_filter_cfg.window_size = dynamic_filter_window_size_;
    dynamic_filter_cfg.stable_threshold = dynamic_filter_stable_threshold_;
    static_voxel_filter_.setConfig(dynamic_filter_cfg);

    esikf_.setProcessNoise(esikf_accel_noise_, esikf_gyro_noise_, esikf_accel_bias_noise_, esikf_gyro_bias_noise_);

    validateAndNormalizeParams();
    configureThreadAffinityQcs8550();
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
    {
        std::lock_guard<std::mutex> lk(esikf_mutex_);
        esikf_.reset(result_t_);
    }

    // 启动阶段沿用旧观测协方差量级，避免行为突变
    last_pose_cov_.setZero();
    last_pose_cov_.diagonal() << 1e6, 1e6, 1e6, 1e-2, 1e-2, 1e-2;

    // 初始化配准成功时间（线程安全）
    {
        std::lock_guard<std::mutex> lock(registration_time_mutex_);
        last_successful_registration_time_ = this->now();
    }

    // 初始化点云
    accumulated_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    global_map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    reloc_pending_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    register_lm_ = std::make_shared<RegLM>();
    register_gn_ = std::make_shared<RegGN>();
    register_lm_->point_factor.robust_kernel.c = robust_enable_ ? huber_c_ : 1e9;
    register_gn_->point_factor.robust_kernel.c = robust_enable_ ? huber_c_ : 1e9;

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

    pose_cov_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(pose_cov_topic_, 10);
    diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(diagnostics_topic_, 10);

    // S1/T8: IMU 订阅
    if (s1_enable_ || slope_roll_pitch_from_imu_) {
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            s1_imu_topic_, 10, std::bind(&LocalizationNode::imuCallback, this, std::placeholders::_1));
    }

    if (uwb_enable_) {
        uwb_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            uwb_topic_, 10, std::bind(&LocalizationNode::uwbCallback, this, std::placeholders::_1));
    }

    dyn_params_handler_ = this->add_on_set_parameters_callback(
        std::bind(&LocalizationNode::dynamicParametersCallback, this, std::placeholders::_1));

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
    dyn_params_handler_.reset();
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

void LocalizationNode::relocWorkerLoop() {
    bool worker_affinity_applied = false;
    while (!shutdown_requested_.load()) {
        // T2: 绑定重定位工作线程到 Gold 核（仅初始化一次）
        if (!worker_affinity_applied && qcs8550_affinity_enable_) {
            const int policy = qcs8550_realtime_enable_ ? SCHED_FIFO : SCHED_OTHER;
            const int prio   = qcs8550_realtime_enable_ ? 60 : 0;
            const ThreadBindResult bind = bindCurrentThread(gold_cpus_, policy, prio);
            if (!bind.affinity_ok) {
                RCLCPP_WARN(get_logger(), "QCS8550 重定位线程绑核失败(errno=%d:%s)，继续运行",
                            bind.affinity_err, std::strerror(bind.affinity_err));
            }
            if (qcs8550_realtime_enable_ && bind.sched_downgraded) {
                RCLCPP_WARN(get_logger(),
                            "QCS8550 重定位线程实时调度失败(errno=%d:%s)，已降级到 SCHED_OTHER",
                            bind.sched_err, std::strerror(bind.sched_err));
            }
            worker_affinity_applied = true;
        }

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
    if (huber_c_ <= 0.0) {
        RCLCPP_WARN(this->get_logger(), "huber_c=%.6f 非法，已回退为 1.0", huber_c_);
        huber_c_ = 1.0;
    }
    if (cov_eig_floor_ <= 0.0) {
        RCLCPP_WARN(this->get_logger(), "cov_eig_floor=%.6f 非法，已回退为 1.0", cov_eig_floor_);
        cov_eig_floor_ = 1.0;
    }
    cov_scale_min_ = std::max(cov_scale_min_, 1e-8);
    cov_scale_max_ = std::max(cov_scale_max_, cov_scale_min_);
    if (gn_auto_trans_threshold_m_ < 0.0) {
        RCLCPP_WARN(this->get_logger(), "gn_auto_trans_threshold_m=%.6f 非法，已回退为 0.05",
                    gn_auto_trans_threshold_m_);
        gn_auto_trans_threshold_m_ = 0.05;
    }
    if (hessian_lambda_hard_ <= 0.0) {
        RCLCPP_WARN(this->get_logger(), "hessian_lambda_hard=%.6f 非法，已回退为 10.0", hessian_lambda_hard_);
        hessian_lambda_hard_ = 10.0;
    }
    std::transform(gicp_optimizer_mode_.begin(), gicp_optimizer_mode_.end(), gicp_optimizer_mode_.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (gicp_optimizer_mode_ != "gn_auto" && gicp_optimizer_mode_ != "gn" && gicp_optimizer_mode_ != "lm") {
        RCLCPP_WARN(this->get_logger(), "gicp_optimizer_mode=%s 非法，已回退为 gn_auto",
                    gicp_optimizer_mode_.c_str());
        gicp_optimizer_mode_ = "gn_auto";
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

    sc_num_rings_ = std::max(sc_num_rings_, 4);
    sc_num_sectors_ = std::max(sc_num_sectors_, 12);
    sc_max_radius_ = std::max(sc_max_radius_, 1.0);
    sc_submap_radius_ = std::max(sc_submap_radius_, 1.0);
    sc_grid_resolution_ = std::max(sc_grid_resolution_, 0.2);
    sc_topk_ = std::max(sc_topk_, 1);
    sc_sim_threshold_ = std::clamp(sc_sim_threshold_, 0.01, 1.0);
    sc_min_points_per_submap_ = std::max(sc_min_points_per_submap_, 20);
    bevplace_bev_resolution_ = std::max(bevplace_bev_resolution_, 0.05);
    bevplace_bev_size_ = std::max(bevplace_bev_size_, 32);
    bevplace_topk_ = std::max(bevplace_topk_, 1);
    if (bevplace_enable_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "bevplace_enable=true，但当前版本未集成 rc26_bevplace，自动回退 Scan Context");
    }
    degen_eigenvalue_ratio_threshold_ = std::clamp(degen_eigenvalue_ratio_threshold_, 1e-4, 0.5);
    if (qcs8550_affinity_enable_ && (prime_cpus_.empty() || gold_cpus_.empty())) {
        RCLCPP_WARN(this->get_logger(),
                    "QCS8550 CPU 分桶配置不完整，已尝试自动检测 prime/gold/silver");
        detectQcs8550Buckets(prime_cpus_, gold_cpus_, silver_cpus_);
    }
    if (gicp_omp_threads_ <= 0) {
        RCLCPP_WARN(this->get_logger(), "gicp_omp_threads=%d 非法，已回退为 4", gicp_omp_threads_);
        gicp_omp_threads_ = 4;
    }
    if (qcs8550_affinity_enable_ && num_threads_ != gicp_omp_threads_) {
        RCLCPP_WARN(this->get_logger(),
                    "num_threads(%d) 与 gicp_omp_threads(%d) 不一致，已统一为 %d",
                    num_threads_, gicp_omp_threads_, gicp_omp_threads_);
        num_threads_ = gicp_omp_threads_;
    }
    if (slope_z_weight_ < 1.0) {
        RCLCPP_WARN(this->get_logger(), "slope_z_weight=%.3f 非法，已钳制为 1.0", slope_z_weight_);
        slope_z_weight_ = 1.0;
    }
    if (dynamic_filter_voxel_size_ <= 0.0) {
        RCLCPP_WARN(this->get_logger(), "dynamic_filter_voxel_size=%.3f 非法，已回退为 0.3", dynamic_filter_voxel_size_);
        dynamic_filter_voxel_size_ = 0.3;
    }
    if (dynamic_filter_window_size_ <= 0) {
        RCLCPP_WARN(this->get_logger(), "dynamic_filter_window_size=%d 非法，已回退为 10", dynamic_filter_window_size_);
        dynamic_filter_window_size_ = 10;
    }
    if (dynamic_filter_stable_threshold_ <= 0 || dynamic_filter_stable_threshold_ > dynamic_filter_window_size_) {
        RCLCPP_WARN(this->get_logger(),
                    "dynamic_filter_stable_threshold=%d 非法，已回退为 window_size 的一半",
                    dynamic_filter_stable_threshold_);
        dynamic_filter_stable_threshold_ = std::max(1, dynamic_filter_window_size_ / 2);
    }
    if (l0_max_imu_gap_ms_ <= 0) {
        RCLCPP_WARN(this->get_logger(), "l0_max_imu_gap_ms=%d 非法，已回退为 1000", l0_max_imu_gap_ms_);
        l0_max_imu_gap_ms_ = 1000;
    }
    if (uwb_max_stale_sec_ <= 0.0) {
        RCLCPP_WARN(this->get_logger(), "uwb_max_stale_sec=%.3f 非法，已回退为 2.0", uwb_max_stale_sec_);
        uwb_max_stale_sec_ = 2.0;
    }
    uwb_yaw_spread_deg_ = std::clamp(uwb_yaw_spread_deg_, 1.0, 180.0);
    esikf_accel_noise_ = std::max(esikf_accel_noise_, 1e-4);
    esikf_gyro_noise_ = std::max(esikf_gyro_noise_, 1e-5);
    esikf_accel_bias_noise_ = std::max(esikf_accel_bias_noise_, 1e-6);
    esikf_gyro_bias_noise_ = std::max(esikf_gyro_bias_noise_, 1e-6);

    StaticVoxelFilter::Config dynamic_filter_cfg;
    dynamic_filter_cfg.voxel_size = dynamic_filter_voxel_size_;
    dynamic_filter_cfg.window_size = dynamic_filter_window_size_;
    dynamic_filter_cfg.stable_threshold = dynamic_filter_stable_threshold_;
    static_voxel_filter_.setConfig(dynamic_filter_cfg);
    esikf_.setProcessNoise(esikf_accel_noise_, esikf_gyro_noise_, esikf_accel_bias_noise_, esikf_gyro_bias_noise_);

    if (competition_mode_) {
        if (!retry_zone_enable_) {
            throw std::runtime_error("competition_mode=true 但 retry_zone_enable=false，拒绝启动");
        }
        if (std::abs(retry_zone_x_) <= kNearZero && std::abs(retry_zone_y_) <= kNearZero) {
            throw std::runtime_error("competition_mode=true 但 retry_zone_x/y 仍为默认占位值(0,0)，拒绝启动");
        }
    }
}

void LocalizationNode::configureThreadAffinityQcs8550() {
    if (!qcs8550_affinity_enable_) {
        return;
    }
    const std::string omp_threads = std::to_string(gicp_omp_threads_);
    setenv("OMP_NUM_THREADS", omp_threads.c_str(), 1);
    setenv("OMP_PROC_BIND", "true", 1);
    setenv("OMP_DYNAMIC", "false", 1);

    auto combined = gold_cpus_;
    combined.insert(combined.end(), prime_cpus_.begin(), prime_cpus_.end());
    const int policy = qcs8550_realtime_enable_ ? SCHED_FIFO : SCHED_OTHER;
    const int priority = qcs8550_realtime_enable_ ? 60 : 0;
    const ThreadBindResult bind = bindCurrentThread(combined, policy, priority);

    if (!bind.affinity_ok) {
        RCLCPP_WARN(get_logger(), "QCS8550 主线程绑核失败(errno=%d:%s)，继续运行",
                    bind.affinity_err, std::strerror(bind.affinity_err));
    }
    if (qcs8550_realtime_enable_ && bind.sched_downgraded) {
        RCLCPP_WARN(get_logger(),
                    "QCS8550 主线程实时调度失败(errno=%d:%s)，已降级到 SCHED_OTHER",
                    bind.sched_err, std::strerror(bind.sched_err));
    }

    RCLCPP_INFO(get_logger(),
                "QCS8550 affinity: prime=[%s] gold=[%s] silver=[%s] omp_threads=%d policy=%s",
                formatCpuList(prime_cpus_).c_str(), formatCpuList(gold_cpus_).c_str(),
                formatCpuList(silver_cpus_).c_str(), gicp_omp_threads_,
                qcs8550_realtime_enable_ ? "SCHED_FIFO(60)->fallback" : "SCHED_OTHER");
}

}  // namespace rc26_localization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rc26_localization::LocalizationNode)

