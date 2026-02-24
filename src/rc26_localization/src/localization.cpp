// Copyright 2025 RC2026
// 基于 small_gicp_relocalization 移植
// 原作者: Lihan Chen
//
// Licensed under the Apache License, Version 2.0

#include "rc26_localization/localization.hpp"

#include <algorithm>
#include <cerrno>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <limits>
#include <pthread.h>
#include <sched.h>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

#include "pcl/common/transforms.h"
#include "pcl/filters/filter.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/registration/gicp.h"
#include "pcl/search/kdtree.h"
#include "pcl_conversions/pcl_conversions.h"
#include "small_gicp/pcl/pcl_registration.hpp"
#include "small_gicp/util/downsampling_omp.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

namespace rc26_localization {

namespace {
constexpr double kNearZero = 1e-6;
constexpr double kCostWf = 0.5;
constexpr double kCostWxy = 0.3;
constexpr double kCostWyaw = 0.2;
constexpr double kMaxSlopeRollPitchCorrectionDeg = 5.0;

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

    // I3: 亚克力过滤参数
    this->declare_parameter("acrylic_filter_enable", false);
    this->declare_parameter("acrylic_roi_boxes", std::vector<double>{});
    this->declare_parameter("acrylic_filter_max_stale_sec", 1.0);

    // L2: Scan Context 参数
    this->declare_parameter("enable_scan_context", true);
    this->declare_parameter("sc_num_rings", 20);
    this->declare_parameter("sc_num_sectors", 60);
    this->declare_parameter("sc_max_radius", 8.0);
    this->declare_parameter("sc_submap_radius", 5.0);
    this->declare_parameter("sc_grid_resolution", 1.0);
    this->declare_parameter("sc_topk", 5);
    this->declare_parameter("sc_sim_threshold", 0.18);
    this->declare_parameter("sc_min_points_per_submap", 80);

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

    // S3: SC 对称歧义拒绝
    this->declare_parameter("s3_enable", false);
    this->declare_parameter("s3_min_score_gap", 0.03);

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

    this->get_parameter("acrylic_filter_enable", acrylic_filter_enable_);
    this->get_parameter("acrylic_roi_boxes", acrylic_roi_boxes_);
    this->get_parameter("acrylic_filter_max_stale_sec", acrylic_filter_max_stale_sec_);

    this->get_parameter("enable_scan_context", enable_scan_context_);
    this->get_parameter("sc_num_rings", sc_num_rings_);
    this->get_parameter("sc_num_sectors", sc_num_sectors_);
    this->get_parameter("sc_max_radius", sc_max_radius_);
    this->get_parameter("sc_submap_radius", sc_submap_radius_);
    this->get_parameter("sc_grid_resolution", sc_grid_resolution_);
    this->get_parameter("sc_topk", sc_topk_);
    this->get_parameter("sc_sim_threshold", sc_sim_threshold_);
    this->get_parameter("sc_min_points_per_submap", sc_min_points_per_submap_);

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

    // S3
    this->get_parameter("s3_enable", s3_enable_);
    this->get_parameter("s3_min_score_gap", s3_min_score_gap_);

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

    // S1/T8: IMU 订阅
    if (s1_enable_ || slope_roll_pitch_from_imu_) {
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            s1_imu_topic_, 10, std::bind(&LocalizationNode::imuCallback, this, std::placeholders::_1));
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

rcl_interfaces::msg::SetParametersResult LocalizationNode::dynamicParametersCallback(
    const std::vector<rclcpp::Parameter>& parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "ok";

    bool need_target_rebuild = false;
    bool need_sc_rebuild = false;

    auto reject = [&](const std::string& reason) {
        result.successful = false;
        result.reason = reason;
    };

    auto log_update = [&](const std::string& param, double old_value, double new_value) {
        RCLCPP_INFO(this->get_logger(), "PARAM_UPDATE,node=localization,param=%s,old=%.6f,new=%.6f",
                    param.c_str(), old_value, new_value);
    };

    auto log_update_int = [&](const std::string& param, int old_value, int new_value) {
        RCLCPP_INFO(this->get_logger(), "PARAM_UPDATE,node=localization,param=%s,old=%d,new=%d",
                    param.c_str(), old_value, new_value);
    };

    auto log_update_bool = [&](const std::string& param, bool old_value, bool new_value) {
        RCLCPP_INFO(this->get_logger(), "PARAM_UPDATE,node=localization,param=%s,old=%d,new=%d",
                    param.c_str(), static_cast<int>(old_value), static_cast<int>(new_value));
    };

    for (const auto& p : parameters) {
        const std::string& name = p.get_name();

        if (name == "registered_leaf_size") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
                reject("registered_leaf_size expects double");
                break;
            }
            const double old_v = registered_leaf_size_;
            registered_leaf_size_ = static_cast<float>(std::clamp(p.as_double(), 0.01, 2.0));
            log_update(name, old_v, registered_leaf_size_);
            continue;
        }
        if (name == "max_dist_sq") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
                reject("max_dist_sq expects double");
                break;
            }
            const double old_v = max_dist_sq_;
            max_dist_sq_ = static_cast<float>(std::clamp(p.as_double(), 0.01, 25.0));
            log_update(name, old_v, max_dist_sq_);
            continue;
        }
        if (name == "gicp_max_iterations") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
                reject("gicp_max_iterations expects integer");
                break;
            }
            const int old_v = gicp_max_iterations_;
            gicp_max_iterations_ = std::clamp(static_cast<int>(p.as_int()), 1, 500);
            log_update_int(name, old_v, gicp_max_iterations_);
            continue;
        }
        if (name == "sc_sim_threshold") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
                reject("sc_sim_threshold expects double");
                break;
            }
            const double old_v = sc_sim_threshold_;
            sc_sim_threshold_ = std::clamp(p.as_double(), 0.01, 1.0);
            log_update(name, old_v, sc_sim_threshold_);
            continue;
        }
        if (name == "sc_topk") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
                reject("sc_topk expects integer");
                break;
            }
            const int old_v = sc_topk_;
            sc_topk_ = std::clamp(static_cast<int>(p.as_int()), 1, 100);
            log_update_int(name, old_v, sc_topk_);
            continue;
        }
        if (name == "sc_num_rings") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
                reject("sc_num_rings expects integer");
                break;
            }
            const int old_v = sc_num_rings_;
            sc_num_rings_ = std::clamp(static_cast<int>(p.as_int()), 4, 200);
            need_sc_rebuild = true;
            log_update_int(name, old_v, sc_num_rings_);
            continue;
        }
        if (name == "sc_num_sectors") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
                reject("sc_num_sectors expects integer");
                break;
            }
            const int old_v = sc_num_sectors_;
            sc_num_sectors_ = std::clamp(static_cast<int>(p.as_int()), 12, 720);
            need_sc_rebuild = true;
            log_update_int(name, old_v, sc_num_sectors_);
            continue;
        }
        if (name == "sc_max_radius") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
                reject("sc_max_radius expects double");
                break;
            }
            const double old_v = sc_max_radius_;
            sc_max_radius_ = std::clamp(p.as_double(), 0.1, 100.0);
            need_sc_rebuild = true;
            log_update(name, old_v, sc_max_radius_);
            continue;
        }
        if (name == "global_icp_max_iterations") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
                reject("global_icp_max_iterations expects integer");
                break;
            }
            const int old_v = global_icp_max_iterations_;
            global_icp_max_iterations_ = std::clamp(static_cast<int>(p.as_int()), 1, 1000);
            log_update_int(name, old_v, global_icp_max_iterations_);
            continue;
        }
        if (name == "global_icp_max_correspondence_distance") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
                reject("global_icp_max_correspondence_distance expects double");
                break;
            }
            const double old_v = global_icp_max_correspondence_distance_;
            global_icp_max_correspondence_distance_ = std::clamp(p.as_double(), 0.01, 20.0);
            log_update(name, old_v, global_icp_max_correspondence_distance_);
            continue;
        }
        if (name == "global_downsample_leaf_size") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
                reject("global_downsample_leaf_size expects double");
                break;
            }
            const double old_v = global_downsample_leaf_size_;
            global_downsample_leaf_size_ = std::clamp(p.as_double(), 0.01, 2.0);
            log_update(name, old_v, global_downsample_leaf_size_);
            continue;
        }
        if (name == "global_leaf_size") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
                reject("global_leaf_size expects double");
                break;
            }
            const double old_v = global_leaf_size_;
            global_leaf_size_ = static_cast<float>(std::clamp(p.as_double(), 0.01, 2.0));
            need_target_rebuild = true;
            need_sc_rebuild = true;
            log_update(name, old_v, global_leaf_size_);
            continue;
        }
        if (name == "s1_enable") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
                reject("s1_enable expects bool");
                break;
            }
            const bool old_v = s1_enable_;
            s1_enable_ = p.as_bool();
            log_update_bool(name, old_v, s1_enable_);
            continue;
        }
        if (name == "s1_accel_threshold") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
                reject("s1_accel_threshold expects double");
                break;
            }
            const double old_v = s1_accel_threshold_;
            s1_accel_threshold_ = std::clamp(p.as_double(), 0.1, 200.0);
            log_update(name, old_v, s1_accel_threshold_);
            continue;
        }
        if (name == "s1_gyro_threshold") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
                reject("s1_gyro_threshold expects double");
                break;
            }
            const double old_v = s1_gyro_threshold_;
            s1_gyro_threshold_ = std::clamp(p.as_double(), 0.01, 50.0);
            log_update(name, old_v, s1_gyro_threshold_);
            continue;
        }
        if (name == "s1_freeze_duration_ms") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
                reject("s1_freeze_duration_ms expects integer");
                break;
            }
            const int old_v = s1_freeze_duration_ms_;
            s1_freeze_duration_ms_ = std::clamp(static_cast<int>(p.as_int()), 1, 5000);
            log_update_int(name, old_v, s1_freeze_duration_ms_);
            continue;
        }
        if (name == "s2_enable") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
                reject("s2_enable expects bool");
                break;
            }
            const bool old_v = s2_enable_;
            s2_enable_ = p.as_bool();
            log_update_bool(name, old_v, s2_enable_);
            continue;
        }
        if (name == "s2_hessian_min_eigenvalue") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
                reject("s2_hessian_min_eigenvalue expects double");
                break;
            }
            const double old_v = s2_hessian_min_eigenvalue_;
            s2_hessian_min_eigenvalue_ = std::clamp(p.as_double(), 1.0, 1e7);
            log_update(name, old_v, s2_hessian_min_eigenvalue_);
            continue;
        }
        if (name == "s2_max_continuous_frames") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
                reject("s2_max_continuous_frames expects integer");
                break;
            }
            const int old_v = s2_max_continuous_frames_;
            s2_max_continuous_frames_ = std::clamp(static_cast<int>(p.as_int()), 1, 1000);
            log_update_int(name, old_v, s2_max_continuous_frames_);
            continue;
        }
        if (name == "s3_enable") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
                reject("s3_enable expects bool");
                break;
            }
            const bool old_v = s3_enable_;
            s3_enable_ = p.as_bool();
            log_update_bool(name, old_v, s3_enable_);
            continue;
        }
        if (name == "s3_min_score_gap") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
                reject("s3_min_score_gap expects double");
                break;
            }
            const double old_v = s3_min_score_gap_;
            s3_min_score_gap_ = std::clamp(p.as_double(), 0.0, 10.0);
            log_update(name, old_v, s3_min_score_gap_);
            continue;
        }
        if (name == "gicp_omp_threads") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
                reject("gicp_omp_threads expects integer");
                break;
            }
            const int old_v = gicp_omp_threads_;
            gicp_omp_threads_ = std::max(1, static_cast<int>(p.as_int()));
            num_threads_ = gicp_omp_threads_;
            configureThreadAffinityQcs8550();
            log_update_int(name, old_v, gicp_omp_threads_);
            continue;
        }

        reject("parameter not in hot-update whitelist: " + name);
        break;
    }

    if (result.successful) {
        if (need_target_rebuild) {
            target_ready_ = false;
        }
        if (need_sc_rebuild) {
            sc_db_ready_ = false;
            std::lock_guard<std::mutex> lk(sc_mutex_);
            sc_database_.clear();
        }
    }

    return result;
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

void LocalizationNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    const double acc_norm = std::sqrt(
        msg->linear_acceleration.x * msg->linear_acceleration.x +
        msg->linear_acceleration.y * msg->linear_acceleration.y +
        msg->linear_acceleration.z * msg->linear_acceleration.z);
    const double gyro_norm = std::sqrt(
        msg->angular_velocity.x * msg->angular_velocity.x +
        msg->angular_velocity.y * msg->angular_velocity.y +
        msg->angular_velocity.z * msg->angular_velocity.z);

    // S1: Spike 检测
    if (s1_enable_ && (acc_norm > s1_accel_threshold_ || gyro_norm > s1_gyro_threshold_)) {
        std::lock_guard<std::mutex> lk(imu_spike_mutex_);
        imu_spike_deadline_ =
            now() + rclcpp::Duration(0, static_cast<int32_t>(s1_freeze_duration_ms_) * 1'000'000);
        imu_spike_active_.store(true);
    }

    // T8: 重力加速度估计 roll/pitch（准静态假设）
    if (slope_roll_pitch_from_imu_) {
        const double ax = msg->linear_acceleration.x;
        const double ay = msg->linear_acceleration.y;
        const double az = msg->linear_acceleration.z;
        const double roll  = std::atan2(ay, az);
        const double pitch = std::atan2(-ax, std::sqrt(ay * ay + az * az));
        std::lock_guard<std::mutex> lk(imu_attitude_mutex_);
        imu_roll_  = roll;
        imu_pitch_ = pitch;
        imu_attitude_valid_ = true;
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

    // T0: 局部配准计时起点
    const auto t_align_start = std::chrono::steady_clock::now();
    auto result = register_->align(*target_, *source_, *target_tree_, initial_guess);
    const double dt_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_align_start).count();

    // S2: Hessian 退化轴拒绝（align 后、S1 门控前）
    double s2_min_eig = 0.0;
    int s2_consec = 0;
    if (s2_enable_ && result.converged) {
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(result.H);
        s2_min_eig = solver.eigenvalues().minCoeff();
        if (s2_min_eig < s2_hessian_min_eigenvalue_) {
            s2_consec = consecutive_s2_count_.fetch_add(1) + 1;
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
                "S2: degenerate axis (min_eig=%.4f < %.4f), update rejected (consec=%d/%d)",
                s2_min_eig, s2_hessian_min_eigenvalue_, s2_consec, s2_max_continuous_frames_);
            if (s2_consec >= s2_max_continuous_frames_) {
                consecutive_s2_count_.store(0);
                RCLCPP_ERROR(get_logger(), "S2: %d frames continuous degeneration, forcing GLOBAL_RECOVERY",
                    s2_max_continuous_frames_);
                requestRelocalization(RelocTriggerReason::KIDNAP, cloud_to_register);
            }
            return;
        }
        consecutive_s2_count_.store(0);
    }

    const double normalized_error =
        (result.num_inliers > 0) ? (result.error / static_cast<double>(result.num_inliers))
                                 : std::numeric_limits<double>::max();

    // S1: IMU Spike 门控（种子更新前，同时暂停绑架计数）
    if (s1_enable_ && imu_spike_active_.load()) {
        rclcpp::Time deadline;
        { std::lock_guard<std::mutex> lk(imu_spike_mutex_); deadline = imu_spike_deadline_; }
        if (now() < deadline) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
                "S1: IMU spike gate active, seed frozen");
            return;
        }
        imu_spike_active_.store(false);
    }

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
    Eigen::Isometry3d slope_corrected_pose = result.T_target_source;

    // T8: 坡道法向一致性门控 + 姿态主导权（transform 更新前）
    if (!bad_quality && result.converged && slope_roll_pitch_from_imu_ && imu_attitude_valid_) {
        const Eigen::Matrix3d R = result.T_target_source.rotation();
        const double pitch_gicp = std::asin(-R(2, 0));
        const double roll_gicp  = std::atan2(R(2, 1), R(2, 2));
        double imu_r, imu_p;
        {
            std::lock_guard<std::mutex> lk(imu_attitude_mutex_);
            imu_r = imu_roll_;
            imu_p = imu_pitch_;
        }
        const double roll_dev  = roll_gicp - imu_r;
        const double pitch_dev = pitch_gicp - imu_p;
        const double dev_deg = std::sqrt(roll_dev * roll_dev + pitch_dev * pitch_dev) * 180.0 / M_PI;
        if (dev_deg > slope_normal_consistency_deg_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
                "T8: slope normal inconsistency %.1f deg > %.1f deg, frame rejected",
                dev_deg, slope_normal_consistency_deg_);
            return;
        }

        // roll/pitch 以 IMU 为主，仅允许 GICP 在有限幅度内修正
        const double max_correction_rad = kMaxSlopeRollPitchCorrectionDeg * M_PI / 180.0;
        const double fused_roll = imu_r + std::clamp(roll_dev, -max_correction_rad, max_correction_rad);
        const double fused_pitch = imu_p + std::clamp(pitch_dev, -max_correction_rad, max_correction_rad);
        const double yaw_gicp = std::atan2(R(1, 0), R(0, 0));
        slope_corrected_pose.linear() =
            (Eigen::AngleAxisd(yaw_gicp, Eigen::Vector3d::UnitZ()) *
             Eigen::AngleAxisd(fused_pitch, Eigen::Vector3d::UnitY()) *
             Eigen::AngleAxisd(fused_roll, Eigen::Vector3d::UnitX()))
                .toRotationMatrix();

        // 将 slope_z_weight 用于 z 轴更新抑制，减小坡道场景 z 抖动
        const double prev_z = initial_guess.translation().z();
        const double z_alpha = 1.0 / slope_z_weight_;
        slope_corrected_pose.translation().z() =
            prev_z + (slope_corrected_pose.translation().z() - prev_z) * z_alpha;
    }

    if (!bad_quality) {
        std::lock_guard<std::mutex> lock(result_mutex_);
        if (result.converged) {
            result_t_ = previous_result_t_ = slope_corrected_pose;
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

    // T0: PERF_METRIC 可观测性日志
    RCLCPP_INFO(get_logger(),
        "PERF_METRIC phase=LOCAL dt_ms=%.1f inliers=%d norm_err=%.4f state=%s S1=%d S2=%d(eig=%.2f,consec=%d)",
        dt_ms, static_cast<int>(result.num_inliers), normalized_error,
        toString(getLocalizationState()),
        static_cast<int>(imu_spike_active_.load()),
        static_cast<int>(s2_enable_ && result.converged && s2_min_eig < s2_hessian_min_eigenvalue_),
        s2_min_eig, s2_consec);
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

        RCLCPP_INFO(this->get_logger(), "RZ候选 yaw=%.1f°: fitness=%.4f, J=%.4f", yaw_deg, fitness, cost);

        if (!found || cost < best_cost || (std::abs(cost - best_cost) < 1e-6 && fitness < best_fitness)) {
            found = true;
            best_fitness = fitness;
            best_cost = cost;
            best_pose = Eigen::Isometry3d::Identity();
            best_pose.matrix() = refined.cast<double>();
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
    std::vector<double> all_fitness;  // S3: 收集所有收敛候选的 fitness

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
        all_fitness.push_back(fitness);  // S3: 收集
        double cost = computeCandidateCost(fitness, seed, refined) + std::max(0.0, sim_cost - sc_sim_threshold_);

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

    // S3: SC 对称歧义拒绝（TopK 遍历结束后）
    if (s3_enable_ && all_fitness.size() >= 2) {
        std::sort(all_fitness.begin(), all_fitness.end());
        if (std::abs(all_fitness[0] - all_fitness[1]) < s3_min_score_gap_) {
            RCLCPP_WARN(get_logger(),
                "S3: SC symmetry ambiguity (best=%.4f, 2nd=%.4f), reloc rejected",
                all_fitness[0], all_fitness[1]);
            return false;
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
            RCLCPP_INFO(
                get_logger(),
                "PERF_METRIC phase=L1 dt_ms=%.1f candidates=%d best_fit=%.4f best_j=%.4f accepted=%d state=%s",
                metrics.t_l1_ms, candidate_count, best_fitness, best_cost, static_cast<int>(accepted),
                toString(getLocalizationState()));
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
            RCLCPP_INFO(
                get_logger(),
                "PERF_METRIC phase=L2 dt_ms=%.1f candidates=%d best_fit=%.4f best_j=%.4f accepted=%d state=%s",
                metrics.t_l2_ms, candidate_count, best_fitness, best_cost, static_cast<int>(accepted),
                toString(getLocalizationState()));
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
