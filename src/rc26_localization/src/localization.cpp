#include "rc26_localization/localization.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "pcl/features/fpfh_omp.h"
#include "pcl/features/normal_3d_omp.h"
#include "pcl/filters/filter.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/registration/ia_ransac.h"
#include "pcl/search/kdtree.h"
#include "pcl_conversions/pcl_conversions.h"
#include "small_gicp/pcl/pcl_registration.hpp"
#include "small_gicp/util/downsampling_omp.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

namespace rc26_localization {

namespace {
constexpr double kNearZero = 1e-9;
constexpr int kMinPointsForRegistration = 20;
constexpr size_t kMaxAccumulatedPoints = 200000;
constexpr int kRegistrationPeriodMs = 500;
constexpr int kTransformPeriodMs = 50;
constexpr double kTfTimeoutSec = 1.0;
constexpr double kBadCovScale = 25.0;
constexpr double kRadToDeg = 180.0 / M_PI;
constexpr double kSacIaMinSampleDistance = 0.1;
constexpr int kSacIaCorrespondenceRandomness = 50;
constexpr int kSacIaNumSamples = 5;
constexpr std::array<double, 6> kPoseCovDiag{{0.05, 0.05, 0.10, 0.05, 0.05, 0.05}};

std::string boolText(bool value) {
    return value ? "true" : "false";
}

template <typename PointT>
void finalizeCloud(pcl::PointCloud<PointT>& cloud) {
    cloud.width = static_cast<uint32_t>(cloud.points.size());
    cloud.height = 1;
    cloud.is_dense = false;
}

Eigen::Isometry3d toIsometry3d(const Eigen::Matrix4f& matrix) {
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.matrix() = matrix.cast<double>();
    return transform;
}

std::string diagnosticHumanMessage(const std::string& reason,
                                   const std::string& startup_state,
                                   const std::string& online_state) {
    if (reason == "map_path_empty") {
        return "没有配置 prior_pcd_file，定位无法加载先验地图。";
    }
    if (reason == "map_load_failed") {
        return "先验 PCD 读取失败，请检查 prior_pcd_file 路径和文件内容。";
    }
    if (reason == "map_not_ready") {
        return "先验地图还没准备好，定位暂时不能配准。";
    }
    if (reason == "startup_relocalizing") {
        return "开局重定位正在收集 registered_scan 点云。";
    }
    if (reason == "startup_relocalization_ok") {
        return "开局重定位成功，map->odom 已接管。";
    }
    if (reason == "startup_relocalization_failed") {
        if (startup_state == "failed_empty_source") {
            return "开局重定位失败：没有可用实时点云。";
        }
        if (startup_state == "failed_target_not_ready") {
            return "开局重定位失败：先验地图目标还没准备好。";
        }
        if (startup_state == "failed_insufficient_source") {
            return "开局重定位失败：实时点云点数太少。";
        }
        if (startup_state == "failed_source_fpfh") {
            return "开局重定位失败：实时点云特征计算失败。";
        }
        if (startup_state == "failed_sac_ia") {
            return "开局重定位失败：SAC-IA 粗配准没有收敛。";
        }
        if (startup_state == "failed_insufficient_gicp_source") {
            return "开局重定位失败：GICP 输入点数太少。";
        }
        if (startup_state == "failed_gicp_quality") {
            return "开局重定位失败：GICP 质量门控未通过。";
        }
        return "开局重定位失败，请结合 inliers 和 normalized_error 查看原因。";
    }
    if (reason == "no_accumulated_cloud") {
        return "暂时没有收到 registered_scan，map->odom 保持上一次结果。";
    }
    if (reason == "local_tracking_waiting_for_initial_pose") {
        return "局部跟踪等待可信初值，需等待开局重定位成功或由 initialpose 接管。";
    }
    if (reason == "local_target_insufficient") {
        return "初值附近局部地图点数不足，拒绝退回整图盲配准，map->odom 保持上一次结果。";
    }
    if (reason == "insufficient_source_points") {
        return "实时点云下采样后点数太少，本次定位不更新。";
    }
    if (reason == "local_registration_ok") {
        return "局部配准通过，map->odom 已更新。";
    }
    if (reason == "local_registration_frozen") {
        return "局部配准质量不够，map->odom 暂时冻结在上一次结果。";
    }
    if (reason == "initialpose_invalid_quaternion") {
        return "initialpose 姿态四元数无效，定位未接管。";
    }
    if (reason == "initialpose_accepted") {
        return "initialpose 已接受，map->odom 已重置。";
    }
    if (reason == "initialpose_tf_lookup_failed") {
        return "initialpose 无法查询 odom->机器人基座帧 TF，定位未接管。";
    }
    if (reason == "online_relocalization_running") {
        return "实验性在线重定位正在后台执行，map->odom 发布不会被阻塞。";
    }
    if (reason == "online_relocalization_ok") {
        return "实验性在线重定位成功，map->odom 已由先验 PCD 配准结果接管。";
    }
    if (reason == "online_relocalization_failed") {
        if (online_state == "failed_empty_source") {
            return "实验性在线重定位失败：没有可用实时点云。";
        }
        if (online_state == "failed_target_not_ready") {
            return "实验性在线重定位失败：先验地图目标还没准备好。";
        }
        if (online_state == "failed_insufficient_source") {
            return "实验性在线重定位失败：实时点云点数太少。";
        }
        if (online_state == "failed_source_fpfh") {
            return "实验性在线重定位失败：实时点云特征计算失败。";
        }
        if (online_state == "failed_sac_ia") {
            return "实验性在线重定位失败：SAC-IA 粗配准没有收敛。";
        }
        if (online_state == "failed_insufficient_gicp_source") {
            return "实验性在线重定位失败：GICP 输入点数太少。";
        }
        if (online_state == "failed_gicp_quality") {
            return "实验性在线重定位失败：GICP 质量门控未通过。";
        }
        if (online_state == "cancelled_by_initialpose") {
            return "实验性在线重定位结果已取消：人工 initialpose 已先接管。";
        }
        return "实验性在线重定位失败，请结合 inliers 和 normalized_error 查看原因。";
    }
    return "定位状态已更新，请查看 reason、accepted、inliers 和 normalized_error。";
}
}  // namespace

LocalizationNode::LocalizationNode(const rclcpp::NodeOptions& options)
    : Node("localization", options),
      accumulated_cloud_(std::make_shared<pcl::PointCloud<pcl::PointXYZ>>()),
      global_map_(std::make_shared<pcl::PointCloud<pcl::PointXYZ>>()),
      startup_target_(std::make_shared<pcl::PointCloud<pcl::PointXYZ>>()),
      startup_target_fpfh_(std::make_shared<pcl::PointCloud<pcl::FPFHSignature33>>()),
      registration_(std::make_shared<
                    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>()) {
    this->declare_parameter("num_threads", num_threads_);
    this->declare_parameter("num_neighbors", num_neighbors_);
    this->declare_parameter("global_leaf_size", global_leaf_size_);
    this->declare_parameter("registered_leaf_size", registered_leaf_size_);
    this->declare_parameter("max_dist_sq", max_dist_sq_);
    this->declare_parameter("gicp_max_iterations", gicp_max_iterations_);
    this->declare_parameter("min_inliers", min_inliers_);
    this->declare_parameter("max_normalized_error", max_normalized_error_);
    this->declare_parameter("require_initial_pose_for_local_tracking", require_initial_pose_for_local_tracking_);
    this->declare_parameter("local_target_enable", local_target_enable_);
    this->declare_parameter("local_target_radius_m", local_target_radius_m_);
    this->declare_parameter("local_target_min_points", local_target_min_points_);
    this->declare_parameter("registration_jump_gate_enable", registration_jump_gate_enable_);
    this->declare_parameter("max_registration_translation_delta_m", max_registration_translation_delta_m_);
    this->declare_parameter("max_registration_z_delta_m", max_registration_z_delta_m_);
    this->declare_parameter("max_registration_yaw_delta_rad", max_registration_yaw_delta_rad_);
    this->declare_parameter("registration_smoothing_enable", registration_smoothing_enable_);
    this->declare_parameter("registration_smoothing_alpha", registration_smoothing_alpha_);
    this->declare_parameter("startup_relocalization_enable", startup_relocalization_enable_);
    this->declare_parameter("startup_collect_ms", startup_collect_ms_);
    this->declare_parameter("startup_leaf_size", startup_leaf_size_);
    this->declare_parameter("startup_global_grid_enable", startup_global_grid_enable_);
    this->declare_parameter("startup_global_grid_resolution", startup_global_grid_resolution_);
    this->declare_parameter("startup_global_grid_yaw_step_deg", startup_global_grid_yaw_step_deg_);
    this->declare_parameter("startup_global_grid_max_candidates", startup_global_grid_max_candidates_);
    this->declare_parameter("startup_global_grid_min_overlap_ratio", startup_global_grid_min_overlap_ratio_);
    this->declare_parameter("startup_global_grid_max_normalized_error", startup_global_grid_max_normalized_error_);
    this->declare_parameter("startup_global_grid_sac_fallback", startup_global_grid_sac_fallback_);
    this->declare_parameter("online_relocalization_enable", online_relocalization_enable_);
    this->declare_parameter("online_relocalization_trigger_after_failures",
                            online_relocalization_trigger_after_failures_);
    this->declare_parameter("online_relocalization_cooldown_ms", online_relocalization_cooldown_ms_);
    this->declare_parameter("online_relocalization_max_attempts", online_relocalization_max_attempts_);
    this->declare_parameter("online_relocalization_collect_ms", online_relocalization_collect_ms_);
    this->declare_parameter("online_relocalization_leaf_size", online_relocalization_leaf_size_);
    this->declare_parameter("map_frame", map_frame_);
    this->declare_parameter("odom_frame", odom_frame_);
    this->declare_parameter("robot_base_frame", robot_base_frame_);
    this->declare_parameter("prior_pcd_file", prior_pcd_file_);
    this->declare_parameter("init_pose", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    this->declare_parameter("input_cloud_topic", input_cloud_topic_);
    this->declare_parameter("input_cloud_queue_size", input_cloud_queue_size_);
    this->declare_parameter("pose_cov_topic", pose_cov_topic_);
    this->declare_parameter("diagnostics_topic", diagnostics_topic_);

    this->get_parameter("num_threads", num_threads_);
    this->get_parameter("num_neighbors", num_neighbors_);
    this->get_parameter("global_leaf_size", global_leaf_size_);
    this->get_parameter("registered_leaf_size", registered_leaf_size_);
    this->get_parameter("max_dist_sq", max_dist_sq_);
    this->get_parameter("gicp_max_iterations", gicp_max_iterations_);
    this->get_parameter("min_inliers", min_inliers_);
    this->get_parameter("max_normalized_error", max_normalized_error_);
    this->get_parameter("require_initial_pose_for_local_tracking", require_initial_pose_for_local_tracking_);
    this->get_parameter("local_target_enable", local_target_enable_);
    this->get_parameter("local_target_radius_m", local_target_radius_m_);
    this->get_parameter("local_target_min_points", local_target_min_points_);
    this->get_parameter("registration_jump_gate_enable", registration_jump_gate_enable_);
    this->get_parameter("max_registration_translation_delta_m", max_registration_translation_delta_m_);
    this->get_parameter("max_registration_z_delta_m", max_registration_z_delta_m_);
    this->get_parameter("max_registration_yaw_delta_rad", max_registration_yaw_delta_rad_);
    this->get_parameter("registration_smoothing_enable", registration_smoothing_enable_);
    this->get_parameter("registration_smoothing_alpha", registration_smoothing_alpha_);
    this->get_parameter("startup_relocalization_enable", startup_relocalization_enable_);
    this->get_parameter("startup_collect_ms", startup_collect_ms_);
    this->get_parameter("startup_leaf_size", startup_leaf_size_);
    this->get_parameter("startup_global_grid_enable", startup_global_grid_enable_);
    this->get_parameter("startup_global_grid_resolution", startup_global_grid_resolution_);
    this->get_parameter("startup_global_grid_yaw_step_deg", startup_global_grid_yaw_step_deg_);
    this->get_parameter("startup_global_grid_max_candidates", startup_global_grid_max_candidates_);
    this->get_parameter("startup_global_grid_min_overlap_ratio", startup_global_grid_min_overlap_ratio_);
    this->get_parameter("startup_global_grid_max_normalized_error", startup_global_grid_max_normalized_error_);
    this->get_parameter("startup_global_grid_sac_fallback", startup_global_grid_sac_fallback_);
    this->get_parameter("online_relocalization_enable", online_relocalization_enable_);
    this->get_parameter("online_relocalization_trigger_after_failures",
                        online_relocalization_trigger_after_failures_);
    this->get_parameter("online_relocalization_cooldown_ms", online_relocalization_cooldown_ms_);
    this->get_parameter("online_relocalization_max_attempts", online_relocalization_max_attempts_);
    this->get_parameter("online_relocalization_collect_ms", online_relocalization_collect_ms_);
    this->get_parameter("online_relocalization_leaf_size", online_relocalization_leaf_size_);
    this->get_parameter("map_frame", map_frame_);
    this->get_parameter("odom_frame", odom_frame_);
    this->get_parameter("robot_base_frame", robot_base_frame_);
    this->get_parameter("prior_pcd_file", prior_pcd_file_);
    this->get_parameter("init_pose", init_pose_);
    this->get_parameter("input_cloud_topic", input_cloud_topic_);
    this->get_parameter("input_cloud_queue_size", input_cloud_queue_size_);
    this->get_parameter("pose_cov_topic", pose_cov_topic_);
    this->get_parameter("diagnostics_topic", diagnostics_topic_);

    num_threads_ = std::max(1, num_threads_);
    num_neighbors_ = std::max(3, num_neighbors_);
    global_leaf_size_ = std::max(0.01, global_leaf_size_);
    registered_leaf_size_ = std::max(0.01, registered_leaf_size_);
    max_dist_sq_ = std::max(0.01, max_dist_sq_);
    gicp_max_iterations_ = std::max(1, gicp_max_iterations_);
    min_inliers_ = std::max(0, min_inliers_);
    max_normalized_error_ = std::max(kNearZero, max_normalized_error_);
    local_target_radius_m_ = std::max(0.1, local_target_radius_m_);
    local_target_min_points_ = std::max(kMinPointsForRegistration, local_target_min_points_);
    max_registration_translation_delta_m_ = std::max(kNearZero, max_registration_translation_delta_m_);
    max_registration_z_delta_m_ = std::max(kNearZero, max_registration_z_delta_m_);
    max_registration_yaw_delta_rad_ = std::max(kNearZero, max_registration_yaw_delta_rad_);
    registration_smoothing_alpha_ = std::clamp(registration_smoothing_alpha_, kNearZero, 1.0);
    startup_collect_ms_ = std::max(0, startup_collect_ms_);
    startup_leaf_size_ = std::max(0.05, startup_leaf_size_);
    startup_global_grid_resolution_ = std::max(0.05, startup_global_grid_resolution_);
    startup_global_grid_yaw_step_deg_ = std::clamp(startup_global_grid_yaw_step_deg_, 1.0, 90.0);
    startup_global_grid_max_candidates_ = std::max(1, startup_global_grid_max_candidates_);
    startup_global_grid_min_overlap_ratio_ = std::clamp(startup_global_grid_min_overlap_ratio_, 0.0, 1.0);
    startup_global_grid_max_normalized_error_ = std::max(kNearZero, startup_global_grid_max_normalized_error_);
    online_relocalization_trigger_after_failures_ =
        std::max(1, online_relocalization_trigger_after_failures_);
    online_relocalization_cooldown_ms_ = std::max(0, online_relocalization_cooldown_ms_);
    online_relocalization_max_attempts_ = std::max(1, online_relocalization_max_attempts_);
    online_relocalization_collect_ms_ = std::max(0, online_relocalization_collect_ms_);
    online_relocalization_leaf_size_ = std::max(0.05, online_relocalization_leaf_size_);
    input_cloud_queue_size_ = std::max(1, input_cloud_queue_size_);
    last_pose_cov_diag_ = kPoseCovDiag;
    startup_relocalization_pending_ = startup_relocalization_enable_;
    startup_relocalization_state_ = startup_relocalization_enable_ ? "pending" : "disabled";
    online_relocalization_state_ = online_relocalization_enable_ ? "idle" : "disabled";
    online_relocalization_reason_ = online_relocalization_enable_ ? "waiting_for_failures" : "disabled";
    last_online_relocalization_attempt_wall_ =
        std::chrono::steady_clock::now() - std::chrono::milliseconds(online_relocalization_cooldown_ms_);
    startup_begin_wall_ = std::chrono::steady_clock::now();
    if (robot_base_frame_.empty()) {
        robot_base_frame_ = "base_footprint";
    }

    if (init_pose_.size() >= 6) {
        result_t_.translation() << init_pose_[0], init_pose_[1], init_pose_[2];
        result_t_.linear() = (Eigen::AngleAxisd(init_pose_[5], Eigen::Vector3d::UnitZ()) *
                              Eigen::AngleAxisd(init_pose_[4], Eigen::Vector3d::UnitY()) *
                              Eigen::AngleAxisd(init_pose_[3], Eigen::Vector3d::UnitX()))
                                 .toRotationMatrix();
        previous_result_t_ = result_t_;
        tracking_initialized_ = std::any_of(init_pose_.begin(), init_pose_.begin() + 6,
                                            [](double value) { return std::abs(value) > kNearZero; });
    }

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    pose_cov_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(pose_cov_topic_, 10);
    diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(diagnostics_topic_, 10);
    pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_cloud_topic_, input_cloud_queue_size_,
        std::bind(&LocalizationNode::registeredPcdCallback, this, std::placeholders::_1));
    initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "initialpose", 10, std::bind(&LocalizationNode::initialPoseCallback, this, std::placeholders::_1));

    loadGlobalMap(prior_pcd_file_);

    registration_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(kRegistrationPeriodMs), std::bind(&LocalizationNode::performRegistration, this));
    transform_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(kTransformPeriodMs), std::bind(&LocalizationNode::publishTransform, this));

    RCLCPP_INFO(this->get_logger(),
                "定位链路已启动: 输入点云=%s(queue=%d), map_frame=%s, odom_frame=%s, base=%s, 开局重定位=%s, 实验性在线重定位=%s",
                input_cloud_topic_.c_str(), input_cloud_queue_size_, map_frame_.c_str(), odom_frame_.c_str(),
                robot_base_frame_.c_str(), startup_relocalization_enable_ ? "开启" : "关闭",
                online_relocalization_enable_ ? "开启" : "关闭");
}

LocalizationNode::~LocalizationNode() {
    online_relocalization_stop_ = true;
    online_relocalization_cancel_requested_ = true;
    if (online_relocalization_worker_.joinable()) {
        online_relocalization_worker_.join();
    }
}

void LocalizationNode::loadGlobalMap(const std::string& file_name) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    map_loaded_ = false;
    target_ready_ = false;
    startup_target_ready_ = false;
    target_.reset();
    target_tree_.reset();
    startup_target_->clear();
    startup_target_fpfh_->clear();
    global_map_->clear();

    if (file_name.empty()) {
        RCLCPP_ERROR(this->get_logger(), "prior_pcd_file 为空，定位无法加载先验地图");
        publishDiagnostics("map_path_empty", diagnostic_msgs::msg::DiagnosticStatus::ERROR, false, false, 0,
                           std::numeric_limits<double>::infinity(), 0);
        return;
    }

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *global_map_) == -1) {
        RCLCPP_ERROR(this->get_logger(), "先验 PCD 读取失败: %s", file_name.c_str());
        publishDiagnostics("map_load_failed", diagnostic_msgs::msg::DiagnosticStatus::ERROR, false, false, 0,
                           std::numeric_limits<double>::infinity(), 0);
        return;
    }

    pcl::Indices indices;
    pcl::removeNaNFromPointCloud(*global_map_, *global_map_, indices);
    map_loaded_ = !global_map_->empty();
    RCLCPP_INFO(this->get_logger(), "先验地图已加载: 点数=%zu, 文件=%s", global_map_->size(), file_name.c_str());
    (void)prepareTargetMap();
    if (startup_relocalization_enable_ || online_relocalization_enable_) {
        (void)prepareStartupTarget();
    }
}

bool LocalizationNode::prepareTargetMap() {
    if (!map_loaded_ || !global_map_ || global_map_->empty()) {
        return false;
    }
    if (target_ready_ && target_tree_) {
        return true;
    }

    target_ = small_gicp::voxelgrid_sampling_omp<pcl::PointCloud<pcl::PointXYZ>,
                                                 pcl::PointCloud<pcl::PointCovariance>>(*global_map_,
                                                                                        global_leaf_size_);
    if (!target_ || target_->size() < static_cast<size_t>(kMinPointsForRegistration)) {
        RCLCPP_ERROR(this->get_logger(), "先验地图下采样后点数太少: %zu < %d",
                     target_ ? target_->size() : 0U, kMinPointsForRegistration);
        target_ready_ = false;
        target_tree_.reset();
        return false;
    }

    small_gicp::estimate_covariances_omp(*target_, num_neighbors_, num_threads_);
    target_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
        target_, small_gicp::KdTreeBuilderOMP(num_threads_));
    target_ready_ = true;
    RCLCPP_INFO(this->get_logger(), "先验地图配准目标已准备好: 点数=%zu", target_->size());
    return true;
}

bool LocalizationNode::prepareStartupTarget() {
    if (!map_loaded_ || !global_map_ || global_map_->empty()) {
        return false;
    }
    if (startup_target_ready_ && startup_target_ && !startup_target_->empty() && startup_target_fpfh_ &&
        !startup_target_fpfh_->empty()) {
        return true;
    }

    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setLeafSize(static_cast<float>(startup_leaf_size_), static_cast<float>(startup_leaf_size_),
                      static_cast<float>(startup_leaf_size_));
    voxel.setInputCloud(global_map_);
    startup_target_->clear();
    voxel.filter(*startup_target_);

    if (startup_target_->size() < static_cast<size_t>(kMinPointsForRegistration)) {
        RCLCPP_WARN(this->get_logger(), "开局重定位地图目标点数太少: %zu < %d",
                    startup_target_->size(), kMinPointsForRegistration);
        startup_target_ready_ = false;
        startup_target_fpfh_->clear();
        return false;
    }

    startup_target_fpfh_ = computeFpfh(startup_target_);
    startup_target_ready_ = startup_target_fpfh_ && !startup_target_fpfh_->empty();
    if (!startup_target_ready_) {
        RCLCPP_WARN(this->get_logger(), "开局重定位地图目标 FPFH 特征计算失败");
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "开局重定位地图目标已准备好: 点数=%zu",
                startup_target_->size());
    return true;
}

LocalizationNode::RegistrationTargetView LocalizationNode::makeLocalRegistrationTarget(
    const Eigen::Isometry3d& initial_guess) const {
    RegistrationTargetView view;
    if (!target_ready_ || !target_ || target_->empty() || !target_tree_) {
        view.failure_reason = "target_not_ready";
        return view;
    }

    if (!local_target_enable_) {
        view.cloud = target_;
        view.tree = target_tree_;
        view.point_count = target_->size();
        view.local_target_used = false;
        return view;
    }

    const Eigen::Vector3d center = initial_guess.translation();
    const double radius_sq = local_target_radius_m_ * local_target_radius_m_;
    auto local_cloud = std::make_shared<pcl::PointCloud<pcl::PointCovariance>>();
    local_cloud->reserve(target_->size());
    for (const auto& point : target_->points) {
        const double dx = static_cast<double>(point.x) - center.x();
        const double dy = static_cast<double>(point.y) - center.y();
        if (dx * dx + dy * dy <= radius_sq) {
            local_cloud->push_back(point);
        }
    }
    finalizeCloud(*local_cloud);

    if (local_cloud->size() < static_cast<size_t>(local_target_min_points_)) {
        view.failure_reason = "local_target_insufficient";
        view.point_count = local_cloud->size();
        return view;
    }

    auto local_tree = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
        local_cloud, small_gicp::KdTreeBuilderOMP(num_threads_));
    view.cloud = std::move(local_cloud);
    view.tree = std::move(local_tree);
    view.point_count = view.cloud->size();
    view.local_target_used = true;
    return view;
}

double LocalizationNode::yawDeltaRad(const Eigen::Isometry3d& from, const Eigen::Isometry3d& to) {
    const Eigen::Matrix3d delta = from.rotation().transpose() * to.rotation();
    return std::abs(std::atan2(delta(1, 0), delta(0, 0)));
}

Eigen::Isometry3d LocalizationNode::interpolateTransform(const Eigen::Isometry3d& from,
                                                         const Eigen::Isometry3d& to,
                                                         double alpha) {
    alpha = std::clamp(alpha, 0.0, 1.0);
    Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
    out.translation() = from.translation() + alpha * (to.translation() - from.translation());
    Eigen::Quaterniond q_from(from.rotation());
    Eigen::Quaterniond q_to(to.rotation());
    if (q_from.dot(q_to) < 0.0) {
        q_to.coeffs() *= -1.0;
    }
    out.linear() = q_from.slerp(alpha, q_to).normalized().toRotationMatrix();
    return out;
}

void LocalizationNode::updateRegistrationDebug(double delta_translation_m, double delta_z_m,
                                               double delta_yaw_rad, size_t active_target_points,
                                               bool local_target_used,
                                               const std::string& rejection_reason) {
    std::lock_guard<std::mutex> lock(diagnostics_mutex_);
    last_registration_delta_translation_m_ = delta_translation_m;
    last_registration_delta_z_m_ = delta_z_m;
    last_registration_delta_yaw_rad_ = delta_yaw_rad;
    last_active_target_points_ = active_target_points;
    last_local_target_used_ = local_target_used;
    last_registration_rejection_reason_ = rejection_reason;
}

void LocalizationNode::registeredPcdCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (!msg) {
        return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr scan(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *scan);

    std::lock_guard<std::mutex> lock(cloud_mutex_);
    last_scan_time_ = rclcpp::Time(msg->header.stamp);
    current_scan_frame_id_ = msg->header.frame_id;
    if (startup_relocalization_pending_ && !startup_first_cloud_seen_) {
        startup_first_cloud_wall_ = std::chrono::steady_clock::now();
        startup_first_cloud_seen_ = true;
    }

    const size_t current_size = accumulated_cloud_->size();
    if (current_size >= kMaxAccumulatedPoints) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "定位累计点云达到上限: %zu，暂时丢弃新点", kMaxAccumulatedPoints);
        return;
    }

    const size_t remaining = kMaxAccumulatedPoints - current_size;
    if (scan->size() > remaining) {
        accumulated_cloud_->points.insert(accumulated_cloud_->points.end(), scan->points.begin(),
                                          scan->points.begin() + static_cast<std::ptrdiff_t>(remaining));
    } else {
        *accumulated_cloud_ += *scan;
    }
    accumulated_cloud_->width = static_cast<uint32_t>(accumulated_cloud_->points.size());
    accumulated_cloud_->height = 1;
}

void LocalizationNode::initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    if (!msg) {
        return;
    }

    const Eigen::Quaterniond q_raw(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                                   msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    if (!q_raw.coeffs().allFinite() || q_raw.norm() < kNearZero) {
        RCLCPP_WARN(this->get_logger(), "initialpose 被拒绝: 姿态四元数无效");
        publishDiagnostics("initialpose_invalid_quaternion", diagnostic_msgs::msg::DiagnosticStatus::WARN, false,
                           false, 0, std::numeric_limits<double>::infinity(), 0);
        return;
    }

    Eigen::Isometry3d map_to_base = Eigen::Isometry3d::Identity();
    map_to_base.translation() << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    map_to_base.linear() = q_raw.normalized().toRotationMatrix();

    try {
        const auto stamp = rclcpp::Time(msg->header.stamp);
        const auto transform =
            tf_buffer_->lookupTransform(odom_frame_, robot_base_frame_, stamp,
                                        rclcpp::Duration::from_seconds(kTfTimeoutSec));
        const Eigen::Isometry3d odom_to_base = tf2::transformToEigen(transform.transform);
        const Eigen::Isometry3d map_to_odom = map_to_base * odom_to_base.inverse();

        {
            std::lock_guard<std::mutex> lock(result_mutex_);
            result_t_ = map_to_odom;
            previous_result_t_ = map_to_odom;
            tracking_initialized_ = true;
            last_pose_cov_diag_ = kPoseCovDiag;
            pending_online_relocalization_result_ = false;
            startup_relocalization_pending_ = false;
            startup_relocalization_attempted_ = true;
            startup_relocalization_state_ = "initialpose_override";
        }
        online_relocalization_cancel_requested_ = true;
        {
            std::lock_guard<std::mutex> lock(online_relocalization_mutex_);
            consecutive_registration_failures_ = 0;
            online_relocalization_attempts_ = 0;
            online_relocalization_state_ =
                online_relocalization_enable_ ? "initialpose_override" : "disabled";
            online_relocalization_reason_ =
                online_relocalization_enable_ ? "initialpose_accepted" : "disabled";
            last_relocalization_source_ = "initialpose";
            last_online_relocalization_attempt_wall_ =
                std::chrono::steady_clock::now() -
                std::chrono::milliseconds(online_relocalization_cooldown_ms_);
        }

        RCLCPP_INFO(this->get_logger(), "initialpose 已接受，map->odom 已重置");
        publishDiagnostics("initialpose_accepted", diagnostic_msgs::msg::DiagnosticStatus::OK, true, true, 0, 0.0, 0);
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "initialpose 被拒绝: 无法查询 TF %s -> %s: %s", odom_frame_.c_str(),
                    robot_base_frame_.c_str(), ex.what());
        publishDiagnostics("initialpose_tf_lookup_failed", diagnostic_msgs::msg::DiagnosticStatus::WARN, false, false,
                           0, std::numeric_limits<double>::infinity(), 0);
    }
}

void LocalizationNode::performRegistration() {
    if (!map_loaded_ || !prepareTargetMap()) {
        publishDiagnostics("map_not_ready", diagnostic_msgs::msg::DiagnosticStatus::ERROR, false, false, 0,
                           std::numeric_limits<double>::infinity(), 0);
        return;
    }
    consumePendingOnlineRelocalizationResult();

    if (startup_relocalization_pending_) {
        size_t accumulated_points = 0;
        bool first_cloud_seen = false;
        std::chrono::steady_clock::time_point first_cloud_wall;
        {
            std::lock_guard<std::mutex> lock(cloud_mutex_);
            accumulated_points = accumulated_cloud_->size();
            first_cloud_seen = startup_first_cloud_seen_;
            first_cloud_wall = startup_first_cloud_wall_;
        }

        const auto elapsed_ms =
            first_cloud_seen
                ? std::chrono::duration_cast<std::chrono::milliseconds>(
                      std::chrono::steady_clock::now() - first_cloud_wall)
                      .count()
                : 0;
        if (!first_cloud_seen || elapsed_ms < startup_collect_ms_ ||
            accumulated_points < static_cast<size_t>(kMinPointsForRegistration)) {
            startup_relocalization_state_ = "collecting";
            publishDiagnostics("startup_relocalizing", diagnostic_msgs::msg::DiagnosticStatus::OK, false, false, 0,
                               std::numeric_limits<double>::infinity(), accumulated_points);
            return;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr startup_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        {
            std::lock_guard<std::mutex> lock(cloud_mutex_);
            std::swap(startup_cloud, accumulated_cloud_);
        }

        startup_relocalization_attempted_ = true;
        startup_relocalization_pending_ = false;
        (void)performStartupRelocalization(startup_cloud);
        return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_register(new pcl::PointCloud<pcl::PointXYZ>());
    {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        if (accumulated_cloud_->empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "暂时没有累计到 registered_scan 点云，定位本轮不更新");
            publishDiagnostics("no_accumulated_cloud", diagnostic_msgs::msg::DiagnosticStatus::WARN, false, false, 0,
                               std::numeric_limits<double>::infinity(), 0);
            return;
        }
        std::swap(cloud_to_register, accumulated_cloud_);
    }

    pcl::Indices indices;
    pcl::removeNaNFromPointCloud(*cloud_to_register, *cloud_to_register, indices);
    source_ = small_gicp::voxelgrid_sampling_omp<pcl::PointCloud<pcl::PointXYZ>,
                                                 pcl::PointCloud<pcl::PointCovariance>>(*cloud_to_register,
                                                                                        registered_leaf_size_);
    const size_t source_points = source_ ? source_->size() : 0U;
    if (!source_ || source_points < static_cast<size_t>(kMinPointsForRegistration)) {
        {
            std::lock_guard<std::mutex> lock(result_mutex_);
            last_pose_cov_diag_ = scaledCovariance(kBadCovScale);
        }
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "实时点云下采样后点数太少: %zu < %d", source_points,
                             kMinPointsForRegistration);
        noteLocalRegistrationFailure(cloud_to_register, "insufficient_source_points");
        publishDiagnostics("insufficient_source_points", diagnostic_msgs::msg::DiagnosticStatus::WARN, false, false,
                           0, std::numeric_limits<double>::infinity(), source_points);
        return;
    }

    small_gicp::estimate_covariances_omp(*source_, num_neighbors_, num_threads_);

    Eigen::Isometry3d initial_guess = Eigen::Isometry3d::Identity();
    bool tracking_initialized = false;
    {
        std::lock_guard<std::mutex> lock(result_mutex_);
        initial_guess = previous_result_t_;
        tracking_initialized = tracking_initialized_;
    }

    if (require_initial_pose_for_local_tracking_ && !tracking_initialized) {
        {
            std::lock_guard<std::mutex> lock(result_mutex_);
            last_pose_cov_diag_ = scaledCovariance(kBadCovScale);
        }
        updateRegistrationDebug(std::numeric_limits<double>::infinity(),
                                std::numeric_limits<double>::infinity(),
                                std::numeric_limits<double>::infinity(), 0, false,
                                "waiting_for_initial_pose");
        noteLocalRegistrationFailure(cloud_to_register, "local_tracking_waiting_for_initial_pose");
        publishDiagnostics("local_tracking_waiting_for_initial_pose", diagnostic_msgs::msg::DiagnosticStatus::WARN,
                           false, false, 0, std::numeric_limits<double>::infinity(), source_points);
        return;
    }

    const auto target_view = makeLocalRegistrationTarget(initial_guess);
    if (!target_view.cloud || !target_view.tree) {
        {
            std::lock_guard<std::mutex> lock(result_mutex_);
            last_pose_cov_diag_ = scaledCovariance(kBadCovScale);
        }
        updateRegistrationDebug(std::numeric_limits<double>::infinity(),
                                std::numeric_limits<double>::infinity(),
                                std::numeric_limits<double>::infinity(), target_view.point_count,
                                target_view.local_target_used, target_view.failure_reason);
        noteLocalRegistrationFailure(cloud_to_register, "local_target_insufficient");
        publishDiagnostics("local_target_insufficient", diagnostic_msgs::msg::DiagnosticStatus::WARN, false, false,
                           0, std::numeric_limits<double>::infinity(), source_points);
        return;
    }

    registration_->reduction.num_threads = num_threads_;
    registration_->rejector.max_dist_sq = max_dist_sq_;
    registration_->optimizer.max_iterations = gicp_max_iterations_;

    const auto result = registration_->align(*target_view.cloud, *source_, *target_view.tree, initial_guess);
    const double normalized_error =
        (result.num_inliers > 0) ? (result.error / static_cast<double>(result.num_inliers))
                                 : std::numeric_limits<double>::infinity();
    const Eigen::Vector3d delta_translation = result.T_target_source.translation() - initial_guess.translation();
    const double delta_translation_m = delta_translation.head<2>().norm();
    const double delta_z_m = std::abs(delta_translation.z());
    const double delta_yaw_rad = yawDeltaRad(initial_guess, result.T_target_source);
    const bool jump_gate_ok =
        !registration_jump_gate_enable_ ||
        (delta_translation_m <= max_registration_translation_delta_m_ &&
         delta_z_m <= max_registration_z_delta_m_ &&
         delta_yaw_rad <= max_registration_yaw_delta_rad_);
    std::string rejection_reason = "none";
    if (!result.converged) {
        rejection_reason = "not_converged";
    } else if (result.num_inliers < static_cast<size_t>(min_inliers_)) {
        rejection_reason = "insufficient_inliers";
    } else if (normalized_error > max_normalized_error_) {
        rejection_reason = "high_normalized_error";
    } else if (!jump_gate_ok) {
        rejection_reason = "jump_gate";
    }
    updateRegistrationDebug(delta_translation_m, delta_z_m, delta_yaw_rad, target_view.point_count,
                            target_view.local_target_used, rejection_reason);
    const bool accepted = result.converged && result.num_inliers >= static_cast<size_t>(min_inliers_) &&
                          normalized_error <= max_normalized_error_ && jump_gate_ok;

    if (accepted) {
        const Eigen::Isometry3d accepted_transform =
            registration_smoothing_enable_
                ? interpolateTransform(initial_guess, result.T_target_source, registration_smoothing_alpha_)
                : result.T_target_source;
        {
            std::lock_guard<std::mutex> lock(result_mutex_);
            result_t_ = accepted_transform;
            previous_result_t_ = accepted_transform;
            tracking_initialized_ = true;
            last_pose_cov_diag_ = kPoseCovDiag;
        }
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "局部配准通过: 内点=%zu, 归一化误差=%.4f, 建议跳变=(xy=%.3f,z=%.3f,yaw=%.2fdeg), 平滑alpha=%.2f",
                             result.num_inliers, normalized_error, delta_translation_m, delta_z_m,
                             delta_yaw_rad * kRadToDeg,
                             registration_smoothing_enable_ ? registration_smoothing_alpha_ : 1.0);
        {
            std::lock_guard<std::mutex> lock(online_relocalization_mutex_);
            consecutive_registration_failures_ = 0;
            if (online_relocalization_enable_ && !online_relocalization_running_) {
                online_relocalization_state_ = "tracking";
                online_relocalization_reason_ = "local_registration_ok";
            }
        }
        publishDiagnostics("local_registration_ok", diagnostic_msgs::msg::DiagnosticStatus::OK, result.converged,
                           true, result.num_inliers, normalized_error, source_points);
    } else {
        {
            std::lock_guard<std::mutex> lock(result_mutex_);
            last_pose_cov_diag_ = scaledCovariance(kBadCovScale);
        }
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "局部配准未通过，map->odom 暂不更新: reason=%s, 收敛=%s, 内点=%zu/%d, 归一化误差=%.4f/%.4f, "
                             "跳变=(xy=%.3f/%.3f,z=%.3f/%.3f,yaw=%.2f/%.2fdeg), target=%zu",
                             rejection_reason.c_str(),
                             result.converged ? "是" : "否", result.num_inliers, min_inliers_, normalized_error,
                             max_normalized_error_, delta_translation_m, max_registration_translation_delta_m_,
                             delta_z_m, max_registration_z_delta_m_, delta_yaw_rad * kRadToDeg,
                             max_registration_yaw_delta_rad_ * kRadToDeg, target_view.point_count);
        noteLocalRegistrationFailure(cloud_to_register, "local_registration_frozen");
        publishDiagnostics("local_registration_frozen", diagnostic_msgs::msg::DiagnosticStatus::WARN,
                           result.converged, false, result.num_inliers, normalized_error, source_points);
    }
}

void LocalizationNode::noteLocalRegistrationFailure(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_to_register,
    const std::string& reason) {
    if (!online_relocalization_enable_) {
        return;
    }

    bool should_try = false;
    {
        std::lock_guard<std::mutex> lock(online_relocalization_mutex_);
        ++consecutive_registration_failures_;
        online_relocalization_reason_ = reason;
        if (online_relocalization_running_) {
            online_relocalization_state_ = "running";
            return;
        }
        if (consecutive_registration_failures_ >= online_relocalization_trigger_after_failures_) {
            should_try = true;
        } else {
            online_relocalization_state_ = "waiting_for_failures";
        }
    }

    if (should_try) {
        maybeStartOnlineRelocalization(cloud_to_register, reason);
    }
}

void LocalizationNode::maybeStartOnlineRelocalization(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_to_register,
    const std::string& reason) {
    if (!online_relocalization_enable_ || !cloud_to_register || cloud_to_register->empty()) {
        return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr trigger_cloud(new pcl::PointCloud<pcl::PointXYZ>(*cloud_to_register));
    finalizeCloud(*trigger_cloud);
    const auto now = std::chrono::steady_clock::now();
    int attempt_snapshot = 0;
    int max_attempts_snapshot = online_relocalization_max_attempts_;
    {
        std::lock_guard<std::mutex> lock(online_relocalization_mutex_);
        if (online_relocalization_running_) {
            online_relocalization_state_ = "running";
            online_relocalization_reason_ = reason;
            return;
        }
        if (consecutive_registration_failures_ < online_relocalization_trigger_after_failures_) {
            online_relocalization_state_ = "waiting_for_failures";
            online_relocalization_reason_ = reason;
            return;
        }
        if (online_relocalization_attempts_ >= online_relocalization_max_attempts_) {
            online_relocalization_state_ = "exhausted";
            online_relocalization_reason_ = reason;
            return;
        }

        const auto cooldown_elapsed =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                now - last_online_relocalization_attempt_wall_)
                .count();
        if (cooldown_elapsed < online_relocalization_cooldown_ms_) {
            online_relocalization_state_ = "cooldown";
            online_relocalization_reason_ = reason;
            return;
        }

        if (online_relocalization_worker_.joinable()) {
            online_relocalization_worker_.join();
        }

        ++online_relocalization_attempts_;
        attempt_snapshot = online_relocalization_attempts_;
        max_attempts_snapshot = online_relocalization_max_attempts_;
        online_relocalization_state_ = "collecting";
        online_relocalization_reason_ = reason;
        last_relocalization_source_ = "auto:" + reason;
        last_online_relocalization_attempt_wall_ = now;
        online_relocalization_cancel_requested_ = false;
        online_relocalization_running_ = true;
        online_relocalization_worker_ = std::thread(&LocalizationNode::runOnlineRelocalization, this,
                                                    trigger_cloud, reason);
    }

    RCLCPP_WARN(this->get_logger(),
                "实验性在线重定位已触发: reason=%s, failure_count>=%d, attempt=%d/%d",
                reason.c_str(), online_relocalization_trigger_after_failures_,
                attempt_snapshot, max_attempts_snapshot);
    publishDiagnostics("online_relocalization_running", diagnostic_msgs::msg::DiagnosticStatus::OK, false, false,
                       0, std::numeric_limits<double>::infinity(), trigger_cloud->size());
}

void LocalizationNode::runOnlineRelocalization(pcl::PointCloud<pcl::PointXYZ>::Ptr trigger_cloud,
                                               std::string trigger_reason) {
    const auto collect_deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(online_relocalization_collect_ms_);
    while (!online_relocalization_stop_ && !online_relocalization_cancel_requested_ &&
           std::chrono::steady_clock::now() < collect_deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    if (online_relocalization_stop_ || online_relocalization_cancel_requested_) {
        {
            std::lock_guard<std::mutex> lock(online_relocalization_mutex_);
            online_relocalization_state_ = online_relocalization_stop_ ? "stopped" : "cancelled_by_initialpose";
            online_relocalization_reason_ = trigger_reason;
        }
        online_relocalization_running_ = false;
        return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    if (trigger_cloud) {
        *source_cloud += *trigger_cloud;
    }
    {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        if (accumulated_cloud_ && !accumulated_cloud_->empty()) {
            *source_cloud += *accumulated_cloud_;
        }
    }
    finalizeCloud(*source_cloud);

    {
        std::lock_guard<std::mutex> lock(online_relocalization_mutex_);
        online_relocalization_state_ = "running";
        online_relocalization_reason_ = trigger_reason;
    }
    publishDiagnostics("online_relocalization_running", diagnostic_msgs::msg::DiagnosticStatus::OK, false, false,
                       0, std::numeric_limits<double>::infinity(), source_cloud->size());

    (void)performOnlineRelocalization(source_cloud, trigger_reason);
    online_relocalization_running_ = false;
}

bool LocalizationNode::performOnlineRelocalization(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_to_register,
    const std::string& trigger_reason) {
    auto fail = [&](const std::string& state, bool converged, size_t inliers,
                    double normalized_error, size_t source_points) {
        {
            std::lock_guard<std::mutex> lock(online_relocalization_mutex_);
            online_relocalization_state_ = state;
            online_relocalization_reason_ = trigger_reason;
        }
        if (state != "cancelled_by_initialpose") {
            std::lock_guard<std::mutex> lock(result_mutex_);
            last_pose_cov_diag_ = scaledCovariance(kBadCovScale);
        }
        publishDiagnostics("online_relocalization_failed", diagnostic_msgs::msg::DiagnosticStatus::WARN,
                           converged, false, inliers, normalized_error, source_points);
        return false;
    };

    if (online_relocalization_stop_ || online_relocalization_cancel_requested_) {
        return fail("cancelled_by_initialpose", false, 0, std::numeric_limits<double>::infinity(), 0);
    }

    if (!cloud_to_register || cloud_to_register->empty()) {
        return fail("failed_empty_source", false, 0, std::numeric_limits<double>::infinity(), 0);
    }

    if (!prepareStartupTarget() || !prepareTargetMap()) {
        return fail("failed_target_not_ready", false, 0, std::numeric_limits<double>::infinity(),
                    cloud_to_register->size());
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr clean_source(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::Indices indices;
    pcl::removeNaNFromPointCloud(*cloud_to_register, *clean_source, indices);
    finalizeCloud(*clean_source);

    pcl::PointCloud<pcl::PointXYZ>::Ptr online_source(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setLeafSize(static_cast<float>(online_relocalization_leaf_size_),
                      static_cast<float>(online_relocalization_leaf_size_),
                      static_cast<float>(online_relocalization_leaf_size_));
    voxel.setInputCloud(clean_source);
    voxel.filter(*online_source);
    finalizeCloud(*online_source);

    const size_t online_source_points = online_source->size();
    if (online_source_points < static_cast<size_t>(kMinPointsForRegistration)) {
        return fail("failed_insufficient_source", false, 0, std::numeric_limits<double>::infinity(),
                    online_source_points);
    }

    const auto source_fpfh = computeFpfh(online_source);
    if (!source_fpfh || source_fpfh->empty()) {
        return fail("failed_source_fpfh", false, 0, std::numeric_limits<double>::infinity(),
                    online_source_points);
    }

    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia;
    sac_ia.setInputSource(online_source);
    sac_ia.setSourceFeatures(source_fpfh);
    sac_ia.setInputTarget(startup_target_);
    sac_ia.setTargetFeatures(startup_target_fpfh_);
    sac_ia.setMinSampleDistance(kSacIaMinSampleDistance);
    sac_ia.setCorrespondenceRandomness(kSacIaCorrespondenceRandomness);
    sac_ia.setNumberOfSamples(kSacIaNumSamples);

    pcl::PointCloud<pcl::PointXYZ> sac_aligned;
    sac_ia.align(sac_aligned);
    if (!sac_ia.hasConverged()) {
        return fail("failed_sac_ia", false, 0, std::numeric_limits<double>::infinity(), online_source_points);
    }

    auto source_gicp = small_gicp::voxelgrid_sampling_omp<pcl::PointCloud<pcl::PointXYZ>,
                                                          pcl::PointCloud<pcl::PointCovariance>>(
        *clean_source, registered_leaf_size_);
    const size_t source_points = source_gicp ? source_gicp->size() : 0U;
    if (!source_gicp || source_points < static_cast<size_t>(kMinPointsForRegistration)) {
        return fail("failed_insufficient_gicp_source", false, 0,
                    std::numeric_limits<double>::infinity(), source_points);
    }

    small_gicp::estimate_covariances_omp(*source_gicp, num_neighbors_, num_threads_);

    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP> registration;
    registration.reduction.num_threads = num_threads_;
    registration.rejector.max_dist_sq = max_dist_sq_;
    registration.optimizer.max_iterations = gicp_max_iterations_;

    const Eigen::Isometry3d initial_guess = toIsometry3d(sac_ia.getFinalTransformation());
    const auto result = registration.align(*target_, *source_gicp, *target_tree_, initial_guess);
    const double normalized_error =
        (result.num_inliers > 0) ? (result.error / static_cast<double>(result.num_inliers))
                                 : std::numeric_limits<double>::infinity();
    const bool accepted = result.converged && result.num_inliers >= static_cast<size_t>(min_inliers_) &&
                          normalized_error <= max_normalized_error_;

    if (!accepted) {
        RCLCPP_WARN(this->get_logger(),
                    "实验性在线重定位未通过: SAC-IA 分数=%.4f, 收敛=%s, 内点=%zu/%d, 归一化误差=%.4f/%.4f",
                    sac_ia.getFitnessScore(), result.converged ? "是" : "否", result.num_inliers, min_inliers_,
                    normalized_error, max_normalized_error_);
        return fail("failed_gicp_quality", result.converged, result.num_inliers, normalized_error, source_points);
    }

    if (online_relocalization_stop_ || online_relocalization_cancel_requested_) {
        return fail("cancelled_by_initialpose", false, result.num_inliers, normalized_error, source_points);
    }

    {
        std::lock_guard<std::mutex> lock(result_mutex_);
        pending_online_relocalization_t_ = result.T_target_source;
        pending_online_relocalization_cov_diag_ = kPoseCovDiag;
        pending_online_relocalization_result_ = true;
    }
    {
        std::lock_guard<std::mutex> lock(online_relocalization_mutex_);
        consecutive_registration_failures_ = 0;
        online_relocalization_state_ = "succeeded";
        online_relocalization_reason_ = trigger_reason;
    }

    RCLCPP_WARN(this->get_logger(),
                "实验性在线重定位成功: trigger=%s, SAC-IA 分数=%.4f, 内点=%zu, 归一化误差=%.4f",
                trigger_reason.c_str(), sac_ia.getFitnessScore(), result.num_inliers, normalized_error);
    publishDiagnostics("online_relocalization_ok", diagnostic_msgs::msg::DiagnosticStatus::OK, result.converged,
                       true, result.num_inliers, normalized_error, source_points);
    return true;
}

bool LocalizationNode::performStartupRelocalization(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_to_register) {
    if (!cloud_to_register || cloud_to_register->empty()) {
        startup_relocalization_state_ = "failed_empty_source";
        {
            std::lock_guard<std::mutex> lock(result_mutex_);
            last_pose_cov_diag_ = scaledCovariance(kBadCovScale);
        }
        publishDiagnostics("startup_relocalization_failed", diagnostic_msgs::msg::DiagnosticStatus::WARN, false, false,
                           0, std::numeric_limits<double>::infinity(), 0);
        return false;
    }

    if (!prepareStartupTarget() || !prepareTargetMap()) {
        startup_relocalization_state_ = "failed_target_not_ready";
        {
            std::lock_guard<std::mutex> lock(result_mutex_);
            last_pose_cov_diag_ = scaledCovariance(kBadCovScale);
        }
        publishDiagnostics("startup_relocalization_failed", diagnostic_msgs::msg::DiagnosticStatus::WARN, false, false,
                           0, std::numeric_limits<double>::infinity(), cloud_to_register->size());
        return false;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr clean_source(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::Indices indices;
    pcl::removeNaNFromPointCloud(*cloud_to_register, *clean_source, indices);
    finalizeCloud(*clean_source);

    pcl::PointCloud<pcl::PointXYZ>::Ptr startup_source(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setLeafSize(static_cast<float>(startup_leaf_size_), static_cast<float>(startup_leaf_size_),
                      static_cast<float>(startup_leaf_size_));
    voxel.setInputCloud(clean_source);
    voxel.filter(*startup_source);
    finalizeCloud(*startup_source);

    const size_t startup_source_points = startup_source->size();
    if (startup_source_points < static_cast<size_t>(kMinPointsForRegistration)) {
        startup_relocalization_state_ = "failed_insufficient_source";
        {
            std::lock_guard<std::mutex> lock(result_mutex_);
            last_pose_cov_diag_ = scaledCovariance(kBadCovScale);
        }
        publishDiagnostics("startup_relocalization_failed", diagnostic_msgs::msg::DiagnosticStatus::WARN, false, false,
                           0, std::numeric_limits<double>::infinity(), startup_source_points);
        return false;
    }

    if (startup_global_grid_enable_ && performStartupGridRelocalization(clean_source, startup_source)) {
        return true;
    }

    if (startup_global_grid_enable_ && !startup_global_grid_sac_fallback_) {
        startup_relocalization_state_ = "failed_grid_quality";
        {
            std::lock_guard<std::mutex> lock(result_mutex_);
            last_pose_cov_diag_ = scaledCovariance(kBadCovScale);
        }
        publishDiagnostics("startup_relocalization_failed", diagnostic_msgs::msg::DiagnosticStatus::WARN,
                           false, false, 0, std::numeric_limits<double>::infinity(), startup_source_points);
        return false;
    }

    const auto source_fpfh = computeFpfh(startup_source);
    if (!source_fpfh || source_fpfh->empty()) {
        startup_relocalization_state_ = "failed_source_fpfh";
        {
            std::lock_guard<std::mutex> lock(result_mutex_);
            last_pose_cov_diag_ = scaledCovariance(kBadCovScale);
        }
        publishDiagnostics("startup_relocalization_failed", diagnostic_msgs::msg::DiagnosticStatus::WARN, false, false,
                           0, std::numeric_limits<double>::infinity(), startup_source_points);
        return false;
    }

    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia;
    sac_ia.setInputSource(startup_source);
    sac_ia.setSourceFeatures(source_fpfh);
    sac_ia.setInputTarget(startup_target_);
    sac_ia.setTargetFeatures(startup_target_fpfh_);
    sac_ia.setMinSampleDistance(kSacIaMinSampleDistance);
    sac_ia.setCorrespondenceRandomness(kSacIaCorrespondenceRandomness);
    sac_ia.setNumberOfSamples(kSacIaNumSamples);

    pcl::PointCloud<pcl::PointXYZ> sac_aligned;
    sac_ia.align(sac_aligned);
    if (!sac_ia.hasConverged()) {
        startup_relocalization_state_ = "failed_sac_ia";
        {
            std::lock_guard<std::mutex> lock(result_mutex_);
            last_pose_cov_diag_ = scaledCovariance(kBadCovScale);
        }
        publishDiagnostics("startup_relocalization_failed", diagnostic_msgs::msg::DiagnosticStatus::WARN, false, false,
                           0, std::numeric_limits<double>::infinity(), startup_source_points);
        return false;
    }

    source_ = small_gicp::voxelgrid_sampling_omp<pcl::PointCloud<pcl::PointXYZ>,
                                                 pcl::PointCloud<pcl::PointCovariance>>(*clean_source,
                                                                                        registered_leaf_size_);
    const size_t source_points = source_ ? source_->size() : 0U;
    if (!source_ || source_points < static_cast<size_t>(kMinPointsForRegistration)) {
        startup_relocalization_state_ = "failed_insufficient_gicp_source";
        {
            std::lock_guard<std::mutex> lock(result_mutex_);
            last_pose_cov_diag_ = scaledCovariance(kBadCovScale);
        }
        publishDiagnostics("startup_relocalization_failed", diagnostic_msgs::msg::DiagnosticStatus::WARN, false, false,
                           0, std::numeric_limits<double>::infinity(), source_points);
        return false;
    }

    small_gicp::estimate_covariances_omp(*source_, num_neighbors_, num_threads_);

    registration_->reduction.num_threads = num_threads_;
    registration_->rejector.max_dist_sq = max_dist_sq_;
    registration_->optimizer.max_iterations = gicp_max_iterations_;

    const Eigen::Isometry3d initial_guess = toIsometry3d(sac_ia.getFinalTransformation());
    const auto result = registration_->align(*target_, *source_, *target_tree_, initial_guess);
    const double normalized_error =
        (result.num_inliers > 0) ? (result.error / static_cast<double>(result.num_inliers))
                                 : std::numeric_limits<double>::infinity();
    const bool accepted = result.converged && result.num_inliers >= static_cast<size_t>(min_inliers_) &&
                          normalized_error <= max_normalized_error_;

    if (!accepted) {
        startup_relocalization_state_ = "failed_gicp_quality";
        {
            std::lock_guard<std::mutex> lock(result_mutex_);
            last_pose_cov_diag_ = scaledCovariance(kBadCovScale);
        }
        RCLCPP_WARN(this->get_logger(),
                    "开局重定位未通过: SAC-IA 分数=%.4f, 收敛=%s, 内点=%zu/%d, 归一化误差=%.4f/%.4f",
                    sac_ia.getFitnessScore(), result.converged ? "是" : "否", result.num_inliers, min_inliers_,
                    normalized_error, max_normalized_error_);
        publishDiagnostics("startup_relocalization_failed", diagnostic_msgs::msg::DiagnosticStatus::WARN,
                           result.converged, false, result.num_inliers, normalized_error, source_points);
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(result_mutex_);
        result_t_ = result.T_target_source;
        previous_result_t_ = result.T_target_source;
        tracking_initialized_ = true;
        last_pose_cov_diag_ = kPoseCovDiag;
        startup_relocalization_state_ = "ok";
    }

    RCLCPP_INFO(this->get_logger(),
                "开局重定位成功: SAC-IA 分数=%.4f, 内点=%zu, 归一化误差=%.4f",
                sac_ia.getFitnessScore(), result.num_inliers, normalized_error);
    publishDiagnostics("startup_relocalization_ok", diagnostic_msgs::msg::DiagnosticStatus::OK, result.converged,
                       true, result.num_inliers, normalized_error, source_points);
    return true;
}

bool LocalizationNode::performStartupGridRelocalization(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& clean_source,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& startup_source) {
    if (!clean_source || clean_source->empty() || !startup_source || startup_source->empty() ||
        !global_map_ || global_map_->empty() || !prepareTargetMap()) {
        return false;
    }

    struct Cell {
        int x{0};
        int y{0};
    };
    struct Candidate {
        Eigen::Isometry3d guess{Eigen::Isometry3d::Identity()};
        int votes{0};
        double overlap_ratio{0.0};
        double yaw{0.0};
    };
    auto cellKey = [](int x, int y) {
        return (static_cast<int64_t>(x) << 32) ^ (static_cast<int64_t>(y) & 0xffffffffLL);
    };

    double min_x = std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    for (const auto& p : global_map_->points) {
        min_x = std::min(min_x, static_cast<double>(p.x));
        min_y = std::min(min_y, static_cast<double>(p.y));
        max_x = std::max(max_x, static_cast<double>(p.x));
        max_y = std::max(max_y, static_cast<double>(p.y));
    }
    if (!std::isfinite(min_x) || !std::isfinite(min_y) || max_x <= min_x || max_y <= min_y) {
        return false;
    }

    const double resolution = startup_global_grid_resolution_;
    const int width = static_cast<int>(std::ceil((max_x - min_x) / resolution)) + 1;
    const int height = static_cast<int>(std::ceil((max_y - min_y) / resolution)) + 1;
    if (width <= 0 || height <= 0) {
        return false;
    }

    std::unordered_set<int64_t> target_set;
    target_set.reserve(global_map_->size());
    std::vector<Cell> target_cells;
    target_cells.reserve(global_map_->size());
    for (const auto& p : global_map_->points) {
        const int cx = static_cast<int>(std::floor((static_cast<double>(p.x) - min_x) / resolution));
        const int cy = static_cast<int>(std::floor((static_cast<double>(p.y) - min_y) / resolution));
        if (cx < 0 || cy < 0 || cx >= width || cy >= height) {
            continue;
        }
        const int64_t key = cellKey(cx, cy);
        if (target_set.insert(key).second) {
            target_cells.push_back(Cell{cx, cy});
        }
    }
    if (target_cells.empty()) {
        return false;
    }

    std::vector<Eigen::Vector2d> source_xy;
    source_xy.reserve(startup_source->size());
    for (const auto& p : startup_source->points) {
        if (std::isfinite(p.x) && std::isfinite(p.y)) {
            source_xy.emplace_back(static_cast<double>(p.x), static_cast<double>(p.y));
        }
    }
    if (source_xy.size() < static_cast<size_t>(kMinPointsForRegistration)) {
        return false;
    }

    std::vector<Candidate> candidates;
    const double yaw_step_rad = startup_global_grid_yaw_step_deg_ * M_PI / 180.0;
    const int yaw_steps = std::max(1, static_cast<int>(std::round((2.0 * M_PI) / yaw_step_rad)));
    const int max_votes_to_keep = std::max(startup_global_grid_max_candidates_ * 3, startup_global_grid_max_candidates_);

    for (int yaw_index = 0; yaw_index < yaw_steps; ++yaw_index) {
        const double yaw = -M_PI + yaw_index * (2.0 * M_PI / static_cast<double>(yaw_steps));
        const double c = std::cos(yaw);
        const double s = std::sin(yaw);

        std::unordered_set<int64_t> source_rotated_set;
        source_rotated_set.reserve(source_xy.size());
        std::vector<Cell> source_cells;
        source_cells.reserve(source_xy.size());
        for (const auto& p : source_xy) {
            const double rx = c * p.x() - s * p.y();
            const double ry = s * p.x() + c * p.y();
            const int cx = static_cast<int>(std::floor(rx / resolution));
            const int cy = static_cast<int>(std::floor(ry / resolution));
            const int64_t key = cellKey(cx, cy);
            if (source_rotated_set.insert(key).second) {
                source_cells.push_back(Cell{cx, cy});
            }
        }
        if (source_cells.empty()) {
            continue;
        }

        std::unordered_map<int64_t, int> votes;
        votes.reserve(target_cells.size() * 2);
        for (const auto& target_cell : target_cells) {
            for (const auto& source_cell : source_cells) {
                const int dx = target_cell.x - source_cell.x;
                const int dy = target_cell.y - source_cell.y;
                ++votes[cellKey(dx, dy)];
            }
        }

        std::vector<std::pair<int64_t, int>> ranked_votes;
        ranked_votes.reserve(votes.size());
        for (const auto& item : votes) {
            ranked_votes.push_back(item);
        }
        const int keep = std::min<int>(max_votes_to_keep, ranked_votes.size());
        std::partial_sort(ranked_votes.begin(), ranked_votes.begin() + keep, ranked_votes.end(),
                          [](const auto& a, const auto& b) { return a.second > b.second; });

        for (int i = 0; i < keep; ++i) {
            const int64_t key = ranked_votes[i].first;
            const int dx = static_cast<int>(key >> 32);
            const int dy = static_cast<int>(key & 0xffffffffLL);
            const int votes_count = ranked_votes[i].second;
            const double overlap_ratio =
                static_cast<double>(votes_count) / static_cast<double>(std::max<size_t>(1, source_cells.size()));
            if (overlap_ratio < startup_global_grid_min_overlap_ratio_) {
                continue;
            }

            Eigen::Isometry3d guess = Eigen::Isometry3d::Identity();
            guess.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
            guess.translation().x() = min_x + (static_cast<double>(dx) + 0.5) * resolution;
            guess.translation().y() = min_y + (static_cast<double>(dy) + 0.5) * resolution;
            guess.translation().z() = 0.0;
            candidates.push_back(Candidate{guess, votes_count, overlap_ratio, yaw});
        }
    }

    if (candidates.empty()) {
        {
            std::lock_guard<std::mutex> lock(diagnostics_mutex_);
            last_startup_grid_candidates_ = 0;
            last_startup_grid_best_votes_ = 0;
            last_startup_grid_best_overlap_ = 0.0;
            last_startup_grid_best_x_ = std::numeric_limits<double>::infinity();
            last_startup_grid_best_y_ = std::numeric_limits<double>::infinity();
            last_startup_grid_best_z_ = std::numeric_limits<double>::infinity();
            last_startup_grid_best_yaw_rad_ = std::numeric_limits<double>::infinity();
            last_startup_grid_best_converged_ = false;
            last_startup_grid_best_inliers_ = 0;
            last_startup_grid_best_normalized_error_ = std::numeric_limits<double>::infinity();
            last_startup_grid_rejection_reason_ = "no_candidates";
        }
        RCLCPP_WARN(this->get_logger(), "开局全局栅格重定位未找到候选");
        return false;
    }

    std::sort(candidates.begin(), candidates.end(), [](const Candidate& a, const Candidate& b) {
        return a.votes > b.votes;
    });
    if (candidates.size() > static_cast<size_t>(startup_global_grid_max_candidates_)) {
        candidates.resize(static_cast<size_t>(startup_global_grid_max_candidates_));
    }

    auto source_gicp = small_gicp::voxelgrid_sampling_omp<pcl::PointCloud<pcl::PointXYZ>,
                                                          pcl::PointCloud<pcl::PointCovariance>>(
        *clean_source, registered_leaf_size_);
    const size_t source_points = source_gicp ? source_gicp->size() : 0U;
    if (!source_gicp || source_points < static_cast<size_t>(kMinPointsForRegistration)) {
        return false;
    }
    small_gicp::estimate_covariances_omp(*source_gicp, num_neighbors_, num_threads_);

    struct ResultView {
        Candidate candidate;
        Eigen::Isometry3d transform{Eigen::Isometry3d::Identity()};
        bool converged{false};
        size_t inliers{0};
        double normalized_error{std::numeric_limits<double>::infinity()};
    };
    ResultView best;

    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP> registration;
    registration.reduction.num_threads = num_threads_;
    registration.rejector.max_dist_sq = max_dist_sq_;
    registration.optimizer.max_iterations = gicp_max_iterations_;

    for (const auto& candidate : candidates) {
        const auto result = registration.align(*target_, *source_gicp, *target_tree_, candidate.guess);
        const double normalized_error =
            (result.num_inliers > 0) ? (result.error / static_cast<double>(result.num_inliers))
                                     : std::numeric_limits<double>::infinity();
        if (normalized_error < best.normalized_error) {
            best.candidate = candidate;
            best.transform = result.T_target_source;
            best.converged = result.converged;
            best.inliers = result.num_inliers;
            best.normalized_error = normalized_error;
        }
    }

    std::string grid_rejection_reason = "none";
    if (!best.converged) {
        grid_rejection_reason = "not_converged";
    } else if (best.inliers < static_cast<size_t>(min_inliers_)) {
        grid_rejection_reason = "insufficient_inliers";
    } else if (best.normalized_error > startup_global_grid_max_normalized_error_) {
        grid_rejection_reason = "high_normalized_error";
    }
    const double best_yaw_rad = std::atan2(best.transform.rotation()(1, 0), best.transform.rotation()(0, 0));
    {
        std::lock_guard<std::mutex> lock(diagnostics_mutex_);
        last_startup_grid_candidates_ = candidates.size();
        last_startup_grid_best_votes_ = best.candidate.votes;
        last_startup_grid_best_overlap_ = best.candidate.overlap_ratio;
        last_startup_grid_best_x_ = best.transform.translation().x();
        last_startup_grid_best_y_ = best.transform.translation().y();
        last_startup_grid_best_z_ = best.transform.translation().z();
        last_startup_grid_best_yaw_rad_ = best_yaw_rad;
        last_startup_grid_best_converged_ = best.converged;
        last_startup_grid_best_inliers_ = best.inliers;
        last_startup_grid_best_normalized_error_ = best.normalized_error;
        last_startup_grid_rejection_reason_ = grid_rejection_reason;
    }

    if (best.converged && best.inliers >= static_cast<size_t>(min_inliers_) &&
        best.normalized_error <= startup_global_grid_max_normalized_error_) {
        {
            std::lock_guard<std::mutex> lock(result_mutex_);
            result_t_ = best.transform;
            previous_result_t_ = best.transform;
            tracking_initialized_ = true;
            last_pose_cov_diag_ = kPoseCovDiag;
            startup_relocalization_state_ = "ok_grid";
        }
        RCLCPP_INFO(this->get_logger(),
                    "开局全局栅格重定位成功: candidates=%zu, votes=%d, overlap=%.3f, "
                    "candidate_yaw=%.1fdeg, pose=(%.3f, %.3f, %.3f, %.1fdeg), 内点=%zu, 归一化误差=%.4f",
                    candidates.size(), best.candidate.votes, best.candidate.overlap_ratio,
                    best.candidate.yaw * kRadToDeg, best.transform.translation().x(),
                    best.transform.translation().y(), best.transform.translation().z(),
                    best_yaw_rad * kRadToDeg, best.inliers, best.normalized_error);
        publishDiagnostics("startup_relocalization_ok", diagnostic_msgs::msg::DiagnosticStatus::OK,
                           best.converged, true, best.inliers, best.normalized_error, source_points);
        return true;
    }

    RCLCPP_WARN(this->get_logger(),
                "开局全局栅格重定位未通过: candidates=%zu, best_votes=%d, best_overlap=%.3f, best_yaw=%.1fdeg, "
                "best_pose=(%.3f, %.3f, %.3f, %.1fdeg), reason=%s, 收敛=%s, 内点=%zu/%d, 归一化误差=%.4f/%.4f",
                candidates.size(), best.candidate.votes, best.candidate.overlap_ratio,
                best.candidate.yaw * kRadToDeg, best.transform.translation().x(),
                best.transform.translation().y(), best.transform.translation().z(),
                best_yaw_rad * kRadToDeg, grid_rejection_reason.c_str(),
                best.converged ? "是" : "否", best.inliers, min_inliers_, best.normalized_error,
                startup_global_grid_max_normalized_error_);
    return false;
}

void LocalizationNode::consumePendingOnlineRelocalizationResult() {
    bool consumed = false;
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();
    {
        std::lock_guard<std::mutex> lock(result_mutex_);
        if (!pending_online_relocalization_result_) {
            return;
        }
        result_t_ = pending_online_relocalization_t_;
        previous_result_t_ = pending_online_relocalization_t_;
        tracking_initialized_ = true;
        last_pose_cov_diag_ = pending_online_relocalization_cov_diag_;
        pending_online_relocalization_result_ = false;
        translation = result_t_.translation();
        consumed = true;
    }

    if (consumed) {
        RCLCPP_WARN(this->get_logger(),
                    "实验性在线重定位结果已接管 map->odom: translation=[%.3f, %.3f, %.3f]",
                    translation.x(), translation.y(), translation.z());
    }
}

void LocalizationNode::publishTransform() {
    if (!map_loaded_) {
        return;
    }

    consumePendingOnlineRelocalizationResult();

    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    std::array<double, 6> cov_diag{};
    {
        std::lock_guard<std::mutex> lock(result_mutex_);
        pose = result_t_;
        cov_diag = last_pose_cov_diag_;
    }

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->now();
    transform.header.frame_id = map_frame_;
    transform.child_frame_id = odom_frame_;
    transform.transform.translation.x = pose.translation().x();
    transform.transform.translation.y = pose.translation().y();
    transform.transform.translation.z = pose.translation().z();

    const Eigen::Quaterniond q(pose.rotation());
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(transform);

    publishPoseWithCov(pose, cov_diag);
}

void LocalizationNode::publishPoseWithCov(const Eigen::Isometry3d& pose,
                                          const std::array<double, 6>& covariance_diag) {
    if (!pose_cov_pub_) {
        return;
    }

    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = map_frame_;
    msg.pose.pose.position.x = pose.translation().x();
    msg.pose.pose.position.y = pose.translation().y();
    msg.pose.pose.position.z = pose.translation().z();

    const Eigen::Quaterniond q(pose.rotation());
    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();
    msg.pose.pose.orientation.w = q.w();

    msg.pose.covariance.fill(0.0);
    for (size_t i = 0; i < covariance_diag.size(); ++i) {
        msg.pose.covariance[i * 6 + i] = covariance_diag[i];
    }
    pose_cov_pub_->publish(msg);
}

void LocalizationNode::publishDiagnostics(const std::string& reason, uint8_t level, bool converged, bool accepted,
                                          size_t inliers, double normalized_error, size_t source_points) {
    if (!diag_pub_) {
        return;
    }

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = this->now();

    std::string online_state;
    std::string online_reason;
    std::string last_relocalization_source;
    int online_attempts = 0;
    {
        std::lock_guard<std::mutex> lock(online_relocalization_mutex_);
        online_state = online_relocalization_state_;
        online_reason = online_relocalization_reason_;
        last_relocalization_source = last_relocalization_source_;
        online_attempts = online_relocalization_attempts_;
    }
    bool tracking_initialized = false;
    {
        std::lock_guard<std::mutex> lock(result_mutex_);
        tracking_initialized = tracking_initialized_;
    }
    double delta_translation_m = std::numeric_limits<double>::infinity();
    double delta_z_m = std::numeric_limits<double>::infinity();
    double delta_yaw_rad = std::numeric_limits<double>::infinity();
    size_t active_target_points = 0;
    bool local_target_used = false;
    std::string registration_rejection_reason;
    size_t startup_grid_candidates = 0;
    int startup_grid_best_votes = 0;
    double startup_grid_best_overlap = 0.0;
    double startup_grid_best_x = std::numeric_limits<double>::infinity();
    double startup_grid_best_y = std::numeric_limits<double>::infinity();
    double startup_grid_best_z = std::numeric_limits<double>::infinity();
    double startup_grid_best_yaw_rad = std::numeric_limits<double>::infinity();
    bool startup_grid_best_converged = false;
    size_t startup_grid_best_inliers = 0;
    double startup_grid_best_normalized_error = std::numeric_limits<double>::infinity();
    std::string startup_grid_rejection_reason;
    {
        std::lock_guard<std::mutex> lock(diagnostics_mutex_);
        delta_translation_m = last_registration_delta_translation_m_;
        delta_z_m = last_registration_delta_z_m_;
        delta_yaw_rad = last_registration_delta_yaw_rad_;
        active_target_points = last_active_target_points_;
        local_target_used = last_local_target_used_;
        registration_rejection_reason = last_registration_rejection_reason_;
        startup_grid_candidates = last_startup_grid_candidates_;
        startup_grid_best_votes = last_startup_grid_best_votes_;
        startup_grid_best_overlap = last_startup_grid_best_overlap_;
        startup_grid_best_x = last_startup_grid_best_x_;
        startup_grid_best_y = last_startup_grid_best_y_;
        startup_grid_best_z = last_startup_grid_best_z_;
        startup_grid_best_yaw_rad = last_startup_grid_best_yaw_rad_;
        startup_grid_best_converged = last_startup_grid_best_converged_;
        startup_grid_best_inliers = last_startup_grid_best_inliers_;
        startup_grid_best_normalized_error = last_startup_grid_best_normalized_error_;
        startup_grid_rejection_reason = last_startup_grid_rejection_reason_;
    }

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "rc26_localization";
    status.hardware_id = "R2";
    status.level = level;
    status.message = reason;

    auto add = [&status](const std::string& key, const std::string& value) {
        diagnostic_msgs::msg::KeyValue kv;
        kv.key = key;
        kv.value = value;
        status.values.push_back(std::move(kv));
    };

    add("accepted", boolText(accepted));
    add("converged", boolText(converged));
    add("human_message", diagnosticHumanMessage(reason, startup_relocalization_state_, online_state));
    add("inliers", std::to_string(inliers));
    add("min_inliers", std::to_string(min_inliers_));
    add("source_points", std::to_string(source_points));
    add("tracking_initialized", boolText(tracking_initialized));
    add("max_normalized_error", std::to_string(max_normalized_error_));
    add("normalized_error", std::isfinite(normalized_error) ? std::to_string(normalized_error) : "inf");
    add("active_target_points", std::to_string(active_target_points));
    add("local_target_used", boolText(local_target_used));
    add("registration_rejection_reason", registration_rejection_reason);
    add("registration_delta_translation_m",
        std::isfinite(delta_translation_m) ? std::to_string(delta_translation_m) : "inf");
    add("registration_delta_z_m", std::isfinite(delta_z_m) ? std::to_string(delta_z_m) : "inf");
    add("registration_delta_yaw_rad", std::isfinite(delta_yaw_rad) ? std::to_string(delta_yaw_rad) : "inf");
    add("max_registration_translation_delta_m", std::to_string(max_registration_translation_delta_m_));
    add("max_registration_z_delta_m", std::to_string(max_registration_z_delta_m_));
    add("max_registration_yaw_delta_rad", std::to_string(max_registration_yaw_delta_rad_));
    add("registration_smoothing_enable", boolText(registration_smoothing_enable_));
    add("registration_smoothing_alpha", std::to_string(registration_smoothing_alpha_));
    add("map_loaded", boolText(map_loaded_));
    add("target_ready", boolText(target_ready_));
    add("startup_relocalization", startup_relocalization_state_);
    add("startup_grid_candidates", std::to_string(startup_grid_candidates));
    add("startup_grid_best_votes", std::to_string(startup_grid_best_votes));
    add("startup_grid_best_overlap", std::to_string(startup_grid_best_overlap));
    add("startup_grid_best_x",
        std::isfinite(startup_grid_best_x) ? std::to_string(startup_grid_best_x) : "inf");
    add("startup_grid_best_y",
        std::isfinite(startup_grid_best_y) ? std::to_string(startup_grid_best_y) : "inf");
    add("startup_grid_best_z",
        std::isfinite(startup_grid_best_z) ? std::to_string(startup_grid_best_z) : "inf");
    add("startup_grid_best_yaw_rad",
        std::isfinite(startup_grid_best_yaw_rad) ? std::to_string(startup_grid_best_yaw_rad) : "inf");
    add("startup_grid_best_converged", boolText(startup_grid_best_converged));
    add("startup_grid_best_inliers", std::to_string(startup_grid_best_inliers));
    add("startup_grid_best_normalized_error",
        std::isfinite(startup_grid_best_normalized_error) ? std::to_string(startup_grid_best_normalized_error)
                                                          : "inf");
    add("startup_grid_rejection_reason", startup_grid_rejection_reason);
    add("online_relocalization_state", online_state);
    add("online_relocalization_attempts", std::to_string(online_attempts));
    add("online_relocalization_reason", online_reason);
    add("last_relocalization_source", last_relocalization_source);
    add("input_cloud_topic", input_cloud_topic_);
    add("current_scan_frame_id", current_scan_frame_id_);

    array.status.push_back(std::move(status));
    diag_pub_->publish(array);
}

pcl::PointCloud<pcl::FPFHSignature33>::Ptr LocalizationNode::computeFpfh(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) const {
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh(new pcl::PointCloud<pcl::FPFHSignature33>());
    if (!cloud || cloud->size() < static_cast<size_t>(kMinPointsForRegistration)) {
        return fpfh;
    }

    const int k_search =
        std::min<int>(std::max(5, num_neighbors_), static_cast<int>(cloud->size()));
    if (k_search < 5) {
        return fpfh;
    }

    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normal_estimation;
    normal_estimation.setNumberOfThreads(num_threads_);
    normal_estimation.setInputCloud(cloud);
    normal_estimation.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr(
        new pcl::search::KdTree<pcl::PointXYZ>()));
    normal_estimation.setKSearch(k_search);
    normal_estimation.compute(*normals);

    if (normals->size() != cloud->size()) {
        fpfh->clear();
        return fpfh;
    }

    pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_estimation;
    fpfh_estimation.setNumberOfThreads(num_threads_);
    fpfh_estimation.setInputCloud(cloud);
    fpfh_estimation.setInputNormals(normals);
    fpfh_estimation.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr(
        new pcl::search::KdTree<pcl::PointXYZ>()));
    fpfh_estimation.setKSearch(k_search);
    fpfh_estimation.compute(*fpfh);
    return fpfh;
}

std::array<double, 6> LocalizationNode::scaledCovariance(double scale) const {
    std::array<double, 6> out = kPoseCovDiag;
    for (double& value : out) {
        value *= scale;
    }
    return out;
}

}  // namespace rc26_localization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rc26_localization::LocalizationNode)
