#include "rc26_localization/localization.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <sstream>
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
constexpr double kSacIaMinSampleDistance = 0.1;
constexpr int kSacIaCorrespondenceRandomness = 50;
constexpr int kSacIaNumSamples = 5;
constexpr std::array<double, 6> kPoseCovDiag{{0.05, 0.05, 0.10, 0.05, 0.05, 0.05}};

std::string boolText(bool value) {
    return value ? "true" : "false";
}

Eigen::Isometry3d toIsometry3d(const Eigen::Matrix4f& matrix) {
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.matrix() = matrix.cast<double>();
    return transform;
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
    this->declare_parameter("startup_relocalization_enable", startup_relocalization_enable_);
    this->declare_parameter("startup_collect_ms", startup_collect_ms_);
    this->declare_parameter("startup_leaf_size", startup_leaf_size_);
    this->declare_parameter("map_frame", map_frame_);
    this->declare_parameter("odom_frame", odom_frame_);
    this->declare_parameter("robot_base_frame", robot_base_frame_);
    this->declare_parameter("prior_pcd_file", prior_pcd_file_);
    this->declare_parameter("init_pose", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    this->declare_parameter("input_cloud_topic", input_cloud_topic_);
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
    this->get_parameter("startup_relocalization_enable", startup_relocalization_enable_);
    this->get_parameter("startup_collect_ms", startup_collect_ms_);
    this->get_parameter("startup_leaf_size", startup_leaf_size_);
    this->get_parameter("map_frame", map_frame_);
    this->get_parameter("odom_frame", odom_frame_);
    this->get_parameter("robot_base_frame", robot_base_frame_);
    this->get_parameter("prior_pcd_file", prior_pcd_file_);
    this->get_parameter("init_pose", init_pose_);
    this->get_parameter("input_cloud_topic", input_cloud_topic_);
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
    startup_collect_ms_ = std::max(0, startup_collect_ms_);
    startup_leaf_size_ = std::max(0.05, startup_leaf_size_);
    last_pose_cov_diag_ = kPoseCovDiag;
    startup_relocalization_pending_ = startup_relocalization_enable_;
    startup_relocalization_state_ = startup_relocalization_enable_ ? "pending" : "disabled";
    startup_begin_wall_ = std::chrono::steady_clock::now();
    if (robot_base_frame_.empty()) {
        robot_base_frame_ = "base_link";
    }

    if (init_pose_.size() >= 6) {
        result_t_.translation() << init_pose_[0], init_pose_[1], init_pose_[2];
        result_t_.linear() = (Eigen::AngleAxisd(init_pose_[5], Eigen::Vector3d::UnitZ()) *
                              Eigen::AngleAxisd(init_pose_[4], Eigen::Vector3d::UnitY()) *
                              Eigen::AngleAxisd(init_pose_[3], Eigen::Vector3d::UnitX()))
                                 .toRotationMatrix();
        previous_result_t_ = result_t_;
    }

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    pose_cov_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(pose_cov_topic_, 10);
    diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(diagnostics_topic_, 10);
    pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_cloud_topic_, 10, std::bind(&LocalizationNode::registeredPcdCallback, this, std::placeholders::_1));
    initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "initialpose", 10, std::bind(&LocalizationNode::initialPoseCallback, this, std::placeholders::_1));

    loadGlobalMap(prior_pcd_file_);

    registration_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(kRegistrationPeriodMs), std::bind(&LocalizationNode::performRegistration, this));
    transform_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(kTransformPeriodMs), std::bind(&LocalizationNode::publishTransform, this));

    RCLCPP_INFO(this->get_logger(),
                "rc26_localization minimal chain started: input=%s, map_frame=%s, odom_frame=%s, base=%s, startup_relocalization=%s",
                input_cloud_topic_.c_str(), map_frame_.c_str(), odom_frame_.c_str(), robot_base_frame_.c_str(),
                boolText(startup_relocalization_enable_).c_str());
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
        RCLCPP_ERROR(this->get_logger(), "prior_pcd_file is empty; localization is unavailable");
        publishDiagnostics("map_path_empty", diagnostic_msgs::msg::DiagnosticStatus::ERROR, false, false, 0,
                           std::numeric_limits<double>::infinity(), 0);
        return;
    }

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *global_map_) == -1) {
        RCLCPP_ERROR(this->get_logger(), "failed to read prior PCD: %s", file_name.c_str());
        publishDiagnostics("map_load_failed", diagnostic_msgs::msg::DiagnosticStatus::ERROR, false, false, 0,
                           std::numeric_limits<double>::infinity(), 0);
        return;
    }

    pcl::Indices indices;
    pcl::removeNaNFromPointCloud(*global_map_, *global_map_, indices);
    map_loaded_ = !global_map_->empty();
    RCLCPP_INFO(this->get_logger(), "loaded prior map: %zu points from %s", global_map_->size(), file_name.c_str());
    (void)prepareTargetMap();
    if (startup_relocalization_enable_) {
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
        RCLCPP_ERROR(this->get_logger(), "prior map has too few target points after downsampling: %zu < %d",
                     target_ ? target_->size() : 0U, kMinPointsForRegistration);
        target_ready_ = false;
        target_tree_.reset();
        return false;
    }

    small_gicp::estimate_covariances_omp(*target_, num_neighbors_, num_threads_);
    target_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
        target_, small_gicp::KdTreeBuilderOMP(num_threads_));
    target_ready_ = true;
    RCLCPP_INFO(this->get_logger(), "prepared prior map target: %zu points", target_->size());
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
        RCLCPP_WARN(this->get_logger(), "startup target has too few points after downsampling: %zu < %d",
                    startup_target_->size(), kMinPointsForRegistration);
        startup_target_ready_ = false;
        startup_target_fpfh_->clear();
        return false;
    }

    startup_target_fpfh_ = computeFpfh(startup_target_);
    startup_target_ready_ = startup_target_fpfh_ && !startup_target_fpfh_->empty();
    if (!startup_target_ready_) {
        RCLCPP_WARN(this->get_logger(), "failed to compute startup target FPFH");
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "prepared startup relocalization target: %zu points",
                startup_target_->size());
    return true;
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

    const size_t current_size = accumulated_cloud_->size();
    if (current_size >= kMaxAccumulatedPoints) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "accumulated localization cloud reached cap: %zu", kMaxAccumulatedPoints);
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
        RCLCPP_WARN(this->get_logger(), "initialpose rejected: invalid quaternion");
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
            last_pose_cov_diag_ = kPoseCovDiag;
            startup_relocalization_pending_ = false;
            startup_relocalization_attempted_ = true;
            startup_relocalization_state_ = "initialpose_override";
        }

        RCLCPP_INFO(this->get_logger(), "initialpose accepted: map->odom reset");
        publishDiagnostics("initialpose_accepted", diagnostic_msgs::msg::DiagnosticStatus::OK, true, true, 0, 0.0, 0);
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "initialpose rejected: cannot lookup %s -> %s: %s", odom_frame_.c_str(),
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

    if (startup_relocalization_pending_) {
        const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                    std::chrono::steady_clock::now() - startup_begin_wall_)
                                    .count();
        size_t accumulated_points = 0;
        {
            std::lock_guard<std::mutex> lock(cloud_mutex_);
            accumulated_points = accumulated_cloud_->size();
        }

        if (elapsed_ms < startup_collect_ms_ ||
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
                                 "no accumulated registered_scan points");
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
                             "too few source points after downsampling: %zu < %d", source_points,
                             kMinPointsForRegistration);
        publishDiagnostics("insufficient_source_points", diagnostic_msgs::msg::DiagnosticStatus::WARN, false, false,
                           0, std::numeric_limits<double>::infinity(), source_points);
        return;
    }

    small_gicp::estimate_covariances_omp(*source_, num_neighbors_, num_threads_);

    Eigen::Isometry3d initial_guess = Eigen::Isometry3d::Identity();
    {
        std::lock_guard<std::mutex> lock(result_mutex_);
        initial_guess = previous_result_t_;
    }

    registration_->reduction.num_threads = num_threads_;
    registration_->rejector.max_dist_sq = max_dist_sq_;
    registration_->optimizer.max_iterations = gicp_max_iterations_;

    const auto result = registration_->align(*target_, *source_, *target_tree_, initial_guess);
    const double normalized_error =
        (result.num_inliers > 0) ? (result.error / static_cast<double>(result.num_inliers))
                                 : std::numeric_limits<double>::infinity();
    const bool accepted = result.converged && result.num_inliers >= static_cast<size_t>(min_inliers_) &&
                          normalized_error <= max_normalized_error_;

    if (accepted) {
        {
            std::lock_guard<std::mutex> lock(result_mutex_);
            result_t_ = result.T_target_source;
            previous_result_t_ = result.T_target_source;
            last_pose_cov_diag_ = kPoseCovDiag;
        }
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "local registration accepted: inliers=%zu, normalized_error=%.4f", result.num_inliers,
                             normalized_error);
        publishDiagnostics("local_registration_ok", diagnostic_msgs::msg::DiagnosticStatus::OK, result.converged,
                           true, result.num_inliers, normalized_error, source_points);
    } else {
        {
            std::lock_guard<std::mutex> lock(result_mutex_);
            last_pose_cov_diag_ = scaledCovariance(kBadCovScale);
        }
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "local registration frozen: converged=%s, inliers=%zu/%d, normalized_error=%.4f/%.4f",
                             boolText(result.converged).c_str(), result.num_inliers, min_inliers_, normalized_error,
                             max_normalized_error_);
        publishDiagnostics("local_registration_frozen", diagnostic_msgs::msg::DiagnosticStatus::WARN,
                           result.converged, false, result.num_inliers, normalized_error, source_points);
    }
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

    pcl::PointCloud<pcl::PointXYZ>::Ptr startup_source(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setLeafSize(static_cast<float>(startup_leaf_size_), static_cast<float>(startup_leaf_size_),
                      static_cast<float>(startup_leaf_size_));
    voxel.setInputCloud(clean_source);
    voxel.filter(*startup_source);

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
                    "startup relocalization rejected: sac_score=%.4f, converged=%s, inliers=%zu/%d, normalized_error=%.4f/%.4f",
                    sac_ia.getFitnessScore(), boolText(result.converged).c_str(), result.num_inliers, min_inliers_,
                    normalized_error, max_normalized_error_);
        publishDiagnostics("startup_relocalization_failed", diagnostic_msgs::msg::DiagnosticStatus::WARN,
                           result.converged, false, result.num_inliers, normalized_error, source_points);
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(result_mutex_);
        result_t_ = result.T_target_source;
        previous_result_t_ = result.T_target_source;
        last_pose_cov_diag_ = kPoseCovDiag;
        startup_relocalization_state_ = "ok";
    }

    RCLCPP_INFO(this->get_logger(),
                "startup relocalization accepted: sac_score=%.4f, inliers=%zu, normalized_error=%.4f",
                sac_ia.getFitnessScore(), result.num_inliers, normalized_error);
    publishDiagnostics("startup_relocalization_ok", diagnostic_msgs::msg::DiagnosticStatus::OK, result.converged,
                       true, result.num_inliers, normalized_error, source_points);
    return true;
}

void LocalizationNode::publishTransform() {
    if (!map_loaded_) {
        return;
    }

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
    add("inliers", std::to_string(inliers));
    add("min_inliers", std::to_string(min_inliers_));
    add("source_points", std::to_string(source_points));
    add("max_normalized_error", std::to_string(max_normalized_error_));
    add("normalized_error", std::isfinite(normalized_error) ? std::to_string(normalized_error) : "inf");
    add("map_loaded", boolText(map_loaded_));
    add("target_ready", boolText(target_ready_));
    add("startup_relocalization", startup_relocalization_state_);
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
