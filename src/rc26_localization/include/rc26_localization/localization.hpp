#ifndef RC26_LOCALIZATION__LOCALIZATION_HPP_
#define RC26_LOCALIZATION__LOCALIZATION_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "pcl/io/pcd_io.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "small_gicp/ann/kdtree_omp.hpp"
#include "small_gicp/factors/gicp_factor.hpp"
#include "small_gicp/pcl/pcl_point.hpp"
#include "small_gicp/registration/reduction_omp.hpp"
#include "small_gicp/registration/registration.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace rc26_localization {

class LocalizationNode : public rclcpp::Node {
public:
    explicit LocalizationNode(const rclcpp::NodeOptions& options);
    ~LocalizationNode() override;

private:
    void loadGlobalMap(const std::string& file_name);
    bool prepareTargetMap();
    bool prepareStartupTarget();
    void registeredPcdCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void performRegistration();
    bool performStartupGridRelocalization(const pcl::PointCloud<pcl::PointXYZ>::Ptr& clean_source,
                                          const pcl::PointCloud<pcl::PointXYZ>::Ptr& startup_source);
    bool performStartupRelocalization(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_to_register);
    struct RegistrationTargetView {
        pcl::PointCloud<pcl::PointCovariance>::Ptr cloud;
        std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> tree;
        size_t point_count{0};
        bool local_target_used{false};
        std::string failure_reason;
    };
    RegistrationTargetView makeLocalRegistrationTarget(const Eigen::Isometry3d& initial_guess) const;
    void updateRegistrationDebug(double delta_translation_m, double delta_z_m, double delta_yaw_rad,
                                 size_t active_target_points, bool local_target_used,
                                 const std::string& rejection_reason);
    static double yawDeltaRad(const Eigen::Isometry3d& from, const Eigen::Isometry3d& to);
    static Eigen::Isometry3d interpolateTransform(const Eigen::Isometry3d& from, const Eigen::Isometry3d& to,
                                                  double alpha);
    void noteLocalRegistrationFailure(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_to_register,
                                      const std::string& reason);
    void maybeStartOnlineRelocalization(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_to_register,
                                        const std::string& reason);
    void runOnlineRelocalization(pcl::PointCloud<pcl::PointXYZ>::Ptr trigger_cloud, std::string trigger_reason);
    bool performOnlineRelocalization(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_to_register,
                                     const std::string& trigger_reason);
    void consumePendingOnlineRelocalizationResult();
    void publishTransform();
    void publishPoseWithCov(const Eigen::Isometry3d& pose, const std::array<double, 6>& covariance_diag);
    void publishDiagnostics(const std::string& reason, uint8_t level, bool converged, bool accepted,
                            size_t inliers, double normalized_error, size_t source_points);
    std::array<double, 6> scaledCovariance(double scale) const;
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr computeFpfh(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) const;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_cov_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;

    rclcpp::TimerBase::SharedPtr registration_timer_;
    rclcpp::TimerBase::SharedPtr transform_timer_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    int num_threads_{4};
    int num_neighbors_{20};
    double global_leaf_size_{0.25};
    double registered_leaf_size_{0.25};
    double max_dist_sq_{1.0};
    int gicp_max_iterations_{20};
    int min_inliers_{200};
    double max_normalized_error_{0.3};
    bool require_initial_pose_for_local_tracking_{true};
    bool local_target_enable_{true};
    double local_target_radius_m_{6.0};
    int local_target_min_points_{50};
    bool registration_jump_gate_enable_{true};
    double max_registration_translation_delta_m_{0.35};
    double max_registration_z_delta_m_{0.20};
    double max_registration_yaw_delta_rad_{0.14};
    bool registration_smoothing_enable_{true};
    double registration_smoothing_alpha_{0.35};
    bool startup_relocalization_enable_{true};
    int startup_collect_ms_{1500};
    double startup_leaf_size_{0.3};
    bool startup_global_grid_enable_{true};
    double startup_global_grid_resolution_{0.25};
    double startup_global_grid_yaw_step_deg_{10.0};
    int startup_global_grid_max_candidates_{12};
    double startup_global_grid_min_overlap_ratio_{0.05};
    double startup_global_grid_max_normalized_error_{0.22};
    bool startup_global_grid_sac_fallback_{true};
    bool online_relocalization_enable_{false};
    int online_relocalization_trigger_after_failures_{3};
    int online_relocalization_cooldown_ms_{5000};
    int online_relocalization_max_attempts_{3};
    int online_relocalization_collect_ms_{1500};
    double online_relocalization_leaf_size_{0.3};
    std::vector<double> init_pose_;

    std::string map_frame_{"map"};
    std::string odom_frame_{"odom"};
    std::string robot_base_frame_{"base_footprint"};
    std::string prior_pcd_file_;
    std::string input_cloud_topic_{"registered_scan"};
    std::string pose_cov_topic_{"/localization/pose_with_cov"};
    std::string diagnostics_topic_{"/localization/diagnostics"};
    int input_cloud_queue_size_{30};
    std::string current_scan_frame_id_;
    rclcpp::Time last_scan_time_;

    bool map_loaded_{false};
    bool target_ready_{false};
    bool startup_target_ready_{false};
    bool startup_relocalization_pending_{true};
    bool startup_relocalization_attempted_{false};
    bool tracking_initialized_{false};
    std::string startup_relocalization_state_{"pending"};
    std::chrono::steady_clock::time_point startup_begin_wall_;
    std::chrono::steady_clock::time_point startup_first_cloud_wall_;
    bool startup_first_cloud_seen_{false};
    std::atomic<bool> online_relocalization_running_{false};
    std::atomic<bool> online_relocalization_stop_{false};
    std::atomic<bool> online_relocalization_cancel_requested_{false};
    std::thread online_relocalization_worker_;
    std::mutex online_relocalization_mutex_;
    int consecutive_registration_failures_{0};
    int online_relocalization_attempts_{0};
    std::string online_relocalization_state_{"disabled"};
    std::string online_relocalization_reason_{"disabled"};
    std::string last_relocalization_source_{"none"};
    std::chrono::steady_clock::time_point last_online_relocalization_attempt_wall_;

    std::mutex cloud_mutex_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud_;

    std::mutex map_mutex_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_;
    pcl::PointCloud<pcl::PointCovariance>::Ptr target_;
    std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> target_tree_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr startup_target_;
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr startup_target_fpfh_;

    pcl::PointCloud<pcl::PointCovariance>::Ptr source_;
    std::shared_ptr<small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>> registration_;

    std::mutex result_mutex_;
    Eigen::Isometry3d result_t_{Eigen::Isometry3d::Identity()};
    Eigen::Isometry3d previous_result_t_{Eigen::Isometry3d::Identity()};
    std::array<double, 6> last_pose_cov_diag_{{0.05, 0.05, 0.10, 0.05, 0.05, 0.05}};
    bool pending_online_relocalization_result_{false};
    Eigen::Isometry3d pending_online_relocalization_t_{Eigen::Isometry3d::Identity()};
    std::array<double, 6> pending_online_relocalization_cov_diag_{{0.05, 0.05, 0.10, 0.05, 0.05, 0.05}};

    std::mutex diagnostics_mutex_;
    double last_registration_delta_translation_m_{std::numeric_limits<double>::infinity()};
    double last_registration_delta_z_m_{std::numeric_limits<double>::infinity()};
    double last_registration_delta_yaw_rad_{std::numeric_limits<double>::infinity()};
    size_t last_active_target_points_{0};
    bool last_local_target_used_{false};
    std::string last_registration_rejection_reason_{"none"};
    size_t last_startup_grid_candidates_{0};
    int last_startup_grid_best_votes_{0};
    double last_startup_grid_best_overlap_{0.0};
    double last_startup_grid_best_x_{std::numeric_limits<double>::infinity()};
    double last_startup_grid_best_y_{std::numeric_limits<double>::infinity()};
    double last_startup_grid_best_z_{std::numeric_limits<double>::infinity()};
    double last_startup_grid_best_yaw_rad_{std::numeric_limits<double>::infinity()};
    bool last_startup_grid_best_converged_{false};
    size_t last_startup_grid_best_inliers_{0};
    double last_startup_grid_best_normalized_error_{std::numeric_limits<double>::infinity()};
    std::string last_startup_grid_rejection_reason_{"none"};
};

}  // namespace rc26_localization

#endif  // RC26_LOCALIZATION__LOCALIZATION_HPP_
