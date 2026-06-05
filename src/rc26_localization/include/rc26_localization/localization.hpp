#ifndef RC26_LOCALIZATION__LOCALIZATION_HPP_
#define RC26_LOCALIZATION__LOCALIZATION_HPP_

#include <array>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
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

private:
    void loadGlobalMap(const std::string& file_name);
    bool prepareTargetMap();
    bool prepareStartupTarget();
    void registeredPcdCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void performRegistration();
    bool performStartupRelocalization(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_to_register);
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
    bool startup_relocalization_enable_{true};
    int startup_collect_ms_{1500};
    double startup_leaf_size_{0.3};
    std::vector<double> init_pose_;

    std::string map_frame_{"map"};
    std::string odom_frame_{"odom"};
    std::string robot_base_frame_{"base_footprint"};
    std::string prior_pcd_file_;
    std::string input_cloud_topic_{"registered_scan"};
    std::string pose_cov_topic_{"/localization/pose_with_cov"};
    std::string diagnostics_topic_{"/localization/diagnostics"};
    std::string current_scan_frame_id_;
    rclcpp::Time last_scan_time_;

    bool map_loaded_{false};
    bool target_ready_{false};
    bool startup_target_ready_{false};
    bool startup_relocalization_pending_{true};
    bool startup_relocalization_attempted_{false};
    std::string startup_relocalization_state_{"pending"};
    std::chrono::steady_clock::time_point startup_begin_wall_;

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
};

}  // namespace rc26_localization

#endif  // RC26_LOCALIZATION__LOCALIZATION_HPP_
