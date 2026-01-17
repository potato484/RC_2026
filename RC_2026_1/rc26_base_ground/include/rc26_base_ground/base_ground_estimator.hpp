#pragma once

#include <cstdint>
#include <deque>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int8.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace rc26_base_ground {

class BaseGroundEstimatorNode : public rclcpp::Node {
public:
    explicit BaseGroundEstimatorNode(const rclcpp::NodeOptions& options);

private:
    enum class State {
        Calibrating,
        Stable,
        Transitioning,
    };

    struct Sample {
        rclcpp::Time stamp;
        double z;
        double roll;
        double pitch;
    };

    void onOdom(const nav_msgs::msg::Odometry::ConstSharedPtr& msg);
    bool resolveBasePose(const nav_msgs::msg::Odometry& msg, geometry_msgs::msg::PoseStamped& out_pose);
    bool updateStabilityWindow(const Sample& sample, bool* stable_out);
    void updateLevelState(const Sample& sample);
    void publishLevel();
    void publishStairDelta(int8_t delta);
    void publishStable(bool stable);
    void publishTf(const geometry_msgs::msg::PoseStamped& base_pose_parent, double yaw, const rclcpp::Time& stamp);

    std::string odom_topic_;
    std::string parent_frame_;
    std::string base_ground_frame_;
    double step_height_m_{0.20};
    double tol_{0.06};
    double t_stable_{0.5};
    double t_confirm_{0.3};
    double tf_timeout_sec_{0.05};
    bool enable_tf_publish_{true};

    State state_{State::Calibrating};
    bool h0_valid_{false};
    double h0_{0.0};
    int32_t current_level_{0};
    double ground_z_{0.0};
    bool candidate_active_{false};
    int32_t candidate_level_{0};
    rclcpp::Time candidate_since_;

    bool stable_{false};
    std::deque<Sample> window_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr level_pub_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr stair_delta_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stable_pub_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace rc26_base_ground
