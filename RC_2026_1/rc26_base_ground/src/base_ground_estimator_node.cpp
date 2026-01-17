#include "rc26_base_ground/base_ground_estimator.hpp"

#include <algorithm>
#include <cmath>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace rc26_base_ground {

namespace {
constexpr double kMinDurationSec = 1e-3;
constexpr double kMaxAngularRateRadPerSec = 0.6;
}  // namespace

BaseGroundEstimatorNode::BaseGroundEstimatorNode(const rclcpp::NodeOptions& options)
    : Node("base_ground_estimator", options) {
    this->declare_parameter<std::string>("odom_topic", "/odom");
    this->declare_parameter<std::string>("parent_frame", "odom");
    this->declare_parameter<std::string>("base_ground_frame", "base_ground");
    this->declare_parameter<double>("step_height_m", 0.20);
    this->declare_parameter<double>("tol", 0.06);
    this->declare_parameter<double>("T_stable", 0.5);
    this->declare_parameter<double>("T_confirm", 0.3);
    this->declare_parameter<double>("tf_timeout_sec", 0.05);
    this->declare_parameter<bool>("enable_tf_publish", true);

    this->get_parameter("odom_topic", odom_topic_);
    this->get_parameter("parent_frame", parent_frame_);
    this->get_parameter("base_ground_frame", base_ground_frame_);
    this->get_parameter("step_height_m", step_height_m_);
    this->get_parameter("tol", tol_);
    this->get_parameter("T_stable", t_stable_);
    this->get_parameter("T_confirm", t_confirm_);
    this->get_parameter("tf_timeout_sec", tf_timeout_sec_);
    this->get_parameter("enable_tf_publish", enable_tf_publish_);

    if (parent_frame_.empty()) {
        parent_frame_ = "odom";
    }
    if (base_ground_frame_.empty()) {
        base_ground_frame_ = "base_ground";
    }

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    level_pub_ = this->create_publisher<std_msgs::msg::Int32>("base_ground/level", 10);
    stair_delta_pub_ = this->create_publisher<std_msgs::msg::Int8>("base_ground/stair_delta", 10);
    stable_pub_ = this->create_publisher<std_msgs::msg::Bool>("base_ground/stable", 10);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 10, std::bind(&BaseGroundEstimatorNode::onOdom, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "base_ground_estimator started: parent=%s, step=%.2fm, tol=%.2fm",
                parent_frame_.c_str(), step_height_m_, tol_);
}

void BaseGroundEstimatorNode::onOdom(const nav_msgs::msg::Odometry::ConstSharedPtr& msg) {
    geometry_msgs::msg::PoseStamped base_pose_parent;
    if (!resolveBasePose(*msg, base_pose_parent)) {
        return;
    }

    tf2::Quaternion q;
    tf2::fromMsg(base_pose_parent.pose.orientation, q);
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    Sample sample{rclcpp::Time(msg->header.stamp), base_pose_parent.pose.position.z, roll, pitch};

    bool stable_now = false;
    updateStabilityWindow(sample, &stable_now);
    stable_ = stable_now;
    publishStable(stable_);

    updateLevelState(sample);
    if (h0_valid_) {
        publishLevel();
    }

    publishTf(base_pose_parent, yaw, sample.stamp);
}

bool BaseGroundEstimatorNode::resolveBasePose(const nav_msgs::msg::Odometry& msg,
                                              geometry_msgs::msg::PoseStamped& out_pose) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = msg.header;
    pose.pose = msg.pose.pose;

    if (parent_frame_ == pose.header.frame_id) {
        out_pose = pose;
        return true;
    }

    if (!tf_buffer_) {
        return false;
    }

    try {
        out_pose = tf_buffer_->transform(pose, parent_frame_, tf2::durationFromSec(tf_timeout_sec_));
        return true;
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "base_ground_estimator: TF %s -> %s unavailable: %s",
                             pose.header.frame_id.c_str(), parent_frame_.c_str(), ex.what());
        return false;
    }
}

bool BaseGroundEstimatorNode::updateStabilityWindow(const Sample& sample, bool* stable_out) {
    // 处理时间戳回退：清空滑窗避免无效计算
    if (!window_.empty() && sample.stamp < window_.back().stamp) {
        window_.clear();
    }

    window_.push_back(sample);
    while (!window_.empty() && (sample.stamp - window_.front().stamp).seconds() > t_stable_) {
        window_.pop_front();
    }

    if (window_.size() < 2) {
        *stable_out = false;
        return false;
    }

    const double duration = (window_.back().stamp - window_.front().stamp).seconds();
    if (duration < t_stable_) {
        *stable_out = false;
        return false;
    }

    double sum_z = 0.0;
    for (const auto& s : window_) {
        sum_z += s.z;
    }
    const double mean_z = sum_z / static_cast<double>(window_.size());

    double var_z = 0.0;
    for (const auto& s : window_) {
        const double dz = s.z - mean_z;
        var_z += dz * dz;
    }
    var_z /= static_cast<double>(window_.size());

    double max_dz_dt = 0.0;
    double max_droll_dt = 0.0;
    double max_dpitch_dt = 0.0;
    for (size_t i = 1; i < window_.size(); ++i) {
        const double dt = (window_[i].stamp - window_[i - 1].stamp).seconds();
        if (dt <= 0.0) {
            continue;
        }
        max_dz_dt = std::max(max_dz_dt, std::abs(window_[i].z - window_[i - 1].z) / dt);
        max_droll_dt = std::max(max_droll_dt, std::abs(window_[i].roll - window_[i - 1].roll) / dt);
        max_dpitch_dt = std::max(max_dpitch_dt, std::abs(window_[i].pitch - window_[i - 1].pitch) / dt);
    }

    const double dz_dt_max = tol_ / std::max(t_stable_, kMinDurationSec);
    const bool stable = (std::sqrt(var_z) <= tol_) && (max_dz_dt <= dz_dt_max) &&
                        (max_droll_dt <= kMaxAngularRateRadPerSec) && (max_dpitch_dt <= kMaxAngularRateRadPerSec);

    *stable_out = stable;
    return true;
}

void BaseGroundEstimatorNode::updateLevelState(const Sample& sample) {
    if (!stable_) {
        candidate_active_ = false;
        if (state_ == State::Transitioning) {
            state_ = State::Stable;
        }
        return;
    }

    if (!h0_valid_) {
        h0_ = sample.z;
        h0_valid_ = true;
        current_level_ = 0;
        ground_z_ = 0.0;
        candidate_active_ = false;
        state_ = State::Stable;
        RCLCPP_INFO(this->get_logger(), "h0 calibrated: %.3f m", h0_);
        return;
    }

    if (step_height_m_ <= 0.0) {
        return;
    }

    const double ground_z_candidate = sample.z - h0_;
    const int32_t k = static_cast<int32_t>(std::llround(ground_z_candidate / step_height_m_));
    const double ground_z_quant = static_cast<double>(k) * step_height_m_;

    if (std::abs(ground_z_candidate - ground_z_quant) < tol_) {
        if (!candidate_active_ || candidate_level_ != k) {
            candidate_active_ = true;
            candidate_level_ = k;
            candidate_since_ = sample.stamp;
            state_ = State::Transitioning;
            return;
        }

        if ((sample.stamp - candidate_since_).seconds() >= t_confirm_ && k != current_level_) {
            const int32_t prev_level = current_level_;
            current_level_ = k;
            ground_z_ = ground_z_quant;
            const int32_t delta_i = k - prev_level;
            const int32_t clamped = std::min<int32_t>(127, std::max<int32_t>(-128, delta_i));
            publishStairDelta(static_cast<int8_t>(clamped));
            RCLCPP_INFO(this->get_logger(), "Level changed: %d -> %d (delta=%d)", prev_level, current_level_, delta_i);
            candidate_active_ = false;
            state_ = State::Stable;
        }
        return;
    }

    candidate_active_ = false;
    if (state_ == State::Transitioning) {
        state_ = State::Stable;
    }
}

void BaseGroundEstimatorNode::publishLevel() {
    std_msgs::msg::Int32 msg;
    msg.data = current_level_;
    level_pub_->publish(msg);
}

void BaseGroundEstimatorNode::publishStairDelta(int8_t delta) {
    std_msgs::msg::Int8 msg;
    msg.data = delta;
    stair_delta_pub_->publish(msg);
}

void BaseGroundEstimatorNode::publishStable(bool stable) {
    std_msgs::msg::Bool msg;
    msg.data = stable;
    stable_pub_->publish(msg);
}

void BaseGroundEstimatorNode::publishTf(const geometry_msgs::msg::PoseStamped& base_pose_parent, double yaw,
                                        const rclcpp::Time& stamp) {
    if (!enable_tf_publish_) {
        return;
    }

    // h0 标定前不发布 TF，避免错误的 Z 值
    if (!h0_valid_) {
        return;
    }

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = stamp;
    tf_msg.header.frame_id = parent_frame_;
    tf_msg.child_frame_id = base_ground_frame_;
    tf_msg.transform.translation.x = base_pose_parent.pose.position.x;
    tf_msg.transform.translation.y = base_pose_parent.pose.position.y;
    tf_msg.transform.translation.z = ground_z_;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    tf_msg.transform.rotation = tf2::toMsg(q);

    tf_broadcaster_->sendTransform(tf_msg);
}

}  // namespace rc26_base_ground

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rc26_base_ground::BaseGroundEstimatorNode>(rclcpp::NodeOptions());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
