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
constexpr double kMaxAngularRateRadPerSec = 0.6;
}  // namespace

BaseGroundEstimatorNode::BaseGroundEstimatorNode(const rclcpp::NodeOptions& options)
    : Node("base_ground_estimator", options) {
    this->declare_parameter<std::string>("odom_topic", "odom");
    this->declare_parameter<std::string>("parent_frame", "odom");
    this->declare_parameter<std::string>("base_ground_frame", "base_ground");
    this->declare_parameter<double>("step_height_m", 0.20);
    this->declare_parameter<double>("tol_level_m", 0.04);
    this->declare_parameter<double>("tol_stable_z_std_m", 0.015);
    this->declare_parameter<double>("tol_stable_lin_vel_mps", 0.05);
    this->declare_parameter<double>("tol_stable_ang_vel_rps", 0.05);
    this->declare_parameter<double>("T_confirm", 0.4);
    this->declare_parameter<int>("window_size", 10);
    this->declare_parameter<int>("h0_calibration_samples", 10);
    this->declare_parameter<bool>("h0_override_enable", false);
    this->declare_parameter<double>("h0_override_m", 0.0);
    this->declare_parameter<double>("lift_thresh_m", 0.15);
    this->declare_parameter<double>("lift_time_s", 0.5);
    this->declare_parameter<double>("tf_timeout_sec", 0.05);
    this->declare_parameter<bool>("enable_tf_publish", true);

    this->get_parameter("odom_topic", odom_topic_);
    this->get_parameter("parent_frame", parent_frame_);
    this->get_parameter("base_ground_frame", base_ground_frame_);
    this->get_parameter("step_height_m", step_height_m_);
    this->get_parameter("tol_level_m", tol_level_m_);
    this->get_parameter("tol_stable_z_std_m", tol_stable_z_std_m_);
    this->get_parameter("tol_stable_lin_vel_mps", tol_stable_lin_vel_mps_);
    this->get_parameter("tol_stable_ang_vel_rps", tol_stable_ang_vel_rps_);
    this->get_parameter("T_confirm", t_confirm_);
    this->get_parameter("window_size", window_size_);
    this->get_parameter("h0_calibration_samples", h0_calibration_samples_);
    this->get_parameter("h0_override_enable", h0_override_enable_);
    this->get_parameter("h0_override_m", h0_override_m_);
    this->get_parameter("lift_thresh_m", lift_thresh_m_);
    this->get_parameter("lift_time_s", lift_time_s_);
    this->get_parameter("tf_timeout_sec", tf_timeout_sec_);
    this->get_parameter("enable_tf_publish", enable_tf_publish_);

    if (window_size_ < 2) {
        RCLCPP_WARN(this->get_logger(), "window_size=%d too small, clamped to 2", window_size_);
        window_size_ = 2;
    }
    if (h0_calibration_samples_ < 1) {
        RCLCPP_WARN(this->get_logger(), "h0_calibration_samples=%d too small, clamped to 1", h0_calibration_samples_);
        h0_calibration_samples_ = 1;
    }
    if (t_confirm_ < 0.0) {
        RCLCPP_WARN(this->get_logger(), "T_confirm=%.3f invalid, clamped to 0.0", t_confirm_);
        t_confirm_ = 0.0;
    }
    if (lift_time_s_ < 0.0) {
        RCLCPP_WARN(this->get_logger(), "lift_time_s=%.3f invalid, clamped to 0.0", lift_time_s_);
        lift_time_s_ = 0.0;
    }
    if (lift_thresh_m_ < 0.0) {
        RCLCPP_WARN(this->get_logger(), "lift_thresh_m=%.3f invalid, clamped to 0.0", lift_thresh_m_);
        lift_thresh_m_ = 0.0;
    }

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
    stable_terrain_pub_ = this->create_publisher<std_msgs::msg::Bool>("base_ground/stable_terrain", 10);
    stable_operation_pub_ = this->create_publisher<std_msgs::msg::Bool>("base_ground/stable_operation", 10);
    is_lifted_pub_ = this->create_publisher<std_msgs::msg::Bool>("base_ground/is_lifted", 10);

    if (h0_override_enable_) {
        h0_ = h0_override_m_;
        h0_valid_ = true;
        h0_cal_count_ = h0_calibration_samples_;
        current_level_ = 0;
        ground_z_ = 0.0;
        state_ = State::Stable;
    }

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 10, std::bind(&BaseGroundEstimatorNode::onOdom, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
                "base_ground_estimator: tol_level=%.3f tol_z_std=%.3f win=%d T_confirm=%.2f",
                tol_level_m_, tol_stable_z_std_m_, window_size_, t_confirm_);
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

    Sample sample{rclcpp::Time(msg->header.stamp),
                  base_pose_parent.pose.position.z,
                  roll,
                  pitch,
                  msg->twist.twist.linear.x,
                  msg->twist.twist.linear.y,
                  msg->twist.twist.angular.z};

    updateStabilityWindow(sample);
    updateLiftedState(sample);
    updateLevelState(sample);

    {
        std_msgs::msg::Bool status_msg;
        status_msg.data = stable_terrain_;
        stable_terrain_pub_->publish(status_msg);
        status_msg.data = stable_operation_;
        stable_operation_pub_->publish(status_msg);
        status_msg.data = is_lifted_;
        is_lifted_pub_->publish(status_msg);
    }

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

bool BaseGroundEstimatorNode::updateStabilityWindow(const Sample& sample) {
    if (!window_.empty() && sample.stamp < window_.back().stamp) {
        window_.clear();
    }

    window_.push_back(sample);
    while (static_cast<int>(window_.size()) > window_size_) {
        window_.pop_front();
    }
    if (static_cast<int>(window_.size()) < window_size_) {
        stable_terrain_ = false;
        stable_operation_ = false;
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

    double max_droll_dt = 0.0;
    double max_dpitch_dt = 0.0;
    for (size_t i = 1; i < window_.size(); ++i) {
        const double dt = (window_[i].stamp - window_[i - 1].stamp).seconds();
        if (dt <= 0.0) {
            continue;
        }
        max_droll_dt = std::max(max_droll_dt, std::abs(window_[i].roll - window_[i - 1].roll) / dt);
        max_dpitch_dt = std::max(max_dpitch_dt, std::abs(window_[i].pitch - window_[i - 1].pitch) / dt);
    }

    stable_terrain_ = (std::sqrt(var_z) <= tol_stable_z_std_m_) &&
                      (max_droll_dt <= kMaxAngularRateRadPerSec) &&
                      (max_dpitch_dt <= kMaxAngularRateRadPerSec);

    double max_lin = 0.0;
    double max_ang = 0.0;
    for (const auto& s : window_) {
        max_lin = std::max(max_lin, std::hypot(s.vx, s.vy));
        max_ang = std::max(max_ang, std::abs(s.wz));
    }
    stable_operation_ = stable_terrain_ &&
                        (max_lin <= tol_stable_lin_vel_mps_) &&
                        (max_ang <= tol_stable_ang_vel_rps_);

    return true;
}

void BaseGroundEstimatorNode::updateLiftedState(const Sample& sample) {
    if (!h0_valid_) {
        return;
    }

    const double ground_z_candidate = sample.z - h0_;
    const double lift_err = std::abs(ground_z_candidate - ground_z_);

    if (!is_lifted_) {
        if (lift_err > lift_thresh_m_) {
            if (!lift_timing_) {
                lift_timing_ = true;
                lift_detect_since_ = sample.stamp;
            } else if ((sample.stamp - lift_detect_since_).seconds() >= lift_time_s_) {
                is_lifted_ = true;
                lift_timing_ = false;
                RCLCPP_WARN(this->get_logger(), "is_lifted=true (err=%.3f m)", lift_err);
            }
        } else {
            lift_timing_ = false;
        }
    } else {
        if (step_height_m_ > 0.0) {
            const int32_t k = static_cast<int32_t>(std::llround(ground_z_candidate / step_height_m_));
            const double ground_z_quant = static_cast<double>(k) * step_height_m_;
            const bool in_level_tol = std::abs(ground_z_candidate - ground_z_quant) < tol_level_m_;

            if (stable_terrain_ && in_level_tol) {
                if (!lift_timing_) {
                    lift_timing_ = true;
                    lift_detect_since_ = sample.stamp;
                } else if ((sample.stamp - lift_detect_since_).seconds() >= t_confirm_) {
                    is_lifted_ = false;
                    lift_timing_ = false;
                    current_level_ = k;
                    ground_z_ = ground_z_quant;
                    candidate_active_ = false;
                    state_ = State::Stable;
                    RCLCPP_INFO(this->get_logger(), "is_lifted=false, synced level=%d", current_level_);
                }
            } else {
                lift_timing_ = false;
            }
        } else {
            lift_timing_ = false;
        }
    }

    if (is_lifted_) {
        stable_terrain_ = false;
        stable_operation_ = false;
    }
}

void BaseGroundEstimatorNode::updateLevelState(const Sample& sample) {
    if (is_lifted_) {
        candidate_active_ = false;
        return;
    }

    if (!stable_terrain_) {
        candidate_active_ = false;
        if (state_ == State::Transitioning) {
            state_ = State::Stable;
        }
        return;
    }

    if (!h0_valid_) {
        if (!stable_terrain_ || is_lifted_) {
            candidate_active_ = false;
            return;
        }

        h0_cal_sum_ += sample.z;
        h0_cal_count_++;
        if (h0_cal_count_ >= h0_calibration_samples_) {
            h0_ = h0_cal_sum_ / static_cast<double>(h0_cal_count_);
            h0_valid_ = true;
            current_level_ = 0;
            ground_z_ = 0.0;
            candidate_active_ = false;
            state_ = State::Stable;
            RCLCPP_INFO(this->get_logger(), "h0 calibrated: %.3f m (%d samples)", h0_, h0_cal_count_);
        }
        return;
    }

    if (step_height_m_ <= 0.0) {
        return;
    }

    const double ground_z_candidate = sample.z - h0_;
    const int32_t k = static_cast<int32_t>(std::llround(ground_z_candidate / step_height_m_));
    const double ground_z_quant = static_cast<double>(k) * step_height_m_;

    if (std::abs(ground_z_candidate - ground_z_quant) < tol_level_m_) {
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

void BaseGroundEstimatorNode::publishTf(const geometry_msgs::msg::PoseStamped& base_pose_parent, double yaw,
                                        const rclcpp::Time& stamp) {
    if (!enable_tf_publish_) {
        return;
    }

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
