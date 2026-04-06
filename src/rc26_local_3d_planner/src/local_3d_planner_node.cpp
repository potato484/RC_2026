#include "rc26_local_3d_planner/planner_core.hpp"

#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <utility>

namespace rc26_local_3d_planner {

namespace {

double yawFromQuaternion(const geometry_msgs::msg::Quaternion& q) {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

std::vector<double> parameterArrayOrDefault(
    rclcpp::Node* node, const std::string& name, const std::vector<double>& fallback) {
    std::vector<double> values;
    try {
        values = node->get_parameter(name).as_double_array();
    } catch (const rclcpp::exceptions::InvalidParameterTypeException&) {
        values = fallback;
    }
    return values.empty() ? fallback : values;
}

}  // namespace

class Local3DPlannerNode : public rclcpp::Node {
public:
    Local3DPlannerNode()
        : rclcpp::Node("local_3d_planner"),
          tf_buffer_(std::make_shared<tf2_ros::Buffer>(get_clock())),
          tf_listener_(*tf_buffer_) {
        declare_parameter<std::string>("odom_topic", "control_state");
        declare_parameter<std::string>("map_frame", "map");
        declare_parameter<std::string>("base_frame", "base_link");
        declare_parameter<double>("publish_frequency_hz", 5.0);
        declare_parameter<double>("lookahead_distance_m", 0.7);
        declare_parameter<double>("horizon_sec", 1.2);
        declare_parameter<double>("integration_step_sec", 0.1);
        declare_parameter<double>("terrain_obstacle_threshold", 0.6);
        declare_parameter<double>("terrain_drop_threshold", 0.8);
        declare_parameter<double>("recovery_heading_threshold_rad", 0.55);
        declare_parameter<double>("recovery_angular_speed", 0.7);
        declare_parameter<double>("slow_zone_speed_scale", 0.6);
        declare_parameter<double>("path_alignment_weight", 2.4);
        declare_parameter<double>("heading_alignment_weight", 1.8);
        declare_parameter<double>("speed_preference_weight", 0.8);
        declare_parameter<double>("angular_effort_weight", 0.5);
        declare_parameter<double>("clearance_weight", 2.2);
        declare_parameter<double>("stop_envelope_half_width_m", 0.24);
        declare_parameter<std::vector<double>>(
            "sample_linear_speeds", std::vector<double>{0.0, 0.15, 0.30, 0.45});
        declare_parameter<std::vector<double>>(
            "sample_angular_speeds", std::vector<double>{-0.8, -0.4, 0.0, 0.4, 0.8});
        declare_parameter<std::string>("robot_geometry_file", "");
        declare_parameter<std::string>("robot_geometry_profile", "compact");

        map_frame_ = get_parameter("map_frame").as_string();
        base_frame_ = get_parameter("base_frame").as_string();
        const auto odom_topic = get_parameter("odom_topic").as_string();
        const auto publish_frequency_hz =
            std::max(1.0, get_parameter("publish_frequency_hz").as_double());

        PlannerConfig config;
        config.lookahead_distance_m = std::max(0.2, get_parameter("lookahead_distance_m").as_double());
        config.horizon_sec = std::max(0.4, get_parameter("horizon_sec").as_double());
        config.integration_step_sec =
            std::max(0.05, get_parameter("integration_step_sec").as_double());
        config.terrain_obstacle_threshold =
            std::clamp(get_parameter("terrain_obstacle_threshold").as_double(), 0.0, 1.0);
        config.terrain_drop_threshold =
            std::clamp(get_parameter("terrain_drop_threshold").as_double(), 0.0, 1.0);
        config.recovery_heading_threshold_rad =
            std::max(0.1, get_parameter("recovery_heading_threshold_rad").as_double());
        config.recovery_angular_speed =
            std::max(0.1, get_parameter("recovery_angular_speed").as_double());
        config.slow_zone_speed_scale =
            std::clamp(get_parameter("slow_zone_speed_scale").as_double(), 0.1, 1.0);
        config.path_alignment_weight =
            std::max(0.0, get_parameter("path_alignment_weight").as_double());
        config.heading_alignment_weight =
            std::max(0.0, get_parameter("heading_alignment_weight").as_double());
        config.speed_preference_weight =
            std::max(0.0, get_parameter("speed_preference_weight").as_double());
        config.angular_effort_weight =
            std::max(0.0, get_parameter("angular_effort_weight").as_double());
        config.clearance_weight = std::max(0.0, get_parameter("clearance_weight").as_double());
        config.stop_envelope_half_width_m =
            std::max(0.0, get_parameter("stop_envelope_half_width_m").as_double());
        config.sample_linear_speeds =
            parameterArrayOrDefault(this, "sample_linear_speeds", config.sample_linear_speeds);
        config.sample_angular_speeds =
            parameterArrayOrDefault(this, "sample_angular_speeds", config.sample_angular_speeds);

        const auto robot_geometry_file = get_parameter("robot_geometry_file").as_string();
        const auto robot_geometry_profile = get_parameter("robot_geometry_profile").as_string();
        if (!robot_geometry_file.empty()) {
            std::string error;
            const auto geometry = PlannerCore::loadRobotGeometryProfile(
                robot_geometry_file, robot_geometry_profile, error);
            if (geometry) {
                config.stop_envelope_half_width_m = std::max(
                    config.stop_envelope_half_width_m, geometry->stop_envelope_half_width_m);
            } else {
                RCLCPP_WARN(get_logger(), "local_3d_planner geometry load failed: %s", error.c_str());
            }
        }
        planner_.setConfig(config);

        corridor_sub_ = create_subscription<rc26_interfaces::msg::XhuSemanticCorridor>(
            "/xhu_nav/corridor_cmd", 10,
            [this](const rc26_interfaces::msg::XhuSemanticCorridor::SharedPtr msg) {
                if (msg) {
                    corridor_ = *msg;
                    has_corridor_ = true;
                }
            });
        mode_sub_ = create_subscription<rc26_interfaces::msg::XhuMotionModeState>(
            "/xhu_nav/motion_mode_state", 10,
            [this](const rc26_interfaces::msg::XhuMotionModeState::SharedPtr msg) {
                if (msg) {
                    mode_state_ = *msg;
                    has_mode_state_ = true;
                }
            });
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, rclcpp::SensorDataQoS(),
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                if (msg) {
                    last_odom_ = *msg;
                    has_odom_ = true;
                }
            });
        terrain_sub_ = create_subscription<rc26_interfaces::msg::TerrainFeatureGrid>(
            "terrain_features", 10,
            [this](const rc26_interfaces::msg::TerrainFeatureGrid::SharedPtr msg) {
                if (msg) {
                    terrain_grid_ = *msg;
                    has_terrain_grid_ = true;
                }
            });
        semantic_sub_ = create_subscription<rc26_interfaces::msg::XhuSemanticLayerSummary>(
            "/xhu_nav/semantic_layer_summary", 10,
            [this](const rc26_interfaces::msg::XhuSemanticLayerSummary::SharedPtr msg) {
                if (msg) {
                    semantic_summary_ = *msg;
                    has_semantic_summary_ = true;
                }
            });

        state_pub_ = create_publisher<rc26_interfaces::msg::XhuLocalPlannerState>(
            "/xhu_nav/local_planner_state", 10);
        recovery_pub_ = create_publisher<rc26_interfaces::msg::XhuRecoveryState>(
            "/xhu_nav/recovery_state", 10);
        preview_pub_ = create_publisher<nav_msgs::msg::Path>("/xhu_nav/local_planner_preview", 10);

        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::duration<double>(1.0 / publish_frequency_hz)),
            std::bind(&Local3DPlannerNode::onTimer, this));
    }

private:
    bool queryPose(double& x, double& y, double& yaw) const {
        try {
            const auto tf = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
            x = tf.transform.translation.x;
            y = tf.transform.translation.y;
            yaw = yawFromQuaternion(tf.transform.rotation);
            return true;
        } catch (const tf2::TransformException&) {
            if (!has_odom_) {
                return false;
            }
            x = last_odom_.pose.pose.position.x;
            y = last_odom_.pose.pose.position.y;
            yaw = yawFromQuaternion(last_odom_.pose.pose.orientation);
            return true;
        }
    }

    void onTimer() {
        rc26_interfaces::msg::XhuLocalPlannerState state_msg;
        state_msg.header.stamp = now();
        state_msg.header.frame_id = map_frame_;
        state_msg.observe_only = true;
        state_msg.status = "HOLD";
        state_msg.reason = "corridor unavailable";

        if (!has_corridor_) {
            state_pub_->publish(state_msg);
            return;
        }

        PlannerInput input;
        input.corridor = corridor_;
        input.has_mode_state = has_mode_state_;
        input.mode_state = mode_state_;
        input.has_terrain_grid = has_terrain_grid_;
        input.terrain_grid = terrain_grid_;
        input.has_semantic_summary = has_semantic_summary_;
        input.semantic_summary = semantic_summary_;
        if (has_odom_) {
            input.current_vx = last_odom_.twist.twist.linear.x;
            input.current_wz = last_odom_.twist.twist.angular.z;
        }
        input.has_pose = queryPose(input.robot_x, input.robot_y, input.robot_yaw);

        const auto result = planner_.plan(input);
        state_msg.corridor_id = corridor_.corridor_id;
        state_msg.edge_id = corridor_.edge_id;
        state_msg.status = result.status;
        state_msg.terminal = false;
        state_msg.semantic_revision = has_semantic_summary_ ? semantic_summary_.revision : 0U;
        state_msg.cmd_vx = static_cast<float>(result.cmd_vx);
        state_msg.cmd_vy = static_cast<float>(result.cmd_vy);
        state_msg.cmd_wz = static_cast<float>(result.cmd_wz);
        state_msg.best_score = static_cast<float>(result.best_score);
        state_msg.clearance_margin_m = static_cast<float>(result.clearance_margin_m);
        state_msg.reason = result.reason;
        state_pub_->publish(state_msg);

        if (!result.preview_path.poses.empty()) {
            preview_pub_->publish(result.preview_path);
        }

        auto recovery_msg = result.recovery_state;
        recovery_msg.header.stamp = state_msg.header.stamp;
        recovery_msg.header.frame_id = map_frame_;
        recovery_msg.corridor_id = corridor_.corridor_id;
        recovery_msg.edge_id = corridor_.edge_id;
        if (recovery_msg.status.empty()) {
            recovery_msg.recovery_name = "none";
            recovery_msg.status = "IDLE";
            recovery_msg.reason = result.reason;
        }
        recovery_pub_->publish(recovery_msg);
    }

    std::string map_frame_{"map"};
    std::string base_frame_{"base_link"};
    PlannerCore planner_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    rclcpp::Subscription<rc26_interfaces::msg::XhuSemanticCorridor>::SharedPtr corridor_sub_;
    rclcpp::Subscription<rc26_interfaces::msg::XhuMotionModeState>::SharedPtr mode_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<rc26_interfaces::msg::TerrainFeatureGrid>::SharedPtr terrain_sub_;
    rclcpp::Subscription<rc26_interfaces::msg::XhuSemanticLayerSummary>::SharedPtr semantic_sub_;
    rclcpp::Publisher<rc26_interfaces::msg::XhuLocalPlannerState>::SharedPtr state_pub_;
    rclcpp::Publisher<rc26_interfaces::msg::XhuRecoveryState>::SharedPtr recovery_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr preview_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool has_corridor_{false};
    bool has_mode_state_{false};
    bool has_odom_{false};
    bool has_terrain_grid_{false};
    bool has_semantic_summary_{false};
    rc26_interfaces::msg::XhuSemanticCorridor corridor_;
    rc26_interfaces::msg::XhuMotionModeState mode_state_;
    nav_msgs::msg::Odometry last_odom_;
    rc26_interfaces::msg::TerrainFeatureGrid terrain_grid_;
    rc26_interfaces::msg::XhuSemanticLayerSummary semantic_summary_;
};

}  // namespace rc26_local_3d_planner

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<rc26_local_3d_planner::Local3DPlannerNode>());
    rclcpp::shutdown();
    return 0;
}
