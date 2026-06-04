#include "rc26_xhu_nav/local_planner/planner_core.hpp"

#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rc26_interfaces/msg/localization_health.hpp"
#include "rc26_interfaces/msg/terrain_feature_grid.hpp"
#include "rc26_interfaces/msg/xhu_local_planner_state.hpp"
#include "rc26_interfaces/msg/xhu_motion_mode_state.hpp"
#include "rc26_interfaces/msg/xhu_recovery_state.hpp"
#include "rc26_interfaces/msg/xhu_semantic_corridor.hpp"
#include "rc26_interfaces/msg/xhu_semantic_layer_summary.hpp"
#include "rc26_interfaces/msg/xhu_tracking_state.hpp"
#include "std_msgs/msg/string.hpp"

namespace rc26_xhu_nav::runtime {

namespace {

double clamp(const double value, const double lower, const double upper) {
    return std::min(std::max(value, lower), upper);
}

double normalizeAngle(double angle) {
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

double yawFromQuaternion(const geometry_msgs::msg::Quaternion& q) {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

std::vector<double> parameterArrayOrDefault(
    rclcpp::Node* node, const std::string& name, const std::vector<double>& fallback) {
    try {
        const auto values = node->get_parameter(name).as_double_array();
        return values.empty() ? fallback : values;
    } catch (const rclcpp::exceptions::InvalidParameterTypeException&) {
        return fallback;
    }
}

}  // namespace

class XhuMotionRuntime : public rclcpp::Node {
public:
    XhuMotionRuntime()
        : rclcpp::Node("xhu_motion_runtime"),
          tf_buffer_(std::make_shared<tf2_ros::Buffer>(get_clock())),
          tf_listener_(*tf_buffer_) {
        declare_parameter<std::string>("odom_topic", "odom");
        declare_parameter<std::string>("map_frame", "map");
        declare_parameter<std::string>("base_frame", "base_link");
        declare_parameter<double>("control_frequency_hz", 20.0);
        declare_parameter<double>("goal_tolerance_xy", 0.15);
        declare_parameter<double>("goal_tolerance_yaw", 0.30);
        declare_parameter<double>("mode_state_timeout_sec", 1.0);
        declare_parameter<double>("local_planner_wait_timeout_sec", 2.0);
        declare_parameter<double>("local_planner_recovery_timeout_sec", 3.0);
        declare_parameter<double>("default_max_linear_accel", 0.6);
        declare_parameter<double>("default_max_angular_accel", 0.8);
        declare_parameter<std::string>("robot_geometry_file", "");
        declare_parameter<std::string>("robot_geometry_profile", "compact");
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

        const auto odom_topic = get_parameter("odom_topic").as_string();
        map_frame_ = get_parameter("map_frame").as_string();
        base_frame_ = get_parameter("base_frame").as_string();
        goal_tolerance_xy_ = std::max(0.02, get_parameter("goal_tolerance_xy").as_double());
        goal_tolerance_yaw_ = std::max(0.05, get_parameter("goal_tolerance_yaw").as_double());
        mode_state_timeout_sec_ =
            std::max(0.1, get_parameter("mode_state_timeout_sec").as_double());
        wait_timeout_sec_ =
            std::max(0.2, get_parameter("local_planner_wait_timeout_sec").as_double());
        recovery_timeout_sec_ =
            std::max(0.2, get_parameter("local_planner_recovery_timeout_sec").as_double());
        default_max_linear_accel_ =
            std::max(0.05, get_parameter("default_max_linear_accel").as_double());
        default_max_angular_accel_ =
            std::max(0.05, get_parameter("default_max_angular_accel").as_double());

        rc26_xhu_nav::local_planner::PlannerConfig planner_config;
        planner_config.lookahead_distance_m =
            std::max(0.2, get_parameter("lookahead_distance_m").as_double());
        planner_config.horizon_sec = std::max(0.4, get_parameter("horizon_sec").as_double());
        planner_config.integration_step_sec =
            std::max(0.05, get_parameter("integration_step_sec").as_double());
        planner_config.terrain_obstacle_threshold =
            std::clamp(get_parameter("terrain_obstacle_threshold").as_double(), 0.0, 1.0);
        planner_config.terrain_drop_threshold =
            std::clamp(get_parameter("terrain_drop_threshold").as_double(), 0.0, 1.0);
        planner_config.recovery_heading_threshold_rad =
            std::max(0.1, get_parameter("recovery_heading_threshold_rad").as_double());
        planner_config.recovery_angular_speed =
            std::max(0.1, get_parameter("recovery_angular_speed").as_double());
        planner_config.slow_zone_speed_scale =
            std::clamp(get_parameter("slow_zone_speed_scale").as_double(), 0.1, 1.0);
        planner_config.path_alignment_weight =
            std::max(0.0, get_parameter("path_alignment_weight").as_double());
        planner_config.heading_alignment_weight =
            std::max(0.0, get_parameter("heading_alignment_weight").as_double());
        planner_config.speed_preference_weight =
            std::max(0.0, get_parameter("speed_preference_weight").as_double());
        planner_config.angular_effort_weight =
            std::max(0.0, get_parameter("angular_effort_weight").as_double());
        planner_config.clearance_weight =
            std::max(0.0, get_parameter("clearance_weight").as_double());
        planner_config.stop_envelope_half_width_m =
            std::max(0.0, get_parameter("stop_envelope_half_width_m").as_double());
        planner_config.sample_linear_speeds =
            parameterArrayOrDefault(this, "sample_linear_speeds", planner_config.sample_linear_speeds);
        planner_config.sample_angular_speeds =
            parameterArrayOrDefault(this, "sample_angular_speeds", planner_config.sample_angular_speeds);

        const auto robot_geometry_file = get_parameter("robot_geometry_file").as_string();
        const auto robot_geometry_profile = get_parameter("robot_geometry_profile").as_string();
        if (!robot_geometry_file.empty()) {
            std::string error;
            const auto geometry = rc26_xhu_nav::local_planner::PlannerCore::loadRobotGeometryProfile(
                robot_geometry_file, robot_geometry_profile, error);
            if (geometry) {
                planner_config.stop_envelope_half_width_m = std::max(
                    planner_config.stop_envelope_half_width_m,
                    geometry->stop_envelope_half_width_m);
            } else {
                RCLCPP_WARN(get_logger(), "runtime geometry load failed: %s", error.c_str());
            }
        }
        planner_.setConfig(planner_config);

        corridor_sub_ = create_subscription<rc26_interfaces::msg::XhuSemanticCorridor>(
            "/xhu_nav/corridor_cmd", 10,
            std::bind(&XhuMotionRuntime::onCorridor, this, std::placeholders::_1));
        mode_sub_ = create_subscription<rc26_interfaces::msg::XhuMotionModeState>(
            "/xhu_nav/motion_mode_state", 10,
            std::bind(&XhuMotionRuntime::onModeState, this, std::placeholders::_1));
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, rclcpp::SensorDataQoS(),
            std::bind(&XhuMotionRuntime::onOdom, this, std::placeholders::_1));
        loc_health_sub_ = create_subscription<rc26_interfaces::msg::LocalizationHealth>(
            "/localization/health", 10,
            [this](const rc26_interfaces::msg::LocalizationHealth::SharedPtr msg) {
                if (msg) {
                    loc_health_level_ = msg->level;
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

        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        lookahead_pub_ = create_publisher<nav_msgs::msg::Path>("/xhu_nav/lookahead_path", 10);
        tracking_pub_ =
            create_publisher<rc26_interfaces::msg::XhuTrackingState>("/xhu_nav/tracking_state", 10);
        semantic_gate_pub_ = create_publisher<std_msgs::msg::String>("/xhu_nav/semantic_gate", 10);
        local_state_pub_ = create_publisher<rc26_interfaces::msg::XhuLocalPlannerState>(
            "/xhu_nav/local_planner_state", 10);
        recovery_pub_ = create_publisher<rc26_interfaces::msg::XhuRecoveryState>(
            "/xhu_nav/recovery_state", 10);

        last_control_stamp_ = now();
        const auto control_frequency_hz =
            std::max(5.0, get_parameter("control_frequency_hz").as_double());
        control_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::duration<double>(1.0 / control_frequency_hz)),
            std::bind(&XhuMotionRuntime::controlLoop, this));
    }

private:
    void onCorridor(const rc26_interfaces::msg::XhuSemanticCorridor::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (!msg || msg->path.poses.empty()) {
            return;
        }
        active_corridor_ = *msg;
        corridor_start_stamp_ = now();
        active_status_since_.reset();
        recovery_started_at_.reset();
        last_cmd_ = geometry_msgs::msg::Twist{};
        publishTrackingLocked("PASS", false, "corridor accepted", 0.0F, 0.0F, 0.0F);
    }

    void onModeState(const rc26_interfaces::msg::XhuMotionModeState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (!msg) {
            return;
        }
        mode_state_ = *msg;
        mode_state_stamp_ = (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0)
                                ? now()
                                : rclcpp::Time(msg->header.stamp);
        has_mode_state_ = true;
    }

    void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (!msg) {
            return;
        }
        last_odom_ = *msg;
        has_odom_ = true;
        last_odom_stamp_ = (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0)
                               ? now()
                               : rclcpp::Time(msg->header.stamp);
    }

    bool queryRobotPose(double& x, double& y, double& yaw) const {
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

    void publishTrackingLocked(const std::string& status, const bool terminal,
                               const std::string& reason, const float heading_error,
                               const float distance_to_goal, const float best_score) {
        if (!active_corridor_) {
            return;
        }

        rc26_interfaces::msg::XhuTrackingState tracking;
        tracking.header.stamp = now();
        tracking.corridor_id = active_corridor_->corridor_id;
        tracking.edge_id = active_corridor_->edge_id;
        tracking.status = status;
        tracking.terminal = terminal;
        tracking.cross_track_error = best_score;
        tracking.heading_error = heading_error;
        tracking.distance_to_goal = distance_to_goal;
        tracking.cmd_vx = static_cast<float>(last_cmd_.linear.x);
        tracking.cmd_vy = static_cast<float>(last_cmd_.linear.y);
        tracking.cmd_wz = static_cast<float>(last_cmd_.angular.z);
        tracking.reason = reason;
        tracking_pub_->publish(tracking);

        std_msgs::msg::String gate;
        gate.data = status;
        semantic_gate_pub_->publish(gate);
    }

    void publishZeroLocked() {
        last_cmd_ = geometry_msgs::msg::Twist{};
        cmd_pub_->publish(last_cmd_);
    }

    void publishPlannerStateLocked(const rc26_xhu_nav::local_planner::PlannerResult& result) {
        if (!active_corridor_) {
            return;
        }

        rc26_interfaces::msg::XhuLocalPlannerState state_msg;
        state_msg.header.stamp = now();
        state_msg.header.frame_id = map_frame_;
        state_msg.corridor_id = active_corridor_->corridor_id;
        state_msg.edge_id = active_corridor_->edge_id;
        state_msg.status = result.status;
        state_msg.terminal = false;
        state_msg.observe_only = false;
        state_msg.semantic_revision = has_semantic_summary_ ? semantic_summary_.revision : 0U;
        state_msg.cmd_vx = static_cast<float>(result.cmd_vx);
        state_msg.cmd_vy = static_cast<float>(result.cmd_vy);
        state_msg.cmd_wz = static_cast<float>(result.cmd_wz);
        state_msg.best_score = static_cast<float>(result.best_score);
        state_msg.clearance_margin_m = static_cast<float>(result.clearance_margin_m);
        state_msg.reason = result.reason;
        local_state_pub_->publish(state_msg);

        auto recovery_msg = result.recovery_state;
        recovery_msg.header.stamp = state_msg.header.stamp;
        recovery_msg.header.frame_id = map_frame_;
        recovery_msg.corridor_id = active_corridor_->corridor_id;
        recovery_msg.edge_id = active_corridor_->edge_id;
        if (recovery_msg.status.empty()) {
            recovery_msg.recovery_name = "none";
            recovery_msg.status = "IDLE";
            recovery_msg.reason = result.reason;
            recovery_msg.terminal = false;
        } else if (recovery_started_at_) {
            recovery_msg.status = "RUNNING";
            recovery_msg.elapsed_sec =
                static_cast<float>((now() - *recovery_started_at_).seconds());
        }
        recovery_pub_->publish(recovery_msg);

        if (!result.preview_path.poses.empty()) {
            lookahead_pub_->publish(result.preview_path);
        }
    }

    void controlLoop() {
        const auto stamp = now();
        const double raw_dt = (stamp - last_control_stamp_).seconds();
        const double dt = raw_dt > 1e-3 ? raw_dt : 0.05;
        last_control_stamp_ = stamp;

        std::lock_guard<std::mutex> lock(data_mutex_);
        if (!active_corridor_) {
            publishZeroLocked();
            return;
        }

        const bool mode_state_fresh =
            has_mode_state_ && (stamp - mode_state_stamp_).seconds() <= mode_state_timeout_sec_;
        const bool mode_hold = has_mode_state_ && mode_state_.active_mode == "hold";
        const bool stop_required = has_mode_state_ && mode_state_.stop_required;
        const bool loc_red =
            loc_health_level_ >= rc26_interfaces::msg::LocalizationHealth::RED;

        if (!mode_state_fresh || mode_hold || stop_required || loc_red) {
            publishZeroLocked();
            publishTrackingLocked("HOLD", false, "mode/localization hold", 0.0F, 0.0F, 0.0F);
            return;
        }

        double robot_x = 0.0;
        double robot_y = 0.0;
        double robot_yaw = 0.0;
        if (!queryRobotPose(robot_x, robot_y, robot_yaw)) {
            publishZeroLocked();
            publishTrackingLocked("HOLD", false, "robot pose unavailable", 0.0F, 0.0F, 0.0F);
            return;
        }

        const auto& goal_pose = active_corridor_->path.poses.back().pose;
        const double distance_to_goal = std::hypot(goal_pose.position.x - robot_x,
                                                   goal_pose.position.y - robot_y);
        const double heading_to_goal = normalizeAngle(
            yawFromQuaternion(goal_pose.orientation) - robot_yaw);
        const bool goal_reached =
            distance_to_goal <= goal_tolerance_xy_ &&
            (!active_corridor_->stop_at_end || std::abs(heading_to_goal) <= goal_tolerance_yaw_);
        if (goal_reached) {
            publishZeroLocked();
            publishTrackingLocked("PASS", true, "corridor completed",
                                  static_cast<float>(heading_to_goal),
                                  static_cast<float>(distance_to_goal), 0.0F);
            active_corridor_.reset();
            active_status_since_.reset();
            recovery_started_at_.reset();
            return;
        }

        rc26_xhu_nav::local_planner::PlannerInput input;
        input.has_pose = true;
        input.robot_x = robot_x;
        input.robot_y = robot_y;
        input.robot_yaw = robot_yaw;
        input.has_mode_state = has_mode_state_;
        input.mode_state = mode_state_;
        input.has_terrain_grid = has_terrain_grid_;
        input.terrain_grid = terrain_grid_;
        input.has_semantic_summary = has_semantic_summary_;
        input.semantic_summary = semantic_summary_;
        input.corridor = *active_corridor_;
        input.current_vx = last_cmd_.linear.x;
        input.current_vy = last_cmd_.linear.y;
        input.current_wz = last_cmd_.angular.z;

        const auto result = planner_.plan(input);
        if (result.status != last_status_) {
            last_status_ = result.status;
            active_status_since_ = stamp;
            if (result.status == "RECOVERY_RUNNING") {
                recovery_started_at_ = stamp;
            } else if (result.status != "RECOVERY_RUNNING") {
                recovery_started_at_.reset();
            }
        }

        publishPlannerStateLocked(result);
        if (result.status == "PASS") {
            const double max_dv = default_max_linear_accel_ * dt;
            const double max_dw = default_max_angular_accel_ * dt;
            double dvx = result.cmd_vx - last_cmd_.linear.x;
            double dvy = result.cmd_vy - last_cmd_.linear.y;
            const double dv_mag = std::hypot(dvx, dvy);
            if (dv_mag > max_dv && dv_mag > 1e-6) {
                const double scale = max_dv / dv_mag;
                dvx *= scale;
                dvy *= scale;
            }
            last_cmd_.linear.x += dvx;
            last_cmd_.linear.y += dvy;
            last_cmd_.angular.z = clamp(result.cmd_wz, last_cmd_.angular.z - max_dw,
                                        last_cmd_.angular.z + max_dw);
            cmd_pub_->publish(last_cmd_);
            publishTrackingLocked("PASS", false, result.reason,
                                  static_cast<float>(heading_to_goal),
                                  static_cast<float>(distance_to_goal),
                                  static_cast<float>(result.best_score));
            return;
        }

        if (result.status == "RECOVERY_RUNNING") {
            last_cmd_.linear.x = 0.0;
            last_cmd_.linear.y = 0.0;
            last_cmd_.angular.z = result.cmd_wz;
            cmd_pub_->publish(last_cmd_);
            publishTrackingLocked("RECOVERY_RUNNING", false, result.reason,
                                  static_cast<float>(heading_to_goal),
                                  static_cast<float>(distance_to_goal),
                                  static_cast<float>(result.best_score));
            if (recovery_started_at_ &&
                (stamp - *recovery_started_at_).seconds() > recovery_timeout_sec_) {
                publishZeroLocked();
                publishTrackingLocked("REPLAN", false, "recovery timeout",
                                      static_cast<float>(heading_to_goal),
                                      static_cast<float>(distance_to_goal),
                                      static_cast<float>(result.best_score));
            }
            return;
        }

        publishZeroLocked();
        if (result.status == "WAITING_ON_BLOCK") {
            publishTrackingLocked("WAITING_ON_BLOCK", false, result.reason,
                                  static_cast<float>(heading_to_goal),
                                  static_cast<float>(distance_to_goal),
                                  static_cast<float>(result.best_score));
            if (active_status_since_ &&
                (stamp - *active_status_since_).seconds() > wait_timeout_sec_) {
                publishTrackingLocked("REPLAN", false, "waiting_on_block timeout",
                                      static_cast<float>(heading_to_goal),
                                      static_cast<float>(distance_to_goal),
                                      static_cast<float>(result.best_score));
            }
            return;
        }

        if (result.status == "LOCAL_COLLISION_BLOCKED") {
            publishTrackingLocked("LOCAL_COLLISION_BLOCKED", false, result.reason,
                                  static_cast<float>(heading_to_goal),
                                  static_cast<float>(distance_to_goal),
                                  static_cast<float>(result.best_score));
            return;
        }

        publishTrackingLocked("HOLD", false, result.reason,
                              static_cast<float>(heading_to_goal),
                              static_cast<float>(distance_to_goal),
                              static_cast<float>(result.best_score));
    }

    mutable std::mutex data_mutex_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    rclcpp::Subscription<rc26_interfaces::msg::XhuSemanticCorridor>::SharedPtr corridor_sub_;
    rclcpp::Subscription<rc26_interfaces::msg::XhuMotionModeState>::SharedPtr mode_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<rc26_interfaces::msg::LocalizationHealth>::SharedPtr loc_health_sub_;
    rclcpp::Subscription<rc26_interfaces::msg::TerrainFeatureGrid>::SharedPtr terrain_sub_;
    rclcpp::Subscription<rc26_interfaces::msg::XhuSemanticLayerSummary>::SharedPtr semantic_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr lookahead_pub_;
    rclcpp::Publisher<rc26_interfaces::msg::XhuTrackingState>::SharedPtr tracking_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr semantic_gate_pub_;
    rclcpp::Publisher<rc26_interfaces::msg::XhuLocalPlannerState>::SharedPtr local_state_pub_;
    rclcpp::Publisher<rc26_interfaces::msg::XhuRecoveryState>::SharedPtr recovery_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    std::optional<rc26_interfaces::msg::XhuSemanticCorridor> active_corridor_;
    std::optional<rclcpp::Time> active_status_since_;
    std::optional<rclcpp::Time> recovery_started_at_;
    std::string last_status_{"IDLE"};
    bool has_mode_state_{false};
    bool has_odom_{false};
    bool has_terrain_grid_{false};
    bool has_semantic_summary_{false};
    rc26_interfaces::msg::XhuMotionModeState mode_state_;
    nav_msgs::msg::Odometry last_odom_;
    rc26_interfaces::msg::TerrainFeatureGrid terrain_grid_;
    rc26_interfaces::msg::XhuSemanticLayerSummary semantic_summary_;
    rclcpp::Time mode_state_stamp_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_odom_stamp_{0, 0, RCL_ROS_TIME};
    rclcpp::Time corridor_start_stamp_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_control_stamp_{0, 0, RCL_ROS_TIME};
    uint8_t loc_health_level_{rc26_interfaces::msg::LocalizationHealth::GREEN};
    geometry_msgs::msg::Twist last_cmd_;
    std::string map_frame_{"map"};
    std::string base_frame_{"base_link"};
    rc26_xhu_nav::local_planner::PlannerCore planner_;
    double goal_tolerance_xy_{0.15};
    double goal_tolerance_yaw_{0.30};
    double mode_state_timeout_sec_{1.0};
    double wait_timeout_sec_{2.0};
    double recovery_timeout_sec_{3.0};
    double default_max_linear_accel_{0.6};
    double default_max_angular_accel_{0.8};
};

}  // namespace rc26_xhu_nav::runtime

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<rc26_xhu_nav::runtime::XhuMotionRuntime>());
    rclcpp::shutdown();
    return 0;
}
