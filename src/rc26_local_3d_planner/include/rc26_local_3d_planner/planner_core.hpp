#pragma once

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rc26_interfaces/msg/terrain_feature_grid.hpp>
#include <rc26_interfaces/msg/xhu_local_planner_state.hpp>
#include <rc26_interfaces/msg/xhu_motion_mode_state.hpp>
#include <rc26_interfaces/msg/xhu_recovery_state.hpp>
#include <rc26_interfaces/msg/xhu_semantic_corridor.hpp>
#include <rc26_interfaces/msg/xhu_semantic_layer_summary.hpp>

#include <optional>
#include <string>
#include <vector>

namespace rc26_local_3d_planner {

struct RobotGeometryProfile {
    std::string name;
    double half_length_m{0.0};
    double half_width_m{0.0};
    double height_m{0.0};
    double stop_envelope_half_width_m{0.0};
};

struct PlannerConfig {
    double lookahead_distance_m{0.7};
    double horizon_sec{1.2};
    double integration_step_sec{0.1};
    double terrain_obstacle_threshold{0.6};
    double terrain_drop_threshold{0.8};
    double recovery_heading_threshold_rad{0.55};
    double recovery_angular_speed{0.7};
    double slow_zone_speed_scale{0.6};
    double path_alignment_weight{2.4};
    double heading_alignment_weight{1.8};
    double speed_preference_weight{0.8};
    double angular_effort_weight{0.5};
    double clearance_weight{2.2};
    double stop_envelope_half_width_m{0.24};
    std::vector<double> sample_linear_speeds{0.0, 0.15, 0.30, 0.45};
    std::vector<double> sample_angular_speeds{-0.8, -0.4, 0.0, 0.4, 0.8};
};

struct PlannerInput {
    bool has_pose{false};
    double robot_x{0.0};
    double robot_y{0.0};
    double robot_yaw{0.0};
    double current_vx{0.0};
    double current_wz{0.0};
    bool has_mode_state{false};
    bool has_terrain_grid{false};
    bool has_semantic_summary{false};
    rc26_interfaces::msg::XhuSemanticCorridor corridor;
    rc26_interfaces::msg::XhuMotionModeState mode_state;
    rc26_interfaces::msg::TerrainFeatureGrid terrain_grid;
    rc26_interfaces::msg::XhuSemanticLayerSummary semantic_summary;
};

struct PlannerResult {
    bool has_solution{false};
    bool blocked_by_keepout{false};
    bool blocked_by_terrain{false};
    bool should_rotate_recovery{false};
    double cmd_vx{0.0};
    double cmd_vy{0.0};
    double cmd_wz{0.0};
    double best_score{0.0};
    double clearance_margin_m{0.0};
    std::string status;
    std::string reason;
    nav_msgs::msg::Path preview_path;
    rc26_interfaces::msg::XhuRecoveryState recovery_state;
};

class PlannerCore {
public:
    explicit PlannerCore(PlannerConfig config = PlannerConfig{});

    void setConfig(const PlannerConfig& config);
    const PlannerConfig& config() const { return config_; }

    PlannerResult plan(const PlannerInput& input) const;

    static std::optional<RobotGeometryProfile> loadRobotGeometryProfile(
        const std::string& geometry_file, const std::string& requested_profile,
        std::string& error);

private:
    struct SimState {
        double x{0.0};
        double y{0.0};
        double yaw{0.0};
    };

    static double normalizeAngle(double angle);
    static double clamp(double value, double lower, double upper);
    static double yawFromQuaternion(const geometry_msgs::msg::Quaternion& q);
    static double terrainProbabilityAt(
        const rc26_interfaces::msg::TerrainFeatureGrid& grid, const std::vector<float>& values,
        double x, double y);
    static double pathDistanceToNearest(
        const nav_msgs::msg::Path& path, double x, double y, std::size_t hint_index,
        std::size_t* nearest_index_out);
    static std::size_t findLookaheadIndex(
        const nav_msgs::msg::Path& path, std::size_t start_index, double lookahead_distance);
    static nav_msgs::msg::Path makePreviewPath(
        const std::vector<SimState>& states, const rclcpp::Time& stamp,
        const std::string& frame_id);

    PlannerConfig config_;
};

}  // namespace rc26_local_3d_planner
