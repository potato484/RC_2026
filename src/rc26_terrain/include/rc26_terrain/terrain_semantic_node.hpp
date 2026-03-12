#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "rclcpp/rclcpp.hpp"
#include "rc26_interfaces/msg/mf_kfs_state.hpp"
#include "rc26_interfaces/msg/terrain_feature_grid.hpp"
#include "rc26_terrain/safety_guard.hpp"
#include "rc26_terrain/tf_chain_validator.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace rc26_terrain {

class TerrainSemanticNode : public rclcpp::Node {
public:
    explicit TerrainSemanticNode(const rclcpp::NodeOptions& options);

private:
    void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg);
    void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);
    void healthTimerCallback();
    std::optional<tf2::Transform> getTransform(const std::string& target_frame,
                                               const std::string& source_frame,
                                               const rclcpp::Time& time);
    void initGrid();
    void estimateCellHeights(double stamp_sec);
    void classifyAndUpdate(double stamp_sec);
    void publishOutputs(const rclcpp::Time& stamp, double base_x, double base_y,
                        double base_z, double cos_yaw, double sin_yaw);
    void publishVirtualFence(const rclcpp::Time& stamp, double base_x, double base_y,
                             double base_z, double cos_yaw, double sin_yaw);
    void publishEmergencyStop(const rclcpp::Time& stamp) const;
    bool isThrottleReady(const rclcpp::Time& stamp, const rclcpp::Time& last_pub_time,
                         double publish_hz) const;
    SafetyGuardInput buildSafetyGuardInput(const rclcpp::Time& now) const;
    void syncSafetyGuardState(const SafetyGuardDecision& decision);
    TfValidationReport validateTfChain(const rclcpp::Time& now) const;
    void publishSpeedLimitValue(const rclcpp::Time& stamp, float v_limit, bool force = false);
    void updateKfsOccupied(const rc26_interfaces::msg::MfKfsState& msg);
    bool loadMfGridLayout(const std::string& path);
    bool sanitizeAndValidateCloud(pcl::PointCloud<pcl::PointXYZI>& cloud,
                                   const std::string& name,
                                   std::string& reason) const;
    void publishDiagnostics(const rclcpp::Time& stamp, int level,
                            const std::string& message) const;

    // 参数
    std::string input_cloud_topic_;
    std::string odom_topic_;
    std::string target_frame_;
    std::string base_frame_;
    std::string output_obstacles_topic_;
    std::string output_drop_topic_;
    std::string output_climbable_topic_;
    std::string diagnostics_topic_;
    std::string base_ground_stable_topic_{"base_ground/stable_terrain"};
    std::string mf_kfs_state_topic_{"/mf_kfs_state"};
    std::string mf_grid_layout_file_{""};
    double kfs_min_confidence_{0.6};

    double tf_timeout_sec_{0.2};
    double cloud_timeout_sec_{0.7};
    double odom_timeout_sec_{0.7};
    double startup_grace_sec_{1.0};
    double perception_radius_m_{3.2};
    double grid_resolution_m_{0.1};
    double voxel_leaf_size_m_{0.05};

    double min_rel_z_m_{-1.5};
    double max_rel_z_m_{0.5};
    double dis_ratio_z_{0.1};

    int min_points_per_cell_{5};
    double ground_quantile_{0.25};
    double top_quantile_{0.95};
    double ground_ema_alpha_{0.6};
    bool enable_sigma_based_alpha_{true};
    double sigma_eps_m_{0.01};
    double sigma_alpha_min_{0.05};
    double sigma_alpha_max_{0.6};

    double h_climb_m_{0.30};
    double h_obstacle_m_{0.33};
    double h_drop_m_{0.15};
    double climbable_min_dz_m_{0.05};

    std::string unknown_policy_{"aggressive"};  // aggressive=忽略 unknown, conservative=unknown 视为风险
    std::string unknown_output_{"drop"};        // unknown 输出到 drop 或 obstacles

    bool enable_hysteresis_{true};
    int score_max_{10};
    int score_inc_{2};
    int score_dec_{1};
    int obstacle_on_score_{6};
    int obstacle_off_score_{3};
    int drop_on_score_{6};
    int drop_off_score_{3};
    double stale_time_sec_{0.7};
    double decay_time_sec_{2.0};  // 点云时间衰减（秒），超过此时间的历史数据将被清除

    // QoS 参数（可配置）
    int cloud_qos_depth_{5};
    std::string cloud_qos_reliability_{"best_effort"};
    std::string cloud_qos_durability_{"volatile"};
    int odom_qos_depth_{10};
    std::string odom_qos_reliability_{"reliable"};
    std::string odom_qos_durability_{"volatile"};
    int output_qos_depth_{5};
    std::string output_qos_reliability_{"best_effort"};
    std::string output_qos_durability_{"volatile"};
    int speed_limit_qos_depth_{10};
    std::string speed_limit_qos_reliability_{"reliable"};
    std::string speed_limit_qos_durability_{"volatile"};
    int diagnostics_qos_depth_{10};
    std::string diagnostics_qos_reliability_{"reliable"};
    std::string diagnostics_qos_durability_{"volatile"};
    bool enable_diagnostics_{true};

    // 失效保护
    bool enable_fail_safe_{true};
    std::string fail_safe_strategy_{"virtual_fence"};  // none=关闭, virtual_fence=虚拟围栏
    double virtual_fence_radius_m_{0.6};
    int virtual_fence_num_points_{36};
    double virtual_fence_height_m_{0.2};

    // TF 健康检测
    double tf_health_timeout_sec_{0.7};

    // 输出点云验证
    bool output_sanity_check_enable_{true};
    int output_max_points_total_{20000};
    int output_max_points_per_cloud_{15000};

    // drop 前向扇区约束
    double drop_forward_sector_deg_{180.0};
    double drop_forward_min_x_m_{0.0};

    // P0.2: neighbor modes + denoising + jump guard
    std::string obstacle_neighbor_mode_{"edge4"};
    std::string drop_neighbor_mode_{"edge8"};
    int         min_obstacle_area_cells_{2};
    int         min_drop_area_cells_{2};
    double      jump_thresh_m_{0.15};
    int         freeze_max_frames_{3};
    double      ground_ema_alpha_slow_{0.25};
    bool        enable_pitch_compensation_{true};
    bool        enable_roll_compensation_{false};
    double      stair_gate_speed_mps_{0.25};
    double      stair_pitch_gate_deg_{6.0};
    double      top_z_max_delta_m_{0.7};
    std::string terrain_features_topic_{"terrain_features"};
    bool        enable_terrain_features_pub_{false};
    double      terrain_features_publish_hz_{3.5};
    std::string terrain_speed_limit_topic_{"terrain_speed_limit"};
    bool        enable_terrain_speed_limit_pub_{true};
    double      terrain_speed_limit_publish_hz_{5.0};
    double      speed_limit_forward_look_m_{2.0};
    double      speed_limit_half_width_m_{0.4};
    double      speed_limit_v_max_mps_{2.0};
    double      speed_limit_min_mps_{0.2};
    double      speed_limit_w_slope_{1.0};
    double      speed_limit_w_roughness_{1.0};
    double      speed_limit_w_drop_{2.0};
    double      speed_limit_w_climbable_{0.8};
    double      speed_limit_k_tci_{1.0};
    double      speed_limit_emergency_drop_thresh_{0.8};

    // P0.3: latency diagnostics
    double latency_warn_ms_{12.0};
    double latency_error_ms_{20.0};
    int latency_trigger_frames_{3};
    int latency_recover_frames_{5};
    std::string latency_intervention_mode_{"virtual_fence"};
    double last_latency_ms_{0.0};
    std::string thermal_throttle_topic_{""};
    double thermal_throttle_release_sec_{0.5};

    // 栅格状态
    int half_width_{0};
    int width_{0};
    int num_cells_{0};
    std::vector<uint8_t> cell_in_radius_;
    std::vector<float> ground_z_filtered_;
    std::vector<float> top_z_;
    std::vector<double> last_seen_sec_;
    std::vector<int> obstacle_score_;
    std::vector<int> drop_score_;
    std::vector<int> freeze_count_;
    std::vector<uint8_t> obstacle_state_;
    std::vector<uint8_t> drop_state_;
    std::vector<uint8_t> kfs_occupied_state_;
    std::vector<float> sigma_h_;
    std::vector<uint16_t> density_;
    std::vector<float> slope_x_;
    std::vector<float> slope_y_;
    std::vector<float> roughness_;

    // 每帧缓存
    std::vector<std::vector<float>> cell_z_samples_;
    std::vector<int> touched_cells_;

    // ROS I/O
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_base_ground_stable_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_thermal_throttle_;
    rclcpp::Subscription<rc26_interfaces::msg::MfKfsState>::SharedPtr sub_mf_kfs_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_obstacles_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_drop_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_climbable_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diagnostics_;
    rclcpp::Publisher<rc26_interfaces::msg::TerrainFeatureGrid>::SharedPtr pub_features_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_speed_limit_;
    rclcpp::TimerBase::SharedPtr health_timer_;
    SafetyGuard safety_guard_;
    std::unique_ptr<TfChainValidator> tf_chain_validator_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

    // 运行时状态（用于诊断与失效保护）
    rclcpp::Time node_start_time_;
    rclcpp::Time last_cloud_stamp_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_odom_stamp_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_good_tf_stamp_{0, 0, RCL_ROS_TIME};
    bool received_cloud_{false};
    bool received_odom_{false};
    bool have_last_pose_{false};
    double last_base_x_{0.0};
    double last_base_y_{0.0};
    double last_base_z_{0.0};
    double last_cos_yaw_{1.0};
    double last_sin_yaw_{0.0};
    bool fail_safe_active_{false};
    std::string fail_safe_reason_;
    double last_linear_speed_mps_{0.0};
    bool base_ground_stable_{true};
    double last_roll_rad_{0.0};
    double last_pitch_rad_{0.0};
    int last_max_freeze_count_{0};
    int latency_overrun_count_{0};
    int latency_recover_count_{0};
    bool latency_intervention_active_{false};
    bool thermal_throttle_requested_{false};
    rclcpp::Time thermal_throttle_last_true_stamp_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_features_pub_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_speed_limit_pub_time_{0, 0, RCL_ROS_TIME};
    bool last_tf_validation_ok_{false};
    TfChainStatusCode last_tf_validation_code_{TfChainStatusCode::kInvalidSpecification};
    std::string last_tf_validation_message_{"未验证"};

    // 诊断统计
    int kfs_occupied_count_{0};
    int obstacle_cells_count_{0};
    int drop_cells_count_{0};
    int climbable_cells_count_{0};

    // MF 布局映射
    std::array<double, 13> mf_grid_x_{};
    std::array<double, 13> mf_grid_y_{};
    std::array<uint8_t, 13> mf_grid_valid_{};
    std::string mf_layout_team_;
};

}  // namespace rc26_terrain
