#include "rc26_terrain/terrain_semantic_node.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <limits>
#include <stdexcept>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "pcl/filters/voxel_grid.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.hpp"
#include "rmw/types.h"
#include "std_msgs/msg/bool.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/time.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "yaml-cpp/yaml.h"

namespace {

float quantileInplace(std::vector<float>& values, double q) {
    if (values.empty()) return 0.0f;
    q = std::clamp(q, 0.0, 1.0);
    const size_t n = values.size();
    const size_t idx = static_cast<size_t>(std::floor(q * static_cast<double>(n - 1)));
    std::nth_element(values.begin(),
                     values.begin() + static_cast<std::vector<float>::difference_type>(idx),
                     values.end());
    return values[idx];
}

rmw_qos_reliability_policy_t parseReliability(const std::string& v) {
    if (v == "best_effort") return RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    if (v == "reliable") return RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    if (v == "system_default") return RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT;
    throw std::invalid_argument("qos_reliability 参数仅支持: best_effort/reliable/system_default");
}

rmw_qos_durability_policy_t parseDurability(const std::string& v) {
    if (v == "volatile") return RMW_QOS_POLICY_DURABILITY_VOLATILE;
    if (v == "transient_local") return RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    if (v == "system_default") return RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT;
    throw std::invalid_argument("qos_durability 参数仅支持: volatile/transient_local/system_default");
}

rclcpp::QoS makeQoS(int depth, const std::string& reliability, const std::string& durability) {
    depth = std::max(1, depth);
    auto qos = rclcpp::QoS(rclcpp::KeepLast(depth));
    qos.reliability(parseReliability(reliability));
    qos.durability(parseDurability(durability));
    return qos;
}

}  // namespace

namespace rc26_terrain {

TerrainSemanticNode::TerrainSemanticNode(const rclcpp::NodeOptions& options)
    : Node("terrain_semantic", options) {
    node_start_time_ = this->get_clock()->now();

    // 参数声明（默认值可在 YAML 中覆盖，并支持 Remap）
    this->declare_parameter<std::string>("input_cloud_topic", "registered_scan");
    this->declare_parameter<std::string>("odom_topic", "odom");
    this->declare_parameter<std::string>("output_obstacles_topic", "terrain_obstacles");
    this->declare_parameter<std::string>("output_drop_topic", "terrain_drop");
    this->declare_parameter<std::string>("output_climbable_topic", "terrain_climbable");
    this->declare_parameter<std::string>("diagnostics_topic", "diagnostics");
    this->declare_parameter<std::string>("base_ground_stable_topic", base_ground_stable_topic_);
    this->declare_parameter<std::string>("mf_kfs_state_topic", mf_kfs_state_topic_);
    this->declare_parameter<std::string>("mf_grid_layout_file", mf_grid_layout_file_);
    this->declare_parameter<double>("kfs_min_confidence", kfs_min_confidence_);

    this->declare_parameter<std::string>("target_frame", "odom");
    this->declare_parameter<std::string>("base_frame", "base_link");

    // TF 查询超时（秒）
    this->declare_parameter<double>("tf_timeout_sec", tf_timeout_sec_);

    // 输入健康检测（用于失效保护）
    this->declare_parameter<double>("cloud_timeout_sec", cloud_timeout_sec_);
    this->declare_parameter<double>("odom_timeout_sec", odom_timeout_sec_);
    this->declare_parameter<double>("startup_grace_sec", startup_grace_sec_);

    // 栅格与滤波参数
    this->declare_parameter<double>("perception_radius_m", perception_radius_m_);
    this->declare_parameter<double>("grid_resolution_m", grid_resolution_m_);
    this->declare_parameter<double>("voxel_leaf_size_m", voxel_leaf_size_m_);
    this->declare_parameter<double>("min_rel_z_m", min_rel_z_m_);
    this->declare_parameter<double>("max_rel_z_m", max_rel_z_m_);
    this->declare_parameter<double>("dis_ratio_z", dis_ratio_z_);

    // 地面/高度统计
    this->declare_parameter<int>("min_points_per_cell", min_points_per_cell_);
    this->declare_parameter<double>("ground_quantile", ground_quantile_);
    this->declare_parameter<double>("top_quantile", top_quantile_);
    this->declare_parameter<double>("ground_ema_alpha", ground_ema_alpha_);
    this->declare_parameter<bool>("enable_sigma_based_alpha", enable_sigma_based_alpha_);
    this->declare_parameter<double>("sigma_eps_m", sigma_eps_m_);
    this->declare_parameter<double>("sigma_alpha_min", sigma_alpha_min_);
    this->declare_parameter<double>("sigma_alpha_max", sigma_alpha_max_);

    // 语义阈值
    this->declare_parameter<double>("h_climb_m", h_climb_m_);
    this->declare_parameter<double>("h_obstacle_m", h_obstacle_m_);
    this->declare_parameter<double>("h_drop_m", h_drop_m_);
    this->declare_parameter<double>("climbable_min_dz_m", climbable_min_dz_m_);

    // Unknown 策略：保守模式将 Unknown 视为风险（输出到 obstacles 或 drop）
    this->declare_parameter<std::string>("unknown_policy", unknown_policy_);
    this->declare_parameter<std::string>("unknown_output", unknown_output_);

    // 迟滞（防闪烁）
    this->declare_parameter<bool>("enable_hysteresis", enable_hysteresis_);
    this->declare_parameter<int>("score_max", score_max_);
    this->declare_parameter<int>("score_inc", score_inc_);
    this->declare_parameter<int>("score_dec", score_dec_);
    this->declare_parameter<int>("obstacle_on_score", obstacle_on_score_);
    this->declare_parameter<int>("obstacle_off_score", obstacle_off_score_);
    this->declare_parameter<int>("drop_on_score", drop_on_score_);
    this->declare_parameter<int>("drop_off_score", drop_off_score_);
    this->declare_parameter<double>("stale_time_sec", stale_time_sec_);
    this->declare_parameter<double>("decay_time_sec", decay_time_sec_);

    // QoS（可配置）
    this->declare_parameter<int>("cloud_qos_depth", cloud_qos_depth_);
    this->declare_parameter<std::string>("cloud_qos_reliability", cloud_qos_reliability_);
    this->declare_parameter<std::string>("cloud_qos_durability", cloud_qos_durability_);
    this->declare_parameter<int>("odom_qos_depth", odom_qos_depth_);
    this->declare_parameter<std::string>("odom_qos_reliability", odom_qos_reliability_);
    this->declare_parameter<std::string>("odom_qos_durability", odom_qos_durability_);
    this->declare_parameter<int>("output_qos_depth", output_qos_depth_);
    this->declare_parameter<std::string>("output_qos_reliability", output_qos_reliability_);
    this->declare_parameter<std::string>("output_qos_durability", output_qos_durability_);
    this->declare_parameter<int>("speed_limit_qos_depth", speed_limit_qos_depth_);
    this->declare_parameter<std::string>("speed_limit_qos_reliability", speed_limit_qos_reliability_);
    this->declare_parameter<std::string>("speed_limit_qos_durability", speed_limit_qos_durability_);
    this->declare_parameter<int>("diagnostics_qos_depth", diagnostics_qos_depth_);
    this->declare_parameter<std::string>("diagnostics_qos_reliability", diagnostics_qos_reliability_);
    this->declare_parameter<std::string>("diagnostics_qos_durability", diagnostics_qos_durability_);
    this->declare_parameter<bool>("enable_diagnostics", enable_diagnostics_);

    // 失效保护：虚拟围栏（在近场生成障碍墙，阻止盲区冲出/冲撞）
    this->declare_parameter<bool>("enable_fail_safe", enable_fail_safe_);
    this->declare_parameter<std::string>("fail_safe_strategy", fail_safe_strategy_);
    this->declare_parameter<double>("virtual_fence_radius_m", virtual_fence_radius_m_);
    this->declare_parameter<int>("virtual_fence_num_points", virtual_fence_num_points_);
    this->declare_parameter<double>("virtual_fence_height_m", virtual_fence_height_m_);

    // TF 健康检测
    this->declare_parameter<double>("tf_health_timeout_sec", tf_health_timeout_sec_);

    // 输出点云验证
    this->declare_parameter<bool>("output_sanity_check_enable", output_sanity_check_enable_);
    this->declare_parameter<int>("output_max_points_total", output_max_points_total_);
    this->declare_parameter<int>("output_max_points_per_cloud", output_max_points_per_cloud_);

    // drop 前向扇区约束
    this->declare_parameter<double>("drop_forward_sector_deg", drop_forward_sector_deg_);
    this->declare_parameter<double>("drop_forward_min_x_m", drop_forward_min_x_m_);

    // P0.2: neighbor modes + denoising + jump guard
    this->declare_parameter<std::string>("obstacle_neighbor_mode", obstacle_neighbor_mode_);
    this->declare_parameter<std::string>("drop_neighbor_mode",     drop_neighbor_mode_);
    this->declare_parameter<int>        ("min_obstacle_area_cells",min_obstacle_area_cells_);
    this->declare_parameter<int>        ("min_drop_area_cells",    min_drop_area_cells_);
    this->declare_parameter<double>     ("jump_thresh_m",          jump_thresh_m_);
    this->declare_parameter<int>        ("freeze_max_frames",      freeze_max_frames_);
    this->declare_parameter<double>     ("ground_ema_alpha_slow",  ground_ema_alpha_slow_);
    this->declare_parameter<bool>       ("enable_pitch_compensation", enable_pitch_compensation_);
    this->declare_parameter<bool>       ("enable_roll_compensation", enable_roll_compensation_);
    this->declare_parameter<std::string>("roll_compensation_mode", roll_compensation_mode_);
    this->declare_parameter<double>     ("roll_gate_deg", roll_gate_deg_);
    this->declare_parameter<bool>       ("include_kfs_in_obstacles", include_kfs_in_obstacles_);
    this->declare_parameter<bool>       ("publish_kfs_obstacles_debug", publish_kfs_obstacles_debug_);
    this->declare_parameter<std::string>("output_kfs_obstacles_debug_topic",
                                         output_kfs_obstacles_debug_topic_);
    this->declare_parameter<double>     ("stair_gate_speed_mps",   stair_gate_speed_mps_);
    this->declare_parameter<double>     ("stair_pitch_gate_deg",   stair_pitch_gate_deg_);
    this->declare_parameter<double>     ("top_z_max_delta_m",      top_z_max_delta_m_);
    this->declare_parameter<std::string>("terrain_features_topic", terrain_features_topic_);
    this->declare_parameter<bool>       ("enable_terrain_features_pub", enable_terrain_features_pub_);
    this->declare_parameter<double>     ("terrain_features_publish_hz", terrain_features_publish_hz_);
    this->declare_parameter<std::string>("terrain_speed_limit_topic", terrain_speed_limit_topic_);
    this->declare_parameter<bool>       ("enable_terrain_speed_limit_pub", enable_terrain_speed_limit_pub_);
    this->declare_parameter<double>     ("terrain_speed_limit_publish_hz", terrain_speed_limit_publish_hz_);
    this->declare_parameter<double>     ("speed_limit_forward_look_m", speed_limit_forward_look_m_);
    this->declare_parameter<double>     ("speed_limit_half_width_m", speed_limit_half_width_m_);
    this->declare_parameter<double>     ("speed_limit_v_max_mps", speed_limit_v_max_mps_);
    this->declare_parameter<double>     ("speed_limit_min_mps", speed_limit_min_mps_);
    this->declare_parameter<double>     ("speed_limit_w_slope", speed_limit_w_slope_);
    this->declare_parameter<double>     ("speed_limit_w_roughness", speed_limit_w_roughness_);
    this->declare_parameter<double>     ("speed_limit_w_drop", speed_limit_w_drop_);
    this->declare_parameter<double>     ("speed_limit_w_climbable", speed_limit_w_climbable_);
    this->declare_parameter<double>     ("speed_limit_k_tci", speed_limit_k_tci_);
    this->declare_parameter<double>     ("speed_limit_emergency_drop_thresh",
                                         speed_limit_emergency_drop_thresh_);
    this->declare_parameter<bool>       ("enable_risk_model", enable_risk_model_);
    this->declare_parameter<std::string>("risk_model_file", risk_model_file_);

    // P0.3: latency diagnostics
    this->declare_parameter<double>     ("latency_warn_ms",        latency_warn_ms_);
    this->declare_parameter<double>     ("latency_error_ms",       latency_error_ms_);
    this->declare_parameter<int>        ("latency_trigger_frames", latency_trigger_frames_);
    this->declare_parameter<int>        ("latency_recover_frames", latency_recover_frames_);
    this->declare_parameter<std::string>("latency_intervention_mode", latency_intervention_mode_);
    this->declare_parameter<std::string>("thermal_throttle_topic", thermal_throttle_topic_);
    this->declare_parameter<double>     ("thermal_throttle_release_sec", thermal_throttle_release_sec_);

    // 读取参数
    this->get_parameter("input_cloud_topic", input_cloud_topic_);
    this->get_parameter("odom_topic", odom_topic_);
    this->get_parameter("output_obstacles_topic", output_obstacles_topic_);
    this->get_parameter("output_drop_topic", output_drop_topic_);
    this->get_parameter("output_climbable_topic", output_climbable_topic_);
    this->get_parameter("diagnostics_topic", diagnostics_topic_);
    this->get_parameter("base_ground_stable_topic", base_ground_stable_topic_);
    this->get_parameter("mf_kfs_state_topic", mf_kfs_state_topic_);
    this->get_parameter("mf_grid_layout_file", mf_grid_layout_file_);
    this->get_parameter("kfs_min_confidence", kfs_min_confidence_);
    this->get_parameter("target_frame", target_frame_);
    this->get_parameter("base_frame", base_frame_);
    this->get_parameter("tf_timeout_sec", tf_timeout_sec_);
    this->get_parameter("cloud_timeout_sec", cloud_timeout_sec_);
    this->get_parameter("odom_timeout_sec", odom_timeout_sec_);
    this->get_parameter("startup_grace_sec", startup_grace_sec_);
    this->get_parameter("perception_radius_m", perception_radius_m_);
    this->get_parameter("grid_resolution_m", grid_resolution_m_);
    this->get_parameter("voxel_leaf_size_m", voxel_leaf_size_m_);
    this->get_parameter("min_rel_z_m", min_rel_z_m_);
    this->get_parameter("max_rel_z_m", max_rel_z_m_);
    this->get_parameter("dis_ratio_z", dis_ratio_z_);
    this->get_parameter("min_points_per_cell", min_points_per_cell_);
    this->get_parameter("ground_quantile", ground_quantile_);
    this->get_parameter("top_quantile", top_quantile_);
    this->get_parameter("ground_ema_alpha", ground_ema_alpha_);
    this->get_parameter("enable_sigma_based_alpha", enable_sigma_based_alpha_);
    this->get_parameter("sigma_eps_m", sigma_eps_m_);
    this->get_parameter("sigma_alpha_min", sigma_alpha_min_);
    this->get_parameter("sigma_alpha_max", sigma_alpha_max_);
    this->get_parameter("h_climb_m", h_climb_m_);
    this->get_parameter("h_obstacle_m", h_obstacle_m_);
    this->get_parameter("h_drop_m", h_drop_m_);
    this->get_parameter("climbable_min_dz_m", climbable_min_dz_m_);
    this->get_parameter("unknown_policy", unknown_policy_);
    this->get_parameter("unknown_output", unknown_output_);
    this->get_parameter("enable_hysteresis", enable_hysteresis_);
    this->get_parameter("score_max", score_max_);
    this->get_parameter("score_inc", score_inc_);
    this->get_parameter("score_dec", score_dec_);
    this->get_parameter("obstacle_on_score", obstacle_on_score_);
    this->get_parameter("obstacle_off_score", obstacle_off_score_);
    this->get_parameter("drop_on_score", drop_on_score_);
    this->get_parameter("drop_off_score", drop_off_score_);
    this->get_parameter("stale_time_sec", stale_time_sec_);
    this->get_parameter("decay_time_sec", decay_time_sec_);
    this->get_parameter("cloud_qos_depth", cloud_qos_depth_);
    this->get_parameter("cloud_qos_reliability", cloud_qos_reliability_);
    this->get_parameter("cloud_qos_durability", cloud_qos_durability_);
    this->get_parameter("odom_qos_depth", odom_qos_depth_);
    this->get_parameter("odom_qos_reliability", odom_qos_reliability_);
    this->get_parameter("odom_qos_durability", odom_qos_durability_);
    this->get_parameter("output_qos_depth", output_qos_depth_);
    this->get_parameter("output_qos_reliability", output_qos_reliability_);
    this->get_parameter("output_qos_durability", output_qos_durability_);
    this->get_parameter("speed_limit_qos_depth", speed_limit_qos_depth_);
    this->get_parameter("speed_limit_qos_reliability", speed_limit_qos_reliability_);
    this->get_parameter("speed_limit_qos_durability", speed_limit_qos_durability_);
    this->get_parameter("diagnostics_qos_depth", diagnostics_qos_depth_);
    this->get_parameter("diagnostics_qos_reliability", diagnostics_qos_reliability_);
    this->get_parameter("diagnostics_qos_durability", diagnostics_qos_durability_);
    this->get_parameter("enable_diagnostics", enable_diagnostics_);
    this->get_parameter("enable_fail_safe", enable_fail_safe_);
    this->get_parameter("fail_safe_strategy", fail_safe_strategy_);
    this->get_parameter("virtual_fence_radius_m", virtual_fence_radius_m_);
    this->get_parameter("virtual_fence_num_points", virtual_fence_num_points_);
    this->get_parameter("virtual_fence_height_m", virtual_fence_height_m_);
    this->get_parameter("tf_health_timeout_sec", tf_health_timeout_sec_);
    this->get_parameter("output_sanity_check_enable", output_sanity_check_enable_);
    this->get_parameter("output_max_points_total", output_max_points_total_);
    this->get_parameter("output_max_points_per_cloud", output_max_points_per_cloud_);
    this->get_parameter("drop_forward_sector_deg", drop_forward_sector_deg_);
    this->get_parameter("drop_forward_min_x_m", drop_forward_min_x_m_);
    this->get_parameter("obstacle_neighbor_mode", obstacle_neighbor_mode_);
    this->get_parameter("drop_neighbor_mode",     drop_neighbor_mode_);
    this->get_parameter("min_obstacle_area_cells",min_obstacle_area_cells_);
    this->get_parameter("min_drop_area_cells",    min_drop_area_cells_);
    this->get_parameter("jump_thresh_m",          jump_thresh_m_);
    this->get_parameter("freeze_max_frames",      freeze_max_frames_);
    this->get_parameter("ground_ema_alpha_slow",  ground_ema_alpha_slow_);
    this->get_parameter("enable_pitch_compensation", enable_pitch_compensation_);
    this->get_parameter("enable_roll_compensation", enable_roll_compensation_);
    this->get_parameter("roll_compensation_mode", roll_compensation_mode_);
    this->get_parameter("roll_gate_deg", roll_gate_deg_);
    this->get_parameter("include_kfs_in_obstacles", include_kfs_in_obstacles_);
    this->get_parameter("publish_kfs_obstacles_debug", publish_kfs_obstacles_debug_);
    this->get_parameter("output_kfs_obstacles_debug_topic", output_kfs_obstacles_debug_topic_);
    this->get_parameter("stair_gate_speed_mps",   stair_gate_speed_mps_);
    this->get_parameter("stair_pitch_gate_deg",   stair_pitch_gate_deg_);
    this->get_parameter("top_z_max_delta_m",      top_z_max_delta_m_);
    this->get_parameter("terrain_features_topic", terrain_features_topic_);
    this->get_parameter("enable_terrain_features_pub", enable_terrain_features_pub_);
    this->get_parameter("terrain_features_publish_hz", terrain_features_publish_hz_);
    this->get_parameter("terrain_speed_limit_topic", terrain_speed_limit_topic_);
    this->get_parameter("enable_terrain_speed_limit_pub", enable_terrain_speed_limit_pub_);
    this->get_parameter("terrain_speed_limit_publish_hz", terrain_speed_limit_publish_hz_);
    this->get_parameter("speed_limit_forward_look_m", speed_limit_forward_look_m_);
    this->get_parameter("speed_limit_half_width_m", speed_limit_half_width_m_);
    this->get_parameter("speed_limit_v_max_mps", speed_limit_v_max_mps_);
    this->get_parameter("speed_limit_min_mps", speed_limit_min_mps_);
    this->get_parameter("speed_limit_w_slope", speed_limit_w_slope_);
    this->get_parameter("speed_limit_w_roughness", speed_limit_w_roughness_);
    this->get_parameter("speed_limit_w_drop", speed_limit_w_drop_);
    this->get_parameter("speed_limit_w_climbable", speed_limit_w_climbable_);
    this->get_parameter("speed_limit_k_tci", speed_limit_k_tci_);
    this->get_parameter("speed_limit_emergency_drop_thresh", speed_limit_emergency_drop_thresh_);
    this->get_parameter("enable_risk_model", enable_risk_model_);
    this->get_parameter("risk_model_file", risk_model_file_);
    this->get_parameter("latency_warn_ms",        latency_warn_ms_);
    this->get_parameter("latency_error_ms",       latency_error_ms_);
    this->get_parameter("latency_trigger_frames", latency_trigger_frames_);
    this->get_parameter("latency_recover_frames", latency_recover_frames_);
    this->get_parameter("latency_intervention_mode", latency_intervention_mode_);
    this->get_parameter("thermal_throttle_topic", thermal_throttle_topic_);
    this->get_parameter("thermal_throttle_release_sec", thermal_throttle_release_sec_);

    // 参数校验与归一化
    if (perception_radius_m_ <= 0.0) throw std::invalid_argument("perception_radius_m 必须 > 0");
    if (grid_resolution_m_ <= 0.0) throw std::invalid_argument("grid_resolution_m 必须 > 0");
    if (voxel_leaf_size_m_ <= 0.0) throw std::invalid_argument("voxel_leaf_size_m 必须 > 0");
    if (stale_time_sec_ <= 0.0) throw std::invalid_argument("stale_time_sec 必须 > 0");
    if (decay_time_sec_ <= 0.0) throw std::invalid_argument("decay_time_sec 必须 > 0");
    if (cloud_timeout_sec_ <= 0.0) throw std::invalid_argument("cloud_timeout_sec 必须 > 0");
    if (odom_timeout_sec_ <= 0.0) throw std::invalid_argument("odom_timeout_sec 必须 > 0");
    if (startup_grace_sec_ < 0.0) startup_grace_sec_ = 0.0;

    if (h_obstacle_m_ < h_climb_m_) {
        RCLCPP_WARN(this->get_logger(),
                    "检测到 h_obstacle_m(%.2f) < h_climb_m(%.2f)，已自动将 h_obstacle_m 提升至 h_climb_m 以避免误判可攀爬台阶为致命障碍。",
                    h_obstacle_m_, h_climb_m_);
        h_obstacle_m_ = h_climb_m_;
    }

    ground_ema_alpha_ = std::clamp(ground_ema_alpha_, 0.01, 1.0);
    sigma_eps_m_ = std::max(1e-6, sigma_eps_m_);
    sigma_alpha_min_ = std::clamp(sigma_alpha_min_, 0.0, 1.0);
    sigma_alpha_max_ = std::clamp(sigma_alpha_max_, 0.0, 1.0);
    if (sigma_alpha_min_ > sigma_alpha_max_) {
        std::swap(sigma_alpha_min_, sigma_alpha_max_);
    }
    sigma_alpha_max_ = std::min(sigma_alpha_max_, ground_ema_alpha_);
    min_points_per_cell_ = std::max(1, min_points_per_cell_);
    score_max_ = std::max(1, score_max_);
    score_inc_ = std::max(0, score_inc_);
    score_dec_ = std::max(0, score_dec_);
    virtual_fence_num_points_ = std::max(3, virtual_fence_num_points_);
    virtual_fence_radius_m_ = std::max(0.05, virtual_fence_radius_m_);
    climbable_min_dz_m_ = std::clamp(climbable_min_dz_m_, 0.0, h_climb_m_);
    freeze_max_frames_ = std::max(1, freeze_max_frames_);
    min_obstacle_area_cells_ = std::max(1, min_obstacle_area_cells_);
    min_drop_area_cells_ = std::max(1, min_drop_area_cells_);
    ground_ema_alpha_slow_ = std::clamp(ground_ema_alpha_slow_, 0.01, 1.0);
    roll_gate_deg_ = std::max(0.0, roll_gate_deg_);
    std::transform(roll_compensation_mode_.begin(), roll_compensation_mode_.end(),
                   roll_compensation_mode_.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (roll_compensation_mode_.empty()) {
        roll_compensation_mode_ = enable_roll_compensation_ ? "always" : "off";
    }
    if (roll_compensation_mode_ != "off" &&
        roll_compensation_mode_ != "gated" &&
        roll_compensation_mode_ != "always") {
        RCLCPP_WARN(this->get_logger(),
                    "roll_compensation_mode='%s' 非法，回退为 off",
                    roll_compensation_mode_.c_str());
        roll_compensation_mode_ = "off";
    }
    enable_roll_compensation_ = roll_compensation_mode_ != "off";
    if (publish_kfs_obstacles_debug_ && output_kfs_obstacles_debug_topic_.empty()) {
        RCLCPP_WARN(this->get_logger(),
                    "publish_kfs_obstacles_debug=true 但 output_kfs_obstacles_debug_topic 为空，已自动禁用");
        publish_kfs_obstacles_debug_ = false;
    }
    stair_gate_speed_mps_ = std::max(0.0, stair_gate_speed_mps_);
    stair_pitch_gate_deg_ = std::max(0.0, stair_pitch_gate_deg_);
    top_z_max_delta_m_ = std::max(0.1, top_z_max_delta_m_);
    terrain_features_publish_hz_ = std::max(terrain_features_publish_hz_, 0.0);
    terrain_speed_limit_publish_hz_ = std::max(terrain_speed_limit_publish_hz_, 0.0);
    speed_limit_forward_look_m_ = std::max(0.0, speed_limit_forward_look_m_);
    speed_limit_half_width_m_ = std::max(0.0, speed_limit_half_width_m_);
    speed_limit_v_max_mps_ = std::max(0.0, speed_limit_v_max_mps_);
    speed_limit_min_mps_ = std::clamp(speed_limit_min_mps_, 0.0, speed_limit_v_max_mps_);
    speed_limit_w_slope_ = std::max(0.0, speed_limit_w_slope_);
    speed_limit_w_roughness_ = std::max(0.0, speed_limit_w_roughness_);
    speed_limit_w_drop_ = std::max(0.0, speed_limit_w_drop_);
    speed_limit_w_climbable_ = std::max(0.0, speed_limit_w_climbable_);
    speed_limit_k_tci_ = std::max(0.0, speed_limit_k_tci_);
    speed_limit_emergency_drop_thresh_ = std::clamp(speed_limit_emergency_drop_thresh_, 0.0, 1.0);
    if (enable_terrain_features_pub_ && terrain_features_topic_.empty()) {
        RCLCPP_WARN(this->get_logger(),
                    "enable_terrain_features_pub=true 但 terrain_features_topic 为空，已自动禁用特征总线发布");
        enable_terrain_features_pub_ = false;
    }
    if (enable_terrain_speed_limit_pub_ && terrain_speed_limit_topic_.empty()) {
        RCLCPP_WARN(this->get_logger(),
                    "enable_terrain_speed_limit_pub=true 但 terrain_speed_limit_topic 为空，已自动禁用限速发布");
        enable_terrain_speed_limit_pub_ = false;
    }
    if (enable_risk_model_) {
        if (risk_model_file_.empty()) {
            try {
                const auto terrain_share =
                    ament_index_cpp::get_package_share_directory("rc26_terrain");
                risk_model_file_ = terrain_share + "/config/terrain_risk_model.yaml";
            } catch (const std::exception& ex) {
                RCLCPP_WARN(this->get_logger(),
                            "risk_model_file 为空且无法解析默认模型文件: %s", ex.what());
            }
        }
        std::string load_error;
        terrain_risk_model_ready_ =
            !risk_model_file_.empty() &&
            terrain_risk_model_.loadFromFile(risk_model_file_, load_error) &&
            terrain_risk_model_.enabled();
        if (!terrain_risk_model_ready_) {
            RCLCPP_WARN(this->get_logger(),
                        "risk model 未启用，回退 score proxy：file=%s err=%s",
                        risk_model_file_.empty() ? "(empty)" : risk_model_file_.c_str(),
                        load_error.empty() ? "model disabled" : load_error.c_str());
        }
    } else {
        terrain_risk_model_ready_ = false;
    }
    latency_trigger_frames_ = std::max(1, latency_trigger_frames_);
    latency_recover_frames_ = std::max(1, latency_recover_frames_);
    thermal_throttle_release_sec_ = std::max(0.0, thermal_throttle_release_sec_);

    if (unknown_policy_ != "aggressive" && unknown_policy_ != "conservative") {
        throw std::invalid_argument("unknown_policy 仅支持: aggressive/conservative");
    }
    if (unknown_output_ != "drop" && unknown_output_ != "obstacles") {
        throw std::invalid_argument("unknown_output 仅支持: drop/obstacles");
    }
    if (fail_safe_strategy_ != "none" && fail_safe_strategy_ != "virtual_fence" &&
        fail_safe_strategy_ != "emergency_stop") {
        throw std::invalid_argument("fail_safe_strategy 仅支持: none/virtual_fence/emergency_stop");
    }
    if (latency_intervention_mode_ != "none" &&
        latency_intervention_mode_ != "virtual_fence" &&
        latency_intervention_mode_ != "emergency_stop") {
        throw std::invalid_argument("latency_intervention_mode 仅支持: none/virtual_fence/emergency_stop");
    }

    // TF 健康超时校验
    if (tf_health_timeout_sec_ <= 0.0) {
        RCLCPP_WARN(this->get_logger(),
            "tf_health_timeout_sec 无效,回退到 cloud_timeout_sec");
        tf_health_timeout_sec_ = cloud_timeout_sec_;
    }

    // 扇区角度归一化
    drop_forward_sector_deg_ = std::clamp(drop_forward_sector_deg_, 0.0, 360.0);

    SafetyGuardConfig safety_config;
    safety_config.cloud_timeout_sec = cloud_timeout_sec_;
    safety_config.odom_timeout_sec = odom_timeout_sec_;
    safety_config.startup_grace_sec = startup_grace_sec_;
    safety_config.tf_health_timeout_sec = tf_health_timeout_sec_;
    safety_config.enable_fail_safe = enable_fail_safe_;
    safety_config.fail_safe_strategy = fail_safe_strategy_;
    safety_config.latency_warn_ms = latency_warn_ms_;
    safety_config.latency_error_ms = latency_error_ms_;
    safety_config.latency_trigger_frames = latency_trigger_frames_;
    safety_config.latency_recover_frames = latency_recover_frames_;
    safety_config.latency_intervention_mode = latency_intervention_mode_;
    safety_config.thermal_throttle_release_sec = thermal_throttle_release_sec_;
    safety_guard_.configure(std::move(safety_config));

    // 构建 QoS（此处会校验字符串有效性）
    const auto cloud_qos = makeQoS(cloud_qos_depth_, cloud_qos_reliability_, cloud_qos_durability_);
    const auto odom_qos = makeQoS(odom_qos_depth_, odom_qos_reliability_, odom_qos_durability_);
    const auto output_qos = makeQoS(output_qos_depth_, output_qos_reliability_, output_qos_durability_);
    const auto speed_limit_qos =
        makeQoS(speed_limit_qos_depth_, speed_limit_qos_reliability_, speed_limit_qos_durability_);
    const auto diag_qos =
        makeQoS(diagnostics_qos_depth_, diagnostics_qos_reliability_, diagnostics_qos_durability_);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    TfChainSpec tf_chain_spec;
    tf_chain_spec.odom_frame = target_frame_;
    tf_chain_spec.base_frame = base_frame_;
    tf_chain_spec.sensor_frame = "livox_frame";
    tf_chain_spec.forbidden_frame = std::string("laser_link");
    tf_chain_spec.max_dynamic_age_sec = tf_health_timeout_sec_;
    tf_chain_validator_ = std::make_unique<TfChainValidator>(std::move(tf_chain_spec));

    initGrid();
    if (mf_grid_layout_file_.empty()) {
        try {
            const auto keepout_share = ament_index_cpp::get_package_share_directory("rc26_kfs_keepout");
            mf_grid_layout_file_ = keepout_share + "/config/r2_mf_world.yaml";
        } catch (const std::exception&) {
            mf_grid_layout_file_.clear();
        }
    }
    if (!mf_grid_layout_file_.empty() && !loadMfGridLayout(mf_grid_layout_file_)) {
        RCLCPP_WARN(this->get_logger(), "mf grid layout load failed: %s", mf_grid_layout_file_.c_str());
    }

    pub_obstacles_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_obstacles_topic_, output_qos);
    pub_drop_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_drop_topic_, output_qos);
    if (!output_climbable_topic_.empty()) {
        pub_climbable_ =
            this->create_publisher<sensor_msgs::msg::PointCloud2>(output_climbable_topic_, output_qos);
    }
    if (publish_kfs_obstacles_debug_) {
        pub_kfs_obstacles_debug_ =
            this->create_publisher<sensor_msgs::msg::PointCloud2>(
                output_kfs_obstacles_debug_topic_, output_qos);
    }
    if (enable_diagnostics_) {
        pub_diagnostics_ =
            this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(diagnostics_topic_, diag_qos);
    }
    if (enable_terrain_features_pub_) {
        pub_features_ =
            this->create_publisher<rc26_interfaces::msg::TerrainFeatureGrid>(terrain_features_topic_, output_qos);
    }
    if (enable_terrain_speed_limit_pub_) {
        pub_speed_limit_ =
            this->create_publisher<std_msgs::msg::Float32>(terrain_speed_limit_topic_, speed_limit_qos);
    }

    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, odom_qos, std::bind(&TerrainSemanticNode::odomCallback, this, std::placeholders::_1));
    sub_base_ground_stable_ = this->create_subscription<std_msgs::msg::Bool>(
        base_ground_stable_topic_, rclcpp::QoS(10),
        [this](const std_msgs::msg::Bool::ConstSharedPtr& msg) {
            if (msg) {
                base_ground_stable_ = msg->data;
            }
        });
    if (!thermal_throttle_topic_.empty()) {
        sub_thermal_throttle_ = this->create_subscription<std_msgs::msg::Bool>(
            thermal_throttle_topic_, rclcpp::QoS(10),
            [this](const std_msgs::msg::Bool::ConstSharedPtr& msg) {
                if (!msg) {
                    return;
                }
                thermal_throttle_requested_ = msg->data;
                if (msg->data) {
                    thermal_throttle_last_true_stamp_ = this->get_clock()->now();
                }
            });
    }
    sub_mf_kfs_ = this->create_subscription<rc26_interfaces::msg::MfKfsState>(
        mf_kfs_state_topic_, rclcpp::QoS(1).reliable(),
        [this](const rc26_interfaces::msg::MfKfsState::ConstSharedPtr& msg) {
            if (msg) {
                updateKfsOccupied(*msg);
            }
        });

    sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_cloud_topic_, cloud_qos,
        std::bind(&TerrainSemanticNode::cloudCallback, this, std::placeholders::_1));

    // 使用 wall timer 做健康检测；实际超时判断基于 ROS Time（兼容 use_sim_time）
    health_timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                            std::bind(&TerrainSemanticNode::healthTimerCallback, this));

    RCLCPP_INFO(this->get_logger(),
                "rc26_terrain 已启动: 栅格=%dx%d 分辨率=%.2fm 半径=%.1fm, 输入=%s, 输出=%s/%s, "
                "roll_mode=%s roll_gate=%.1fdeg include_kfs_in_obstacles=%s kfs_debug_topic=%s "
                "risk_model=%s model_file=%s",
                width_, width_, grid_resolution_m_, perception_radius_m_,
                input_cloud_topic_.c_str(), output_obstacles_topic_.c_str(), output_drop_topic_.c_str(),
                roll_compensation_mode_.c_str(), roll_gate_deg_,
                include_kfs_in_obstacles_ ? "true" : "false",
                publish_kfs_obstacles_debug_ ? output_kfs_obstacles_debug_topic_.c_str() : "disabled",
                terrain_risk_model_ready_ ? "enabled" : "fallback_proxy",
                risk_model_file_.empty() ? "(empty)" : risk_model_file_.c_str());
}

void TerrainSemanticNode::odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg) {
    if (!msg) return;

    received_odom_ = true;
    last_odom_stamp_ = rclcpp::Time(msg->header.stamp);
    last_linear_speed_mps_ = std::hypot(msg->twist.twist.linear.x, msg->twist.twist.linear.y);

    // 仅当里程计本身就在 target_frame 下时，才可直接作为失效保护的备用位姿
    if (msg->header.frame_id != target_frame_) return;

    const auto& p = msg->pose.pose.position;
    const auto& q = msg->pose.pose.orientation;
    tf2::Quaternion quat(q.x, q.y, q.z, q.w);
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    last_base_x_ = p.x;
    last_base_y_ = p.y;
    last_base_z_ = p.z;
    last_cos_yaw_ = std::cos(yaw);
    last_sin_yaw_ = std::sin(yaw);
    have_last_pose_ = true;
}

SafetyGuardInput TerrainSemanticNode::buildSafetyGuardInput(const rclcpp::Time& now) const {
    SafetyGuardInput input;
    input.since_start_sec = (now - node_start_time_).seconds();
    input.received_cloud = received_cloud_;
    input.cloud_age_sec = received_cloud_ ? (now - last_cloud_stamp_).seconds()
                                          : std::numeric_limits<double>::infinity();
    input.received_odom = received_odom_;
    input.odom_age_sec = received_odom_ ? (now - last_odom_stamp_).seconds()
                                        : std::numeric_limits<double>::infinity();
    input.need_tf = received_cloud_ && input.cloud_age_sec <= cloud_timeout_sec_;
    input.tf_age_sec = (last_good_tf_stamp_.nanoseconds() > 0)
                          ? (now - last_good_tf_stamp_).seconds()
                          : std::numeric_limits<double>::infinity();
    input.latency_ms = last_latency_ms_;
    input.thermal_throttle_requested = thermal_throttle_requested_;
    input.thermal_throttle_last_true_age_sec =
        (thermal_throttle_last_true_stamp_.nanoseconds() > 0)
            ? (now - thermal_throttle_last_true_stamp_).seconds()
            : std::numeric_limits<double>::infinity();
    return input;
}

void TerrainSemanticNode::syncSafetyGuardState(const SafetyGuardDecision& decision) {
    fail_safe_active_ = decision.fail_safe_active;
    fail_safe_reason_ = decision.fail_safe_reason;
    latency_overrun_count_ = decision.latency_overrun_count;
    latency_recover_count_ = decision.latency_recover_count;
    latency_intervention_active_ = decision.latency_intervention_active;
}

TfValidationReport TerrainSemanticNode::validateTfChain(const rclcpp::Time& now) const {
    if (!tf_chain_validator_ || !tf_buffer_) {
        TfValidationReport report;
        report.ok = false;
        report.code = TfChainStatusCode::kInvalidSpecification;
        report.message = "TF validator 未初始化";
        return report;
    }

    const tf2::TimePoint reference_time = tf2::timeFromSec(now.seconds());
    return tf_chain_validator_->validate(*tf_buffer_, tf2::TimePointZero, reference_time);
}

void TerrainSemanticNode::healthTimerCallback() {
    const auto now = this->get_clock()->now();
    const auto input = buildSafetyGuardInput(now);
    const auto tf_report = validateTfChain(now);
    last_tf_validation_ok_ = tf_report.ok;
    last_tf_validation_code_ = tf_report.code;
    last_tf_validation_message_ = tf_report.message;

    const bool after_grace = input.since_start_sec >= startup_grace_sec_;
    const bool force_tf_chain_fail_safe =
        after_grace && !tf_report.ok &&
        (input.need_tf ||
         tf_report.code == TfChainStatusCode::kLegacyFrameDetected ||
         tf_report.code == TfChainStatusCode::kUnexpectedStaticExtrinsic ||
         tf_report.code == TfChainStatusCode::kInvalidSpecification);

    if (tf_report.ok || !force_tf_chain_fail_safe) {
        safety_guard_.clearForcedFailSafe("tf_chain");
    } else {
        safety_guard_.forceFailSafe("tf_chain", "TF 链路验证失败: " + tf_report.message);
    }

    const auto decision = safety_guard_.evaluate(input);
    syncSafetyGuardState(decision);

    if (decision.fail_safe_entered) {
        RCLCPP_ERROR(this->get_logger(), "进入降级保护模式: %s", decision.fail_safe_reason.c_str());
    } else if (decision.fail_safe_cleared) {
        RCLCPP_INFO(this->get_logger(), "退出降级保护模式");
    }

    if (decision.latency_intervention_entered) {
        if (decision.thermal_throttle_active) {
            RCLCPP_ERROR(this->get_logger(),
                "latency intervention activated by thermal throttle, overrun_count=%d",
                decision.latency_overrun_count);
        } else {
            RCLCPP_ERROR(this->get_logger(),
                "latency intervention activated, latency=%.2fms overrun_count=%d",
                last_latency_ms_, decision.latency_overrun_count);
        }
    } else if (decision.latency_intervention_cleared) {
        RCLCPP_INFO(this->get_logger(),
            "latency intervention cleared, recover_count=%d", decision.latency_recover_count);
    }

    if (decision.required_intervention == SafetyIntervention::kVirtualFence && have_last_pose_) {
        publishVirtualFence(now, last_base_x_, last_base_y_, last_base_z_, last_cos_yaw_, last_sin_yaw_);
        publishSpeedLimitValue(now, 0.0f, true);
    } else if (decision.required_intervention != SafetyIntervention::kNone) {
        publishEmergencyStop(now);
        publishSpeedLimitValue(now, 0.0f, true);
    }

    publishDiagnostics(now, decision.diagnostic_level, decision.diagnostic_message);
}

void TerrainSemanticNode::publishVirtualFence(const rclcpp::Time& stamp, double base_x,
                                              double base_y, double base_z,
                                              double /*cos_yaw*/, double /*sin_yaw*/) {
    pcl::PointCloud<pcl::PointXYZI> fence_cloud;
    fence_cloud.reserve(static_cast<size_t>(virtual_fence_num_points_));

    constexpr double kPi = 3.14159265358979323846;
    const double r = virtual_fence_radius_m_;
    for (int i = 0; i < virtual_fence_num_points_; i++) {
        const double a = 2.0 * kPi * static_cast<double>(i) / static_cast<double>(virtual_fence_num_points_);
        pcl::PointXYZI p;
        p.x = static_cast<float>(base_x + r * std::cos(a));
        p.y = static_cast<float>(base_y + r * std::sin(a));
        p.z = static_cast<float>(base_z + virtual_fence_height_m_);
        p.intensity = 1.0f;
        fence_cloud.push_back(p);
    }

    sensor_msgs::msg::PointCloud2 fence_msg;
    pcl::toROSMsg(fence_cloud, fence_msg);
    fence_msg.header.stamp = stamp;
    fence_msg.header.frame_id = target_frame_;

    // 发布到两个话题，确保下游执行链任一观测源配置都能生效
    if (pub_obstacles_) pub_obstacles_->publish(fence_msg);
    if (pub_drop_) pub_drop_->publish(fence_msg);
}

void TerrainSemanticNode::publishDiagnostics(const rclcpp::Time& stamp, int level,
                                             const std::string& message) const {
    if (!pub_diagnostics_) return;

    const auto decision = safety_guard_.decision();
    const auto stats = safety_guard_.stats();

    diagnostic_msgs::msg::DiagnosticStatus st;
    st.level = static_cast<uint8_t>(std::clamp(level, 0, 3));
    st.name = this->get_fully_qualified_name();
    st.hardware_id = "R2";
    st.message = message;

    auto addKV = [&](const std::string& key, const std::string& value) {
        diagnostic_msgs::msg::KeyValue kv;
        kv.key = key;
        kv.value = value;
        st.values.push_back(kv);
    };

    addKV("input_cloud_topic", input_cloud_topic_);
    addKV("odom_topic", odom_topic_);
    addKV("target_frame", target_frame_);
    addKV("base_frame", base_frame_);
    addKV("received_cloud", received_cloud_ ? "true" : "false");
    addKV("received_odom", received_odom_ ? "true" : "false");
    addKV("fail_safe_active", decision.fail_safe_active ? "true" : "false");
    addKV("fail_safe_reason", decision.fail_safe_reason);
    addKV("base_ground_stable", base_ground_stable_ ? "true" : "false");
    addKV("last_linear_speed_mps", std::to_string(last_linear_speed_mps_));
    addKV("last_pitch_deg", std::to_string(last_pitch_rad_ * 180.0 / M_PI));
    addKV("last_roll_deg", std::to_string(last_roll_rad_ * 180.0 / M_PI));
    addKV("roll_compensation_mode", roll_compensation_mode_);
    addKV("roll_gate_deg", std::to_string(roll_gate_deg_));
    addKV("freeze_max_frames", std::to_string(freeze_max_frames_));
    addKV("last_max_freeze_count", std::to_string(last_max_freeze_count_));

    const double cloud_age = received_cloud_ ? (stamp - last_cloud_stamp_).seconds()
                                             : std::numeric_limits<double>::infinity();
    const double odom_age = received_odom_ ? (stamp - last_odom_stamp_).seconds()
                                           : std::numeric_limits<double>::infinity();
    const double tf_age = (last_good_tf_stamp_.nanoseconds() > 0)
                              ? (stamp - last_good_tf_stamp_).seconds()
                              : std::numeric_limits<double>::infinity();
    addKV("cloud_age_sec", std::isfinite(cloud_age) ? std::to_string(cloud_age) : "inf");
    addKV("odom_age_sec", std::isfinite(odom_age) ? std::to_string(odom_age) : "inf");
    addKV("tf_age_sec", std::isfinite(tf_age) ? std::to_string(tf_age) : "inf");
    addKV("cloud_to_publish_latency_ms",
          std::isfinite(last_latency_ms_) ? std::to_string(last_latency_ms_) : "n/a");
    addKV("latency_intervention_active", decision.latency_intervention_active ? "true" : "false");
    addKV("latency_intervention_mode", latency_intervention_mode_);
    addKV("latency_overrun_count", std::to_string(decision.latency_overrun_count));
    addKV("latency_recover_count", std::to_string(decision.latency_recover_count));
    addKV("thermal_throttle_topic", thermal_throttle_topic_.empty() ? "disabled" : thermal_throttle_topic_);
    addKV("thermal_throttle_requested", thermal_throttle_requested_ ? "true" : "false");
    const double thermal_age =
        (thermal_throttle_last_true_stamp_.nanoseconds() > 0)
            ? (stamp - thermal_throttle_last_true_stamp_).seconds()
            : std::numeric_limits<double>::infinity();
    addKV("thermal_throttle_last_true_age_sec",
          std::isfinite(thermal_age) ? std::to_string(thermal_age) : "inf");
    addKV("thermal_throttle_release_sec", std::to_string(thermal_throttle_release_sec_));
    addKV("tf_chain_ok", last_tf_validation_ok_ ? "true" : "false");
    addKV("tf_chain_code", std::to_string(static_cast<int>(last_tf_validation_code_)));
    addKV("tf_chain_message", last_tf_validation_message_);
    addKV("kfs_occupied_cells", std::to_string(stats.kfs_occupied_cells));
    addKV("include_kfs_in_obstacles", include_kfs_in_obstacles_ ? "true" : "false");
    addKV("publish_kfs_obstacles_debug", publish_kfs_obstacles_debug_ ? "true" : "false");
    addKV("output_kfs_obstacles_debug_topic",
          publish_kfs_obstacles_debug_ ? output_kfs_obstacles_debug_topic_ : "disabled");
    addKV("risk_model_enabled", enable_risk_model_ ? "true" : "false");
    addKV("risk_model_ready", terrain_risk_model_ready_ ? "true" : "false");
    addKV("risk_model_file", risk_model_file_.empty() ? "(empty)" : risk_model_file_);
    addKV("obstacle_cells", std::to_string(stats.obstacle_cells));
    addKV("drop_cells", std::to_string(stats.drop_cells));
    addKV("climbable_cells", std::to_string(stats.climbable_cells));
    addKV("fail_safe_strategy", fail_safe_strategy_);

    diagnostic_msgs::msg::DiagnosticArray arr;
    arr.header.stamp = stamp;
    arr.status.push_back(st);
    pub_diagnostics_->publish(arr);
}

std::optional<tf2::Transform> TerrainSemanticNode::getTransform(
    const std::string& target_frame, const std::string& source_frame,
    const rclcpp::Time& time) {
    try {
        auto ts = tf_buffer_->lookupTransform(
            target_frame, source_frame, time,
            rclcpp::Duration::from_seconds(tf_timeout_sec_));
        tf2::Transform transform;
        tf2::fromMsg(ts.transform, transform);
        return transform;
    } catch (tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "TF 查询失败 (%s -> %s): %s",
                             source_frame.c_str(), target_frame.c_str(), ex.what());
        return std::nullopt;
    }
}

bool TerrainSemanticNode::isThrottleReady(const rclcpp::Time& stamp,
                                          const rclcpp::Time& last_pub_time,
                                          double publish_hz) const {
    if (publish_hz <= 0.0 || last_pub_time.nanoseconds() == 0) {
        return true;
    }
    if (stamp.nanoseconds() < last_pub_time.nanoseconds()) {
        return true;
    }
    return (stamp - last_pub_time).seconds() >= (1.0 / publish_hz);
}

void TerrainSemanticNode::publishSpeedLimitValue(const rclcpp::Time& stamp, float v_limit, bool force) {
    if (!pub_speed_limit_) {
        return;
    }
    if (!force && !isThrottleReady(stamp, last_speed_limit_pub_time_, terrain_speed_limit_publish_hz_)) {
        return;
    }

    std_msgs::msg::Float32 speed_limit_msg;
    const float safe_limit = std::clamp(std::isfinite(v_limit) ? v_limit : 0.0f,
                                        0.0f,
                                        static_cast<float>(speed_limit_v_max_mps_));
    speed_limit_msg.data = safe_limit;
    pub_speed_limit_->publish(speed_limit_msg);
    last_speed_limit_pub_time_ = stamp;
}

void TerrainSemanticNode::initGrid() {
    const double half_width_cells = std::ceil(perception_radius_m_ / grid_resolution_m_);
    if (!std::isfinite(half_width_cells)) {
        throw std::invalid_argument("栅格尺寸计算失败: perception_radius_m/grid_resolution_m 非有限值");
    }

    constexpr int kMaxHalfWidth = (std::numeric_limits<int>::max() - 1) / 2;
    if (half_width_cells < 0.0 || half_width_cells > static_cast<double>(kMaxHalfWidth)) {
        throw std::invalid_argument("栅格尺寸非法: half_width 超出 int 可表示范围");
    }

    half_width_ = static_cast<int>(half_width_cells);
    width_ = 2 * half_width_ + 1;

    const size_t num_cells_candidate = static_cast<size_t>(width_) * static_cast<size_t>(width_);
    if (num_cells_candidate > static_cast<size_t>(std::numeric_limits<int>::max())) {
        throw std::invalid_argument("栅格尺寸非法: 单元总数超出实现支持范围");
    }
    num_cells_ = static_cast<int>(num_cells_candidate);

    cell_in_radius_.assign(static_cast<size_t>(num_cells_), 0);
    ground_z_filtered_.assign(static_cast<size_t>(num_cells_), 0.0f);
    top_z_.assign(static_cast<size_t>(num_cells_), 0.0f);
    last_seen_sec_.assign(static_cast<size_t>(num_cells_), -1.0);
    obstacle_score_.assign(static_cast<size_t>(num_cells_), 0);
    drop_score_.assign(static_cast<size_t>(num_cells_), 0);
    freeze_count_.assign(static_cast<size_t>(num_cells_), 0);
    obstacle_state_.assign(static_cast<size_t>(num_cells_), 0);
    drop_state_.assign(static_cast<size_t>(num_cells_), 0);
    kfs_occupied_state_.assign(static_cast<size_t>(num_cells_), 0);
    sigma_h_.assign(static_cast<size_t>(num_cells_), 0.0f);
    density_.assign(static_cast<size_t>(num_cells_), 0U);
    slope_x_.assign(static_cast<size_t>(num_cells_), 0.0f);
    slope_y_.assign(static_cast<size_t>(num_cells_), 0.0f);
    roughness_.assign(static_cast<size_t>(num_cells_), 0.0f);
    cell_z_samples_.assign(static_cast<size_t>(num_cells_), {});
    touched_cells_.reserve(static_cast<size_t>(num_cells_));

    const double r2 = perception_radius_m_ * perception_radius_m_;
    for (int ix = 0; ix < width_; ix++) {
        for (int iy = 0; iy < width_; iy++) {
            const double x = (static_cast<double>(ix - half_width_) + 0.5) * grid_resolution_m_;
            const double y = (static_cast<double>(iy - half_width_) + 0.5) * grid_resolution_m_;
            if (x * x + y * y <= r2) {
                cell_in_radius_[static_cast<size_t>(ix * width_ + iy)] = 1;
            }
        }
    }
}

bool TerrainSemanticNode::loadMfGridLayout(const std::string& path) {
    try {
        std::filesystem::path resolved(path);
        if (resolved.is_relative()) {
            resolved = std::filesystem::current_path() / resolved;
        }
        YAML::Node root = YAML::LoadFile(resolved.string());
        if (root["world_layout_file"]) {
            std::filesystem::path nested = root["world_layout_file"].as<std::string>();
            if (nested.empty()) {
                return false;
            }
            if (nested.is_relative()) {
                nested = resolved.parent_path() / nested;
            }
            return loadMfGridLayout(nested.string());
        }

        const bool use_blocks = root["blocks"] && root["blocks"].IsSequence();
        const YAML::Node cells_node = use_blocks ? root["blocks"] : root["grids"];
        if (!cells_node) {
            return false;
        }
        mf_grid_valid_.fill(0U);
        if (root["meta"] && root["meta"]["team"]) {
            mf_layout_team_ = root["meta"]["team"].as<std::string>();
        }
        for (const auto& grid : cells_node) {
            const int id = grid["id"].as<int>();
            if (id < 1 || id > 12) {
                continue;
            }
            mf_grid_x_[static_cast<size_t>(id)] = grid["x"].as<double>();
            mf_grid_y_[static_cast<size_t>(id)] = grid["y"].as<double>();
            mf_grid_valid_[static_cast<size_t>(id)] = 1U;
        }
        return true;
    } catch (const std::exception& ex) {
        RCLCPP_WARN(this->get_logger(), "loadMfGridLayout error: %s", ex.what());
        return false;
    }
}

void TerrainSemanticNode::updateKfsOccupied(const rc26_interfaces::msg::MfKfsState& msg) {
    std::fill(kfs_occupied_state_.begin(), kfs_occupied_state_.end(), 0U);
    kfs_occupied_count_ = 0;
    if (!have_last_pose_) {
        return;
    }

    if (!mf_layout_team_.empty() && !msg.team.empty() && mf_layout_team_ != msg.team) {
        return;
    }

    for (const auto& cell : msg.cells) {
        if (cell.grid_id < 1 || cell.grid_id > 12) {
            continue;
        }
        if (cell.kfs_type < 1 || cell.kfs_type > 3) {
            continue;
        }
        if (cell.confidence < static_cast<float>(kfs_min_confidence_)) {
            continue;
        }
        if (!mf_grid_valid_[cell.grid_id]) {
            continue;
        }

        const double dx = mf_grid_x_[cell.grid_id] - last_base_x_;
        const double dy = mf_grid_y_[cell.grid_id] - last_base_y_;

        const double rx = dx * last_cos_yaw_ + dy * last_sin_yaw_;
        const double ry = -dx * last_sin_yaw_ + dy * last_cos_yaw_;

        const int gx = static_cast<int>(std::round(rx / grid_resolution_m_)) + half_width_;
        const int gy = static_cast<int>(std::round(ry / grid_resolution_m_)) + half_width_;
        if (gx < 0 || gx >= width_ || gy < 0 || gy >= width_) {
            continue;
        }
        const int idx = gx * width_ + gy;
        if (idx < 0 || idx >= num_cells_) {
            continue;
        }
        const size_t uidx = static_cast<size_t>(idx);
        if (cell_in_radius_[uidx]) {
            if (!kfs_occupied_state_[uidx]) {
                ++kfs_occupied_count_;
            }
            kfs_occupied_state_[uidx] = 1U;
        }
    }
}

void TerrainSemanticNode::estimateCellHeights(double stamp_sec) {
    int max_freeze_count = 0;
    std::fill(density_.begin(), density_.end(), static_cast<uint16_t>(0));
    const double pitch_gate_rad = stair_pitch_gate_deg_ * M_PI / 180.0;
    const bool stair_motion_gate =
        (last_linear_speed_mps_ >= stair_gate_speed_mps_) ||
        (!base_ground_stable_) ||
        (std::abs(last_pitch_rad_) >= pitch_gate_rad);
    const bool reuse_ground_as_q25 = std::abs(ground_quantile_ - 0.25) <= 1e-6;

    for (const int cell : touched_cells_) {
        const size_t idx = static_cast<size_t>(cell);
        auto& samples = cell_z_samples_[static_cast<size_t>(cell)];
        density_[idx] = static_cast<uint16_t>(
            std::min(samples.size(), static_cast<size_t>(std::numeric_limits<uint16_t>::max())));
        if (static_cast<int>(samples.size()) < min_points_per_cell_) {
            sigma_h_[idx] = 0.0f;
            continue;
        }

        const float ground_z = quantileInplace(samples, ground_quantile_);
        float top_z = quantileInplace(samples, top_quantile_);
        const float q25 = reuse_ground_as_q25 ? ground_z : quantileInplace(samples, 0.25f);
        const float q75 = quantileInplace(samples, 0.75f);
        const float sigma_h = std::max(0.0f, q75 - q25) / 1.349f;
        sigma_h_[idx] = sigma_h;

        float final_alpha_n = static_cast<float>(ground_ema_alpha_);
        float final_alpha_s = static_cast<float>(ground_ema_alpha_slow_);
        if (enable_sigma_based_alpha_) {
            const float s_alpha = std::clamp(
                static_cast<float>(ground_ema_alpha_ /
                                   (static_cast<double>(sigma_h) + sigma_eps_m_)),
                static_cast<float>(sigma_alpha_min_),
                static_cast<float>(sigma_alpha_max_));
            final_alpha_n = std::min(final_alpha_n, s_alpha);
            final_alpha_s = std::min(final_alpha_s, s_alpha);
        }

        if (top_z < ground_z) top_z = ground_z;
        if (top_z > ground_z + static_cast<float>(top_z_max_delta_m_)) {
            top_z = ground_z + static_cast<float>(top_z_max_delta_m_);
        }

        if (last_seen_sec_[idx] < 0.0) {
            ground_z_filtered_[idx] = ground_z;
            freeze_count_[idx] = 0;
        } else {
            const float jump = std::abs(ground_z - ground_z_filtered_[idx]);
            if (jump > static_cast<float>(jump_thresh_m_)) {
                if (stair_motion_gate) {
                    ground_z_filtered_[idx] = final_alpha_s * ground_z +
                                              (1.0f - final_alpha_s) * ground_z_filtered_[idx];
                    freeze_count_[idx] = 0;
                } else {
                    int& freeze = freeze_count_[idx];
                    freeze = std::max(0, freeze + 1);
                    max_freeze_count = std::max(max_freeze_count, freeze);
                    if (freeze >= freeze_max_frames_) {
                        ground_z_filtered_[idx] = final_alpha_s * ground_z +
                                                  (1.0f - final_alpha_s) * ground_z_filtered_[idx];
                        freeze = 0;
                    }
                }
            } else {
                ground_z_filtered_[idx] = final_alpha_n * ground_z +
                                          (1.0f - final_alpha_n) * ground_z_filtered_[idx];
                freeze_count_[idx] = 0;
            }
        }
        top_z_[idx] = top_z;
        last_seen_sec_[idx] = stamp_sec;
    }
    last_max_freeze_count_ = max_freeze_count;
}

void TerrainSemanticNode::classifyAndUpdate(double stamp_sec) {
    const bool obstacle_use_edge8 = (obstacle_neighbor_mode_ == "edge8");
    bool drop_use_edge4 = false;
    if (drop_neighbor_mode_ == "edge4") {
        drop_use_edge4 = true;
    } else if (drop_neighbor_mode_ != "edge8") {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
            "drop_neighbor_mode='%s' 非法，回退为 edge8", drop_neighbor_mode_.c_str());
    }
    std::fill(slope_x_.begin(), slope_x_.end(), 0.0f);
    std::fill(slope_y_.begin(), slope_y_.end(), 0.0f);
    std::fill(roughness_.begin(), roughness_.end(), 0.0f);

    for (int cell = 0; cell < num_cells_; cell++) {
        const size_t idx = static_cast<size_t>(cell);

        if (last_seen_sec_[idx] >= 0.0 && (stamp_sec - last_seen_sec_[idx]) > decay_time_sec_) {
            last_seen_sec_[idx] = -1.0;
            ground_z_filtered_[idx] = 0.0f;
            top_z_[idx] = 0.0f;
            obstacle_score_[idx] = 0;
            drop_score_[idx] = 0;
            freeze_count_[idx] = 0;
            obstacle_state_[idx] = 0;
            drop_state_[idx] = 0;
            sigma_h_[idx] = 0.0f;
        }

        if (!cell_in_radius_[idx]) {
            if (enable_hysteresis_) {
                obstacle_score_[idx] = std::max(0, obstacle_score_[idx] - score_dec_);
                drop_score_[idx] = std::max(0, drop_score_[idx] - score_dec_);
                if (obstacle_state_[idx] && obstacle_score_[idx] <= obstacle_off_score_)
                    obstacle_state_[idx] = 0;
                if (drop_state_[idx] && drop_score_[idx] <= drop_off_score_)
                    drop_state_[idx] = 0;
            } else {
                obstacle_state_[idx] = 0;
                drop_state_[idx] = 0;
            }
            continue;
        }

        const double last = last_seen_sec_[idx];
        const bool fresh = (last >= 0.0) && ((stamp_sec - last) <= stale_time_sec_);

        if (!fresh) {
            if (enable_hysteresis_) {
                obstacle_score_[idx] = std::max(0, obstacle_score_[idx] - score_dec_);
                drop_score_[idx] = std::max(0, drop_score_[idx] - score_dec_);
                if (obstacle_state_[idx] && obstacle_score_[idx] <= obstacle_off_score_)
                    obstacle_state_[idx] = 0;
                if (drop_state_[idx] && drop_score_[idx] <= drop_off_score_)
                    drop_state_[idx] = 0;
            } else {
                obstacle_state_[idx] = 0;
                drop_state_[idx] = 0;
            }
            continue;
        }

        const int ix = cell / width_;
        const int iy = cell % width_;

        // obstacle dz_up: 4邻域 (可通过 obstacle_neighbor_mode_=="edge8" 回退)
        float dz_up = 0.0f;
        if (obstacle_use_edge8) {
            for (int dx = -1; dx <= 1; dx++) {
                for (int dy = -1; dy <= 1; dy++) {
                    if (dx == 0 && dy == 0) continue;
                    const int nx = ix + dx, ny = iy + dy;
                    if (nx < 0 || nx >= width_ || ny < 0 || ny >= width_) continue;
                    const size_t nidx = static_cast<size_t>(nx * width_ + ny);
                    if (last_seen_sec_[nidx] < 0.0 || (stamp_sec - last_seen_sec_[nidx]) > stale_time_sec_) continue;
                    dz_up = std::max(dz_up, ground_z_filtered_[nidx] - ground_z_filtered_[idx]);
                }
            }
        } else {
            for (auto [dx, dy] : std::array<std::pair<int,int>,4>{{{1,0},{-1,0},{0,1},{0,-1}}}) {
                const int nx = ix + dx, ny = iy + dy;
                if (nx < 0 || nx >= width_ || ny < 0 || ny >= width_) continue;
                const size_t nidx = static_cast<size_t>(nx * width_ + ny);
                if (last_seen_sec_[nidx] < 0.0 || (stamp_sec - last_seen_sec_[nidx]) > stale_time_sec_) continue;
                dz_up = std::max(dz_up, ground_z_filtered_[nidx] - ground_z_filtered_[idx]);
            }
        }

        // drop dz_down: edge4/edge8 可配置
        float dz_down = 0.0f;
        if (drop_use_edge4) {
            for (auto [dx, dy] : std::array<std::pair<int, int>, 4>{{{1, 0}, {-1, 0}, {0, 1}, {0, -1}}}) {
                const int nx = ix + dx, ny = iy + dy;
                if (nx < 0 || nx >= width_ || ny < 0 || ny >= width_) continue;
                const size_t nidx = static_cast<size_t>(nx * width_ + ny);
                if (last_seen_sec_[nidx] < 0.0 || (stamp_sec - last_seen_sec_[nidx]) > stale_time_sec_) continue;
                dz_down = std::min(dz_down, ground_z_filtered_[nidx] - ground_z_filtered_[idx]);
            }
        } else {
            for (int dx = -1; dx <= 1; dx++) {
                for (int dy = -1; dy <= 1; dy++) {
                    if (dx == 0 && dy == 0) continue;
                    const int nx = ix + dx, ny = iy + dy;
                    if (nx < 0 || nx >= width_ || ny < 0 || ny >= width_) continue;
                    const size_t nidx = static_cast<size_t>(nx * width_ + ny);
                    if (last_seen_sec_[nidx] < 0.0 || (stamp_sec - last_seen_sec_[nidx]) > stale_time_sec_) continue;
                    dz_down = std::min(dz_down, ground_z_filtered_[nidx] - ground_z_filtered_[idx]);
                }
            }
        }

        const float h_above = top_z_[idx] - ground_z_filtered_[idx];
        const bool obstacle_candidate =
            (dz_up > static_cast<float>(h_obstacle_m_)) ||
            (h_above > static_cast<float>(h_obstacle_m_));
        const bool drop_candidate = (dz_down < static_cast<float>(-h_drop_m_));

        if (!enable_hysteresis_) {
            obstacle_state_[idx] = obstacle_candidate ? 1 : 0;
            drop_state_[idx] = drop_candidate ? 1 : 0;
            continue;
        }

        // 基于积分的迟滞（防止语义在“障碍/地面”之间高频闪烁）
        if (obstacle_candidate)
            obstacle_score_[idx] = std::min(score_max_, obstacle_score_[idx] + score_inc_);
        else
            obstacle_score_[idx] = std::max(0, obstacle_score_[idx] - score_dec_);

        if (drop_candidate)
            drop_score_[idx] = std::min(score_max_, drop_score_[idx] + score_inc_);
        else
            drop_score_[idx] = std::max(0, drop_score_[idx] - score_dec_);

        if (!obstacle_state_[idx] && obstacle_score_[idx] >= obstacle_on_score_)
            obstacle_state_[idx] = 1;
        if (obstacle_state_[idx] && obstacle_score_[idx] <= obstacle_off_score_)
            obstacle_state_[idx] = 0;

        if (!drop_state_[idx] && drop_score_[idx] >= drop_on_score_)
            drop_state_[idx] = 1;
        if (drop_state_[idx] && drop_score_[idx] <= drop_off_score_)
            drop_state_[idx] = 0;
    }

    // BFS 连通域去噪：清除孤立的 obstacle 小簇（面积 < min_obstacle_area_cells_）
    if (min_obstacle_area_cells_ > 1) {
        std::vector<bool> visited(static_cast<size_t>(num_cells_), false);
        for (int seed = 0; seed < num_cells_; seed++) {
            if (!obstacle_state_[static_cast<size_t>(seed)] || visited[static_cast<size_t>(seed)]) continue;
            std::vector<int> comp, stk{seed};
            visited[static_cast<size_t>(seed)] = true;
            while (!stk.empty()) {
                int cur = stk.back(); stk.pop_back();
                comp.push_back(cur);
                int cx = cur / width_, cy = cur % width_;
                for (auto [ddx, ddy] : std::array<std::pair<int,int>,4>{{{1,0},{-1,0},{0,1},{0,-1}}}) {
                    int nx = cx + ddx, ny = cy + ddy;
                    if (nx < 0 || nx >= width_ || ny < 0 || ny >= width_) continue;
                    int nc = nx * width_ + ny;
                    if (obstacle_state_[static_cast<size_t>(nc)] && !visited[static_cast<size_t>(nc)]) {
                        visited[static_cast<size_t>(nc)] = true;
                        stk.push_back(nc);
                    }
                }
            }
            if (static_cast<int>(comp.size()) < min_obstacle_area_cells_)
                for (int c : comp) obstacle_state_[static_cast<size_t>(c)] = 0;
        }
    }

    if (min_drop_area_cells_ > 1) {
        const bool use8 = (drop_neighbor_mode_ != "edge4");
        const std::array<std::pair<int, int>, 8> d8{{{1, 0}, {-1, 0}, {0, 1}, {0, -1},
                                                      {1, 1}, {1, -1}, {-1, 1}, {-1, -1}}};
        const std::array<std::pair<int, int>, 4> d4{{{1, 0}, {-1, 0}, {0, 1}, {0, -1}}};
        std::vector<bool> visited(static_cast<size_t>(num_cells_), false);
        for (int seed = 0; seed < num_cells_; seed++) {
            if (!drop_state_[static_cast<size_t>(seed)] || visited[static_cast<size_t>(seed)]) continue;
            std::vector<int> comp;
            std::vector<int> stk{seed};
            visited[static_cast<size_t>(seed)] = true;
            while (!stk.empty()) {
                const int cur = stk.back();
                stk.pop_back();
                comp.push_back(cur);
                const int cx = cur / width_;
                const int cy = cur % width_;
                auto check = [&](int dx, int dy) {
                    const int nx = cx + dx;
                    const int ny = cy + dy;
                    if (nx < 0 || nx >= width_ || ny < 0 || ny >= width_) return;
                    const int nc = nx * width_ + ny;
                    if (drop_state_[static_cast<size_t>(nc)] && !visited[static_cast<size_t>(nc)]) {
                        visited[static_cast<size_t>(nc)] = true;
                        stk.push_back(nc);
                    }
                };
                if (use8) {
                    for (const auto& [dx, dy] : d8) check(dx, dy);
                } else {
                    for (const auto& [dx, dy] : d4) check(dx, dy);
                }
            }
            if (static_cast<int>(comp.size()) < min_drop_area_cells_) {
                for (const int c : comp) {
                    drop_state_[static_cast<size_t>(c)] = 0;
                }
            }
        }
    }

    for (int cell = 0; cell < num_cells_; ++cell) {
        const size_t idx = static_cast<size_t>(cell);
        if (!cell_in_radius_[idx]) {
            continue;
        }
        const bool fresh = (last_seen_sec_[idx] >= 0.0) &&
                           ((stamp_sec - last_seen_sec_[idx]) <= stale_time_sec_);
        if (!fresh) {
            continue;
        }

        const int ix = cell / width_;
        const int iy = cell % width_;
        const float center = ground_z_filtered_[idx];

        auto getFreshGround = [&](int nx, int ny, float& val) -> bool {
            if (nx < 0 || nx >= width_ || ny < 0 || ny >= width_) {
                return false;
            }
            const size_t nidx = static_cast<size_t>(nx * width_ + ny);
            const bool nfresh = (last_seen_sec_[nidx] >= 0.0) &&
                                ((stamp_sec - last_seen_sec_[nidx]) <= stale_time_sec_);
            if (!nfresh) {
                return false;
            }
            val = ground_z_filtered_[nidx];
            return true;
        };

        float xp = 0.0f, xm = 0.0f, yp = 0.0f, ym = 0.0f;
        const bool has_xp = getFreshGround(ix + 1, iy, xp);
        const bool has_xm = getFreshGround(ix - 1, iy, xm);
        const bool has_yp = getFreshGround(ix, iy + 1, yp);
        const bool has_ym = getFreshGround(ix, iy - 1, ym);

        if (has_xp && has_xm) {
            slope_x_[idx] = (xp - xm) / static_cast<float>(2.0 * grid_resolution_m_);
        } else if (has_xp) {
            slope_x_[idx] = (xp - center) / static_cast<float>(grid_resolution_m_);
        } else if (has_xm) {
            slope_x_[idx] = (center - xm) / static_cast<float>(grid_resolution_m_);
        }

        if (has_yp && has_ym) {
            slope_y_[idx] = (yp - ym) / static_cast<float>(2.0 * grid_resolution_m_);
        } else if (has_yp) {
            slope_y_[idx] = (yp - center) / static_cast<float>(grid_resolution_m_);
        } else if (has_ym) {
            slope_y_[idx] = (center - ym) / static_cast<float>(grid_resolution_m_);
        }

        std::array<float, 8> neigh{};
        int neigh_count = 0;
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                if (dx == 0 && dy == 0) {
                    continue;
                }
                float value = 0.0f;
                if (getFreshGround(ix + dx, iy + dy, value)) {
                    neigh[static_cast<size_t>(neigh_count++)] = value;
                }
            }
        }
        if (neigh_count >= 3) {
            float mean = 0.0f;
            for (int i = 0; i < neigh_count; ++i) {
                mean += neigh[static_cast<size_t>(i)];
            }
            mean /= static_cast<float>(neigh_count);
            float var = 0.0f;
            for (int i = 0; i < neigh_count; ++i) {
                const float d = neigh[static_cast<size_t>(i)] - mean;
                var += d * d;
            }
            roughness_[idx] = var / static_cast<float>(neigh_count);
        }
    }
}

void TerrainSemanticNode::publishOutputs(const rclcpp::Time& stamp, double base_x,
                                         double base_y, double base_z,
                                         double cos_yaw, double sin_yaw) {
    pcl::PointCloud<pcl::PointXYZI> obs_cloud, drop_cloud, climb_cloud, kfs_debug_cloud;
    const double stamp_sec = stamp.seconds();
    const bool publish_climbable = static_cast<bool>(pub_climbable_);
    const bool publish_kfs_debug = static_cast<bool>(pub_kfs_obstacles_debug_);
    const bool should_publish_features =
        static_cast<bool>(pub_features_) && isThrottleReady(stamp, last_features_pub_time_, terrain_features_publish_hz_);
    const bool should_publish_speed_limit =
        static_cast<bool>(pub_speed_limit_) && isThrottleReady(stamp, last_speed_limit_pub_time_, terrain_speed_limit_publish_hz_);

    rc26_interfaces::msg::TerrainFeatureGrid feature_msg;
    if (should_publish_features) {
        feature_msg.header.stamp = stamp;
        feature_msg.header.frame_id = base_frame_;
        feature_msg.resolution_m = static_cast<float>(grid_resolution_m_);
        feature_msg.width = static_cast<uint32_t>(width_);
        feature_msg.height = static_cast<uint32_t>(width_);
        feature_msg.origin.position.x = -static_cast<double>(half_width_) * grid_resolution_m_;
        feature_msg.origin.position.y = -static_cast<double>(half_width_) * grid_resolution_m_;
        feature_msg.origin.position.z = 0.0;
        feature_msg.origin.orientation.x = 0.0;
        feature_msg.origin.orientation.y = 0.0;
        feature_msg.origin.orientation.z = 0.0;
        feature_msg.origin.orientation.w = 1.0;
        const size_t sz = static_cast<size_t>(num_cells_);
        feature_msg.in_radius.assign(sz, 0U);
        feature_msg.fresh.assign(sz, 0U);
        feature_msg.density.assign(sz, 0U);
        feature_msg.h_ground.assign(sz, 0.0f);
        feature_msg.sigma_h.assign(sz, 0.0f);
        feature_msg.h_top.assign(sz, 0.0f);
        feature_msg.slope_x.assign(sz, 0.0f);
        feature_msg.slope_y.assign(sz, 0.0f);
        feature_msg.roughness.assign(sz, 0.0f);
        feature_msg.p_obstacle.assign(sz, 0.0f);
        feature_msg.p_drop.assign(sz, 0.0f);
        feature_msg.step_up.assign(sz, 0.0f);
        feature_msg.p_climbable.assign(sz, 0.0f);
    }

    obstacle_cells_count_ = 0;
    drop_cells_count_ = 0;
    climbable_cells_count_ = 0;
    int kfs_cells_count = 0;
    int roi_fresh_count = 0;
    double roi_max_slope = 0.0;
    double roi_max_rough = 0.0;
    double roi_max_p_drop = 0.0;
    double roi_max_p_climbable = 0.0;

    for (int cell = 0; cell < num_cells_; cell++) {
        const size_t idx = static_cast<size_t>(cell);

        const int ix = cell / width_;
        const int iy = cell % width_;
        const double x_rel = (static_cast<double>(ix - half_width_) + 0.5) * grid_resolution_m_;
        const double y_rel = (static_cast<double>(iy - half_width_) + 0.5) * grid_resolution_m_;
        const double x = base_x + cos_yaw * x_rel - sin_yaw * y_rel;
        const double y = base_y + sin_yaw * x_rel + cos_yaw * y_rel;

        float z = static_cast<float>(base_z);
        const double last = last_seen_sec_[idx];
        const bool fresh = (last >= 0.0) && ((stamp_sec - last) <= stale_time_sec_);
        if (fresh) z = ground_z_filtered_[idx];
        const float obstacle_prob_proxy = std::clamp(
            static_cast<float>(obstacle_score_[idx]) / static_cast<float>(score_max_), 0.0f, 1.0f);
        const float drop_prob_proxy = std::clamp(
            static_cast<float>(drop_score_[idx]) / static_cast<float>(score_max_), 0.0f, 1.0f);
        float obstacle_prob = obstacle_prob_proxy;
        float drop_prob = drop_prob_proxy;
        float step_up = 0.0f;
        float climbable_prob = 0.0f;

        if (should_publish_features) {
            feature_msg.in_radius[idx] = cell_in_radius_[idx] ? 1U : 0U;
            feature_msg.fresh[idx] = fresh ? 1U : 0U;
            feature_msg.density[idx] = fresh ? density_[idx] : 0U;
            feature_msg.h_ground[idx] = fresh ? (ground_z_filtered_[idx] - static_cast<float>(base_z)) : 0.0f;
            feature_msg.sigma_h[idx] = fresh ? sigma_h_[idx] : 0.0f;
            feature_msg.h_top[idx] = fresh ? (top_z_[idx] - static_cast<float>(base_z)) : 0.0f;
            feature_msg.slope_x[idx] = fresh ? slope_x_[idx] : 0.0f;
            feature_msg.slope_y[idx] = fresh ? slope_y_[idx] : 0.0f;
            feature_msg.roughness[idx] = fresh ? roughness_[idx] : 0.0f;
            feature_msg.step_up[idx] = 0.0f;
            feature_msg.p_climbable[idx] = 0.0f;
        }

        if (!cell_in_radius_[idx]) continue;

        const bool is_obstacle = obstacle_state_[idx] != 0;
        const bool is_drop = drop_state_[idx] != 0;
        const bool is_kfs_occupied = kfs_occupied_state_[idx] != 0;

        if (is_obstacle) {
            ++obstacle_cells_count_;
        }
        if (is_kfs_occupied) {
            ++kfs_cells_count;
        }

        if (fresh && !is_obstacle && !is_drop) {
            for (int dx = -1; dx <= 1; dx++) {
                for (int dy = -1; dy <= 1; dy++) {
                    if (dx == 0 && dy == 0) continue;
                    const int nx = ix + dx, ny = iy + dy;
                    if (nx < 0 || nx >= width_ || ny < 0 || ny >= width_) continue;

                    const size_t nidx = static_cast<size_t>(nx * width_ + ny);
                    const double nlast = last_seen_sec_[nidx];
                    const bool nfresh = (nlast >= 0.0) && ((stamp_sec - nlast) <= stale_time_sec_);
                    if (!nfresh) continue;

                    const float diff = ground_z_filtered_[nidx] - ground_z_filtered_[idx];
                    step_up = std::max(step_up, diff);
                }
            }

            if (step_up >= static_cast<float>(climbable_min_dz_m_) &&
                step_up <= static_cast<float>(h_climb_m_)) {
                const double denom = std::max(1e-6, h_climb_m_ - climbable_min_dz_m_);
                climbable_prob = std::clamp(
                    static_cast<float>((static_cast<double>(step_up) - climbable_min_dz_m_) / denom),
                    0.0f, 1.0f);
                if (publish_climbable) {
                    pcl::PointXYZI p;
                    p.x = static_cast<float>(x);
                    p.y = static_cast<float>(y);
                    p.z = z;
                    p.intensity = step_up;
                    climb_cloud.push_back(p);
                    ++climbable_cells_count_;
                }
            }
        }

        if (fresh && terrain_risk_model_ready_) {
            TerrainRiskFeatures risk_features;
            risk_features.slope_abs = std::max(std::abs(slope_x_[idx]), std::abs(slope_y_[idx]));
            risk_features.roughness = roughness_[idx];
            risk_features.sigma_h = sigma_h_[idx];
            risk_features.step_up = step_up;
            risk_features.height_span = std::max(0.0f, top_z_[idx] - ground_z_filtered_[idx]);
            risk_features.obstacle_proxy = obstacle_prob_proxy;
            risk_features.drop_proxy = drop_prob_proxy;
            risk_features.climbable_prob = climbable_prob;
            obstacle_prob = terrain_risk_model_.predictObstacle(risk_features, obstacle_prob_proxy);
            drop_prob = terrain_risk_model_.predictDrop(risk_features, drop_prob_proxy);
        }

        if (should_publish_features) {
            feature_msg.p_obstacle[idx] = obstacle_prob;
            feature_msg.p_drop[idx] = drop_prob;
            feature_msg.step_up[idx] = fresh ? step_up : 0.0f;
            feature_msg.p_climbable[idx] = climbable_prob;
        }

        if (fresh &&
            x_rel >= 0.0 && x_rel <= speed_limit_forward_look_m_ &&
            std::abs(y_rel) <= speed_limit_half_width_m_) {
            ++roi_fresh_count;
            const double cell_slope = std::max(std::abs(static_cast<double>(slope_x_[idx])),
                                               std::abs(static_cast<double>(slope_y_[idx])));
            roi_max_slope = std::max(roi_max_slope, cell_slope);
            roi_max_rough = std::max(roi_max_rough, static_cast<double>(roughness_[idx]));
            roi_max_p_drop = std::max(roi_max_p_drop, static_cast<double>(drop_prob));
            roi_max_p_climbable = std::max(roi_max_p_climbable, static_cast<double>(climbable_prob));
        }

        if (is_obstacle) {
            pcl::PointXYZI p;
            p.x = static_cast<float>(x);
            p.y = static_cast<float>(y);
            p.z = z;
            p.intensity = static_cast<float>(obstacle_score_[idx]);
            obs_cloud.push_back(p);
        }
        if (is_kfs_occupied) {
            pcl::PointXYZI p;
            p.x = static_cast<float>(x);
            p.y = static_cast<float>(y);
            p.z = z + 0.35F;
            p.intensity = 200.0F;
            if (include_kfs_in_obstacles_ && !is_obstacle) {
                obs_cloud.push_back(p);
            }
            if (publish_kfs_debug) {
                kfs_debug_cloud.push_back(p);
            }
        }
        if (is_drop) {
            bool in_sector = true;
            if (drop_forward_sector_deg_ < 360.0) {
                if (x_rel < drop_forward_min_x_m_) {
                    in_sector = false;
                } else {
                    double angle_rad = std::abs(std::atan2(y_rel, x_rel));
                    double sector_half_rad = drop_forward_sector_deg_ * 0.5 * M_PI / 180.0;
                    if (angle_rad > sector_half_rad) {
                        in_sector = false;
                    }
                }
            }

            if (in_sector) {
                pcl::PointXYZI p;
                p.x = static_cast<float>(x);
                p.y = static_cast<float>(y);
                p.z = z;
                p.intensity = static_cast<float>(drop_score_[idx]);
                drop_cloud.push_back(p);
                ++drop_cells_count_;
            }
        }

        // Unknown 区域策略：保守模式下将 Unknown 视为风险点输出给下游导航链
        if (!fresh && !is_obstacle && !is_drop && unknown_policy_ == "conservative") {
            pcl::PointXYZI p;
            p.x = static_cast<float>(x);
            p.y = static_cast<float>(y);
            p.z = z;
            p.intensity = 0.0f;
            if (unknown_output_ == "drop")
                drop_cloud.push_back(p);
            else
                obs_cloud.push_back(p);
        }

    }
    kfs_occupied_count_ = kfs_cells_count;

    TerrainStats stats;
    stats.kfs_occupied_cells = kfs_occupied_count_;
    stats.obstacle_cells = obstacle_cells_count_;
    stats.drop_cells = drop_cells_count_;
    stats.climbable_cells = climbable_cells_count_;
    safety_guard_.updateStats(stats);

    if (output_sanity_check_enable_) {
        std::string reason;
        bool ok_obs = sanitizeAndValidateCloud(obs_cloud, "obstacles", reason);
        bool ok_drop = sanitizeAndValidateCloud(drop_cloud, "drop", reason);
        bool ok_kfs_debug = true;
        if (publish_kfs_debug) {
            ok_kfs_debug = sanitizeAndValidateCloud(kfs_debug_cloud, "kfs_debug", reason);
        }

        int total_points = static_cast<int>(obs_cloud.size() + drop_cloud.size());
        if (publish_climbable) total_points += static_cast<int>(climb_cloud.size());
        if (publish_kfs_debug) total_points += static_cast<int>(kfs_debug_cloud.size());

        if (!ok_obs || !ok_drop || !ok_kfs_debug || total_points > output_max_points_total_) {
            const auto decision = safety_guard_.forceFailSafe("output_sanity", "输出点云异常: " + reason);
            syncSafetyGuardState(decision);

            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "输出点云异常: obs=%zu drop=%zu kfs_dbg=%zu total=%d, %s",
                obs_cloud.size(), drop_cloud.size(), kfs_debug_cloud.size(), total_points, reason.c_str());

            publishSpeedLimitValue(stamp, 0.0f, true);
            publishEmergencyStop(stamp);
            publishDiagnostics(stamp, diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                "输出点云异常: " + reason);
            return;
        }

        safety_guard_.clearForcedFailSafe("output_sanity");
        syncSafetyGuardState(safety_guard_.decision());
    }

    sensor_msgs::msg::PointCloud2 obs_msg, drop_msg, climb_msg, kfs_debug_msg;
    pcl::toROSMsg(obs_cloud, obs_msg);
    obs_msg.header.stamp = stamp;
    obs_msg.header.frame_id = target_frame_;
    if (pub_obstacles_) pub_obstacles_->publish(obs_msg);

    pcl::toROSMsg(drop_cloud, drop_msg);
    drop_msg.header.stamp = stamp;
    drop_msg.header.frame_id = target_frame_;
    if (pub_drop_) pub_drop_->publish(drop_msg);

    if (publish_climbable) {
        pcl::toROSMsg(climb_cloud, climb_msg);
        climb_msg.header.stamp = stamp;
        climb_msg.header.frame_id = target_frame_;
        pub_climbable_->publish(climb_msg);
    }
    if (publish_kfs_debug) {
        pcl::toROSMsg(kfs_debug_cloud, kfs_debug_msg);
        kfs_debug_msg.header.stamp = stamp;
        kfs_debug_msg.header.frame_id = target_frame_;
        pub_kfs_obstacles_debug_->publish(kfs_debug_msg);
    }

    if (should_publish_features) {
        pub_features_->publish(feature_msg);
        last_features_pub_time_ = stamp;
    }

    if (should_publish_speed_limit) {
        float v_limit = static_cast<float>(speed_limit_v_max_mps_);
        if (fail_safe_active_ || latency_intervention_active_) {
            v_limit = 0.0f;
        } else if (roi_fresh_count > 0) {
            const double tci = speed_limit_w_slope_ * roi_max_slope +
                               speed_limit_w_roughness_ * roi_max_rough +
                               speed_limit_w_drop_ * roi_max_p_drop +
                               speed_limit_w_climbable_ * roi_max_p_climbable;
            v_limit = static_cast<float>(speed_limit_v_max_mps_ * std::exp(-speed_limit_k_tci_ * tci));
            v_limit = std::clamp(v_limit, static_cast<float>(speed_limit_min_mps_),
                                 static_cast<float>(speed_limit_v_max_mps_));
            if (roi_max_p_drop >= speed_limit_emergency_drop_thresh_) {
                v_limit = 0.0f;
            }
        }
        publishSpeedLimitValue(stamp, v_limit, true);
    }
}

void TerrainSemanticNode::cloudCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
    if (!msg) return;

    const auto t_start = std::chrono::steady_clock::now();

    const rclcpp::Time stamp(msg->header.stamp);
    const double stamp_sec = stamp.seconds();

    auto publishDiagnosticsWithLatency = [&](int base_level, const std::string& base_msg) {
        last_latency_ms_ = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t_start).count();

        int level = base_level;
        std::string msg_text = base_msg;
        if (last_latency_ms_ > latency_error_ms_) {
            level = std::max(level, static_cast<int>(diagnostic_msgs::msg::DiagnosticStatus::ERROR));
            if (msg_text.find("处理延迟超限") == std::string::npos) msg_text += "; 处理延迟超限";
        } else if (last_latency_ms_ > latency_warn_ms_) {
            level = std::max(level, static_cast<int>(diagnostic_msgs::msg::DiagnosticStatus::WARN));
            if (msg_text.find("处理延迟告警") == std::string::npos) msg_text += "; 处理延迟告警";
        }
        publishDiagnostics(stamp, level, msg_text);
    };

    received_cloud_ = true;
    last_cloud_stamp_ = stamp;

    // 清空上一帧缓存（即使中途 return，也不会累积到下一帧）
    for (int cell : touched_cells_) {
        cell_z_samples_[static_cast<size_t>(cell)].clear();
    }
    touched_cells_.clear();

    const auto tf_base = getTransform(target_frame_, base_frame_, stamp);
    if (!tf_base) {
        // TF 异常：进入降级模式（可选），同时仍对历史栅格做衰减
        if (enable_fail_safe_ && fail_safe_strategy_ != "none") {
            const bool was_fail_safe_active = fail_safe_active_;
            const auto decision = safety_guard_.forceFailSafe("tf_lookup_base", "TF(base) 查询失败");
            syncSafetyGuardState(decision);
            if (!was_fail_safe_active && fail_safe_active_) {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                      "进入降级保护模式: TF(base) 查询失败");
            }
        }
        classifyAndUpdate(stamp_sec);

        if (fail_safe_strategy_ == "emergency_stop") {
            publishEmergencyStop(this->get_clock()->now());
        } else if (fail_safe_strategy_ == "virtual_fence" && have_last_pose_) {
            publishVirtualFence(stamp, last_base_x_, last_base_y_, last_base_z_, last_cos_yaw_, last_sin_yaw_);
        } else {
            publishEmergencyStop(this->get_clock()->now());
        }

        publishSpeedLimitValue(stamp, 0.0f, true);

        publishDiagnosticsWithLatency(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "TF(base) 查询失败");
        return;
    }
    safety_guard_.clearForcedFailSafe("tf_lookup_base");
    syncSafetyGuardState(safety_guard_.decision());

    // 获取机器人位姿（严格按时间戳 TF 同步）
    const auto& origin = tf_base->getOrigin();
    const double base_x = origin.x();
    const double base_y = origin.y();
    const double base_z = origin.z();

    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_base->getRotation()).getRPY(roll, pitch, yaw);
    last_roll_rad_ = roll;
    last_pitch_rad_ = pitch;
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);

    // 保存最近一次可用位姿，供失效保护备用
    last_base_x_ = base_x;
    last_base_y_ = base_y;
    last_base_z_ = base_z;
    last_cos_yaw_ = cos_yaw;
    last_sin_yaw_ = sin_yaw;
    have_last_pose_ = true;

    if (msg->data.empty() || msg->header.frame_id.empty()) {
        classifyAndUpdate(stamp_sec);
        publishOutputs(stamp, base_x, base_y, base_z, cos_yaw, sin_yaw);
        publishDiagnosticsWithLatency(diagnostic_msgs::msg::DiagnosticStatus::WARN, "点云为空或 frame_id 为空");
        return;
    }

    const auto tf_cloud = getTransform(target_frame_, msg->header.frame_id, stamp);
    if (!tf_cloud) {
        if (enable_fail_safe_ && fail_safe_strategy_ != "none") {
            const bool was_fail_safe_active = fail_safe_active_;
            const auto decision = safety_guard_.forceFailSafe("tf_lookup_cloud", "TF(点云) 查询失败");
            syncSafetyGuardState(decision);
            if (!was_fail_safe_active && fail_safe_active_) {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                      "进入降级保护模式: TF(点云) 查询失败");
            }
        }
        classifyAndUpdate(stamp_sec);
        publishOutputs(stamp, base_x, base_y, base_z, cos_yaw, sin_yaw);
        if (fail_safe_active_) {
            publishSpeedLimitValue(stamp, 0.0f, true);
        }
        if (fail_safe_active_ && enable_fail_safe_ && fail_safe_strategy_ == "virtual_fence") {
            publishVirtualFence(stamp, base_x, base_y, base_z, cos_yaw, sin_yaw);
        }
        publishDiagnosticsWithLatency(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "TF(点云) 查询失败");
        return;
    }

    safety_guard_.clearForcedFailSafe("tf_lookup_cloud");
    syncSafetyGuardState(safety_guard_.decision());
    last_good_tf_stamp_ = stamp;

    // 将点云严格变换到 target_frame（基于 header 时间戳）
    sensor_msgs::msg::PointCloud2 cloud_transformed;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ds(new pcl::PointCloud<pcl::PointXYZ>());
    try {
        pcl_ros::transformPointCloud(target_frame_, *tf_cloud, *msg, cloud_transformed);
        pcl::fromROSMsg(cloud_transformed, *cloud);

        // 体素下采样，降低计算量
        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        voxel.setLeafSize(static_cast<float>(voxel_leaf_size_m_),
                          static_cast<float>(voxel_leaf_size_m_),
                          static_cast<float>(voxel_leaf_size_m_));
        voxel.setInputCloud(cloud);
        voxel.filter(*cloud_ds);
    } catch (const std::exception& ex) {
        if (enable_fail_safe_ && fail_safe_strategy_ != "none") {
            const bool was_fail_safe_active = fail_safe_active_;
            const auto decision = safety_guard_.forceFailSafe(
                "cloud_preprocess", std::string("点云预处理失败: ") + ex.what());
            syncSafetyGuardState(decision);
            if (!was_fail_safe_active && fail_safe_active_) {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                      "进入降级保护模式: 点云预处理失败: %s", ex.what());
            }
        }
        classifyAndUpdate(stamp_sec);
        publishOutputs(stamp, base_x, base_y, base_z, cos_yaw, sin_yaw);
        if (fail_safe_active_ && enable_fail_safe_ && fail_safe_strategy_ == "virtual_fence" && have_last_pose_) {
            publishVirtualFence(stamp, base_x, base_y, base_z, cos_yaw, sin_yaw);
        } else if (fail_safe_active_) {
            publishEmergencyStop(stamp);
        }
        if (fail_safe_active_) {
            publishSpeedLimitValue(stamp, 0.0f, true);
        }
        publishDiagnosticsWithLatency(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                                      std::string("点云预处理失败: ") + ex.what());
        return;
    }

    safety_guard_.clearForcedFailSafe("cloud_preprocess");
    syncSafetyGuardState(safety_guard_.decision());

    const double r2 = perception_radius_m_ * perception_radius_m_;
    const bool apply_roll_compensation =
        (roll_compensation_mode_ == "always") ||
        (roll_compensation_mode_ == "gated" &&
         std::abs(last_roll_rad_) >= (roll_gate_deg_ * M_PI / 180.0));

    // 将点云投影到以机器人为中心的局部栅格（核心判别使用相对高度，不依赖绝对 Z）
    for (const auto& p : cloud_ds->points) {
        // 跳过非法点，避免后续出现 NaN/UB
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;

        const double dx = static_cast<double>(p.x) - base_x;
        const double dy = static_cast<double>(p.y) - base_y;

        // 旋转到机器人朝向坐标系（仅用 yaw）
        const double x_rel = cos_yaw * dx + sin_yaw * dy;
        const double y_rel = -sin_yaw * dx + cos_yaw * dy;
        double z_corr = static_cast<double>(p.z);
        if (enable_pitch_compensation_ && x_rel > 0.0) {
            z_corr -= x_rel * std::tan(last_pitch_rad_);
        }
        if (apply_roll_compensation) {
            z_corr -= y_rel * std::tan(last_roll_rad_);
        }
        const double rel_z = z_corr - base_z;
        const double d2 = x_rel * x_rel + y_rel * y_rel;
        if (d2 > r2) continue;

        const double r = std::sqrt(d2);
        if (rel_z < (min_rel_z_m_ - dis_ratio_z_ * r) ||
            rel_z > (max_rel_z_m_ + dis_ratio_z_ * r))
            continue;

        int ix = static_cast<int>(std::floor(x_rel / grid_resolution_m_)) + half_width_;
        int iy = static_cast<int>(std::floor(y_rel / grid_resolution_m_)) + half_width_;
        ix = std::clamp(ix, 0, width_ - 1);
        iy = std::clamp(iy, 0, width_ - 1);

        const int cell = ix * width_ + iy;
        auto& bucket = cell_z_samples_[static_cast<size_t>(cell)];
        if (bucket.empty()) touched_cells_.push_back(cell);
        bucket.push_back(static_cast<float>(z_corr));
    }

    estimateCellHeights(stamp_sec);
    classifyAndUpdate(stamp_sec);
    publishOutputs(stamp, base_x, base_y, base_z, cos_yaw, sin_yaw);

    last_latency_ms_ = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t_start).count();
}

void TerrainSemanticNode::publishEmergencyStop(const rclcpp::Time& stamp) const {
    pcl::PointCloud<pcl::PointXYZI> fence_cloud;
    fence_cloud.reserve(static_cast<size_t>(virtual_fence_num_points_));

    const double r = virtual_fence_radius_m_;
    for (int i = 0; i < virtual_fence_num_points_; i++) {
        const double a = 2.0 * M_PI * i / virtual_fence_num_points_;
        pcl::PointXYZI p;
        p.x = static_cast<float>(r * std::cos(a));
        p.y = static_cast<float>(r * std::sin(a));
        p.z = static_cast<float>(virtual_fence_height_m_);
        p.intensity = 1.0f;
        fence_cloud.push_back(p);
    }

    sensor_msgs::msg::PointCloud2 fence_msg;
    pcl::toROSMsg(fence_cloud, fence_msg);
    fence_msg.header.stamp = stamp;
    fence_msg.header.frame_id = base_frame_;

    if (pub_obstacles_) pub_obstacles_->publish(fence_msg);
    if (pub_drop_) pub_drop_->publish(fence_msg);
}

bool TerrainSemanticNode::sanitizeAndValidateCloud(
    pcl::PointCloud<pcl::PointXYZI>& cloud,
    const std::string& name,
    std::string& reason) const {

    size_t original_size = cloud.size();
    auto it = std::remove_if(cloud.begin(), cloud.end(),
        [](const pcl::PointXYZI& p) {
            return !std::isfinite(p.x) || !std::isfinite(p.y) ||
                   !std::isfinite(p.z) || !std::isfinite(p.intensity);
        });
    cloud.erase(it, cloud.end());

    size_t removed = original_size - cloud.size();
    if (removed > 0) {
        reason = name + " 移除了 " + std::to_string(removed) + " 个非法点";
    }

    if (output_max_points_per_cloud_ > 0 &&
        static_cast<int>(cloud.size()) > output_max_points_per_cloud_) {
        reason = name + " 点数超限: " + std::to_string(cloud.size()) +
                 " > " + std::to_string(output_max_points_per_cloud_);
        return false;
    }

    return true;
}

}  // namespace rc26_terrain

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rc26_terrain::TerrainSemanticNode)
