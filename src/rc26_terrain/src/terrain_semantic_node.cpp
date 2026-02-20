#include "rc26_terrain/terrain_semantic_node.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>

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
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

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

    this->declare_parameter<std::string>("target_frame", "odom");
    this->declare_parameter<std::string>("base_frame", "base_link");

    // TF 查询超时（兼容旧参数 tf_timeout_sec，同时提供 transform_tolerance）
    this->declare_parameter<double>("tf_timeout_sec", tf_timeout_sec_);
    this->declare_parameter<double>("transform_tolerance", -1.0);

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
    this->declare_parameter<double>     ("jump_thresh_m",          jump_thresh_m_);
    this->declare_parameter<int>        ("freeze_max_frames",      freeze_max_frames_);
    this->declare_parameter<double>     ("ground_ema_alpha_slow",  ground_ema_alpha_slow_);
    this->declare_parameter<bool>       ("enable_pitch_compensation", enable_pitch_compensation_);
    this->declare_parameter<double>     ("stair_gate_speed_mps",   stair_gate_speed_mps_);
    this->declare_parameter<double>     ("stair_pitch_gate_deg",   stair_pitch_gate_deg_);
    this->declare_parameter<double>     ("top_z_max_delta_m",      top_z_max_delta_m_);

    // P0.3: latency diagnostics
    this->declare_parameter<double>     ("latency_warn_ms",        latency_warn_ms_);
    this->declare_parameter<double>     ("latency_error_ms",       latency_error_ms_);
    this->declare_parameter<int>        ("latency_trigger_frames", latency_trigger_frames_);
    this->declare_parameter<int>        ("latency_recover_frames", latency_recover_frames_);
    this->declare_parameter<std::string>("latency_intervention_mode", latency_intervention_mode_);

    // 读取参数
    this->get_parameter("input_cloud_topic", input_cloud_topic_);
    this->get_parameter("odom_topic", odom_topic_);
    this->get_parameter("output_obstacles_topic", output_obstacles_topic_);
    this->get_parameter("output_drop_topic", output_drop_topic_);
    this->get_parameter("output_climbable_topic", output_climbable_topic_);
    this->get_parameter("diagnostics_topic", diagnostics_topic_);
    this->get_parameter("base_ground_stable_topic", base_ground_stable_topic_);
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
    this->get_parameter("diagnostics_qos_depth", diagnostics_qos_depth_);
    this->get_parameter("diagnostics_qos_reliability", diagnostics_qos_reliability_);
    this->get_parameter("diagnostics_qos_durability", diagnostics_qos_durability_);
    this->get_parameter("enable_diagnostics", enable_diagnostics_);
    this->get_parameter("enable_fail_safe", enable_fail_safe_);
    this->get_parameter("fail_safe_strategy", fail_safe_strategy_);
    this->get_parameter("virtual_fence_radius_m", virtual_fence_radius_m_);
    this->get_parameter("virtual_fence_num_points", virtual_fence_num_points_);
    this->get_parameter("virtual_fence_height_m", virtual_fence_height_m_);
    this->get_parameter("obstacle_neighbor_mode", obstacle_neighbor_mode_);
    this->get_parameter("drop_neighbor_mode",     drop_neighbor_mode_);
    this->get_parameter("min_obstacle_area_cells",min_obstacle_area_cells_);
    this->get_parameter("jump_thresh_m",          jump_thresh_m_);
    this->get_parameter("freeze_max_frames",      freeze_max_frames_);
    this->get_parameter("ground_ema_alpha_slow",  ground_ema_alpha_slow_);
    this->get_parameter("enable_pitch_compensation", enable_pitch_compensation_);
    this->get_parameter("stair_gate_speed_mps",   stair_gate_speed_mps_);
    this->get_parameter("stair_pitch_gate_deg",   stair_pitch_gate_deg_);
    this->get_parameter("top_z_max_delta_m",      top_z_max_delta_m_);
    this->get_parameter("latency_warn_ms",        latency_warn_ms_);
    this->get_parameter("latency_error_ms",       latency_error_ms_);
    this->get_parameter("latency_trigger_frames", latency_trigger_frames_);
    this->get_parameter("latency_recover_frames", latency_recover_frames_);
    this->get_parameter("latency_intervention_mode", latency_intervention_mode_);

    // transform_tolerance: 若配置了该参数，则覆盖 tf_timeout_sec
    double transform_tolerance = -1.0;
    this->get_parameter("transform_tolerance", transform_tolerance);
    if (transform_tolerance >= 0.0) {
        tf_timeout_sec_ = transform_tolerance;
    }

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
    min_points_per_cell_ = std::max(1, min_points_per_cell_);
    score_max_ = std::max(1, score_max_);
    score_inc_ = std::max(0, score_inc_);
    score_dec_ = std::max(0, score_dec_);
    virtual_fence_num_points_ = std::max(3, virtual_fence_num_points_);
    virtual_fence_radius_m_ = std::max(0.05, virtual_fence_radius_m_);
    climbable_min_dz_m_ = std::clamp(climbable_min_dz_m_, 0.0, h_climb_m_);
    freeze_max_frames_ = std::max(1, freeze_max_frames_);
    ground_ema_alpha_slow_ = std::clamp(ground_ema_alpha_slow_, 0.01, 1.0);
    stair_gate_speed_mps_ = std::max(0.0, stair_gate_speed_mps_);
    stair_pitch_gate_deg_ = std::max(0.0, stair_pitch_gate_deg_);
    top_z_max_delta_m_ = std::max(0.1, top_z_max_delta_m_);
    latency_trigger_frames_ = std::max(1, latency_trigger_frames_);
    latency_recover_frames_ = std::max(1, latency_recover_frames_);

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

    // 构建 QoS（此处会校验字符串有效性）
    const auto cloud_qos = makeQoS(cloud_qos_depth_, cloud_qos_reliability_, cloud_qos_durability_);
    const auto odom_qos = makeQoS(odom_qos_depth_, odom_qos_reliability_, odom_qos_durability_);
    const auto output_qos = makeQoS(output_qos_depth_, output_qos_reliability_, output_qos_durability_);
    const auto diag_qos =
        makeQoS(diagnostics_qos_depth_, diagnostics_qos_reliability_, diagnostics_qos_durability_);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    initGrid();

    pub_obstacles_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_obstacles_topic_, output_qos);
    pub_drop_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_drop_topic_, output_qos);
    if (!output_climbable_topic_.empty()) {
        pub_climbable_ =
            this->create_publisher<sensor_msgs::msg::PointCloud2>(output_climbable_topic_, output_qos);
    }
    if (enable_diagnostics_) {
        pub_diagnostics_ =
            this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(diagnostics_topic_, diag_qos);
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

    sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_cloud_topic_, cloud_qos,
        std::bind(&TerrainSemanticNode::cloudCallback, this, std::placeholders::_1));

    // 使用 wall timer 做健康检测；实际超时判断基于 ROS Time（兼容 use_sim_time）
    health_timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                            std::bind(&TerrainSemanticNode::healthTimerCallback, this));

    RCLCPP_INFO(this->get_logger(),
                "rc26_terrain 已启动: 栅格=%dx%d 分辨率=%.2fm 半径=%.1fm, 输入=%s, 输出=%s/%s",
                width_, width_, grid_resolution_m_, perception_radius_m_,
                input_cloud_topic_.c_str(), output_obstacles_topic_.c_str(), output_drop_topic_.c_str());
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

void TerrainSemanticNode::healthTimerCallback() {
    const auto now = this->get_clock()->now();
    const double since_start = (now - node_start_time_).seconds();
    const bool in_grace = since_start < startup_grace_sec_;

    const double cloud_age = received_cloud_ ? (now - last_cloud_stamp_).seconds()
                                             : std::numeric_limits<double>::infinity();
    const double odom_age = received_odom_ ? (now - last_odom_stamp_).seconds()
                                           : std::numeric_limits<double>::infinity();
    const double tf_age = (last_good_tf_stamp_.nanoseconds() > 0)
                              ? (now - last_good_tf_stamp_).seconds()
                              : std::numeric_limits<double>::infinity();

    const bool cloud_ok = in_grace || (received_cloud_ && cloud_age <= cloud_timeout_sec_);
    const bool odom_ok = in_grace || (received_odom_ && odom_age <= odom_timeout_sec_);

    // 只有当点云仍在持续输入时，才强制要求近期 TF 成功过
    const bool need_tf = (received_cloud_ && cloud_age <= cloud_timeout_sec_);
    const bool tf_ok = !need_tf || (tf_age <= tf_health_timeout_sec_);

    bool want_fail_safe = false;
    std::string reason;
    if (enable_fail_safe_ && fail_safe_strategy_ != "none" && !in_grace) {
        if (!cloud_ok) {
            want_fail_safe = true;
            reason = "点云输入超时/中断";
        } else if (!odom_ok) {
            want_fail_safe = true;
            reason = "里程计输入超时/中断";
        } else if (!tf_ok) {
            want_fail_safe = true;
            reason = "TF 变换链异常/超时";
        }
    }

    if (want_fail_safe && !fail_safe_active_) {
        fail_safe_active_ = true;
        fail_safe_reason_ = reason;
        RCLCPP_ERROR(this->get_logger(), "进入降级保护模式: %s", fail_safe_reason_.c_str());
    } else if (!want_fail_safe && fail_safe_active_) {
        fail_safe_active_ = false;
        fail_safe_reason_.clear();
        RCLCPP_INFO(this->get_logger(), "退出降级保护模式");
    }

    if (fail_safe_active_ && enable_fail_safe_ && fail_safe_strategy_ == "virtual_fence" && have_last_pose_) {
        publishVirtualFence(now, last_base_x_, last_base_y_, last_base_z_, last_cos_yaw_, last_sin_yaw_);
    }

    if (std::isfinite(last_latency_ms_)) {
        if (last_latency_ms_ > latency_error_ms_) {
            latency_overrun_count_++;
            latency_recover_count_ = 0;
        } else if (last_latency_ms_ <= latency_warn_ms_) {
            latency_recover_count_++;
            latency_overrun_count_ = 0;
        } else {
            latency_overrun_count_ = 0;
            latency_recover_count_ = 0;
        }
    } else {
        latency_overrun_count_ = 0;
        latency_recover_count_ = 0;
    }

    if (!latency_intervention_active_ &&
        latency_intervention_mode_ != "none" &&
        latency_overrun_count_ >= latency_trigger_frames_) {
        latency_intervention_active_ = true;
        RCLCPP_ERROR(this->get_logger(),
            "latency intervention activated, latency=%.2fms overrun_count=%d",
            last_latency_ms_, latency_overrun_count_);
    }
    if (latency_intervention_active_ && latency_recover_count_ >= latency_recover_frames_) {
        latency_intervention_active_ = false;
        RCLCPP_INFO(this->get_logger(),
            "latency intervention cleared, recover_count=%d", latency_recover_count_);
    }
    if (latency_intervention_active_) {
        if (latency_intervention_mode_ == "virtual_fence" && have_last_pose_) {
            publishVirtualFence(now, last_base_x_, last_base_y_, last_base_z_, last_cos_yaw_, last_sin_yaw_);
        } else if (latency_intervention_mode_ == "emergency_stop") {
            publishEmergencyStop(now);
        }
    }

    int level = fail_safe_active_
                    ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
                    : diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string msg = fail_safe_active_ ? ("降级保护: " + fail_safe_reason_) : "正常";

    if (std::isfinite(last_latency_ms_)) {
        if (last_latency_ms_ > latency_error_ms_) {
            level = std::max(level, static_cast<int>(diagnostic_msgs::msg::DiagnosticStatus::ERROR));
            msg = fail_safe_active_ ? (msg + "; 处理延迟超限") : "处理延迟超限";
        } else if (last_latency_ms_ > latency_warn_ms_) {
            level = std::max(level, static_cast<int>(diagnostic_msgs::msg::DiagnosticStatus::WARN));
            msg = fail_safe_active_ ? (msg + "; 处理延迟告警") : "处理延迟告警";
        }
    }
    if (latency_intervention_active_) {
        level = std::max(level, static_cast<int>(diagnostic_msgs::msg::DiagnosticStatus::ERROR));
        msg = (msg == "正常") ? "延迟干预激活" : (msg + "; 延迟干预激活");
    }
    publishDiagnostics(now, level, msg);
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

    // 发布到两个话题，确保 Nav2 任一观测源配置都能生效
    if (pub_obstacles_) pub_obstacles_->publish(fence_msg);
    if (pub_drop_) pub_drop_->publish(fence_msg);
}

void TerrainSemanticNode::publishDiagnostics(const rclcpp::Time& stamp, int level,
                                             const std::string& message) const {
    if (!pub_diagnostics_) return;

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
    addKV("fail_safe_active", fail_safe_active_ ? "true" : "false");
    addKV("fail_safe_reason", fail_safe_reason_);
    addKV("base_ground_stable", base_ground_stable_ ? "true" : "false");
    addKV("last_linear_speed_mps", std::to_string(last_linear_speed_mps_));
    addKV("last_pitch_deg", std::to_string(last_pitch_rad_ * 180.0 / M_PI));
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
    addKV("latency_intervention_active", latency_intervention_active_ ? "true" : "false");
    addKV("latency_intervention_mode", latency_intervention_mode_);
    addKV("latency_overrun_count", std::to_string(latency_overrun_count_));
    addKV("latency_recover_count", std::to_string(latency_recover_count_));

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

void TerrainSemanticNode::initGrid() {
    half_width_ = static_cast<int>(std::ceil(perception_radius_m_ / grid_resolution_m_));
    width_ = 2 * half_width_ + 1;
    num_cells_ = width_ * width_;

    cell_in_radius_.assign(static_cast<size_t>(num_cells_), 0);
    ground_z_filtered_.assign(static_cast<size_t>(num_cells_), 0.0f);
    top_z_.assign(static_cast<size_t>(num_cells_), 0.0f);
    last_seen_sec_.assign(static_cast<size_t>(num_cells_), -1.0);
    obstacle_score_.assign(static_cast<size_t>(num_cells_), 0);
    drop_score_.assign(static_cast<size_t>(num_cells_), 0);
    freeze_count_.assign(static_cast<size_t>(num_cells_), 0);
    obstacle_state_.assign(static_cast<size_t>(num_cells_), 0);
    drop_state_.assign(static_cast<size_t>(num_cells_), 0);
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

void TerrainSemanticNode::estimateCellHeights(double stamp_sec) {
    int max_freeze_count = 0;
    const double pitch_gate_rad = stair_pitch_gate_deg_ * M_PI / 180.0;
    const bool stair_motion_gate =
        (last_linear_speed_mps_ >= stair_gate_speed_mps_) ||
        (!base_ground_stable_) ||
        (std::abs(last_pitch_rad_) >= pitch_gate_rad);

    for (const int cell : touched_cells_) {
        auto& samples = cell_z_samples_[static_cast<size_t>(cell)];
        if (static_cast<int>(samples.size()) < min_points_per_cell_) continue;

        const float ground_z = quantileInplace(samples, ground_quantile_);
        float top_z = quantileInplace(samples, top_quantile_);
        if (top_z < ground_z) top_z = ground_z;
        if (top_z > ground_z + static_cast<float>(top_z_max_delta_m_)) {
            top_z = ground_z + static_cast<float>(top_z_max_delta_m_);
        }

        const size_t idx = static_cast<size_t>(cell);
        if (last_seen_sec_[idx] < 0.0) {
            ground_z_filtered_[idx] = ground_z;
            freeze_count_[idx] = 0;
        } else {
            const float jump = std::abs(ground_z - ground_z_filtered_[idx]);
            if (jump > static_cast<float>(jump_thresh_m_)) {
                if (stair_motion_gate) {
                    ground_z_filtered_[idx] = static_cast<float>(ground_ema_alpha_slow_) * ground_z +
                                              static_cast<float>(1.0 - ground_ema_alpha_slow_) * ground_z_filtered_[idx];
                    freeze_count_[idx] = 0;
                } else {
                    int& freeze = freeze_count_[idx];
                    freeze = std::max(0, freeze + 1);
                    max_freeze_count = std::max(max_freeze_count, freeze);
                    if (freeze >= freeze_max_frames_) {
                        ground_z_filtered_[idx] = static_cast<float>(ground_ema_alpha_slow_) * ground_z +
                                                  static_cast<float>(1.0 - ground_ema_alpha_slow_) * ground_z_filtered_[idx];
                        freeze = 0;
                    }
                }
            } else {
                ground_z_filtered_[idx] = static_cast<float>(ground_ema_alpha_) * ground_z +
                                          static_cast<float>(1.0 - ground_ema_alpha_) * ground_z_filtered_[idx];
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
}

void TerrainSemanticNode::publishOutputs(const rclcpp::Time& stamp, double base_x,
                                         double base_y, double base_z,
                                         double cos_yaw, double sin_yaw) {
    pcl::PointCloud<pcl::PointXYZI> obs_cloud, drop_cloud, climb_cloud;
    const double stamp_sec = stamp.seconds();
    const bool publish_climbable = static_cast<bool>(pub_climbable_);

    for (int cell = 0; cell < num_cells_; cell++) {
        const size_t idx = static_cast<size_t>(cell);
        if (!cell_in_radius_[idx]) continue;

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

        const bool is_obstacle = obstacle_state_[idx] != 0;
        const bool is_drop = drop_state_[idx] != 0;

        if (is_obstacle) {
            pcl::PointXYZI p;
            p.x = static_cast<float>(x);
            p.y = static_cast<float>(y);
            p.z = z;
            p.intensity = static_cast<float>(obstacle_score_[idx]);
            obs_cloud.push_back(p);
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
            }
        }

        // Unknown 区域策略：保守模式下将 Unknown 视为风险点输出给 Nav2
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

        // 可攀爬台阶特征（调试/定位用）：0 < ΔZ <= h_climb 且不属于致命障碍/跌落
        if (publish_climbable && fresh && !is_obstacle && !is_drop) {
            float dz_up = 0.0f;
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
                    dz_up = std::max(dz_up, diff);
                }
            }
            if (dz_up >= static_cast<float>(climbable_min_dz_m_) &&
                dz_up <= static_cast<float>(h_climb_m_)) {
                pcl::PointXYZI p;
                p.x = static_cast<float>(x);
                p.y = static_cast<float>(y);
                p.z = z;
                p.intensity = dz_up;  // 便于 RViz 可视化
                climb_cloud.push_back(p);
            }
        }
    }

    if (output_sanity_check_enable_) {
        std::string reason;
        bool ok_obs = sanitizeAndValidateCloud(obs_cloud, "obstacles", reason);
        bool ok_drop = sanitizeAndValidateCloud(drop_cloud, "drop", reason);

        int total_points = static_cast<int>(obs_cloud.size() + drop_cloud.size());
        if (publish_climbable) total_points += static_cast<int>(climb_cloud.size());

        if (!ok_obs || !ok_drop || total_points > output_max_points_total_) {
            fail_safe_active_ = true;
            fail_safe_reason_ = "输出点云异常: " + reason;

            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "输出点云异常: obs=%zu drop=%zu total=%d, %s",
                obs_cloud.size(), drop_cloud.size(), total_points, reason.c_str());

            publishEmergencyStop(stamp);
            publishDiagnostics(stamp, diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                "输出点云异常: " + reason);
            return;
        }
    }

    sensor_msgs::msg::PointCloud2 obs_msg, drop_msg, climb_msg;
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
            if (!fail_safe_active_) {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                      "进入降级保护模式: TF(base) 查询失败");
            }
            fail_safe_active_ = true;
            fail_safe_reason_ = "TF(base) 查询失败";
        }
        classifyAndUpdate(stamp_sec);

        if (fail_safe_strategy_ == "emergency_stop") {
            publishEmergencyStop(this->get_clock()->now());
        } else if (fail_safe_strategy_ == "virtual_fence" && have_last_pose_) {
            publishVirtualFence(stamp, last_base_x_, last_base_y_, last_base_z_, last_cos_yaw_, last_sin_yaw_);
        } else {
            publishEmergencyStop(this->get_clock()->now());
        }

        publishDiagnosticsWithLatency(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "TF(base) 查询失败");
        return;
    }

    // 获取机器人位姿（严格按时间戳 TF 同步）
    const auto& origin = tf_base->getOrigin();
    const double base_x = origin.x();
    const double base_y = origin.y();
    const double base_z = origin.z();

    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_base->getRotation()).getRPY(roll, pitch, yaw);
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
            if (!fail_safe_active_) {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                      "进入降级保护模式: TF(点云) 查询失败");
            }
            fail_safe_active_ = true;
            fail_safe_reason_ = "TF(点云) 查询失败";
        }
        classifyAndUpdate(stamp_sec);
        publishOutputs(stamp, base_x, base_y, base_z, cos_yaw, sin_yaw);
        if (fail_safe_active_ && enable_fail_safe_ && fail_safe_strategy_ == "virtual_fence") {
            publishVirtualFence(stamp, base_x, base_y, base_z, cos_yaw, sin_yaw);
        }
        publishDiagnosticsWithLatency(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "TF(点云) 查询失败");
        return;
    }

    last_good_tf_stamp_ = stamp;

    // 将点云严格变换到 target_frame（基于 header 时间戳）
    sensor_msgs::msg::PointCloud2 cloud_transformed;
    pcl_ros::transformPointCloud(target_frame_, *tf_cloud, *msg, cloud_transformed);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(cloud_transformed, *cloud);

    // 体素下采样，降低计算量
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ds(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setLeafSize(static_cast<float>(voxel_leaf_size_m_),
                      static_cast<float>(voxel_leaf_size_m_),
                      static_cast<float>(voxel_leaf_size_m_));
    voxel.setInputCloud(cloud);
    voxel.filter(*cloud_ds);

    const double r2 = perception_radius_m_ * perception_radius_m_;

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

    last_latency_ms_ = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t_start).count();

    publishOutputs(stamp, base_x, base_y, base_z, cos_yaw, sin_yaw);
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
