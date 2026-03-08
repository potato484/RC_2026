#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav2_msgs/msg/costmap_filter_info.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rc26_interfaces/msg/localization_backend_status.hpp>
#include <rc26_interfaces/msg/localization_health.hpp>
#include <rc26_interfaces/msg/mechanism_state.hpp>
#include <rc26_interfaces/msg/nav_safety_state.hpp>
#include <rc26_interfaces/msg/operator_status.hpp>
#include <rc26_interfaces/msg/visualization_event_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

#include "rc26_visualization/visualization_status_core.hpp"

namespace rc26_visualization {

namespace {

template <typename HeaderT>
rclcpp::Time stampOrNow(rclcpp::Node& node, const HeaderT& header) {
  const rclcpp::Time stamp(header.stamp, node.get_clock()->get_clock_type());
  if (stamp.nanoseconds() > 0) {
    return stamp;
  }
  return node.now();
}

double ageSec(rclcpp::Clock& clock, bool received, const rclcpp::Time& stamp) {
  if (!received || stamp.nanoseconds() <= 0) {
    return kInf;
  }
  return std::max(0.0, (clock.now() - stamp).seconds());
}

bool cloudActive(const sensor_msgs::msg::PointCloud2& msg) {
  return msg.width > 0 && msg.height > 0;
}

template <typename MsgT>
struct MessageCache {
  bool received{false};
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  MsgT msg{};
};

template <typename ValueT>
struct ValueCache {
  bool received{false};
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  ValueT value{};
};

struct TopicWatchConfig {
  std::string code_suffix;
  std::string topic_name;
  double max_age_sec{1.0};
  bool required{true};
};

}  // namespace

class VisualizationStatusNode : public rclcpp::Node {
public:
  VisualizationStatusNode()
      : Node("rc26_visualization_status_node") {
    declareParameters();
    loadParameters();
    setupPublishers();
    setupSubscriptions();
    setupTimer();
  }

private:
  void declareParameters() {
    this->declare_parameter<double>("publish_rate_hz", 5.0);
    this->declare_parameter<std::string>("topics.localization_health", "/localization/health");
    this->declare_parameter<std::string>("topics.localization_backend_status", "/localization/backend_status");
    this->declare_parameter<std::string>("topics.control_degraded", "/control_degraded");
    this->declare_parameter<std::string>("topics.control_degenerate_score", "control_degenerate_score");
    this->declare_parameter<std::string>("topics.compute_time_ms", "compute_time_ms");
    this->declare_parameter<std::string>("topics.pose_age_ms", "pose_age_ms");
    this->declare_parameter<std::string>("topics.collision_d_min", "collision_d_min");
    this->declare_parameter<std::string>("topics.controller_mode", "controller_server/NMPCFollowPath/mode");
    this->declare_parameter<std::string>("topics.nav_safety_state", "nav_safety_state");
    this->declare_parameter<std::string>("topics.mechanism_state", "/mechanism/state");
    this->declare_parameter<std::string>("topics.costmap_filter_info", "/costmap_filter_info");
    this->declare_parameter<std::string>("topics.kfs_filter_mask", "/kfs_filter_mask");
    this->declare_parameter<std::string>("topics.kfs_keepout_heartbeat", "/kfs_keepout_heartbeat");
    this->declare_parameter<std::string>("topics.terrain_obstacles", "terrain_obstacles");
    this->declare_parameter<std::string>("topics.terrain_drop", "terrain_drop");
    this->declare_parameter<std::string>("topics.odom", "odom");
    this->declare_parameter<std::string>("topics.control_state", "control_state");

    this->declare_parameter<double>("thresholds.loc_timeout_sec", 0.2);
    this->declare_parameter<double>("thresholds.controller_period_ms", 33.333);
    this->declare_parameter<double>("thresholds.brake_margin_m", 0.15);
    this->declare_parameter<double>("thresholds.keepout_max_age_ms", 300.0);
    this->declare_parameter<double>("thresholds.terrain_max_age_ms", 1000.0);
    this->declare_parameter<double>("thresholds.nav_safety_max_age_ms", 2500.0);
    this->declare_parameter<double>("thresholds.mechanism_max_age_ms", 1000.0);
    this->declare_parameter<double>("thresholds.backend_status_max_age_ms", 1000.0);
    this->declare_parameter<double>("thresholds.backend_graph_health_warn", 0.6);
    this->declare_parameter<double>("thresholds.backend_graph_health_error", 0.3);
    this->declare_parameter<double>("thresholds.backend_local_reg_warn_sec", 0.75);
    this->declare_parameter<double>("thresholds.backend_local_reg_error_sec", 1.5);
    this->declare_parameter<int>("thresholds.mechanism_warn_level", 1);
    this->declare_parameter<int>("thresholds.mechanism_error_level", 2);
    this->declare_parameter<int>("thresholds.topics_orange_count", 3);

    this->declare_parameter<double>("watchdog.localization_health_max_age_ms", 1000.0);
    this->declare_parameter<double>("watchdog.localization_backend_status_max_age_ms", 1500.0);
    this->declare_parameter<double>("watchdog.control_degraded_max_age_ms", 500.0);
    this->declare_parameter<double>("watchdog.control_degenerate_score_max_age_ms", 500.0);
    this->declare_parameter<double>("watchdog.compute_time_ms_max_age_ms", 500.0);
    this->declare_parameter<double>("watchdog.pose_age_ms_max_age_ms", 500.0);
    this->declare_parameter<double>("watchdog.collision_d_min_max_age_ms", 500.0);
    this->declare_parameter<double>("watchdog.nav_safety_state_max_age_ms", 2500.0);
    this->declare_parameter<double>("watchdog.mechanism_state_max_age_ms", 1000.0);
    this->declare_parameter<double>("watchdog.costmap_filter_info_max_age_ms", 300.0);
    this->declare_parameter<double>("watchdog.kfs_filter_mask_max_age_ms", 300.0);
    this->declare_parameter<double>("watchdog.kfs_keepout_heartbeat_max_age_ms", 300.0);
    this->declare_parameter<double>("watchdog.terrain_obstacles_max_age_ms", 1000.0);
    this->declare_parameter<double>("watchdog.terrain_drop_max_age_ms", 1000.0);
    this->declare_parameter<double>("watchdog.odom_max_age_ms", 500.0);
    this->declare_parameter<double>("watchdog.control_state_max_age_ms", 500.0);
  }

  void loadParameters() {
    publish_rate_hz_ = this->get_parameter("publish_rate_hz").as_double();
    topic_localization_health_ = this->get_parameter("topics.localization_health").as_string();
    topic_localization_backend_ = this->get_parameter("topics.localization_backend_status").as_string();
    topic_control_degraded_ = this->get_parameter("topics.control_degraded").as_string();
    topic_control_degenerate_score_ = this->get_parameter("topics.control_degenerate_score").as_string();
    topic_compute_time_ms_ = this->get_parameter("topics.compute_time_ms").as_string();
    topic_pose_age_ms_ = this->get_parameter("topics.pose_age_ms").as_string();
    topic_collision_d_min_ = this->get_parameter("topics.collision_d_min").as_string();
    topic_controller_mode_ = this->get_parameter("topics.controller_mode").as_string();
    topic_nav_safety_ = this->get_parameter("topics.nav_safety_state").as_string();
    topic_mechanism_state_ = this->get_parameter("topics.mechanism_state").as_string();
    topic_costmap_filter_info_ = this->get_parameter("topics.costmap_filter_info").as_string();
    topic_kfs_filter_mask_ = this->get_parameter("topics.kfs_filter_mask").as_string();
    topic_kfs_heartbeat_ = this->get_parameter("topics.kfs_keepout_heartbeat").as_string();
    topic_terrain_obstacles_ = this->get_parameter("topics.terrain_obstacles").as_string();
    topic_terrain_drop_ = this->get_parameter("topics.terrain_drop").as_string();
    topic_odom_ = this->get_parameter("topics.odom").as_string();
    topic_control_state_ = this->get_parameter("topics.control_state").as_string();

    VisualizationStatusConfig config;
    config.loc_timeout_sec = this->get_parameter("thresholds.loc_timeout_sec").as_double();
    config.controller_period_ms = this->get_parameter("thresholds.controller_period_ms").as_double();
    config.brake_margin_m = this->get_parameter("thresholds.brake_margin_m").as_double();
    config.keepout_max_age_ms = this->get_parameter("thresholds.keepout_max_age_ms").as_double();
    config.terrain_max_age_ms = this->get_parameter("thresholds.terrain_max_age_ms").as_double();
    config.nav_safety_max_age_ms = this->get_parameter("thresholds.nav_safety_max_age_ms").as_double();
    config.mechanism_max_age_ms = this->get_parameter("thresholds.mechanism_max_age_ms").as_double();
    config.backend_status_max_age_ms = this->get_parameter("thresholds.backend_status_max_age_ms").as_double();
    config.backend_graph_health_warn = this->get_parameter("thresholds.backend_graph_health_warn").as_double();
    config.backend_graph_health_error = this->get_parameter("thresholds.backend_graph_health_error").as_double();
    config.backend_local_reg_warn_sec = this->get_parameter("thresholds.backend_local_reg_warn_sec").as_double();
    config.backend_local_reg_error_sec = this->get_parameter("thresholds.backend_local_reg_error_sec").as_double();
    config.mechanism_warn_level = static_cast<uint8_t>(this->get_parameter("thresholds.mechanism_warn_level").as_int());
    config.mechanism_error_level = static_cast<uint8_t>(this->get_parameter("thresholds.mechanism_error_level").as_int());
    config.topics_orange_count = static_cast<uint32_t>(this->get_parameter("thresholds.topics_orange_count").as_int());
    config.localization_health_topic = topic_localization_health_;
    config.localization_backend_topic = topic_localization_backend_;
    config.control_degraded_topic = topic_control_degraded_;
    config.control_degenerate_score_topic = topic_control_degenerate_score_;
    config.compute_time_ms_topic = topic_compute_time_ms_;
    config.pose_age_ms_topic = topic_pose_age_ms_;
    config.collision_d_min_topic = topic_collision_d_min_;
    config.nav_safety_topic = topic_nav_safety_;
    config.mechanism_state_topic = topic_mechanism_state_;
    config.costmap_filter_info_topic = topic_costmap_filter_info_;
    config.kfs_filter_mask_topic = topic_kfs_filter_mask_;
    config.kfs_heartbeat_topic = topic_kfs_heartbeat_;
    config.terrain_obstacles_topic = topic_terrain_obstacles_;
    config.terrain_drop_topic = topic_terrain_drop_;
    core_.setConfig(config);

    topic_watch_configs_ = {
        {"LOCALIZATION_HEALTH", topic_localization_health_, this->get_parameter("watchdog.localization_health_max_age_ms").as_double() / 1000.0, true},
        {"LOCALIZATION_BACKEND_STATUS", topic_localization_backend_, this->get_parameter("watchdog.localization_backend_status_max_age_ms").as_double() / 1000.0, true},
        {"CONTROL_DEGRADED", topic_control_degraded_, this->get_parameter("watchdog.control_degraded_max_age_ms").as_double() / 1000.0, true},
        {"CONTROL_DEGENERATE_SCORE", topic_control_degenerate_score_, this->get_parameter("watchdog.control_degenerate_score_max_age_ms").as_double() / 1000.0, true},
        {"COMPUTE_TIME_MS", topic_compute_time_ms_, this->get_parameter("watchdog.compute_time_ms_max_age_ms").as_double() / 1000.0, true},
        {"POSE_AGE_MS", topic_pose_age_ms_, this->get_parameter("watchdog.pose_age_ms_max_age_ms").as_double() / 1000.0, true},
        {"COLLISION_D_MIN", topic_collision_d_min_, this->get_parameter("watchdog.collision_d_min_max_age_ms").as_double() / 1000.0, true},
        {"NAV_SAFETY_STATE", topic_nav_safety_, this->get_parameter("watchdog.nav_safety_state_max_age_ms").as_double() / 1000.0, true},
        {"MECHANISM_STATE", topic_mechanism_state_, this->get_parameter("watchdog.mechanism_state_max_age_ms").as_double() / 1000.0, true},
        {"COSTMAP_FILTER_INFO", topic_costmap_filter_info_, this->get_parameter("watchdog.costmap_filter_info_max_age_ms").as_double() / 1000.0, true},
        {"KFS_FILTER_MASK", topic_kfs_filter_mask_, this->get_parameter("watchdog.kfs_filter_mask_max_age_ms").as_double() / 1000.0, true},
        {"KFS_KEEPOUT_HEARTBEAT", topic_kfs_heartbeat_, this->get_parameter("watchdog.kfs_keepout_heartbeat_max_age_ms").as_double() / 1000.0, true},
        {"TERRAIN_OBSTACLES", topic_terrain_obstacles_, this->get_parameter("watchdog.terrain_obstacles_max_age_ms").as_double() / 1000.0, true},
        {"TERRAIN_DROP", topic_terrain_drop_, this->get_parameter("watchdog.terrain_drop_max_age_ms").as_double() / 1000.0, true},
        {"ODOM", topic_odom_, this->get_parameter("watchdog.odom_max_age_ms").as_double() / 1000.0, true},
        {"CONTROL_STATE", topic_control_state_, this->get_parameter("watchdog.control_state_max_age_ms").as_double() / 1000.0, true},
    };
  }

  void setupPublishers() {
    summary_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("r2/diag/summary", 10);
    operator_status_pub_ = this->create_publisher<rc26_interfaces::msg::OperatorStatus>("r2/diag/operator_status", 10);
    events_pub_ = this->create_publisher<rc26_interfaces::msg::VisualizationEventArray>("r2/diag/events", 10);
  }

  void setupSubscriptions() {
    const auto sensor_qos = rclcpp::SensorDataQoS();
    const auto keepout_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability(rclcpp::DurabilityPolicy::TransientLocal);
    const auto reliable_qos = rclcpp::QoS(10).reliable();

    localization_health_sub_ = this->create_subscription<rc26_interfaces::msg::LocalizationHealth>(
        topic_localization_health_, sensor_qos,
        [this](const rc26_interfaces::msg::LocalizationHealth::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(data_mutex_);
          localization_health_.received = true;
          localization_health_.stamp = stampOrNow(*this, msg->header);
          localization_health_.msg = *msg;
        });

    localization_backend_sub_ = this->create_subscription<rc26_interfaces::msg::LocalizationBackendStatus>(
        topic_localization_backend_, sensor_qos,
        [this](const rc26_interfaces::msg::LocalizationBackendStatus::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(data_mutex_);
          localization_backend_.received = true;
          localization_backend_.stamp = stampOrNow(*this, msg->header);
          localization_backend_.msg = *msg;
        });

    control_degraded_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        topic_control_degraded_, sensor_qos,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(data_mutex_);
          control_degraded_.received = true;
          control_degraded_.stamp = this->now();
          control_degraded_.value = msg->data;
        });

    control_degenerate_score_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        topic_control_degenerate_score_, sensor_qos,
        [this](const std_msgs::msg::Float64::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(data_mutex_);
          control_degenerate_score_.received = true;
          control_degenerate_score_.stamp = this->now();
          control_degenerate_score_.value = msg->data;
        });

    compute_time_ms_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        topic_compute_time_ms_, reliable_qos,
        [this](const std_msgs::msg::Float64::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(data_mutex_);
          compute_time_ms_.received = true;
          compute_time_ms_.stamp = this->now();
          compute_time_ms_.value = msg->data;
        });

    pose_age_ms_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        topic_pose_age_ms_, reliable_qos,
        [this](const std_msgs::msg::Float64::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(data_mutex_);
          pose_age_ms_.received = true;
          pose_age_ms_.stamp = this->now();
          pose_age_ms_.value = msg->data;
        });

    collision_d_min_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        topic_collision_d_min_, reliable_qos,
        [this](const std_msgs::msg::Float64::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(data_mutex_);
          collision_d_min_.received = true;
          collision_d_min_.stamp = this->now();
          collision_d_min_.value = msg->data;
        });

    controller_mode_sub_ = this->create_subscription<std_msgs::msg::String>(
        topic_controller_mode_, reliable_qos,
        [this](const std_msgs::msg::String::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(data_mutex_);
          controller_mode_.received = true;
          controller_mode_.stamp = this->now();
          controller_mode_.value = msg->data;
        });

    nav_safety_sub_ = this->create_subscription<rc26_interfaces::msg::NavSafetyState>(
        topic_nav_safety_, reliable_qos,
        [this](const rc26_interfaces::msg::NavSafetyState::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(data_mutex_);
          nav_safety_.received = true;
          nav_safety_.stamp = stampOrNow(*this, msg->header);
          nav_safety_.msg = *msg;
        });

    mechanism_state_sub_ = this->create_subscription<rc26_interfaces::msg::MechanismState>(
        topic_mechanism_state_, reliable_qos,
        [this](const rc26_interfaces::msg::MechanismState::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(data_mutex_);
          mechanism_state_.received = true;
          mechanism_state_.stamp = stampOrNow(*this, msg->header);
          mechanism_state_.msg = *msg;
        });

    costmap_filter_info_sub_ = this->create_subscription<nav2_msgs::msg::CostmapFilterInfo>(
        topic_costmap_filter_info_, keepout_qos,
        [this](const nav2_msgs::msg::CostmapFilterInfo::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(data_mutex_);
          costmap_filter_info_.received = true;
          costmap_filter_info_.stamp = stampOrNow(*this, msg->header);
          costmap_filter_info_.msg = *msg;
        });

    kfs_filter_mask_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        topic_kfs_filter_mask_, keepout_qos,
        [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(data_mutex_);
          kfs_filter_mask_.received = true;
          kfs_filter_mask_.stamp = stampOrNow(*this, msg->header);
          kfs_filter_mask_.msg = *msg;
        });

    kfs_heartbeat_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        topic_kfs_heartbeat_, reliable_qos,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(data_mutex_);
          kfs_heartbeat_.received = true;
          kfs_heartbeat_.stamp = this->now();
          kfs_heartbeat_.value = msg->data;
        });

    terrain_obstacles_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        topic_terrain_obstacles_, sensor_qos,
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(data_mutex_);
          terrain_obstacles_.received = true;
          terrain_obstacles_.stamp = stampOrNow(*this, msg->header);
          terrain_obstacles_.msg = *msg;
        });

    terrain_drop_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        topic_terrain_drop_, sensor_qos,
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(data_mutex_);
          terrain_drop_.received = true;
          terrain_drop_.stamp = stampOrNow(*this, msg->header);
          terrain_drop_.msg = *msg;
        });

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        topic_odom_, sensor_qos,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(data_mutex_);
          odom_.received = true;
          odom_.stamp = stampOrNow(*this, msg->header);
          odom_.msg = *msg;
        });

    control_state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        topic_control_state_, sensor_qos,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(data_mutex_);
          control_state_.received = true;
          control_state_.stamp = stampOrNow(*this, msg->header);
          control_state_.msg = *msg;
        });
  }

  void setupTimer() {
    const auto period_ms = std::max<int64_t>(100, static_cast<int64_t>(1000.0 / std::max(1.0, publish_rate_hz_)));
    publish_timer_ = this->create_wall_timer(std::chrono::milliseconds(period_ms), [this]() { publishStatus(); });
  }

  void publishStatus() {
    EvaluationInput input;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      input.localization.received = localization_health_.received;
      input.localization.age_sec = ageSec(*this->get_clock(), localization_health_.received, localization_health_.stamp);
      if (localization_health_.received) {
        input.localization.level = localization_health_.msg.level;
        input.localization.reason = localization_health_.msg.reason;
        input.localization.state = localization_health_.msg.localization_state;
        input.localization.control_degraded = localization_health_.msg.control_degraded;
        input.localization.degenerate_score = localization_health_.msg.degenerate_score;
      }

      input.backend.received = localization_backend_.received;
      input.backend.age_sec = ageSec(*this->get_clock(), localization_backend_.received, localization_backend_.stamp);
      if (localization_backend_.received) {
        input.backend.optimizer_ready = localization_backend_.msg.optimizer_ready;
        input.backend.optimizer_state = localization_backend_.msg.optimizer_state;
        input.backend.graph_health = localization_backend_.msg.graph_health;
        input.backend.last_local_reg_age_sec = localization_backend_.msg.last_local_reg_age_sec;
        input.backend.last_loop_age_sec = localization_backend_.msg.last_loop_age_sec;
        input.backend.last_anchor_age_sec = localization_backend_.msg.last_anchor_age_sec;
        input.backend.imu_spike = localization_backend_.msg.imu_spike;
      }

      input.control_degraded.received = control_degraded_.received;
      input.control_degraded.age_sec = ageSec(*this->get_clock(), control_degraded_.received, control_degraded_.stamp);
      input.control_degraded.value = control_degraded_.value;

      input.control_degenerate_score.received = control_degenerate_score_.received;
      input.control_degenerate_score.age_sec = ageSec(*this->get_clock(), control_degenerate_score_.received, control_degenerate_score_.stamp);
      input.control_degenerate_score.value = control_degenerate_score_.value;

      input.compute_time_ms.received = compute_time_ms_.received;
      input.compute_time_ms.age_sec = ageSec(*this->get_clock(), compute_time_ms_.received, compute_time_ms_.stamp);
      input.compute_time_ms.value = compute_time_ms_.value;

      input.pose_age_ms.received = pose_age_ms_.received;
      input.pose_age_ms.age_sec = ageSec(*this->get_clock(), pose_age_ms_.received, pose_age_ms_.stamp);
      input.pose_age_ms.value = pose_age_ms_.value;

      input.collision_d_min.received = collision_d_min_.received;
      input.collision_d_min.age_sec = ageSec(*this->get_clock(), collision_d_min_.received, collision_d_min_.stamp);
      input.collision_d_min.value = collision_d_min_.value;

      input.controller_mode.received = controller_mode_.received;
      input.controller_mode.age_sec = ageSec(*this->get_clock(), controller_mode_.received, controller_mode_.stamp);
      input.controller_mode.value = controller_mode_.value;

      input.nav_safety.received = nav_safety_.received;
      input.nav_safety.age_sec = ageSec(*this->get_clock(), nav_safety_.received, nav_safety_.stamp);
      if (nav_safety_.received) {
        input.nav_safety.current_profile = nav_safety_.msg.current_profile;
        input.nav_safety.reason = nav_safety_.msg.reason;
        input.nav_safety.stop_required = nav_safety_.msg.stop_required;
        input.nav_safety.timed_out = nav_safety_.msg.timed_out;
      }

      input.mechanism.received = mechanism_state_.received;
      input.mechanism.age_sec = ageSec(*this->get_clock(), mechanism_state_.received, mechanism_state_.stamp);
      if (mechanism_state_.received) {
        input.mechanism.comm_health_level = mechanism_state_.msg.comm_health_level;
      }

      input.keepout.filter_info_received = costmap_filter_info_.received;
      input.keepout.filter_info_age_sec = ageSec(*this->get_clock(), costmap_filter_info_.received, costmap_filter_info_.stamp);
      input.keepout.mask_received = kfs_filter_mask_.received;
      input.keepout.mask_age_sec = ageSec(*this->get_clock(), kfs_filter_mask_.received, kfs_filter_mask_.stamp);
      input.keepout.heartbeat_received = kfs_heartbeat_.received;
      input.keepout.heartbeat_age_sec = ageSec(*this->get_clock(), kfs_heartbeat_.received, kfs_heartbeat_.stamp);
      input.keepout.heartbeat_enabled = kfs_heartbeat_.value;

      input.terrain.obstacles_received = terrain_obstacles_.received;
      input.terrain.obstacles_age_sec = ageSec(*this->get_clock(), terrain_obstacles_.received, terrain_obstacles_.stamp);
      input.terrain.obstacles_active = terrain_obstacles_.received && cloudActive(terrain_obstacles_.msg);
      input.terrain.drop_received = terrain_drop_.received;
      input.terrain.drop_age_sec = ageSec(*this->get_clock(), terrain_drop_.received, terrain_drop_.stamp);
      input.terrain.drop_active = terrain_drop_.received && cloudActive(terrain_drop_.msg);

      for (const auto& topic_watch : topic_watch_configs_) {
        TopicWatchInput watch;
        watch.code_suffix = topic_watch.code_suffix;
        watch.topic_name = topic_watch.topic_name;
        watch.max_age_sec = topic_watch.max_age_sec;
        watch.required = topic_watch.required;
        if (topic_watch.code_suffix == "LOCALIZATION_HEALTH") {
          watch.received = localization_health_.received;
          watch.age_sec = input.localization.age_sec;
        } else if (topic_watch.code_suffix == "LOCALIZATION_BACKEND_STATUS") {
          watch.received = localization_backend_.received;
          watch.age_sec = input.backend.age_sec;
        } else if (topic_watch.code_suffix == "CONTROL_DEGRADED") {
          watch.received = control_degraded_.received;
          watch.age_sec = input.control_degraded.age_sec;
        } else if (topic_watch.code_suffix == "CONTROL_DEGENERATE_SCORE") {
          watch.received = control_degenerate_score_.received;
          watch.age_sec = input.control_degenerate_score.age_sec;
        } else if (topic_watch.code_suffix == "COMPUTE_TIME_MS") {
          watch.received = compute_time_ms_.received;
          watch.age_sec = input.compute_time_ms.age_sec;
        } else if (topic_watch.code_suffix == "POSE_AGE_MS") {
          watch.received = pose_age_ms_.received;
          watch.age_sec = input.pose_age_ms.age_sec;
        } else if (topic_watch.code_suffix == "COLLISION_D_MIN") {
          watch.received = collision_d_min_.received;
          watch.age_sec = input.collision_d_min.age_sec;
        } else if (topic_watch.code_suffix == "NAV_SAFETY_STATE") {
          watch.received = nav_safety_.received;
          watch.age_sec = input.nav_safety.age_sec;
        } else if (topic_watch.code_suffix == "MECHANISM_STATE") {
          watch.received = mechanism_state_.received;
          watch.age_sec = input.mechanism.age_sec;
        } else if (topic_watch.code_suffix == "COSTMAP_FILTER_INFO") {
          watch.received = costmap_filter_info_.received;
          watch.age_sec = input.keepout.filter_info_age_sec;
        } else if (topic_watch.code_suffix == "KFS_FILTER_MASK") {
          watch.received = kfs_filter_mask_.received;
          watch.age_sec = input.keepout.mask_age_sec;
        } else if (topic_watch.code_suffix == "KFS_KEEPOUT_HEARTBEAT") {
          watch.received = kfs_heartbeat_.received;
          watch.age_sec = input.keepout.heartbeat_age_sec;
        } else if (topic_watch.code_suffix == "TERRAIN_OBSTACLES") {
          watch.received = terrain_obstacles_.received;
          watch.age_sec = input.terrain.obstacles_age_sec;
        } else if (topic_watch.code_suffix == "TERRAIN_DROP") {
          watch.received = terrain_drop_.received;
          watch.age_sec = input.terrain.drop_age_sec;
        } else if (topic_watch.code_suffix == "ODOM") {
          watch.received = odom_.received;
          watch.age_sec = ageSec(*this->get_clock(), odom_.received, odom_.stamp);
        } else if (topic_watch.code_suffix == "CONTROL_STATE") {
          watch.received = control_state_.received;
          watch.age_sec = ageSec(*this->get_clock(), control_state_.received, control_state_.stamp);
        }
        input.monitored_topics.push_back(std::move(watch));
      }
    }

    std_msgs::msg::Header header;
    header.stamp = this->now();
    header.frame_id = "map";
    auto output = core_.evaluate(input, header);
    summary_pub_->publish(output.summary);
    operator_status_pub_->publish(output.operator_status);
    events_pub_->publish(output.events);
  }

  double publish_rate_hz_{5.0};
  std::string topic_localization_health_;
  std::string topic_localization_backend_;
  std::string topic_control_degraded_;
  std::string topic_control_degenerate_score_;
  std::string topic_compute_time_ms_;
  std::string topic_pose_age_ms_;
  std::string topic_collision_d_min_;
  std::string topic_controller_mode_;
  std::string topic_nav_safety_;
  std::string topic_mechanism_state_;
  std::string topic_costmap_filter_info_;
  std::string topic_kfs_filter_mask_;
  std::string topic_kfs_heartbeat_;
  std::string topic_terrain_obstacles_;
  std::string topic_terrain_drop_;
  std::string topic_odom_;
  std::string topic_control_state_;

  std::mutex data_mutex_;
  VisualizationStatusCore core_;
  std::vector<TopicWatchConfig> topic_watch_configs_;

  MessageCache<rc26_interfaces::msg::LocalizationHealth> localization_health_;
  MessageCache<rc26_interfaces::msg::LocalizationBackendStatus> localization_backend_;
  ValueCache<bool> control_degraded_;
  ValueCache<double> control_degenerate_score_;
  ValueCache<double> compute_time_ms_;
  ValueCache<double> pose_age_ms_;
  ValueCache<double> collision_d_min_;
  ValueCache<std::string> controller_mode_;
  MessageCache<rc26_interfaces::msg::NavSafetyState> nav_safety_;
  MessageCache<rc26_interfaces::msg::MechanismState> mechanism_state_;
  MessageCache<nav2_msgs::msg::CostmapFilterInfo> costmap_filter_info_;
  MessageCache<nav_msgs::msg::OccupancyGrid> kfs_filter_mask_;
  ValueCache<bool> kfs_heartbeat_;
  MessageCache<sensor_msgs::msg::PointCloud2> terrain_obstacles_;
  MessageCache<sensor_msgs::msg::PointCloud2> terrain_drop_;
  MessageCache<nav_msgs::msg::Odometry> odom_;
  MessageCache<nav_msgs::msg::Odometry> control_state_;

  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr summary_pub_;
  rclcpp::Publisher<rc26_interfaces::msg::OperatorStatus>::SharedPtr operator_status_pub_;
  rclcpp::Publisher<rc26_interfaces::msg::VisualizationEventArray>::SharedPtr events_pub_;

  rclcpp::Subscription<rc26_interfaces::msg::LocalizationHealth>::SharedPtr localization_health_sub_;
  rclcpp::Subscription<rc26_interfaces::msg::LocalizationBackendStatus>::SharedPtr localization_backend_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr control_degraded_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr control_degenerate_score_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr compute_time_ms_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr pose_age_ms_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr collision_d_min_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr controller_mode_sub_;
  rclcpp::Subscription<rc26_interfaces::msg::NavSafetyState>::SharedPtr nav_safety_sub_;
  rclcpp::Subscription<rc26_interfaces::msg::MechanismState>::SharedPtr mechanism_state_sub_;
  rclcpp::Subscription<nav2_msgs::msg::CostmapFilterInfo>::SharedPtr costmap_filter_info_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr kfs_filter_mask_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr kfs_heartbeat_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr terrain_obstacles_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr terrain_drop_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr control_state_sub_;

  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace rc26_visualization

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rc26_visualization::VisualizationStatusNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
