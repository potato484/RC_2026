#include "rc26_omni_controller/xhu_motion_follower.hpp"

#include <tf2/exceptions.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <utility>

namespace rc26_omni_controller {

namespace {

double normalizeAngle(double angle) {
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double clamp(double value, double lower, double upper) {
  return std::min(std::max(value, lower), upper);
}

double yawFromQuaternion(const geometry_msgs::msg::Quaternion &q) {
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

std::string toLowerCopy(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return value;
}

}  // namespace

XhuMotionFollower::XhuMotionFollower(const rclcpp::NodeOptions &options)
    : Node("xhu_motion_follower", options),
      tf_buffer_(std::make_shared<tf2_ros::Buffer>(get_clock())),
      tf_listener_(*tf_buffer_) {
  declare_parameter<std::string>("odom_topic", "control_state");
  declare_parameter<std::string>("map_frame", "map");
  declare_parameter<std::string>("base_frame", "base_link");
  declare_parameter<std::string>("chassis_model", "tracked_diff");
  declare_parameter<double>("control_frequency_hz", 30.0);
  declare_parameter<double>("lookahead_distance", 0.6);
  declare_parameter<double>("goal_tolerance_xy", 0.15);
  declare_parameter<double>("goal_tolerance_yaw", 0.3);
  declare_parameter<double>("corridor_timeout_sec", 45.0);
  declare_parameter<double>("odom_timeout_sec", 0.3);
  declare_parameter<double>("mode_state_timeout_sec", 1.0);
  declare_parameter<double>("hold_to_abort_sec", 3.0);
  declare_parameter<double>("kp_linear_x", 1.2);
  declare_parameter<double>("kp_linear_y", 1.0);
  declare_parameter<double>("kp_angular", 1.8);
  declare_parameter<double>("curvature_gain", 1.0);
  declare_parameter<double>("heading_slowdown_start_rad", 0.35);
  declare_parameter<double>("heading_stop_rad", 0.8);
  declare_parameter<double>("default_max_linear_speed", 0.8);
  declare_parameter<double>("default_max_angular_speed", 1.0);
  declare_parameter<double>("default_max_linear_accel", 0.6);
  declare_parameter<double>("default_max_angular_accel", 0.8);
  declare_parameter<double>("lhi_yellow_v_scale", 0.8);
  declare_parameter<double>("lhi_yellow_w_scale", 0.8);
  declare_parameter<double>("lhi_orange_v_scale", 0.5);
  declare_parameter<double>("lhi_orange_w_scale", 0.6);
  declare_parameter<double>("lhi_orange_vy_scale", 0.5);
  declare_parameter<double>("terrain_obstacle_threshold", 0.6);
  declare_parameter<double>("terrain_drop_threshold", 0.8);
  declare_parameter<double>("terrain_sample_spacing_m", 0.12);
  declare_parameter<double>("stop_envelope_half_width_m", 0.20);
  declare_parameter<double>("brake_margin_m", 0.30);
  declare_parameter<double>("max_cross_track_error_m", 0.60);

  const auto odom_topic = get_parameter("odom_topic").as_string();
  map_frame_ = get_parameter("map_frame").as_string();
  base_frame_ = get_parameter("base_frame").as_string();
  chassis_model_ = toLowerCopy(get_parameter("chassis_model").as_string());
  if (chassis_model_ != "tracked_diff" && chassis_model_ != "mecanum_4wheel") {
    RCLCPP_WARN(get_logger(), "chassis_model=%s invalid, fallback to tracked_diff",
                chassis_model_.c_str());
    chassis_model_ = "tracked_diff";
  }
  tracked_diff_mode_ = chassis_model_ == "tracked_diff";
  control_frequency_hz_ = std::max(5.0, get_parameter("control_frequency_hz").as_double());
  lookahead_distance_ = std::max(0.05, get_parameter("lookahead_distance").as_double());
  goal_tolerance_xy_ = std::max(0.02, get_parameter("goal_tolerance_xy").as_double());
  goal_tolerance_yaw_ = std::max(0.05, get_parameter("goal_tolerance_yaw").as_double());
  corridor_timeout_sec_ = std::max(1.0, get_parameter("corridor_timeout_sec").as_double());
  odom_timeout_sec_ = std::max(0.05, get_parameter("odom_timeout_sec").as_double());
  mode_state_timeout_sec_ = std::max(0.1, get_parameter("mode_state_timeout_sec").as_double());
  hold_to_abort_sec_ = std::max(0.2, get_parameter("hold_to_abort_sec").as_double());
  kp_linear_x_ = std::max(0.0, get_parameter("kp_linear_x").as_double());
  kp_linear_y_ = std::max(0.0, get_parameter("kp_linear_y").as_double());
  kp_angular_ = std::max(0.0, get_parameter("kp_angular").as_double());
  curvature_gain_ = std::max(0.0, get_parameter("curvature_gain").as_double());
  heading_slowdown_start_rad_ =
      std::max(0.0, get_parameter("heading_slowdown_start_rad").as_double());
  heading_stop_rad_ = std::max(heading_slowdown_start_rad_ + 1e-3,
                               get_parameter("heading_stop_rad").as_double());
  default_max_linear_speed_ =
      std::max(0.05, get_parameter("default_max_linear_speed").as_double());
  default_max_angular_speed_ =
      std::max(0.05, get_parameter("default_max_angular_speed").as_double());
  default_max_linear_accel_ =
      std::max(0.05, get_parameter("default_max_linear_accel").as_double());
  default_max_angular_accel_ =
      std::max(0.05, get_parameter("default_max_angular_accel").as_double());
  lhi_yellow_v_scale_ = clamp(get_parameter("lhi_yellow_v_scale").as_double(), 0.0, 1.0);
  lhi_yellow_w_scale_ = clamp(get_parameter("lhi_yellow_w_scale").as_double(), 0.0, 1.0);
  lhi_orange_v_scale_ = clamp(get_parameter("lhi_orange_v_scale").as_double(), 0.0, 1.0);
  lhi_orange_w_scale_ = clamp(get_parameter("lhi_orange_w_scale").as_double(), 0.0, 1.0);
  lhi_orange_vy_scale_ = clamp(get_parameter("lhi_orange_vy_scale").as_double(), 0.0, 1.0);
  terrain_obstacle_threshold_ =
      clamp(get_parameter("terrain_obstacle_threshold").as_double(), 0.0, 1.0);
  terrain_drop_threshold_ = clamp(get_parameter("terrain_drop_threshold").as_double(), 0.0, 1.0);
  terrain_sample_spacing_m_ =
      std::max(0.02, get_parameter("terrain_sample_spacing_m").as_double());
  stop_envelope_half_width_m_ =
      std::max(0.0, get_parameter("stop_envelope_half_width_m").as_double());
  brake_margin_m_ = std::max(0.0, get_parameter("brake_margin_m").as_double());
  max_cross_track_error_m_ =
      std::max(goal_tolerance_xy_, get_parameter("max_cross_track_error_m").as_double());

  corridor_sub_ = create_subscription<rc26_interfaces::msg::XhuSemanticCorridor>(
      "/xhu_nav/corridor_cmd", 10,
      std::bind(&XhuMotionFollower::onCorridor, this, std::placeholders::_1));
  mode_sub_ = create_subscription<rc26_interfaces::msg::XhuMotionModeState>(
      "/xhu_nav/motion_mode_state", 10,
      std::bind(&XhuMotionFollower::onModeState, this, std::placeholders::_1));
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::SensorDataQoS(),
      std::bind(&XhuMotionFollower::onOdom, this, std::placeholders::_1));
  loc_health_sub_ = create_subscription<rc26_interfaces::msg::LocalizationHealth>(
      "/localization/health", 10,
      std::bind(&XhuMotionFollower::onLocalizationHealth, this, std::placeholders::_1));
  terrain_sub_ = create_subscription<rc26_interfaces::msg::TerrainFeatureGrid>(
      "terrain_features", 10,
      std::bind(&XhuMotionFollower::onTerrainGrid, this, std::placeholders::_1));

  cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  lookahead_pub_ = create_publisher<nav_msgs::msg::Path>("/xhu_nav/lookahead_path", 10);
  tracking_pub_ =
      create_publisher<rc26_interfaces::msg::XhuTrackingState>("/xhu_nav/tracking_state", 10);
  semantic_gate_pub_ = create_publisher<std_msgs::msg::String>("/xhu_nav/semantic_gate", 10);

  last_control_stamp_ = now();
  control_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::duration<double>(1.0 / control_frequency_hz_)),
      std::bind(&XhuMotionFollower::controlLoop, this));

  RCLCPP_INFO(get_logger(), "xhu_motion_follower ready, chassis_model=%s", chassis_model_.c_str());
}

void XhuMotionFollower::onCorridor(
    const rc26_interfaces::msg::XhuSemanticCorridor::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  if (msg->path.poses.empty()) {
    publishTerminalStateLocked(*msg, "ABORT", "empty corridor", 0.0F, 0.0F, 0.0F);
    return;
  }

  active_corridor_ = msg;
  nearest_index_ = 0U;
  corridor_start_stamp_ = now();
  hold_since_.reset();
  last_cmd_ = geometry_msgs::msg::Twist{};
  publishRuntimeStateLocked(*active_corridor_, "PASS", false, "corridor accepted", 0.0F, 0.0F,
                            0.0F);
}

void XhuMotionFollower::onModeState(
    const rc26_interfaces::msg::XhuMotionModeState::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  mode_state_ = *msg;
  mode_state_stamp_ = (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0)
                          ? now()
                          : rclcpp::Time(msg->header.stamp);
  has_mode_state_ = true;
}

void XhuMotionFollower::onOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  last_odom_ = *msg;
  has_odom_ = true;
  if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) {
    last_odom_stamp_ = now();
  } else {
    last_odom_stamp_ = rclcpp::Time(msg->header.stamp);
  }
}

void XhuMotionFollower::onLocalizationHealth(
    const rc26_interfaces::msg::LocalizationHealth::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  loc_health_level_ = msg->level;
}

void XhuMotionFollower::onTerrainGrid(
    const rc26_interfaces::msg::TerrainFeatureGrid::SharedPtr msg) {
  auto cache = std::make_shared<TerrainCache>();
  cache->valid = msg->resolution_m > 0.0F && msg->width > 0 && msg->height > 0;
  cache->resolution_m = msg->resolution_m;
  cache->width = msg->width;
  cache->height = msg->height;
  cache->origin_x = msg->origin.position.x;
  cache->origin_y = msg->origin.position.y;
  cache->p_obstacle = msg->p_obstacle;
  cache->p_drop = msg->p_drop;

  std::lock_guard<std::mutex> lock(data_mutex_);
  terrain_cache_ = std::move(cache);
}

bool XhuMotionFollower::queryRobotPose(double &x, double &y, double &yaw) const {
  try {
    const auto tf = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
    x = tf.transform.translation.x;
    y = tf.transform.translation.y;
    yaw = yawFromQuaternion(tf.transform.rotation);
    return true;
  } catch (const tf2::TransformException &) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!has_odom_) {
      return false;
    }
    if ((now() - last_odom_stamp_).seconds() > odom_timeout_sec_) {
      return false;
    }
    x = last_odom_.pose.pose.position.x;
    y = last_odom_.pose.pose.position.y;
    yaw = yawFromQuaternion(last_odom_.pose.pose.orientation);
    return true;
  }
}

bool XhuMotionFollower::terrainRiskAt(const std::shared_ptr<const TerrainCache> &cache, double x,
                                      double y) const {
  if (!cache || !cache->valid) {
    return false;
  }

  const int gx = static_cast<int>(std::floor((x - cache->origin_x) / cache->resolution_m));
  const int gy = static_cast<int>(std::floor((y - cache->origin_y) / cache->resolution_m));
  if (gx < 0 || gy < 0 || gx >= static_cast<int>(cache->width) ||
      gy >= static_cast<int>(cache->height)) {
    return false;
  }

  const size_t flat_index =
      static_cast<size_t>(gy) * cache->width + static_cast<size_t>(gx);
  if (flat_index >= cache->p_obstacle.size() || flat_index >= cache->p_drop.size()) {
    return false;
  }

  return cache->p_obstacle[flat_index] >= terrain_obstacle_threshold_ ||
         cache->p_drop[flat_index] >= terrain_drop_threshold_;
}

bool XhuMotionFollower::terrainRiskAhead(const std::shared_ptr<const TerrainCache> &cache,
                                         const nav_msgs::msg::Path &path, size_t start_index,
                                         size_t end_index) const {
  if (!cache || !cache->valid || path.poses.empty()) {
    return false;
  }

  const size_t last_index = path.poses.size() - 1;
  const size_t begin = std::min(start_index, last_index);
  const size_t end = std::min(end_index, last_index);
  if (begin == end) {
    const auto &pose = path.poses[begin].pose.position;
    return terrainRiskAt(cache, pose.x, pose.y);
  }

  for (size_t i = begin; i < end; ++i) {
    const auto &from = path.poses[i].pose.position;
    const auto &to = path.poses[i + 1].pose.position;
    const double dx = to.x - from.x;
    const double dy = to.y - from.y;
    const double segment_length = std::hypot(dx, dy);
    const size_t steps =
        std::max<size_t>(1U, static_cast<size_t>(std::ceil(segment_length / terrain_sample_spacing_m_)));

    double normal_x = 0.0;
    double normal_y = 0.0;
    if (segment_length > 1e-6) {
      normal_x = -dy / segment_length;
      normal_y = dx / segment_length;
    }

    for (size_t step = 0; step <= steps; ++step) {
      const double t = static_cast<double>(step) / static_cast<double>(steps);
      const double sample_x = from.x + dx * t;
      const double sample_y = from.y + dy * t;
      if (terrainRiskAt(cache, sample_x, sample_y) ||
          terrainRiskAt(cache, sample_x + normal_x * stop_envelope_half_width_m_,
                        sample_y + normal_y * stop_envelope_half_width_m_) ||
          terrainRiskAt(cache, sample_x - normal_x * stop_envelope_half_width_m_,
                        sample_y - normal_y * stop_envelope_half_width_m_)) {
        return true;
      }
    }
  }

  const auto &goal_pose = path.poses[end].pose.position;
  return terrainRiskAt(cache, goal_pose.x, goal_pose.y);
}

size_t XhuMotionFollower::findNearestIndex(const nav_msgs::msg::Path &path, double x, double y,
                                           size_t hint) const {
  if (path.poses.empty()) {
    return 0U;
  }

  size_t best = std::min(hint, path.poses.size() - 1);
  double best_dist = std::numeric_limits<double>::infinity();
  for (size_t i = best; i < path.poses.size(); ++i) {
    const auto &pose = path.poses[i].pose.position;
    const double dx = pose.x - x;
    const double dy = pose.y - y;
    const double dist = std::hypot(dx, dy);
    if (dist < best_dist) {
      best_dist = dist;
      best = i;
    }
  }

  return best;
}

size_t XhuMotionFollower::findLookaheadIndex(const nav_msgs::msg::Path &path, size_t start_index,
                                             double lookahead_distance) const {
  if (path.poses.empty()) {
    return 0U;
  }

  double accumulated = 0.0;
  for (size_t i = start_index; i + 1 < path.poses.size(); ++i) {
    const auto &from = path.poses[i].pose.position;
    const auto &to = path.poses[i + 1].pose.position;
    accumulated += std::hypot(to.x - from.x, to.y - from.y);
    if (accumulated >= lookahead_distance) {
      return i + 1;
    }
  }
  return path.poses.size() - 1;
}

void XhuMotionFollower::publishLookaheadPath(const nav_msgs::msg::Path &source, size_t nearest_index,
                                             size_t lookahead_index) {
  nav_msgs::msg::Path msg;
  msg.header.frame_id = map_frame_;
  msg.header.stamp = now();
  if (source.poses.empty()) {
    lookahead_pub_->publish(msg);
    return;
  }

  const size_t begin = std::min(nearest_index, source.poses.size() - 1);
  const size_t end = std::min(lookahead_index, source.poses.size() - 1);
  for (size_t i = begin; i <= end; ++i) {
    msg.poses.push_back(source.poses[i]);
    msg.poses.back().header = msg.header;
  }
  lookahead_pub_->publish(msg);
}

void XhuMotionFollower::publishSemanticGate(const std::string &status) {
  std_msgs::msg::String gate;
  gate.data = status;
  semantic_gate_pub_->publish(gate);
}

void XhuMotionFollower::publishRuntimeStateLocked(
    const rc26_interfaces::msg::XhuSemanticCorridor &corridor, const std::string &status,
    bool terminal, const std::string &reason, float cross_track_error, float heading_error,
    float distance_to_goal) {
  rc26_interfaces::msg::XhuTrackingState tracking;
  tracking.header.stamp = now();
  tracking.corridor_id = corridor.corridor_id;
  tracking.edge_id = corridor.edge_id;
  tracking.status = status;
  tracking.terminal = terminal;
  tracking.cross_track_error = cross_track_error;
  tracking.heading_error = heading_error;
  tracking.distance_to_goal = distance_to_goal;
  tracking.cmd_vx = static_cast<float>(last_cmd_.linear.x);
  tracking.cmd_vy = static_cast<float>(last_cmd_.linear.y);
  tracking.cmd_wz = static_cast<float>(last_cmd_.angular.z);
  tracking.reason = reason;
  tracking_pub_->publish(tracking);
  publishSemanticGate(status);
}

void XhuMotionFollower::publishTerminalStateLocked(
    const rc26_interfaces::msg::XhuSemanticCorridor &corridor, const std::string &status,
    const std::string &reason, float cross_track_error, float heading_error,
    float distance_to_goal) {
  publishZeroCommandLocked();
  if (!corridor.corridor_id.empty()) {
    publishRuntimeStateLocked(corridor, status, true, reason, cross_track_error, heading_error,
                              distance_to_goal);
  }
  active_corridor_.reset();
  hold_since_.reset();
}

void XhuMotionFollower::publishHoldStateLocked(
    const rc26_interfaces::msg::XhuSemanticCorridor &corridor, const rclcpp::Time &stamp,
    const std::string &reason, float distance_to_goal) {
  publishZeroCommandLocked();
  if (!hold_since_.has_value()) {
    hold_since_ = stamp;
  }
  const bool timeout = (stamp - *hold_since_).seconds() > hold_to_abort_sec_;
  if (timeout) {
    publishTerminalStateLocked(corridor, "ABORT", reason, 0.0F, 0.0F, distance_to_goal);
  } else {
    publishRuntimeStateLocked(corridor, "HOLD", false, reason, 0.0F, 0.0F, distance_to_goal);
  }
}

void XhuMotionFollower::publishZeroCommandLocked() {
  last_cmd_ = geometry_msgs::msg::Twist{};
  cmd_pub_->publish(last_cmd_);
}

void XhuMotionFollower::controlLoop() {
  const auto stamp = now();
  const double raw_dt = (stamp - last_control_stamp_).seconds();
  const double dt = (raw_dt > 1e-3) ? raw_dt : (1.0 / control_frequency_hz_);
  last_control_stamp_ = stamp;

  std::shared_ptr<const rc26_interfaces::msg::XhuSemanticCorridor> corridor;
  std::shared_ptr<const TerrainCache> terrain_cache;
  size_t nearest_index_hint = 0U;
  rclcpp::Time corridor_start_stamp{0, 0, RCL_ROS_TIME};
  uint8_t loc_health_level = rc26_interfaces::msg::LocalizationHealth::GREEN;
  bool stop_required = false;
  bool mode_state_fresh = false;
  std::string active_mode;
  double mode_linear = default_max_linear_speed_;
  double mode_angular = default_max_angular_speed_;
  double mode_linear_accel = default_max_linear_accel_;
  double mode_angular_accel = default_max_angular_accel_;
  geometry_msgs::msg::Twist last_cmd_snapshot;

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!active_corridor_) {
      publishZeroCommandLocked();
      return;
    }

    corridor = active_corridor_;
    terrain_cache = terrain_cache_;
    nearest_index_hint = nearest_index_;
    corridor_start_stamp = corridor_start_stamp_;
    loc_health_level = loc_health_level_;
    last_cmd_snapshot = last_cmd_;

    if (has_mode_state_) {
      mode_state_fresh = (stamp - mode_state_stamp_).seconds() <= mode_state_timeout_sec_;
      active_mode = toLowerCopy(mode_state_.active_mode);
      stop_required = mode_state_.stop_required;
      if (mode_state_.max_linear_speed > 0.0F) {
        mode_linear = mode_state_.max_linear_speed;
      }
      if (mode_state_.max_angular_speed > 0.0F) {
        mode_angular = mode_state_.max_angular_speed;
      }
      if (mode_state_.max_linear_accel > 0.0F) {
        mode_linear_accel = mode_state_.max_linear_accel;
      }
      if (mode_state_.max_angular_accel > 0.0F) {
        mode_angular_accel = mode_state_.max_angular_accel;
      }
    }
  }

  if (!corridor) {
    return;
  }

  if ((stamp - corridor_start_stamp).seconds() > corridor_timeout_sec_) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (active_corridor_ && active_corridor_->corridor_id == corridor->corridor_id) {
      publishTerminalStateLocked(*corridor, "REPLAN", "corridor timeout", 0.0F, 0.0F, 0.0F);
    }
    return;
  }

  double robot_x = 0.0;
  double robot_y = 0.0;
  double robot_yaw = 0.0;
  if (!queryRobotPose(robot_x, robot_y, robot_yaw)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "xhu_motion_follower cannot get robot pose");
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (active_corridor_ && active_corridor_->corridor_id == corridor->corridor_id) {
      publishHoldStateLocked(*corridor, stamp, "robot pose unavailable", 0.0F);
    }
    return;
  }

  const auto &path = corridor->path;
  if (path.poses.empty()) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (active_corridor_ && active_corridor_->corridor_id == corridor->corridor_id) {
      publishTerminalStateLocked(*corridor, "ABORT", "corridor path empty", 0.0F, 0.0F, 0.0F);
    }
    return;
  }

  const auto &goal_pose = path.poses.back().pose;
  const auto goal_dx = goal_pose.position.x - robot_x;
  const auto goal_dy = goal_pose.position.y - robot_y;
  const auto distance_to_goal = std::hypot(goal_dx, goal_dy);

  const bool mode_hold = active_mode == "hold";
  const bool loc_red =
      loc_health_level >= rc26_interfaces::msg::LocalizationHealth::RED;
  if (loc_red || stop_required || mode_hold || !mode_state_fresh) {
    std::string reason = "mode/localization hold";
    if (!mode_state_fresh) {
      reason = "motion mode state stale";
    } else if (mode_hold) {
      reason = "hold mode";
    } else if (stop_required) {
      reason = "mode stop_required";
    } else if (loc_red) {
      reason = "localization hold";
    }

    std::lock_guard<std::mutex> lock(data_mutex_);
    if (active_corridor_ && active_corridor_->corridor_id == corridor->corridor_id) {
      publishHoldStateLocked(*corridor, stamp, reason, static_cast<float>(distance_to_goal));
    }
    return;
  }

  const size_t nearest_index = findNearestIndex(path, robot_x, robot_y, nearest_index_hint);
  const double current_speed = tracked_diff_mode_
                                   ? std::fabs(last_cmd_snapshot.linear.x)
                                   : std::hypot(last_cmd_snapshot.linear.x, last_cmd_snapshot.linear.y);
  const double dynamic_lookahead =
      std::max(lookahead_distance_, brake_margin_m_ + current_speed * 0.8);
  const size_t lookahead_index = findLookaheadIndex(path, nearest_index, dynamic_lookahead);

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!active_corridor_ || active_corridor_->corridor_id != corridor->corridor_id) {
      return;
    }
    nearest_index_ = nearest_index;
    hold_since_.reset();
  }

  const auto goal_heading_error =
      std::fabs(normalizeAngle(yawFromQuaternion(goal_pose.orientation) - robot_yaw));
  const bool goal_reached = distance_to_goal <= goal_tolerance_xy_ &&
                            (!corridor->stop_at_end || goal_heading_error <= goal_tolerance_yaw_);
  if (goal_reached) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (active_corridor_ && active_corridor_->corridor_id == corridor->corridor_id) {
      publishTerminalStateLocked(*corridor, "PASS", "corridor completed", 0.0F,
                                 static_cast<float>(goal_heading_error),
                                 static_cast<float>(distance_to_goal));
    }
    return;
  }

  if (terrainRiskAhead(terrain_cache, path, nearest_index, lookahead_index)) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (active_corridor_ && active_corridor_->corridor_id == corridor->corridor_id) {
      publishTerminalStateLocked(*corridor, "REPLAN", "terrain risk", 0.0F, 0.0F,
                                 static_cast<float>(distance_to_goal));
    }
    return;
  }

  const auto &lookahead_pose = path.poses[lookahead_index].pose;
  const double dx = lookahead_pose.position.x - robot_x;
  const double dy = lookahead_pose.position.y - robot_y;
  const double cos_yaw = std::cos(robot_yaw);
  const double sin_yaw = std::sin(robot_yaw);
  const double err_x = cos_yaw * dx + sin_yaw * dy;
  const double err_y = -sin_yaw * dx + cos_yaw * dy;
  const double target_yaw = yawFromQuaternion(lookahead_pose.orientation);
  const double heading_error = normalizeAngle(target_yaw - robot_yaw);

  if (std::fabs(err_y) > max_cross_track_error_m_) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (active_corridor_ && active_corridor_->corridor_id == corridor->corridor_id) {
      publishTerminalStateLocked(*corridor, "REPLAN", "cross track error exceeded",
                                 static_cast<float>(err_y), static_cast<float>(heading_error),
                                 static_cast<float>(distance_to_goal));
    }
    return;
  }

  double cmd_vx = 0.0;
  double cmd_vy = 0.0;
  double cmd_wz = 0.0;
  if (tracked_diff_mode_) {
    cmd_vx = kp_linear_x_ * err_x;
    const double curvature = (dx * dx + dy * dy) > 1e-6 ? (2.0 * err_y / (dx * dx + dy * dy)) : 0.0;
    cmd_wz = kp_angular_ * heading_error + curvature_gain_ * cmd_vx * curvature;

    const double abs_heading_error = std::fabs(heading_error);
    if (abs_heading_error >= heading_stop_rad_) {
      cmd_vx = 0.0;
      cmd_wz = kp_angular_ * heading_error;
    } else if (abs_heading_error >= heading_slowdown_start_rad_) {
      const double denom = std::max(heading_stop_rad_ - heading_slowdown_start_rad_, 1e-6);
      const double scale = 1.0 - (abs_heading_error - heading_slowdown_start_rad_) / denom;
      cmd_vx *= clamp(scale, 0.0, 1.0);
    }
  } else {
    cmd_vx = kp_linear_x_ * err_x;
    cmd_vy = kp_linear_y_ * err_y;
    cmd_wz = kp_angular_ * heading_error;
  }

  if (!corridor->allow_reverse && cmd_vx < 0.0) {
    cmd_vx = 0.0;
  }

  if (loc_health_level == rc26_interfaces::msg::LocalizationHealth::YELLOW) {
    cmd_vx *= lhi_yellow_v_scale_;
    if (!tracked_diff_mode_) {
      cmd_vy *= lhi_yellow_v_scale_;
    }
    cmd_wz *= lhi_yellow_w_scale_;
  } else if (loc_health_level == rc26_interfaces::msg::LocalizationHealth::ORANGE) {
    cmd_vx *= lhi_orange_v_scale_;
    if (!tracked_diff_mode_) {
      cmd_vy *= lhi_orange_vy_scale_;
    }
    cmd_wz *= lhi_orange_w_scale_;
  }

  double linear_limit = mode_linear;
  if (corridor->max_linear_speed > 0.0F) {
    linear_limit = std::min(linear_limit, static_cast<double>(corridor->max_linear_speed));
  }
  double angular_limit = mode_angular;
  if (corridor->max_angular_speed > 0.0F) {
    angular_limit = std::min(angular_limit, static_cast<double>(corridor->max_angular_speed));
  }

  if (tracked_diff_mode_) {
    cmd_vx = clamp(cmd_vx, -linear_limit, linear_limit);
    cmd_vy = 0.0;
  } else {
    const double planar_norm = std::hypot(cmd_vx, cmd_vy);
    if (planar_norm > linear_limit && planar_norm > 1e-6) {
      const double scale = linear_limit / planar_norm;
      cmd_vx *= scale;
      cmd_vy *= scale;
    }
  }
  cmd_wz = clamp(cmd_wz, -angular_limit, angular_limit);

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!active_corridor_ || active_corridor_->corridor_id != corridor->corridor_id) {
      return;
    }

    const double max_dv = std::max(0.0, mode_linear_accel) * dt;
    const double max_dw = std::max(0.0, mode_angular_accel) * dt;
    cmd_vx = clamp(cmd_vx, last_cmd_.linear.x - max_dv, last_cmd_.linear.x + max_dv);
    if (tracked_diff_mode_) {
      cmd_vy = 0.0;
    } else {
      cmd_vy = clamp(cmd_vy, last_cmd_.linear.y - max_dv, last_cmd_.linear.y + max_dv);
    }
    cmd_wz = clamp(cmd_wz, last_cmd_.angular.z - max_dw, last_cmd_.angular.z + max_dw);

    last_cmd_.linear.x = cmd_vx;
    last_cmd_.linear.y = tracked_diff_mode_ ? 0.0 : cmd_vy;
    last_cmd_.angular.z = cmd_wz;
    cmd_pub_->publish(last_cmd_);

    publishRuntimeStateLocked(*corridor, "PASS", false, "tracking", static_cast<float>(err_y),
                              static_cast<float>(heading_error),
                              static_cast<float>(distance_to_goal));
  }

  publishLookaheadPath(path, nearest_index, lookahead_index);
}

}  // namespace rc26_omni_controller

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rc26_omni_controller::XhuMotionFollower>());
  rclcpp::shutdown();
  return 0;
}
