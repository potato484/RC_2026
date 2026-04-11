#include "rc26_xhu_nav/mode_manager/xhu_motion_mode_manager.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

namespace rc26_xhu_nav::mode_manager {

namespace {

double readDoubleOr(const YAML::Node &node, const char *key, double default_value) {
  const auto value = node[key];
  if (!value) {
    return default_value;
  }
  return value.as<double>();
}

double readControllerAccel(const YAML::Node &controller, const char *legacy_key,
                           const char *canonical_key, double default_value) {
  const auto canonical = controller[canonical_key];
  if (canonical) {
    return canonical.as<double>();
  }
  const auto legacy = controller[legacy_key];
  if (legacy) {
    return legacy.as<double>();
  }
  return default_value;
}

void normalizeProfile(XhuProfile &profile) {
  profile.watchdog_timeout_sec = std::max(0.0, profile.watchdog_timeout_sec);
  profile.max_linear_speed = std::max(0.0, profile.max_linear_speed);
  profile.max_angular_speed = std::max(0.0, profile.max_angular_speed);
  profile.max_linear_accel = std::max(0.0, profile.max_linear_accel);
  profile.max_angular_accel = std::max(0.0, profile.max_angular_accel);
}

void ensureAlias(std::unordered_map<std::string, XhuProfile> &profiles, const std::string &target,
                 const std::string &source, const std::string &fallback_override,
                 std::vector<std::string> &synthesized) {
  if (profiles.find(target) != profiles.end()) {
    return;
  }
  const auto source_it = profiles.find(source);
  if (source_it == profiles.end()) {
    return;
  }

  auto alias = source_it->second;
  alias.name = target;
  if (!fallback_override.empty()) {
    alias.fallback_profile = fallback_override;
  }
  normalizeProfile(alias);
  profiles.emplace(target, std::move(alias));
  synthesized.push_back(target + "<-" + source);
}

void ensureHoldProfile(std::unordered_map<std::string, XhuProfile> &profiles,
                       std::vector<std::string> &synthesized) {
  if (profiles.find("hold") != profiles.end()) {
    return;
  }

  XhuProfile hold_profile;
  auto source_it = profiles.find("loc_red_hold");
  if (source_it != profiles.end()) {
    hold_profile = source_it->second;
    synthesized.push_back("hold<-loc_red_hold");
  } else {
    hold_profile.name = "hold";
    hold_profile.fallback_profile = "hold";
    hold_profile.require_stopped = true;
    hold_profile.watchdog_timeout_sec = 0.0;
    hold_profile.stop_required_on_timeout = true;
    hold_profile.max_linear_speed = 0.0;
    hold_profile.max_angular_speed = 0.0;
    hold_profile.max_linear_accel = 0.2;
    hold_profile.max_angular_accel = 0.2;
    synthesized.push_back("hold<default>");
  }

  hold_profile.name = "hold";
  hold_profile.fallback_profile = "hold";
  hold_profile.require_stopped = true;
  hold_profile.stop_required_on_timeout = true;
  hold_profile.max_linear_speed = 0.0;
  hold_profile.max_angular_speed = 0.0;
  normalizeProfile(hold_profile);
  profiles["hold"] = std::move(hold_profile);
}

std::unordered_map<std::string, XhuProfile> loadProfiles(const std::string &file_path,
                                                         std::vector<std::string> &synthesized) {
  std::unordered_map<std::string, XhuProfile> profiles;
  const YAML::Node root = YAML::LoadFile(file_path);
  const auto profile_root = root["profiles"];
  if (!profile_root || !profile_root.IsMap()) {
    throw std::runtime_error("profiles root is missing or not a map");
  }

  for (const auto &item : profile_root) {
    XhuProfile profile;
    profile.name = item.first.as<std::string>();
    const auto cfg = item.second;
    profile.fallback_profile =
        cfg["fallback_profile"] ? cfg["fallback_profile"].as<std::string>() : profile.name;

    if (const auto watchdog = cfg["watchdog"]) {
      profile.watchdog_timeout_sec = readDoubleOr(watchdog, "timeout_sec", 0.0);
      if (watchdog["stop_required_on_timeout"]) {
        profile.stop_required_on_timeout = watchdog["stop_required_on_timeout"].as<bool>();
      }
    }

    if (const auto precheck = cfg["precheck"]) {
      if (precheck["require_stopped"]) {
        profile.require_stopped = precheck["require_stopped"].as<bool>();
      }
    }

    if (const auto controller = cfg["controller"]) {
      profile.max_linear_speed = readDoubleOr(controller, "v_linear_max", profile.max_linear_speed);
      profile.max_angular_speed =
          readDoubleOr(controller, "v_angular_max", profile.max_angular_speed);
      profile.max_linear_accel = readControllerAccel(
          controller, "acc_linear", "a_linear_max", profile.max_linear_accel);
      profile.max_angular_accel = readControllerAccel(
          controller, "acc_angular", "a_angular_max", profile.max_angular_accel);
    }

    normalizeProfile(profile);
    profiles[profile.name] = std::move(profile);
  }

  ensureHoldProfile(profiles, synthesized);
  ensureAlias(profiles, "plane_move", "normal", "hold", synthesized);
  ensureAlias(profiles, "plane_move", "safe", "hold", synthesized);
  ensureAlias(profiles, "ramp_up", "stair_up", "hold", synthesized);
  ensureAlias(profiles, "ramp_down", "stair_down", "hold", synthesized);
  ensureAlias(profiles, "mf_exit", "mf_traverse", "hold", synthesized);

  for (auto &[name, profile] : profiles) {
    if (profile.fallback_profile.empty()) {
      profile.fallback_profile = "hold";
    }
    if (profiles.find(profile.fallback_profile) == profiles.end()) {
      throw std::runtime_error("profile '" + name + "' fallback_profile '" +
                               profile.fallback_profile + "' does not exist");
    }
  }

  return profiles;
}

}  // namespace

XhuMotionModeManager::XhuMotionModeManager(const rclcpp::NodeOptions &options)
    : Node("xhu_motion_mode_manager", options) {
  declare_parameter<std::string>("profiles_file", "");
  declare_parameter<std::string>("odom_topic", "odom");
  declare_parameter<double>("stop_linear_eps_mps", 0.05);
  declare_parameter<double>("stop_angular_eps_rps", 0.05);
  declare_parameter<double>("odom_stale_timeout_sec", 0.25);
  declare_parameter<double>("stop_window_sec", 0.35);
  declare_parameter<double>("publish_rate_hz", 5.0);
  declare_parameter<std::string>("default_mode", "hold");

  auto profiles_file = get_parameter("profiles_file").as_string();
  if (profiles_file.empty()) {
    const auto pkg_dir = ament_index_cpp::get_package_share_directory("rc26_xhu_nav");
    profiles_file = pkg_dir + "/config/nav_profiles.yaml";
  }

  std::vector<std::string> synthesized_profiles;
  profiles_ = loadProfiles(profiles_file, synthesized_profiles);
  RCLCPP_INFO(get_logger(), "Loaded %zu xhu motion profiles from %s", profiles_.size(),
              profiles_file.c_str());
  for (const auto &alias : synthesized_profiles) {
    RCLCPP_WARN(get_logger(), "Synthesized xhu motion profile alias %s", alias.c_str());
  }

  const auto odom_topic = get_parameter("odom_topic").as_string();
  stop_linear_eps_mps_ = std::max(0.0, get_parameter("stop_linear_eps_mps").as_double());
  stop_angular_eps_rps_ = std::max(0.0, get_parameter("stop_angular_eps_rps").as_double());
  odom_stale_timeout_sec_ = std::max(0.05, get_parameter("odom_stale_timeout_sec").as_double());
  stop_window_sec_ = std::max(0.05, get_parameter("stop_window_sec").as_double());

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::SensorDataQoS(),
      std::bind(&XhuMotionModeManager::onOdom, this, std::placeholders::_1));

  mode_state_pub_ = create_publisher<rc26_interfaces::msg::XhuMotionModeState>(
      "/xhu_nav/motion_mode_state", 10);

  set_mode_srv_ = create_service<rc26_interfaces::srv::SetXhuMotionMode>(
      "set_xhu_motion_mode",
      std::bind(&XhuMotionModeManager::handleSetMode, this, std::placeholders::_1,
                std::placeholders::_2));

  const auto default_mode = get_parameter("default_mode").as_string();
  if (profiles_.find(default_mode) != profiles_.end()) {
    current_mode_ = default_mode;
  } else if (profiles_.find("hold") != profiles_.end()) {
    current_mode_ = "hold";
  } else {
    current_mode_ = profiles_.begin()->first;
  }
  current_reason_ = "startup";

  const auto publish_rate_hz = std::max(1.0, get_parameter("publish_rate_hz").as_double());
  const auto publish_period = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_hz));

  publish_timer_ = create_wall_timer(publish_period,
                                     std::bind(&XhuMotionModeManager::publishState, this));

  watchdog_timer_ = create_wall_timer(std::chrono::milliseconds(100),
                                      std::bind(&XhuMotionModeManager::checkWatchdog, this));

  armWatchdogForMode(current_mode_, std::nullopt);
  publishState();
}

void XhuMotionModeManager::onOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
  const auto now = get_clock()->now();
  const auto linear = std::hypot(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
  const auto angular = std::fabs(msg->twist.twist.angular.z);

  std::lock_guard<std::mutex> lock(odom_mutex_);
  last_odom_time_ = now;
  odom_window_.push_back(VelocitySample{now, linear, angular});
  pruneOdomWindowLocked(now);
}

void XhuMotionModeManager::pruneOdomWindowLocked(const rclcpp::Time &now) {
  const auto keep_duration = rclcpp::Duration::from_seconds(stop_window_sec_ * 2.0);
  while (!odom_window_.empty() && (now - odom_window_.front().stamp) > keep_duration) {
    odom_window_.pop_front();
  }
}

bool XhuMotionModeManager::isRobotStopped() const {
  std::lock_guard<std::mutex> lock(odom_mutex_);
  if (odom_window_.empty()) {
    return false;
  }
  if ((this->now() - last_odom_time_).seconds() > odom_stale_timeout_sec_) {
    return false;
  }

  const auto newest_stamp = odom_window_.back().stamp;
  const auto required_window = rclcpp::Duration::from_seconds(stop_window_sec_);
  if ((newest_stamp - odom_window_.front().stamp) < required_window) {
    return false;
  }

  for (const auto &sample : odom_window_) {
    if ((newest_stamp - sample.stamp) > required_window) {
      continue;
    }
    if (sample.linear >= stop_linear_eps_mps_ || sample.angular >= stop_angular_eps_rps_) {
      return false;
    }
  }
  return true;
}

void XhuMotionModeManager::handleSetMode(
    const rc26_interfaces::srv::SetXhuMotionMode::Request::SharedPtr request,
    rc26_interfaces::srv::SetXhuMotionMode::Response::SharedPtr response) {
  if (request->mode.empty()) {
    response->success = false;
    response->message = "mode cannot be empty";
    return;
  }

  const auto profile_it = profiles_.find(request->mode);
  if (profile_it == profiles_.end()) {
    response->success = false;
    response->message = "unknown mode '" + request->mode + "'";
    return;
  }

  const auto &profile = profile_it->second;
  if (profile.require_stopped && !isRobotStopped()) {
    response->success = false;
    response->message = "mode '" + request->mode + "' requires robot stopped precheck";
    return;
  }

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_mode_ = profile.name;
    current_reason_ = request->reason.empty() ? "service_request" : request->reason;
    stop_required_ = false;
    timed_out_ = false;
  }

  std::optional<double> timeout_override;
  if (request->timeout > 0.0F) {
    timeout_override = static_cast<double>(request->timeout);
  }
  armWatchdogForMode(profile.name, timeout_override);
  publishState();

  response->success = true;
  response->message = "switched to mode '" + profile.name + "'";
}

void XhuMotionModeManager::armWatchdogForMode(const std::string &mode,
                                              std::optional<double> timeout_override) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  const auto profile_it = profiles_.find(mode);
  if (profile_it == profiles_.end()) {
    watchdog_deadline_.reset();
    return;
  }

  const double timeout =
      timeout_override.has_value() ? *timeout_override : profile_it->second.watchdog_timeout_sec;
  if (timeout <= 0.0) {
    watchdog_deadline_.reset();
    return;
  }

  watchdog_deadline_ = get_clock()->now() + rclcpp::Duration::from_seconds(timeout);
}

void XhuMotionModeManager::checkWatchdog() {
  std::string from_mode;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!watchdog_deadline_.has_value() || get_clock()->now() < *watchdog_deadline_) {
      return;
    }
    from_mode = current_mode_;
  }

  const auto profile_it = profiles_.find(from_mode);
  if (profile_it == profiles_.end()) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    watchdog_deadline_.reset();
    return;
  }

  const auto &profile = profile_it->second;
  auto fallback = profile.fallback_profile;
  if (profiles_.find(fallback) == profiles_.end()) {
    fallback = "hold";
    if (profiles_.find(fallback) == profiles_.end()) {
      fallback = from_mode;
    }
  }

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_mode_ = fallback;
    current_reason_ = "watchdog_timeout";
    timed_out_ = true;
    stop_required_ = profile.stop_required_on_timeout;
  }

  RCLCPP_WARN(get_logger(), "watchdog timeout in mode '%s', fallback to '%s'", from_mode.c_str(),
              fallback.c_str());
  armWatchdogForMode(fallback, std::nullopt);
  publishState();
}

void XhuMotionModeManager::publishState() {
  rc26_interfaces::msg::XhuMotionModeState msg;
  msg.header.stamp = now();

  std::string mode;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    mode = current_mode_;
    msg.active_mode = current_mode_;
    msg.reason = current_reason_;
    msg.stop_required = stop_required_;
    msg.timed_out = timed_out_;
  }

  const auto profile_it = profiles_.find(mode);
  if (profile_it != profiles_.end()) {
    msg.max_linear_speed = static_cast<float>(profile_it->second.max_linear_speed);
    msg.max_angular_speed = static_cast<float>(profile_it->second.max_angular_speed);
    msg.max_linear_accel = static_cast<float>(profile_it->second.max_linear_accel);
    msg.max_angular_accel = static_cast<float>(profile_it->second.max_angular_accel);
  }

  if (msg.stop_required || mode == "hold") {
    msg.max_linear_speed = 0.0F;
    msg.max_angular_speed = 0.0F;
  }

  mode_state_pub_->publish(msg);
}

}  // namespace rc26_xhu_nav::mode_manager

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rc26_xhu_nav::mode_manager::XhuMotionModeManager>());
  rclcpp::shutdown();
  return 0;
}
