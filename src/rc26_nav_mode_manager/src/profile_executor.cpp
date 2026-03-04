#include "rc26_nav_mode_manager/profile_executor.hpp"

#include <algorithm>
#include <cmath>
#include <thread>

namespace rc26_nav_mode_manager {

namespace {

double getDoubleParamOr(rclcpp::Node* node, const std::string& name, double default_value) {
    if (!node->has_parameter(name)) {
        node->declare_parameter<double>(name, default_value);
    }
    double out = default_value;
    (void)node->get_parameter(name, out);
    if (!std::isfinite(out) || out < 0.0) {
        RCLCPP_WARN(node->get_logger(), "Invalid %s=%.6f, using default %.6f", name.c_str(), out, default_value);
        out = default_value;
    }
    return out;
}

std::string getStringParamOr(rclcpp::Node* node, const std::string& name, const std::string& default_value) {
    if (!node->has_parameter(name)) {
        node->declare_parameter<std::string>(name, default_value);
    }
    std::string out = default_value;
    (void)node->get_parameter(name, out);
    if (out.empty()) {
        out = default_value;
    }
    return out;
}

}  // namespace

ProfileExecutor::ProfileExecutor(rclcpp::Node* node, ProfileDB* db)
    : node_(node), db_(db) {
    odom_topic_ = getStringParamOr(node_, "odom_topic", "odom");
    controller_server_node_ = getStringParamOr(node_, "controller_server_node", "controller_server");
    costmap_node_name_ = getStringParamOr(node_, "costmap_node_name", "local_costmap/local_costmap");

    stop_linear_eps_ = getDoubleParamOr(node_, "stop_linear_eps_mps", 0.05);
    stop_angular_eps_ = getDoubleParamOr(node_, "stop_angular_eps_rps", 0.05);
    clear_timeout_sec_ = getDoubleParamOr(node_, "clear_timeout_sec", 2.0);
    param_timeout_sec_ = getDoubleParamOr(node_, "param_timeout_sec", 2.0);

    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, rclcpp::SensorDataQoS(),
        std::bind(&ProfileExecutor::odomCallback, this, std::placeholders::_1));

    const auto clear_srv = deriveLocalCostmapClearServiceName(costmap_node_name_);
    clear_costmap_client_ = node_->create_client<ClearEntireCostmap>(
        clear_srv);

    controller_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
        node_, controller_server_node_);
}

void ProfileExecutor::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    double vx = msg->twist.twist.linear.x;
    double vy = msg->twist.twist.linear.y;
    double linear = std::sqrt(vx * vx + vy * vy);
    double angular = std::abs(msg->twist.twist.angular.z);
    last_linear_speed_ = linear;
    last_angular_speed_ = angular;
    odom_received_ = true;

    std::lock_guard<std::mutex> lock(odom_window_mutex_);
    if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) {
        last_odom_time_ = node_->now();
    } else {
        last_odom_time_ = rclcpp::Time(msg->header.stamp);
    }
    linear_window_.push_back(linear);
    angular_window_.push_back(angular);
    if (static_cast<int>(linear_window_.size()) > kOdomWindowSize) {
        linear_window_.pop_front();
    }
    if (static_cast<int>(angular_window_.size()) > kOdomWindowSize) {
        angular_window_.pop_front();
    }
}

bool ProfileExecutor::checkRobotStopped() const {
    std::lock_guard<std::mutex> lock(odom_window_mutex_);
    if (linear_window_.empty()) return false;
    if ((node_->now() - last_odom_time_).seconds() > 0.2) return false;
    double max_lin = *std::max_element(linear_window_.begin(), linear_window_.end());
    double max_ang = *std::max_element(angular_window_.begin(), angular_window_.end());
    return max_lin < stop_linear_eps_ && max_ang < stop_angular_eps_;
}

bool ProfileExecutor::isCancelled(uint64_t epoch) const {
    return cancel_epoch_.load() != epoch;
}

uint64_t ProfileExecutor::cancel() {
    return ++cancel_epoch_;
}

std::string ProfileExecutor::getCurrentProfile() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return current_profile_;
}

bool ProfileExecutor::stepValidate(const NavProfile& profile, std::string& error) {
    if (!db_->exists(profile.name)) {
        error = "Profile '" + profile.name + "' not found";
        return false;
    }
    return true;
}

bool ProfileExecutor::stepPrecheck(const NavProfile& profile, std::string& error) {
    if (profile.precheck.require_stopped) {
        if (!odom_received_.load()) {
            error = "No odom received - cannot verify robot stopped for '" + profile.name + "'";
            return false;
        }
        if (!checkRobotStopped()) {
            error = "Robot must be stopped before switching to '" + profile.name + "'";
            return false;
        }
    }
    return true;
}

bool ProfileExecutor::stepCostmap(const NavProfile& profile, std::string& error) {
    if (!profile.costmap.clear_on_switch) {
        return true;
    }

    if (!clear_costmap_client_->wait_for_service(std::chrono::duration<double>(clear_timeout_sec_))) {
        error = "Costmap clear service not available";
        return false;
    }

    auto request = std::make_shared<ClearEntireCostmap::Request>();
    auto future = clear_costmap_client_->async_send_request(request);

    if (future.wait_for(std::chrono::duration<double>(clear_timeout_sec_)) != std::future_status::ready) {
        error = "Timeout clearing costmap";
        return false;
    }

    RCLCPP_INFO(node_->get_logger(), "Costmap cleared for profile '%s'", profile.name.c_str());
    return true;
}

bool ProfileExecutor::captureDefaults() {
    if (defaults_captured_) return true;

    if (!controller_param_client_->wait_for_service(std::chrono::duration<double>(param_timeout_sec_))) {
        RCLCPP_WARN(node_->get_logger(), "Controller param service not available for defaults capture");
        return false;
    }

    std::vector<std::string> param_names = {
        "FollowPath.v_linear_max",
        "FollowPath.v_angular_max",
        "FollowPath.v_linear_min",
        "FollowPath.acc_linear",
        "FollowPath.acc_angular"
    };

    auto future = controller_param_client_->get_parameters(param_names);
    if (future.wait_for(std::chrono::duration<double>(param_timeout_sec_)) != std::future_status::ready) {
        RCLCPP_WARN(node_->get_logger(), "Timeout getting controller defaults");
        return false;
    }

    auto params = future.get();
    for (const auto& p : params) {
        if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
            param_defaults_[p.get_name()] = p.as_double();
        }
    }

    if (param_defaults_.empty()) {
        RCLCPP_WARN(node_->get_logger(), "No controller defaults captured");
        return false;
    }

    defaults_captured_ = true;
    RCLCPP_INFO(node_->get_logger(), "Captured %zu controller defaults", param_defaults_.size());
    return true;
}

bool ProfileExecutor::rollbackParams() {
    if (param_defaults_.empty()) return true;

    if (!controller_param_client_->wait_for_service(std::chrono::duration<double>(param_timeout_sec_))) {
        RCLCPP_ERROR(node_->get_logger(), "Controller param service not available for rollback");
        return false;
    }

    std::vector<rclcpp::Parameter> params;
    for (const auto& [name, value] : param_defaults_) {
        params.emplace_back(name, value);
    }

    auto future = controller_param_client_->set_parameters(params);
    if (future.wait_for(std::chrono::duration<double>(param_timeout_sec_)) != std::future_status::ready) {
        RCLCPP_ERROR(node_->get_logger(), "Timeout rolling back params");
        return false;
    }

    auto results = future.get();
    for (const auto& r : results) {
        if (!r.successful) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to rollback param: %s", r.reason.c_str());
            return false;
        }
    }

    RCLCPP_INFO(node_->get_logger(), "Successfully rolled back %zu params", params.size());
    return true;
}

bool ProfileExecutor::stepParams(const NavProfile& profile, std::string& error) {
    const auto& ctrl = profile.controller;
    if (!ctrl.v_linear_max && !ctrl.v_angular_max && !ctrl.v_linear_min &&
        !ctrl.acc_linear && !ctrl.acc_angular) {
        return true;
    }

    if (!captureDefaults()) {
        error = "Failed to capture controller defaults";
        return false;
    }

    if (!controller_param_client_->wait_for_service(std::chrono::duration<double>(param_timeout_sec_))) {
        error = "Controller param service not available";
        return false;
    }

    bool need_phase1 = ctrl.v_linear_max && ctrl.acc_linear && [&]() {
        auto it = param_defaults_.find("FollowPath.v_linear_max");
        return it != param_defaults_.end() &&
               *ctrl.v_linear_max < it->second - 0.05;
    }();

    if (need_phase1) {
        double cur_acc = param_defaults_.count("FollowPath.acc_linear")
                         ? param_defaults_["FollowPath.acc_linear"] : *ctrl.acc_linear;
        std::vector<rclcpp::Parameter> phase1;
        phase1.emplace_back("FollowPath.acc_linear",
                            std::min(cur_acc, *ctrl.acc_linear * 0.5));
        if (ctrl.acc_angular && param_defaults_.count("FollowPath.acc_angular")) {
            double cur_ang = param_defaults_["FollowPath.acc_angular"];
            phase1.emplace_back("FollowPath.acc_angular",
                                std::min(cur_ang, *ctrl.acc_angular * 0.5));
        }
        auto f1 = controller_param_client_->set_parameters(phase1);
        if (f1.wait_for(std::chrono::duration<double>(param_timeout_sec_)) !=
                std::future_status::ready) {
            error = "Timeout on phase1 acc reduction";
            rollbackParams();
            return false;
        }
        for (const auto& r : f1.get()) {
            if (!r.successful) {
                error = "Failed phase1 param set: " + r.reason;
                rollbackParams();
                return false;
            }
        }
        std::this_thread::sleep_for(
            std::chrono::milliseconds(ctrl.transition_timeout_ms));
    }

    std::vector<rclcpp::Parameter> params;
    if (ctrl.v_linear_max) params.emplace_back("FollowPath.v_linear_max", *ctrl.v_linear_max);
    if (ctrl.v_angular_max) params.emplace_back("FollowPath.v_angular_max", *ctrl.v_angular_max);
    if (ctrl.v_linear_min) params.emplace_back("FollowPath.v_linear_min", *ctrl.v_linear_min);
    if (ctrl.acc_linear)   params.emplace_back("FollowPath.acc_linear",   *ctrl.acc_linear);
    if (ctrl.acc_angular)  params.emplace_back("FollowPath.acc_angular",  *ctrl.acc_angular);

    auto future = controller_param_client_->set_parameters(params);
    if (future.wait_for(std::chrono::duration<double>(param_timeout_sec_)) != std::future_status::ready) {
        error = "Timeout setting controller params";
        rollbackParams();
        return false;
    }

    auto results = future.get();
    for (const auto& r : results) {
        if (!r.successful) {
            error = "Failed to set param: " + r.reason;
            rollbackParams();
            return false;
        }
    }

    return true;
}

void ProfileExecutor::stepState(const NavProfile& profile, const std::string& reason) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_profile_ = profile.name;
    current_reason_ = reason;
}

std::string ProfileExecutor::deriveLocalCostmapClearServiceName(const std::string& costmap_node_name) const {
    // Nav2 common: node name "local_costmap/local_costmap", clear service "local_costmap/clear_entirely_local_costmap"
    // We take the namespace prefix before the last '/' as the service namespace.
    const auto pos = costmap_node_name.find_last_of('/');
    const std::string ns = (pos == std::string::npos) ? costmap_node_name : costmap_node_name.substr(0, pos);
    if (ns.empty()) {
        return "local_costmap/clear_entirely_local_costmap";
    }
    return ns + "/clear_entirely_local_costmap";
}

ProfileExecutor::SwitchResult ProfileExecutor::execute(
    const NavProfile& profile, const std::string& reason) {
    std::lock_guard<std::mutex> lock(execution_mutex_);
    uint64_t epoch = cancel_epoch_.load();
    return executeInternal(profile, reason, epoch);
}

ProfileExecutor::SwitchResult ProfileExecutor::executeForFallback(
    const NavProfile& profile, const std::string& reason) {
    std::lock_guard<std::mutex> lock(execution_mutex_);
    uint64_t epoch = cancel_epoch_.load();
    return executeInternal(profile, reason, epoch);
}

ProfileExecutor::SwitchResult ProfileExecutor::executeInternal(
    const NavProfile& profile, const std::string& reason, uint64_t epoch) {
    SwitchResult result;
    std::string error;

    // Step 1: Validate
    if (isCancelled(epoch)) {
        result.message = "Cancelled before validate";
        return result;
    }
    if (!stepValidate(profile, error)) {
        result.message = error;
        return result;
    }

    // Step 2: Precheck
    if (isCancelled(epoch)) {
        result.message = "Cancelled before precheck";
        return result;
    }
    if (!stepPrecheck(profile, error)) {
        result.message = error;
        return result;
    }

    // Step 3: Costmap
    if (isCancelled(epoch)) {
        result.message = "Cancelled before costmap";
        return result;
    }
    if (!stepCostmap(profile, error)) {
        result.message = error;
        return result;
    }

    // Step 4: Params
    if (isCancelled(epoch)) {
        result.message = "Cancelled before params";
        return result;
    }
    if (!stepParams(profile, error)) {
        result.message = error;
        return result;
    }

    // Step 5: State
    if (isCancelled(epoch)) {
        result.message = "Cancelled before state update";
        rollbackParams();
        return result;
    }
    stepState(profile, reason);

    result.success = true;
    result.message = "Switched to profile '" + profile.name + "'";
    RCLCPP_INFO(node_->get_logger(), "Profile switched to '%s' (reason: %s)",
                profile.name.c_str(), reason.c_str());
    return result;
}

}  // namespace rc26_nav_mode_manager
