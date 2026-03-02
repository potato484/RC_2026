#include "rc26_nav_mode_manager/terrain_mode_adapter.hpp"

#include <algorithm>
#include <chrono>
#include <thread>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace rc26_nav_mode_manager {

TerrainModeAdapter::TerrainModeAdapter(const rclcpp::NodeOptions& options)
    : Node("terrain_mode_adapter", options) {
    this->declare_parameter<std::string>("terrain_node_name", "terrain_semantic");
    this->declare_parameter<std::string>("diagnostics_topic", diagnostics_topic_);
    this->declare_parameter<int>("wait_service_ms", wait_service_ms_);
    this->declare_parameter<int>("set_timeout_ms", set_timeout_ms_);
    this->declare_parameter<int>("retry_count", retry_count_);
    this->declare_parameter<int>("retry_interval_ms", retry_interval_ms_);
    this->declare_parameter<int>("readback_timeout_ms", readback_timeout_ms_);
    this->get_parameter("terrain_node_name", terrain_node_name_);
    this->get_parameter("diagnostics_topic", diagnostics_topic_);
    this->get_parameter("wait_service_ms", wait_service_ms_);
    this->get_parameter("set_timeout_ms", set_timeout_ms_);
    this->get_parameter("retry_count", retry_count_);
    this->get_parameter("retry_interval_ms", retry_interval_ms_);
    this->get_parameter("readback_timeout_ms", readback_timeout_ms_);

    wait_service_ms_ = std::max(10, wait_service_ms_);
    set_timeout_ms_ = std::max(10, set_timeout_ms_);
    retry_count_ = std::max(0, retry_count_);
    retry_interval_ms_ = std::max(0, retry_interval_ms_);
    readback_timeout_ms_ = std::max(10, readback_timeout_ms_);

    profile_map_ = {
        {"normal",      {"aggressive",   180.0, 3, "edge4", "edge8", 0.23}},
        {"safe",        {"conservative", 360.0, 1, "edge8", "edge8", 0.23}},
        {"mf_traverse", {"conservative", 360.0, 1, "edge8", "edge8", 0.23}},
        {"mf_exit",     {"conservative", 360.0, 1, "edge8", "edge8", 0.23}},
        {"mf_approach", {"conservative", 360.0, 1, "edge8", "edge8", 0.23}},
    };

    param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
        this->get_node_base_interface(),
        this->get_node_topics_interface(),
        this->get_node_graph_interface(),
        this->get_node_services_interface(),
        terrain_node_name_);
    nav_mode_client_ = this->create_client<rc26_interfaces::srv::SetNavMode>("set_nav_mode");
    pub_diagnostics_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        diagnostics_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

    sub_ = this->create_subscription<rc26_interfaces::msg::NavSafetyState>(
        "nav_safety_state", rclcpp::QoS(10),
        std::bind(&TerrainModeAdapter::onNavSafetyState, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "TerrainModeAdapter started, target: %s",
                terrain_node_name_.c_str());
}

void TerrainModeAdapter::onNavSafetyState(
    const rc26_interfaces::msg::NavSafetyState::ConstSharedPtr& msg) {
    const std::string& profile = msg->current_profile;
    if (profile == last_applied_profile_) return;

    auto it = profile_map_.find(profile);
    if (it == profile_map_.end()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "unknown profile: %s", profile.c_str());
        publishDiagnostics(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                           "unknown profile", profile);
        return;
    }
    applyConfig(profile, it->second);
}

void TerrainModeAdapter::applyConfig(const std::string& profile, const TerrainCfg& cfg) {
    std::string reason;
    if (!applyConfigWithRetry(profile, cfg, reason)) {
        const std::string full_reason = "terrain profile apply failed: " + reason;
        RCLCPP_ERROR(this->get_logger(), "%s, profile=%s", full_reason.c_str(), profile.c_str());
        publishDiagnostics(diagnostic_msgs::msg::DiagnosticStatus::ERROR, full_reason, profile);
        requestSafeMode("terrain_mode_adapter_apply_failed");
        return;
    }

    last_applied_profile_ = profile;
    RCLCPP_INFO(this->get_logger(),
                "applied profile: %s (policy=%s sector=%.0f obstacle_area=%d obstacle_mode=%s)",
                profile.c_str(), cfg.unknown_policy.c_str(), cfg.drop_forward_sector_deg,
                cfg.min_obstacle_area_cells, cfg.obstacle_neighbor_mode.c_str());
    publishDiagnostics(diagnostic_msgs::msg::DiagnosticStatus::OK, "terrain profile applied", profile);
}

bool TerrainModeAdapter::applyConfigWithRetry(const std::string& profile, const TerrainCfg& cfg, std::string& reason) {
    for (int attempt = 0; attempt <= retry_count_; ++attempt) {
        if (!service_ready_) {
            if (!param_client_->wait_for_service(std::chrono::milliseconds(wait_service_ms_))) {
                reason = "terrain param service unavailable";
                continue;
            }
            service_ready_ = true;
        }

        auto future = param_client_->set_parameters({
            rclcpp::Parameter("unknown_policy", cfg.unknown_policy),
            rclcpp::Parameter("drop_forward_sector_deg", cfg.drop_forward_sector_deg),
            rclcpp::Parameter("min_obstacle_area_cells", cfg.min_obstacle_area_cells),
            rclcpp::Parameter("obstacle_neighbor_mode", cfg.obstacle_neighbor_mode),
            rclcpp::Parameter("drop_neighbor_mode", cfg.drop_neighbor_mode),
            rclcpp::Parameter("jump_thresh_m", cfg.jump_thresh_m),
        });

        if (future.wait_for(std::chrono::milliseconds(set_timeout_ms_)) != std::future_status::ready) {
            reason = "set_parameters timeout";
            service_ready_ = false;
        } else {
            const auto results = future.get();
            bool accepted = true;
            for (const auto& r : results) {
                if (!r.successful) {
                    accepted = false;
                    reason = "set_parameters rejected: " + r.reason;
                    break;
                }
            }
            if (accepted && verifyAppliedConfig(cfg, reason)) {
                return true;
            }
            if (accepted && reason.empty()) {
                reason = "readback mismatch";
            }
        }

        if (attempt < retry_count_ && retry_interval_ms_ > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(retry_interval_ms_));
        }
    }
    if (reason.empty()) {
        reason = "unknown error";
    }
    (void)profile;
    return false;
}

bool TerrainModeAdapter::verifyAppliedConfig(const TerrainCfg& cfg, std::string& reason) {
    auto future = param_client_->get_parameters({
        "unknown_policy",
        "min_obstacle_area_cells",
        "obstacle_neighbor_mode",
    });
    if (future.wait_for(std::chrono::milliseconds(readback_timeout_ms_)) != std::future_status::ready) {
        reason = "get_parameters timeout";
        return false;
    }

    bool have_policy = false;
    bool have_area = false;
    bool have_mode = false;
    std::string policy;
    int area = 0;
    std::string mode;

    for (const auto& p : future.get()) {
        if (p.get_name() == "unknown_policy" &&
            p.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
            have_policy = true;
            policy = p.as_string();
        } else if (p.get_name() == "min_obstacle_area_cells" &&
                   p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
            have_area = true;
            area = static_cast<int>(p.as_int());
        } else if (p.get_name() == "obstacle_neighbor_mode" &&
                   p.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
            have_mode = true;
            mode = p.as_string();
        }
    }

    if (!have_policy || !have_area || !have_mode) {
        reason = "get_parameters missing required fields";
        return false;
    }
    if (policy != cfg.unknown_policy) {
        reason = "readback mismatch unknown_policy";
        return false;
    }
    if (area != cfg.min_obstacle_area_cells) {
        reason = "readback mismatch min_obstacle_area_cells";
        return false;
    }
    if (mode != cfg.obstacle_neighbor_mode) {
        reason = "readback mismatch obstacle_neighbor_mode";
        return false;
    }
    reason.clear();
    return true;
}

void TerrainModeAdapter::requestSafeMode(const std::string& reason) {
    if (!nav_mode_client_ || !nav_mode_client_->wait_for_service(std::chrono::seconds(0))) {
        return;
    }
    auto request = std::make_shared<rc26_interfaces::srv::SetNavMode::Request>();
    request->profile = "safe";
    request->timeout = 0.0f;
    request->reason = reason;
    (void)nav_mode_client_->async_send_request(request);
}

void TerrainModeAdapter::publishDiagnostics(uint8_t level, const std::string& message, const std::string& profile) {
    if (!pub_diagnostics_) {
        return;
    }
    diagnostic_msgs::msg::DiagnosticStatus st;
    st.level = level;
    st.name = this->get_fully_qualified_name();
    st.hardware_id = "R2";
    st.message = message;

    auto add_kv = [&](const std::string& key, const std::string& value) {
        diagnostic_msgs::msg::KeyValue kv;
        kv.key = key;
        kv.value = value;
        st.values.push_back(kv);
    };
    add_kv("profile", profile);
    add_kv("terrain_node_name", terrain_node_name_);
    add_kv("wait_service_ms", std::to_string(wait_service_ms_));
    add_kv("set_timeout_ms", std::to_string(set_timeout_ms_));
    add_kv("retry_count", std::to_string(retry_count_));
    add_kv("readback_timeout_ms", std::to_string(readback_timeout_ms_));

    diagnostic_msgs::msg::DiagnosticArray arr;
    arr.header.stamp = this->get_clock()->now();
    arr.status.push_back(st);
    pub_diagnostics_->publish(arr);
}

}  // namespace rc26_nav_mode_manager

RCLCPP_COMPONENTS_REGISTER_NODE(rc26_nav_mode_manager::TerrainModeAdapter)
