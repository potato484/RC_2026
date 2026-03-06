#include "rc26_nav_mode_manager/terrain_mode_adapter.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <thread>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace rc26_nav_mode_manager {

TerrainModeAdapter::TerrainModeAdapter(const rclcpp::NodeOptions& options)
    : Node("terrain_mode_adapter", options) {
    this->declare_parameter<std::string>("terrain_node_name", "terrain_semantic");
    this->declare_parameter<std::string>("terrain_profiles_file", "");
    this->declare_parameter<std::string>("diagnostics_topic", diagnostics_topic_);
    this->declare_parameter<int>("wait_service_ms", wait_service_ms_);
    this->declare_parameter<int>("set_timeout_ms", set_timeout_ms_);
    this->declare_parameter<int>("retry_count", retry_count_);
    this->declare_parameter<int>("retry_interval_ms", retry_interval_ms_);
    this->declare_parameter<int>("readback_timeout_ms", readback_timeout_ms_);
    this->get_parameter("terrain_node_name", terrain_node_name_);
    this->get_parameter("terrain_profiles_file", terrain_profiles_file_);
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

    if (terrain_profiles_file_.empty()) {
        terrain_profiles_file_ = ament_index_cpp::get_package_share_directory(
            "rc26_nav_mode_manager") + "/config/terrain_profiles.yaml";
    }
    {
        YAML::Node root;
        try {
            root = YAML::LoadFile(terrain_profiles_file_);
        } catch (const YAML::Exception& e) {
            throw std::runtime_error("Failed to load terrain_profiles: " + std::string(e.what()));
        }
        if (!root["terrain_profiles"]) {
            throw std::runtime_error("terrain_profiles.yaml missing 'terrain_profiles' key");
        }
        for (const auto& item : root["terrain_profiles"]) {
            const std::string name = item.first.as<std::string>();
            const auto& n = item.second;
            TerrainCfg cfg;
            cfg.unknown_policy = n["unknown_policy"].as<std::string>();
            cfg.drop_forward_sector_deg = n["drop_forward_sector_deg"].as<double>();
            cfg.min_obstacle_area_cells = n["min_obstacle_area_cells"].as<int>();
            cfg.obstacle_neighbor_mode = n["obstacle_neighbor_mode"].as<std::string>();
            cfg.drop_neighbor_mode = n["drop_neighbor_mode"].as<std::string>();
            cfg.jump_thresh_m = n["jump_thresh_m"].as<double>();
            profile_map_[name] = cfg;
        }
        RCLCPP_INFO(this->get_logger(), "Loaded %zu terrain profiles from %s",
                    profile_map_.size(), terrain_profiles_file_.c_str());
    }

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

    worker_thread_ = std::thread(&TerrainModeAdapter::workerLoop, this);
    RCLCPP_INFO(this->get_logger(), "TerrainModeAdapter started, target: %s",
                terrain_node_name_.c_str());
}

TerrainModeAdapter::~TerrainModeAdapter() {
    {
        std::lock_guard<std::mutex> lock(worker_mutex_);
        stop_worker_ = true;
    }
    worker_cv_.notify_one();
    if (worker_thread_.joinable()) {
        worker_thread_.join();
    }
}

void TerrainModeAdapter::onNavSafetyState(
    const rc26_interfaces::msg::NavSafetyState::ConstSharedPtr& msg) {
    {
        std::lock_guard<std::mutex> lock(worker_mutex_);
        pending_profile_ = msg->current_profile;
    }
    worker_cv_.notify_one();
}

void TerrainModeAdapter::workerLoop() {
    while (true) {
        std::string profile;
        {
            std::unique_lock<std::mutex> lock(worker_mutex_);
            worker_cv_.wait(lock, [this] {
                return stop_worker_ || !pending_profile_.empty();
            });
            if (stop_worker_) {
                break;
            }
            if (pending_profile_ == last_applied_profile_) {
                pending_profile_.clear();
                continue;
            }
            profile = std::move(pending_profile_);
            pending_profile_.clear();
        }
        auto it = profile_map_.find(profile);
        if (it == profile_map_.end()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "unknown profile: %s", profile.c_str());
            publishDiagnostics(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                               "unknown profile", profile);
            continue;
        }
        try {
            applyConfig(profile, it->second);
        } catch (const std::exception& e) {
            const std::string reason = "terrain worker exception: " + std::string(e.what());
            RCLCPP_ERROR(this->get_logger(), "%s", reason.c_str());
            publishDiagnostics(diagnostic_msgs::msg::DiagnosticStatus::ERROR, reason, profile);
            requestSafeMode("terrain_mode_adapter_worker_exception");
        }
    }
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

        try {
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
        } catch (const std::exception& e) {
            reason = "terrain parameter RPC failed: " + std::string(e.what());
            service_ready_ = false;
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
        "drop_forward_sector_deg",
        "drop_neighbor_mode",
        "jump_thresh_m",
    });
    if (future.wait_for(std::chrono::milliseconds(readback_timeout_ms_)) != std::future_status::ready) {
        reason = "get_parameters timeout";
        return false;
    }

    auto approx_eq = [](double a, double b) {
        return std::fabs(a - b) <= std::max(1e-6, 1e-6 * std::fabs(b));
    };

    std::string policy;
    int area = 0;
    std::string obstacle_mode;
    double sector_deg = 0.0;
    std::string drop_mode;
    double jump_thresh = 0.0;
    int found = 0;

    std::vector<rclcpp::Parameter> readback;
    try {
        readback = future.get();
    } catch (const std::exception& e) {
        reason = "get_parameters failed: " + std::string(e.what());
        service_ready_ = false;
        return false;
    }

    for (const auto& p : readback) {
        const auto& n = p.get_name();
        if (n == "unknown_policy" && p.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
            policy = p.as_string();
            ++found;
        } else if (n == "min_obstacle_area_cells" && p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
            area = static_cast<int>(p.as_int());
            ++found;
        } else if (n == "obstacle_neighbor_mode" && p.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
            obstacle_mode = p.as_string();
            ++found;
        } else if (n == "drop_forward_sector_deg" && p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
            sector_deg = p.as_double();
            ++found;
        } else if (n == "drop_neighbor_mode" && p.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
            drop_mode = p.as_string();
            ++found;
        } else if (n == "jump_thresh_m" && p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
            jump_thresh = p.as_double();
            ++found;
        }
    }

    if (found != 6) { reason = "get_parameters missing fields"; return false; }
    if (policy != cfg.unknown_policy) { reason = "mismatch unknown_policy"; return false; }
    if (area != cfg.min_obstacle_area_cells) { reason = "mismatch min_obstacle_area_cells"; return false; }
    if (obstacle_mode != cfg.obstacle_neighbor_mode) { reason = "mismatch obstacle_neighbor_mode"; return false; }
    if (!approx_eq(sector_deg, cfg.drop_forward_sector_deg)) { reason = "mismatch drop_forward_sector_deg"; return false; }
    if (drop_mode != cfg.drop_neighbor_mode) { reason = "mismatch drop_neighbor_mode"; return false; }
    if (!approx_eq(jump_thresh, cfg.jump_thresh_m)) { reason = "mismatch jump_thresh_m"; return false; }
    reason.clear();
    return true;
}

void TerrainModeAdapter::requestSafeMode(const std::string& reason) {
    if (!nav_mode_client_) {
        return;
    }

    if (!nav_mode_client_->wait_for_service(std::chrono::milliseconds(wait_service_ms_))) {
        RCLCPP_ERROR(this->get_logger(),
                     "set_nav_mode unavailable while requesting safe mode: %s",
                     reason.c_str());
        publishDiagnostics(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                           "set_nav_mode unavailable", "safe");
        return;
    }

    auto request = std::make_shared<rc26_interfaces::srv::SetNavMode::Request>();
    request->profile = "safe";
    request->timeout = 0.0f;
    request->reason = reason;
    try {
        (void)nav_mode_client_->async_send_request(request);
    } catch (const std::exception& e) {
        const std::string error = "failed to request safe mode: " + std::string(e.what());
        RCLCPP_ERROR(this->get_logger(), "%s", error.c_str());
        publishDiagnostics(diagnostic_msgs::msg::DiagnosticStatus::ERROR, error, "safe");
    }
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
