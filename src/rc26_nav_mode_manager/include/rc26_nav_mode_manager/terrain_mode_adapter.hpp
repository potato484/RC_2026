#pragma once

#include <condition_variable>
#include <map>
#include <mutex>
#include <string>
#include <thread>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rc26_interfaces/msg/nav_safety_state.hpp"
#include "rc26_interfaces/srv/set_nav_mode.hpp"

namespace rc26_nav_mode_manager {

class TerrainModeAdapter : public rclcpp::Node {
public:
    explicit TerrainModeAdapter(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~TerrainModeAdapter();

private:
    struct TerrainCfg {
        std::string unknown_policy;
        double      drop_forward_sector_deg;
        int         min_obstacle_area_cells;
        std::string obstacle_neighbor_mode;
        std::string drop_neighbor_mode;
        double      jump_thresh_m;
    };

    void onNavSafetyState(const rc26_interfaces::msg::NavSafetyState::ConstSharedPtr& msg);
    void applyConfig(const std::string& profile, const TerrainCfg& cfg);
    bool applyConfigWithRetry(const std::string& profile, const TerrainCfg& cfg, std::string& reason);
    bool verifyAppliedConfig(const TerrainCfg& cfg, std::string& reason);
    void requestSafeMode(const std::string& reason);
    void publishDiagnostics(uint8_t level, const std::string& message, const std::string& profile);
    void workerLoop();

    std::string terrain_profiles_file_;
    std::string terrain_node_name_;
    std::string last_applied_profile_;
    std::string diagnostics_topic_{"diagnostics"};
    std::map<std::string, TerrainCfg> profile_map_;
    int wait_service_ms_{120};
    int set_timeout_ms_{120};
    int retry_count_{2};
    int retry_interval_ms_{30};
    int readback_timeout_ms_{50};

    rclcpp::Subscription<rc26_interfaces::msg::NavSafetyState>::SharedPtr sub_;
    rclcpp::AsyncParametersClient::SharedPtr param_client_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diagnostics_;
    rclcpp::Client<rc26_interfaces::srv::SetNavMode>::SharedPtr nav_mode_client_;
    bool service_ready_{false};
    std::string pending_profile_;
    bool stop_worker_{false};
    std::mutex worker_mutex_;
    std::condition_variable worker_cv_;
    std::thread worker_thread_;
};

}  // namespace rc26_nav_mode_manager
