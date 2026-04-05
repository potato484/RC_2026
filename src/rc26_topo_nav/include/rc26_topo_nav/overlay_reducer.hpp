#pragma once

#include "rc26_topo_nav/types.hpp"

#include <cstdint>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rc26_interfaces/msg/localization_backend_status.hpp>
#include <rc26_interfaces/msg/localization_health.hpp>
#include <rc26_interfaces/msg/mf_block_overlay.hpp>
#include <rc26_interfaces/msg/route_observability.hpp>
#include <rc26_interfaces/msg/surface_graph_overlay.hpp>
#include <rc26_interfaces/msg/terrain_feature_grid.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>

namespace rc26_topo_nav {

enum class OverlayApplicationMode : uint8_t {
    STATIC_ONLY = 0,
    ALL = 1,
};

struct OverlaySnapshot {
    uint64_t version = 0;
    std::vector<std::string> active_dynamic_sources;
    std::unordered_map<std::string, std::string> dynamic_blocked_nodes;
    std::unordered_map<std::string, std::string> dynamic_blocked_edges;
};

class OverlayReducer {
public:
    struct DynamicSurfaceOverlayState {
        std::string source;
        bool has_expiry = false;
        int64_t expires_at_ns = 0;
        std::unordered_set<std::string> blocked_node_ids;
        std::unordered_set<std::string> blocked_edge_ids;
    };

    explicit OverlayReducer(rclcpp::Node* node);

    void applyOverlays(
        const FieldGraph& graph,
        std::unordered_map<std::string, NodeOverlay>& node_overlays,
        std::unordered_map<std::string, EdgeOverlay>& edge_overlays,
        OverlayApplicationMode mode = OverlayApplicationMode::ALL);

    bool shouldHold() const;
    uint64_t overlayVersion();
    OverlaySnapshot snapshot();

private:
    void onLocHealth(const rc26_interfaces::msg::LocalizationHealth::SharedPtr msg);
    void onLocBackend(const rc26_interfaces::msg::LocalizationBackendStatus::SharedPtr msg);
    void onRouteObs(const rc26_interfaces::msg::RouteObservability::SharedPtr msg);
    void onTerrainGrid(const rc26_interfaces::msg::TerrainFeatureGrid::SharedPtr msg);
    void onBlockOverlay(const rc26_interfaces::msg::MfBlockOverlay::SharedPtr msg);
    void onSurfaceGraphOverlay(const rc26_interfaces::msg::SurfaceGraphOverlay::SharedPtr msg);
    void onLevel(const std_msgs::msg::Int32::SharedPtr msg);
    void onStableTerrain(const std_msgs::msg::Bool::SharedPtr msg);
    void onStableOperation(const std_msgs::msg::Bool::SharedPtr msg);

    bool edgeHitsTerrainRisk(const FieldGraph& graph, const GraphEdge& edge) const;
    void pruneExpiredDynamicOverlaysLocked(int64_t now_ns);
    void bumpOverlayVersionLocked();
    static std::string toLowerCopy(std::string value);

    rclcpp::Node* node_;
    rclcpp::Subscription<rc26_interfaces::msg::LocalizationHealth>::SharedPtr loc_health_sub_;
    rclcpp::Subscription<rc26_interfaces::msg::LocalizationBackendStatus>::SharedPtr loc_backend_sub_;
    rclcpp::Subscription<rc26_interfaces::msg::RouteObservability>::SharedPtr route_obs_sub_;
    rclcpp::Subscription<rc26_interfaces::msg::TerrainFeatureGrid>::SharedPtr terrain_sub_;
    rclcpp::Subscription<rc26_interfaces::msg::MfBlockOverlay>::SharedPtr block_overlay_sub_;
    rclcpp::Subscription<rc26_interfaces::msg::SurfaceGraphOverlay>::SharedPtr surface_graph_overlay_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr level_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stable_terrain_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stable_operation_sub_;

    mutable std::mutex mu_;
    std::string expected_team_;
    uint8_t loc_health_level_ = 0;  // GREEN
    uint8_t route_obs_risk_ = 0;    // LOW
    int32_t current_level_ = 0;
    bool stable_terrain_ = false;
    bool stable_operation_ = false;
    std::unordered_map<int, float> block_confidence_;

    float terrain_resolution_m_ = 0.0F;
    uint32_t terrain_width_ = 0;
    uint32_t terrain_height_ = 0;
    double terrain_origin_x_ = 0.0;
    double terrain_origin_y_ = 0.0;
    std::vector<float> terrain_p_obstacle_;
    std::vector<float> terrain_p_drop_;
    uint64_t overlay_version_ = 1;
    std::unordered_map<std::string, DynamicSurfaceOverlayState> dynamic_surface_overlays_;
};

}  // namespace rc26_topo_nav
