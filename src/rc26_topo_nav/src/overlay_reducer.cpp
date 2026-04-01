#include "rc26_topo_nav/overlay_reducer.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>

namespace rc26_topo_nav {

OverlayReducer::OverlayReducer(rclcpp::Node* node) : node_(node) {
    expected_team_ = toLowerCopy(node_->get_parameter("team").as_string());

    loc_health_sub_ = node_->create_subscription<rc26_interfaces::msg::LocalizationHealth>(
        "/localization/health", 10,
        std::bind(&OverlayReducer::onLocHealth, this, std::placeholders::_1));

    loc_backend_sub_ = node_->create_subscription<rc26_interfaces::msg::LocalizationBackendStatus>(
        "/localization/backend_status", 10,
        std::bind(&OverlayReducer::onLocBackend, this, std::placeholders::_1));

    route_obs_sub_ = node_->create_subscription<rc26_interfaces::msg::RouteObservability>(
        "/localization/route_observability", 10,
        std::bind(&OverlayReducer::onRouteObs, this, std::placeholders::_1));

    terrain_sub_ = node_->create_subscription<rc26_interfaces::msg::TerrainFeatureGrid>(
        "terrain_features", 10,
        std::bind(&OverlayReducer::onTerrainGrid, this, std::placeholders::_1));

    block_overlay_sub_ = node_->create_subscription<rc26_interfaces::msg::MfBlockOverlay>(
        "/mf_block_overlay", 10,
        std::bind(&OverlayReducer::onBlockOverlay, this, std::placeholders::_1));

    level_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
        "base_ground/level", 10,
        std::bind(&OverlayReducer::onLevel, this, std::placeholders::_1));

    stable_terrain_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "base_ground/stable_terrain", 10,
        std::bind(&OverlayReducer::onStableTerrain, this, std::placeholders::_1));

    stable_operation_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "base_ground/stable_operation", 10,
        std::bind(&OverlayReducer::onStableOperation, this, std::placeholders::_1));
}

std::string OverlayReducer::toLowerCopy(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return value;
}

void OverlayReducer::onLocHealth(const rc26_interfaces::msg::LocalizationHealth::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mu_);
    loc_health_level_ = msg->level;
}

void OverlayReducer::onLocBackend(
    const rc26_interfaces::msg::LocalizationBackendStatus::SharedPtr /*msg*/) {
    // Reserved for future optimizer/backend-specific overlay rules.
}

void OverlayReducer::onRouteObs(const rc26_interfaces::msg::RouteObservability::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mu_);
    route_obs_risk_ = msg->risk_level;
}

void OverlayReducer::onTerrainGrid(
    const rc26_interfaces::msg::TerrainFeatureGrid::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mu_);
    terrain_resolution_m_ = msg->resolution_m;
    terrain_width_ = msg->width;
    terrain_height_ = msg->height;
    terrain_origin_x_ = msg->origin.position.x;
    terrain_origin_y_ = msg->origin.position.y;
    terrain_p_obstacle_ = msg->p_obstacle;
    terrain_p_drop_ = msg->p_drop;
}

void OverlayReducer::onBlockOverlay(
    const rc26_interfaces::msg::MfBlockOverlay::SharedPtr msg) {
    const std::string msg_team = toLowerCopy(msg->team);
    const bool has_team = !msg_team.empty();
    const bool accept_team =
        !has_team || msg_team == "shared" || expected_team_.empty() || msg_team == expected_team_;
    if (!accept_team) {
        return;
    }

    std::lock_guard<std::mutex> lock(mu_);
    block_confidence_.clear();
    for (const auto& cell : msg->cells) {
        float confidence = cell.confidence;
        if (cell.state == rc26_interfaces::msg::MfBlockOverlayCell::BLOCKED) {
            confidence = std::max(confidence, 1.0F);
        }
        block_confidence_[cell.grid_id] = confidence;
    }
}

void OverlayReducer::onLevel(const std_msgs::msg::Int32::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mu_);
    current_level_ = msg->data;
}

void OverlayReducer::onStableTerrain(const std_msgs::msg::Bool::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mu_);
    stable_terrain_ = msg->data;
}

void OverlayReducer::onStableOperation(const std_msgs::msg::Bool::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mu_);
    stable_operation_ = msg->data;
}

bool OverlayReducer::shouldHold() const {
    std::lock_guard<std::mutex> lock(mu_);
    return loc_health_level_ >= rc26_interfaces::msg::LocalizationHealth::RED;
}

bool OverlayReducer::edgeHitsTerrainRisk(const FieldGraph& graph, const GraphEdge& edge) const {
    if (terrain_resolution_m_ <= 0.0F || terrain_width_ == 0 || terrain_height_ == 0 ||
        terrain_p_obstacle_.empty() || terrain_p_drop_.empty()) {
        return false;
    }

    auto from_it = graph.nodes.find(edge.from);
    auto to_it = graph.nodes.find(edge.to);
    if (from_it == graph.nodes.end() || to_it == graph.nodes.end()) {
        return false;
    }

    std::vector<Pose3> waypoints;
    waypoints.push_back(from_it->second.pose);
    for (const auto& control_point : edge.control_points) {
        waypoints.push_back(control_point);
    }
    waypoints.push_back(to_it->second.pose);

    const auto inBounds = [this](int gx, int gy) {
        return gx >= 0 && gy >= 0 &&
               gx < static_cast<int>(terrain_width_) &&
               gy < static_cast<int>(terrain_height_);
    };

    for (size_t segment_index = 1; segment_index < waypoints.size(); ++segment_index) {
        const auto& from = waypoints[segment_index - 1];
        const auto& to = waypoints[segment_index];
        const double dx = to.x - from.x;
        const double dy = to.y - from.y;
        const double dist = std::hypot(dx, dy);
        const int samples = std::max(2, static_cast<int>(dist / 0.10) + 1);

        for (int i = 0; i <= samples; ++i) {
            const double t = static_cast<double>(i) / samples;
            const double x = from.x + t * dx;
            const double y = from.y + t * dy;
            const int gx = static_cast<int>(std::floor((x - terrain_origin_x_) / terrain_resolution_m_));
            const int gy = static_cast<int>(std::floor((y - terrain_origin_y_) / terrain_resolution_m_));
            if (!inBounds(gx, gy)) {
                continue;
            }

            const size_t flat_index = static_cast<size_t>(gy) * terrain_width_ + static_cast<size_t>(gx);
            if (flat_index >= terrain_p_obstacle_.size() || flat_index >= terrain_p_drop_.size()) {
                continue;
            }

            if (terrain_p_obstacle_[flat_index] >= 0.6F || terrain_p_drop_[flat_index] >= 0.8F) {
                return true;
            }
        }
    }

    return false;
}

void OverlayReducer::applyOverlays(
    const FieldGraph& graph,
    std::unordered_map<std::string, NodeOverlay>& node_overlays,
    std::unordered_map<std::string, EdgeOverlay>& edge_overlays) const {
    std::lock_guard<std::mutex> lock(mu_);

    node_overlays.clear();
    edge_overlays.clear();

    for (const auto& [id, _] : graph.nodes) {
        node_overlays[id] = NodeOverlay{NodeState::FREE, 0.0};
    }

    for (const auto& [grid_id, confidence] : block_confidence_) {
        if (confidence < 0.7F) {
            continue;
        }
        for (const auto& [node_id, node] : graph.nodes) {
            if (node.block_id == grid_id) {
                node_overlays[node_id].state = NodeState::BLOCKED;
                node_overlays[node_id].extra_cost = 1000.0;
            }
        }
    }

    const bool orange_or_high_obs =
        loc_health_level_ >= rc26_interfaces::msg::LocalizationHealth::ORANGE ||
        route_obs_risk_ >= rc26_interfaces::msg::RouteObservability::HIGH;

    for (const auto& edge : graph.edges) {
        EdgeOverlay overlay{EdgeState::ENABLED, 0.0};

        if (loc_health_level_ == rc26_interfaces::msg::LocalizationHealth::YELLOW) {
            overlay.state = EdgeState::SLOW_ONLY;
        }

        if (orange_or_high_obs &&
            (edge.motion_type == "ramp_up" || edge.motion_type == "ramp_down" ||
             edge.motion_type == "drop_risky")) {
            overlay.state = EdgeState::BLOCKED;
            overlay.extra_cost = 1000.0;
        }

        auto target_node_it = node_overlays.find(edge.to);
        if (target_node_it != node_overlays.end() &&
            target_node_it->second.state == NodeState::BLOCKED) {
            overlay.state = EdgeState::BLOCKED;
            overlay.extra_cost = 1000.0;
        }

        if (overlay.state != EdgeState::BLOCKED && edgeHitsTerrainRisk(graph, edge)) {
            overlay.state = EdgeState::BLOCKED;
            overlay.extra_cost = 1000.0;
        }

        edge_overlays[edge.id] = overlay;
    }
}

}  // namespace rc26_topo_nav
