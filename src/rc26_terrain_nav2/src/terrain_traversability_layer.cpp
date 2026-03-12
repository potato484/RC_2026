#include "rc26_terrain_nav2/terrain_traversability_layer.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include "grid_map_ros/GridMapRosConverter.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace {

std::string normalizePolicy(std::string policy) {
    std::transform(policy.begin(), policy.end(), policy.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return policy;
}

void applyUnknownPolicy(nav2_costmap_2d::Costmap2D& master_grid,
                        unsigned int mx,
                        unsigned int my,
                        const std::string& policy) {
    if (policy == "keep") {
        return;
    }

    unsigned char target = nav2_costmap_2d::NO_INFORMATION;
    if (policy == "lethal") {
        target = nav2_costmap_2d::LETHAL_OBSTACLE;
    } else if (policy == "inscribed") {
        target = nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
    } else if (policy == "no_information") {
        target = nav2_costmap_2d::NO_INFORMATION;
    } else {
        return;
    }

    const unsigned char old_cost = master_grid.getCost(mx, my);
    if (target == nav2_costmap_2d::NO_INFORMATION || old_cost == nav2_costmap_2d::NO_INFORMATION) {
        master_grid.setCost(mx, my, target);
        return;
    }
    master_grid.setCost(mx, my, std::max(old_cost, target));
}

}  // namespace

namespace rc26_terrain_nav2 {

unsigned char mapTraversabilityToCost(float traversability,
                                      double lethal_threshold,
                                      double inscribed_threshold) {
    if (!std::isfinite(static_cast<double>(traversability))) {
        return nav2_costmap_2d::NO_INFORMATION;
    }

    const float t = std::clamp(traversability, 0.0f, 1.0f);
    if (t <= static_cast<float>(lethal_threshold)) {
        return nav2_costmap_2d::LETHAL_OBSTACLE;
    }
    if (t <= static_cast<float>(inscribed_threshold)) {
        return nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
    }

    const int scaled = static_cast<int>(std::lround((1.0f - t) * 252.0f));
    return static_cast<unsigned char>(std::clamp(scaled, 0, 252));
}

TerrainTraversabilityLayer::TerrainTraversabilityLayer() = default;

void TerrainTraversabilityLayer::onInitialize() {
    auto node = node_.lock();
    if (!node) {
        throw std::runtime_error("TerrainTraversabilityLayer failed to lock lifecycle node");
    }

    declareParameter("enabled", rclcpp::ParameterValue(true));
    declareParameter("terrain_grid_topic", rclcpp::ParameterValue(terrain_grid_topic_));
    declareParameter("traversability_layer", rclcpp::ParameterValue(traversability_layer_));
    declareParameter("fresh_layer", rclcpp::ParameterValue(fresh_layer_));
    declareParameter("drop_layer", rclcpp::ParameterValue(drop_layer_));
    declareParameter("climbable_layer", rclcpp::ParameterValue(climbable_layer_));
    declareParameter("lethal_threshold", rclcpp::ParameterValue(lethal_threshold_));
    declareParameter("inscribed_threshold", rclcpp::ParameterValue(inscribed_threshold_));
    declareParameter("drop_lethal_threshold", rclcpp::ParameterValue(drop_lethal_threshold_));
    declareParameter("climbable_soft_cost_max", rclcpp::ParameterValue(climbable_soft_cost_max_));
    declareParameter("unknown_policy", rclcpp::ParameterValue(unknown_policy_));

    node->get_parameter(getFullName("enabled"), enabled_);
    node->get_parameter(getFullName("terrain_grid_topic"), terrain_grid_topic_);
    node->get_parameter(getFullName("traversability_layer"), traversability_layer_);
    node->get_parameter(getFullName("fresh_layer"), fresh_layer_);
    node->get_parameter(getFullName("drop_layer"), drop_layer_);
    node->get_parameter(getFullName("climbable_layer"), climbable_layer_);
    node->get_parameter(getFullName("lethal_threshold"), lethal_threshold_);
    node->get_parameter(getFullName("inscribed_threshold"), inscribed_threshold_);
    node->get_parameter(getFullName("drop_lethal_threshold"), drop_lethal_threshold_);
    node->get_parameter(getFullName("climbable_soft_cost_max"), climbable_soft_cost_max_);
    node->get_parameter(getFullName("unknown_policy"), unknown_policy_);

    lethal_threshold_ = std::clamp(lethal_threshold_, 0.0, 1.0);
    inscribed_threshold_ = std::clamp(inscribed_threshold_, 0.0, 1.0);
    if (inscribed_threshold_ < lethal_threshold_) {
        std::swap(inscribed_threshold_, lethal_threshold_);
    }
    drop_lethal_threshold_ = std::clamp(drop_lethal_threshold_, 0.0, 1.0);
    climbable_soft_cost_max_ = std::clamp(climbable_soft_cost_max_, 0.0, 252.0);
    unknown_policy_ = normalizePolicy(unknown_policy_);
    if (unknown_policy_ != "keep" && unknown_policy_ != "lethal" &&
        unknown_policy_ != "inscribed" && unknown_policy_ != "no_information") {
        RCLCPP_WARN(
            logger_,
            "Unknown unknown_policy '%s', fallback to 'keep'",
            unknown_policy_.c_str());
        unknown_policy_ = "keep";
    }

    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.reliable();
    qos.transient_local();
    terrain_sub_ = node->create_subscription<grid_map_msgs::msg::GridMap>(
        terrain_grid_topic_,
        qos,
        std::bind(&TerrainTraversabilityLayer::terrainGridCallback, this, std::placeholders::_1));

    current_ = false;
    RCLCPP_INFO(
        logger_,
        "TerrainTraversabilityLayer initialized: topic=%s traversability_layer=%s fresh_layer=%s "
        "drop_layer=%s climbable_layer=%s lethal_threshold=%.3f inscribed_threshold=%.3f "
        "climbable_soft_cost_max=%.1f unknown_policy=%s",
        terrain_grid_topic_.c_str(),
        traversability_layer_.c_str(),
        fresh_layer_.c_str(),
        drop_layer_.c_str(),
        climbable_layer_.c_str(),
        lethal_threshold_,
        inscribed_threshold_,
        climbable_soft_cost_max_,
        unknown_policy_.c_str());
}

void TerrainTraversabilityLayer::terrainGridCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg) {
    if (!msg) {
        return;
    }

    grid_map::GridMap converted;
    if (!grid_map::GridMapRosConverter::fromMessage(*msg, converted)) {
        RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "Failed to convert terrain grid map message");
        current_ = false;
        return;
    }

    std::lock_guard<std::mutex> lock(terrain_mutex_);
    terrain_map_ = std::make_shared<grid_map::GridMap>(std::move(converted));
    last_terrain_stamp_ = rclcpp::Time(msg->header.stamp);
    current_ = true;
}

void TerrainTraversabilityLayer::updateBounds(double, double, double,
                                              double* min_x, double* min_y,
                                              double* max_x, double* max_y) {
    if (!enabled_ || !layered_costmap_) {
        return;
    }

    auto* costmap = layered_costmap_->getCostmap();
    if (!costmap) {
        return;
    }

    const double origin_x = costmap->getOriginX();
    const double origin_y = costmap->getOriginY();
    const double max_x_world =
        origin_x + static_cast<double>(costmap->getSizeInCellsX()) * costmap->getResolution();
    const double max_y_world =
        origin_y + static_cast<double>(costmap->getSizeInCellsY()) * costmap->getResolution();

    *min_x = std::min(*min_x, origin_x);
    *min_y = std::min(*min_y, origin_y);
    *max_x = std::max(*max_x, max_x_world);
    *max_y = std::max(*max_y, max_y_world);
}

void TerrainTraversabilityLayer::updateCosts(nav2_costmap_2d::Costmap2D& master_grid,
                                             int min_i, int min_j, int max_i, int max_j) {
    if (!enabled_) {
        return;
    }

    std::shared_ptr<grid_map::GridMap> terrain_map;
    {
        std::lock_guard<std::mutex> lock(terrain_mutex_);
        terrain_map = terrain_map_;
    }

    if (!terrain_map) {
        current_ = false;
        return;
    }

    const std::string global_frame = layered_costmap_ ? layered_costmap_->getGlobalFrameID() : "";
    if (!global_frame.empty() && terrain_map->getFrameId() != global_frame) {
        RCLCPP_WARN_THROTTLE(
            logger_, *clock_, 3000,
            "TerrainTraversabilityLayer frame mismatch: terrain=%s costmap=%s",
            terrain_map->getFrameId().c_str(), global_frame.c_str());
        current_ = false;
        return;
    }

    const int clamped_min_i = std::max(0, min_i);
    const int clamped_min_j = std::max(0, min_j);
    const int clamped_max_i = std::min(static_cast<int>(master_grid.getSizeInCellsX()), max_i);
    const int clamped_max_j = std::min(static_cast<int>(master_grid.getSizeInCellsY()), max_j);

    for (int j = clamped_min_j; j < clamped_max_j; ++j) {
        for (int i = clamped_min_i; i < clamped_max_i; ++i) {
            double wx = 0.0;
            double wy = 0.0;
            master_grid.mapToWorld(static_cast<unsigned int>(i), static_cast<unsigned int>(j), wx, wy);
            const grid_map::Position pos(wx, wy);

            float fresh = 0.0f;
            if (!readLayerValue(*terrain_map, fresh_layer_, pos, fresh) || fresh < 0.5f) {
                applyUnknownPolicy(master_grid, static_cast<unsigned int>(i), static_cast<unsigned int>(j), unknown_policy_);
                continue;
            }

            float drop_prob = 0.0f;
            if (!drop_layer_.empty() && readLayerValue(*terrain_map, drop_layer_, pos, drop_prob) &&
                drop_prob >= static_cast<float>(drop_lethal_threshold_)) {
                const unsigned int mx = static_cast<unsigned int>(i);
                const unsigned int my = static_cast<unsigned int>(j);
                const unsigned char old_cost = master_grid.getCost(mx, my);
                const unsigned char target = nav2_costmap_2d::LETHAL_OBSTACLE;
                master_grid.setCost(
                    mx, my,
                    old_cost == nav2_costmap_2d::NO_INFORMATION ? target : std::max(old_cost, target));
                continue;
            }

            float traversability = 0.0f;
            if (!readLayerValue(*terrain_map, traversability_layer_, pos, traversability)) {
                applyUnknownPolicy(master_grid, static_cast<unsigned int>(i), static_cast<unsigned int>(j), unknown_policy_);
                continue;
            }

            unsigned char target_cost =
                mapTraversabilityToCost(traversability, lethal_threshold_, inscribed_threshold_);
            if (target_cost == nav2_costmap_2d::NO_INFORMATION) {
                applyUnknownPolicy(master_grid, static_cast<unsigned int>(i), static_cast<unsigned int>(j), unknown_policy_);
                continue;
            }

            if (!climbable_layer_.empty() &&
                target_cost < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
                float climbable_prob = 0.0f;
                if (readLayerValue(*terrain_map, climbable_layer_, pos, climbable_prob)) {
                    const int penalty = static_cast<int>(std::lround(
                        std::clamp(climbable_prob, 0.0f, 1.0f) * climbable_soft_cost_max_));
                    const int adjusted_cost = std::clamp(
                        static_cast<int>(target_cost) + penalty, 0, 252);
                    target_cost = static_cast<unsigned char>(adjusted_cost);
                }
            }

            const unsigned int mx = static_cast<unsigned int>(i);
            const unsigned int my = static_cast<unsigned int>(j);
            const unsigned char old_cost = master_grid.getCost(mx, my);
            if (old_cost == nav2_costmap_2d::NO_INFORMATION) {
                master_grid.setCost(mx, my, target_cost);
            } else {
                master_grid.setCost(mx, my, std::max(old_cost, target_cost));
            }
        }
    }

    current_ = true;
}

bool TerrainTraversabilityLayer::readLayerValue(const grid_map::GridMap& map,
                                                const std::string& layer,
                                                const grid_map::Position& pos,
                                                float& value) const {
    if (!map.exists(layer)) {
        return false;
    }

    grid_map::Index index;
    if (!map.getIndex(pos, index)) {
        return false;
    }
    if (!map.isValid(index, layer)) {
        return false;
    }

    value = map.at(layer, index);
    return std::isfinite(static_cast<double>(value));
}

void TerrainTraversabilityLayer::reset() {
    current_ = false;
}

bool TerrainTraversabilityLayer::isClearable() {
    return false;
}

void TerrainTraversabilityLayer::matchSize() {
    // No internal costmap buffer; this layer writes directly into the master grid.
}

}  // namespace rc26_terrain_nav2

PLUGINLIB_EXPORT_CLASS(rc26_terrain_nav2::TerrainTraversabilityLayer, nav2_costmap_2d::Layer)
