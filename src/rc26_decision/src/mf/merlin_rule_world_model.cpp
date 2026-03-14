#include "rc26_decision/mf/merlin_rule_world_model.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>

#include "grid_map_core/iterators/CircleIterator.hpp"
#include "grid_map_ros/GridMapRosConverter.hpp"
#include "tf2/exceptions.h"

namespace rc26_decision {

MerlinRuleWorldModel::MerlinRuleWorldModel(rclcpp::Node &node) : node_(node) {
  node_.declare_parameter<std::string>(
      "merlin_rule_world_model.terrain_grid_topic", terrain_grid_topic_);
  node_.declare_parameter<std::string>("merlin_rule_world_model.base_frame",
                                       base_frame_);
  node_.declare_parameter<std::string>("merlin_rule_world_model.block_id_layer",
                                       block_id_layer_);
  node_.declare_parameter<std::string>(
      "merlin_rule_world_model.block_occupied_layer", block_occupied_layer_);
  node_.declare_parameter<std::string>("merlin_rule_world_model.keepout_layer",
                                       keepout_layer_);
  node_.declare_parameter<std::string>(
      "merlin_rule_world_model.traversable_edge_layer",
      traversable_edge_layer_);
  node_.declare_parameter<std::string>(
      "merlin_rule_world_model.rule_legality_layer", rule_legality_layer_);
  node_.declare_parameter<std::string>(
      "merlin_rule_world_model.traversability_layer", traversability_layer_);
  node_.declare_parameter<std::string>("merlin_rule_world_model.fresh_layer",
                                       fresh_layer_);
  node_.declare_parameter<double>("merlin_rule_world_model.tf_timeout_sec",
                                  tf_timeout_sec_);
  node_.declare_parameter<double>(
      "merlin_rule_world_model.map_stale_timeout_sec", map_stale_timeout_sec_);
  node_.declare_parameter<double>("merlin_rule_world_model.footprint_radius_m",
                                  footprint_radius_m_);

  node_.get_parameter("merlin_rule_world_model.terrain_grid_topic",
                      terrain_grid_topic_);
  node_.get_parameter("merlin_rule_world_model.base_frame", base_frame_);
  node_.get_parameter("merlin_rule_world_model.block_id_layer",
                      block_id_layer_);
  node_.get_parameter("merlin_rule_world_model.block_occupied_layer",
                      block_occupied_layer_);
  node_.get_parameter("merlin_rule_world_model.keepout_layer", keepout_layer_);
  node_.get_parameter("merlin_rule_world_model.traversable_edge_layer",
                      traversable_edge_layer_);
  node_.get_parameter("merlin_rule_world_model.rule_legality_layer",
                      rule_legality_layer_);
  node_.get_parameter("merlin_rule_world_model.traversability_layer",
                      traversability_layer_);
  node_.get_parameter("merlin_rule_world_model.fresh_layer", fresh_layer_);
  node_.get_parameter("merlin_rule_world_model.tf_timeout_sec",
                      tf_timeout_sec_);
  node_.get_parameter("merlin_rule_world_model.map_stale_timeout_sec",
                      map_stale_timeout_sec_);
  node_.get_parameter("merlin_rule_world_model.footprint_radius_m",
                      footprint_radius_m_);
  tf_timeout_sec_ = std::max(0.01, tf_timeout_sec_);
  map_stale_timeout_sec_ = std::max(0.0, map_stale_timeout_sec_);
  footprint_radius_m_ = std::max(0.0, footprint_radius_m_);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_.get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.reliable();
  qos.transient_local();
  terrain_sub_ = node_.create_subscription<grid_map_msgs::msg::GridMap>(
      terrain_grid_topic_, qos,
      std::bind(&MerlinRuleWorldModel::terrainGridCallback, this,
                std::placeholders::_1));
}

bool MerlinRuleWorldModel::copyTerrainMap(
    std::shared_ptr<grid_map::GridMap> &out) const {
  std::lock_guard<std::mutex> lock(map_mutex_);
  out = terrain_map_;
  return static_cast<bool>(out);
}

bool MerlinRuleWorldModel::isReady() const {
  std::shared_ptr<grid_map::GridMap> map;
  if (!copyTerrainMap(map)) {
    return false;
  }
  if (map_stale_timeout_sec_ <= 0.0) {
    return true;
  }
  std::lock_guard<std::mutex> lock(map_mutex_);
  const double age_sec = (node_.get_clock()->now() - terrain_stamp_).seconds();
  return age_sec <= map_stale_timeout_sec_;
}

std::array<MerlinBlockSummary, 13> MerlinRuleWorldModel::buildSummaryCache(
    const grid_map::GridMap &map, const std::string &block_id_layer,
    const std::string &block_occupied_layer, const std::string &keepout_layer,
    const std::string &rule_legality_layer,
    const std::string &traversability_layer, const std::string &fresh_layer) {
  std::array<MerlinBlockSummary, 13> cache{};
  for (int block_id = 1; block_id <= 12; ++block_id) {
    auto &summary = cache[static_cast<size_t>(block_id)];
    summary.block_id = block_id;
    summary.min_rule_legality = 1.0F;
    summary.min_traversability = 1.0F;
  }

  if (!map.exists(block_id_layer)) {
    return cache;
  }

  const bool has_fresh = map.exists(fresh_layer);
  const bool has_occupied = map.exists(block_occupied_layer);
  const bool has_keepout = map.exists(keepout_layer);
  const bool has_rule = map.exists(rule_legality_layer);
  const bool has_traversability = map.exists(traversability_layer);

  for (int block_id = 1; block_id <= 12; ++block_id) {
    auto &summary = cache[static_cast<size_t>(block_id)];
    summary.occupied_layer_available = has_occupied;
    summary.rule_layer_available = has_rule;
  }

  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
    const grid_map::Index index(*it);
    if (has_fresh && map.isValid(index, fresh_layer) &&
        map.at(fresh_layer, index) < 0.5F) {
      continue;
    }
    if (!map.isValid(index, block_id_layer)) {
      continue;
    }
    const int block_id =
        static_cast<int>(std::lround(map.at(block_id_layer, index)));
    if (block_id < 1 || block_id > 12) {
      continue;
    }

    auto &summary = cache[static_cast<size_t>(block_id)];
    summary.sampled_cells++;

    if (has_occupied) {
      if (map.isValid(index, block_occupied_layer)) {
        if (map.at(block_occupied_layer, index) >= 0.5F) {
          summary.occupied_cells++;
        }
      } else {
        summary.unknown_occupied_cells++;
      }
    }
    if (has_keepout && map.isValid(index, keepout_layer) &&
        map.at(keepout_layer, index) >= 0.5F) {
      summary.keepout_cells++;
    }
    if (has_rule && map.isValid(index, rule_legality_layer)) {
      summary.min_rule_legality = std::min(summary.min_rule_legality,
                                           map.at(rule_legality_layer, index));
    }
    if (has_traversability && map.isValid(index, traversability_layer)) {
      summary.min_traversability = std::min(
          summary.min_traversability, map.at(traversability_layer, index));
    }
  }

  return cache;
}

void MerlinRuleWorldModel::terrainGridCallback(
    const grid_map_msgs::msg::GridMap::SharedPtr msg) {
  if (!msg) {
    return;
  }
  grid_map::GridMap converted;
  if (!grid_map::GridMapRosConverter::fromMessage(*msg, converted)) {
    RCLCPP_WARN_THROTTLE(
        node_.get_logger(), *node_.get_clock(), 2000,
        "MerlinRuleWorldModel failed to convert terrain_grid_map");
    return;
  }

  auto summary_cache = buildSummaryCache(
      converted, block_id_layer_, block_occupied_layer_, keepout_layer_,
      rule_legality_layer_, traversability_layer_, fresh_layer_);

  std::lock_guard<std::mutex> lock(map_mutex_);
  terrain_map_ = std::make_shared<grid_map::GridMap>(std::move(converted));
  summary_cache_ = std::move(summary_cache);
  terrain_stamp_ = rclcpp::Time(msg->header.stamp);
}

bool MerlinRuleWorldModel::resolveCurrentBlock(int &block_id,
                                               std::string *reason) const {
  block_id = -1;
  std::shared_ptr<grid_map::GridMap> map;
  if (!copyTerrainMap(map)) {
    if (reason) {
      *reason = "terrain_map_unavailable";
    }
    return false;
  }

  if (!map->exists(block_id_layer_)) {
    if (reason) {
      *reason = "block_id_layer_missing";
    }
    return false;
  }

  const std::string map_frame = map->getFrameId();
  geometry_msgs::msg::TransformStamped tf_map_base;
  try {
    tf_map_base = tf_buffer_->lookupTransform(
        map_frame, base_frame_, rclcpp::Time(0),
        rclcpp::Duration::from_seconds(tf_timeout_sec_));
  } catch (const tf2::TransformException &ex) {
    if (reason) {
      *reason = std::string("tf_lookup_failed: ") + ex.what();
    }
    return false;
  }

  const grid_map::Position base_pos(tf_map_base.transform.translation.x,
                                    tf_map_base.transform.translation.y);
  if (!map->isInside(base_pos)) {
    if (reason) {
      *reason = "base_out_of_grid_map";
    }
    return false;
  }

  std::array<int, 13> hit_count{};
  if (footprint_radius_m_ > 1e-4) {
    for (grid_map::CircleIterator it(*map, base_pos, footprint_radius_m_);
         !it.isPastEnd(); ++it) {
      const grid_map::Index index(*it);
      if (!map->isValid(index, block_id_layer_)) {
        continue;
      }
      const int id =
          static_cast<int>(std::lround(map->at(block_id_layer_, index)));
      if (id >= 1 && id <= 12) {
        hit_count[static_cast<size_t>(id)]++;
      }
    }
  }

  grid_map::Index index;
  if (!map->getIndex(base_pos, index)) {
    if (reason) {
      *reason = "base_out_of_grid_map";
    }
    return false;
  }
  if (map->isValid(index, block_id_layer_)) {
    const int id =
        static_cast<int>(std::lround(map->at(block_id_layer_, index)));
    if (id >= 1 && id <= 12) {
      hit_count[static_cast<size_t>(id)] += 2;
    }
  }

  const grid_map::Size size = map->getSize();
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      const grid_map::Index sample(index(0) + dx, index(1) + dy);
      if (sample(0) < 0 || sample(1) < 0 || sample(0) >= size(0) ||
          sample(1) >= size(1)) {
        continue;
      }
      if (!map->isValid(sample, block_id_layer_)) {
        continue;
      }
      const int id =
          static_cast<int>(std::lround(map->at(block_id_layer_, sample)));
      if (id >= 1 && id <= 12) {
        hit_count[static_cast<size_t>(id)]++;
      }
    }
  }

  int best_id = -1;
  int best_count = 0;
  for (int id = 1; id <= 12; ++id) {
    const int count = hit_count[static_cast<size_t>(id)];
    if (count > best_count) {
      best_count = count;
      best_id = id;
    }
  }
  if (best_id > 0) {
    block_id = best_id;
    return true;
  }
  if (reason) {
    *reason = "block_id_not_resolved";
  }
  return false;
}

MerlinBlockSummary MerlinRuleWorldModel::summarizeBlock(int block_id) const {
  if (block_id < 1 || block_id > 12) {
    MerlinBlockSummary summary;
    summary.block_id = block_id;
    return summary;
  }

  std::lock_guard<std::mutex> lock(map_mutex_);
  MerlinBlockSummary summary = summary_cache_[static_cast<size_t>(block_id)];
  summary.block_id = block_id;
  return summary;
}

TransitionVerdict MerlinRuleWorldModel::canMove(int from_block,
                                                int to_block) const {
  TransitionVerdict verdict;
  verdict.allowed = false;
  verdict.reason = "unknown";
  verdict.target_summary = summarizeBlock(to_block);

  if (from_block < 1 || from_block > 12 || to_block < 1 || to_block > 12) {
    verdict.reason = "invalid_block_id";
    return verdict;
  }
  if (!verdict.target_summary.hasSamples()) {
    verdict.reason = "target_no_samples";
    return verdict;
  }
  if (!verdict.target_summary.hasOccupiedLayer()) {
    verdict.reason = "target_occupancy_layer_missing";
    return verdict;
  }
  if (!verdict.target_summary.hasRuleLayer()) {
    verdict.reason = "target_rule_layer_missing";
    return verdict;
  }
  if (verdict.target_summary.hasUnknownOccupancy()) {
    verdict.reason = "target_occupancy_unknown";
    return verdict;
  }
  if (verdict.target_summary.isOccupied()) {
    verdict.reason = "target_occupied";
    return verdict;
  }
  if (verdict.target_summary.hasKeepout()) {
    verdict.reason = "target_keepout";
    return verdict;
  }
  if (verdict.target_summary.min_rule_legality <= 0.5F) {
    verdict.reason = "target_rule_illegal";
    return verdict;
  }

  if (from_block == to_block) {
    verdict.allowed = true;
    verdict.reason = "same_block";
    return verdict;
  }
  if (!areGridAdjacent(from_block, to_block)) {
    verdict.reason = "non_adjacent";
    return verdict;
  }

  std::shared_ptr<grid_map::GridMap> map;
  if (!copyTerrainMap(map)) {
    verdict.reason = "terrain_map_unavailable";
    return verdict;
  }
  if (!hasTraversableEdge(*map, from_block, to_block)) {
    verdict.reason = "edge_blocked";
    return verdict;
  }

  verdict.allowed = true;
  verdict.reason = "ok";
  return verdict;
}

std::vector<int> MerlinRuleWorldModel::legalGrabTargetsFromEntry(
    const std::vector<int> &candidates) const {
  std::vector<int> targets;
  targets.reserve(candidates.size());
  for (const int id : candidates) {
    const auto summary = summarizeBlock(id);
    if (!summary.hasSamples()) {
      continue;
    }
    if (!summary.hasOccupiedLayer() || !summary.hasRuleLayer()) {
      continue;
    }
    if (summary.hasUnknownOccupancy() || summary.isOccupied() ||
        summary.hasKeepout()) {
      continue;
    }
    if (summary.min_rule_legality <= 0.5F) {
      continue;
    }
    targets.push_back(id);
  }
  return targets;
}

bool MerlinRuleWorldModel::hasTraversableEdge(const grid_map::GridMap &map,
                                              int from_block,
                                              int to_block) const {
  if (!map.exists(block_id_layer_) || !map.exists(traversable_edge_layer_)) {
    return false;
  }
  const bool has_fresh = map.exists(fresh_layer_);
  const grid_map::Size size = map.getSize();
  const std::array<grid_map::Index, 4> offsets = {
      grid_map::Index(1, 0),
      grid_map::Index(-1, 0),
      grid_map::Index(0, 1),
      grid_map::Index(0, -1),
  };

  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
    const grid_map::Index index(*it);
    if (has_fresh && map.isValid(index, fresh_layer_) &&
        map.at(fresh_layer_, index) < 0.5F) {
      continue;
    }
    if (!map.isValid(index, block_id_layer_) ||
        !map.isValid(index, traversable_edge_layer_)) {
      continue;
    }
    const int center_block =
        static_cast<int>(std::lround(map.at(block_id_layer_, index)));
    if (center_block != from_block) {
      continue;
    }
    if (map.at(traversable_edge_layer_, index) < 0.5F) {
      continue;
    }
    for (const auto &offset : offsets) {
      const grid_map::Index neighbor = index + offset;
      if (neighbor(0) < 0 || neighbor(1) < 0 || neighbor(0) >= size(0) ||
          neighbor(1) >= size(1)) {
        continue;
      }
      if (!map.isValid(neighbor, block_id_layer_)) {
        continue;
      }
      const int nb_block =
          static_cast<int>(std::lround(map.at(block_id_layer_, neighbor)));
      if (nb_block == to_block) {
        return true;
      }
    }
  }
  return false;
}

bool MerlinRuleWorldModel::areGridAdjacent(int a, int b) {
  if (a < 1 || a > 12 || b < 1 || b > 12) {
    return false;
  }
  const int row_a = (a - 1) / 3;
  const int col_a = (a - 1) % 3;
  const int row_b = (b - 1) / 3;
  const int col_b = (b - 1) % 3;
  return std::abs(row_a - row_b) + std::abs(col_a - col_b) == 1;
}

} // namespace rc26_decision
