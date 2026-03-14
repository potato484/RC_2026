#include "rc26_decision/navigation/waypoint_manager.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace rc26_decision {

namespace {

struct Vec2 {
  double x = 0.0;
  double y = 0.0;
};

Vec2 operator+(const Vec2 &a, const Vec2 &b) {
  return Vec2{a.x + b.x, a.y + b.y};
}
Vec2 operator-(const Vec2 &a, const Vec2 &b) {
  return Vec2{a.x - b.x, a.y - b.y};
}
Vec2 operator*(double s, const Vec2 &v) { return Vec2{s * v.x, s * v.y}; }

double dot(const Vec2 &a, const Vec2 &b) { return a.x * b.x + a.y * b.y; }
double norm(const Vec2 &v) { return std::sqrt(dot(v, v)); }

Vec2 normalize(const Vec2 &v, double *out_norm = nullptr) {
  const double n = norm(v);
  if (out_norm) {
    *out_norm = n;
  }
  if (n < 1e-6) {
    return Vec2{0.0, 0.0};
  }
  return (1.0 / n) * v;
}

bool isValidMerlinAnchors(const MerlinAnchors &a) {
  const Vec2 p1{a.block_1.x, a.block_1.y};
  const Vec2 p2{a.block_2.x, a.block_2.y};
  const Vec2 p4{a.block_4.x, a.block_4.y};
  return norm(p2 - p1) > 0.1 && norm(p4 - p1) > 0.1;
}

double getScalarDouble(const YAML::Node &node, const char *key) {
  if (!node || !node[key]) {
    throw std::runtime_error(std::string("missing key: ") + key);
  }
  return node[key].as<double>();
}

std::string getScalarStringOrEmpty(const YAML::Node &node, const char *key) {
  if (!node || !node[key]) {
    return "";
  }
  return node[key].as<std::string>();
}

std::string parseNavProfile(const YAML::Node &profile_node) {
  if (profile_node && profile_node.IsScalar()) {
    std::string s = profile_node.as<std::string>();
    if (!s.empty()) {
      return s;
    }
  }
  return "normal";
}

Pose2D parsePose2D(const YAML::Node &point_node) {
  const YAML::Node pose_node =
      point_node["pose"] ? point_node["pose"] : point_node;

  Pose2D pose;
  pose.x = getScalarDouble(pose_node, "x");
  pose.y = getScalarDouble(pose_node, "y");
  if (pose_node["theta"]) {
    pose.yaw = pose_node["theta"].as<double>();
  } else if (pose_node["yaw"]) {
    pose.yaw = pose_node["yaw"].as<double>();
  } else {
    pose.yaw = 0.0;
  }
  return pose;
}

NavTolerance parseTolerance(const YAML::Node &point_node) {
  NavTolerance tol;
  const YAML::Node t = point_node["tolerance"];
  if (!t) {
    return tol;
  }
  if (t["xy"]) {
    tol.xy_tolerance = t["xy"].as<double>();
  } else if (t["xy_tolerance"]) {
    tol.xy_tolerance = t["xy_tolerance"].as<double>();
  }

  if (t["yaw"]) {
    tol.yaw_tolerance = t["yaw"].as<double>();
  } else if (t["yaw_tolerance"]) {
    tol.yaw_tolerance = t["yaw_tolerance"].as<double>();
  }
  return tol;
}

std::unordered_map<std::string, double>
parsePayload(const YAML::Node &point_node) {
  std::unordered_map<std::string, double> payload;
  const YAML::Node p = point_node["payload"];
  if (!p || !p.IsMap()) {
    return payload;
  }

  for (auto it = p.begin(); it != p.end(); ++it) {
    const auto k = it->first.as<std::string>();
    const auto v = it->second.as<double>();
    payload.emplace(k, v);
  }
  return payload;
}

SmartWaypointSpec parseSmartWaypoint(const YAML::Node &point_node) {
  SmartWaypointSpec spec;
  spec.pose = parsePose2D(point_node);
  spec.strategy_tag = getScalarStringOrEmpty(point_node, "strategy_tag");
  spec.tolerance = parseTolerance(point_node);
  spec.nav_profile = parseNavProfile(point_node["nav_profile"]);
  spec.speed_profile = getScalarStringOrEmpty(point_node, "speed_profile");
  if (point_node["timeout_sec"]) {
    spec.timeout_sec = point_node["timeout_sec"].as<float>();
  }

  spec.payload = parsePayload(point_node);
  return spec;
}

MerlinAnchors parseMerlinAnchors(const YAML::Node &anchors_node) {
  MerlinAnchors a;
  const YAML::Node b1 = anchors_node["block_1"];
  const YAML::Node b2 = anchors_node["block_2"];
  const YAML::Node b4 = anchors_node["block_4"];
  if (!b1 || !b2 || !b4) {
    throw std::runtime_error(
        "merlin_config.anchors requires block_1, block_2, block_4");
  }

  a.block_1.x = getScalarDouble(b1, "x");
  a.block_1.y = getScalarDouble(b1, "y");
  a.block_2.x = getScalarDouble(b2, "x");
  a.block_2.y = getScalarDouble(b2, "y");
  a.block_4.x = getScalarDouble(b4, "x");
  a.block_4.y = getScalarDouble(b4, "y");
  return a;
}

MerlinParams parseMerlinParams(const YAML::Node &params_node) {
  MerlinParams p;
  if (!params_node) {
    return p;
  }
  if (params_node["safe_offset"]) {
    p.safe_offset = params_node["safe_offset"].as<double>();
  }
  if (params_node["jump_margin"]) {
    p.jump_margin = params_node["jump_margin"].as<double>();
  }
  return p;
}

MerlinGraph buildMerlinAdjacencyGraph() {
  MerlinGraph g;
  for (int r = 1; r <= 4; ++r) {
    for (int c = 1; c <= 3; ++c) {
      const int id = (r - 1) * 3 + c;
      std::vector<int> nb;
      if (r > 1) {
        nb.push_back(id - 3);
      }
      if (r < 4) {
        nb.push_back(id + 3);
      }
      if (c > 1) {
        nb.push_back(id - 1);
      }
      if (c < 3) {
        nb.push_back(id + 1);
      }
      g.adjacency.emplace(id, std::move(nb));
    }
  }
  return g;
}

bool generateMerlinPoints(
    const MerlinAnchors &anchors, const MerlinParams &params,
    std::unordered_map<std::string, SmartWaypointSpec> &out_points,
    rclcpp::Logger &logger) {
  if (!isValidMerlinAnchors(anchors)) {
    RCLCPP_WARN(logger, "Merlin anchors are degenerate (zero distance), "
                        "skipping topology generation");
    return false;
  }

  const Vec2 p1{anchors.block_1.x, anchors.block_1.y};
  const Vec2 p2{anchors.block_2.x, anchors.block_2.y};
  const Vec2 p4{anchors.block_4.x, anchors.block_4.y};

  const Vec2 vx_raw = p2 - p1;
  double dx = 0.0;
  const Vec2 vx = normalize(vx_raw, &dx);
  if (dx < 1e-6) {
    return false;
  }

  const Vec2 vy_raw = p4 - p1;
  const Vec2 vy_proj = vy_raw - dot(vy_raw, vx) * vx;
  double dy_proj = 0.0;
  Vec2 vy = normalize(vy_proj, &dy_proj);
  if (dy_proj < 1e-6) {
    return false;
  }
  double dy = dot(vy_raw, vy);
  if (dy < 0.0) {
    dy = -dy;
    vy = Vec2{-vy.x, -vy.y};
  }

  const double yaw_align = std::atan2(vx.y, vx.x);

  auto make_spec = [&](const Vec2 &p) {
    SmartWaypointSpec s;
    s.pose.x = p.x;
    s.pose.y = p.y;
    s.pose.yaw = yaw_align;
    s.nav_profile = "safe";
    s.speed_profile = "SLOW";
    s.timeout_sec = 10.0f;
    s.tolerance.xy_tolerance = 0.10;
    s.tolerance.yaw_tolerance = 0.25;
    return s;
  };

  for (int r = 1; r <= 4; ++r) {
    for (int c = 1; c <= 3; ++c) {
      const int id = (r - 1) * 3 + c;
      const Vec2 center = p1 + (static_cast<double>(c - 1) * dx) * vx +
                          (static_cast<double>(r - 1) * dy) * vy;
      const Vec2 stand = center - (params.safe_offset * vy);

      const std::string center_name =
          "mf_block_" + std::to_string(id) + "_center";
      const std::string stand_name =
          "mf_block_" + std::to_string(id) + "_stand";
      const std::string grid_name = "mf_grid_" + std::to_string(id);

      out_points.insert_or_assign(center_name, make_spec(center));
      out_points.insert_or_assign(stand_name, make_spec(stand));
      out_points.insert_or_assign(grid_name, make_spec(center));
    }
  }

  return true;
}

bool loadWorldLayoutYaml(const std::string &layout_path, YAML::Node &out_root,
                         std::string &resolved_path, std::string &err) {
  std::filesystem::path current(layout_path);
  if (current.is_relative()) {
    current = std::filesystem::current_path() / current;
  }
  current = current.lexically_normal();

  for (int hop = 0; hop < 4; ++hop) {
    YAML::Node root = YAML::LoadFile(current.string());
    if (root["world_layout_file"]) {
      std::filesystem::path next = root["world_layout_file"].as<std::string>();
      if (next.empty()) {
        err = "world_layout_file is empty";
        return false;
      }
      if (next.is_relative()) {
        next = current.parent_path() / next;
      }
      current = next.lexically_normal();
      continue;
    }
    out_root = root;
    resolved_path = current.string();
    return true;
  }
  err = "too many world_layout_file redirects";
  return false;
}

bool generateMerlinPointsFromWorldLayout(
    const std::string &layout_path,
    std::unordered_map<std::string, SmartWaypointSpec> &out_points,
    rclcpp::Logger &logger) {
  YAML::Node root;
  std::string resolved;
  std::string err;
  if (!loadWorldLayoutYaml(layout_path, root, resolved, err)) {
    RCLCPP_WARN(logger, "WaypointManager: world layout load failed (%s): %s",
                layout_path.c_str(), err.c_str());
    return false;
  }

  const YAML::Node blocks = root["blocks"];
  if (!blocks || !blocks.IsSequence()) {
    RCLCPP_WARN(logger,
                "WaypointManager: world layout missing blocks sequence: %s",
                resolved.c_str());
    return false;
  }

  std::array<Vec2, 13> centers{};
  std::array<bool, 13> valid{};
  for (const auto &block : blocks) {
    const int id = block["id"].as<int>();
    if (id < 1 || id > 12) {
      continue;
    }
    centers[static_cast<size_t>(id)] =
        Vec2{block["x"].as<double>(), block["y"].as<double>()};
    valid[static_cast<size_t>(id)] = true;
  }
  for (int id = 1; id <= 12; ++id) {
    if (!valid[static_cast<size_t>(id)]) {
      RCLCPP_WARN(logger,
                  "WaypointManager: world layout missing block id=%d (%s)", id,
                  resolved.c_str());
      return false;
    }
  }

  const YAML::Node meta = root["meta"];
  const double safe_offset =
      meta && meta["safe_offset"] ? meta["safe_offset"].as<double>() : 0.20;
  const double block_half_extent =
      meta && meta["block_half_extent_m"]
          ? meta["block_half_extent_m"].as<double>()
          : 0.60;

  Vec2 vx = normalize(centers[2] - centers[1]);
  if (norm(vx) < 1e-6) {
    vx = Vec2{1.0, 0.0};
  }
  Vec2 vy = normalize(centers[4] - centers[1]);
  if (norm(vy) < 1e-6) {
    vy = Vec2{-vx.y, vx.x};
  }
  const double yaw_align = std::atan2(vx.y, vx.x);
  const double yaw_forward = std::atan2(vy.y, vy.x);

  auto make_spec = [&](const Vec2 &p) {
    SmartWaypointSpec s;
    s.pose.x = p.x;
    s.pose.y = p.y;
    s.pose.yaw = yaw_align;
    s.nav_profile = "safe";
    s.speed_profile = "SLOW";
    s.timeout_sec = 10.0f;
    s.tolerance.xy_tolerance = 0.10;
    s.tolerance.yaw_tolerance = 0.25;
    return s;
  };

  auto parse_id_list =
      [&](const YAML::Node &node,
          const std::vector<int> &fallback) -> std::vector<int> {
    std::vector<int> ids;
    if (node && node.IsSequence()) {
      ids.reserve(node.size());
      for (const auto &value : node) {
        const int id = value.as<int>();
        if (id >= 1 && id <= 12 && valid[static_cast<size_t>(id)]) {
          ids.push_back(id);
        }
      }
    }
    if (ids.empty()) {
      ids = fallback;
    }
    return ids;
  };

  auto average_center = [&](const std::vector<int> &ids) {
    Vec2 sum{};
    int count = 0;
    for (const int id : ids) {
      if (id < 1 || id > 12 || !valid[static_cast<size_t>(id)]) {
        continue;
      }
      sum = sum + centers[static_cast<size_t>(id)];
      ++count;
    }
    if (count == 0) {
      return Vec2{};
    }
    return (1.0 / static_cast<double>(count)) * sum;
  };

  auto assign_pose_with_template =
      [&](const std::string &name, const Vec2 &p, double yaw,
          const std::string &default_tag,
          const std::string &default_nav_profile,
          const std::string &default_speed_profile, double default_xy_tol,
          double default_yaw_tol, float default_timeout_sec) {
        SmartWaypointSpec spec;
        if (const auto it = out_points.find(name); it != out_points.end()) {
          spec = it->second;
        } else {
          spec.nav_profile = default_nav_profile;
          spec.speed_profile = default_speed_profile;
          spec.timeout_sec = default_timeout_sec;
          spec.tolerance.xy_tolerance = default_xy_tol;
          spec.tolerance.yaw_tolerance = default_yaw_tol;
          spec.strategy_tag = default_tag;
        }

        if (spec.strategy_tag.empty()) {
          spec.strategy_tag = default_tag;
        }
        if (spec.nav_profile.empty()) {
          spec.nav_profile = default_nav_profile;
        }
        if (spec.speed_profile.empty()) {
          spec.speed_profile = default_speed_profile;
        }
        if (spec.timeout_sec <= 0.0f) {
          spec.timeout_sec = default_timeout_sec;
        }
        if (spec.tolerance.xy_tolerance <= 0.0) {
          spec.tolerance.xy_tolerance = default_xy_tol;
        }
        if (spec.tolerance.yaw_tolerance <= 0.0) {
          spec.tolerance.yaw_tolerance = default_yaw_tol;
        }

        spec.pose.x = p.x;
        spec.pose.y = p.y;
        spec.pose.yaw = yaw;
        out_points.insert_or_assign(name, std::move(spec));
      };

  for (int id = 1; id <= 12; ++id) {
    const Vec2 center = centers[static_cast<size_t>(id)];
    const Vec2 stand = center - (safe_offset * vy);
    out_points.insert_or_assign("mf_grid_" + std::to_string(id),
                                make_spec(center));
    out_points.insert_or_assign("mf_block_" + std::to_string(id) + "_center",
                                make_spec(center));
    out_points.insert_or_assign("mf_block_" + std::to_string(id) + "_stand",
                                make_spec(stand));
  }

  const auto entry_blocks = parse_id_list(root["entry_blocks"], {1, 2, 3});
  const auto exit_blocks = parse_id_list(root["exit_blocks"], {10, 11, 12});
  const Vec2 entry_center = average_center(entry_blocks);
  const Vec2 exit_center = average_center(exit_blocks);
  const Vec2 entry_pose = entry_center - ((block_half_extent + 0.10) * vy);
  const Vec2 entry_back_pose = entry_pose - (1.50 * vy);
  const Vec2 exit_pose = exit_center + ((block_half_extent + 1.00) * vy);

  assign_pose_with_template("mf_entry", entry_pose, yaw_forward, "TAG_MF_ENTRY",
                            "mf_safe", "SLOW", 0.05, 0.10, 15.0f);
  assign_pose_with_template("mf_entry_back", entry_back_pose, yaw_forward,
                            "TAG_MF_ENTRY_BACK", "normal", "MEDIUM", 0.10, 0.15,
                            10.0f);
  assign_pose_with_template("mf_exit", exit_pose, yaw_forward, "TAG_MF_EXIT",
                            "mf_exit", "SLOW", 0.10, 0.15, 20.0f);

  RCLCPP_INFO(logger,
              "WaypointManager: generated Merlin points from world layout: %s",
              resolved.c_str());
  return true;
}

} // namespace

bool WaypointManager::loadFromYamlFile(const std::string &yaml_path) {
  try {
    YAML::Node root = YAML::LoadFile(yaml_path);

    // Parse into temporaries first (M2 fix: don't clear until success)
    std::unordered_map<std::string, SmartWaypointSpec> new_points;
    std::optional<MerlinGraph> new_merlin_graph;

    const YAML::Node static_points = root["static_points"];
    if (static_points && static_points.IsMap()) {
      for (auto it = static_points.begin(); it != static_points.end(); ++it) {
        const auto name = it->first.as<std::string>();
        const YAML::Node &point_node = it->second;
        new_points.emplace(name, parseSmartWaypoint(point_node));
      }
    }

    const YAML::Node merlin = root["merlin_config"];
    if (merlin && merlin.IsMap()) {
      bool generated_from_world_layout = false;
      std::string world_layout_file;
      if (merlin["world_layout_file"] &&
          merlin["world_layout_file"].IsScalar()) {
        world_layout_file = merlin["world_layout_file"].as<std::string>();
      }
      if (world_layout_file.empty()) {
        try {
          world_layout_file =
              ament_index_cpp::get_package_share_directory("rc26_kfs_keepout") +
              "/config/r2_mf_world.yaml";
        } catch (const std::exception &) {
          world_layout_file.clear();
        }
      } else {
        std::filesystem::path path(world_layout_file);
        if (path.is_relative()) {
          const std::filesystem::path yaml_parent =
              std::filesystem::path(yaml_path).parent_path();
          world_layout_file = (yaml_parent / path).lexically_normal().string();
        }
      }
      if (!world_layout_file.empty()) {
        generated_from_world_layout = generateMerlinPointsFromWorldLayout(
            world_layout_file, new_points, logger_);
        if (generated_from_world_layout) {
          new_merlin_graph = buildMerlinAdjacencyGraph();
        }
      }

      const YAML::Node anchors_node = merlin["anchors"];
      const YAML::Node params_node = merlin["params"];

      if (!generated_from_world_layout && anchors_node &&
          anchors_node.IsMap() && anchors_node["block_1"] &&
          anchors_node["block_2"] && anchors_node["block_4"]) {
        const MerlinAnchors anchors = parseMerlinAnchors(anchors_node);
        const MerlinParams params = parseMerlinParams(params_node);
        if (generateMerlinPoints(anchors, params, new_points, logger_)) {
          new_merlin_graph = buildMerlinAdjacencyGraph();
        }
      }
    }

    // Swap only on complete success
    points_ = std::move(new_points);
    merlin_graph_ = std::move(new_merlin_graph);

    RCLCPP_INFO(logger_, "WaypointManager: loaded %zu points from %s",
                points_.size(), yaml_path.c_str());
    return true;

  } catch (const std::exception &ex) {
    RCLCPP_ERROR(logger_, "WaypointManager: failed to load YAML [%s]: %s",
                 yaml_path.c_str(), ex.what());
    return false;
  }
}

const SmartWaypointSpec *WaypointManager::find(const std::string &name) const {
  const auto it = points_.find(name);
  if (it == points_.end()) {
    return nullptr;
  }
  return &it->second;
}

std::vector<std::string> WaypointManager::listNames() const {
  std::vector<std::string> out;
  out.reserve(points_.size());
  for (const auto &kv : points_) {
    out.push_back(kv.first);
  }
  std::sort(out.begin(), out.end());
  return out;
}

const MerlinGraph &WaypointManager::merlinGraph() const {
  if (!merlin_graph_) {
    throw std::runtime_error("WaypointManager: merlin graph not available");
  }
  return *merlin_graph_;
}

void WaypointManager::injectPointsToBlackboard(
    const BT::Blackboard::Ptr &blackboard,
    const std::string &key_prefix) const {
  if (!blackboard) {
    return;
  }
  for (const auto &kv : points_) {
    blackboard->set(key_prefix + kv.first, kv.second);
  }
}

} // namespace rc26_decision
