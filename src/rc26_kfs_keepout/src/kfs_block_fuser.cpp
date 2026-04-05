#include "rc26_kfs_keepout/kfs_block_fuser.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <filesystem>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "yaml-cpp/yaml.h"

namespace rc26_kfs_keepout {

namespace {
double probToLogOdds(double p) {
    p = std::clamp(p, 1e-4, 1.0 - 1e-4);
    return std::log(p / (1.0 - p));
}
double logOddsToProb(double lo) { return 1.0 / (1.0 + std::exp(-lo)); }

std::string toLowerCopy(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return value;
}
}  // namespace

KfsBlockFuser::KfsBlockFuser(const rclcpp::NodeOptions& options)
    : Node("kfs_block_fuser", options) {
    this->declare_parameter<std::string>("kfs_state_topic",   "mf_kfs_state");
    this->declare_parameter<std::string>("mask_topic",        "/kfs_filter_mask");
    this->declare_parameter<std::string>("heartbeat_topic",   heartbeat_topic_);
    this->declare_parameter<std::string>("grid_layout_file",  "");
    this->declare_parameter<std::string>("diagnostics_topic", "diagnostics");
    this->declare_parameter<std::string>("force_release_topic", "/kfs_force_release_grid");
    this->declare_parameter<double>("min_confidence",    min_confidence_);
    this->declare_parameter<double>("inflate_radius_m",  inflate_radius_m_);
    this->declare_parameter<double>("map_resolution",    map_resolution_);
    this->declare_parameter<std::string>("keepout_shape", keepout_shape_);
    this->declare_parameter<double>("block_half_size_m", block_half_size_m_);
    this->declare_parameter<double>("keepout_margin_m", keepout_margin_m_);
    this->declare_parameter<double>("block_thresh",      block_thresh_);
    this->declare_parameter<double>("free_thresh",       free_thresh_);
    this->declare_parameter<double>("lo_hit",            lo_hit_block_);
    this->declare_parameter<double>("lo_hit_block",      lo_hit_block_);
    this->declare_parameter<double>("lo_hit_fake",       lo_hit_fake_);
    this->declare_parameter<double>("lo_miss",           lo_miss_);
    this->declare_parameter<double>("decay_rate",        decay_rate_);
    this->declare_parameter<double>("decay_target_prob", decay_target_prob_);
    this->declare_parameter<double>("ttl_sec",           ttl_sec_);
    this->declare_parameter<std::string>("ttl_mode",     ttl_mode_);
    this->declare_parameter<int>("dwell_cycles",         dwell_cycles_);
    this->declare_parameter<double>("grid_spacing_tolerance_m", grid_spacing_tolerance_m_);

    this->get_parameter("mask_topic",       mask_topic_);
    this->get_parameter("heartbeat_topic",  heartbeat_topic_);
    this->get_parameter("grid_layout_file", grid_layout_file_);
    this->get_parameter("diagnostics_topic", diagnostics_topic_);
    this->get_parameter("force_release_topic", force_release_topic_);
    this->get_parameter("min_confidence",   min_confidence_);
    this->get_parameter("inflate_radius_m", inflate_radius_m_);
    this->get_parameter("map_resolution",   map_resolution_);
    this->get_parameter("keepout_shape",    keepout_shape_);
    this->get_parameter("block_half_size_m", block_half_size_m_);
    this->get_parameter("keepout_margin_m", keepout_margin_m_);
    this->get_parameter("block_thresh",     block_thresh_);
    this->get_parameter("free_thresh",      free_thresh_);
    double lo_hit_alias = lo_hit_block_;
    this->get_parameter("lo_hit",           lo_hit_alias);
    this->get_parameter("lo_hit_block",     lo_hit_block_);
    this->get_parameter("lo_hit_fake",      lo_hit_fake_);
    this->get_parameter("lo_miss",          lo_miss_);
    this->get_parameter("decay_rate",       decay_rate_);
    this->get_parameter("decay_target_prob", decay_target_prob_);
    this->get_parameter("ttl_sec",          ttl_sec_);
    this->get_parameter("ttl_mode",         ttl_mode_);
    this->get_parameter("dwell_cycles",     dwell_cycles_);
    this->get_parameter("grid_spacing_tolerance_m", grid_spacing_tolerance_m_);
    const auto& overrides = this->get_node_options().parameter_overrides();
    const auto has_override = [&overrides](const std::string& name) {
        return std::any_of(
            overrides.begin(), overrides.end(),
            [&name](const rclcpp::Parameter& param) {
                return param.get_name() == name;
            });
    };
    if (!has_override("lo_hit_block") && has_override("lo_hit")) {
        lo_hit_block_ = lo_hit_alias;
    }

    keepout_shape_ = toLowerCopy(keepout_shape_);
    if (heartbeat_topic_.empty()) {
        heartbeat_topic_ = "/kfs_keepout_heartbeat";
    }
    if (keepout_shape_ != "square" && keepout_shape_ != "circle") {
        RCLCPP_WARN(this->get_logger(), "invalid keepout_shape=%s, fallback to square", keepout_shape_.c_str());
        keepout_shape_ = "square";
    }
    decay_target_prob_ = std::clamp(decay_target_prob_, 1e-3, 0.49);
    decay_rate_ = std::max(0.0, decay_rate_);
    ttl_sec_ = std::max(0.0, ttl_sec_);
    map_resolution_ = std::max(0.01, map_resolution_);
    block_half_size_m_ = std::max(0.05, block_half_size_m_);
    inflate_radius_m_ = std::max(0.05, inflate_radius_m_);
    keepout_margin_m_ = std::max(0.0, keepout_margin_m_);
    grid_spacing_tolerance_m_ = std::max(0.01, grid_spacing_tolerance_m_);
    dwell_cycles_ = std::max(1, dwell_cycles_);
    ttl_mode_ = toLowerCopy(ttl_mode_);
    if (ttl_mode_ != "hard" && ttl_mode_ != "soft") {
        RCLCPP_WARN(this->get_logger(), "invalid ttl_mode=%s, fallback to hard", ttl_mode_.c_str());
        ttl_mode_ = "hard";
    }
    if (free_thresh_ >= block_thresh_) {
        free_thresh_ = std::max(0.01, block_thresh_ - 0.05);
    }
    if (decay_target_prob_ >= free_thresh_) {
        decay_target_prob_ = std::max(0.01, free_thresh_ - 0.05);
    }

    std::string kfs_topic;
    this->get_parameter("kfs_state_topic", kfs_topic);

    log_odds_.fill(probToLogOdds(0.5));  // prior = 0.5
    blocked_state_.fill(0);
    pending_state_.fill(0);
    dwell_count_.fill(0);
    cell_x_.fill(0.0);
    cell_y_.fill(0.0);
    for (auto& t : last_hit_time_) {
        t = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    }

    if (grid_layout_file_.empty()) {
        try {
            const auto share_dir = ament_index_cpp::get_package_share_directory("rc26_kfs_keepout");
            grid_layout_file_ = share_dir + "/config/r2_mf_world.yaml";
        } catch (const std::exception& ex) {
            keepout_disable_reason_ = std::string("cannot resolve default grid layout: ") + ex.what();
        }
    }
    if (!grid_layout_file_.empty()) {
        layout_loaded_ = loadGridLayout(grid_layout_file_);
        keepout_enabled_ = layout_loaded_;
        if (!keepout_enabled_ && keepout_disable_reason_.empty()) {
            keepout_disable_reason_ = "invalid grid layout";
        }
    } else {
        keepout_disable_reason_ = "grid_layout_file is empty";
        keepout_enabled_ = false;
    }

    if (!keepout_enabled_) {
        RCLCPP_ERROR(this->get_logger(), "keepout disabled: %s", keepout_disable_reason_.c_str());
    }

    auto mask_qos = rclcpp::QoS(rclcpp::KeepLast(1))
                        .reliable()
                        .durability(rclcpp::DurabilityPolicy::TransientLocal);
    pub_mask_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(mask_topic_, mask_qos);
    pub_diagnostics_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        diagnostics_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    pub_heartbeat_ = this->create_publisher<std_msgs::msg::Bool>(
        heartbeat_topic_,
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
    pub_block_overlay_ = this->create_publisher<rc26_interfaces::msg::MfBlockOverlay>(
        "/mf_block_overlay",
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability(rclcpp::DurabilityPolicy::TransientLocal));

    sub_ = this->create_subscription<rc26_interfaces::msg::MfKfsState>(
        kfs_topic, rclcpp::QoS(10).reliable(),
        std::bind(&KfsBlockFuser::onKfsState, this, std::placeholders::_1));
    sub_force_release_ = this->create_subscription<std_msgs::msg::UInt8>(
        force_release_topic_, rclcpp::QoS(10).reliable(),
        std::bind(&KfsBlockFuser::onForceReleaseGrid, this, std::placeholders::_1));

    last_decay_time_ = this->get_clock()->now();
    decay_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&KfsBlockFuser::decayTimer, this));

    RCLCPP_INFO(this->get_logger(), "KfsBlockFuser started, keepout_enabled=%s",
                keepout_enabled_ ? "true" : "false");
    if (keepout_enabled_) {
        publishMask();
    }
    publishDiagnostics();
    publishHeartbeat();
}

bool KfsBlockFuser::loadGridLayout(const std::string& path) {
    try {
        std::filesystem::path resolved(path);
        if (resolved.is_relative()) {
            resolved = std::filesystem::current_path() / resolved;
        }
        YAML::Node root = YAML::LoadFile(resolved.string());
        if (root["world_layout_file"]) {
            std::filesystem::path nested = root["world_layout_file"].as<std::string>();
            if (nested.empty()) {
                keepout_disable_reason_ = "world_layout_file is empty";
                return false;
            }
            if (nested.is_relative()) {
                nested = resolved.parent_path() / nested;
            }
            return loadGridLayout(nested.string());
        }
        if (!root["meta"]) {
            keepout_disable_reason_ = "missing 'meta' key";
            return false;
        }
        const auto meta = root["meta"];
        if (!meta["team"] || !meta["grid_spacing_m"]) {
            keepout_disable_reason_ = "meta requires team/grid_spacing_m";
            return false;
        }
        layout_team_ = meta["team"].as<std::string>();
        active_team_ = layout_team_;
        layout_version_ = meta["layout_version"] ? meta["layout_version"].as<std::string>() : "unknown";
        layout_validated_ = meta["validated"] ? meta["validated"].as<bool>() : true;
        layout_grid_spacing_m_ = meta["grid_spacing_m"].as<double>();
        if (layout_team_.empty()) {
            keepout_disable_reason_ = "layout team is empty";
            return false;
        }
        if (!layout_validated_) {
            keepout_disable_reason_ = "layout validated=false";
            return false;
        }
        if (layout_grid_spacing_m_ <= 0.0) {
            keepout_disable_reason_ = "layout grid_spacing_m must be > 0";
            return false;
        }

        const bool use_blocks = root["blocks"] && root["blocks"].IsSequence();
        const YAML::Node cells_node = use_blocks ? root["blocks"] : root["grids"];
        if (!cells_node) {
            keepout_disable_reason_ = "missing blocks/grids key";
            return false;
        }
        std::array<bool, kGridCount> seen{};
        for (const auto& g : cells_node) {
            int id = g["id"].as<int>();
            if (id < 1 || id > 12) {
                keepout_disable_reason_ = "grid id out of range";
                return false;
            }
            if (seen[static_cast<size_t>(id)]) {
                keepout_disable_reason_ = "duplicate grid id: " + std::to_string(id);
                return false;
            }
            const double x = g["x"].as<double>();
            const double y = g["y"].as<double>();
            if (!std::isfinite(x) || !std::isfinite(y)) {
                keepout_disable_reason_ = "grid coordinates must be finite";
                return false;
            }
            cell_x_[static_cast<size_t>(id)] = x;
            cell_y_[static_cast<size_t>(id)] = y;
            seen[static_cast<size_t>(id)] = true;
        }
        for (int id = 1; id <= 12; id++) {
            if (!seen[static_cast<size_t>(id)]) {
                keepout_disable_reason_ = "grid layout missing ids 1..12";
                return false;
            }
        }
        std::string spacing_detail;
        if (!validateGridSpacing(layout_grid_spacing_m_, grid_spacing_tolerance_m_, spacing_detail)) {
            keepout_disable_reason_ = spacing_detail;
            return false;
        }
        return true;
    } catch (const std::exception& e) {
        keepout_disable_reason_ = std::string("yaml parse error: ") + e.what();
        RCLCPP_ERROR(this->get_logger(), "loadGridLayout failed: %s", e.what());
        return false;
    }
}

bool KfsBlockFuser::validateGridSpacing(double expected_spacing_m, double tolerance_m, std::string& detail) const {
    auto dist = [&](int a, int b) -> double {
        const double dx = cell_x_[static_cast<size_t>(a)] - cell_x_[static_cast<size_t>(b)];
        const double dy = cell_y_[static_cast<size_t>(a)] - cell_y_[static_cast<size_t>(b)];
        return std::hypot(dx, dy);
    };

    for (int id = 1; id <= 12; ++id) {
        const bool has_right = (id % 3 != 0);
        const bool has_front = (id <= 9);
        if (has_right) {
            const double d = dist(id, id + 1);
            if (std::abs(d - expected_spacing_m) > tolerance_m) {
                detail = "grid spacing check failed for " + std::to_string(id) + "-" + std::to_string(id + 1);
                return false;
            }
        }
        if (has_front) {
            const double d = dist(id, id + 3);
            if (std::abs(d - expected_spacing_m) > tolerance_m) {
                detail = "grid spacing check failed for " + std::to_string(id) + "-" + std::to_string(id + 3);
                return false;
            }
        }
    }
    return true;
}

void KfsBlockFuser::onKfsState(
    const rc26_interfaces::msg::MfKfsState::ConstSharedPtr& msg) {
    if (!msg) {
        return;
    }
    if (!layout_team_.empty() && toLowerCopy(layout_team_) != "shared") {
        const std::string msg_team = toLowerCopy(msg->team);
        const std::string expected_team = toLowerCopy(layout_team_);
        if (msg_team.empty() || msg_team != expected_team) {
            const bool first_mismatch = !team_mismatch_detected_;
            keepout_enabled_ = false;
            team_mismatch_detected_ = true;
            keepout_disable_reason_ = "team mismatch: layout=" + layout_team_ +
                                      ", msg=" + (msg->team.empty() ? std::string("<empty>") : msg->team);
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                  "%s", keepout_disable_reason_.c_str());
            if (first_mismatch) {
                log_odds_.fill(probToLogOdds(0.5));
                blocked_state_.fill(0);
                pending_state_.fill(0);
                dwell_count_.fill(0);
                mask_dirty_ = false;
                publishMask();
                RCLCPP_WARN(this->get_logger(),
                            "keepout disabled due to layout/team mismatch; downstream gate must handle degraded navigation safety");
            }
            publishDiagnostics();
            publishHeartbeat();
            return;
        }
        if (team_mismatch_detected_) {
            team_mismatch_detected_ = false;
            keepout_enabled_ = layout_loaded_;
            if (keepout_enabled_) {
                keepout_disable_reason_.clear();
            } else if (keepout_disable_reason_.empty()) {
                keepout_disable_reason_ = "invalid grid layout";
            }
            RCLCPP_INFO(this->get_logger(),
                        "team match restored, keepout re-enabled=%s",
                        keepout_enabled_ ? "true" : "false");
            if (keepout_enabled_) {
                publishMask();
            }
            publishDiagnostics();
            publishHeartbeat();
        }
    }
    if (!msg->team.empty()) {
        active_team_ = msg->team;
    }
    if (!keepout_enabled_) {
        return;
    }

    const auto now = this->get_clock()->now();
    for (const auto& cell : msg->cells) {
        if (cell.grid_id < 1 || cell.grid_id > 12) {
            continue;
        }
        if (cell.confidence < static_cast<float>(min_confidence_)) {
            continue;
        }

        const size_t idx = static_cast<size_t>(cell.grid_id);
        auto& lo = log_odds_[idx];
        switch (cell.kfs_type) {
        case 1U:  // R1
        case 2U:  // R2
            lo = std::clamp(lo + lo_hit_block_, -8.0, 8.0);
            last_hit_time_[idx] = now;
            break;
        case 3U:  // FAKE
            lo = std::clamp(lo + lo_hit_fake_, -8.0, 8.0);
            last_hit_time_[idx] = now;
            break;
        case 0U:  // NONE
            lo = std::clamp(lo + lo_miss_, -8.0, 8.0);
            break;
        default:  // UNKNOWN(4) and others
            continue;
        }

        const double p = logOddsToProb(lo);
        if (p >= block_thresh_ && blocked_state_[idx] == 0) {
            blocked_state_[idx] = 1;
            pending_state_[idx] = 1;
            dwell_count_[idx] = dwell_cycles_;
            mask_dirty_ = true;
        }
    }
    if (mask_dirty_ && keepout_enabled_) {
        publishMask();
        mask_dirty_ = false;
    }
}

void KfsBlockFuser::onForceReleaseGrid(const std_msgs::msg::UInt8::ConstSharedPtr& msg) {
    if (!msg) {
        return;
    }
    const int grid_id = static_cast<int>(msg->data);
    if (grid_id < 1 || grid_id > 12) {
        RCLCPP_WARN(this->get_logger(), "force release ignored: invalid grid_id=%d", grid_id);
        return;
    }
    const size_t idx = static_cast<size_t>(grid_id);
    log_odds_[idx] = probToLogOdds(decay_target_prob_);
    blocked_state_[idx] = 0;
    pending_state_[idx] = 0;
    dwell_count_[idx] = dwell_cycles_;
    mask_dirty_ = true;
    last_hit_time_[idx] = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    force_release_count_++;
    if (mask_dirty_ && keepout_enabled_) {
        publishMask();
        mask_dirty_ = false;
    }
    publishDiagnostics();
}

void KfsBlockFuser::decayTimer() {
    const auto now = this->get_clock()->now();
    const double dt = std::max(0.0, (now - last_decay_time_).seconds());
    last_decay_time_ = now;

    const double lo_decay = decay_rate_ * dt;
    const double lo_target = probToLogOdds(decay_target_prob_);
    for (int i = 1; i <= 12; i++) {
        const size_t idx = static_cast<size_t>(i);
        auto& lo = log_odds_[idx];

        if (ttl_mode_ == "soft") {
            double effective_decay = lo_decay;
            if (ttl_sec_ > 0.0 && last_hit_time_[idx].nanoseconds() > 0) {
                const double idle_sec = (now - last_hit_time_[idx]).seconds();
                const double half_ttl = ttl_sec_ * 0.5;
                if (idle_sec >= ttl_sec_) {
                    effective_decay *= std::min(10.0, std::exp(0.5 * (idle_sec - ttl_sec_)));
                } else if (half_ttl > 0.0 && idle_sec > half_ttl) {
                    effective_decay *= (1.0 + (idle_sec - half_ttl) / half_ttl);
                }
            }
            if (lo > lo_target) {
                lo = std::max(lo_target, lo - effective_decay);
            } else if (lo < lo_target) {
                lo = std::min(lo_target, lo + effective_decay);
            }
        } else {
            if (ttl_sec_ > 0.0 && last_hit_time_[idx].nanoseconds() > 0) {
                const double idle_sec = (now - last_hit_time_[idx]).seconds();
                if (idle_sec > ttl_sec_) {
                    lo = lo_target;
                    if (blocked_state_[idx] != 0) {
                        blocked_state_[idx] = 0;
                        mask_dirty_ = true;
                    }
                    pending_state_[idx] = 0;
                    dwell_count_[idx] = dwell_cycles_;
                    last_hit_time_[idx] = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
                    continue;
                }
            }

            if (lo > lo_target) {
                lo = std::max(lo_target, lo - lo_decay);
            } else if (lo < lo_target) {
                lo = std::min(lo_target, lo + lo_decay);
            }
        }
    }

    if (keepout_enabled_) {
        for (int i = 1; i <= 12; i++) {
            const size_t idx = static_cast<size_t>(i);
            const double p = logOddsToProb(log_odds_[idx]);
            uint8_t candidate = blocked_state_[idx];
            if (p >= block_thresh_) {
                candidate = 1;
            } else if (p <= free_thresh_) {
                candidate = 0;
            }

            if (candidate == pending_state_[idx]) {
                if (++dwell_count_[idx] >= dwell_cycles_ &&
                    blocked_state_[idx] != candidate) {
                    blocked_state_[idx] = candidate;
                    mask_dirty_ = true;
                }
            } else {
                pending_state_[idx] = candidate;
                dwell_count_[idx] = 0;
            }
        }
        if (mask_dirty_) {
            publishMask();
            mask_dirty_ = false;
        }
    }
    publishHeartbeat();
    publishDiagnostics();
}

void KfsBlockFuser::publishMask() {
    if (!layout_loaded_) {
        return;
    }

    // 计算地图边界
    double xmin = 1e9, xmax = -1e9, ymin = 1e9, ymax = -1e9;
    const double half_size = block_half_size_m_ + keepout_margin_m_;
    const double radius = inflate_radius_m_ + keepout_margin_m_;
    const double pad = ((keepout_shape_ == "square") ? half_size : radius) + map_resolution_;
    for (int i = 1; i <= 12; i++) {
        xmin = std::min(xmin, cell_x_[static_cast<size_t>(i)] - pad);
        xmax = std::max(xmax, cell_x_[static_cast<size_t>(i)] + pad);
        ymin = std::min(ymin, cell_y_[static_cast<size_t>(i)] - pad);
        ymax = std::max(ymax, cell_y_[static_cast<size_t>(i)] + pad);
    }

    const int W = static_cast<int>((xmax - xmin) / map_resolution_) + 1;
    const int H = static_cast<int>((ymax - ymin) / map_resolution_) + 1;

    nav_msgs::msg::OccupancyGrid grid;
    grid.header.stamp = this->get_clock()->now();
    grid.header.frame_id = "map";
    grid.info.resolution = static_cast<float>(map_resolution_);
    grid.info.width  = static_cast<uint32_t>(W);
    grid.info.height = static_cast<uint32_t>(H);
    grid.info.origin.position.x = xmin;
    grid.info.origin.position.y = ymin;
    grid.info.origin.orientation.w = 1.0;
    grid.data.assign(static_cast<size_t>(W * H), 0);

    const double r2 = radius * radius;

    for (int i = 1; i <= 12; i++) {
        const size_t idx = static_cast<size_t>(i);
        if (blocked_state_[idx] == 0) continue;

        const double cx = cell_x_[idx];
        const double cy = cell_y_[idx];

        const double extent = (keepout_shape_ == "square") ? half_size : radius;
        const int px_min = static_cast<int>((cx - extent - xmin) / map_resolution_);
        const int px_max = static_cast<int>((cx + extent - xmin) / map_resolution_) + 1;
        const int py_min = static_cast<int>((cy - extent - ymin) / map_resolution_);
        const int py_max = static_cast<int>((cy + extent - ymin) / map_resolution_) + 1;

        for (int py = py_min; py <= py_max; py++) {
            for (int px = px_min; px <= px_max; px++) {
                if (px < 0 || px >= W || py < 0 || py >= H) continue;
                const double wx = xmin + (static_cast<double>(px) + 0.5) * map_resolution_;
                const double wy = ymin + (static_cast<double>(py) + 0.5) * map_resolution_;
                const double dx = wx - cx;
                const double dy = wy - cy;
                bool occupied = false;
                if (keepout_shape_ == "square") {
                    occupied = (std::abs(dx) <= half_size) && (std::abs(dy) <= half_size);
                } else {
                    occupied = (dx * dx + dy * dy <= r2);
                }
                if (occupied) {
                    grid.data[static_cast<size_t>(py * W + px)] = 100;
                }
            }
        }
    }

    pub_mask_->publish(grid);
    publishBlockOverlay();
}

void KfsBlockFuser::publishBlockOverlay() {
    rc26_interfaces::msg::MfBlockOverlay overlay;
    overlay.header.stamp = this->get_clock()->now();
    overlay.header.frame_id = "map";
    overlay.team = active_team_.empty() ? layout_team_ : active_team_;

    for (int i = 1; i <= 12; i++) {
        const size_t idx = static_cast<size_t>(i);
        rc26_interfaces::msg::MfBlockOverlayCell cell;
        cell.grid_id = static_cast<uint8_t>(i);

        double prob = 1.0 / (1.0 + std::exp(-log_odds_[idx]));
        cell.confidence = static_cast<float>(prob);

        if (blocked_state_[idx] == 1) {
            cell.state = rc26_interfaces::msg::MfBlockOverlayCell::BLOCKED;
        } else if (prob < free_thresh_) {
            cell.state = rc26_interfaces::msg::MfBlockOverlayCell::FREE;
        } else {
            cell.state = rc26_interfaces::msg::MfBlockOverlayCell::UNKNOWN;
        }

        cell.keepout_active = (keepout_enabled_ && blocked_state_[idx] == 1);
        overlay.cells.push_back(cell);
    }

    pub_block_overlay_->publish(overlay);
}

void KfsBlockFuser::publishHeartbeat() {
    if (!pub_heartbeat_) {
        return;
    }

    std_msgs::msg::Bool heartbeat_msg;
    heartbeat_msg.data = keepout_enabled_;
    pub_heartbeat_->publish(heartbeat_msg);
}

void KfsBlockFuser::publishDiagnostics() {
    if (!pub_diagnostics_) return;

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.level = keepout_enabled_ ? diagnostic_msgs::msg::DiagnosticStatus::OK
                                    : diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.name = this->get_fully_qualified_name();
    status.hardware_id = "R2";
    status.message = keepout_enabled_ ? "正常" : ("keepout disabled: " + keepout_disable_reason_);

    auto addKV = [&](const std::string& key, const std::string& value) {
        diagnostic_msgs::msg::KeyValue kv;
        kv.key = key;
        kv.value = value;
        status.values.push_back(kv);
    };
    addKV("keepout_enabled", keepout_enabled_ ? "true" : "false");
    addKV("keepout_disable_reason", keepout_disable_reason_);
    addKV("mask_topic", mask_topic_);
    addKV("heartbeat_topic", heartbeat_topic_);
    addKV("grid_layout_file", grid_layout_file_);
    addKV("force_release_topic", force_release_topic_);
    addKV("keepout_shape", keepout_shape_);
    addKV("block_thresh", std::to_string(block_thresh_));
    addKV("free_thresh", std::to_string(free_thresh_));
    addKV("lo_hit_block", std::to_string(lo_hit_block_));
    addKV("lo_hit_fake", std::to_string(lo_hit_fake_));
    addKV("lo_miss", std::to_string(lo_miss_));
    addKV("min_confidence", std::to_string(min_confidence_));
    addKV("inflate_radius_m", std::to_string(inflate_radius_m_));
    addKV("block_half_size_m", std::to_string(block_half_size_m_));
    addKV("keepout_margin_m", std::to_string(keepout_margin_m_));
    addKV("map_resolution", std::to_string(map_resolution_));
    addKV("decay_rate", std::to_string(decay_rate_));
    addKV("decay_target_prob", std::to_string(decay_target_prob_));
    addKV("ttl_sec", std::to_string(ttl_sec_));
    addKV("ttl_mode", ttl_mode_);
    addKV("dwell_cycles", std::to_string(dwell_cycles_));
    addKV("layout_team", layout_team_);
    addKV("active_team", active_team_);
    addKV("layout_version", layout_version_);
    addKV("layout_validated", layout_validated_ ? "true" : "false");
    addKV("layout_grid_spacing_m", std::to_string(layout_grid_spacing_m_));
    addKV("force_release_count", std::to_string(force_release_count_));
    addKV("team_mismatch_detected", team_mismatch_detected_ ? "true" : "false");

    diagnostic_msgs::msg::DiagnosticArray arr;
    arr.header.stamp = this->get_clock()->now();
    arr.status.push_back(status);
    pub_diagnostics_->publish(arr);
}

}  // namespace rc26_kfs_keepout

RCLCPP_COMPONENTS_REGISTER_NODE(rc26_kfs_keepout::KfsBlockFuser)
