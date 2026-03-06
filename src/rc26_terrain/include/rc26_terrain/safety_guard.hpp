#pragma once

#include <cstdint>
#include <map>
#include <mutex>
#include <string>

namespace rc26_terrain {

enum class SafetyIntervention {
    kNone = 0,
    kVirtualFence,
    kEmergencyStop,
};

struct SafetyGuardConfig {
    double cloud_timeout_sec{0.7};
    double odom_timeout_sec{0.7};
    double startup_grace_sec{1.0};
    double tf_health_timeout_sec{0.7};
    bool enable_fail_safe{true};
    std::string fail_safe_strategy{"virtual_fence"};
    double latency_warn_ms{12.0};
    double latency_error_ms{20.0};
    int latency_trigger_frames{3};
    int latency_recover_frames{5};
    std::string latency_intervention_mode{"virtual_fence"};
    double thermal_throttle_release_sec{0.5};
};

struct SafetyGuardInput {
    double since_start_sec{0.0};
    bool received_cloud{false};
    double cloud_age_sec{0.0};
    bool received_odom{false};
    double odom_age_sec{0.0};
    bool need_tf{false};
    double tf_age_sec{0.0};
    double latency_ms{0.0};
    bool thermal_throttle_requested{false};
    double thermal_throttle_last_true_age_sec{0.0};
};

struct SafetyGuardDecision {
    bool in_startup_grace{false};
    bool cloud_ok{true};
    bool odom_ok{true};
    bool tf_ok{true};

    bool fail_safe_active{false};
    std::string fail_safe_reason;
    bool fail_safe_entered{false};
    bool fail_safe_cleared{false};
    SafetyIntervention fail_safe_intervention{SafetyIntervention::kNone};

    bool thermal_throttle_active{false};
    bool latency_warn{false};
    bool latency_error{false};
    int latency_overrun_count{0};
    int latency_recover_count{0};
    bool latency_intervention_active{false};
    bool latency_intervention_entered{false};
    bool latency_intervention_cleared{false};
    SafetyIntervention latency_intervention{SafetyIntervention::kNone};

    SafetyIntervention required_intervention{SafetyIntervention::kNone};
    int diagnostic_level{0};
    std::string diagnostic_message{"正常"};
};

struct TerrainStats {
    int kfs_occupied_cells{0};
    int obstacle_cells{0};
    int drop_cells{0};
    int climbable_cells{0};
};

class SafetyGuard {
public:
    explicit SafetyGuard(SafetyGuardConfig config = {});

    void configure(SafetyGuardConfig config);
    SafetyGuardConfig config() const;

    SafetyGuardDecision evaluate(const SafetyGuardInput& input);
    SafetyGuardDecision forceFailSafe(std::string reason);
    SafetyGuardDecision forceFailSafe(const std::string& source, std::string reason);
    void clearForcedFailSafe();
    void clearForcedFailSafe(const std::string& source);
    SafetyGuardDecision decision() const;

    void updateStats(const TerrainStats& stats);
    TerrainStats stats() const;

    static bool isValidStrategy(const std::string& strategy);
    static SafetyIntervention strategyToIntervention(const std::string& strategy);

private:
    static SafetyGuardConfig normalizeConfig(SafetyGuardConfig config);
    static std::string normalizeForcedReason(std::string reason);
    std::string currentForcedReasonLocked() const;
    void refreshForcedStateLocked(SafetyGuardDecision* decision);

    mutable std::mutex mutex_;
    SafetyGuardConfig config_{};
    SafetyGuardDecision last_decision_{};
    std::map<std::string, std::string> forced_fail_safe_reasons_;
    TerrainStats stats_{};
};

}  // namespace rc26_terrain
