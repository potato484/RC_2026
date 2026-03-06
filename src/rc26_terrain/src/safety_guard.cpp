#include "rc26_terrain/safety_guard.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

namespace {

using rc26_terrain::SafetyGuardConfig;
using rc26_terrain::SafetyGuardDecision;
using rc26_terrain::SafetyGuardInput;
using rc26_terrain::SafetyIntervention;

constexpr int kDiagnosticOk = 0;
constexpr int kDiagnosticWarn = 1;
constexpr int kDiagnosticError = 2;
constexpr char kDefaultForcedSource[] = "default";

bool isFinite(double value) {
    return std::isfinite(value);
}

int interventionRank(SafetyIntervention intervention) {
    switch (intervention) {
        case SafetyIntervention::kEmergencyStop:
            return 2;
        case SafetyIntervention::kVirtualFence:
            return 1;
        case SafetyIntervention::kNone:
        default:
            return 0;
    }
}

SafetyIntervention strongerIntervention(SafetyIntervention lhs, SafetyIntervention rhs) {
    return interventionRank(lhs) >= interventionRank(rhs) ? lhs : rhs;
}

void appendMessage(std::vector<std::string>& messages, const std::string& message) {
    if (!message.empty()) {
        messages.push_back(message);
    }
}

std::string joinMessages(const std::vector<std::string>& messages) {
    if (messages.empty()) {
        return "正常";
    }

    std::string combined;
    for (size_t index = 0; index < messages.size(); ++index) {
        if (index > 0U) {
            combined += "; ";
        }
        combined += messages[index];
    }
    return combined;
}

}  // namespace

namespace rc26_terrain {

SafetyGuard::SafetyGuard(SafetyGuardConfig config)
    : config_(normalizeConfig(std::move(config))) {}

void SafetyGuard::configure(SafetyGuardConfig config) {
    std::scoped_lock<std::mutex> lock(mutex_);
    config_ = normalizeConfig(std::move(config));
    if (!config_.enable_fail_safe || config_.fail_safe_strategy == "none") {
        forced_fail_safe_reasons_.clear();
        last_decision_.fail_safe_active = false;
        last_decision_.fail_safe_reason.clear();
        last_decision_.fail_safe_intervention = SafetyIntervention::kNone;
        last_decision_.required_intervention = strongerIntervention(
            last_decision_.fail_safe_intervention, last_decision_.latency_intervention);
    }
}

SafetyGuardConfig SafetyGuard::config() const {
    std::scoped_lock<std::mutex> lock(mutex_);
    return config_;
}

SafetyGuardDecision SafetyGuard::evaluate(const SafetyGuardInput& input) {
    std::scoped_lock<std::mutex> lock(mutex_);

    SafetyGuardDecision decision = last_decision_;
    decision.fail_safe_entered = false;
    decision.fail_safe_cleared = false;
    decision.latency_intervention_entered = false;
    decision.latency_intervention_cleared = false;

    decision.in_startup_grace = input.since_start_sec < config_.startup_grace_sec;
    decision.cloud_ok = decision.in_startup_grace ||
                        (input.received_cloud && input.cloud_age_sec <= config_.cloud_timeout_sec);
    decision.odom_ok = decision.in_startup_grace ||
                       (input.received_odom && input.odom_age_sec <= config_.odom_timeout_sec);
    decision.tf_ok = !input.need_tf || (input.tf_age_sec <= config_.tf_health_timeout_sec);

    const bool thermal_window_open =
        config_.thermal_throttle_release_sec > 0.0 &&
        input.thermal_throttle_last_true_age_sec >= 0.0 &&
        input.thermal_throttle_last_true_age_sec <= config_.thermal_throttle_release_sec;
    decision.thermal_throttle_active = input.thermal_throttle_requested || thermal_window_open;

    decision.latency_error = isFinite(input.latency_ms) && input.latency_ms > config_.latency_error_ms;
    decision.latency_warn = !decision.latency_error &&
                            isFinite(input.latency_ms) &&
                            input.latency_ms > config_.latency_warn_ms;

    if (decision.thermal_throttle_active) {
        decision.latency_overrun_count = std::max(decision.latency_overrun_count,
                                                  config_.latency_trigger_frames);
        decision.latency_recover_count = 0;
    } else if (isFinite(input.latency_ms)) {
        if (input.latency_ms > config_.latency_error_ms) {
            ++decision.latency_overrun_count;
            decision.latency_recover_count = 0;
        } else if (input.latency_ms <= config_.latency_warn_ms) {
            ++decision.latency_recover_count;
            decision.latency_overrun_count = 0;
        } else {
            decision.latency_overrun_count = 0;
            decision.latency_recover_count = 0;
        }
    } else {
        decision.latency_overrun_count = 0;
        decision.latency_recover_count = 0;
    }

    if (!decision.latency_intervention_active &&
        config_.latency_intervention_mode != "none" &&
        decision.latency_overrun_count >= config_.latency_trigger_frames) {
        decision.latency_intervention_active = true;
        decision.latency_intervention_entered = true;
    } else if (decision.latency_intervention_active &&
               !decision.thermal_throttle_active &&
               decision.latency_recover_count >= config_.latency_recover_frames) {
        decision.latency_intervention_active = false;
        decision.latency_intervention_cleared = true;
    }

    decision.latency_intervention = decision.latency_intervention_active
        ? strategyToIntervention(config_.latency_intervention_mode)
        : SafetyIntervention::kNone;

    refreshForcedStateLocked(&decision);

    if (!decision.fail_safe_active &&
        config_.enable_fail_safe &&
        config_.fail_safe_strategy != "none" &&
        !decision.in_startup_grace) {
        if (!decision.cloud_ok) {
            decision.fail_safe_active = true;
            decision.fail_safe_reason = "点云输入超时/中断";
        } else if (!decision.odom_ok) {
            decision.fail_safe_active = true;
            decision.fail_safe_reason = "里程计输入超时/中断";
        } else if (!decision.tf_ok) {
            decision.fail_safe_active = true;
            decision.fail_safe_reason = "TF 变换链异常/超时";
        }
        if (decision.fail_safe_active) {
            decision.fail_safe_entered = !last_decision_.fail_safe_active;
            decision.fail_safe_cleared = false;
            decision.fail_safe_intervention = strategyToIntervention(config_.fail_safe_strategy);
        }
    }

    if (!decision.fail_safe_active && last_decision_.fail_safe_active) {
        decision.fail_safe_cleared = true;
        decision.fail_safe_reason.clear();
        decision.fail_safe_intervention = SafetyIntervention::kNone;
    }

    decision.required_intervention = strongerIntervention(
        decision.fail_safe_intervention, decision.latency_intervention);

    int diagnostic_level = kDiagnosticOk;
    std::vector<std::string> diagnostic_messages;

    const bool upstream_unhealthy = !decision.in_startup_grace &&
                                    (!decision.cloud_ok || !decision.odom_ok || !decision.tf_ok);
    if (decision.fail_safe_active) {
        diagnostic_level = std::max(diagnostic_level, kDiagnosticError);
        appendMessage(diagnostic_messages, "降级保护: " + decision.fail_safe_reason);
    } else if (upstream_unhealthy) {
        diagnostic_level = std::max(diagnostic_level, kDiagnosticError);
        if (!decision.cloud_ok) {
            appendMessage(diagnostic_messages, "点云输入超时/中断");
        }
        if (!decision.odom_ok) {
            appendMessage(diagnostic_messages, "里程计输入超时/中断");
        }
        if (!decision.tf_ok) {
            appendMessage(diagnostic_messages, "TF 变换链异常/超时");
        }
    }

    if (decision.latency_error) {
        diagnostic_level = std::max(diagnostic_level, kDiagnosticError);
        appendMessage(diagnostic_messages, "处理延迟超限");
    } else if (decision.latency_warn) {
        diagnostic_level = std::max(diagnostic_level, kDiagnosticWarn);
        appendMessage(diagnostic_messages, "处理延迟告警");
    }

    if (decision.thermal_throttle_active) {
        diagnostic_level = std::max(diagnostic_level, kDiagnosticError);
        appendMessage(diagnostic_messages, "热降频触发干预");
    }
    if (decision.latency_intervention_active) {
        diagnostic_level = std::max(diagnostic_level, kDiagnosticError);
        appendMessage(diagnostic_messages, "延迟干预激活");
    }

    decision.diagnostic_level = diagnostic_level;
    decision.diagnostic_message = joinMessages(diagnostic_messages);

    last_decision_ = decision;
    return last_decision_;
}

SafetyGuardDecision SafetyGuard::forceFailSafe(std::string reason) {
    return forceFailSafe(kDefaultForcedSource, std::move(reason));
}

SafetyGuardDecision SafetyGuard::forceFailSafe(const std::string& source, std::string reason) {
    std::scoped_lock<std::mutex> lock(mutex_);

    const std::string normalized_source = source.empty() ? std::string(kDefaultForcedSource) : source;
    forced_fail_safe_reasons_[normalized_source] = normalizeForcedReason(std::move(reason));
    refreshForcedStateLocked(&last_decision_);
    last_decision_.required_intervention = strongerIntervention(
        last_decision_.fail_safe_intervention, last_decision_.latency_intervention);
    last_decision_.diagnostic_level = std::max(last_decision_.diagnostic_level, kDiagnosticError);

    std::vector<std::string> messages;
    appendMessage(messages, "降级保护: " + last_decision_.fail_safe_reason);
    if (last_decision_.latency_error) {
        appendMessage(messages, "处理延迟超限");
    } else if (last_decision_.latency_warn) {
        appendMessage(messages, "处理延迟告警");
    }
    if (last_decision_.thermal_throttle_active) {
        appendMessage(messages, "热降频触发干预");
    }
    if (last_decision_.latency_intervention_active) {
        appendMessage(messages, "延迟干预激活");
    }
    last_decision_.diagnostic_message = joinMessages(messages);
    return last_decision_;
}

void SafetyGuard::clearForcedFailSafe() {
    clearForcedFailSafe(kDefaultForcedSource);
}

void SafetyGuard::clearForcedFailSafe(const std::string& source) {
    std::scoped_lock<std::mutex> lock(mutex_);
    const std::string normalized_source = source.empty() ? std::string(kDefaultForcedSource) : source;
    const size_t erased = forced_fail_safe_reasons_.erase(normalized_source);
    if (erased == 0U) {
        return;
    }

    const bool was_fail_safe_active = last_decision_.fail_safe_active;
    refreshForcedStateLocked(&last_decision_);
    if (was_fail_safe_active && !last_decision_.fail_safe_active) {
        last_decision_.fail_safe_cleared = true;
    }
    if (!last_decision_.fail_safe_active && last_decision_.diagnostic_message.rfind("降级保护:", 0) == 0) {
        last_decision_.diagnostic_message = last_decision_.latency_intervention_active
            ? std::string("延迟干预激活")
            : std::string("正常");
        last_decision_.diagnostic_level = last_decision_.latency_intervention_active
            ? kDiagnosticError
            : (last_decision_.latency_warn ? kDiagnosticWarn : kDiagnosticOk);
    }
}

SafetyGuardDecision SafetyGuard::decision() const {
    std::scoped_lock<std::mutex> lock(mutex_);
    return last_decision_;
}

void SafetyGuard::updateStats(const TerrainStats& stats) {
    std::scoped_lock<std::mutex> lock(mutex_);
    stats_ = stats;
}

TerrainStats SafetyGuard::stats() const {
    std::scoped_lock<std::mutex> lock(mutex_);
    return stats_;
}

bool SafetyGuard::isValidStrategy(const std::string& strategy) {
    return strategy == "none" || strategy == "virtual_fence" || strategy == "emergency_stop";
}

SafetyIntervention SafetyGuard::strategyToIntervention(const std::string& strategy) {
    if (strategy == "virtual_fence") {
        return SafetyIntervention::kVirtualFence;
    }
    if (strategy == "emergency_stop") {
        return SafetyIntervention::kEmergencyStop;
    }
    return SafetyIntervention::kNone;
}

SafetyGuardConfig SafetyGuard::normalizeConfig(SafetyGuardConfig config) {
    config.cloud_timeout_sec = std::max(config.cloud_timeout_sec, 1e-3);
    config.odom_timeout_sec = std::max(config.odom_timeout_sec, 1e-3);
    config.startup_grace_sec = std::max(0.0, config.startup_grace_sec);
    config.tf_health_timeout_sec = std::max(config.tf_health_timeout_sec, 1e-3);
    config.latency_warn_ms = std::max(0.0, config.latency_warn_ms);
    config.latency_error_ms = std::max(config.latency_warn_ms, config.latency_error_ms);
    config.latency_trigger_frames = std::max(1, config.latency_trigger_frames);
    config.latency_recover_frames = std::max(1, config.latency_recover_frames);
    config.thermal_throttle_release_sec = std::max(0.0, config.thermal_throttle_release_sec);

    if (!isValidStrategy(config.fail_safe_strategy)) {
        throw std::invalid_argument("fail_safe_strategy 仅支持: none/virtual_fence/emergency_stop");
    }
    if (!isValidStrategy(config.latency_intervention_mode)) {
        throw std::invalid_argument("latency_intervention_mode 仅支持: none/virtual_fence/emergency_stop");
    }
    return config;
}

std::string SafetyGuard::normalizeForcedReason(std::string reason) {
    return reason.empty() ? std::string("未指定故障") : std::move(reason);
}

std::string SafetyGuard::currentForcedReasonLocked() const {
    std::vector<std::string> reasons;
    reasons.reserve(forced_fail_safe_reasons_.size());
    for (const auto& [source, reason] : forced_fail_safe_reasons_) {
        if (source.empty()) {
            reasons.push_back(reason);
        } else {
            reasons.push_back(source + ": " + reason);
        }
    }
    return joinMessages(reasons);
}

void SafetyGuard::refreshForcedStateLocked(SafetyGuardDecision* decision) {
    if (decision == nullptr) {
        return;
    }

    const bool active = !forced_fail_safe_reasons_.empty();
    const bool was_active = decision->fail_safe_active;
    decision->fail_safe_active = active;
    decision->fail_safe_reason = active ? currentForcedReasonLocked() : std::string{};
    decision->fail_safe_entered = active && !was_active;
    decision->fail_safe_cleared = !active && was_active;
    decision->fail_safe_intervention = active && config_.enable_fail_safe
        ? strategyToIntervention(config_.fail_safe_strategy)
        : SafetyIntervention::kNone;
}

}  // namespace rc26_terrain
