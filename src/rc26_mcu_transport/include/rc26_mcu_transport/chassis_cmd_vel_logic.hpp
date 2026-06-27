#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <optional>
#include <string>
#include <utility>

namespace rc26_mcu_transport {

struct ChassisCmdVelConfig {
    bool enabled{true};
    std::string topic{"cmd_vel"};
    int target_send_rate_hz{50};
    int cmd_vel_timeout_ms{200};
    double v_max_mps{2.0};
    double w_max_radps{2.0};
    int stop_repeat_n{10};
};

struct ChassisBodyCommand {
    float vx{0.0f};
    float vy{0.0f};
    float wz{0.0f};
};

inline ChassisCmdVelConfig normalizeChassisCmdVelConfig(ChassisCmdVelConfig config) {
    if (config.topic.empty()) {
        config.topic = "cmd_vel";
    }
    config.target_send_rate_hz = std::max(1, config.target_send_rate_hz);
    config.cmd_vel_timeout_ms = std::max(0, config.cmd_vel_timeout_ms);
    config.v_max_mps = std::fabs(config.v_max_mps);
    config.w_max_radps = std::fabs(config.w_max_radps);
    config.stop_repeat_n = std::max(0, config.stop_repeat_n);
    return config;
}

inline ChassisBodyCommand clampChassisCommand(
    double vx, double vy, double wz, const ChassisCmdVelConfig& config) {
    ChassisBodyCommand command;
    if (!std::isfinite(vx)) {
        vx = 0.0;
    }
    if (!std::isfinite(vy)) {
        vy = 0.0;
    }
    if (!std::isfinite(wz)) {
        wz = 0.0;
    }
    const double linear_limit = std::fabs(config.v_max_mps);
    const double linear_mag = std::hypot(vx, vy);
    if (linear_limit <= 0.0) {
        vx = 0.0;
        vy = 0.0;
    } else if (linear_mag > linear_limit && linear_mag > 0.0) {
        const double scale = linear_limit / linear_mag;
        vx *= scale;
        vy *= scale;
    }

    const double angular_limit = std::fabs(config.w_max_radps);
    if (angular_limit <= 0.0) {
        wz = 0.0;
    } else {
        wz = std::clamp(wz, -angular_limit, angular_limit);
    }

    command.vx = static_cast<float>(vx);
    command.vy = static_cast<float>(vy);
    command.wz = static_cast<float>(wz);
    return command;
}

class ChassisCmdVelState {
public:
    using Clock = std::chrono::steady_clock;

    explicit ChassisCmdVelState(ChassisCmdVelConfig config = {})
        : config_(normalizeChassisCmdVelConfig(std::move(config))) {}

    const ChassisCmdVelConfig& config() const {
        return config_;
    }

    void updateConfig(ChassisCmdVelConfig config) {
        config_ = normalizeChassisCmdVelConfig(std::move(config));
        reset();
    }

    void reset() {
        target_ = {};
        last_cmd_time_ = {};
        command_received_ = false;
        timeout_zero_remaining_ = 0;
        timeout_zero_done_ = false;
    }

    ChassisBodyCommand receive(double vx, double vy, double wz, Clock::time_point now) {
        target_ = clampChassisCommand(vx, vy, wz, config_);
        last_cmd_time_ = now;
        command_received_ = true;
        timeout_zero_remaining_ = 0;
        timeout_zero_done_ = false;
        return target_;
    }

    std::optional<ChassisBodyCommand> nextCommand(Clock::time_point now) {
        if (!command_received_) {
            return std::nullopt;
        }

        const auto timeout = std::chrono::milliseconds(config_.cmd_vel_timeout_ms);
        if (config_.cmd_vel_timeout_ms == 0 || now - last_cmd_time_ <= timeout) {
            timeout_zero_remaining_ = 0;
            return target_;
        }

        if (timeout_zero_done_) {
            return std::nullopt;
        }
        if (timeout_zero_remaining_ == 0) {
            timeout_zero_remaining_ = config_.stop_repeat_n;
        }
        if (timeout_zero_remaining_ <= 0) {
            timeout_zero_done_ = true;
            return std::nullopt;
        }

        --timeout_zero_remaining_;
        if (timeout_zero_remaining_ == 0) {
            timeout_zero_done_ = true;
        }
        return ChassisBodyCommand{};
    }

private:
    ChassisCmdVelConfig config_;
    ChassisBodyCommand target_;
    Clock::time_point last_cmd_time_;
    bool command_received_{false};
    int timeout_zero_remaining_{0};
    bool timeout_zero_done_{false};
};

}  // namespace rc26_mcu_transport
