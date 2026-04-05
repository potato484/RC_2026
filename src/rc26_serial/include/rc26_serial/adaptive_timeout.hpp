#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>

namespace rc26_serial {

class AdaptiveTimeout {
public:
    void update(float measured_ms) {
        if (!initialized_) {
            srtt_ = measured_ms;
            rttvar_ = measured_ms / 2.0F;
            rto_backoff_ = std::clamp(srtt_ + K * rttvar_, RTO_MIN, RTO_MAX);
            initialized_ = true;
            return;
        }

        rttvar_ = (1.0F - BETA) * rttvar_ + BETA * std::fabs(measured_ms - srtt_);
        srtt_ = (1.0F - ALPHA) * srtt_ + ALPHA * measured_ms;
        rto_backoff_ = std::clamp(srtt_ + K * rttvar_, RTO_MIN, RTO_MAX);
    }

    void onTimeout() {
        rto_backoff_ = std::min(rto_backoff_ * 2.0F, RTO_MAX);
    }

    std::chrono::milliseconds get() const {
        return std::chrono::milliseconds(static_cast<int>(rto_backoff_));
    }

    float ewmaMs() const { return srtt_; }

private:
    float srtt_{30.0F};
    float rttvar_{15.0F};
    float rto_backoff_{100.0F};
    bool initialized_{false};

    static constexpr float ALPHA = 0.125F;
    static constexpr float BETA = 0.25F;
    static constexpr float K = 4.0F;
    static constexpr float RTO_MIN = 50.0F;
    static constexpr float RTO_MAX = 500.0F;
};

}  // namespace rc26_serial
