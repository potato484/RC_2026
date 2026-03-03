#pragma once

#include <algorithm>
#include <chrono>

namespace rc26_serial {

class AdaptiveTimeout {
public:
    void update(float measured_ms) {
        ewma_rtt_ms_ = (1.0F - ALPHA) * ewma_rtt_ms_ + ALPHA * measured_ms;
    }

    std::chrono::milliseconds get() const {
        return std::chrono::milliseconds(
            static_cast<int>(std::clamp(ewma_rtt_ms_ * SAFETY, MIN_MS, MAX_MS)));
    }

    float ewmaMs() const { return ewma_rtt_ms_; }

private:
    float ewma_rtt_ms_{30.0F};
    static constexpr float ALPHA = 0.125F;
    static constexpr float MIN_MS = 50.0F;
    static constexpr float MAX_MS = 500.0F;
    static constexpr float SAFETY = 4.0F;
};

}  // namespace rc26_serial
