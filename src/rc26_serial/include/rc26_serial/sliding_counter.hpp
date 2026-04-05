#pragma once

#include <array>
#include <cstdint>
#include <mutex>

namespace rc26_serial {

template <uint32_t N>
class SlidingCounter {
public:
    void record(bool ok) {
        std::lock_guard<std::mutex> lock(mutex_);
        const uint32_t index = pos_++ % N;
        bad_ -= slots_[index];
        slots_[index] = ok ? 0U : 1U;
        bad_ += slots_[index];
        if (filled_ < N) {
            ++filled_;
        }
    }

    float errorRate() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return filled_ ? static_cast<float>(bad_) / static_cast<float>(filled_) : 0.0F;
    }

    void reset() {
        std::lock_guard<std::mutex> lock(mutex_);
        slots_.fill(0U);
        pos_ = 0;
        filled_ = 0;
        bad_ = 0;
    }

private:
    std::array<uint32_t, N> slots_{};
    uint32_t pos_{0};
    uint32_t filled_{0};
    uint32_t bad_{0};
    mutable std::mutex mutex_;
};

}  // namespace rc26_serial
