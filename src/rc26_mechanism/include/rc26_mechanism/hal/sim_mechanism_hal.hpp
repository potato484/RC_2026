#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <random>

#include "rc26_mechanism/hal/i_mechanism_hal.hpp"

namespace rc26_mechanism {

class SimMechanismHAL : public IMechanismHAL {
public:
    struct Config {
        std::chrono::milliseconds action_latency{500};
        float fail_rate{0.0F};
        uint8_t fail_error_code{0x01};
        uint32_t random_seed{0x2601U};
    };

    SimMechanismHAL();
    explicit SimMechanismHAL(const Config& config);

    bool open() override;
    void close() override;
    bool isOpen() const override;
    bool sendCommand(uint8_t cmd_id, const std::vector<uint8_t>& payload = {}) override;
    bool sendCommand(uint8_t cmd_id, const std::vector<uint8_t>& payload, uint8_t& out_seq) override;
    void setFeedbackCallback(FeedbackCallback cb) override;
    CommHealthSnapshot commHealthSnapshot() const override;

private:
    uint8_t nextSeq();

    Config config_;
    std::atomic<bool> open_{false};
    std::atomic<uint8_t> seq_{0};
    mutable std::mutex callback_mutex_;
    FeedbackCallback callback_;
    mutable std::mutex rng_mutex_;
    std::mt19937 rng_;
    std::uniform_real_distribution<float> fail_dist_{0.0F, 1.0F};
    std::atomic<uint32_t> fail_count_{0};
    std::shared_ptr<std::atomic<bool>> alive_ = std::make_shared<std::atomic<bool>>(false);
};

}  // namespace rc26_mechanism
