#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>

#include "rc26_mechanism/hal/i_mechanism_hal.hpp"

namespace rc26_mechanism {

class FaultInjectingHAL : public IMechanismHAL {
public:
    struct Config {
        std::chrono::milliseconds action_latency{200};
        std::string mode{"action_fail_payload"};
        uint8_t fault_error_code{0x01};
    };

    FaultInjectingHAL();
    explicit FaultInjectingHAL(const Config& config);

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
    std::atomic<uint32_t> injected_count_{0};
    mutable std::mutex callback_mutex_;
    FeedbackCallback callback_;
};

}  // namespace rc26_mechanism
