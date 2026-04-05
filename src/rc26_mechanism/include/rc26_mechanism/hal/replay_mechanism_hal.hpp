#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "rc26_mechanism/hal/i_mechanism_hal.hpp"

namespace rc26_mechanism {

class ReplayMechanismHAL : public IMechanismHAL {
public:
    struct Config {
        std::string replay_file{};
        std::chrono::milliseconds action_latency{100};
        bool loop{false};
    };

    ReplayMechanismHAL();
    explicit ReplayMechanismHAL(const Config& config);

    bool open() override;
    void close() override;
    bool isOpen() const override;
    bool sendCommand(uint8_t cmd_id, const std::vector<uint8_t>& payload = {}) override;
    bool sendCommand(uint8_t cmd_id, const std::vector<uint8_t>& payload, uint8_t& out_seq) override;
    void setFeedbackCallback(FeedbackCallback cb) override;
    CommHealthSnapshot commHealthSnapshot() const override;

private:
    struct ReplayEvent {
        uint8_t seq{0xFF};
        uint8_t fb_id{0};
        std::vector<uint8_t> payload;
    };

    uint8_t nextSeq();
    bool loadReplayFile();
    static bool parseUint8Token(const std::string& token, uint8_t& out);
    static std::vector<uint8_t> parsePayloadHex(const std::string& token);
    static std::optional<uint8_t> doneFeedbackForCommand(uint8_t cmd_id);

    Config config_;
    std::atomic<bool> open_{false};
    std::atomic<uint8_t> seq_{0};
    mutable std::mutex callback_mutex_;
    FeedbackCallback callback_;
    mutable std::mutex events_mutex_;
    std::vector<ReplayEvent> events_;
    size_t event_index_{0};
    std::atomic<uint32_t> parse_error_count_{0};
    std::shared_ptr<std::atomic<bool>> alive_ = std::make_shared<std::atomic<bool>>(false);
};

}  // namespace rc26_mechanism
