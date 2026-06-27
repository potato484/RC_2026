#pragma once

#include <cstdint>
#include <functional>
#include <vector>

namespace rc26_mechanism {

using FeedbackCallback = std::function<void(uint8_t, uint8_t, const std::vector<uint8_t>&)>;

struct CommHealthSnapshot {
    uint32_t ack_timeout_count{0};
    uint32_t reconnect_count{0};
    uint32_t parse_error_count{0};
    float avg_rtt_ms{0.0F};
    uint8_t comm_health_level{0};
};

class IMechanismHAL {
public:
    virtual ~IMechanismHAL() = default;

    virtual bool open() = 0;
    virtual void close() = 0;
    virtual bool isOpen() const = 0;

    virtual bool sendCommand(uint8_t cmd_id, const std::vector<uint8_t>& payload = {}) = 0;
    virtual bool sendCommand(uint8_t cmd_id, const std::vector<uint8_t>& payload, uint8_t& out_seq) {
        out_seq = 0;
        return sendCommand(cmd_id, payload);
    }
    virtual void setFeedbackCallback(FeedbackCallback cb) = 0;
    virtual CommHealthSnapshot commHealthSnapshot() const { return {}; }
};

}  // namespace rc26_mechanism
