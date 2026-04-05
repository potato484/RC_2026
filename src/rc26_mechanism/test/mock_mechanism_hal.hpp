#pragma once

#include <utility>

#include "rc26_mechanism/hal/i_mechanism_hal.hpp"

namespace rc26_mechanism {

class MockMechanismHAL : public IMechanismHAL {
public:
    bool open() override { return true; }
    void close() override {}
    bool isOpen() const override { return true; }

    bool sendCommand(uint8_t cmd_id, const std::vector<uint8_t>& payload = {}) override {
        last_cmd_ = cmd_id;
        last_payload_ = payload;
        return true;
    }
    bool sendCommand(uint8_t cmd_id, const std::vector<uint8_t>& payload, uint8_t& out_seq) override {
        last_cmd_ = cmd_id;
        last_payload_ = payload;
        out_seq = next_seq_++;
        return true;
    }

    void setFeedbackCallback(FeedbackCallback cb) override { cb_ = std::move(cb); }

    void injectFeedback(uint8_t seq, uint8_t fb_id, const std::vector<uint8_t>& payload = {}) {
        if (cb_) {
            cb_(seq, fb_id, payload);
        }
    }

    uint8_t last_cmd_{0};
    std::vector<uint8_t> last_payload_;

private:
    FeedbackCallback cb_;
    uint8_t next_seq_{0};
};

}  // namespace rc26_mechanism
