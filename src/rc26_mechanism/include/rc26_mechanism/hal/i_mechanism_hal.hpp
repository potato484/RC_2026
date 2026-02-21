#pragma once

#include <cstdint>
#include <functional>
#include <vector>

namespace rc26_mechanism {

using FeedbackCallback = std::function<void(uint8_t, const std::vector<uint8_t>&)>;

class IMechanismHAL {
public:
    virtual ~IMechanismHAL() = default;

    virtual bool open() = 0;
    virtual void close() = 0;
    virtual bool isOpen() const = 0;

    virtual bool sendCommand(uint8_t cmd_id, const std::vector<uint8_t>& payload = {}) = 0;
    virtual void setFeedbackCallback(FeedbackCallback cb) = 0;
};

}  // namespace rc26_mechanism
