#pragma once

#include <memory>
#include <string>

#include "rc26_mechanism/hal/i_mechanism_hal.hpp"
#include "rc26_serial/serial_driver.hpp"

namespace rc26_mechanism {

class SerialMechanismHAL : public IMechanismHAL {
public:
    SerialMechanismHAL(std::string port, int baud);

    bool open() override;
    void close() override;
    bool isOpen() const override;
    bool sendCommand(uint8_t cmd_id, const std::vector<uint8_t>& payload = {}) override;
    bool sendCommand(uint8_t cmd_id, const std::vector<uint8_t>& payload, uint8_t& out_seq) override;
    void setFeedbackCallback(FeedbackCallback cb) override;
    CommHealthSnapshot commHealthSnapshot() const override;

private:
    std::string port_;
    int baud_;
    std::shared_ptr<rc26_decision::SerialDriver> driver_;
    FeedbackCallback callback_;
};

}  // namespace rc26_mechanism
