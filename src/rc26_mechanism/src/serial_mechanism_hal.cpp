#include "rc26_mechanism/hal/serial_mechanism_hal.hpp"

namespace rc26_mechanism {

SerialMechanismHAL::SerialMechanismHAL(std::string port, int baud)
    : port_(std::move(port)), baud_(baud) {
    driver_ = std::make_shared<rc26_decision::SerialDriver>();
}

bool SerialMechanismHAL::open() {
    if (!driver_) {
        return false;
    }
    return driver_->open(port_, baud_);
}

void SerialMechanismHAL::close() {
    if (driver_) {
        driver_->close();
    }
}

bool SerialMechanismHAL::isOpen() const {
    return driver_ && driver_->isOpen();
}

bool SerialMechanismHAL::sendCommand(uint8_t cmd_id, const std::vector<uint8_t>& payload) {
    if (!driver_) {
        return false;
    }
    return driver_->sendCommand(cmd_id, payload);
}

void SerialMechanismHAL::setFeedbackCallback(FeedbackCallback cb) {
    callback_ = std::move(cb);
    if (!driver_) {
        return;
    }
    driver_->setReceiveCallback(
        [this](uint8_t /*seq*/, uint8_t cmd, const std::vector<uint8_t>& payload) {
            if (callback_) {
                callback_(cmd, payload);
            }
        });
}

}  // namespace rc26_mechanism
