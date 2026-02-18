#include "rc26_nav_mode_manager/watchdog_timer.hpp"

namespace rc26_nav_mode_manager {

WatchdogTimer::WatchdogTimer(rclcpp::Node* node) : node_(node) {}

void WatchdogTimer::start(double timeout_sec, TimeoutCallback callback) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (timer_) {
        timer_->cancel();
    }

    if (timeout_sec <= 0) {
        active_ = false;
        return;
    }

    callback_ = std::move(callback);
    active_ = true;

    timer_ = node_->create_wall_timer(
        std::chrono::duration<double>(timeout_sec),
        std::bind(&WatchdogTimer::onTimeout, this));
}

void WatchdogTimer::reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (timer_ && active_) {
        timer_->reset();
    }
}

void WatchdogTimer::cancel() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (timer_) {
        timer_->cancel();
        timer_.reset();
    }
    active_ = false;
}

bool WatchdogTimer::isActive() const {
    return active_.load();
}

void WatchdogTimer::onTimeout() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (active_ && callback_) {
        active_ = false;
        callback_();
    }
    if (timer_) {
        timer_->cancel();
        timer_.reset();
    }
}

}  // namespace rc26_nav_mode_manager
