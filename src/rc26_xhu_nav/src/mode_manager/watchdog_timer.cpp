#include "rc26_xhu_nav/mode_manager/watchdog_timer.hpp"

namespace rc26_xhu_nav::mode_manager {

WatchdogTimer::WatchdogTimer(rclcpp::Node* node) : node_(node) {}

void WatchdogTimer::start(double timeout_sec, TimeoutCallback callback,
                          rclcpp::CallbackGroup::SharedPtr group) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (timer_) {
        timer_->cancel();
        timer_.reset();
    }

    if (timeout_sec <= 0) {
        active_ = false;
        callback_ = nullptr;
        return;
    }

    callback_ = std::move(callback);
    active_ = true;

    timer_ = node_->create_wall_timer(
        std::chrono::duration<double>(timeout_sec),
        std::bind(&WatchdogTimer::onTimeout, this),
        group);
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
    callback_ = nullptr;
    active_ = false;
}

bool WatchdogTimer::isActive() const {
    return active_.load();
}

void WatchdogTimer::onTimeout() {
    TimeoutCallback callback;
    rclcpp::TimerBase::SharedPtr timer;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!active_) {
            return;
        }

        active_ = false;
        callback = callback_;
        callback_ = nullptr;
        timer = std::move(timer_);
    }

    if (timer) {
        timer->cancel();
    }
    if (callback) {
        callback();
    }
}

}  // namespace rc26_xhu_nav::mode_manager
