#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>

namespace rc26_xhu_nav::mode_manager {

class WatchdogTimer {
public:
    using TimeoutCallback = std::function<void()>;

    explicit WatchdogTimer(rclcpp::Node* node);

    void start(double timeout_sec, TimeoutCallback callback,
               rclcpp::CallbackGroup::SharedPtr group = nullptr);
    void reset();
    void cancel();
    bool isActive() const;

private:
    void onTimeout();

    rclcpp::Node* node_;
    rclcpp::TimerBase::SharedPtr timer_;
    TimeoutCallback callback_;
    mutable std::mutex mutex_;
    std::atomic<bool> active_{false};
};

}  // namespace rc26_xhu_nav::mode_manager
