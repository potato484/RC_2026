#pragma once

#include <memory>

#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "rc26_mechanism/hal/contracts/i_mechanism_hal.hpp"

namespace rc26_mechanism {

class MechanismLifecycleServer : public rclcpp_lifecycle::LifecycleNode {
public:
    explicit MechanismLifecycleServer(const rclcpp::NodeOptions& opts);
    ~MechanismLifecycleServer() override;

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
    CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override;
    CallbackReturn on_error(const rclcpp_lifecycle::State&) override;

private:
    std::unique_ptr<IMechanismHAL> hal_;
};

}  // namespace rc26_mechanism
