#include "rc26_mechanism/nodes/mechanism_lifecycle_server.hpp"

#include <string>

#include "rclcpp_components/register_node_macro.hpp"

#include "rc26_mechanism/hal/shared_serial/shared_serial_mechanism_hal.hpp"

namespace rc26_mechanism {

MechanismLifecycleServer::MechanismLifecycleServer(const rclcpp::NodeOptions& opts)
    : rclcpp_lifecycle::LifecycleNode("mechanism_server", opts) {
    this->declare_parameter<std::string>("hal_type", "shared_serial");
}

MechanismLifecycleServer::~MechanismLifecycleServer() {
    if (hal_) {
        hal_->close();
    }
}

MechanismLifecycleServer::CallbackReturn
MechanismLifecycleServer::on_configure(const rclcpp_lifecycle::State&) {
    const auto hal_type = this->get_parameter("hal_type").as_string();
    if (hal_type != "shared_serial") {
        RCLCPP_ERROR(this->get_logger(),
                     "unsupported hal_type: %s (shared_serial is the only supported runtime HAL)",
                     hal_type.c_str());
        return CallbackReturn::FAILURE;
    }

    hal_ = std::make_unique<SharedSerialMechanismHAL>(*this);
    RCLCPP_INFO(this->get_logger(),
                "mechanism high-level actions were removed; use /mechanism/send_command raw transport");
    return CallbackReturn::SUCCESS;
}

MechanismLifecycleServer::CallbackReturn
MechanismLifecycleServer::on_activate(const rclcpp_lifecycle::State&) {
    return CallbackReturn::SUCCESS;
}

MechanismLifecycleServer::CallbackReturn
MechanismLifecycleServer::on_deactivate(const rclcpp_lifecycle::State&) {
    if (hal_) {
        hal_->close();
    }
    return CallbackReturn::SUCCESS;
}

MechanismLifecycleServer::CallbackReturn
MechanismLifecycleServer::on_cleanup(const rclcpp_lifecycle::State&) {
    if (hal_) {
        hal_->close();
        hal_.reset();
    }
    return CallbackReturn::SUCCESS;
}

MechanismLifecycleServer::CallbackReturn
MechanismLifecycleServer::on_error(const rclcpp_lifecycle::State&) {
    if (hal_) {
        hal_->close();
    }
    return CallbackReturn::SUCCESS;
}

}  // namespace rc26_mechanism

RCLCPP_COMPONENTS_REGISTER_NODE(rc26_mechanism::MechanismLifecycleServer)
