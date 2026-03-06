// RC2026 CAN里程计独立节点
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>

#include "rc26_merge_odom/can/can_odom.hpp"

class CanOdomNode : public rclcpp::Node {
public:
    CanOdomNode() : Node("can_odom_node") {
        this->declare_parameter("can_interface", "can0");
        this->declare_parameter("wheel_radius", 0.07625);
        this->declare_parameter("wheel_base", 0.62326);
        this->declare_parameter("track_width", 0.7);
        this->declare_parameter("gear_ratio", 3591.0 / 187.0);
        this->declare_parameter("publish_rate_hz", 50);
        this->declare_parameter("odom_topic", "Can_Odom");
        this->declare_parameter("odom_frame", "odom");
        this->declare_parameter("base_frame", "base_link");
        this->declare_parameter("imu_topic", "DM_IMU");
        this->declare_parameter("data_timeout_ms", 100.0);
        this->declare_parameter("slip_enable", true);
        this->declare_parameter("slip_threshold", 0.5);
        this->declare_parameter("slip_k_acc", 0.5);
        this->declare_parameter("cov_nominal_v", 0.01);
        this->declare_parameter("cov_nominal_wz", 0.03);
        this->declare_parameter("cov_slip_v", 0.5);
        this->declare_parameter("cov_slip_wz", 0.5);
        this->declare_parameter("recovery_tau_s", 0.5);

        rc26_merge_odom::CanOdom::Config config;
        config.can_interface = this->get_parameter("can_interface").as_string();
        config.wheel_radius = this->get_parameter("wheel_radius").as_double();
        config.wheel_base = this->get_parameter("wheel_base").as_double();
        config.track_width = this->get_parameter("track_width").as_double();
        config.gear_ratio = this->get_parameter("gear_ratio").as_double();
        config.publish_rate_hz = this->get_parameter("publish_rate_hz").as_int();
        config.odom_topic = this->get_parameter("odom_topic").as_string();
        config.odom_frame = this->get_parameter("odom_frame").as_string();
        config.base_frame = this->get_parameter("base_frame").as_string();
        config.imu_topic = this->get_parameter("imu_topic").as_string();
        config.data_timeout_ms = this->get_parameter("data_timeout_ms").as_double();
        config.slip_enable = this->get_parameter("slip_enable").as_bool();
        config.slip_threshold = this->get_parameter("slip_threshold").as_double();
        config.slip_k_acc = this->get_parameter("slip_k_acc").as_double();
        config.cov_nominal_v = this->get_parameter("cov_nominal_v").as_double();
        config.cov_nominal_wz = this->get_parameter("cov_nominal_wz").as_double();
        config.cov_slip_v = this->get_parameter("cov_slip_v").as_double();
        config.cov_slip_wz = this->get_parameter("cov_slip_wz").as_double();
        config.recovery_tau_s = this->get_parameter("recovery_tau_s").as_double();

        can_odom_ = std::make_unique<rc26_merge_odom::CanOdom>(*this, config);
        if (!can_odom_->isReady()) {
            throw std::runtime_error("CanOdom 初始化失败");
        }
    }

private:
    std::unique_ptr<rc26_merge_odom::CanOdom> can_odom_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    int exit_code = 0;
    try {
        auto node = std::make_shared<CanOdomNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("can_odom_node"), "Exception: %s", e.what());
        exit_code = 1;
    }
    rclcpp::shutdown();
    return exit_code;
}
