// RC2026 轮式里程计融合节点
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "rc26_merge_odom/fuser/wheel_odom_fuser.hpp"

class WheelOdomFuserNode : public rclcpp::Node {
public:
    WheelOdomFuserNode() : Node("wheel_odom_fuser_node") {
        this->declare_parameter("publish_rate_hz", 50);
        this->declare_parameter("data_timeout_ms", 100.0);
        this->declare_parameter("omega_sigma_rps", 0.5);
        this->declare_parameter("chi2_threshold_dof3", 11.34);
        this->declare_parameter("outlier_penalty", 0.1);
        this->declare_parameter("recovery_tau_s", 0.5);
        this->declare_parameter("can_odom_topic", "Can_Odom");
        this->declare_parameter("wheel_odom_topic", "wheel_odom");
        this->declare_parameter("imu_topic", "DM_IMU");
        this->declare_parameter("fused_odom_topic", "wheel_odom_fused");
        this->declare_parameter("health_topic", "wheel_odom_fuser/health");

        rc26_merge_odom::WheelOdomFuser::Config config;
        config.publish_rate_hz = static_cast<int>(this->get_parameter("publish_rate_hz").as_int());
        config.data_timeout_ms = this->get_parameter("data_timeout_ms").as_double();
        config.omega_sigma_rps = this->get_parameter("omega_sigma_rps").as_double();
        config.chi2_threshold_dof3 = this->get_parameter("chi2_threshold_dof3").as_double();
        config.outlier_penalty = this->get_parameter("outlier_penalty").as_double();
        config.recovery_tau_s = this->get_parameter("recovery_tau_s").as_double();
        config.can_odom_topic = this->get_parameter("can_odom_topic").as_string();
        config.wheel_odom_topic = this->get_parameter("wheel_odom_topic").as_string();
        config.imu_topic = this->get_parameter("imu_topic").as_string();
        config.fused_odom_topic = this->get_parameter("fused_odom_topic").as_string();
        config.health_topic = this->get_parameter("health_topic").as_string();

        fuser_ = std::make_unique<rc26_merge_odom::WheelOdomFuser>(*this, config);
    }

private:
    std::unique_ptr<rc26_merge_odom::WheelOdomFuser> fuser_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WheelOdomFuserNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
