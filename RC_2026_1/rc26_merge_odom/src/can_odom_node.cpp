// RC2026 CAN里程计独立节点
#include <rclcpp/rclcpp.hpp>
#include "rc26_merge_odom/can_odom.hpp"

class CanOdomNode : public rclcpp::Node
{
public:
    CanOdomNode() : Node("can_odom_node")
    {
        this->declare_parameter("can_interface", "can0");
        this->declare_parameter("wheel_radius", 0.07625);
        this->declare_parameter("wheel_base", 0.62326);
        this->declare_parameter("track_width", 0.7);
        this->declare_parameter("gear_ratio", 3591.0 / 187.0);
        this->declare_parameter("publish_rate_hz", 50);
        this->declare_parameter("odom_topic", "Can_Odom");
        this->declare_parameter("odom_frame", "odom");
        this->declare_parameter("base_frame", "base_link");

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

        can_odom_ = std::make_unique<rc26_merge_odom::CanOdom>(*this, config);
    }

private:
    std::unique_ptr<rc26_merge_odom::CanOdom> can_odom_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CanOdomNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
