/*
 * @Author: potato potato@potato.com
 * @Date: 2026-01-04 19:10:47
 * @LastEditors: potato potato@potato.com
 * @LastEditTime: 2026-01-04 19:13:19
 * @FilePath: /RC_2026/RC_2026_1/rc26_decision/src/gain_odom/can_odom_test_node.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
// RC2026 CAN里程计测试节点
// 从CAN总线读取电机转速，计算并发布底盘速度
#include <rclcpp/rclcpp.hpp>

#include "rc26_decision/gain_odom/can_odom.hpp"

class CanOdomTestNode : public rclcpp::Node
{
public:
    CanOdomTestNode() : Node("can_odom_test")
    {
        // 声明参数
        this->declare_parameter<std::string>("can_interface", "can0");
        this->declare_parameter<double>("wheel_radius", 0.076);
        this->declare_parameter<double>("wheel_base", 0.3);
        this->declare_parameter<double>("track_width", 0.3);
        this->declare_parameter<double>("gear_ratio", 3591.0 / 187.0);
        this->declare_parameter<int>("publish_rate_hz", 50);
        this->declare_parameter<std::string>("odom_topic", "wheel_odom");

        // 配置CAN里程计
        rc26_decision::CanOdom::Config config;
        config.can_interface = this->get_parameter("can_interface").as_string();
        config.wheel_radius = this->get_parameter("wheel_radius").as_double();
        config.wheel_base = this->get_parameter("wheel_base").as_double();
        config.track_width = this->get_parameter("track_width").as_double();
        config.gear_ratio = this->get_parameter("gear_ratio").as_double();
        config.publish_rate_hz = this->get_parameter("publish_rate_hz").as_int();
        config.odom_topic = this->get_parameter("odom_topic").as_string();

        can_odom_ = std::make_unique<rc26_decision::CanOdom>(*this, config);

        RCLCPP_INFO(this->get_logger(), "CAN 里程计测试节点启动");
    }

private:
    std::unique_ptr<rc26_decision::CanOdom> can_odom_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CanOdomTestNode>());
    rclcpp::shutdown();
    return 0;
}
