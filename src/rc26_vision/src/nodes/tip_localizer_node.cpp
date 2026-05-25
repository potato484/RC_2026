#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "rc26_vision/postprocess/tip_localizer.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    int exit_code = 0;
    try {
        auto node = std::make_shared<rc26_vision::TipLocalizer>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("tip_localizer_node"), "启动失败: %s", e.what());
        exit_code = 1;
    }

    rclcpp::shutdown();
    return exit_code;
}
