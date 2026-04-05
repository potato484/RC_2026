#include "rc26_topo_nav/topo_nav_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rc26_topo_nav::TopoNavNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
