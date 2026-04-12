#include "rc26_xhu_nav/topology/topo_nav_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rc26_xhu_nav::topology::TopoNavNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
