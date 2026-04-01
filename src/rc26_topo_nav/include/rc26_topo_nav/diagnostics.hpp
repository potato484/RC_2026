#pragma once

#include "rc26_topo_nav/types.hpp"
#include "rc26_topo_nav/edge_executor.hpp"
#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace rc26_topo_nav {

class Diagnostics {
public:
    explicit Diagnostics(rclcpp::Node* node);

    void publishRoute(const std::vector<std::string>& node_path, const FieldGraph& graph);
    void publishCorridor(const nav_msgs::msg::Path& corridor);
    void publishDiagnostic(const std::string& status, const std::string& message);
    void publishActiveEdge(const GraphEdge& edge, const FieldGraph& graph, const std::string& gate_status);

private:
    visualization_msgs::msg::MarkerArray buildRiskMarkers(
        const GraphEdge& edge,
        const FieldGraph& graph,
        const std::string& gate_status) const;

    rclcpp::Node* node_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr route_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr corridor_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr xhu_route_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr xhu_corridor_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr xhu_diag_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr active_edge_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr semantic_gate_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr risk_markers_pub_;
};

}  // namespace rc26_topo_nav
