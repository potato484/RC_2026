#include "rc26_topo_nav/diagnostics.hpp"

#include <cmath>
#include <std_msgs/msg/color_rgba.hpp>

namespace rc26_topo_nav {

namespace {

std_msgs::msg::ColorRGBA makeColor(float r, float g, float b, float a = 1.0F) {
    std_msgs::msg::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    return color;
}

std_msgs::msg::ColorRGBA gateColor(const std::string& status) {
    if (status == "PASS") {
        return makeColor(0.10F, 0.75F, 0.20F, 0.9F);
    }
    if (status == "HOLD") {
        return makeColor(1.00F, 0.85F, 0.10F, 0.95F);
    }
    if (status == "REPLAN") {
        return makeColor(1.00F, 0.45F, 0.10F, 0.95F);
    }
    return makeColor(0.95F, 0.10F, 0.10F, 0.95F);
}

}  // namespace

Diagnostics::Diagnostics(rclcpp::Node* node) : node_(node) {
    route_pub_ = node_->create_publisher<nav_msgs::msg::Path>("/topo_nav/route", 10);
    corridor_pub_ = node_->create_publisher<nav_msgs::msg::Path>("/topo_nav/corridor", 10);
    xhu_route_pub_ = node_->create_publisher<nav_msgs::msg::Path>("/xhu_nav/route", 10);
    xhu_corridor_pub_ = node_->create_publisher<nav_msgs::msg::Path>("/xhu_nav/corridor", 10);
    diag_pub_ = node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
    xhu_diag_pub_ = node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/xhu_nav/diagnostics", 10);
    active_edge_pub_ = node_->create_publisher<std_msgs::msg::String>("/xhu_nav/active_edge", 10);
    semantic_gate_pub_ = node_->create_publisher<std_msgs::msg::String>("/xhu_nav/semantic_gate", 10);
    risk_markers_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/xhu_nav/risk_markers", 10);
}

void Diagnostics::publishRoute(const std::vector<std::string>& node_path,
                               const FieldGraph& graph) {
    nav_msgs::msg::Path msg;
    msg.header.frame_id = "map";
    msg.header.stamp = node_->now();

    for (const auto& nid : node_path) {
        auto it = graph.nodes.find(nid);
        if (it == graph.nodes.end()) continue;
        geometry_msgs::msg::PoseStamped ps;
        ps.header = msg.header;
        ps.pose.position.x = it->second.pose.x;
        ps.pose.position.y = it->second.pose.y;
        ps.pose.position.z = it->second.pose.z;
        ps.pose.orientation.z = std::sin(it->second.pose.yaw / 2.0);
        ps.pose.orientation.w = std::cos(it->second.pose.yaw / 2.0);
        msg.poses.push_back(ps);
    }
    route_pub_->publish(msg);
    xhu_route_pub_->publish(msg);
}

void Diagnostics::publishCorridor(const nav_msgs::msg::Path& corridor) {
    corridor_pub_->publish(corridor);
    xhu_corridor_pub_->publish(corridor);
}

void Diagnostics::publishDiagnostic(const std::string& status, const std::string& message) {
    diagnostic_msgs::msg::DiagnosticArray arr;
    arr.header.stamp = node_->now();
    diagnostic_msgs::msg::DiagnosticStatus ds;
    ds.name = "topo_nav";
    ds.hardware_id = "rc26_topo_nav";
    ds.message = message;

    if (status == "OK") ds.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    else if (status == "WARN") ds.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    else ds.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;

    arr.status.push_back(ds);
    diag_pub_->publish(arr);
    xhu_diag_pub_->publish(arr);
}

visualization_msgs::msg::MarkerArray Diagnostics::buildRiskMarkers(
    const GraphEdge& edge,
    const FieldGraph& graph,
    const std::string& gate_status) const {
    visualization_msgs::msg::MarkerArray markers;
    const auto color = gateColor(gate_status);
    const auto stamp = node_->now();

    auto from_it = graph.nodes.find(edge.from);
    auto to_it = graph.nodes.find(edge.to);
    if (from_it == graph.nodes.end() || to_it == graph.nodes.end()) {
        return markers;
    }

    visualization_msgs::msg::Marker line;
    line.header.frame_id = "map";
    line.header.stamp = stamp;
    line.ns = "xhu_active_edge";
    line.id = 1;
    line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line.action = visualization_msgs::msg::Marker::ADD;
    line.scale.x = 0.08;
    line.color = color;

    geometry_msgs::msg::Point p;
    p.x = from_it->second.pose.x;
    p.y = from_it->second.pose.y;
    p.z = from_it->second.pose.z;
    line.points.push_back(p);
    for (const auto& cp : edge.control_points) {
        geometry_msgs::msg::Point cp_point;
        cp_point.x = cp.x;
        cp_point.y = cp.y;
        cp_point.z = cp.z;
        line.points.push_back(cp_point);
    }
    geometry_msgs::msg::Point p_to;
    p_to.x = to_it->second.pose.x;
    p_to.y = to_it->second.pose.y;
    p_to.z = to_it->second.pose.z;
    line.points.push_back(p_to);
    markers.markers.push_back(line);

    visualization_msgs::msg::Marker text;
    text.header.frame_id = "map";
    text.header.stamp = stamp;
    text.ns = "xhu_active_edge";
    text.id = 2;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;
    text.scale.z = 0.25;
    text.color = color;
    text.pose.position.x = (from_it->second.pose.x + to_it->second.pose.x) * 0.5;
    text.pose.position.y = (from_it->second.pose.y + to_it->second.pose.y) * 0.5;
    text.pose.position.z = std::max(from_it->second.pose.z, to_it->second.pose.z) + 0.35;
    text.text = edge.id + " [" + gate_status + "]";
    markers.markers.push_back(text);

    return markers;
}

void Diagnostics::publishActiveEdge(
    const GraphEdge& edge,
    const FieldGraph& graph,
    const std::string& gate_status) {
    std_msgs::msg::String edge_msg;
    edge_msg.data = edge.id + ":" + edge.from + "->" + edge.to;
    active_edge_pub_->publish(edge_msg);

    std_msgs::msg::String gate_msg;
    gate_msg.data = gate_status;
    semantic_gate_pub_->publish(gate_msg);

    risk_markers_pub_->publish(buildRiskMarkers(edge, graph, gate_status));
}

}  // namespace rc26_topo_nav
