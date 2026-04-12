#pragma once

#include "rc26_xhu_nav/topology/types.hpp"
#include "rc26_xhu_nav/topology/planner.hpp"
#include <atomic>
#include <functional>
#include <mutex>
#include <optional>
#include <unordered_map>
#include <utility>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rc26_interfaces/srv/set_xhu_motion_mode.hpp>
#include <rc26_interfaces/msg/xhu_semantic_corridor.hpp>
#include <rc26_interfaces/msg/xhu_tracking_state.hpp>

namespace rc26_xhu_nav::topology {

enum class EdgeExecState : uint8_t {
    IDLE,
    EXECUTING,
    LOCAL_RETRY,
    SUCCEEDED,
    FAILED,
    REPLAN_REQUESTED
};

class EdgeExecutor {
public:
    struct ExecProgress {
        std::string corridor_id;
        std::string edge_id;
        std::string exec_state;
        std::string reason;
        bool terminal = false;
    };

    using ProgressCallback = std::function<void(const ExecProgress&)>;

    explicit EdgeExecutor(rclcpp::Node* node);

    struct ExecResult {
        bool success = false;
        EdgeExecState final_state = EdgeExecState::FAILED;
        std::string exec_state;
        std::string corridor_id;
        std::string edge_id;
        std::string failure_code;
        std::string failure_reason;
    };

    ExecResult executeEdge(
        const FieldGraph& graph,
        const GraphEdge& edge);

    ExecResult executeCorridor(
        const std::string& corridor_label,
        const std::string& from_node_id,
        const std::string& to_node_id,
        const std::string& motion_type,
        const std::string& required_mode,
        const nav_msgs::msg::Path& corridor_path);

    nav_msgs::msg::Path generateCorridor(
        const FieldGraph& graph,
        const GraphEdge& edge) const;

    void cancel();
    bool requestMode(const std::string& profile, const std::string& reason, std::string& error);
    EdgeExecState state() const { return state_; }
    bool usingXhuBackend() const { return true; }
    void setProgressCallback(ProgressCallback callback);

private:
    struct TrackingEntry {
        rc26_interfaces::msg::XhuTrackingState state;
        rclcpp::Time received_at;
    };

    void onTrackingState(const rc26_interfaces::msg::XhuTrackingState::SharedPtr msg);
    std::optional<rc26_interfaces::msg::XhuTrackingState> latestTrackingState(
        const std::string& corridor_id) const;
    void clearTrackingState(const std::string& corridor_id);
    void pruneTrackingStatesLocked(const rclcpp::Time& now);
    void notifyProgress(const ExecProgress& progress) const;
    static std::pair<float, float> inferSpeedLimits(
        const std::string& required_mode,
        const std::string& motion_type);

    ExecResult executeEdgeViaXhu(
        const FieldGraph& graph,
        const GraphEdge& edge);

    ExecResult executeCorridorViaXhu(
        const std::string& corridor_label,
        const std::string& from_node_id,
        const std::string& to_node_id,
        const std::string& motion_type,
        const std::string& required_mode,
        const nav_msgs::msg::Path& corridor_path);

    rclcpp::Node* node_;
    rclcpp::Client<rc26_interfaces::srv::SetXhuMotionMode>::SharedPtr xhu_mode_client_;
    rclcpp::Publisher<rc26_interfaces::msg::XhuSemanticCorridor>::SharedPtr corridor_pub_;
    rclcpp::Subscription<rc26_interfaces::msg::XhuTrackingState>::SharedPtr tracking_sub_;
    mutable std::mutex tracking_mutex_;
    std::unordered_map<std::string, TrackingEntry> tracking_state_map_;
    mutable std::mutex progress_mutex_;
    ProgressCallback progress_callback_;
    std::atomic<uint64_t> corridor_seq_{0};
    EdgeExecState state_ = EdgeExecState::IDLE;
    std::atomic<bool> cancelled_{false};
    double xhu_exec_timeout_sec_ = 45.0;
    double xhu_hold_replan_timeout_sec_ = 2.0;
    double xhu_corridor_accept_timeout_sec_ = 2.0;
    double xhu_corridor_republish_period_sec_ = 0.5;
    double xhu_tracking_state_timeout_sec_ = 1.5;
    double xhu_wait_on_block_timeout_sec_ = 2.0;
    double xhu_recovery_timeout_sec_ = 3.0;
    double tracking_state_ttl_sec_ = 120.0;

    static constexpr double INTERPOLATION_SPACING = 0.10;
};

}  // namespace rc26_xhu_nav::topology
