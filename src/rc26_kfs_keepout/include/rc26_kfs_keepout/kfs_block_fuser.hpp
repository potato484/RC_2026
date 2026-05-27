#pragma once

#include <array>
#include <cstdint>
#include <set>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rc26_interfaces/msg/mf_block_overlay.hpp"
#include "rc26_interfaces/msg/mf_block_overlay_cell.hpp"
#include "rc26_interfaces/msg/mf_kfs_state.hpp"
#include "rc26_interfaces/srv/set_keepout_runtime.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8.hpp"

namespace rc26_kfs_keepout {

class KfsBlockFuser : public rclcpp::Node {
public:
    explicit KfsBlockFuser(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    using SetKeepoutRuntime = rc26_interfaces::srv::SetKeepoutRuntime;

    void onKfsState(const rc26_interfaces::msg::MfKfsState::ConstSharedPtr& msg);
    void onForceReleaseGrid(const std_msgs::msg::UInt8::ConstSharedPtr& msg);
    void onRuntimeControl(
        const SetKeepoutRuntime::Request::SharedPtr request,
        SetKeepoutRuntime::Response::SharedPtr response);
    void decayTimer();
    void publishMask();
    void publishBlockOverlay();
    bool publishClearedOutputs();
    void publishHeartbeat();
    void publishDiagnostics();
    bool loadGridLayout(const std::string& yaml_path);
    bool validateGridSpacing(double expected_spacing_m, double tolerance_m, std::string& detail) const;
    bool isSlowGrid(uint8_t grid_id) const;
    void clearKeepoutState();
    nav_msgs::msg::OccupancyGrid buildMaskGrid() const;
    void publishMaskGrid(const nav_msgs::msg::OccupancyGrid& grid);

    // 参数
    double min_confidence_{0.60};
    double inflate_radius_m_{0.60};
    double map_resolution_{0.10};
    double block_thresh_{0.70};   // log-odds 对应 P=0.70
    double free_thresh_{0.35};    // log-odds 对应 P=0.35
    double lo_hit_block_{1.099};  // R1/R2 阻挡证据
    double lo_hit_fake_{0.693};   // FAKE 阻挡证据
    double lo_miss_{-0.693};      // NONE 负证据
    double decay_rate_{2.0};      // per second toward decay_target_prob
    double decay_target_prob_{0.05};
    double ttl_sec_{10.0};
    std::string ttl_mode_{"hard"};
    std::string keepout_shape_{"square"};
    double block_half_size_m_{0.60};
    double keepout_margin_m_{0.03};
    std::string mask_topic_;
    std::string heartbeat_topic_{"/kfs_keepout_heartbeat"};
    std::string grid_layout_file_;
    std::string diagnostics_topic_;
    std::string force_release_topic_;
    std::string layout_team_;
    std::string active_team_;
    std::string layout_version_;
    bool layout_validated_{false};
    double layout_grid_spacing_m_{1.2};
    double grid_spacing_tolerance_m_{0.05};
    std::set<uint8_t> slow_grid_ids_;

    // 状态
    static constexpr int kGridCount = 13;  // index 0 unused, 1..12
    std::array<double, kGridCount> log_odds_{};
    std::array<uint8_t, kGridCount> blocked_state_{};
    std::array<uint8_t, kGridCount> pending_state_{};
    std::array<int, kGridCount> dwell_count_{};
    bool mask_dirty_{false};
    int dwell_cycles_{3};
    std::array<double, kGridCount> cell_x_{};
    std::array<double, kGridCount> cell_y_{};
    std::array<rclcpp::Time, kGridCount> last_hit_time_;
    bool layout_loaded_{false};
    bool keepout_enabled_{false};
    bool runtime_active_{false};
    std::string keepout_disable_reason_;
    uint32_t force_release_count_{0};
    bool team_mismatch_detected_{false};
    std::string runtime_control_service_{"set_runtime"};

    rclcpp::Subscription<rc26_interfaces::msg::MfKfsState>::SharedPtr sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_force_release_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_mask_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diagnostics_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_heartbeat_;
    rclcpp::Publisher<rc26_interfaces::msg::MfBlockOverlay>::SharedPtr pub_block_overlay_;
    rclcpp::Service<SetKeepoutRuntime>::SharedPtr runtime_control_srv_;
    rclcpp::TimerBase::SharedPtr decay_timer_;

    rclcpp::Time last_decay_time_;
};

}  // namespace rc26_kfs_keepout
