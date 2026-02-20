#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rc26_interfaces/msg/mf_kfs_state.hpp"
#include "rc26_interfaces/srv/set_nav_mode.hpp"
#include "std_msgs/msg/u_int8.hpp"

namespace rc26_kfs_keepout {

class KfsBlockFuser : public rclcpp::Node {
public:
    explicit KfsBlockFuser(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    void onKfsState(const rc26_interfaces::msg::MfKfsState::ConstSharedPtr& msg);
    void onForceReleaseGrid(const std_msgs::msg::UInt8::ConstSharedPtr& msg);
    void decayTimer();
    void publishMask();
    void publishDiagnostics();
    bool loadGridLayout(const std::string& yaml_path);
    bool validateGridSpacing(double expected_spacing_m, double tolerance_m, std::string& detail) const;
    bool isBlockingKfsType(uint8_t kfs_type) const;
    void triggerSafeMode(const std::string& reason);

    // 参数
    double min_confidence_{0.60};
    double inflate_radius_m_{0.60};
    double map_resolution_{0.10};
    double block_thresh_{0.70};   // log-odds 对应 P=0.70
    double free_thresh_{0.35};    // log-odds 对应 P=0.35
    double lo_hit_{1.099};        // ln(P/(1-P)) for P=0.75
    double decay_rate_{2.0};      // per second toward decay_target_prob
    double decay_target_prob_{0.05};
    double ttl_sec_{10.0};
    std::string keepout_shape_{"square"};
    double block_half_size_m_{0.60};
    double keepout_margin_m_{0.03};
    std::string mask_topic_;
    std::string grid_layout_file_;
    std::string diagnostics_topic_;
    std::string force_release_topic_;
    std::string layout_team_;
    std::string layout_version_;
    bool layout_validated_{false};
    double layout_grid_spacing_m_{1.2};
    double grid_spacing_tolerance_m_{0.05};

    // 状态
    static constexpr int kGridCount = 13;  // index 0 unused, 1..12
    std::array<double, kGridCount> log_odds_{};
    std::array<uint8_t, kGridCount> blocked_state_{};
    std::array<double, kGridCount> cell_x_{};
    std::array<double, kGridCount> cell_y_{};
    std::array<rclcpp::Time, kGridCount> last_hit_time_;
    bool layout_loaded_{false};
    bool keepout_enabled_{false};
    std::string keepout_disable_reason_;
    uint32_t force_release_count_{0};
    bool team_mismatch_detected_{false};

    rclcpp::Subscription<rc26_interfaces::msg::MfKfsState>::SharedPtr sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_force_release_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_mask_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diagnostics_;
    rclcpp::Client<rc26_interfaces::srv::SetNavMode>::SharedPtr nav_mode_client_;
    rclcpp::TimerBase::SharedPtr decay_timer_;

    rclcpp::Time last_decay_time_;
};

}  // namespace rc26_kfs_keepout
