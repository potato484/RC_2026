// 武馆区(MC)运行参数集合：启动时来自当前红/蓝运行配置或 launch，经 loadMCParams 写入黑板。
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace rc26_decision {

struct McParams {
    // 相机 / 推理
    std::string vision_config_file;
    std::string model_id{"tip_default"};
    int camera_index{2};
    std::string camera_device;
    bool auto_scan_camera{true};
    int width{640};
    int height{480};
    int fps{30};
    std::vector<std::string> target_labels{"JK"};

    // 视觉对齐（横移 P 控制）
    std::string align_cmd_vel_topic{"cmd_vel"};
    int align_tolerance_px{20};
    int align_stable_frames{3};
    double align_kp{0.0015};
    double align_min_speed_mps{0.04};
    double align_max_speed_mps{0.15};
    double align_command_rate_hz{20.0};
    int align_lost_stop_frames{3};
    bool align_target_lock_enable{true};
    int align_target_lock_max_jump_px{160};
    bool align_invert_direction{true};
    bool align_heading_hold_enable{true};
    double align_target_yaw_rad{0.0};
    double align_heading_kp{1.2};
    double align_heading_max_speed_radps{0.30};
    double align_heading_tolerance_deg{3.0};
    double align_heading_gate_deg{8.0};
    double align_odom_timeout_s{0.5};

    // 夹取（/mechanism/send_command 下发 GRAB_TIP）
    int grab_command_id{1};
    std::vector<uint8_t> grab_payload;
    std::string grab_service_name{"/mechanism/send_command"};
    std::string grab_limit_switch_feedback_topic{"/mechanism/command_feedback"};
    int grab_limit_switch_feedback_id{6};
    double grab_approach_speed_mps{0.04};
    double grab_approach_timeout_s{5.0};
    double grab_done_lost_time_s{1.5};  // 夹取后端头持续消失多久判定完成
    double servo_timeout_s{60.0};       // 视觉伺服整体安全超时

    // 原地旋转
    double rotate_angle_deg{180.0};
    double rotate_speed_radps{0.6};
    double rotate_min_speed_radps{0.12};
    double rotate_slowdown_angle_deg{30.0};
    int rotate_direction{1};  // +1 逆时针 / -1 顺时针
    double rotate_yaw_tolerance_deg{3.0};
    std::string rotate_cmd_vel_topic{"cmd_vel"};
    std::string odom_topic{"odom"};
    double rotate_odom_timeout_s{0.5};
    double rotate_timeout_s{15.0};

    // 红色元素等待（MC 结束后进入 MF 预选前的视觉 gate）
    int red_hue_low1{0};
    int red_hue_high1{10};
    int red_hue_low2{170};
    int red_hue_high2{180};
    int red_saturation_min{80};
    int red_value_min{60};
    int red_min_area_px{1500};
    int red_stable_frames{3};
    double red_detect_timeout_s{120.0};
    double red_log_period_s{1.0};

    // MC + MF 组合树启动 gate（等待人工 0x06，再下发比赛开始 0x10）
    std::string start_signal_feedback_topic{"/mechanism/command_feedback"};
    int start_signal_feedback_id{6};
    double start_signal_timeout_s{0.0};
    std::string start_command_service{"/mechanism/send_command"};
    int start_command_id{0x10};
    double start_command_timeout_s{5.0};
    int start_done_feedback_id{0x0C};
    double start_done_timeout_s{5.0};
    double start_log_period_s{1.0};
};

}  // namespace rc26_decision
