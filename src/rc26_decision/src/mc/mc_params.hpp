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

    // MC 末尾视觉 gate：背景配准到启动基准帧后，确认中央夹取区域出现稳定端头前景。
    bool registration_gate_enable{true};
    std::string registration_reference_blackboard_key{"mc_registration_reference_gray"};
    double registration_capture_settle_s{0.2};
    int registration_stable_frames{5};
    double registration_detect_timeout_s{120.0};
    double registration_log_period_s{1.0};
    double registration_roi_x_min_ratio{0.0};
    double registration_roi_x_max_ratio{1.0};
    double registration_roi_y_min_ratio{0.42};
    double registration_roi_y_max_ratio{0.78};
    double registration_foreground_roi_x_min_ratio{0.20};
    double registration_foreground_roi_x_max_ratio{0.80};
    double registration_foreground_roi_y_min_ratio{0.42};
    double registration_foreground_roi_y_max_ratio{0.95};
    int registration_max_corners{400};
    double registration_quality_level{0.01};
    double registration_min_distance_px{8.0};
    int registration_min_inliers{16};
    double registration_max_reproj_error_px{4.0};
    int registration_diff_threshold{28};
    int registration_min_changed_area_px{24000};
    double registration_min_changed_ratio{0.20};
    int registration_min_changed_bbox_height_px{180};
    int registration_min_changed_bbox_width_px{80};
    double registration_min_match_score{0.20};

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
