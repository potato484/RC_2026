// 武馆区(MC)运行参数集合：启动时来自 r2_runtime.yaml/launch，经 loadMCParams 写入黑板。
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

    // 夹取（/mechanism/send_command 下发 GRAB_TIP）
    int grab_command_id{1};
    std::vector<uint8_t> grab_payload;
    std::string grab_service_name{"/mechanism/send_command"};
    std::string grab_limit_switch_feedback_topic{"/mechanism/command_feedback"};
    int grab_limit_switch_feedback_id{25};
    double grab_approach_speed_mps{0.04};
    double grab_approach_timeout_s{5.0};
    double grab_done_lost_time_s{1.5};  // 夹取后端头持续消失多久判定完成
    double servo_timeout_s{60.0};       // 视觉伺服整体安全超时

    // 原地旋转
    double rotate_angle_deg{180.0};
    double rotate_speed_radps{0.6};
    int rotate_direction{1};  // +1 逆时针 / -1 顺时针
    double rotate_yaw_tolerance_deg{3.0};
    std::string rotate_cmd_vel_topic{"cmd_vel"};
    std::string odom_topic{"merge_odom"};
    double rotate_timeout_s{15.0};
};

}  // namespace rc26_decision
