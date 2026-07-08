// 武馆区 (MC Area) 节点注册与参数加载。
#include "rc26_decision/mc/mc_area.hpp"

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <cmath>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "mc_params.hpp"
#include "mc_preselection_repeat_control.hpp"
#include "preselection_branch_gate.hpp"
#include "rotate_in_place.hpp"
#include "visual_servo_grab.hpp"
#include "wait_for_registration_confirm.hpp"
#include "wait_start_signal_and_notify.hpp"
#include "wait_forever.hpp"
#include "rc26_decision/team_color.hpp"

namespace rc26_decision {

namespace {

// 解析视觉配置路径：绝对路径或存在即用，否则回退到 rc26_vision 包 share 目录。
std::string resolveVisionConfig(const std::string& configured) {
    namespace fs = std::filesystem;
    if (!configured.empty() && fs::exists(configured)) {
        return fs::path(configured).lexically_normal().string();
    }
    try {
        const fs::path share = ament_index_cpp::get_package_share_directory("rc26_vision");
        const fs::path candidate =
            configured.empty() ? (share / "config" / "vision_models.yaml") : (share / configured);
        if (fs::exists(candidate)) {
            return candidate.lexically_normal().string();
        }
    } catch (...) {
    }
    return configured;
}

}  // namespace

void loadMCParams(rclcpp::Node& node, const BT::Blackboard::Ptr& blackboard) {
    McParams p;
    int mirror_sign = 1;
    if (blackboard) {
        (void)blackboard->get("team_mirror_sign", mirror_sign);
    }
    mirror_sign = normalizedMirrorSign(mirror_sign);

    // 去程 odom 相对分段导航参数（写入黑板键供 mc_tree.xml 端口重映射）
    const double nav_forward_x_m = node.declare_parameter<double>("mc_nav_forward_x_m", 0.2);
    const double configured_nav_right_turn_delta_rad =
        node.declare_parameter<double>("mc_nav_right_turn_delta_rad", -1.5707963267948966);
    const double nav_right_turn_delta_rad =
        configured_nav_right_turn_delta_rad * static_cast<double>(mirror_sign);
    const double nav_reverse_x_m = node.declare_parameter<double>("mc_nav_reverse_x_m", -0.6);
    const double nav_timeout = node.declare_parameter<double>("mc_nav_timeout_sec", 60.0);
    const bool preselection_repeat_enable =
        node.declare_parameter<bool>("first_preselection_mc_repeat_enable", true);
    int preselection_repeat_max_count =
        node.declare_parameter<int>("first_preselection_mc_repeat_max_count", 1);
    double preselection_repeat_forward_x_step_m =
        node.declare_parameter<double>("first_preselection_mc_repeat_forward_x_step_m", 0.2);

    // 相机 / 推理
    p.vision_config_file = node.declare_parameter<std::string>("mc_vision_config_file", "");
    p.model_id = node.declare_parameter<std::string>("mc_model_id", p.model_id);
    p.camera_index = node.declare_parameter<int>("mc_camera_index", p.camera_index);
    p.camera_device = node.declare_parameter<std::string>("mc_camera_device", "");
    p.auto_scan_camera = node.declare_parameter<bool>("mc_auto_scan_camera", p.auto_scan_camera);
    p.width = node.declare_parameter<int>("mc_width", p.width);
    p.height = node.declare_parameter<int>("mc_height", p.height);
    p.fps = node.declare_parameter<int>("mc_fps", p.fps);
    p.target_labels = node.declare_parameter<std::vector<std::string>>("mc_target_labels", p.target_labels);

    // 视觉对齐
    p.align_cmd_vel_topic = node.declare_parameter<std::string>("mc_align_cmd_vel_topic", p.align_cmd_vel_topic);
    p.align_tolerance_px = node.declare_parameter<int>("mc_align_tolerance_px", p.align_tolerance_px);
    p.align_stable_frames = node.declare_parameter<int>("mc_align_stable_frames", p.align_stable_frames);
    p.align_kp = node.declare_parameter<double>("mc_align_kp", p.align_kp);
    p.align_min_speed_mps = node.declare_parameter<double>("mc_align_min_speed_mps", p.align_min_speed_mps);
    p.align_max_speed_mps = node.declare_parameter<double>("mc_align_max_speed_mps", p.align_max_speed_mps);
    p.align_search_speed_mps =
        node.declare_parameter<double>("mc_align_search_speed_mps", p.align_search_speed_mps);
    p.align_command_rate_hz = node.declare_parameter<double>("mc_align_command_rate_hz", p.align_command_rate_hz);
    p.align_lost_stop_frames =
        node.declare_parameter<int>("mc_align_lost_stop_frames", p.align_lost_stop_frames);
    p.align_lost_servo_speed_scale =
        node.declare_parameter<double>("mc_align_lost_servo_speed_scale",
                                       p.align_lost_servo_speed_scale);
    p.align_offset_filter_alpha =
        node.declare_parameter<double>("mc_align_offset_filter_alpha",
                                       p.align_offset_filter_alpha);
    p.align_target_lock_enable =
        node.declare_parameter<bool>("mc_align_target_lock_enable", p.align_target_lock_enable);
    p.align_target_lock_max_jump_px =
        node.declare_parameter<int>("mc_align_target_lock_max_jump_px", p.align_target_lock_max_jump_px);
    p.align_invert_direction = node.declare_parameter<bool>("mc_align_invert_direction", p.align_invert_direction);
    p.align_heading_hold_enable =
        node.declare_parameter<bool>("mc_align_heading_hold_enable", p.align_heading_hold_enable);
    p.align_target_yaw_rad = node.declare_parameter<double>("mc_align_target_yaw_rad", 0.0);
    p.align_heading_kp = node.declare_parameter<double>("mc_align_heading_kp", p.align_heading_kp);
    p.align_heading_max_speed_radps =
        node.declare_parameter<double>("mc_align_heading_max_speed_radps", p.align_heading_max_speed_radps);
    p.align_heading_tolerance_deg =
        node.declare_parameter<double>("mc_align_heading_tolerance_deg", p.align_heading_tolerance_deg);
    p.align_heading_gate_deg =
        node.declare_parameter<double>("mc_align_heading_gate_deg", p.align_heading_gate_deg);
    p.align_odom_timeout_s =
        node.declare_parameter<double>("mc_align_odom_timeout_s", p.align_odom_timeout_s);

    // 夹取
    p.grab_command_id = node.declare_parameter<int>("mc_grab_command_id", p.grab_command_id);
    const auto payload = node.declare_parameter<std::vector<int64_t>>("mc_grab_payload", std::vector<int64_t>{});
    p.grab_payload.clear();
    for (const auto v : payload) {
        p.grab_payload.push_back(static_cast<uint8_t>(v & 0xFF));
    }
    p.grab_service_name = node.declare_parameter<std::string>("mc_grab_service_name", p.grab_service_name);
    p.grab_limit_switch_feedback_topic =
        node.declare_parameter<std::string>("mc_grab_limit_switch_feedback_topic",
                                            p.grab_limit_switch_feedback_topic);
    p.grab_limit_switch_feedback_id =
        node.declare_parameter<int>("mc_grab_limit_switch_feedback_id", p.grab_limit_switch_feedback_id);
    p.grab_approach_speed_mps =
        node.declare_parameter<double>("mc_grab_approach_speed_mps", p.grab_approach_speed_mps);
    p.grab_approach_timeout_s =
        node.declare_parameter<double>("mc_grab_approach_timeout_s", p.grab_approach_timeout_s);
    p.grab_done_lost_time_s = node.declare_parameter<double>("mc_grab_done_lost_time_s", p.grab_done_lost_time_s);
    p.servo_timeout_s = node.declare_parameter<double>("mc_servo_timeout_s", p.servo_timeout_s);

    p.grab_limit_switch_feedback_id = std::clamp(p.grab_limit_switch_feedback_id, 0, 255);
    p.grab_approach_speed_mps = std::max(0.0, std::abs(p.grab_approach_speed_mps));
    p.grab_approach_timeout_s = std::max(0.001, p.grab_approach_timeout_s);

    // 原地旋转
    p.rotate_angle_deg = node.declare_parameter<double>("mc_rotate_angle_deg", p.rotate_angle_deg);
    p.rotate_speed_radps = node.declare_parameter<double>("mc_rotate_speed_radps", p.rotate_speed_radps);
    p.rotate_min_speed_radps =
        node.declare_parameter<double>("mc_rotate_min_speed_radps", p.rotate_min_speed_radps);
    p.rotate_slowdown_angle_deg =
        node.declare_parameter<double>("mc_rotate_slowdown_angle_deg", p.rotate_slowdown_angle_deg);
    p.rotate_direction = node.declare_parameter<int>("mc_rotate_direction", p.rotate_direction);
    p.rotate_direction = (p.rotate_direction < 0 ? -1 : 1) * mirror_sign;
    p.rotate_yaw_tolerance_deg = node.declare_parameter<double>("mc_rotate_yaw_tolerance_deg", p.rotate_yaw_tolerance_deg);
    p.rotate_cmd_vel_topic = node.declare_parameter<std::string>("mc_rotate_cmd_vel_topic", p.rotate_cmd_vel_topic);
    p.odom_topic = node.declare_parameter<std::string>("mc_odom_topic", p.odom_topic);
    p.rotate_odom_timeout_s =
        node.declare_parameter<double>("mc_rotate_odom_timeout_s", p.rotate_odom_timeout_s);
    p.rotate_timeout_s = node.declare_parameter<double>("mc_rotate_timeout_s", p.rotate_timeout_s);

    // MC 末尾视觉 gate：启动时采集基准帧，MC 结束时配准差分确认端头场景变化。
    p.registration_gate_enable =
        node.declare_parameter<bool>("mc_registration_gate_enable", p.registration_gate_enable);
    p.registration_reference_blackboard_key =
        node.declare_parameter<std::string>("mc_registration_reference_blackboard_key",
                                            p.registration_reference_blackboard_key);
    p.registration_capture_settle_s =
        node.declare_parameter<double>("mc_registration_capture_settle_s",
                                       p.registration_capture_settle_s);
    p.registration_stable_frames =
        node.declare_parameter<int>("mc_registration_stable_frames",
                                    p.registration_stable_frames);
    p.registration_detect_timeout_s =
        node.declare_parameter<double>("mc_registration_detect_timeout_s",
                                       p.registration_detect_timeout_s);
    p.registration_log_period_s =
        node.declare_parameter<double>("mc_registration_log_period_s",
                                       p.registration_log_period_s);
    p.registration_roi_x_min_ratio =
        node.declare_parameter<double>("mc_registration_roi_x_min_ratio",
                                       p.registration_roi_x_min_ratio);
    p.registration_roi_x_max_ratio =
        node.declare_parameter<double>("mc_registration_roi_x_max_ratio",
                                       p.registration_roi_x_max_ratio);
    p.registration_roi_y_min_ratio =
        node.declare_parameter<double>("mc_registration_roi_y_min_ratio",
                                       p.registration_roi_y_min_ratio);
    p.registration_roi_y_max_ratio =
        node.declare_parameter<double>("mc_registration_roi_y_max_ratio",
                                       p.registration_roi_y_max_ratio);
    p.registration_foreground_roi_x_min_ratio =
        node.declare_parameter<double>("mc_registration_foreground_roi_x_min_ratio",
                                       p.registration_foreground_roi_x_min_ratio);
    p.registration_foreground_roi_x_max_ratio =
        node.declare_parameter<double>("mc_registration_foreground_roi_x_max_ratio",
                                       p.registration_foreground_roi_x_max_ratio);
    p.registration_foreground_roi_y_min_ratio =
        node.declare_parameter<double>("mc_registration_foreground_roi_y_min_ratio",
                                       p.registration_foreground_roi_y_min_ratio);
    p.registration_foreground_roi_y_max_ratio =
        node.declare_parameter<double>("mc_registration_foreground_roi_y_max_ratio",
                                       p.registration_foreground_roi_y_max_ratio);
    p.registration_max_corners =
        node.declare_parameter<int>("mc_registration_max_corners",
                                    p.registration_max_corners);
    p.registration_quality_level =
        node.declare_parameter<double>("mc_registration_quality_level",
                                       p.registration_quality_level);
    p.registration_min_distance_px =
        node.declare_parameter<double>("mc_registration_min_distance_px",
                                       p.registration_min_distance_px);
    p.registration_min_inliers =
        node.declare_parameter<int>("mc_registration_min_inliers",
                                    p.registration_min_inliers);
    p.registration_max_reproj_error_px =
        node.declare_parameter<double>("mc_registration_max_reproj_error_px",
                                       p.registration_max_reproj_error_px);
    p.registration_diff_threshold =
        node.declare_parameter<int>("mc_registration_diff_threshold",
                                    p.registration_diff_threshold);
    p.registration_min_changed_area_px =
        node.declare_parameter<int>("mc_registration_min_changed_area_px",
                                    p.registration_min_changed_area_px);
    p.registration_min_changed_ratio =
        node.declare_parameter<double>("mc_registration_min_changed_ratio",
                                       p.registration_min_changed_ratio);
    p.registration_min_changed_bbox_height_px =
        node.declare_parameter<int>("mc_registration_min_changed_bbox_height_px",
                                    p.registration_min_changed_bbox_height_px);
    p.registration_min_changed_bbox_width_px =
        node.declare_parameter<int>("mc_registration_min_changed_bbox_width_px",
                                    p.registration_min_changed_bbox_width_px);
    p.registration_min_match_score =
        node.declare_parameter<double>("mc_registration_min_match_score",
                                       p.registration_min_match_score);

    // 预选入口 branch gate：0x06 继续，0x10 切树。
    int preselection_entry_continue_delay_msec =
        node.declare_parameter<int>("preselection_entry_continue_delay_msec", 500);
    int preselection_after_mc_continue_delay_msec =
        node.declare_parameter<int>("preselection_after_mc_continue_delay_msec", 500);

    // 组合树启动 gate：等待人工 0x06 后通知下位机比赛开始。
    p.start_signal_feedback_topic =
        node.declare_parameter<std::string>("mc_mf_start_signal_feedback_topic",
                                            p.start_signal_feedback_topic);
    p.start_signal_feedback_id =
        node.declare_parameter<int>("mc_mf_start_signal_feedback_id",
                                    p.start_signal_feedback_id);
    p.start_signal_timeout_s =
        node.declare_parameter<double>("mc_mf_start_signal_timeout_s",
                                       p.start_signal_timeout_s);
    p.start_command_service =
        node.declare_parameter<std::string>("mc_mf_start_command_service",
                                            p.start_command_service);
    p.start_command_id =
        node.declare_parameter<int>("mc_mf_start_command_id", p.start_command_id);
    p.start_command_timeout_s =
        node.declare_parameter<double>("mc_mf_start_command_timeout_s",
                                       p.start_command_timeout_s);
    p.start_done_feedback_id =
        node.declare_parameter<int>("mc_mf_start_done_feedback_id",
                                    p.start_done_feedback_id);
    p.start_done_timeout_s =
        node.declare_parameter<double>("mc_mf_start_done_timeout_s",
                                       p.start_done_timeout_s);
    p.start_log_period_s =
        node.declare_parameter<double>("mc_mf_start_log_period_s", p.start_log_period_s);
    preselection_entry_continue_delay_msec =
        std::max(0, preselection_entry_continue_delay_msec);
    preselection_after_mc_continue_delay_msec =
        std::max(0, preselection_after_mc_continue_delay_msec);
    const auto preselection_entry_continue_delay_msec_bt =
        static_cast<unsigned int>(preselection_entry_continue_delay_msec);
    const auto preselection_after_mc_continue_delay_msec_bt =
        static_cast<unsigned int>(preselection_after_mc_continue_delay_msec);

    p.align_tolerance_px = std::max(0, p.align_tolerance_px);
    p.align_stable_frames = std::max(1, p.align_stable_frames);
    p.align_kp = std::max(0.0, p.align_kp);
    p.align_min_speed_mps = std::max(0.0, std::abs(p.align_min_speed_mps));
    p.align_max_speed_mps = std::max(0.0, std::abs(p.align_max_speed_mps));
    if (p.align_min_speed_mps > p.align_max_speed_mps) {
        p.align_min_speed_mps = p.align_max_speed_mps;
    }
    p.align_search_speed_mps =
        std::min(std::abs(p.align_search_speed_mps), p.align_max_speed_mps) *
        static_cast<double>(mirror_sign);
    p.align_command_rate_hz = std::max(1e-6, p.align_command_rate_hz);
    p.align_lost_stop_frames = std::max(1, p.align_lost_stop_frames);
    p.align_lost_servo_speed_scale =
        std::isfinite(p.align_lost_servo_speed_scale)
            ? std::clamp(p.align_lost_servo_speed_scale, 0.0, 1.0)
            : McParams{}.align_lost_servo_speed_scale;
    p.align_offset_filter_alpha =
        std::isfinite(p.align_offset_filter_alpha)
            ? std::clamp(p.align_offset_filter_alpha, 0.05, 1.0)
            : McParams{}.align_offset_filter_alpha;
    p.align_target_lock_max_jump_px = std::max(0, p.align_target_lock_max_jump_px);
    p.align_heading_kp = std::max(0.0, p.align_heading_kp);
    p.align_heading_max_speed_radps = std::max(0.0, p.align_heading_max_speed_radps);
    p.align_heading_tolerance_deg = std::max(0.0, p.align_heading_tolerance_deg);
    p.align_heading_gate_deg = std::max(p.align_heading_tolerance_deg, p.align_heading_gate_deg);
    p.align_odom_timeout_s = std::max(0.001, p.align_odom_timeout_s);
    p.rotate_speed_radps = std::max(0.0, std::abs(p.rotate_speed_radps));
    p.rotate_min_speed_radps = std::max(0.0, std::abs(p.rotate_min_speed_radps));
    p.rotate_slowdown_angle_deg = std::max(0.0, p.rotate_slowdown_angle_deg);
    p.rotate_odom_timeout_s = std::max(0.001, p.rotate_odom_timeout_s);
    p.registration_capture_settle_s = std::max(0.0, p.registration_capture_settle_s);
    p.registration_stable_frames = std::max(1, p.registration_stable_frames);
    p.registration_detect_timeout_s = std::max(0.001, p.registration_detect_timeout_s);
    p.registration_log_period_s = std::max(0.1, p.registration_log_period_s);
    p.registration_roi_x_min_ratio = std::clamp(p.registration_roi_x_min_ratio, 0.0, 1.0);
    p.registration_roi_x_max_ratio = std::clamp(p.registration_roi_x_max_ratio, 0.0, 1.0);
    p.registration_roi_y_min_ratio = std::clamp(p.registration_roi_y_min_ratio, 0.0, 1.0);
    p.registration_roi_y_max_ratio = std::clamp(p.registration_roi_y_max_ratio, 0.0, 1.0);
    p.registration_foreground_roi_x_min_ratio =
        std::clamp(p.registration_foreground_roi_x_min_ratio, 0.0, 1.0);
    p.registration_foreground_roi_x_max_ratio =
        std::clamp(p.registration_foreground_roi_x_max_ratio, 0.0, 1.0);
    p.registration_foreground_roi_y_min_ratio =
        std::clamp(p.registration_foreground_roi_y_min_ratio, 0.0, 1.0);
    p.registration_foreground_roi_y_max_ratio =
        std::clamp(p.registration_foreground_roi_y_max_ratio, 0.0, 1.0);
    if (p.registration_roi_x_max_ratio <= p.registration_roi_x_min_ratio) {
        p.registration_roi_x_min_ratio = 0.0;
        p.registration_roi_x_max_ratio = 1.0;
    }
    if (p.registration_roi_y_max_ratio <= p.registration_roi_y_min_ratio) {
        p.registration_roi_y_min_ratio = 0.0;
        p.registration_roi_y_max_ratio = 1.0;
    }
    if (p.registration_foreground_roi_x_max_ratio <=
        p.registration_foreground_roi_x_min_ratio) {
        p.registration_foreground_roi_x_min_ratio = 0.20;
        p.registration_foreground_roi_x_max_ratio = 0.80;
    }
    if (p.registration_foreground_roi_y_max_ratio <=
        p.registration_foreground_roi_y_min_ratio) {
        p.registration_foreground_roi_y_min_ratio = 0.30;
        p.registration_foreground_roi_y_max_ratio = 0.95;
    }
    p.registration_max_corners = std::max(16, p.registration_max_corners);
    p.registration_quality_level = std::clamp(p.registration_quality_level, 0.001, 0.2);
    p.registration_min_distance_px = std::max(1.0, p.registration_min_distance_px);
    p.registration_min_inliers = std::max(4, p.registration_min_inliers);
    p.registration_max_reproj_error_px = std::max(0.5, p.registration_max_reproj_error_px);
    p.registration_diff_threshold = std::clamp(p.registration_diff_threshold, 1, 255);
    p.registration_min_changed_area_px = std::max(1, p.registration_min_changed_area_px);
    p.registration_min_changed_ratio = std::clamp(p.registration_min_changed_ratio, 0.0, 1.0);
    p.registration_min_changed_bbox_height_px =
        std::max(1, p.registration_min_changed_bbox_height_px);
    p.registration_min_changed_bbox_width_px =
        std::max(1, p.registration_min_changed_bbox_width_px);
    p.registration_min_match_score = std::clamp(p.registration_min_match_score, 0.0, 1.0);
    p.start_signal_feedback_id = std::clamp(p.start_signal_feedback_id, 0, 255);
    p.start_signal_timeout_s = std::max(0.0, p.start_signal_timeout_s);
    p.start_command_id = std::clamp(p.start_command_id, 0, 255);
    p.start_command_timeout_s = std::max(0.001, p.start_command_timeout_s);
    p.start_done_feedback_id = std::clamp(p.start_done_feedback_id, 0, 255);
    p.start_done_timeout_s = std::max(0.001, p.start_done_timeout_s);
    p.start_log_period_s = std::max(0.1, p.start_log_period_s);
    preselection_repeat_max_count = std::max(0, preselection_repeat_max_count);
    if (!std::isfinite(preselection_repeat_forward_x_step_m)) {
        preselection_repeat_forward_x_step_m = 0.2;
    }
    preselection_repeat_forward_x_step_m = std::abs(preselection_repeat_forward_x_step_m);

    p.vision_config_file = resolveVisionConfig(p.vision_config_file);

    blackboard->set("mc_params", p);
    blackboard->set("mc_nav_forward_x_m", nav_forward_x_m);
    blackboard->set("mc_preselection_effective_forward_x_m", nav_forward_x_m);
    blackboard->set("mc_preselection_run_index", 0);
    blackboard->set("mc_preselection_repeat_count", 0);
    blackboard->set("mc_after_gate_accepted_branch", std::string("both"));
    blackboard->set("mc_after_gate_switch_tree_file",
                    std::string("mf_preselection_tree.xml"));
    blackboard->set("first_preselection_mc_repeat_enable", preselection_repeat_enable);
    blackboard->set("first_preselection_mc_repeat_max_count",
                    preselection_repeat_max_count);
    blackboard->set("first_preselection_mc_repeat_forward_x_step_m",
                    preselection_repeat_forward_x_step_m);
    blackboard->set("mc_nav_right_turn_delta_rad", nav_right_turn_delta_rad);
    blackboard->set("mc_nav_reverse_x_m", nav_reverse_x_m);
    blackboard->set("mc_nav_right_turn_target_yaw", 0.0);
    blackboard->set("mc_nav_timeout_sec", nav_timeout);
    blackboard->set("mc_registration_detect_timeout_s", p.registration_detect_timeout_s);
    blackboard->set("mc_registration_stable_frames", p.registration_stable_frames);
    blackboard->set("mc_mf_start_signal_feedback_topic", p.start_signal_feedback_topic);
    blackboard->set("mc_mf_start_signal_feedback_id", p.start_signal_feedback_id);
    blackboard->set("mc_mf_start_signal_timeout_s", p.start_signal_timeout_s);
    blackboard->set("mc_mf_start_command_service", p.start_command_service);
    blackboard->set("mc_mf_start_command_id", p.start_command_id);
    blackboard->set("mc_mf_start_command_timeout_s", p.start_command_timeout_s);
    blackboard->set("mc_mf_start_done_feedback_id", p.start_done_feedback_id);
    blackboard->set("mc_mf_start_done_timeout_s", p.start_done_timeout_s);
    blackboard->set("mc_mf_start_log_period_s", p.start_log_period_s);
    blackboard->set("preselection_entry_continue_delay_msec",
                    preselection_entry_continue_delay_msec_bt);
    blackboard->set("preselection_after_mc_continue_delay_msec",
                    preselection_after_mc_continue_delay_msec_bt);
    RCLCPP_INFO(node.get_logger(),
                "武馆区参数已加载: vision_config=%s mirror_sign=%d relative_nav=+x %.2fm, yaw_delta %.2frad, x %.2fm, repeat=%s max=%d base=mc_nav_forward_x_m step=%.2fm, align_search_vy=%.3f, lost_servo_scale=%.2f, offset_filter_alpha=%.2f, rotate_direction=%d registration_gate=%s bg_roi_x=%.2f-%.2f bg_roi_y=%.2f-%.2f fg_roi_x=%.2f-%.2f fg_roi_y=%.2f-%.2f diff_threshold=%d area>=%d ratio>=%.3f stable=%d start_signal=0x%02X start_cmd=0x%02X start_done=0x%02X entry_delay_ms=%d after_mc_delay_ms=%d",
                p.vision_config_file.c_str(), mirror_sign, nav_forward_x_m,
                nav_right_turn_delta_rad, nav_reverse_x_m,
                preselection_repeat_enable ? "true" : "false",
                preselection_repeat_max_count,
                preselection_repeat_forward_x_step_m,
                p.align_search_speed_mps, p.align_lost_servo_speed_scale,
                p.align_offset_filter_alpha,
                p.rotate_direction, p.registration_gate_enable ? "true" : "false",
                p.registration_roi_x_min_ratio, p.registration_roi_x_max_ratio,
                p.registration_roi_y_min_ratio, p.registration_roi_y_max_ratio,
                p.registration_foreground_roi_x_min_ratio,
                p.registration_foreground_roi_x_max_ratio,
                p.registration_foreground_roi_y_min_ratio,
                p.registration_foreground_roi_y_max_ratio,
                p.registration_diff_threshold, p.registration_min_changed_area_px,
                p.registration_min_changed_ratio, p.registration_stable_frames,
                static_cast<unsigned int>(p.start_signal_feedback_id & 0xFF),
                static_cast<unsigned int>(p.start_command_id & 0xFF),
                static_cast<unsigned int>(p.start_done_feedback_id & 0xFF),
                preselection_entry_continue_delay_msec,
                preselection_after_mc_continue_delay_msec);
}

void registerMCAreaNodes(BT::BehaviorTreeFactory& factory) {
    factory.registerNodeType<VisualServoGrabAction>("VisualServoGrab");
    factory.registerNodeType<RotateInPlaceAction>("RotateInPlace");
    factory.registerNodeType<RotateRetreatAction>("RotateRetreat");
    factory.registerNodeType<MCPreselectionRepeatControl>("MCPreselectionRepeatControl");
    factory.registerNodeType<WaitForeverAction>("WaitForever");
    factory.registerNodeType<PreselectionBranchGateAction>("WaitPreselectionBranchGate");
    factory.registerNodeType<WaitForRegistrationConfirmAction>("WaitForRegistrationConfirm");
    factory.registerNodeType<WaitStartSignalAndNotifyAction>("WaitStartSignalAndNotify");
}

}  // namespace rc26_decision
