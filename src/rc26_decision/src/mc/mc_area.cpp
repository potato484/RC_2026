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
#include "rotate_in_place.hpp"
#include "visual_servo_grab.hpp"
#include "wait_forever.hpp"

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

    // 导航目标（写入黑板键供 mc_tree.xml 端口重映射）
    const double nav_x = node.declare_parameter<double>("mc_nav_x", 0.0);
    const double nav_y = node.declare_parameter<double>("mc_nav_y", 0.0);
    const double nav_yaw = node.declare_parameter<double>("mc_nav_yaw", 0.0);
    const std::string nav_frame = node.declare_parameter<std::string>("mc_nav_frame_id", "map");
    const double nav_timeout = node.declare_parameter<double>("mc_nav_timeout_sec", 60.0);

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
    p.align_command_rate_hz = node.declare_parameter<double>("mc_align_command_rate_hz", p.align_command_rate_hz);
    p.align_lost_stop_frames =
        node.declare_parameter<int>("mc_align_lost_stop_frames", p.align_lost_stop_frames);
    p.align_target_lock_enable =
        node.declare_parameter<bool>("mc_align_target_lock_enable", p.align_target_lock_enable);
    p.align_target_lock_max_jump_px =
        node.declare_parameter<int>("mc_align_target_lock_max_jump_px", p.align_target_lock_max_jump_px);
    p.align_invert_direction = node.declare_parameter<bool>("mc_align_invert_direction", p.align_invert_direction);
    p.align_heading_hold_enable =
        node.declare_parameter<bool>("mc_align_heading_hold_enable", p.align_heading_hold_enable);
    p.align_target_yaw_rad = node.declare_parameter<double>("mc_align_target_yaw_rad", nav_yaw);
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
    p.rotate_yaw_tolerance_deg = node.declare_parameter<double>("mc_rotate_yaw_tolerance_deg", p.rotate_yaw_tolerance_deg);
    p.rotate_cmd_vel_topic = node.declare_parameter<std::string>("mc_rotate_cmd_vel_topic", p.rotate_cmd_vel_topic);
    p.odom_topic = node.declare_parameter<std::string>("mc_odom_topic", p.odom_topic);
    p.rotate_odom_timeout_s =
        node.declare_parameter<double>("mc_rotate_odom_timeout_s", p.rotate_odom_timeout_s);
    p.rotate_timeout_s = node.declare_parameter<double>("mc_rotate_timeout_s", p.rotate_timeout_s);

    p.align_heading_kp = std::max(0.0, p.align_heading_kp);
    p.align_heading_max_speed_radps = std::max(0.0, p.align_heading_max_speed_radps);
    p.align_heading_tolerance_deg = std::max(0.0, p.align_heading_tolerance_deg);
    p.align_heading_gate_deg = std::max(p.align_heading_tolerance_deg, p.align_heading_gate_deg);
    p.align_odom_timeout_s = std::max(0.001, p.align_odom_timeout_s);
    p.rotate_speed_radps = std::max(0.0, std::abs(p.rotate_speed_radps));
    p.rotate_min_speed_radps = std::max(0.0, std::abs(p.rotate_min_speed_radps));
    p.rotate_slowdown_angle_deg = std::max(0.0, p.rotate_slowdown_angle_deg);
    p.rotate_odom_timeout_s = std::max(0.001, p.rotate_odom_timeout_s);

    p.vision_config_file = resolveVisionConfig(p.vision_config_file);

    blackboard->set("mc_params", p);
    blackboard->set("mc_nav_x", nav_x);
    blackboard->set("mc_nav_y", nav_y);
    blackboard->set("mc_nav_yaw", nav_yaw);
    blackboard->set("mc_nav_frame_id", nav_frame);
    blackboard->set("mc_nav_timeout_sec", nav_timeout);

    RCLCPP_INFO(node.get_logger(), "武馆区参数已加载: vision_config=%s target=(%.2f,%.2f)",
                p.vision_config_file.c_str(), nav_x, nav_y);
}

void registerMCAreaNodes(BT::BehaviorTreeFactory& factory) {
    factory.registerNodeType<VisualServoGrabAction>("VisualServoGrab");
    factory.registerNodeType<RotateInPlaceAction>("RotateInPlace");
    factory.registerNodeType<WaitForeverAction>("WaitForever");
}

}  // namespace rc26_decision
