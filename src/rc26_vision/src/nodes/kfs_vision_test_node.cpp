/// kfs 主链(D455 + kfs.onnx)联调测试节点,带实时 OpenCV overlay 窗口。
/// 推荐用法:
///   ros2 launch rc26_vision test_kfs_vision.launch.py
/// 也可单独运行:
///   ros2 run rc26_vision kfs_vision_test_node --ros-args \
///     -p vision_config_file:=$(ros2 pkg prefix rc26_vision)/share/rc26_vision/config/vision_models.yaml
#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>

#include "rc26_interfaces/msg/mechanism_transport_feedback.hpp"
#include "rc26_interfaces/srv/send_mechanism_transport_command.hpp"
#include "rc26_serial/protocol.hpp"
#include "rc26_vision/inference/config/model_profile_loader.hpp"
#include "rc26_vision/inference/runtime/vision_inference_manager.hpp"
#include "rc26_vision/shared/sensors/depth_roi_sampler.hpp"
#include "rc26_vision/shared/target/visual_target_match.hpp"

namespace rc26_vision {

namespace {

using TwistMsg = geometry_msgs::msg::Twist;
using FeedbackMsg = rc26_interfaces::msg::MechanismTransportFeedback;
using SendCommandSrv = rc26_interfaces::srv::SendMechanismTransportCommand;

constexpr uint8_t kDefaultGrabKfsUpCommand =
    static_cast<uint8_t>(rc26_serial::CommandID::GRAB_KFS_UP);
constexpr uint8_t kDefaultGrabKfsDownCommand =
    static_cast<uint8_t>(rc26_serial::CommandID::GRAB_KFS_DOWN);

std::string detectionName(const Detection& det) {
    return visualTargetLabel(det);
}

cv::Scalar featureColorForLabel(const std::string& label) {
    if (label.rfind("R_", 0) == 0) {
        return cv::Scalar(40, 40, 230);
    }
    if (label.rfind("B_", 0) == 0) {
        return cv::Scalar(230, 110, 40);
    }
    if (label.rfind("T_", 0) == 0) {
        return cv::Scalar(70, 220, 70);
    }
    if (label.rfind("F_", 0) == 0) {
        return cv::Scalar(0, 140, 255);
    }
    return cv::Scalar(220, 220, 220);
}

cv::Scalar labelBackgroundColor(const cv::Scalar& line_color) {
    return cv::Scalar(line_color[0] * 0.35, line_color[1] * 0.35, line_color[2] * 0.35);
}

int clampTextX(int requested_x, int text_width, int frame_width) {
    if (frame_width <= 0) {
        return 0;
    }
    const int max_x = std::max(0, frame_width - text_width - 8);
    return std::clamp(requested_x, 0, max_x);
}

std::string trimCopy(const std::string& input) {
    const auto begin = input.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) {
        return "";
    }
    const auto end = input.find_last_not_of(" \t\r\n");
    return input.substr(begin, end - begin + 1);
}

std::string lowerCopy(std::string input) {
    std::transform(input.begin(), input.end(), input.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return input;
}

bool labelMatches(const std::string& label,
                  const std::vector<std::string>& exact_labels,
                  const std::vector<std::string>& prefixes) {
    for (const auto& exact : exact_labels) {
        if (!exact.empty() && label == exact) {
            return true;
        }
    }
    for (const auto& prefix : prefixes) {
        if (!prefix.empty() && label.rfind(prefix, 0) == 0) {
            return true;
        }
    }
    return false;
}

std::string byteToHex(uint8_t value) {
    std::ostringstream ss;
    ss << "0x" << std::uppercase << std::hex << std::setw(2) << std::setfill('0')
       << static_cast<unsigned int>(value);
    return ss.str();
}

std::vector<std::string> sanitizeStringList(std::vector<std::string> values) {
    std::vector<std::string> result;
    result.reserve(values.size());
    for (auto& value : values) {
        value = trimCopy(value);
        if (!value.empty()) {
            result.push_back(value);
        }
    }
    return result;
}

struct ActionTarget {
    Detection detection;
    cv::Rect box;
    int cx{-1};
    int cy{-1};
    int offset_px{0};
    double depth_m{0.0};
    bool has_depth{false};
};

int boxArea(const cv::Rect& box) {
    return std::max(0, box.width) * std::max(0, box.height);
}

std::optional<ActionTarget> chooseClosestTarget(
    const std::vector<ActionTarget>& targets,
    int frame_center_x) {
    std::optional<ActionTarget> best;
    int best_center_distance = 0;
    int best_area = 0;
    for (const auto& target : targets) {
        const int center_distance = std::abs(target.cx - frame_center_x);
        const int area = boxArea(target.box);
        if (!best.has_value() ||
            center_distance < best_center_distance ||
            (center_distance == best_center_distance && target.detection.score > best->detection.score) ||
            (center_distance == best_center_distance && target.detection.score == best->detection.score &&
             area > best_area)) {
            best = target;
            best_center_distance = center_distance;
            best_area = area;
        }
    }
    return best;
}

std::optional<ActionTarget> chooseClosestToLockedTarget(
    const std::vector<ActionTarget>& targets,
    const ActionTarget& locked_target,
    int max_jump_px) {
    std::optional<ActionTarget> best;
    int best_jump = 0;
    int best_area = 0;
    for (const auto& target : targets) {
        const int jump = std::abs(target.cx - locked_target.cx);
        if (jump > max_jump_px) {
            continue;
        }
        const int area = boxArea(target.box);
        if (!best.has_value() ||
            jump < best_jump ||
            (jump == best_jump && target.detection.score > best->detection.score) ||
            (jump == best_jump && target.detection.score == best->detection.score &&
             area > best_area)) {
            best = target;
            best_jump = jump;
            best_area = area;
        }
    }
    return best;
}

std::vector<ActionTarget> collectActionTargets(
    const VisionInferenceManager::FrameSnapshot& snapshot,
    const DepthRoiSamplerConfig& depth_config,
    const std::vector<std::string>& exact_labels,
    const std::vector<std::string>& prefixes) {
    std::vector<ActionTarget> targets;
    if (!snapshot.has_color || snapshot.color_bgr.empty()) {
        return targets;
    }

    for (const auto& det : snapshot.detections) {
        const std::string name = detectionName(det);
        if (!labelMatches(name, exact_labels, prefixes)) {
            continue;
        }
        const int x1 = static_cast<int>(std::floor(det.x1));
        const int y1 = static_cast<int>(std::floor(det.y1));
        const int x2 = static_cast<int>(std::ceil(det.x2));
        const int y2 = static_cast<int>(std::ceil(det.y2));
        const cv::Rect box(x1, y1, std::max(0, x2 - x1), std::max(0, y2 - y1));
        if (box.width <= 0 || box.height <= 0) {
            continue;
        }

        ActionTarget target;
        target.detection = det;
        target.box = box;
        target.cx = static_cast<int>((det.x1 + det.x2) * 0.5f);
        target.cy = static_cast<int>((det.y1 + det.y2) * 0.5f);
        target.offset_px = target.cx - std::max(0, snapshot.color_bgr.cols / 2);

        if (snapshot.has_depth && !snapshot.depth.empty()) {
            DepthRoiSamplerConfig raw_depth_config = depth_config;
            raw_depth_config.min_depth_m = 0.0;
            raw_depth_config.max_depth_m = std::numeric_limits<double>::infinity();
            const auto sampled = sampleMedianDepth(snapshot.depth, target.cx, target.cy, raw_depth_config);
            if (sampled.has_value()) {
                target.depth_m = *sampled;
                target.has_depth = true;
            }
        }

        targets.push_back(std::move(target));
    }

    return targets;
}

}  // namespace

class VisionTestNode : public rclcpp::Node {
public:
    VisionTestNode() : Node("kfs_vision_test_node") {
        this->declare_parameter<int>("print_rate_ms", 500);
        this->declare_parameter<std::string>("vision_config_file", "");
        this->declare_parameter<std::string>("model_id", "");
        this->declare_parameter<bool>("log_detections", false);
        this->declare_parameter<bool>("log_status", false);
        this->declare_parameter<bool>("show_window", true);
        this->declare_parameter<std::string>("window_name", std::string("KFS Vision - kfs.onnx"));
        this->declare_parameter<int>("display_rate_ms", 33);
        declareActionParameters();

        int print_rate_ms = this->get_parameter("print_rate_ms").as_int();
        if (print_rate_ms <= 0) {
            print_rate_ms = 500;
        }
        const std::string config_file = this->get_parameter("vision_config_file").as_string();
        const std::string requested_model_id = this->get_parameter("model_id").as_string();
        log_detections_ = this->get_parameter("log_detections").as_bool();
        log_status_ = this->get_parameter("log_status").as_bool();
        show_window_ = this->get_parameter("show_window").as_bool();
        window_name_ = this->get_parameter("window_name").as_string();
        int display_rate_ms = this->get_parameter("display_rate_ms").as_int();
        if (display_rate_ms <= 0) {
            display_rate_ms = 33;
        }

        if (config_file.empty()) {
            RCLCPP_FATAL(this->get_logger(), "vision_config_file 参数为空");
            throw std::runtime_error("vision_config_file 参数为空");
        }

        manager_ = std::make_shared<VisionInferenceManager>(*this);

        try {
            auto config = ProfileLoader::loadFromYaml(config_file);
            manager_->loadConfig(config);
            active_model_id_ = requested_model_id.empty() ? config.default_model : requested_model_id;
            if (active_model_id_.empty()) {
                RCLCPP_FATAL(this->get_logger(), "model_id 为空且视觉配置未声明 default_model");
                throw std::runtime_error("model_id 为空且视觉配置未声明 default_model");
            }
            manager_->selectModel(active_model_id_);
            RCLCPP_INFO(this->get_logger(), "视觉配置已加载: %s，模型: %s",
                config_file.c_str(), active_model_id_.c_str());
        } catch (const std::exception& e) {
            RCLCPP_FATAL(this->get_logger(), "视觉配置加载失败: %s", e.what());
            throw;
        }

        depth_config_.roi_size = static_cast<int>(this->get_parameter("vision_depth_roi").as_int());
        depth_config_.min_valid_count =
            static_cast<int>(this->get_parameter("vision_depth_min_valid_count").as_int());
        depth_config_.min_depth_m = this->get_parameter("vision_depth_min_m").as_double();
        depth_config_.max_depth_m = this->get_parameter("vision_depth_max_m").as_double();
        RCLCPP_INFO(this->get_logger(), "KFS UI 深度窗口: roi=%d min_valid=%d range=[%.2f, %.2f]m",
            depth_config_.roi_size, depth_config_.min_valid_count,
            depth_config_.min_depth_m, depth_config_.max_depth_m);

        loadActionParameters();

        manager_->setResultCallback([this](const TargetResult& result) {
            if (log_detections_ && result.has_target) {
                RCLCPP_INFO(this->get_logger(), "[检测] attr=%d dist=%.2fm score=%.2f bbox=(%d,%d)",
                    static_cast<int>(result.attr_kind), result.distance_m, result.score,
                    result.bbox_cx, result.bbox_cy);
            }
        });

        if (!manager_->start()) {
            RCLCPP_FATAL(this->get_logger(), "视觉模块启动失败");
            throw std::runtime_error("视觉模块启动失败");
        }

        RCLCPP_INFO(this->get_logger(), "视觉推理已启动");

        // 仅在需要显示时创建窗口;无显示环境(无 DISPLAY)会抛 cv::Exception,降级为无头。
        if (show_window_) {
            try {
                cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
            } catch (const cv::Exception& e) {
                RCLCPP_WARN(this->get_logger(),
                    "无法创建显示窗口(无显示环境?),降级为无头运行: %s", e.what());
                show_window_ = false;
            }
        }

        if (log_status_) {
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(print_rate_ms),
                [this]() { printStatus(); });
        }

        // 显示定时器在主线程(单线程 spin)执行,满足 OpenCV HighGUI 必须主线程的约束。
        if (show_window_) {
            display_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(display_rate_ms),
                [this]() { renderDisplay(); });
        }

        if (action_enable_) {
            initializeActionRuntime();
        } else {
            RCLCPP_INFO(this->get_logger(), "KFS 动作测试已禁用；节点只做视觉 overlay/日志。");
        }
    }

    ~VisionTestNode() {
        if (action_timer_) {
            action_timer_->cancel();
        }
        publishActionStop(true);
        if (display_timer_) {
            display_timer_->cancel();   // 先停显示回调,避免析构期间再画窗口
        }
        if (manager_) {
            manager_->stop();
        }
        if (show_window_) {
            cv::destroyWindow(window_name_);
        }
    }

private:
    enum class ActionPhase {
        Disabled,
        SendingPrep,
        WaitingPrepDone,
        Search,
        Align,
        WaitingSecondArmLower,
        Approach,
        SendingGrab,
        GrabVerify,
        Succeeded,
        Failed
    };

    void declareActionParameters() {
        this->declare_parameter<bool>("kfs_action_enable", false);
        this->declare_parameter<std::string>("kfs_action_direction", "up");
        this->declare_parameter<std::vector<std::string>>(
            "kfs_action_target_label_prefixes", std::vector<std::string>{"T_"});
        this->declare_parameter<std::vector<std::string>>(
            "kfs_action_target_labels", std::vector<std::string>{});
        this->declare_parameter<std::string>("kfs_action_cmd_vel_topic", "cmd_vel");
        this->declare_parameter<std::string>(
            "kfs_action_send_command_service", "/mechanism/send_command");
        this->declare_parameter<std::string>(
            "kfs_action_feedback_topic", "/mechanism/command_feedback");
        this->declare_parameter<int>("kfs_action_arm_raise_command_id",
            static_cast<int>(rc26_serial::CommandID::ARM_RAISE));
        this->declare_parameter<int>("kfs_action_arm_lower_command_id",
            static_cast<int>(rc26_serial::CommandID::ARM_LOWER));
        this->declare_parameter<int>("kfs_action_arm_raise_done_feedback_id",
            static_cast<int>(rc26_serial::FeedbackID::ARM_RAISE_DONE));
        this->declare_parameter<int>("kfs_action_arm_lower_done_feedback_id",
            static_cast<int>(rc26_serial::FeedbackID::ARM_LOWER_DONE));
        this->declare_parameter<int>("kfs_action_second_arm_lower_command_id",
            static_cast<int>(rc26_serial::CommandID::ARM_SECOND_LOWER));
        this->declare_parameter<int>("kfs_action_second_arm_lower_done_feedback_id",
            static_cast<int>(rc26_serial::FeedbackID::ARM_SECOND_LOWER_DONE));
        this->declare_parameter<double>("kfs_action_service_wait_timeout_s", 5.0);
        this->declare_parameter<int>("kfs_action_arm_prep_service_timeout_ms", 6000);
        this->declare_parameter<double>("kfs_action_arm_prep_done_timeout_s", 10.0);
        this->declare_parameter<int>("kfs_action_align_tolerance_px", 20);
        this->declare_parameter<int>("kfs_action_align_stable_frames", 5);
        this->declare_parameter<double>("kfs_action_align_kp", 0.0010);
        this->declare_parameter<double>("kfs_action_align_min_speed_mps", 0.015);
        this->declare_parameter<double>("kfs_action_align_max_speed_mps", 0.06);
        this->declare_parameter<double>("kfs_action_command_rate_hz", 20.0);
        this->declare_parameter<int>("kfs_action_lost_stop_frames", 3);
        this->declare_parameter<bool>("kfs_action_target_lock_enable", true);
        this->declare_parameter<int>("kfs_action_target_lock_max_jump_px", 160);
        this->declare_parameter<bool>("kfs_action_invert_lateral_direction", false);
        this->declare_parameter<double>("kfs_action_approach_speed_mps", 0.04);
        this->declare_parameter<int>("kfs_action_approach_x_sign", 1);
        this->declare_parameter<double>("kfs_action_approach_timeout_s", 8.0);
        this->declare_parameter<double>("kfs_action_grab_distance_m", 0.70);
        this->declare_parameter<int>("kfs_action_grab_stable_frames", 2);
        this->declare_parameter<double>("kfs_action_total_timeout_s", 40.0);
        this->declare_parameter<int>("kfs_action_grab_service_timeout_ms", 6000);
        this->declare_parameter<double>("kfs_action_grab_verify_timeout_s", 3.0);
        this->declare_parameter<int>("kfs_action_grab_verify_lost_stable_frames", 3);
        this->declare_parameter<double>("kfs_action_grab_verify_iou_threshold", 0.30);
        this->declare_parameter<bool>("kfs_action_grab_once", true);
        this->declare_parameter<int>("kfs_action_grab_kfs_up_command_id", kDefaultGrabKfsUpCommand);
        this->declare_parameter<int>("kfs_action_grab_kfs_down_command_id", kDefaultGrabKfsDownCommand);
    }

    void loadActionParameters() {
        action_enable_ = this->get_parameter("kfs_action_enable").as_bool();
        action_direction_ = lowerCopy(trimCopy(
            this->get_parameter("kfs_action_direction").as_string()));
        if (action_direction_ != "up" && action_direction_ != "down") {
            throw std::runtime_error("kfs_action_direction must be up or down");
        }

        action_target_prefixes_ = sanitizeStringList(
            this->get_parameter("kfs_action_target_label_prefixes").as_string_array());
        action_target_labels_ = sanitizeStringList(
            this->get_parameter("kfs_action_target_labels").as_string_array());
        if (action_enable_ && action_target_prefixes_.empty() && action_target_labels_.empty()) {
            throw std::runtime_error(
                "kfs action target labels/prefixes cannot both be empty when action is enabled");
        }

        action_cmd_vel_topic_ = trimCopy(
            this->get_parameter("kfs_action_cmd_vel_topic").as_string());
        action_send_command_service_ = trimCopy(
            this->get_parameter("kfs_action_send_command_service").as_string());
        action_feedback_topic_ = trimCopy(
            this->get_parameter("kfs_action_feedback_topic").as_string());
        if (action_enable_ && action_cmd_vel_topic_.empty()) {
            throw std::runtime_error("kfs_action_cmd_vel_topic cannot be empty when action is enabled");
        }
        if (action_enable_ && action_send_command_service_.empty()) {
            throw std::runtime_error(
                "kfs_action_send_command_service cannot be empty when action is enabled");
        }
        if (action_enable_ && action_feedback_topic_.empty()) {
            throw std::runtime_error("kfs_action_feedback_topic cannot be empty when action is enabled");
        }

        action_arm_raise_command_id_ = validateCommandId(
            this->get_parameter("kfs_action_arm_raise_command_id").as_int(),
            "kfs_action_arm_raise_command_id");
        action_arm_lower_command_id_ = validateCommandId(
            this->get_parameter("kfs_action_arm_lower_command_id").as_int(),
            "kfs_action_arm_lower_command_id");
        action_arm_raise_done_feedback_id_ = validateCommandId(
            this->get_parameter("kfs_action_arm_raise_done_feedback_id").as_int(),
            "kfs_action_arm_raise_done_feedback_id");
        action_arm_lower_done_feedback_id_ = validateCommandId(
            this->get_parameter("kfs_action_arm_lower_done_feedback_id").as_int(),
            "kfs_action_arm_lower_done_feedback_id");
        action_second_arm_lower_command_id_ = validateCommandId(
            this->get_parameter("kfs_action_second_arm_lower_command_id").as_int(),
            "kfs_action_second_arm_lower_command_id");
        action_second_arm_lower_done_feedback_id_ = validateCommandId(
            this->get_parameter("kfs_action_second_arm_lower_done_feedback_id").as_int(),
            "kfs_action_second_arm_lower_done_feedback_id");
        action_service_wait_timeout_s_ = std::max(
            0.0, this->get_parameter("kfs_action_service_wait_timeout_s").as_double());
        action_arm_prep_service_timeout_ms_ = std::max(1, static_cast<int>(
            this->get_parameter("kfs_action_arm_prep_service_timeout_ms").as_int()));
        action_arm_prep_done_timeout_s_ = std::max(
            0.1, this->get_parameter("kfs_action_arm_prep_done_timeout_s").as_double());

        action_align_tolerance_px_ =
            std::max(0, static_cast<int>(this->get_parameter("kfs_action_align_tolerance_px").as_int()));
        action_align_stable_frames_ =
            std::max(1, static_cast<int>(this->get_parameter("kfs_action_align_stable_frames").as_int()));
        action_align_kp_ = std::max(0.0, this->get_parameter("kfs_action_align_kp").as_double());
        action_align_min_speed_mps_ =
            std::max(0.0, this->get_parameter("kfs_action_align_min_speed_mps").as_double());
        action_align_max_speed_mps_ =
            std::max(0.0, this->get_parameter("kfs_action_align_max_speed_mps").as_double());
        if (action_align_min_speed_mps_ > action_align_max_speed_mps_) {
            action_align_min_speed_mps_ = action_align_max_speed_mps_;
        }
        action_command_rate_hz_ =
            std::max(1.0, this->get_parameter("kfs_action_command_rate_hz").as_double());
        action_lost_stop_frames_ =
            std::max(1, static_cast<int>(this->get_parameter("kfs_action_lost_stop_frames").as_int()));
        action_target_lock_enable_ =
            this->get_parameter("kfs_action_target_lock_enable").as_bool();
        action_target_lock_max_jump_px_ =
            std::max(0, static_cast<int>(
                this->get_parameter("kfs_action_target_lock_max_jump_px").as_int()));
        action_invert_lateral_direction_ =
            this->get_parameter("kfs_action_invert_lateral_direction").as_bool();
        action_approach_speed_mps_ =
            this->get_parameter("kfs_action_approach_speed_mps").as_double();
        if (!std::isfinite(action_approach_speed_mps_) || action_approach_speed_mps_ < 0.0) {
            throw std::runtime_error("kfs_action_approach_speed_mps must be finite and >= 0");
        }
        action_approach_x_sign_ =
            this->get_parameter("kfs_action_approach_x_sign").as_int() < 0 ? -1 : 1;
        action_approach_timeout_s_ =
            this->get_parameter("kfs_action_approach_timeout_s").as_double();
        if (!std::isfinite(action_approach_timeout_s_) || action_approach_timeout_s_ <= 0.0) {
            throw std::runtime_error("kfs_action_approach_timeout_s must be finite and > 0");
        }
        action_grab_distance_m_ =
            this->get_parameter("kfs_action_grab_distance_m").as_double();
        if (!std::isfinite(action_grab_distance_m_) || action_grab_distance_m_ < 0.0) {
            throw std::runtime_error("kfs_action_grab_distance_m must be finite and >= 0");
        }
        action_grab_stable_frames_ =
            std::max(1, static_cast<int>(this->get_parameter("kfs_action_grab_stable_frames").as_int()));
        action_total_timeout_s_ =
            std::max(0.1, this->get_parameter("kfs_action_total_timeout_s").as_double());
        action_grab_service_timeout_ms_ =
            std::max(1, static_cast<int>(
                this->get_parameter("kfs_action_grab_service_timeout_ms").as_int()));
        action_grab_verify_timeout_s_ =
            std::max(0.1, this->get_parameter("kfs_action_grab_verify_timeout_s").as_double());
        action_grab_verify_lost_stable_frames_ = std::max(1, static_cast<int>(
            this->get_parameter("kfs_action_grab_verify_lost_stable_frames").as_int()));
        action_grab_verify_iou_threshold_ = std::clamp(
            this->get_parameter("kfs_action_grab_verify_iou_threshold").as_double(), 0.0, 1.0);
        action_grab_once_ = this->get_parameter("kfs_action_grab_once").as_bool();
        action_grab_kfs_up_command_id_ = validateCommandId(
            this->get_parameter("kfs_action_grab_kfs_up_command_id").as_int(),
            "kfs_action_grab_kfs_up_command_id");
        action_grab_kfs_down_command_id_ = validateCommandId(
            this->get_parameter("kfs_action_grab_kfs_down_command_id").as_int(),
            "kfs_action_grab_kfs_down_command_id");
        action_grab_command_id_ =
            action_direction_ == "down" ? action_grab_kfs_down_command_id_ : action_grab_kfs_up_command_id_;
        action_prep_command_id_ =
            action_direction_ == "down" ? action_arm_lower_command_id_ : action_arm_raise_command_id_;
        action_prep_done_feedback_id_ = action_direction_ == "down"
            ? action_arm_lower_done_feedback_id_
            : action_arm_raise_done_feedback_id_;
        action_prep_label_ = action_direction_ == "down" ? "ARM_LOWER" : "ARM_RAISE";

        if (action_enable_) {
            if (depth_config_.max_depth_m < depth_config_.min_depth_m) {
                throw std::runtime_error(
                    "vision_depth_max_m must be greater than or equal to vision_depth_min_m");
            }
        }
    }

    uint8_t validateCommandId(int value, const char* param_name) const {
        if (value < 0 || value > 255) {
            std::ostringstream ss;
            ss << param_name << " must be in [0,255]";
            throw std::runtime_error(ss.str());
        }
        return static_cast<uint8_t>(value);
    }

    void initializeActionRuntime() {
        action_cmd_pub_ = create_publisher<TwistMsg>(action_cmd_vel_topic_, rclcpp::QoS(10));
        action_send_client_ = create_client<SendCommandSrv>(action_send_command_service_);
        action_feedback_sub_ = create_subscription<FeedbackMsg>(
            action_feedback_topic_, rclcpp::QoS(32).reliable(),
            [this](const FeedbackMsg::SharedPtr msg) {
                if (!msg) {
                    return;
                }
                if (msg->feedback_id == action_prep_done_feedback_id_) {
                    action_latest_prep_done_seq_ = msg->seq;
                    if (action_prep_response_seen_ &&
                        msg->seq == action_prep_response_seq_) {
                        action_prep_done_seen_ = true;
                    }
                }
                if (msg->feedback_id == action_second_arm_lower_done_feedback_id_) {
                    action_latest_second_arm_lower_done_seq_ = msg->seq;
                    if (action_second_arm_lower_response_seen_ &&
                        msg->seq == action_second_arm_lower_response_seq_) {
                        action_second_arm_lower_done_seen_ = true;
                    }
                }
            });
        action_phase_ = ActionPhase::SendingPrep;
        action_started_tp_ = std::chrono::steady_clock::now();
        action_service_wait_started_tp_ = action_started_tp_;
        last_action_command_tp_ = std::chrono::steady_clock::time_point{};
        action_timer_ = create_wall_timer(
            std::chrono::milliseconds(20),
            [this]() { tickAction(); });

        RCLCPP_WARN(
            get_logger(),
            "KFS 动作测试已启用: direction=%s target_prefixes=%zu cmd_vel=%s service=%s "
            "feedback=%s prep=%s(%s) done=0x%02X service_wait=%.1fs "
            "second_lower=%s/0x%02X approach_vx_sign=%d approach_speed=%.3fm/s arm_reach=%.2fm "
            "approach_timeout=%.1fs depth_lock_range=[%.2f, %.2f]m grab=%s。"
            "运行前必须停用 Nav2/遥控等其它速度权威。",
            action_direction_.c_str(), action_target_prefixes_.size(),
            action_cmd_vel_topic_.c_str(), action_send_command_service_.c_str(),
            action_feedback_topic_.c_str(), action_prep_label_.c_str(),
            byteToHex(action_prep_command_id_).c_str(),
            static_cast<unsigned int>(action_prep_done_feedback_id_),
            action_service_wait_timeout_s_,
            byteToHex(action_second_arm_lower_command_id_).c_str(),
            static_cast<unsigned int>(action_second_arm_lower_done_feedback_id_),
            action_approach_x_sign_, action_approach_speed_mps_, action_grab_distance_m_,
            action_approach_timeout_s_,
            depth_config_.min_depth_m, depth_config_.max_depth_m,
            byteToHex(action_grab_command_id_).c_str());
    }

    void printStatus() {
        const bool running = manager_->isRunning();
        const bool ready = manager_->isReady();
        const auto result = manager_->getLatestResult();

        RCLCPP_INFO(this->get_logger(),
            "[状态] running=%d ready=%d has_target=%d attr=%d dist=%.2fm action=%s target=%s z=%.2fm",
            running, ready, result.has_target,
            static_cast<int>(result.attr_kind), result.distance_m,
            actionPhaseLabel().c_str(),
            last_action_label_.empty() ? "-" : last_action_label_.c_str(),
            last_action_depth_m_);
    }

    std::string actionPhaseLabel() const {
        switch (action_phase_) {
            case ActionPhase::Disabled:
                return "DISABLED";
            case ActionPhase::SendingPrep:
                return "SEND_PREP";
            case ActionPhase::WaitingPrepDone:
                return "WAIT_PREP";
            case ActionPhase::Search:
                return "SEARCH";
            case ActionPhase::Align:
                return "ALIGN";
            case ActionPhase::WaitingSecondArmLower:
                return "WAIT_SECOND_ARM_LOWER";
            case ActionPhase::Approach:
                return "APPROACH";
            case ActionPhase::SendingGrab:
                return "SEND_GRAB";
            case ActionPhase::GrabVerify:
                return "GRAB_VERIFY";
            case ActionPhase::Succeeded:
                return "SUCCESS";
            case ActionPhase::Failed:
                return "FAILED";
        }
        return "UNKNOWN";
    }

    void failAction(const std::string& reason) {
        if (action_phase_ == ActionPhase::Failed || action_phase_ == ActionPhase::Succeeded) {
            return;
        }
        action_failure_reason_ = reason;
        action_phase_ = ActionPhase::Failed;
        publishActionStop(true);
        RCLCPP_WARN(get_logger(), "KFS 动作测试失败: %s", reason.c_str());
    }

    bool shouldPublishActionCommand(
        const std::chrono::steady_clock::time_point& now,
        bool force) const {
        if (!action_cmd_pub_) {
            return false;
        }
        if (force || last_action_command_tp_ == std::chrono::steady_clock::time_point{}) {
            return true;
        }
        const double min_period_s = 1.0 / std::max(1e-6, action_command_rate_hz_);
        return std::chrono::duration<double>(now - last_action_command_tp_).count() >= min_period_s;
    }

    bool publishActionCommand(double vx, double vy, double wz, bool force = false) {
        if (!action_enable_ || !action_cmd_pub_) {
            return false;
        }
        const auto now = std::chrono::steady_clock::now();
        if (!shouldPublishActionCommand(now, force)) {
            return false;
        }
        TwistMsg msg;
        msg.linear.x = vx;
        msg.linear.y = vy;
        msg.angular.z = wz;
        action_cmd_pub_->publish(msg);
        last_action_command_tp_ = now;
        action_zero_published_ =
            std::abs(vx) < 1e-9 && std::abs(vy) < 1e-9 && std::abs(wz) < 1e-9;
        return true;
    }

    void publishActionStop(bool force) {
        if (!action_enable_ || !action_cmd_pub_) {
            return;
        }
        if (!force && action_zero_published_) {
            return;
        }
        publishActionCommand(0.0, 0.0, 0.0, force);
    }

    double computeActionVy(int offset_px) const {
        const int abs_offset = std::abs(offset_px);
        if (abs_offset <= action_align_tolerance_px_ || action_align_max_speed_mps_ <= 0.0) {
            return 0.0;
        }
        double speed = static_cast<double>(abs_offset) * action_align_kp_;
        speed = std::clamp(speed, action_align_min_speed_mps_, action_align_max_speed_mps_);

        double direction = offset_px > 0 ? -1.0 : 1.0;
        if (action_invert_lateral_direction_) {
            direction = -direction;
        }
        return direction * speed;
    }

    double computeActionApproachVx() const {
        return static_cast<double>(action_approach_x_sign_) * action_approach_speed_mps_;
    }

    bool beginOpenLoopApproach(
        const ActionTarget& target,
        int64_t sequence,
        const std::chrono::steady_clock::time_point& now) {
        if (!target.has_depth) {
            publishActionStop(true);
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "KFS 目标已对齐但深度无效，继续等待有效深度后再进入开环趋近");
            return false;
        }
        if (target.depth_m < depth_config_.min_depth_m || target.depth_m > depth_config_.max_depth_m) {
            publishActionStop(true);
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "KFS 目标已对齐但锁定深度 %.2fm 不在有效窗口 [%.2f, %.2f]m，继续等待",
                target.depth_m, depth_config_.min_depth_m, depth_config_.max_depth_m);
            return false;
        }

        const double planned_distance_m = std::max(0.0, target.depth_m - action_grab_distance_m_);
        if (planned_distance_m > 0.0 && action_approach_speed_mps_ <= 0.0) {
            failAction("KFS 开环趋近需要前进距离但 kfs_action_approach_speed_mps <= 0");
            return false;
        }

        const double planned_duration_s =
            action_approach_speed_mps_ > 0.0 ? planned_distance_m / action_approach_speed_mps_ : 0.0;
        if (planned_duration_s > action_approach_timeout_s_) {
            std::ostringstream ss;
            ss << "KFS 开环趋近计划超出安全超时: distance="
               << std::fixed << std::setprecision(3) << planned_distance_m
               << "m speed=" << action_approach_speed_mps_
               << "m/s duration=" << planned_duration_s
               << "s timeout=" << action_approach_timeout_s_ << "s";
            failAction(ss.str());
            return false;
        }

        action_approach_started_tp_ = std::chrono::steady_clock::time_point{};
        action_open_loop_start_depth_m_ = target.depth_m;
        action_open_loop_distance_m_ = planned_distance_m;
        action_open_loop_duration_s_ = planned_duration_s;
        action_open_loop_sequence_ = sequence;
        action_pending_grab_target_ = makeVisualTargetSnapshot(target.detection, sequence);
        action_grab_verify_last_sequence_ = sequence;
        action_grab_verify_lost_count_ = 0;
        action_grab_verify_seen_new_frame_ = false;
        action_grab_verify_visible_logged_ = false;
        action_grab_verify_last_logged_lost_count_ = 0;

        RCLCPP_INFO(
            get_logger(),
            "KFS 目标对齐稳定，锁定开环趋近: label=%s offset=%d depth=%.3fm arm_reach=%.3fm "
            "distance=%.3fm speed=%.3fm/s duration=%.3fs vx=%.3f",
            last_action_label_.c_str(), last_action_offset_px_,
            action_open_loop_start_depth_m_, action_grab_distance_m_,
            action_open_loop_distance_m_, action_approach_speed_mps_,
            action_open_loop_duration_s_, computeActionApproachVx());
        if (action_direction_ == "down") {
            beginSecondArmLowerRequest();
            return true;
        }
        startOpenLoopApproach(now);
        return true;
    }

    void startOpenLoopApproach(const std::chrono::steady_clock::time_point& now) {
        action_phase_ = ActionPhase::Approach;
        action_approach_started_tp_ = now;
        if (action_open_loop_duration_s_ <= 0.0) {
            publishActionStop(true);
            beginGrabRequest();
        } else {
            publishActionCommand(computeActionApproachVx(), 0.0, 0.0, true);
        }
    }

    void tickOpenLoopApproach(const std::chrono::steady_clock::time_point& now) {
        if (action_approach_started_tp_ == std::chrono::steady_clock::time_point{}) {
            action_approach_started_tp_ = now;
        }

        const double elapsed_s =
            std::chrono::duration<double>(now - action_approach_started_tp_).count();
        if (elapsed_s > action_approach_timeout_s_) {
            failAction("KFS 开环趋近阶段超时");
            return;
        }

        if (elapsed_s >= action_open_loop_duration_s_) {
            publishActionStop(true);
            beginGrabRequest();
            return;
        }

        publishActionCommand(computeActionApproachVx(), 0.0, 0.0);
    }

    bool sendActionCommandRequest(
        uint8_t command_id,
        const std::string& label,
        std::chrono::steady_clock::time_point* request_tp,
        bool* request_pending,
        bool* response_seen,
        bool* response_accepted,
        uint8_t* response_seq,
        uint64_t* generation,
        const std::string& error_prefix) {
        if (!action_send_client_) {
            failAction(error_prefix + " service client 未初始化");
            return false;
        }
        if (!action_send_client_->service_is_ready()) {
            failAction(error_prefix + " service 暂不可用");
            return false;
        }

        auto request = std::make_shared<SendCommandSrv::Request>();
        request->command_id = command_id;
        request->payload.clear();

        *request_tp = std::chrono::steady_clock::now();
        *request_pending = true;
        *response_seen = false;
        *response_accepted = false;
        *response_seq = 0U;
        const uint64_t token = ++(*generation);

        try {
            action_send_client_->async_send_request(
                request,
                [this, token, generation, response_seen, response_accepted, response_seq,
                 error_prefix](rclcpp::Client<SendCommandSrv>::SharedFuture future) {
                    if (token != *generation) {
                        return;
                    }
                    bool accepted = false;
                    uint8_t seq = 0U;
                    try {
                        const auto response = future.get();
                        accepted = response && response->accepted;
                        if (response) {
                            seq = response->seq;
                        }
                    } catch (const std::exception& e) {
                        RCLCPP_WARN(get_logger(), "%s service 响应异常: %s",
                            error_prefix.c_str(), e.what());
                    }
                    *response_accepted = accepted;
                    *response_seq = seq;
                    *response_seen = true;
                });
        } catch (const std::exception& e) {
            *request_pending = false;
            failAction(error_prefix + " service 请求异常: " + e.what());
            return false;
        }

        RCLCPP_INFO(get_logger(), "%s service 请求已发送: cmd=%s",
            label.c_str(), byteToHex(command_id).c_str());
        return true;
    }

    std::optional<ActionTarget> selectActionTarget(
        const VisionInferenceManager::FrameSnapshot& snapshot) {
        const auto targets = collectActionTargets(
            snapshot, depth_config_, action_target_labels_, action_target_prefixes_);
        if (targets.empty()) {
            if (action_target_locked_) {
                ++action_target_lost_count_;
                if (action_target_lost_count_ < action_lost_stop_frames_) {
                    return std::nullopt;
                }
                action_target_locked_ = false;
                action_locked_target_.reset();
            }
            return std::nullopt;
        }

        const int frame_center_x = std::max(0, snapshot.color_bgr.cols / 2);
        if (action_target_lock_enable_ && action_target_locked_ && action_locked_target_.has_value()) {
            const auto tracked = chooseClosestToLockedTarget(
                targets, *action_locked_target_, action_target_lock_max_jump_px_);
            if (tracked.has_value()) {
                action_locked_target_ = *tracked;
                action_target_lost_count_ = 0;
                return tracked;
            }
            ++action_target_lost_count_;
            if (action_target_lost_count_ >= action_lost_stop_frames_) {
                action_target_locked_ = false;
                action_locked_target_.reset();
            }
            return std::nullopt;
        }

        const auto selected = chooseClosestTarget(targets, frame_center_x);
        if (selected.has_value() && action_target_lock_enable_) {
            action_target_locked_ = true;
            action_locked_target_ = *selected;
            action_target_lost_count_ = 0;
        }
        return selected;
    }

    void rememberActionTarget(const ActionTarget& target) {
        last_action_label_ = detectionName(target.detection);
        last_action_offset_px_ = target.offset_px;
        last_action_depth_m_ = target.depth_m;
        last_action_has_depth_ = target.has_depth;
        last_action_score_ = target.detection.score;
    }

    void beginPrepRequest() {
        action_latest_prep_done_seq_ = -1;
        action_prep_done_seen_ = false;
        action_prep_done_wait_started_tp_ = std::chrono::steady_clock::time_point{};
        action_phase_ = ActionPhase::WaitingPrepDone;
        if (!sendActionCommandRequest(
                action_prep_command_id_,
                action_prep_label_,
                &action_prep_request_tp_,
                &action_prep_request_pending_,
                &action_prep_response_seen_,
                &action_prep_response_accepted_,
                &action_prep_response_seq_,
                &action_prep_request_generation_,
                "KFS 机械臂预调")) {
            return;
        }
        RCLCPP_INFO(get_logger(), "KFS 机械臂预调已发送: %s cmd=%s 等待反馈=0x%02X",
            action_prep_label_.c_str(), byteToHex(action_prep_command_id_).c_str(),
            static_cast<unsigned int>(action_prep_done_feedback_id_));
    }

    void beginSecondArmLowerRequest() {
        action_latest_second_arm_lower_done_seq_ = -1;
        action_second_arm_lower_done_seen_ = false;
        action_second_arm_lower_done_wait_started_tp_ = std::chrono::steady_clock::time_point{};
        action_phase_ = ActionPhase::WaitingSecondArmLower;
        publishActionStop(true);
        if (!sendActionCommandRequest(
                action_second_arm_lower_command_id_,
                "KFS 第二节机械臂放下",
                &action_second_arm_lower_request_tp_,
                &action_second_arm_lower_request_pending_,
                &action_second_arm_lower_response_seen_,
                &action_second_arm_lower_response_accepted_,
                &action_second_arm_lower_response_seq_,
                &action_second_arm_lower_request_generation_,
                "KFS 第二节机械臂放下")) {
            return;
        }
        RCLCPP_INFO(get_logger(),
            "KFS 向下夹取开环前已发送第二节机械臂放下: cmd=%s 等待反馈=0x%02X",
            byteToHex(action_second_arm_lower_command_id_).c_str(),
            static_cast<unsigned int>(action_second_arm_lower_done_feedback_id_));
    }

    bool waitForActionService(const std::chrono::steady_clock::time_point& now) {
        if (!action_send_client_) {
            failAction("KFS 机械臂预调 service client 未初始化");
            return false;
        }
        if (action_send_client_->service_is_ready()) {
            return true;
        }

        publishActionStop(false);
        if (action_service_wait_started_tp_ == std::chrono::steady_clock::time_point{}) {
            action_service_wait_started_tp_ = now;
        }
        const double elapsed =
            std::chrono::duration<double>(now - action_service_wait_started_tp_).count();
        if (elapsed >= action_service_wait_timeout_s_) {
            std::ostringstream ss;
            ss << "KFS 机械臂预调 service 等待超时: service="
               << action_send_command_service_ << " timeout="
               << std::fixed << std::setprecision(1) << action_service_wait_timeout_s_ << "s";
            failAction(ss.str());
            return false;
        }

        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "KFS 机械臂预调 service 尚未被本节点发现，继续等待: service=%s elapsed=%.1fs timeout=%.1fs",
            action_send_command_service_.c_str(), elapsed, action_service_wait_timeout_s_);
        return false;
    }

    void pollPrepResponse(const std::chrono::steady_clock::time_point& now) {
        publishActionStop(false);
        if (!action_prep_request_pending_) {
            failAction("KFS 机械臂预调请求状态异常");
            return;
        }
        if (!action_prep_response_seen_) {
            if (std::chrono::duration<double, std::milli>(now - action_prep_request_tp_).count() >
                static_cast<double>(action_arm_prep_service_timeout_ms_)) {
                action_prep_request_pending_ = false;
                ++action_prep_request_generation_;
                std::ostringstream ss;
                ss << "KFS 机械臂预调 service 响应超时: timeout="
                   << action_arm_prep_service_timeout_ms_
                   << "ms，底层可靠发送可能仍在重试但未返回最终 ACK";
                failAction(ss.str());
            }
            return;
        }
        if (!action_prep_response_accepted_) {
            action_prep_request_pending_ = false;
            failAction("KFS 机械臂预调命令被 transport 拒绝");
            return;
        }
        if (action_prep_done_wait_started_tp_ == std::chrono::steady_clock::time_point{}) {
            action_prep_done_wait_started_tp_ = now;
            RCLCPP_INFO(
                get_logger(),
                "KFS 机械臂预调命令已 ACK: %s seq=%u，开始等待完成反馈=0x%02X timeout=%.1fs",
                action_prep_label_.c_str(),
                static_cast<unsigned int>(action_prep_response_seq_),
                static_cast<unsigned int>(action_prep_done_feedback_id_),
                action_arm_prep_done_timeout_s_);
        }
        if (action_latest_prep_done_seq_ == static_cast<int>(action_prep_response_seq_)) {
            action_prep_done_seen_ = true;
        }
        if (action_prep_done_seen_) {
            action_prep_request_pending_ = false;
            action_phase_ = ActionPhase::Search;
            action_align_stable_count_ = 0;
            action_grab_stable_count_ = 0;
            action_target_locked_ = false;
            action_locked_target_.reset();
            action_target_lost_count_ = 0;
            RCLCPP_INFO(get_logger(), "KFS 机械臂预调完成: %s seq=%u，开始视觉对齐",
                action_prep_label_.c_str(),
                static_cast<unsigned int>(action_prep_response_seq_));
            return;
        }
        if (std::chrono::duration<double>(now - action_prep_done_wait_started_tp_).count() >
            action_arm_prep_done_timeout_s_) {
            action_prep_request_pending_ = false;
            std::ostringstream ss;
            ss << "KFS 机械臂预调完成反馈超时: " << action_prep_label_
               << " seq=" << static_cast<unsigned int>(action_prep_response_seq_)
               << " feedback=0x" << std::uppercase << std::hex << std::setw(2)
               << std::setfill('0') << static_cast<unsigned int>(action_prep_done_feedback_id_)
               << std::nouppercase << std::dec << std::setfill(' ')
               << " timeout=" << std::fixed << std::setprecision(1)
               << action_arm_prep_done_timeout_s_ << "s";
            failAction(ss.str());
        }
    }

    void pollSecondArmLowerResponse(const std::chrono::steady_clock::time_point& now) {
        publishActionStop(false);
        if (!action_second_arm_lower_request_pending_) {
            failAction("KFS 第二节机械臂放下请求状态异常");
            return;
        }
        if (!action_second_arm_lower_response_seen_) {
            if (std::chrono::duration<double, std::milli>(
                    now - action_second_arm_lower_request_tp_).count() >
                static_cast<double>(action_arm_prep_service_timeout_ms_)) {
                action_second_arm_lower_request_pending_ = false;
                ++action_second_arm_lower_request_generation_;
                std::ostringstream ss;
                ss << "KFS 第二节机械臂放下 service 响应超时: timeout="
                   << action_arm_prep_service_timeout_ms_
                   << "ms，底层可靠发送可能仍在重试但未返回最终 ACK";
                failAction(ss.str());
            }
            return;
        }
        if (!action_second_arm_lower_response_accepted_) {
            action_second_arm_lower_request_pending_ = false;
            failAction("KFS 第二节机械臂放下命令被 transport 拒绝");
            return;
        }
        if (action_second_arm_lower_done_wait_started_tp_ ==
            std::chrono::steady_clock::time_point{}) {
            action_second_arm_lower_done_wait_started_tp_ = now;
            RCLCPP_INFO(
                get_logger(),
                "KFS 第二节机械臂放下命令已 ACK: seq=%u，开始等待完成反馈=0x%02X timeout=%.1fs",
                static_cast<unsigned int>(action_second_arm_lower_response_seq_),
                static_cast<unsigned int>(action_second_arm_lower_done_feedback_id_),
                action_arm_prep_done_timeout_s_);
        }
        if (action_latest_second_arm_lower_done_seq_ ==
            static_cast<int>(action_second_arm_lower_response_seq_)) {
            action_second_arm_lower_done_seen_ = true;
        }
        if (action_second_arm_lower_done_seen_) {
            action_second_arm_lower_request_pending_ = false;
            RCLCPP_INFO(get_logger(),
                "KFS 第二节机械臂放下完成: seq=%u，开始开环趋近",
                static_cast<unsigned int>(action_second_arm_lower_response_seq_));
            startOpenLoopApproach(now);
            return;
        }
        if (std::chrono::duration<double>(
                now - action_second_arm_lower_done_wait_started_tp_).count() >
            action_arm_prep_done_timeout_s_) {
            action_second_arm_lower_request_pending_ = false;
            std::ostringstream ss;
            ss << "KFS 第二节机械臂放下完成反馈超时: seq="
               << static_cast<unsigned int>(action_second_arm_lower_response_seq_)
               << " feedback=0x" << std::uppercase << std::hex << std::setw(2)
               << std::setfill('0') << static_cast<unsigned int>(
                      action_second_arm_lower_done_feedback_id_)
               << std::nouppercase << std::dec << std::setfill(' ')
               << " timeout=" << std::fixed << std::setprecision(1)
               << action_arm_prep_done_timeout_s_ << "s";
            failAction(ss.str());
        }
    }

    void tickAction() {
        if (!action_enable_) {
            return;
        }

        const auto now = std::chrono::steady_clock::now();
        if (action_phase_ == ActionPhase::Succeeded || action_phase_ == ActionPhase::Failed) {
            publishActionStop(false);
            return;
        }

        if (std::chrono::duration<double>(now - action_started_tp_).count() > action_total_timeout_s_) {
            failAction("动作整体超时");
            return;
        }

        if (action_phase_ == ActionPhase::SendingPrep) {
            if (!waitForActionService(now)) {
                return;
            }
            beginPrepRequest();
            return;
        }

        if (action_phase_ == ActionPhase::WaitingPrepDone) {
            pollPrepResponse(now);
            return;
        }

        if (action_phase_ == ActionPhase::WaitingSecondArmLower) {
            pollSecondArmLowerResponse(now);
            return;
        }

        if (action_phase_ == ActionPhase::SendingGrab) {
            pollGrabResponse(now);
            return;
        }

        if (action_phase_ == ActionPhase::GrabVerify) {
            tickGrabVerify(now);
            return;
        }

        if (action_phase_ == ActionPhase::Approach) {
            tickOpenLoopApproach(now);
            return;
        }

        VisionInferenceManager::FrameSnapshot snapshot;
        if (!manager_->getLatestFrameSnapshot(snapshot) || !snapshot.has_color || snapshot.color_bgr.empty()) {
            publishActionStop(false);
            return;
        }

        auto selected = selectActionTarget(snapshot);
        if (!selected.has_value()) {
            action_phase_ = ActionPhase::Search;
            action_align_stable_count_ = 0;
            action_grab_stable_count_ = 0;
            publishActionStop(false);
            return;
        }

        rememberActionTarget(*selected);
        const bool aligned = std::abs(selected->offset_px) <= action_align_tolerance_px_;
        if (aligned) {
            action_align_stable_count_ = std::min(action_align_stable_count_ + 1,
                                                 action_align_stable_frames_);
        } else {
            action_align_stable_count_ = 0;
        }

        const double vy = computeActionVy(selected->offset_px);
        if (action_phase_ == ActionPhase::Search || action_phase_ == ActionPhase::Align) {
            action_phase_ = ActionPhase::Align;
            action_grab_stable_count_ = 0;
            publishActionCommand(0.0, vy, 0.0);
            if (action_align_stable_count_ >= action_align_stable_frames_) {
                beginOpenLoopApproach(*selected, snapshot.display_sequence, now);
            }
            return;
        }
    }

    void beginGrabRequest() {
        if (action_grab_once_ && action_grab_sent_) {
            action_phase_ = ActionPhase::Succeeded;
            publishActionStop(true);
            return;
        }
        if (!action_pending_grab_target_.has_value()) {
            failAction("KFS 夹取请求缺少开环锁定目标");
            return;
        }
        action_phase_ = ActionPhase::SendingGrab;
        action_grab_verify_lost_count_ = 0;
        action_grab_verify_seen_new_frame_ = false;
        action_grab_verify_visible_logged_ = false;
        action_grab_verify_last_logged_lost_count_ = 0;
        if (!sendActionCommandRequest(
                action_grab_command_id_,
                "KFS 夹取",
                &action_grab_request_tp_,
                &action_grab_request_pending_,
                &action_grab_response_seen_,
                &action_grab_response_accepted_,
                &action_grab_response_seq_,
                &action_grab_request_generation_,
                "KFS 夹取")) {
            return;
        }

        RCLCPP_INFO(
            get_logger(),
            "KFS 开环趋近完成，已发送夹取 service 请求: direction=%s cmd=%s label=%s locked_z=%.2fm "
            "planned=%.3fm/%.3fs seq=%ld bbox=[%.1f %.1f %.1f %.1f]",
            action_direction_.c_str(), byteToHex(action_grab_command_id_).c_str(),
            action_pending_grab_target_->label.c_str(), action_open_loop_start_depth_m_,
            action_open_loop_distance_m_, action_open_loop_duration_s_,
            static_cast<long>(action_open_loop_sequence_),
            action_pending_grab_target_->x1, action_pending_grab_target_->y1,
            action_pending_grab_target_->x2, action_pending_grab_target_->y2);
    }

    void pollGrabResponse(const std::chrono::steady_clock::time_point& now) {
        publishActionStop(false);
        if (!action_grab_request_pending_) {
            failAction("KFS 夹取请求状态异常");
            return;
        }
        if (!action_grab_response_seen_) {
            if (std::chrono::duration<double, std::milli>(now - action_grab_request_tp_).count() >
                static_cast<double>(action_grab_service_timeout_ms_)) {
                action_grab_request_pending_ = false;
                ++action_grab_request_generation_;
                std::ostringstream ss;
                ss << "KFS 夹取 service 响应超时: timeout="
                   << action_grab_service_timeout_ms_
                   << "ms，底层可靠发送可能仍在重试但未返回最终 ACK";
                failAction(ss.str());
            }
            return;
        }
        action_grab_request_pending_ = false;
        if (!action_grab_response_accepted_) {
            failAction("KFS 夹取命令被 transport 拒绝");
            return;
        }
        action_last_grab_seq_ = action_grab_response_seq_;
        action_grab_sent_ = true;
        action_grab_verify_started_tp_ = now;
        VisionInferenceManager::FrameSnapshot start_snapshot;
        if (manager_->getLatestFrameSnapshot(start_snapshot) &&
            start_snapshot.has_display &&
            start_snapshot.display_sequence > 0) {
            action_grab_verify_last_sequence_ = start_snapshot.display_sequence;
        }
        action_phase_ = ActionPhase::GrabVerify;
        publishActionStop(true);
        RCLCPP_INFO(
            get_logger(),
            "KFS 夹取命令已 ACK，开始视觉消失确认: direction=%s cmd=%s seq=%u target=%s timeout=%.2fs lost=%d iou>=%.2f",
            action_direction_.c_str(), byteToHex(action_grab_command_id_).c_str(),
            static_cast<unsigned int>(action_last_grab_seq_),
            action_pending_grab_target_.has_value() ? action_pending_grab_target_->label.c_str() : "-",
            action_grab_verify_timeout_s_, action_grab_verify_lost_stable_frames_,
            action_grab_verify_iou_threshold_);
    }

    void tickGrabVerify(const std::chrono::steady_clock::time_point& now) {
        publishActionStop(false);
        if (!action_pending_grab_target_.has_value()) {
            failAction("KFS 夹取视觉验证目标缺失");
            return;
        }

        const double elapsed =
            std::chrono::duration<double>(now - action_grab_verify_started_tp_).count();
        VisionInferenceManager::FrameSnapshot snapshot;
        const bool has_snapshot =
            manager_->getLatestFrameSnapshot(snapshot) &&
            snapshot.has_display &&
            snapshot.display_sequence > 0;
        if (!has_snapshot || snapshot.display_sequence <= action_grab_verify_last_sequence_) {
            if (elapsed >= action_grab_verify_timeout_s_) {
                failAction("KFS 夹取视觉验证失败: 没有新推理帧");
            }
            return;
        }

        action_grab_verify_seen_new_frame_ = true;
        action_grab_verify_last_sequence_ = snapshot.display_sequence;
        bool still_visible = false;
        double best_iou = 0.0;
        for (const auto& det : snapshot.detections) {
            const auto candidate = makeVisualTargetSnapshot(det, snapshot.display_sequence);
            if (candidate.label != action_pending_grab_target_->label) {
                continue;
            }
            const double iou = bboxIou(*action_pending_grab_target_, candidate);
            best_iou = std::max(best_iou, iou);
            if (iou >= action_grab_verify_iou_threshold_) {
                still_visible = true;
                break;
            }
        }

        if (still_visible) {
            action_grab_verify_lost_count_ = 0;
            action_grab_verify_last_logged_lost_count_ = 0;
            if (!action_grab_verify_visible_logged_) {
                RCLCPP_INFO(
                    get_logger(),
                    "KFS 夹取视觉验证: 原目标仍可见 target=%s seq=%ld iou=%.3f",
                    action_pending_grab_target_->label.c_str(),
                    static_cast<long>(snapshot.display_sequence), best_iou);
                action_grab_verify_visible_logged_ = true;
            }
        } else {
            ++action_grab_verify_lost_count_;
            action_grab_verify_visible_logged_ = false;
            if (action_grab_verify_lost_count_ != action_grab_verify_last_logged_lost_count_) {
                RCLCPP_INFO(
                    get_logger(),
                    "KFS 夹取视觉验证: 原目标未匹配 stable=%d/%d seq=%ld best_iou=%.3f",
                    action_grab_verify_lost_count_,
                    action_grab_verify_lost_stable_frames_,
                    static_cast<long>(snapshot.display_sequence), best_iou);
                action_grab_verify_last_logged_lost_count_ = action_grab_verify_lost_count_;
            }
            if (action_grab_verify_lost_count_ >= action_grab_verify_lost_stable_frames_) {
                action_phase_ = ActionPhase::Succeeded;
                action_grab_success_ = true;
                publishActionStop(true);
                RCLCPP_INFO(
                    get_logger(), "KFS 物理夹取确认成功: 连续%d个新推理帧未识别到原目标",
                    action_grab_verify_lost_stable_frames_);
                return;
            }
        }

        if (elapsed >= action_grab_verify_timeout_s_) {
            failAction(action_grab_verify_seen_new_frame_
                           ? "KFS 夹取视觉验证失败: 原目标仍可见"
                           : "KFS 夹取视觉验证失败: 没有新推理帧");
        }
    }

    void renderDisplay() {
        cv::Mat frame;
        std::vector<Detection> dets;
        TargetResult result;
        if (!manager_->getLatestDisplay(frame, dets, result) || frame.empty()) {
            // 首帧到达前显示占位图，避免窗口全黑
            cv::Mat placeholder(480, 640, CV_8UC3, cv::Scalar(30, 30, 30));
            cv::putText(placeholder, "Waiting for camera...",
                cv::Point(140, 250), cv::FONT_HERSHEY_SIMPLEX, 1.0,
                cv::Scalar(200, 200, 200), 2, cv::LINE_AA);
            cv::imshow(window_name_, placeholder);
            cv::waitKey(1);
            return;
        }

        VisionInferenceManager::FrameSnapshot snapshot;
        cv::Mat depth;
        const bool has_depth =
            manager_->getLatestFrameSnapshot(snapshot) && snapshot.has_depth && !snapshot.depth.empty();
        if (has_depth) {
            depth = snapshot.depth;
        }

        // 帧率(按显示回调次数 / 经过秒数统计,区别于推理帧率)
        ++fps_frame_count_;
        const auto now = std::chrono::steady_clock::now();
        const double elapsed = std::chrono::duration<double>(now - fps_last_tp_).count();
        if (elapsed >= 0.5) {
            display_fps_ = static_cast<double>(fps_frame_count_) / elapsed;
            fps_frame_count_ = 0;
            fps_last_tp_ = now;
        }

        // 1) 所有检测框 + 类别名[id] + 置信度 + 中心 ROI 深度
        for (const auto& det : dets) {
            const int x1 = static_cast<int>(std::floor(det.x1));
            const int y1 = static_cast<int>(std::floor(det.y1));
            const int x2 = static_cast<int>(std::ceil(det.x2));
            const int y2 = static_cast<int>(std::ceil(det.y2));
            const cv::Rect box(x1, y1, std::max(0, x2 - x1), std::max(0, y2 - y1));
            if (box.width <= 0 || box.height <= 0) {
                continue;
            }
            const std::string name = detectionName(det);
            const cv::Scalar line_color = featureColorForLabel(name);
            cv::rectangle(frame, box, line_color, 2, cv::LINE_AA);

            const int cx = static_cast<int>((det.x1 + det.x2) * 0.5f);
            const int cy = static_cast<int>((det.y1 + det.y2) * 0.5f);

            std::string depth_text = "z=-- no-depth";
            if (has_depth) {
                DepthRoiSamplerConfig raw_depth_config = depth_config_;
                raw_depth_config.min_depth_m = 0.0;
                raw_depth_config.max_depth_m = std::numeric_limits<double>::infinity();
                const auto sampled = sampleMedianDepth(depth, cx, cy, raw_depth_config);
                if (sampled.has_value()) {
                    const bool in_range =
                        *sampled >= depth_config_.min_depth_m && *sampled <= depth_config_.max_depth_m;
                    std::ostringstream depth_ss;
                    depth_ss << "z=" << std::fixed << std::setprecision(2)
                             << *sampled << "m " << (in_range ? "OK" : "OUT");
                    depth_text = depth_ss.str();
                } else {
                    depth_text = "z=-- invalid";
                }
            }

            std::ostringstream label_ss;
            label_ss << name << "[" << det.class_id << "] "
                     << std::fixed << std::setprecision(2) << det.score;
            const std::string label_text = label_ss.str();

            int baseline = 0;
            const cv::Size label_size =
                cv::getTextSize(label_text, cv::FONT_HERSHEY_SIMPLEX, 0.52, 2, &baseline);
            const cv::Size depth_size =
                cv::getTextSize(depth_text, cv::FONT_HERSHEY_SIMPLEX, 0.52, 2, &baseline);
            const int text_width = std::max(label_size.width, depth_size.width) + 8;
            const int text_height = label_size.height + depth_size.height + 16;
            const int bx = clampTextX(box.x, text_width, frame.cols);
            int by = box.y - text_height - 4;
            if (by < 0) {
                by = std::min(frame.rows - text_height, box.y + box.height + 4);
            }
            by = std::max(0, by);
            const cv::Rect bg(bx, by,
                std::min(frame.cols - bx, text_width), std::min(frame.rows - by, text_height));
            cv::rectangle(frame, bg, labelBackgroundColor(line_color), cv::FILLED);
            cv::putText(frame, label_text, cv::Point(bg.x + 4, bg.y + label_size.height + 4),
                cv::FONT_HERSHEY_SIMPLEX, 0.52, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
            cv::putText(frame, depth_text,
                cv::Point(bg.x + 4, bg.y + label_size.height + depth_size.height + 10),
                cv::FONT_HERSHEY_SIMPLEX, 0.52, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
            cv::drawMarker(frame, cv::Point(cx, cy), line_color, cv::MARKER_CROSS, 12, 1, cv::LINE_AA);
        }

        // 2) 角落帧率 + 检测数
        std::ostringstream fps_ss;
        fps_ss << "FPS:" << std::fixed << std::setprecision(1) << display_fps_
               << "  Det:" << dets.size()
               << "  Model:" << active_model_id_;
        cv::putText(frame, fps_ss.str(), cv::Point(12, 28),
            cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(50, 220, 50), 2, cv::LINE_AA);

        // 3) 主目标(best target)类别 + D455 深度距离(kfs 主链特有)
        if (result.has_target) {
            std::ostringstream tgt_ss;
            tgt_ss << "TARGET " << result.class_name
                   << "  dist=" << std::fixed << std::setprecision(2) << result.distance_m << "m"
                   << "  score=" << std::setprecision(2) << result.score;
            cv::putText(frame, tgt_ss.str(), cv::Point(12, 56),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 215, 255), 2, cv::LINE_AA);
            cv::drawMarker(frame, cv::Point(result.bbox_cx, result.bbox_cy),
                cv::Scalar(0, 215, 255), cv::MARKER_CROSS, 18, 2, cv::LINE_AA);
        }

        if (action_enable_) {
            drawActionOverlay(frame);
        }

        cv::imshow(window_name_, frame);
        const int key = cv::waitKey(1) & 0xFF;
        if (key == 'q' || key == 27) {
            RCLCPP_INFO(this->get_logger(), "键盘请求退出。");
            rclcpp::shutdown();
        }
    }

    void drawActionOverlay(cv::Mat& frame) {
        const int x = 12;
        int y = resultOverlayBaseY();
        const cv::Scalar color =
            action_phase_ == ActionPhase::Failed ? cv::Scalar(40, 40, 230) :
            action_phase_ == ActionPhase::Succeeded ? cv::Scalar(70, 220, 70) :
            cv::Scalar(0, 215, 255);

        std::ostringstream line1;
        line1 << "KFS ACTION " << actionPhaseLabel()
              << " dir=" << action_direction_
              << " prep=" << byteToHex(action_prep_command_id_)
              << " cmd=" << byteToHex(action_grab_command_id_);
        if (action_grab_sent_) {
            line1 << " seq=" << static_cast<unsigned int>(action_last_grab_seq_);
        }
        cv::putText(frame, line1.str(), cv::Point(x, y),
            cv::FONT_HERSHEY_SIMPLEX, 0.65, color, 2, cv::LINE_AA);
        y += 26;

        std::ostringstream line2;
        line2 << "target=" << (last_action_label_.empty() ? "-" : last_action_label_)
              << " off=" << last_action_offset_px_ << "px"
              << " z=";
        if (last_action_has_depth_) {
            line2 << std::fixed << std::setprecision(2) << last_action_depth_m_ << "m";
        } else {
            line2 << "--";
        }
        line2 << " score=" << std::fixed << std::setprecision(2) << last_action_score_;
        if (action_phase_ == ActionPhase::GrabVerify) {
            line2 << " lost=" << action_grab_verify_lost_count_
                  << "/" << action_grab_verify_lost_stable_frames_;
        }
        cv::putText(frame, line2.str(), cv::Point(x, y),
            cv::FONT_HERSHEY_SIMPLEX, 0.65, color, 2, cv::LINE_AA);
        y += 26;

        if (action_open_loop_sequence_ > 0) {
            double elapsed_s = 0.0;
            if (action_approach_started_tp_ != std::chrono::steady_clock::time_point{}) {
                elapsed_s = std::max(0.0, std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - action_approach_started_tp_).count());
            }
            std::ostringstream line3;
            line3 << "open locked_z=" << std::fixed << std::setprecision(2)
                  << action_open_loop_start_depth_m_
                  << "m plan=" << action_open_loop_distance_m_
                  << "m t=" << elapsed_s << "/" << action_open_loop_duration_s_ << "s";
            cv::putText(frame, line3.str(), cv::Point(x, y),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv::LINE_AA);
            y += 24;
        }

        if (action_phase_ == ActionPhase::Failed && !action_failure_reason_.empty()) {
            cv::putText(frame, "reason=" + action_failure_reason_, cv::Point(x, y),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv::LINE_AA);
        }
    }

    int resultOverlayBaseY() const {
        return 84;
    }

    std::shared_ptr<VisionInferenceManager> manager_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr display_timer_;
    rclcpp::TimerBase::SharedPtr action_timer_;
    rclcpp::Publisher<TwistMsg>::SharedPtr action_cmd_pub_;
    rclcpp::Client<SendCommandSrv>::SharedPtr action_send_client_;
    rclcpp::Subscription<FeedbackMsg>::SharedPtr action_feedback_sub_;

    bool show_window_ = true;
    std::string window_name_;
    std::string active_model_id_;
    DepthRoiSamplerConfig depth_config_;
    std::chrono::steady_clock::time_point fps_last_tp_ = std::chrono::steady_clock::now();
    int fps_frame_count_ = 0;
    double display_fps_ = 0.0;
    bool log_detections_{false};
    bool log_status_{false};

    bool action_enable_{false};
    std::string action_direction_{"up"};
    std::vector<std::string> action_target_prefixes_{"T_"};
    std::vector<std::string> action_target_labels_;
    std::string action_cmd_vel_topic_{"cmd_vel"};
    std::string action_send_command_service_{"/mechanism/send_command"};
    std::string action_feedback_topic_{"/mechanism/command_feedback"};
    uint8_t action_arm_raise_command_id_{static_cast<uint8_t>(rc26_serial::CommandID::ARM_RAISE)};
    uint8_t action_arm_lower_command_id_{static_cast<uint8_t>(rc26_serial::CommandID::ARM_LOWER)};
    uint8_t action_arm_raise_done_feedback_id_{
        static_cast<uint8_t>(rc26_serial::FeedbackID::ARM_RAISE_DONE)};
    uint8_t action_arm_lower_done_feedback_id_{
        static_cast<uint8_t>(rc26_serial::FeedbackID::ARM_LOWER_DONE)};
    uint8_t action_second_arm_lower_command_id_{
        static_cast<uint8_t>(rc26_serial::CommandID::ARM_SECOND_LOWER)};
    uint8_t action_second_arm_lower_done_feedback_id_{
        static_cast<uint8_t>(rc26_serial::FeedbackID::ARM_SECOND_LOWER_DONE)};
    uint8_t action_prep_command_id_{static_cast<uint8_t>(rc26_serial::CommandID::ARM_RAISE)};
    uint8_t action_prep_done_feedback_id_{
        static_cast<uint8_t>(rc26_serial::FeedbackID::ARM_RAISE_DONE)};
    std::string action_prep_label_{"ARM_RAISE"};
    double action_service_wait_timeout_s_{5.0};
    int action_arm_prep_service_timeout_ms_{6000};
    double action_arm_prep_done_timeout_s_{10.0};
    int action_align_tolerance_px_{20};
    int action_align_stable_frames_{5};
    double action_align_kp_{0.0010};
    double action_align_min_speed_mps_{0.015};
    double action_align_max_speed_mps_{0.06};
    double action_command_rate_hz_{20.0};
    int action_lost_stop_frames_{3};
    bool action_target_lock_enable_{true};
    int action_target_lock_max_jump_px_{160};
    bool action_invert_lateral_direction_{false};
    double action_approach_speed_mps_{0.04};
    int action_approach_x_sign_{1};
    double action_approach_timeout_s_{8.0};
    double action_grab_distance_m_{0.70};
    int action_grab_stable_frames_{2};
    double action_total_timeout_s_{40.0};
    int action_grab_service_timeout_ms_{6000};
    double action_grab_verify_timeout_s_{3.0};
    int action_grab_verify_lost_stable_frames_{3};
    double action_grab_verify_iou_threshold_{0.30};
    bool action_grab_once_{true};
    uint8_t action_grab_kfs_up_command_id_{kDefaultGrabKfsUpCommand};
    uint8_t action_grab_kfs_down_command_id_{kDefaultGrabKfsDownCommand};
    uint8_t action_grab_command_id_{kDefaultGrabKfsUpCommand};

    ActionPhase action_phase_{ActionPhase::Disabled};
    std::chrono::steady_clock::time_point action_started_tp_{};
    std::chrono::steady_clock::time_point action_service_wait_started_tp_{};
    std::chrono::steady_clock::time_point action_approach_started_tp_{};
    double action_open_loop_start_depth_m_{0.0};
    double action_open_loop_distance_m_{0.0};
    double action_open_loop_duration_s_{0.0};
    int64_t action_open_loop_sequence_{0};
    std::chrono::steady_clock::time_point last_action_command_tp_{};
    bool action_zero_published_{true};
    bool action_target_locked_{false};
    std::optional<ActionTarget> action_locked_target_;
    int action_target_lost_count_{0};
    int action_align_stable_count_{0};
    int action_grab_stable_count_{0};
    std::string last_action_label_;
    int last_action_offset_px_{0};
    double last_action_depth_m_{0.0};
    bool last_action_has_depth_{false};
    double last_action_score_{0.0};
    std::string action_failure_reason_;
    bool action_grab_sent_{false};
    uint8_t action_last_grab_seq_{0U};
    bool action_grab_request_pending_{false};
    std::chrono::steady_clock::time_point action_grab_request_tp_{};
    uint64_t action_grab_request_generation_{0U};
    bool action_grab_response_seen_{false};
    bool action_grab_response_accepted_{false};
    uint8_t action_grab_response_seq_{0U};
    bool action_prep_request_pending_{false};
    std::chrono::steady_clock::time_point action_prep_request_tp_{};
    uint64_t action_prep_request_generation_{0U};
    bool action_prep_response_seen_{false};
    bool action_prep_response_accepted_{false};
    uint8_t action_prep_response_seq_{0U};
    int action_latest_prep_done_seq_{-1};
    bool action_prep_done_seen_{false};
    std::chrono::steady_clock::time_point action_prep_done_wait_started_tp_{};
    bool action_second_arm_lower_request_pending_{false};
    std::chrono::steady_clock::time_point action_second_arm_lower_request_tp_{};
    uint64_t action_second_arm_lower_request_generation_{0U};
    bool action_second_arm_lower_response_seen_{false};
    bool action_second_arm_lower_response_accepted_{false};
    uint8_t action_second_arm_lower_response_seq_{0U};
    int action_latest_second_arm_lower_done_seq_{-1};
    bool action_second_arm_lower_done_seen_{false};
    std::chrono::steady_clock::time_point action_second_arm_lower_done_wait_started_tp_{};
    std::optional<VisualTargetSnapshot> action_pending_grab_target_;
    std::chrono::steady_clock::time_point action_grab_verify_started_tp_{};
    int action_grab_verify_lost_count_{0};
    int64_t action_grab_verify_last_sequence_{0};
    bool action_grab_verify_seen_new_frame_{false};
    bool action_grab_verify_visible_logged_{false};
    int action_grab_verify_last_logged_lost_count_{0};
    bool action_grab_success_{false};
};

}  // namespace rc26_vision

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<rc26_vision::VisionTestNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("kfs_vision_test_node"), "启动失败: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
