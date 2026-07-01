#include "wait_for_red_element.hpp"

#include <algorithm>
#include <chrono>
#include <string>
#include <vector>

#include "rc26_decision/decision_failure.hpp"

namespace rc26_decision {

namespace {

double elapsedSec(const std::chrono::steady_clock::time_point& since) {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - since).count();
}

cv::Scalar hsvLower(int hue, int saturation_min, int value_min) {
    return cv::Scalar(std::clamp(hue, 0, 180), std::clamp(saturation_min, 0, 255),
                      std::clamp(value_min, 0, 255));
}

cv::Scalar hsvUpper(int hue) {
    return cv::Scalar(std::clamp(hue, 0, 180), 255, 255);
}

}  // namespace

WaitForRedElementAction::WaitForRedElementAction(const std::string& name,
                                                 const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config) {}

WaitForRedElementAction::~WaitForRedElementAction() { releaseCamera(); }

BT::NodeStatus WaitForRedElementAction::onStart() {
    if (!config().blackboard->get("node", node_) || node_ == nullptr) {
        writeDecisionFailure(config().blackboard, "WaitForRedElement",
                             "运行上下文缺失：node 不可用");
        return BT::NodeStatus::FAILURE;
    }
    if (!config().blackboard->get("mc_params", params_)) {
        RCLCPP_ERROR(node_->get_logger(), "等待红色元素: 黑板缺少 mc_params");
        writeDecisionFailure(config().blackboard, "WaitForRedElement", "黑板缺少 mc_params");
        return BT::NodeStatus::FAILURE;
    }

    stable_count_ = 0;
    start_tp_ = std::chrono::steady_clock::now();
    last_log_tp_ = start_tp_;
    if (!initCamera()) {
        writeDecisionFailure(config().blackboard, "WaitForRedElement",
                             "无法打开用于红色元素检测的 MC 相机");
        return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(node_->get_logger(),
                "等待红色元素: camera_index=%d device='%s' hsv=[%d-%d,%d-%d] s>=%d v>=%d area>=%d stable=%d timeout=%.1fs",
                params_.camera_index, params_.camera_device.c_str(), params_.red_hue_low1,
                params_.red_hue_high1, params_.red_hue_low2, params_.red_hue_high2,
                params_.red_saturation_min, params_.red_value_min, params_.red_min_area_px,
                params_.red_stable_frames, params_.red_detect_timeout_s);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitForRedElementAction::onRunning() {
    if (node_ == nullptr || !camera_.isOpened()) {
        writeDecisionFailure(config().blackboard, "WaitForRedElement",
                             "红色元素检测运行时相机不可用");
        releaseCamera();
        return BT::NodeStatus::FAILURE;
    }

    if (elapsedSec(start_tp_) > params_.red_detect_timeout_s) {
        RCLCPP_ERROR(node_->get_logger(), "等待红色元素超时: timeout=%.1fs stable=%d/%d",
                     params_.red_detect_timeout_s, stable_count_, params_.red_stable_frames);
        writeDecisionFailure(config().blackboard, "WaitForRedElement",
                             "等待摄像头识别红色元素超时");
        releaseCamera();
        return BT::NodeStatus::FAILURE;
    }

    cv::Mat frame;
    if (!camera_.read(frame) || frame.empty()) {
        stable_count_ = 0;
        if (elapsedSec(last_log_tp_) >= params_.red_log_period_s) {
            RCLCPP_WARN(node_->get_logger(), "等待红色元素: 相机当前帧为空，继续等待");
            last_log_tp_ = std::chrono::steady_clock::now();
        }
        return BT::NodeStatus::RUNNING;
    }

    double best_area_px = 0.0;
    const bool seen = detectRedElement(frame, best_area_px);
    stable_count_ = seen ? (stable_count_ + 1) : 0;

    if (stable_count_ >= params_.red_stable_frames) {
        RCLCPP_INFO(node_->get_logger(),
                    "红色元素已稳定识别: best_area=%.0fpx stable=%d/%d，准备延时进入梅林预选",
                    best_area_px, stable_count_, params_.red_stable_frames);
        releaseCamera();
        return BT::NodeStatus::SUCCESS;
    }

    if (elapsedSec(last_log_tp_) >= params_.red_log_period_s) {
        RCLCPP_INFO(node_->get_logger(),
                    "等待红色元素: seen=%s best_area=%.0fpx stable=%d/%d elapsed=%.1fs",
                    seen ? "true" : "false", best_area_px, stable_count_,
                    params_.red_stable_frames, elapsedSec(start_tp_));
        last_log_tp_ = std::chrono::steady_clock::now();
    }
    return BT::NodeStatus::RUNNING;
}

void WaitForRedElementAction::onHalted() { releaseCamera(); }

bool WaitForRedElementAction::initCamera() {
    bool ok = params_.camera_device.empty() ? openCamera(params_.camera_index, "")
                                             : openCamera(-1, params_.camera_device);
    if (!ok && params_.auto_scan_camera) {
        for (int i = 0; i < 10 && !ok; ++i) {
            if (i == params_.camera_index) {
                continue;
            }
            ok = openCamera(i, "");
        }
    }
    if (!ok && node_ != nullptr) {
        RCLCPP_ERROR(node_->get_logger(),
                     "等待红色元素: 无法打开相机 (index=%d device='%s')",
                     params_.camera_index, params_.camera_device.c_str());
    }
    return ok;
}

bool WaitForRedElementAction::openCamera(int index, const std::string& path) {
    releaseCamera();
    const bool opened = path.empty() ? (camera_.open(index, cv::CAP_V4L2) || camera_.open(index))
                                     : (camera_.open(path, cv::CAP_V4L2) || camera_.open(path));
    if (!opened) {
        return false;
    }
    camera_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    camera_.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(params_.width));
    camera_.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(params_.height));
    camera_.set(cv::CAP_PROP_FPS, static_cast<double>(params_.fps));

    cv::Mat test_frame;
    if (!camera_.read(test_frame) || test_frame.empty()) {
        releaseCamera();
        return false;
    }
    return true;
}

void WaitForRedElementAction::releaseCamera() {
    if (camera_.isOpened()) {
        camera_.release();
    }
}

bool WaitForRedElementAction::detectRedElement(const cv::Mat& frame_bgr,
                                               double& best_area_px) const {
    best_area_px = 0.0;
    cv::Mat hsv;
    cv::cvtColor(frame_bgr, hsv, cv::COLOR_BGR2HSV);

    cv::Mat mask1;
    cv::Mat mask2;
    cv::inRange(hsv,
                hsvLower(params_.red_hue_low1, params_.red_saturation_min,
                         params_.red_value_min),
                hsvUpper(params_.red_hue_high1), mask1);
    cv::inRange(hsv,
                hsvLower(params_.red_hue_low2, params_.red_saturation_min,
                         params_.red_value_min),
                hsvUpper(params_.red_hue_high2), mask2);

    cv::Mat mask;
    cv::bitwise_or(mask1, mask2, mask);
    const cv::Mat kernel =
        cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (const auto& contour : contours) {
        best_area_px = std::max(best_area_px, cv::contourArea(contour));
    }
    return best_area_px >= static_cast<double>(params_.red_min_area_px);
}

}  // namespace rc26_decision
