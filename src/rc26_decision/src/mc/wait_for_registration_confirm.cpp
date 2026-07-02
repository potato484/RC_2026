#include "wait_for_registration_confirm.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <vector>

#include "rc26_decision/decision_failure.hpp"

namespace rc26_decision {

namespace {

double elapsedSec(const std::chrono::steady_clock::time_point& since) {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - since).count();
}

cv::Mat toGlobalAffine(const cv::Mat& local_affine, const cv::Point& roi_origin) {
    cv::Mat global_affine = local_affine.clone();
    const double a00 = local_affine.at<double>(0, 0);
    const double a01 = local_affine.at<double>(0, 1);
    const double a10 = local_affine.at<double>(1, 0);
    const double a11 = local_affine.at<double>(1, 1);
    global_affine.at<double>(0, 2) +=
        static_cast<double>(roi_origin.x) -
        (a00 * static_cast<double>(roi_origin.x) + a01 * static_cast<double>(roi_origin.y));
    global_affine.at<double>(1, 2) +=
        static_cast<double>(roi_origin.y) -
        (a10 * static_cast<double>(roi_origin.x) + a11 * static_cast<double>(roi_origin.y));
    return global_affine;
}

}  // namespace

WaitForRegistrationConfirmAction::WaitForRegistrationConfirmAction(
    const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config) {}

WaitForRegistrationConfirmAction::~WaitForRegistrationConfirmAction() { releaseCamera(); }

BT::NodeStatus WaitForRegistrationConfirmAction::onStart() {
    if (!config().blackboard->get("node", node_) || node_ == nullptr) {
        writeDecisionFailure(config().blackboard, "WaitForRegistrationConfirm",
                             "运行上下文缺失：node 不可用");
        return BT::NodeStatus::FAILURE;
    }
    if (!config().blackboard->get("mc_params", params_)) {
        RCLCPP_ERROR(node_->get_logger(), "等待 MC 配准确认: 黑板缺少 mc_params");
        writeDecisionFailure(config().blackboard, "WaitForRegistrationConfirm",
                             "黑板缺少 mc_params");
        return BT::NodeStatus::FAILURE;
    }

    stable_count_ = 0;
    start_tp_ = std::chrono::steady_clock::now();
    last_log_tp_ = start_tp_;
    if (!params_.registration_gate_enable) {
        RCLCPP_WARN(node_->get_logger(),
                    "MC 配准变化 gate 已禁用，直接允许 MC 子树进入后续延时");
        return BT::NodeStatus::SUCCESS;
    }

    bool tip_grab_confirmed = false;
    std::string tip_grab_method;
    if (config().blackboard &&
        config().blackboard->get("mc_tip_grab_confirmed", tip_grab_confirmed) &&
        tip_grab_confirmed) {
        const bool has_tip_grab_method =
            config().blackboard->get("mc_tip_grab_confirm_method", tip_grab_method);
        RCLCPP_WARN(node_->get_logger(),
                    "MC 视觉伺服已确认夹取完成(method=%s)，端头可能已离开配准 ROI，跳过末尾配准前景确认",
                    (!has_tip_grab_method || tip_grab_method.empty()) ? "unknown"
                                                                      : tip_grab_method.c_str());
        return BT::NodeStatus::SUCCESS;
    }

    if (!initCamera()) {
        writeDecisionFailure(config().blackboard, "WaitForRegistrationConfirm",
                             "无法打开用于 MC 配准确认的相机");
        return BT::NodeStatus::FAILURE;
    }
    if (!loadReferenceFrame()) {
        writeDecisionFailure(config().blackboard, "WaitForRegistrationConfirm",
                             "缺少组合树启动时采集的 MC 相机基准帧，无法进行配准确认");
        releaseCamera();
        return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(node_->get_logger(),
                "等待 MC 配准确认: camera_index=%d device='%s' bg_roi_x=%.2f-%.2f bg_roi_y=%.2f-%.2f fg_roi_x=%.2f-%.2f fg_roi_y=%.2f-%.2f diff_threshold=%d area>=%d ratio>=%.3f bbox>=%dx%d score>=%.2f inliers>=%d stable=%d timeout=%.1fs",
                params_.camera_index, params_.camera_device.c_str(),
                params_.registration_roi_x_min_ratio, params_.registration_roi_x_max_ratio,
                params_.registration_roi_y_min_ratio, params_.registration_roi_y_max_ratio,
                params_.registration_foreground_roi_x_min_ratio,
                params_.registration_foreground_roi_x_max_ratio,
                params_.registration_foreground_roi_y_min_ratio,
                params_.registration_foreground_roi_y_max_ratio,
                params_.registration_diff_threshold,
                params_.registration_min_changed_area_px,
                params_.registration_min_changed_ratio,
                params_.registration_min_changed_bbox_width_px,
                params_.registration_min_changed_bbox_height_px,
                params_.registration_min_match_score,
                params_.registration_min_inliers,
                params_.registration_stable_frames,
                params_.registration_detect_timeout_s);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitForRegistrationConfirmAction::onRunning() {
    if (node_ == nullptr || !camera_.isOpened()) {
        writeDecisionFailure(config().blackboard, "WaitForRegistrationConfirm",
                             "MC 配准确认运行时相机不可用");
        releaseCamera();
        return BT::NodeStatus::FAILURE;
    }

    if (elapsedSec(start_tp_) > params_.registration_detect_timeout_s) {
        RCLCPP_ERROR(node_->get_logger(), "等待 MC 配准确认超时: timeout=%.1fs stable=%d/%d",
                     params_.registration_detect_timeout_s, stable_count_,
                     params_.registration_stable_frames);
        writeDecisionFailure(config().blackboard, "WaitForRegistrationConfirm",
                             "等待 MC 相机画面配准确认超时");
        releaseCamera();
        return BT::NodeStatus::FAILURE;
    }

    cv::Mat frame;
    if (!camera_.read(frame) || frame.empty()) {
        stable_count_ = 0;
        if (elapsedSec(last_log_tp_) >= params_.registration_log_period_s) {
            RCLCPP_WARN(node_->get_logger(), "等待 MC 配准确认: 相机当前帧为空，继续等待");
            last_log_tp_ = std::chrono::steady_clock::now();
        }
        return BT::NodeStatus::RUNNING;
    }

    int changed_area_px = 0;
    double changed_ratio = 0.0;
    double match_score = 0.0;
    int inliers = 0;
    cv::Rect changed_bbox;
    const bool matched = detectRegisteredForeground(frame, changed_area_px, changed_ratio,
                                                    match_score, inliers, changed_bbox);
    stable_count_ = matched ? (stable_count_ + 1) : 0;

    if (stable_count_ >= params_.registration_stable_frames) {
        RCLCPP_INFO(node_->get_logger(),
                    "MC 夹取前景已稳定确认: changed_area=%dpx ratio=%.3f bbox=%dx%d score=%.3f inliers=%d stable=%d/%d，准备延时进入梅林预选",
                    changed_area_px, changed_ratio, changed_bbox.width, changed_bbox.height,
                    match_score, inliers, stable_count_,
                    params_.registration_stable_frames);
        releaseCamera();
        return BT::NodeStatus::SUCCESS;
    }

    if (elapsedSec(last_log_tp_) >= params_.registration_log_period_s) {
        RCLCPP_INFO(node_->get_logger(),
                    "等待 MC 配准确认: foreground=%s area=%dpx ratio=%.3f bbox=%dx%d score=%.3f inliers=%d stable=%d/%d elapsed=%.1fs",
                    matched ? "true" : "false", changed_area_px, changed_ratio,
                    changed_bbox.width, changed_bbox.height, match_score, inliers, stable_count_,
                    params_.registration_stable_frames, elapsedSec(start_tp_));
        last_log_tp_ = std::chrono::steady_clock::now();
    }
    return BT::NodeStatus::RUNNING;
}

void WaitForRegistrationConfirmAction::onHalted() { releaseCamera(); }

bool WaitForRegistrationConfirmAction::initCamera() {
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
                     "等待 MC 配准确认: 无法打开相机 (index=%d device='%s')",
                     params_.camera_index, params_.camera_device.c_str());
    }
    return ok;
}

bool WaitForRegistrationConfirmAction::openCamera(int index, const std::string& path) {
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

void WaitForRegistrationConfirmAction::releaseCamera() {
    if (camera_.isOpened()) {
        camera_.release();
    }
}

bool WaitForRegistrationConfirmAction::loadReferenceFrame() {
    reference_gray_.release();
    if (!config().blackboard->get(params_.registration_reference_blackboard_key,
                                  reference_gray_) ||
        reference_gray_.empty()) {
        return false;
    }
    if (reference_gray_.channels() != 1) {
        cv::cvtColor(reference_gray_, reference_gray_, cv::COLOR_BGR2GRAY);
    }
    if (reference_gray_.type() != CV_8UC1) {
        reference_gray_.convertTo(reference_gray_, CV_8UC1);
    }
    return !reference_gray_.empty();
}

cv::Rect WaitForRegistrationConfirmAction::makeRegistrationRoi(const cv::Size& size) const {
    const int x_min = std::clamp(
        static_cast<int>(std::lround(params_.registration_roi_x_min_ratio * size.width)), 0,
        size.width);
    const int x_max = std::clamp(
        static_cast<int>(std::lround(params_.registration_roi_x_max_ratio * size.width)), 0,
        size.width);
    const int y_min = std::clamp(
        static_cast<int>(std::lround(params_.registration_roi_y_min_ratio * size.height)), 0,
        size.height);
    const int y_max = std::clamp(
        static_cast<int>(std::lround(params_.registration_roi_y_max_ratio * size.height)), 0,
        size.height);
    if (x_max <= x_min || y_max <= y_min) {
        return cv::Rect(0, 0, size.width, size.height);
    }
    return cv::Rect(x_min, y_min, x_max - x_min, y_max - y_min);
}

cv::Rect WaitForRegistrationConfirmAction::makeForegroundRoi(const cv::Size& size) const {
    const int x_min = std::clamp(
        static_cast<int>(std::lround(params_.registration_foreground_roi_x_min_ratio *
                                     size.width)),
        0, size.width);
    const int x_max = std::clamp(
        static_cast<int>(std::lround(params_.registration_foreground_roi_x_max_ratio *
                                     size.width)),
        0, size.width);
    const int y_min = std::clamp(
        static_cast<int>(std::lround(params_.registration_foreground_roi_y_min_ratio *
                                     size.height)),
        0, size.height);
    const int y_max = std::clamp(
        static_cast<int>(std::lround(params_.registration_foreground_roi_y_max_ratio *
                                     size.height)),
        0, size.height);
    if (x_max <= x_min || y_max <= y_min) {
        return cv::Rect(0, 0, size.width, size.height);
    }
    return cv::Rect(x_min, y_min, x_max - x_min, y_max - y_min);
}

bool WaitForRegistrationConfirmAction::detectRegisteredForeground(const cv::Mat& frame_bgr,
                                                                  int& changed_area_px,
                                                                  double& changed_ratio,
                                                                  double& match_score,
                                                                  int& inliers,
                                                                  cv::Rect& changed_bbox) const {
    changed_area_px = 0;
    changed_ratio = 0.0;
    match_score = 0.0;
    inliers = 0;
    changed_bbox = cv::Rect();
    if (reference_gray_.empty() || frame_bgr.empty()) {
        return false;
    }

    cv::Mat current_gray;
    cv::cvtColor(frame_bgr, current_gray, cv::COLOR_BGR2GRAY);
    if (current_gray.size() != reference_gray_.size()) {
        cv::resize(current_gray, current_gray, reference_gray_.size());
    }

    const cv::Rect bg_roi = makeRegistrationRoi(reference_gray_.size());
    const cv::Rect fg_roi = makeForegroundRoi(reference_gray_.size());
    if (bg_roi.empty() || fg_roi.empty()) {
        return false;
    }
    const cv::Mat ref_roi = reference_gray_(bg_roi);
    const cv::Mat cur_roi = current_gray(bg_roi);

    std::vector<cv::Point2f> ref_points;
    cv::Mat feature_mask(ref_roi.size(), CV_8UC1, cv::Scalar(255));
    const cv::Rect overlap = bg_roi & fg_roi;
    if (!overlap.empty()) {
        cv::Rect local_overlap(overlap.x - bg_roi.x, overlap.y - bg_roi.y,
                               overlap.width, overlap.height);
        cv::rectangle(feature_mask, local_overlap, cv::Scalar(0), cv::FILLED);
    }
    cv::goodFeaturesToTrack(ref_roi, ref_points, params_.registration_max_corners,
                            params_.registration_quality_level,
                            params_.registration_min_distance_px, feature_mask);
    if (ref_points.size() < static_cast<size_t>(params_.registration_min_inliers)) {
        return false;
    }

    std::vector<cv::Point2f> cur_points;
    std::vector<uint8_t> status;
    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(ref_roi, cur_roi, ref_points, cur_points, status, err);

    std::vector<cv::Point2f> src;
    std::vector<cv::Point2f> dst;
    src.reserve(ref_points.size());
    dst.reserve(ref_points.size());
    for (size_t i = 0; i < status.size(); ++i) {
        if (status[i]) {
            src.push_back(ref_points[i]);
            dst.push_back(cur_points[i]);
        }
    }
    if (src.size() < static_cast<size_t>(params_.registration_min_inliers)) {
        return false;
    }

    cv::Mat inlier_mask;
    cv::Mat affine = cv::estimateAffinePartial2D(
        src, dst, inlier_mask, cv::RANSAC, params_.registration_max_reproj_error_px);
    if (affine.empty() || inlier_mask.empty()) {
        return false;
    }
    inliers = cv::countNonZero(inlier_mask);
    if (inliers < params_.registration_min_inliers) {
        return false;
    }

    match_score = static_cast<double>(inliers) / static_cast<double>(src.size());
    if (match_score < params_.registration_min_match_score) {
        return false;
    }

    const cv::Mat global_affine = toGlobalAffine(affine, bg_roi.tl());
    cv::Mat aligned_ref_full;
    cv::warpAffine(reference_gray_, aligned_ref_full, global_affine, current_gray.size(),
                   cv::INTER_LINEAR, cv::BORDER_REPLICATE);

    cv::Mat diff;
    cv::absdiff(aligned_ref_full(fg_roi), current_gray(fg_roi), diff);
    cv::GaussianBlur(diff, diff, cv::Size(5, 5), 0.0);

    cv::Mat mask;
    cv::threshold(diff, mask, params_.registration_diff_threshold, 255, cv::THRESH_BINARY);
    const cv::Mat kernel =
        cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

    changed_area_px = cv::countNonZero(mask);
    changed_ratio = static_cast<double>(changed_area_px) /
                    static_cast<double>(std::max(1, fg_roi.area()));

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (const auto& contour : contours) {
        const auto bbox = cv::boundingRect(contour);
        if (bbox.area() > changed_bbox.area()) {
            changed_bbox = bbox;
        }
    }

    return changed_area_px >= params_.registration_min_changed_area_px &&
           changed_ratio >= params_.registration_min_changed_ratio &&
           changed_bbox.height >= params_.registration_min_changed_bbox_height_px &&
           changed_bbox.width >= params_.registration_min_changed_bbox_width_px;
}

}  // namespace rc26_decision
