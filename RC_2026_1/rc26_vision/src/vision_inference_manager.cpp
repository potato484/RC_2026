#include "rc26_vision/vision_inference_manager.hpp"
#include "rc26_vision/yolo_engine.hpp"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>

#include <algorithm>
#include <cmath>

namespace rc26_vision {

namespace {

constexpr int64_t kFrameExpiryNs = 1'000'000'000LL;

AttributeKind attributeFromClassName(const std::string& name) {
    if (name == "R_R1") return AttributeKind::R_R1;
    if (name == "B_R1") return AttributeKind::B_R1;
    if (name.size() >= 2 && name[0] == 'T' && name[1] == '_') return AttributeKind::Truth;
    if (name.size() >= 2 && name[0] == 'F' && name[1] == '_') return AttributeKind::False;
    return AttributeKind::Unknown;
}

}  // namespace

VisionInferenceManager::VisionInferenceManager(rclcpp::Node& node)
    : node_(node) {
    if (!node_.has_parameter("vision_color_topic")) {
        node_.declare_parameter("vision_color_topic", std::string("/camera/color/image_raw"));
    }
    if (!node_.has_parameter("vision_depth_topic")) {
        node_.declare_parameter("vision_depth_topic",
                                std::string("/camera/aligned_depth_to_color/image_raw"));
    }
    if (!node_.has_parameter("vision_info_topic")) {
        node_.declare_parameter("vision_info_topic", std::string("/camera/color/camera_info"));
    }
    if (!node_.has_parameter("vision_depth_roi")) {
        node_.declare_parameter("vision_depth_roi", depth_roi_size_);
    }
    if (!node_.has_parameter("vision_depth_min_valid_count")) {
        node_.declare_parameter("vision_depth_min_valid_count", depth_min_valid_count_);
    }
    if (!node_.has_parameter("vision_depth_min_m")) {
        node_.declare_parameter("vision_depth_min_m", depth_min_m_);
    }
    if (!node_.has_parameter("vision_depth_max_m")) {
        node_.declare_parameter("vision_depth_max_m", depth_max_m_);
    }

    const std::string color_topic = node_.get_parameter("vision_color_topic").as_string();
    const std::string depth_topic = node_.get_parameter("vision_depth_topic").as_string();
    const std::string info_topic = node_.get_parameter("vision_info_topic").as_string();

    depth_roi_size_ = static_cast<int>(node_.get_parameter("vision_depth_roi").as_int());
    depth_min_valid_count_ =
        static_cast<int>(node_.get_parameter("vision_depth_min_valid_count").as_int());
    depth_min_m_ = node_.get_parameter("vision_depth_min_m").as_double();
    depth_max_m_ = node_.get_parameter("vision_depth_max_m").as_double();

    color_sub_ = node_.create_subscription<sensor_msgs::msg::Image>(
        color_topic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Image::ConstSharedPtr msg) { onColorImage(msg); });

    depth_sub_ = node_.create_subscription<sensor_msgs::msg::Image>(
        depth_topic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Image::ConstSharedPtr msg) { onDepthImage(msg); });

    info_sub_ = node_.create_subscription<sensor_msgs::msg::CameraInfo>(
        info_topic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) { onCameraInfo(msg); });
}

VisionInferenceManager::~VisionInferenceManager() {
    stop();
}

bool VisionInferenceManager::configure(
    const std::string& model_path,
    const std::vector<std::string>& class_names,
    float conf_thresh) {
    if (running_.load(std::memory_order_relaxed) || inference_thread_.joinable()) {
        return false;
    }

    try {
        auto engine = std::make_unique<YoloEngine>(model_path, class_names, conf_thresh);
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (running_.load(std::memory_order_relaxed) || inference_thread_.joinable()) {
                return false;
            }
            engine_ = std::move(engine);
            class_names_ = class_names;
        }
        configured_.store(true, std::memory_order_relaxed);
        vision_ok_.store(true, std::memory_order_relaxed);
        return true;
    } catch (...) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            engine_.reset();
            class_names_.clear();
        }
        configured_.store(false, std::memory_order_relaxed);
        vision_ok_.store(false, std::memory_order_relaxed);
        return false;
    }
}

bool VisionInferenceManager::start() {
    if (running_.load(std::memory_order_relaxed)) return true;
    if (!configured_.load(std::memory_order_relaxed)) return false;

    stop();

    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (running_.load(std::memory_order_relaxed)) return true;

        stop_requested_.store(false, std::memory_order_relaxed);
        running_.store(true, std::memory_order_relaxed);
        vision_ok_.store(true, std::memory_order_relaxed);
        inference_thread_ = std::thread(&VisionInferenceManager::inferenceLoop, this);
    }
    return true;
}

void VisionInferenceManager::stop() {
    std::thread thread_to_join;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!running_.load(std::memory_order_relaxed) && !inference_thread_.joinable()) {
            return;
        }

        stop_requested_.store(true, std::memory_order_relaxed);
        running_.store(false, std::memory_order_relaxed);
        cv_.notify_all();

        if (inference_thread_.joinable() &&
            std::this_thread::get_id() != inference_thread_.get_id()) {
            thread_to_join = std::move(inference_thread_);
        }
    }

    if (thread_to_join.joinable()) {
        thread_to_join.join();
    }
}

bool VisionInferenceManager::isRunning() const {
    return running_.load();
}

bool VisionInferenceManager::isReady() const {
    if (!configured_.load(std::memory_order_relaxed)) return false;
    if (!vision_ok_.load(std::memory_order_relaxed)) return false;

    const int64_t last_ns = last_frame_time_ns_.load(std::memory_order_relaxed);
    if (last_ns <= 0) return false;

    const int64_t now_ns = node_.get_clock()->now().nanoseconds();
    const int64_t age_ns = now_ns - last_ns;
    return age_ns >= 0 && age_ns <= kFrameExpiryNs;
}

TargetResult VisionInferenceManager::getLatestResult() const {
    std::lock_guard<std::mutex> lock(result_mutex_);
    return latest_result_;
}

void VisionInferenceManager::setResultCallback(ResultCallback cb) {
    std::lock_guard<std::mutex> lock(result_mutex_);
    result_callback_ = std::move(cb);
}

void VisionInferenceManager::inferenceLoop() {
    while (true) {
        try {
            sensor_msgs::msg::Image::ConstSharedPtr color_msg;
            sensor_msgs::msg::Image::ConstSharedPtr depth_msg;

            {
                std::unique_lock<std::mutex> lock(mutex_);
                cv_.wait(lock, [this] {
                    return stop_requested_.load(std::memory_order_relaxed) ||
                           static_cast<bool>(latest_color_);
                });

                if (stop_requested_.load(std::memory_order_relaxed)) {
                    break;
                }

                color_msg = latest_color_;
                depth_msg = latest_depth_;
                latest_color_.reset();
            }

            if (!color_msg) {
                continue;
            }

            TargetResult result;
            result.timestamp_ns = rclcpp::Time(color_msg->header.stamp).nanoseconds();

            const int64_t now_ns = node_.get_clock()->now().nanoseconds();
            if ((now_ns - result.timestamp_ns) > kFrameExpiryNs) {
                vision_ok_.store(false, std::memory_order_relaxed);
            } else {
                try {
                    if (!engine_) {
                        vision_ok_.store(false, std::memory_order_relaxed);
                    } else {
                        const cv::Mat color = cv_bridge::toCvCopy(color_msg, "bgr8")->image;
                        std::vector<Detection> detections = engine_->infer(color);

                        const Detection* best_det = nullptr;
                        for (const auto& det : detections) {
                            if (!best_det || det.score > best_det->score) {
                                best_det = &det;
                            }
                        }

                        if (best_det) {
                            result.score = best_det->score;
                            result.bbox_cx = static_cast<int>((best_det->x1 + best_det->x2) * 0.5f);
                            result.bbox_cy = static_cast<int>((best_det->y1 + best_det->y2) * 0.5f);

                            std::string class_name = best_det->class_name;
                            if (class_name.empty() && best_det->class_id >= 0 &&
                                best_det->class_id < static_cast<int>(class_names_.size())) {
                                class_name = class_names_[best_det->class_id];
                            }
                            result.attr_kind = attributeFromClassName(class_name);

                            if (depth_msg) {
                                const cv::Mat depth = cv_bridge::toCvShare(depth_msg)->image;
                                const double dist_m = computeDepth(depth, result.bbox_cx, result.bbox_cy);
                                result.distance_m = dist_m;
                                result.has_target = (dist_m > 0.0);
                            } else {
                                result.has_target = false;
                            }
                        } else {
                            result.has_target = false;
                        }

                        vision_ok_.store(true, std::memory_order_relaxed);
                    }
                } catch (...) {
                    vision_ok_.store(false, std::memory_order_relaxed);
                    result = TargetResult{};
                    result.timestamp_ns = rclcpp::Time(color_msg->header.stamp).nanoseconds();
                }
            }

            ResultCallback cb;
            {
                std::lock_guard<std::mutex> lock(result_mutex_);
                latest_result_ = result;
                cb = result_callback_;
            }
            if (cb) {
                try {
                    cb(result);
                } catch (...) {
                }
            }
        } catch (...) {
        }
    }

    running_.store(false, std::memory_order_relaxed);
}

void VisionInferenceManager::onColorImage(
    const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_color_ = msg;
        last_frame_time_ns_.store(rclcpp::Time(msg->header.stamp).nanoseconds(),
                                  std::memory_order_relaxed);
    }
    cv_.notify_one();
}

void VisionInferenceManager::onDepthImage(
    const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_depth_ = msg;
        last_frame_time_ns_.store(rclcpp::Time(msg->header.stamp).nanoseconds(),
                                  std::memory_order_relaxed);
    }
    cv_.notify_one();
}

void VisionInferenceManager::onCameraInfo(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_info_ = msg;
}

double VisionInferenceManager::computeDepth(
    const cv::Mat& depth, int cx, int cy) {
    if (depth.empty() || depth.rows <= 0 || depth.cols <= 0) return 0.0;
    if (depth.channels() != 1) return 0.0;
    if (depth_roi_size_ <= 0 || depth_min_valid_count_ <= 0) return 0.0;

    const int half = depth_roi_size_ / 2;

    cx = std::max(0, std::min(cx, depth.cols - 1));
    cy = std::max(0, std::min(cy, depth.rows - 1));

    const int x0 = std::max(0, cx - half);
    const int x1 = std::min(depth.cols - 1, cx + half);
    const int y0 = std::max(0, cy - half);
    const int y1 = std::min(depth.rows - 1, cy + half);
    if (x0 > x1 || y0 > y1) return 0.0;

    std::vector<double> samples;
    samples.reserve(static_cast<size_t>((x1 - x0 + 1) * (y1 - y0 + 1)));

    for (int y = y0; y <= y1; ++y) {
        for (int x = x0; x <= x1; ++x) {
            double z_m = 0.0;
            if (depth.type() == CV_16UC1) {
                const uint16_t z_mm = depth.at<uint16_t>(y, x);
                if (z_mm == 0) continue;
                z_m = static_cast<double>(z_mm) * 1e-3;
            } else if (depth.type() == CV_32FC1) {
                const float z = depth.at<float>(y, x);
                if (!std::isfinite(z) || z <= 0.0f) continue;
                z_m = static_cast<double>(z);
            } else {
                return 0.0;
            }

            if (z_m < depth_min_m_ || z_m > depth_max_m_) continue;
            samples.push_back(z_m);
        }
    }

    if (static_cast<int>(samples.size()) < depth_min_valid_count_) {
        return 0.0;
    }

    const size_t mid = samples.size() / 2;
    std::nth_element(samples.begin(), samples.begin() + mid, samples.end());
    return samples[mid];
}

}  // namespace rc26_vision
