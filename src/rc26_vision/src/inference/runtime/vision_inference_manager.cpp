#include "rc26_vision/inference/runtime/engine_factory.hpp"
#include "rc26_vision/inference/runtime/vision_inference_manager.hpp"
#include "rc26_vision/shared/sensors/depth_roi_sampler.hpp"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>

#include <stdexcept>

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

    // 先销毁旧引擎，确保同一时刻最多一个引擎实例 (PBT INVARIANT)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        engine_.reset();
        class_names_.clear();
        active_model_id_.clear();
    }
    configured_.store(false, std::memory_order_relaxed);

    ModelProfile profile;
    profile.id = "__direct_config__";
    profile.engine = EngineType::Auto;
    profile.model_path = model_path;
    profile.labels = class_names;
    profile.conf_thresh = conf_thresh;

    try {
        auto engine = createInferenceEngine(profile);
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (running_.load(std::memory_order_relaxed) || inference_thread_.joinable()) {
                return false;
            }
            engine_ = std::move(engine);
            class_names_ = class_names;
            active_model_id_.clear();
        }
        configured_.store(true, std::memory_order_relaxed);
        vision_ok_.store(true, std::memory_order_relaxed);
        return true;
    } catch (...) {
        configured_.store(false, std::memory_order_relaxed);
        vision_ok_.store(false, std::memory_order_relaxed);
        return false;
    }
}

void VisionInferenceManager::loadConfig(const VisionConfig& config) {
    ProfileLoader::validate(config);
    config_ = config;
}

void VisionInferenceManager::selectModel(const std::string& model_id) {
    if (running_.load(std::memory_order_relaxed)) {
        throw std::runtime_error("Cannot select model while running");
    }

    auto it = config_.profiles.find(model_id);
    if (it == config_.profiles.end()) {
        // 清空状态后再抛异常，确保失败后不保留旧状态
        {
            std::lock_guard<std::mutex> lock(mutex_);
            engine_.reset();
            class_names_.clear();
            active_model_id_.clear();
        }
        configured_.store(false, std::memory_order_relaxed);
        vision_ok_.store(false, std::memory_order_relaxed);
        throw std::runtime_error("Unknown model_id: " + model_id);
    }

    const auto& profile = it->second;
    // 先销毁旧引擎，确保同一时刻最多一个引擎实例 (HC-01)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        engine_.reset();
        class_names_.clear();
        active_model_id_.clear();
    }
    configured_.store(false, std::memory_order_relaxed);

    try {
        InferenceEnginePtr new_engine = createInferenceEngine(profile);

        {
            std::lock_guard<std::mutex> lock(mutex_);
            engine_ = std::move(new_engine);
            class_names_ = profile.labels;
            active_model_id_ = model_id;
        }
        configured_.store(true, std::memory_order_relaxed);
        vision_ok_.store(true, std::memory_order_relaxed);
    } catch (...) {
        vision_ok_.store(false, std::memory_order_relaxed);
        throw;
    }
}

void VisionInferenceManager::switchModel(const std::string& model_id) {
    stop();
    selectModel(model_id);
}

std::string VisionInferenceManager::getActiveModel() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return active_model_id_;
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

bool VisionInferenceManager::getLatestFrameSnapshot(FrameSnapshot& snapshot_out) const {
    FrameSnapshot snapshot;
    sensor_msgs::msg::Image::ConstSharedPtr color_msg;
    sensor_msgs::msg::Image::ConstSharedPtr depth_msg;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        color_msg = latest_color_;
        depth_msg = latest_depth_;
    }

    if (color_msg) {
        try {
            snapshot.color_bgr = cv_bridge::toCvCopy(color_msg, "bgr8")->image.clone();
            snapshot.color_stamp_ns = rclcpp::Time(color_msg->header.stamp).nanoseconds();
            snapshot.has_color = true;
        } catch (...) {
            snapshot.has_color = false;
        }
    }

    if (depth_msg) {
        try {
            snapshot.depth = cv_bridge::toCvShare(depth_msg)->image.clone();
            snapshot.depth_stamp_ns = rclcpp::Time(depth_msg->header.stamp).nanoseconds();
            snapshot.has_depth = true;
        } catch (...) {
            snapshot.has_depth = false;
        }
    }

    {
        std::lock_guard<std::mutex> lock(display_mutex_);
        snapshot.detections = display_detections_;
        snapshot.result = display_result_;
        snapshot.display_stamp_ns = display_result_.timestamp_ns;
        snapshot.display_sequence = display_sequence_;
        snapshot.has_display = display_valid_;
    }
    if (!snapshot.has_display) {
        std::lock_guard<std::mutex> lock(result_mutex_);
        snapshot.result = latest_result_;
    }

    if (!snapshot.has_color && !snapshot.has_depth && !snapshot.has_display) {
        return false;
    }
    snapshot_out = std::move(snapshot);
    return true;
}

bool VisionInferenceManager::getLatestDisplay(cv::Mat& frame_out,
                                              std::vector<Detection>& dets_out,
                                              TargetResult& result_out) const {
    std::lock_guard<std::mutex> lock(display_mutex_);
    if (!display_valid_) {
        return false;
    }
    frame_out = display_frame_.clone();   // 深拷贝:node 可无锁绘制
    dets_out = display_detections_;
    result_out = display_result_;
    return true;
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

            cv::Mat display_frame_local;            // decode 成功即有原图(infer 抛异常仍可显示)
            std::vector<Detection> display_dets_local;

            const int64_t now_ns = node_.get_clock()->now().nanoseconds();
            if ((now_ns - result.timestamp_ns) > kFrameExpiryNs) {
                vision_ok_.store(false, std::memory_order_relaxed);
            } else {
                try {
                    if (!engine_) {
                        vision_ok_.store(false, std::memory_order_relaxed);
                    } else {
                        const cv::Mat color = cv_bridge::toCvCopy(color_msg, "bgr8")->image;
                        display_frame_local = color;
                        std::vector<Detection> detections = engine_->infer(color);
                        display_dets_local = detections;

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
                            result.class_id = best_det->class_id;

                            std::string class_name = best_det->class_name;
                            if (class_name.empty() && best_det->class_id >= 0 &&
                                best_det->class_id < static_cast<int>(class_names_.size())) {
                                class_name = class_names_[best_det->class_id];
                            }
                            result.class_name = class_name;
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

            if (!display_frame_local.empty()) {
                std::lock_guard<std::mutex> lock(display_mutex_);
                display_frame_ = display_frame_local;
                display_detections_ = std::move(display_dets_local);
                display_result_ = result;
                ++display_sequence_;
                display_valid_ = true;
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
    DepthRoiSamplerConfig config;
    config.roi_size = depth_roi_size_;
    config.min_valid_count = depth_min_valid_count_;
    config.min_depth_m = depth_min_m_;
    config.max_depth_m = depth_max_m_;
    const auto sampled = sampleMedianDepth(depth, cx, cy, config);
    return sampled.value_or(0.0);
}

}  // namespace rc26_vision
