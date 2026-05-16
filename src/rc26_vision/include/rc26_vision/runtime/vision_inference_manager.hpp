#pragma once

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "rc26_vision/engines/inference_engine.hpp"
#include "rc26_vision/runtime/profile_loader.hpp"
#include "rc26_vision/runtime/types.hpp"

namespace cv {
class Mat;
}

namespace rc26_vision {

class VisionInferenceManager {
public:
    explicit VisionInferenceManager(rclcpp::Node& node);
    ~VisionInferenceManager();

    VisionInferenceManager(const VisionInferenceManager&) = delete;
    VisionInferenceManager& operator=(const VisionInferenceManager&) = delete;

    bool configure(const std::string& model_path,
                   const std::vector<std::string>& class_names,
                   float conf_thresh);

    void loadConfig(const VisionConfig& config);
    void selectModel(const std::string& model_id);
    void switchModel(const std::string& model_id);
    std::string getActiveModel() const;

    bool start();
    void stop();
    bool isRunning() const;
    bool isReady() const;

    TargetResult getLatestResult() const;
    void setResultCallback(ResultCallback cb);

private:
    void inferenceLoop();
    void onColorImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
    void onDepthImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
    void onCameraInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg);
    double computeDepth(const cv::Mat& depth, int cx, int cy);

    rclcpp::Node& node_;
    InferenceEnginePtr engine_;
    std::vector<std::string> class_names_;
    VisionConfig config_;
    std::string active_model_id_;

    // Subscriptions
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;

    // Thread control
    std::atomic<bool> running_{false};
    std::atomic<bool> stop_requested_{false};
    std::atomic<bool> configured_{false};
    std::atomic<bool> vision_ok_{true};
    std::thread inference_thread_;
    std::condition_variable cv_;
    mutable std::mutex mutex_;

    // Frame cache
    sensor_msgs::msg::Image::ConstSharedPtr latest_color_;
    sensor_msgs::msg::Image::ConstSharedPtr latest_depth_;
    sensor_msgs::msg::CameraInfo::ConstSharedPtr latest_info_;
    std::atomic<int64_t> last_frame_time_ns_{0};

    // Result
    TargetResult latest_result_;
    mutable std::mutex result_mutex_;
    ResultCallback result_callback_;

    // Parameters
    int depth_roi_size_ = 7;
    int depth_min_valid_count_ = 10;
    double depth_min_m_ = 0.2;
    double depth_max_m_ = 5.0;
};

}  // namespace rc26_vision
