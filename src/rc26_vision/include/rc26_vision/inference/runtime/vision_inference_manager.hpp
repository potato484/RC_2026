#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "rc26_vision/inference/config/model_profile_loader.hpp"
#include "rc26_vision/inference/contracts/inference_engine.hpp"
#include "rc26_vision/shared/contracts/vision_types.hpp"

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

    struct FrameSnapshot {
        cv::Mat color_bgr;
        cv::Mat depth;
        std::vector<Detection> detections;
        TargetResult result;
        int64_t color_stamp_ns{0};
        int64_t depth_stamp_ns{0};
        int64_t display_stamp_ns{0};
        int64_t display_sequence{0};
        bool has_color{false};
        bool has_depth{false};
        bool has_display{false};
    };

    // 深拷贝最近彩色帧、深度帧、完整检测结果和目标结果，供决策层只读消费。
    // 首帧前返回 false；返回 true 时调用方可无锁访问输出数据。
    bool getLatestFrameSnapshot(FrameSnapshot& snapshot_out) const;

    // 快照最近可显示帧 + 完整检测 + 最新目标;首帧前返回 false。
    // frame_out 为深拷贝,调用方可安全绘制,无需持锁。
    bool getLatestDisplay(cv::Mat& frame_out,
                          std::vector<Detection>& dets_out,
                          TargetResult& result_out) const;

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

    // Display cache (node 主线程读取用于 overlay)
    cv::Mat display_frame_;
    std::vector<Detection> display_detections_;
    TargetResult display_result_;
    int64_t display_sequence_ = 0;
    bool display_valid_ = false;
    mutable std::mutex display_mutex_;

    // Parameters
    int depth_roi_size_ = 7;
    int depth_min_valid_count_ = 10;
    double depth_min_m_ = 0.6;
    double depth_max_m_ = 1.2;
};

}  // namespace rc26_vision
