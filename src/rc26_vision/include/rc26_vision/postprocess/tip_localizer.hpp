#pragma once

#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rc26_interfaces/msg/tip_detection.hpp"
#include "rc26_interfaces/msg/tip_detection_array.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "rc26_vision/inference/inference_engine.hpp"

namespace rc26_vision {

class TipLocalizer : public rclcpp::Node {
public:
    explicit TipLocalizer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    struct CameraIntrinsics {
        bool ready{false};
        double fx{0.0};
        double fy{0.0};
        double cx{0.0};
        double cy{0.0};
        std::vector<double> dist;
    };

    void onColor(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
    void onDepth(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
    void onCameraInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg);

    std::optional<geometry_msgs::msg::PointStamped> toMapFrame(
        const geometry_msgs::msg::PointStamped& pt_camera);
    bool validateSpacing(std::vector<rc26_interfaces::msg::TipDetection>& detections) const;

    InferenceEnginePtr engine_;
    CameraIntrinsics intrinsics_;

    mutable std::mutex sensor_mutex_;
    sensor_msgs::msg::Image::ConstSharedPtr latest_depth_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
    rclcpp::Publisher<rc26_interfaces::msg::TipDetectionArray>::SharedPtr detections_pub_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    double spacing_expected_m_{0.2};
    double spacing_tolerance_m_{0.03};
    double max_depth_m_{5.0};
};

}  // namespace rc26_vision
