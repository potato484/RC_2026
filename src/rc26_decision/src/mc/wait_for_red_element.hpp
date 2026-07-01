// 武馆区结束 gate：等待 USB 相机画面中出现稳定红色元素。
#pragma once

#include <chrono>
#include <string>

#include <behaviortree_cpp/bt_factory.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mc_params.hpp"

namespace rc26_decision {

class WaitForRedElementAction : public BT::StatefulActionNode {
public:
    WaitForRedElementAction(const std::string& name, const BT::NodeConfig& config);
    ~WaitForRedElementAction() override;

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    bool initCamera();
    bool openCamera(int index, const std::string& path);
    void releaseCamera();
    bool detectRedElement(const cv::Mat& frame_bgr, double& best_area_px) const;

    McParams params_;
    rclcpp::Node* node_{nullptr};
    cv::VideoCapture camera_;
    std::chrono::steady_clock::time_point start_tp_{};
    std::chrono::steady_clock::time_point last_log_tp_{};
    int stable_count_{0};
};

}  // namespace rc26_decision
