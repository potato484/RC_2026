// 武馆区结束 gate：把当前 USB 相机画面配准到启动基准帧，确认端头区域已稳定变化。
#pragma once

#include <chrono>
#include <string>

#include <behaviortree_cpp/bt_factory.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mc_params.hpp"

namespace rc26_decision {

class WaitForRegistrationConfirmAction : public BT::StatefulActionNode {
public:
    WaitForRegistrationConfirmAction(const std::string& name, const BT::NodeConfig& config);
    ~WaitForRegistrationConfirmAction() override;

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    bool initCamera();
    bool openCamera(int index, const std::string& path);
    void releaseCamera();
    bool loadReferenceFrame();
    bool detectRegisteredForeground(const cv::Mat& frame_bgr,
                                    int& changed_area_px,
                                    double& changed_ratio,
                                    double& match_score,
                                    int& inliers,
                                    cv::Rect& changed_bbox) const;
    cv::Rect makeRegistrationRoi(const cv::Size& size) const;
    cv::Rect makeForegroundRoi(const cv::Size& size) const;

    McParams params_;
    rclcpp::Node* node_{nullptr};
    cv::VideoCapture camera_;
    cv::Mat reference_gray_;
    std::chrono::steady_clock::time_point start_tp_{};
    std::chrono::steady_clock::time_point last_log_tp_{};
    int stable_count_{0};
};

}  // namespace rc26_decision
