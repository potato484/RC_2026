// 武馆区视觉伺服夹取动作：内嵌直连相机 + 推理引擎，横移对齐端头后下发 GRAB_TIP，
// 夹取后端头持续消失即判定完成。业务逻辑复刻 rc26_vision/test/tip_vision_test_node.cpp。
#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>
#include <geometry_msgs/msg/twist.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

#include "rc26_interfaces/msg/mechanism_transport_feedback.hpp"
#include "rc26_interfaces/srv/send_mechanism_transport_command.hpp"
#include "rc26_vision/inference/config/model_profile.hpp"
#include "rc26_vision/inference/contracts/inference_engine.hpp"
#include "rc26_vision/postprocess/alignment/tip_alignment.hpp"

#include "mc_params.hpp"

namespace rc26_decision {

class VisualServoGrabAction : public BT::StatefulActionNode {
public:
    VisualServoGrabAction(const std::string& name, const BT::NodeConfig& config);
    ~VisualServoGrabAction() override;

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    using TwistMsg = geometry_msgs::msg::Twist;
    using FeedbackMsg = rc26_interfaces::msg::MechanismTransportFeedback;
    using SendCommandSrv = rc26_interfaces::srv::SendMechanismTransportCommand;

    enum class ServoPhase { Aligning, ApproachingLimit, SendingGrab, WaitingDone };
    enum class GrabStepStatus { Running, Success, Failure };

    void workerLoop();
    bool initEngine();
    bool initCamera();
    bool openCamera(int index, const std::string& path);
    void resolveTargetClassIds();
    rc26_vision::TipAlignmentConfig makeAlignmentConfig() const;
    double computeAlignmentVy(int offset_px) const;
    double computeApproachVx() const;
    void publishCmd(double vx, double vy, bool force);
    void publishStop(bool force);
    void setupFeedbackSubscription();
    void handleFeedback(const FeedbackMsg::SharedPtr msg);
    void beginApproach();
    GrabStepStatus tickGrabCommand();
    bool tryStartGrabCommand();
    void stopWorker();

    McParams params_;
    rclcpp::Node* node_{nullptr};
    rclcpp::Publisher<TwistMsg>::SharedPtr cmd_pub_;
    rclcpp::Subscription<FeedbackMsg>::SharedPtr feedback_sub_;
    rclcpp::Client<SendCommandSrv>::SharedPtr grab_client_;

    rc26_vision::InferenceEnginePtr engine_;
    std::vector<std::string> class_names_;
    std::vector<int> target_class_ids_;
    rc26_vision::TipTargetLockState target_lock_state_;
    cv::VideoCapture camera_;

    std::thread worker_;
    std::atomic<bool> stop_requested_{false};
    std::atomic<bool> done_{false};
    std::atomic<bool> failed_{false};

    ServoPhase phase_{ServoPhase::Aligning};
    int stable_count_{0};
    bool grab_attempted_{false};
    std::atomic<bool> grab_response_seen_{false};
    std::atomic<bool> grab_accepted_{false};
    std::atomic<uint64_t> grab_generation_{0};
    std::atomic<bool> waiting_for_limit_switch_{false};
    std::atomic<bool> limit_switch_triggered_{false};
    std::chrono::steady_clock::time_point start_tp_{};
    std::chrono::steady_clock::time_point approach_start_tp_{};
    std::chrono::steady_clock::time_point last_pub_tp_{};
    std::chrono::steady_clock::time_point last_grab_tp_{};
    std::chrono::steady_clock::time_point lost_since_tp_{};
    bool lost_active_{false};
};

}  // namespace rc26_decision
