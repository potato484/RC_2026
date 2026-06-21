#pragma once

#include <chrono>
#include <cmath>
#include <future>
#include <mutex>
#include <string>

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace rc26_decision {

template <class ActionT>
class BtActionNode : public BT::StatefulActionNode {
public:
    using Goal = typename ActionT::Goal;
    using Feedback = typename ActionT::Feedback;
    using GoalHandle = rclcpp_action::ClientGoalHandle<ActionT>;
    using WrappedResult = typename GoalHandle::WrappedResult;

    BtActionNode(const std::string& name, const BT::NodeConfig& config,
                 std::string action_name, std::chrono::milliseconds default_timeout)
        : BT::StatefulActionNode(name, config),
          action_name_(std::move(action_name)),
          default_timeout_(default_timeout) {}

    static BT::PortsList basePorts(double default_timeout_sec) {
        return {
            BT::InputPort<double>("timeout_sec", default_timeout_sec, "Action timeout in seconds"),
            BT::OutputPort<uint16_t>("error_code", "Action error code"),
        };
    }

    BT::NodeStatus onStart() override {
        rclcpp::Node* node = nullptr;
        if (!config().blackboard->get("node", node) || !node) {
            return failWithError(kErrorNoNode, "ACTION_NODE_MISSING",
                                 "missing rclcpp node on BT blackboard");
        }
        node_ = node;

        if (!client_) {
            client_ = rclcpp_action::create_client<ActionT>(node, action_name_);
        }

        double timeout_sec = toSeconds(default_timeout_);
        (void)getInput("timeout_sec", timeout_sec);
        timeout_ = toTimeout(timeout_sec, default_timeout_);
        start_time_ = std::chrono::steady_clock::now();

        {
            std::lock_guard<std::mutex> lock(result_mutex_);
            have_result_ = false;
            wrapped_result_ = WrappedResult{};
        }

        goal_handle_.reset();
        goal_handle_future_ = {};
        goal_sent_ = false;
        waiting_goal_handle_ = false;
        setErrorCode(0);

        return sendGoalIfServerReady();
    }

    BT::NodeStatus onRunning() override {
        if (std::chrono::steady_clock::now() - start_time_ > timeout_) {
            cancelGoal();
            if (!goal_sent_) {
                return failWithError(kErrorNoServer, "ACTION_SERVER_MISSING",
                                     "action server did not become available before timeout");
            }
            return failWithError(kErrorTimeout, "ACTION_TIMEOUT",
                                 "action timed out");
        }

        if (!goal_sent_) {
            return sendGoalIfServerReady();
        }

        if (waiting_goal_handle_) {
            if (!goal_handle_future_.valid() ||
                goal_handle_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
                return BT::NodeStatus::RUNNING;
            }
            goal_handle_ = goal_handle_future_.get();
            waiting_goal_handle_ = false;
            if (!goal_handle_) {
                return failWithError(kErrorRejected, "ACTION_REJECTED",
                                     "action goal was rejected");
            }
            onGoalAccepted();
        }

        WrappedResult result{};
        {
            std::lock_guard<std::mutex> lock(result_mutex_);
            if (!have_result_) {
                return BT::NodeStatus::RUNNING;
            }
            result = wrapped_result_;
            have_result_ = false;
        }

        uint16_t error_code = 0;
        const auto status = handleResult(result, error_code);
        setErrorCode(error_code);
        return status;
    }

    void onHalted() override {
        cancelGoal();
        onHaltHook();
    }

protected:
    virtual bool isActionReady(rclcpp::Node& /*node*/) { return true; }
    virtual bool buildGoal(Goal& goal) = 0;
    virtual void onFeedback(const std::shared_ptr<const Feedback>& /*feedback*/) {}
    virtual BT::NodeStatus handleResult(const WrappedResult& result, uint16_t& error_code) = 0;
    virtual void onGoalAccepted() {}
    virtual void onActionFailure(uint16_t /*error_code*/,
                                 const std::string& /*failure_code*/,
                                 const std::string& /*failure_reason*/) {}
    virtual void onHaltHook() {}

    static std::chrono::milliseconds toTimeout(double timeout_sec,
                                               std::chrono::milliseconds fallback) {
        if (!std::isfinite(timeout_sec) || timeout_sec <= 0.0) {
            return fallback;
        }
        return std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::duration<double>(timeout_sec));
    }

    static double toSeconds(std::chrono::milliseconds timeout) {
        return std::chrono::duration<double>(timeout).count();
    }

    void setErrorCode(uint16_t error_code) {
        (void)setOutput<uint16_t>("error_code", error_code);
    }

private:
    BT::NodeStatus sendGoalIfServerReady() {
        if (!node_ || !client_ ||
            !client_->wait_for_action_server(std::chrono::milliseconds(0))) {
            return BT::NodeStatus::RUNNING;
        }
        if (!isActionReady(*node_)) {
            return BT::NodeStatus::RUNNING;
        }

        Goal goal{};
        if (!buildGoal(goal)) {
            return failWithError(kErrorInvalidGoal, "INVALID_GOAL",
                                 "failed to build action goal");
        }

        {
            std::lock_guard<std::mutex> lock(result_mutex_);
            have_result_ = false;
            wrapped_result_ = WrappedResult{};
        }

        goal_handle_.reset();
        goal_handle_future_ = {};
        waiting_goal_handle_ = true;
        goal_sent_ = true;
        setErrorCode(0);

        typename rclcpp_action::Client<ActionT>::SendGoalOptions options;
        options.feedback_callback =
            [this](typename GoalHandle::SharedPtr /*goal_handle*/,
                   const std::shared_ptr<const Feedback> feedback) {
                onFeedback(feedback);
            };
        options.result_callback = [this](const WrappedResult& result) {
            std::lock_guard<std::mutex> lock(result_mutex_);
            wrapped_result_ = result;
            have_result_ = true;
        };

        goal_handle_future_ = client_->async_send_goal(goal, options);
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus failWithError(uint16_t error_code,
                                 const std::string& failure_code,
                                 const std::string& failure_reason) {
        setErrorCode(error_code);
        onActionFailure(error_code, failure_code, failure_reason);
        return BT::NodeStatus::FAILURE;
    }

    void cancelGoal() {
        if (client_ && goal_handle_) {
            (void)client_->async_cancel_goal(goal_handle_);
        }
        goal_handle_.reset();
        goal_sent_ = false;
        waiting_goal_handle_ = false;
    }

    static constexpr uint16_t kErrorNoNode = 0xFF01;
    static constexpr uint16_t kErrorNoServer = 0xFF02;
    static constexpr uint16_t kErrorInvalidGoal = 0xFF03;
    static constexpr uint16_t kErrorRejected = 0xFF04;
    static constexpr uint16_t kErrorTimeout = 0xFF05;

    std::string action_name_;
    std::chrono::milliseconds default_timeout_;
    std::chrono::milliseconds timeout_{std::chrono::seconds(8)};
    std::chrono::steady_clock::time_point start_time_{};
    rclcpp::Node* node_{nullptr};

    typename rclcpp_action::Client<ActionT>::SharedPtr client_;
    std::shared_future<typename GoalHandle::SharedPtr> goal_handle_future_;
    typename GoalHandle::SharedPtr goal_handle_;
    bool goal_sent_{false};
    bool waiting_goal_handle_{false};

    std::mutex result_mutex_;
    WrappedResult wrapped_result_{};
    bool have_result_{false};
};

}  // namespace rc26_decision
