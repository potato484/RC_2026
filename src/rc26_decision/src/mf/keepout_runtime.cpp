#include "rc26_decision/mf/keepout_runtime.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "behaviortree_cpp/decorator_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rc26_interfaces/srv/set_keepout_runtime.hpp"

namespace rc26_decision {

namespace {

using SetKeepoutRuntime = rc26_interfaces::srv::SetKeepoutRuntime;
using RuntimeFuture = rclcpp::Client<SetKeepoutRuntime>::FutureAndRequestId;
constexpr auto kServiceTimeout = std::chrono::milliseconds(3000);

class WithKeepoutRuntimeDecorator : public BT::DecoratorNode {
public:
  WithKeepoutRuntimeDecorator(const std::string& name, const BT::NodeConfig& config)
      : BT::DecoratorNode(name, config) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override {
    if (!child_node_) {
      recordTransitionReason("keepout runtime decorator missing child");
      return BT::NodeStatus::FAILURE;
    }
    if (!ensureClient()) {
      return BT::NodeStatus::FAILURE;
    }

    switch (pending_request_) {
      case PendingRequest::kActivate:
        return pollRuntimeResponse(/*for_activation=*/true);
      case PendingRequest::kDeactivate:
        return pollRuntimeResponse(/*for_activation=*/false);
      case PendingRequest::kNone:
        break;
    }

    if (!keepout_active_) {
      if (!sendRuntimeRequest(true, "enter_mf_subtree")) {
        recordTransitionReason("failed to request MF keepout activation");
        return BT::NodeStatus::FAILURE;
      }
      return BT::NodeStatus::RUNNING;
    }

    const auto child_status = child_node_->executeTick();
    if (child_status == BT::NodeStatus::RUNNING) {
      return BT::NodeStatus::RUNNING;
    }

    terminal_child_status_ = child_status;
    if (!sendRuntimeRequest(false, "leave_mf_subtree")) {
      recordTransitionReason("failed to request MF keepout deactivation");
      keepout_active_ = false;
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  void halt() override {
    if (child_node_ && child_node_->status() == BT::NodeStatus::RUNNING) {
      haltChild();
    }

    if (ensureClient()) {
      cleanup_only_request_ = true;
      if (!sendRuntimeRequest(false, "halt_mf_subtree")) {
        cleanup_only_request_ = false;
        recordTransitionReason(
            "failed to request MF keepout deactivation during halt");
      }
      terminal_child_status_ = BT::NodeStatus::IDLE;
    }
    keepout_active_ = false;
    BT::DecoratorNode::halt();
  }

private:
  enum class PendingRequest : uint8_t {
    kNone = 0,
    kActivate = 1,
    kDeactivate = 2,
  };

  bool ensureClient() {
    if (client_ && node_) {
      return true;
    }

    if (!config().blackboard || !config().blackboard->get("node", node_) || !node_) {
      recordTransitionReason("decision blackboard missing node for keepout runtime client");
      return false;
    }

    if (service_name_.empty()) {
      service_name_ = node_->get_parameter("keepout_runtime_service").as_string();
    }
    if (service_name_.empty()) {
      service_name_ = "/kfs_keepout/set_runtime";
    }

    client_ = node_->create_client<SetKeepoutRuntime>(service_name_);
    return static_cast<bool>(client_);
  }

  bool sendRuntimeRequest(const bool activate, const std::string& reason) {
    if (!client_) {
      return false;
    }
    if (!client_->wait_for_service(kServiceTimeout)) {
      recordTransitionReason("keepout runtime service unavailable: " + service_name_);
      return false;
    }

    auto request = std::make_shared<SetKeepoutRuntime::Request>();
    request->activate = activate;
    request->reason = reason;
    request_started_at_ = std::chrono::steady_clock::now();
    if (runtime_future_ && runtime_future_->valid()) {
      client_->remove_pending_request(*runtime_future_);
      runtime_future_.reset();
    }
    pending_request_ = activate ? PendingRequest::kActivate : PendingRequest::kDeactivate;
    if (activate) {
      cleanup_only_request_ = false;
    }
    runtime_future_.emplace(client_->async_send_request(request));
    return true;
  }

  BT::NodeStatus pollRuntimeResponse(const bool for_activation) {
    if (!runtime_future_ || !runtime_future_->valid()) {
      pending_request_ = PendingRequest::kNone;
      recordTransitionReason("keepout runtime future became invalid");
      keepout_active_ = false;
      return BT::NodeStatus::FAILURE;
    }

    if (runtime_future_->wait_for(std::chrono::milliseconds(0)) !=
        std::future_status::ready) {
      if (std::chrono::steady_clock::now() - request_started_at_ > kServiceTimeout) {
        client_->remove_pending_request(*runtime_future_);
        runtime_future_.reset();
        pending_request_ = PendingRequest::kNone;
        keepout_active_ = false;
        recordTransitionReason(for_activation
                                   ? "keepout runtime activation timed out"
                                   : "keepout runtime deactivation timed out");
        return BT::NodeStatus::FAILURE;
      }
      return BT::NodeStatus::RUNNING;
    }

    const auto response = runtime_future_->get();
    runtime_future_.reset();
    pending_request_ = PendingRequest::kNone;
    if (!response) {
      keepout_active_ = false;
      cleanup_only_request_ = false;
      recordTransitionReason(for_activation
                                 ? "empty keepout activation response"
                                 : "empty keepout deactivation response");
      return BT::NodeStatus::FAILURE;
    }

    if (for_activation) {
      if (!response->ok || !response->active) {
        keepout_active_ = false;
        cleanup_only_request_ = false;
        recordTransitionReason("MF keepout activation failed: " + response->message);
        return BT::NodeStatus::FAILURE;
      }
      keepout_active_ = true;
      recordTransitionReason("");
      return BT::NodeStatus::RUNNING;
    }

    keepout_active_ = false;
    if (!response->outputs_cleared) {
      cleanup_only_request_ = false;
      recordTransitionReason("MF keepout clear failed: " + response->message);
      return BT::NodeStatus::FAILURE;
    }
    if (response->component_loaded) {
      RCLCPP_WARN(node_->get_logger(),
                  "MF keepout outputs cleared but component remained loaded: %s",
                  response->message.c_str());
    }
    recordTransitionReason("");
    if (cleanup_only_request_) {
      cleanup_only_request_ = false;
      return BT::NodeStatus::RUNNING;
    }

    const auto child_status = terminal_child_status_;
    terminal_child_status_ = BT::NodeStatus::IDLE;
    return child_status == BT::NodeStatus::IDLE ? BT::NodeStatus::SUCCESS : child_status;
  }

  void recordTransitionReason(const std::string& reason) {
    if (config().blackboard) {
      config().blackboard->set("merlin_last_transition_reason", reason);
    }
    if (node_ && !reason.empty()) {
      RCLCPP_ERROR(node_->get_logger(), "%s", reason.c_str());
    }
  }

  rclcpp::Node* node_{nullptr};
  std::string service_name_;
  rclcpp::Client<SetKeepoutRuntime>::SharedPtr client_;
  std::optional<RuntimeFuture> runtime_future_;
  std::chrono::steady_clock::time_point request_started_at_{};
  PendingRequest pending_request_{PendingRequest::kNone};
  bool keepout_active_{false};
  bool cleanup_only_request_{false};
  BT::NodeStatus terminal_child_status_{BT::NodeStatus::IDLE};
};

}  // namespace

void registerKeepoutRuntimeNodes(BT::BehaviorTreeFactory& factory) {
  factory.registerNodeType<WithKeepoutRuntimeDecorator>("WithKeepoutRuntime");
}

}  // namespace rc26_decision
