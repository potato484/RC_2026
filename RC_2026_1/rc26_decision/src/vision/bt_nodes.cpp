#include "rc26_decision/vision/bt_nodes.hpp"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <string>

#include "rc26_vision/vision_inference_manager.hpp"

namespace {

std::string trimCopy(const std::string& input) {
    const auto is_space = [](unsigned char c) { return std::isspace(c) != 0; };

    auto begin_it = std::find_if_not(input.begin(), input.end(), is_space);
    if (begin_it == input.end()) {
        return {};
    }
    auto rbegin_it = std::find_if_not(input.rbegin(), input.rend(), is_space);
    return std::string(begin_it, rbegin_it.base());
}

std::string toLowerCopy(std::string s) {
    for (char& c : s) {
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    return s;
}

bool parseAttributeKind(const std::string& text, rc26_vision::AttributeKind& out) {
    const std::string trimmed = trimCopy(text);
    if (trimmed.empty()) {
        return false;
    }

    char* end = nullptr;
    const long value = std::strtol(trimmed.c_str(), &end, 10);
    if (end && *end == '\0') {
        switch (value) {
        case 0:
            out = rc26_vision::AttributeKind::Unknown;
            return true;
        case 1:
            out = rc26_vision::AttributeKind::R_R1;
            return true;
        case 2:
            out = rc26_vision::AttributeKind::B_R1;
            return true;
        case 3:
            out = rc26_vision::AttributeKind::Truth;
            return true;
        case 4:
            out = rc26_vision::AttributeKind::False;
            return true;
        default:
            return false;
        }
    }

    const std::string lower = toLowerCopy(trimmed);
    if (lower == "unknown") {
        out = rc26_vision::AttributeKind::Unknown;
        return true;
    }
    if (lower == "r_r1") {
        out = rc26_vision::AttributeKind::R_R1;
        return true;
    }
    if (lower == "b_r1") {
        out = rc26_vision::AttributeKind::B_R1;
        return true;
    }
    if (lower == "truth") {
        out = rc26_vision::AttributeKind::Truth;
        return true;
    }
    if (lower == "false") {
        out = rc26_vision::AttributeKind::False;
        return true;
    }

    return false;
}

}  // namespace

namespace rc26_decision {

VisionStartAction::VisionStartAction(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config) {}

BT::PortsList VisionStartAction::providedPorts() {
    return {};
}

BT::NodeStatus VisionStartAction::onStart() {
    std::shared_ptr<rc26_vision::VisionInferenceManager> manager;
    if (!config().blackboard->get("vision_manager", manager) || !manager) {
        config().blackboard->set("vision_running", false);
        config().blackboard->set("vision_ok", false);
        return BT::NodeStatus::FAILURE;
    }

    const bool started = manager->start();
    config().blackboard->set("vision_running", manager->isRunning());
    config().blackboard->set("vision_ok", started);
    return started ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus VisionStartAction::onRunning() {
    return BT::NodeStatus::SUCCESS;
}

void VisionStartAction::onHalted() {}

VisionStopAction::VisionStopAction(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config) {}

BT::PortsList VisionStopAction::providedPorts() {
    return {};
}

BT::NodeStatus VisionStopAction::onStart() {
    std::shared_ptr<rc26_vision::VisionInferenceManager> manager;
    if (!config().blackboard->get("vision_manager", manager) || !manager) {
        config().blackboard->set("vision_running", false);
        config().blackboard->set("vision_ok", false);
        return BT::NodeStatus::FAILURE;
    }

    manager->stop();
    config().blackboard->set("vision_running", false);
    config().blackboard->set("vision_ok", false);
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus VisionStopAction::onRunning() {
    return BT::NodeStatus::SUCCESS;
}

void VisionStopAction::onHalted() {}

WaitVisionTargetAction::WaitVisionTargetAction(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config) {}

BT::PortsList WaitVisionTargetAction::providedPorts() {
    return {
        BT::InputPort<std::string>("target_attr", "", "Expected target attribute (enum name like Truth or integer)"),
        BT::InputPort<double>("max_dist", -1.0, "Max valid distance (m). <=0 disables"),
        BT::InputPort<int>("timeout", 1000, "Timeout (ms) [0,10000]"),
    };
}

BT::NodeStatus WaitVisionTargetAction::onStart() {
    std::shared_ptr<rc26_vision::VisionInferenceManager> manager;
    if (!config().blackboard->get("vision_manager", manager) || !manager) {
        config().blackboard->set("vision_running", false);
        config().blackboard->set("vision_ok", false);
        return BT::NodeStatus::FAILURE;
    }
    manager_ = std::move(manager);

    rclcpp::Node* node_ptr = nullptr;
    if (config().blackboard->get("node", node_ptr) && node_ptr) {
        clock_ = node_ptr->get_clock();
    } else {
        clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    }
    start_time_ = clock_->now();

    int timeout_ms = 1000;
    if (!getInput("timeout", timeout_ms)) {
        return BT::NodeStatus::FAILURE;
    }
    timeout_ms = std::max(0, std::min(timeout_ms, 10'000));
    timeout_ms_ = timeout_ms;

    double max_dist_m = -1.0;
    if (!getInput("max_dist", max_dist_m)) {
        return BT::NodeStatus::FAILURE;
    }
    if (max_dist_m > 0.0) {
        max_dist_m_ = max_dist_m;
    } else {
        max_dist_m_.reset();
    }

    std::string target_attr_str;
    if (!getInput("target_attr", target_attr_str)) {
        return BT::NodeStatus::FAILURE;
    }
    target_attr_str = trimCopy(target_attr_str);
    if (target_attr_str.empty()) {
        target_attr_.reset();
    } else {
        rc26_vision::AttributeKind parsed = rc26_vision::AttributeKind::Unknown;
        if (!parseAttributeKind(target_attr_str, parsed)) {
            return BT::NodeStatus::FAILURE;
        }
        target_attr_ = parsed;
    }

    const auto immediate = onRunning();
    if (immediate == BT::NodeStatus::SUCCESS) {
        return BT::NodeStatus::SUCCESS;
    }
    if (timeout_ms_ == 0) {
        return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitVisionTargetAction::onRunning() {
    if (!manager_) {
        return BT::NodeStatus::FAILURE;
    }
    if (!clock_) {
        clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    }

    const auto now = clock_->now();
    int64_t elapsed_ns = (now - start_time_).nanoseconds();
    if (elapsed_ns < 0) {
        start_time_ = now;
        elapsed_ns = 0;
    }

    if (timeout_ms_ == 0) {
        // No waiting: evaluate once.
        // Fall through to evaluation and return SUCCESS/FAILURE.
    } else {
        const int64_t timeout_ns = static_cast<int64_t>(timeout_ms_) * 1'000'000LL;
        if (elapsed_ns >= timeout_ns) {
            return BT::NodeStatus::FAILURE;
        }
    }

    const bool running = manager_->isRunning();
    config().blackboard->set("vision_running", running);

    const bool ready = manager_->isReady();
    config().blackboard->set("vision_ok", ready);

    if (!running || !ready) {
        config().blackboard->set("vision_has_target", false);
        return timeout_ms_ == 0 ? BT::NodeStatus::FAILURE : BT::NodeStatus::RUNNING;
    }

    const rc26_vision::TargetResult result = manager_->getLatestResult();
    config().blackboard->set("vision_has_target", result.has_target);
    config().blackboard->set("vision_attr_kind", static_cast<int>(result.attr_kind));
    config().blackboard->set("vision_distance_m", result.distance_m);
    config().blackboard->set("vision_score", result.score);
    config().blackboard->set("vision_bbox_cx", result.bbox_cx);
    config().blackboard->set("vision_bbox_cy", result.bbox_cy);

    if (!result.has_target) {
        return timeout_ms_ == 0 ? BT::NodeStatus::FAILURE : BT::NodeStatus::RUNNING;
    }
    if (target_attr_.has_value() && result.attr_kind != *target_attr_) {
        return timeout_ms_ == 0 ? BT::NodeStatus::FAILURE : BT::NodeStatus::RUNNING;
    }
    if (max_dist_m_.has_value() && result.distance_m > *max_dist_m_) {
        return timeout_ms_ == 0 ? BT::NodeStatus::FAILURE : BT::NodeStatus::RUNNING;
    }
    return BT::NodeStatus::SUCCESS;
}

void WaitVisionTargetAction::onHalted() {
    manager_.reset();
    clock_.reset();
    target_attr_.reset();
    max_dist_m_.reset();
    timeout_ms_ = 1000;
    start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
}

void registerVisionNodes(BT::BehaviorTreeFactory& factory) {
    factory.registerNodeType<VisionStartAction>("VisionStart");
    factory.registerNodeType<VisionStopAction>("VisionStop");
    factory.registerNodeType<WaitVisionTargetAction>("WaitVisionTarget");
}

}  // namespace rc26_decision
