#include "mc_preselection_repeat_control.hpp"

#include <algorithm>
#include <cmath>

#include "rc26_decision/mc_preselection_repeat_logic.hpp"

namespace rc26_decision {

MCPreselectionRepeatControl::MCPreselectionRepeatControl(
    const std::string &name, const BT::NodeConfig &config)
    : BT::DecoratorNode(name, config) {}

BT::PortsList MCPreselectionRepeatControl::providedPorts() {
  return {
      BT::InputPort<bool>("enabled", true,
                          "Whether MC preselection repeat is enabled"),
      BT::InputPort<int>("max_repeat_count", 1,
                         "Maximum repeats after the first MC run"),
      BT::InputPort<double>("forward_step_m", 0.2,
                            "Signed forward X delta added per repeat")};
}

void MCPreselectionRepeatControl::resetState() {
  enabled_ = true;
  max_repeat_count_ = 1;
  forward_step_m_ = 0.2;
  (void)getInput("enabled", enabled_);
  (void)getInput("max_repeat_count", max_repeat_count_);
  (void)getInput("forward_step_m", forward_step_m_);

  max_repeat_count_ = std::max(0, max_repeat_count_);
  if (!std::isfinite(forward_step_m_)) {
    forward_step_m_ = 0.2;
  }
  run_index_ = 0;
  repeat_count_ = 0;
  initialized_ = true;
  publishCurrentDistance();
}

void MCPreselectionRepeatControl::publishCurrentDistance() {
  if (!config().blackboard) {
    return;
  }
  double initial_forward_x_m = 0.2;
  (void)config().blackboard->get("mc_nav_forward_x_m", initial_forward_x_m);
  if (!std::isfinite(initial_forward_x_m)) {
    initial_forward_x_m = 0.2;
  }
  const double effective_forward_x_m =
      mcPreselectionEffectiveForwardX(initial_forward_x_m, forward_step_m_,
                                      repeat_count_);
  config().blackboard->set("mc_preselection_run_index", run_index_);
  config().blackboard->set("mc_preselection_repeat_count", repeat_count_);
  config().blackboard->set("mc_preselection_effective_forward_x_m",
                           effective_forward_x_m);
}

BT::NodeStatus MCPreselectionRepeatControl::tick() {
  if (!child_node_) {
    return BT::NodeStatus::FAILURE;
  }
  if (!initialized_ || status() == BT::NodeStatus::IDLE) {
    resetState();
  }

  setStatus(BT::NodeStatus::RUNNING);
  publishCurrentDistance();

  const BT::NodeStatus child_status = child_node_->executeTick();
  if (child_status == BT::NodeStatus::RUNNING) {
    return BT::NodeStatus::RUNNING;
  }
  if (child_status == BT::NodeStatus::FAILURE) {
    initialized_ = false;
    return BT::NodeStatus::FAILURE;
  }

  if (enabled_ && repeat_count_ < max_repeat_count_) {
    ++repeat_count_;
    ++run_index_;
    resetChild();
    publishCurrentDistance();
    return BT::NodeStatus::RUNNING;
  }

  initialized_ = false;
  return BT::NodeStatus::SUCCESS;
}

void MCPreselectionRepeatControl::halt() {
  initialized_ = false;
  DecoratorNode::halt();
}

} // namespace rc26_decision
