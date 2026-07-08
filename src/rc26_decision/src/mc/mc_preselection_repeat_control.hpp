#pragma once

#include <behaviortree_cpp/decorator_node.h>

namespace rc26_decision {

class MCPreselectionRepeatControl : public BT::DecoratorNode {
public:
  MCPreselectionRepeatControl(const std::string &name,
                              const BT::NodeConfig &config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
  void halt() override;

private:
  void resetState();
  void publishCurrentDistance();

  bool initialized_{false};
  bool enabled_{true};
  int run_index_{0};
  int repeat_count_{0};
  int max_repeat_count_{2};
  double forward_step_m_{0.2};
};

} // namespace rc26_decision
