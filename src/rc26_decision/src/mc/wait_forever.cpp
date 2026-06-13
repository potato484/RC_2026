#include "wait_forever.hpp"

namespace rc26_decision {

WaitForeverAction::WaitForeverAction(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config) {}

// onStart: 行为树第一次 tick 到本节点时调用，直接返回 RUNNING 表示无限期运行。
BT::NodeStatus WaitForeverAction::onStart() { return BT::NodeStatus::RUNNING; }

// onRunning: 后续每次 tick 均进入此函数，恒返回 RUNNING 使行为树在此节点永久停留。
BT::NodeStatus WaitForeverAction::onRunning() { return BT::NodeStatus::RUNNING; }

// onHalted: 行为树被外部中断时调用，无资源需要释放，空实现即可。
void WaitForeverAction::onHalted() {}

}  // namespace rc26_decision
