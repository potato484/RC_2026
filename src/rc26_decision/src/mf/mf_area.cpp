#include "rc26_decision/mf/mf_area.hpp"

namespace rc26_decision {

void registerMFAreaNodes(BT::BehaviorTreeFactory &factory) {
  factory.registerNodeType<SelectNextGridAction>("SelectNextGrid");
  factory.registerNodeType<PlanGridTransitionAction>("PlanGridTransition");
  registerGridHeadingNodes(factory);
  factory.registerNodeType<GridTransitionAction>("GridTransition");
  factory.registerNodeType<CheckExitCondition>("CheckExitCondition");
}

} // namespace rc26_decision
