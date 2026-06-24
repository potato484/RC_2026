// MF area aggregate header.
#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include "rc26_decision/mf/conditions.hpp"
#include "rc26_decision/mf/grid_heading.hpp"
#include "rc26_decision/mf/grid_transition.hpp"
#include "rc26_decision/mf/grid_transition_plan.hpp"
#include "rc26_decision/mf/merlin_map.hpp"
#include "rc26_decision/mf/select_next_grid.hpp"

namespace rc26_decision {

void registerMFAreaNodes(BT::BehaviorTreeFactory &factory);

} // namespace rc26_decision
