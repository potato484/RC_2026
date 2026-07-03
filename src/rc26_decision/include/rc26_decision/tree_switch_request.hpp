#pragma once

#include <filesystem>
#include <string>

#include <behaviortree_cpp/bt_factory.h>

namespace rc26_decision {

inline constexpr const char *kDecisionSwitchTreeRequestedKey =
    "decision_switch_tree_requested";
inline constexpr const char *kDecisionSwitchTreeFileKey =
    "decision_switch_tree_file";
inline constexpr const char *kPreselectionGateSecondStartDoneKey =
    "preselection_branch_gate_second_start_done";

inline void clearBehaviorTreeSwitchRequest(
    const BT::Blackboard::Ptr &blackboard) {
  if (!blackboard) {
    return;
  }
  blackboard->set(kDecisionSwitchTreeRequestedKey, false);
  blackboard->set(kDecisionSwitchTreeFileKey, std::string(""));
}

inline void requestBehaviorTreeSwitch(const BT::Blackboard::Ptr &blackboard,
                                      const std::string &tree_file) {
  if (!blackboard) {
    return;
  }
  blackboard->set(kDecisionSwitchTreeFileKey, tree_file);
  blackboard->set(kDecisionSwitchTreeRequestedKey, true);
}

inline bool consumeBehaviorTreeSwitchRequest(
    const BT::Blackboard::Ptr &blackboard, std::string &tree_file) {
  tree_file.clear();
  if (!blackboard) {
    return false;
  }
  bool requested = false;
  (void)blackboard->get(kDecisionSwitchTreeRequestedKey, requested);
  if (!requested) {
    return false;
  }
  (void)blackboard->get(kDecisionSwitchTreeFileKey, tree_file);
  clearBehaviorTreeSwitchRequest(blackboard);
  return !tree_file.empty();
}

inline bool shouldSwitchToRequestedTree(const std::string &current_tree_path,
                                        const std::string &target_tree_path,
                                        bool current_tree_active) {
  if (target_tree_path.empty()) {
    return false;
  }
  if (current_tree_path.empty()) {
    return true;
  }
  const bool already_on_target =
      std::filesystem::path(current_tree_path).lexically_normal() ==
      std::filesystem::path(target_tree_path).lexically_normal();
  return !already_on_target || !current_tree_active;
}

} // namespace rc26_decision
