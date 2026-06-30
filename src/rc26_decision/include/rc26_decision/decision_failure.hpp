#pragma once

#include <behaviortree_cpp/blackboard.h>

#include <string>

namespace rc26_decision {

inline constexpr const char *kDecisionFailureSourceKey =
    "decision_last_failure_source";
inline constexpr const char *kDecisionFailureReasonKey =
    "decision_last_failure_reason";
inline constexpr const char *kDecisionFailureDetailKey =
    "decision_last_failure_detail";

inline void clearDecisionFailure(const BT::Blackboard::Ptr &blackboard) {
  if (!blackboard) {
    return;
  }
  blackboard->set(kDecisionFailureSourceKey, std::string(""));
  blackboard->set(kDecisionFailureReasonKey, std::string(""));
  blackboard->set(kDecisionFailureDetailKey, std::string(""));
}

inline void writeDecisionFailure(const BT::Blackboard::Ptr &blackboard,
                                 const std::string &source,
                                 const std::string &reason) {
  if (!blackboard) {
    return;
  }
  const std::string safe_source = source.empty() ? "未知来源" : source;
  const std::string safe_reason = reason.empty() ? "未知原因" : reason;
  blackboard->set(kDecisionFailureSourceKey, safe_source);
  blackboard->set(kDecisionFailureReasonKey, safe_reason);
  blackboard->set(kDecisionFailureDetailKey, safe_source + ": " + safe_reason);
}

inline std::string readDecisionFailureDetail(
    const BT::Blackboard::Ptr &blackboard) {
  if (!blackboard) {
    return "未知原因";
  }
  std::string detail;
  if (!blackboard->get(kDecisionFailureDetailKey, detail) || detail.empty()) {
    return "未知原因";
  }
  return detail;
}

} // namespace rc26_decision
