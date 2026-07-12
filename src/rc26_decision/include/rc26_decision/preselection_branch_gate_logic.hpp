#pragma once

#include <cstdint>
#include <sstream>
#include <string>

namespace rc26_decision {

enum class PreselectionBranchSelection { None, ContinueFirst, SwitchTarget };
enum class PreselectionStartProfile { Mc, Second };
enum class PreselectionBranchMode { Both, ContinueOnly, SwitchOnly };

inline PreselectionBranchSelection selectPreselectionBranch(
    uint8_t feedback_id, int continue_feedback_id, int switch_feedback_id) {
  if (feedback_id == static_cast<uint8_t>(continue_feedback_id & 0xFF)) {
    return PreselectionBranchSelection::ContinueFirst;
  }
  if (feedback_id == static_cast<uint8_t>(switch_feedback_id & 0xFF)) {
    return PreselectionBranchSelection::SwitchTarget;
  }
  return PreselectionBranchSelection::None;
}

inline bool isSameSeqDoneFeedback(uint8_t msg_seq, uint8_t msg_feedback_id,
                                  int command_seq, int done_feedback_id) {
  return command_seq >= 0 &&
         msg_seq == static_cast<uint8_t>(command_seq & 0xFF) &&
         msg_feedback_id == static_cast<uint8_t>(done_feedback_id & 0xFF);
}

inline PreselectionStartProfile parsePreselectionStartProfile(
    const std::string &profile) {
  return profile == "second" ? PreselectionStartProfile::Second
                             : PreselectionStartProfile::Mc;
}

inline PreselectionBranchMode parsePreselectionBranchMode(
    const std::string &mode) {
  if (mode == "continue_only") {
    return PreselectionBranchMode::ContinueOnly;
  }
  if (mode == "switch_only") {
    return PreselectionBranchMode::SwitchOnly;
  }
  return PreselectionBranchMode::Both;
}

inline bool isPreselectionBranchAllowed(
    PreselectionBranchSelection branch, PreselectionBranchMode mode) {
  if (branch == PreselectionBranchSelection::None) {
    return false;
  }
  if (mode == PreselectionBranchMode::Both) {
    return true;
  }
  if (mode == PreselectionBranchMode::ContinueOnly) {
    return branch == PreselectionBranchSelection::ContinueFirst;
  }
  return branch == PreselectionBranchSelection::SwitchTarget;
}

inline PreselectionStartProfile selectPreselectionStartProfile(
    PreselectionBranchSelection branch, const std::string &continue_profile,
    const std::string &switch_profile) {
  return parsePreselectionStartProfile(
      branch == PreselectionBranchSelection::SwitchTarget ? switch_profile
                                                          : continue_profile);
}

inline bool usesSecondPreselectionStart(PreselectionStartProfile profile) {
  return profile == PreselectionStartProfile::Second;
}

inline std::string makePreselectionGateStateJson(
    bool waiting, const std::string &gate_name,
    const std::string &accepted_branch,
    const std::string &continue_start_profile,
    const std::string &switch_start_profile) {
  std::ostringstream out;
  out << "{"
      << "\"waiting\":" << (waiting ? "true" : "false")
      << ",\"gate\":\"" << gate_name << "\""
      << ",\"accepted_branch\":\"" << accepted_branch << "\""
      << ",\"continue_start_profile\":\"" << continue_start_profile << "\""
      << ",\"switch_start_profile\":\"" << switch_start_profile << "\""
      << "}";
  return out.str();
}

} // namespace rc26_decision
