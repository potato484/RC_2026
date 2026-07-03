#pragma once

#include <cstdint>
#include <string>

namespace rc26_decision {

enum class PreselectionBranchSelection { None, ContinueFirst, SwitchTarget };
enum class PreselectionStartProfile { Mc, Second };

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

} // namespace rc26_decision
