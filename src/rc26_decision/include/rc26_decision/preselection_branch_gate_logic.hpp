#pragma once

#include <cstdint>

namespace rc26_decision {

enum class PreselectionBranchSelection { None, ContinueFirst, SwitchTarget };

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

} // namespace rc26_decision
