#pragma once

#include <cstdint>
#include <filesystem>
#include <string>

#include "rc26_serial/protocol.hpp"

namespace rc26_decision {

struct MfPreselectionExternalTriggerConfig {
  bool enable{true};
  std::string feedback_topic{"/mechanism/command_feedback"};
  int feedback_id{
      static_cast<int>(rc26_serial::FeedbackID::MF_PRESELECTION_TRIGGER)};
  std::string tree_file{"mf_preselection_tree.xml"};
};

inline bool isMfPreselectionExternalTriggerFeedback(
    uint8_t feedback_id, const MfPreselectionExternalTriggerConfig &config) {
  return config.enable &&
         feedback_id == static_cast<uint8_t>(config.feedback_id & 0xFF);
}

inline bool shouldReloadForMfPreselectionTrigger(
    const std::string &current_tree_path, const std::string &target_tree_path,
    bool pending_trigger, bool current_tree_active) {
  if (pending_trigger) {
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

}  // namespace rc26_decision
