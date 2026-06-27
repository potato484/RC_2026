#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <vector>

namespace rc26_mechanism {

struct MechanismCommandCatalogEntry {
    uint8_t command_id{0};
    bool execute_supported{false};
    std::vector<uint8_t> terminal_success_feedback_ids;
    std::chrono::milliseconds default_timeout{0};
};

const std::vector<MechanismCommandCatalogEntry>& mechanismCommandCatalog();
const MechanismCommandCatalogEntry* findMechanismCommandCatalogEntry(uint8_t cmd_id);
bool isExecuteSupportedMechanismCommand(uint8_t cmd_id);
bool isTerminalSuccessFeedbackForMechanismCommand(uint8_t cmd_id, uint8_t fb_id);
bool isAnyMechanismTerminalSuccessFeedback(uint8_t fb_id);
bool isTerminalMechanismFeedback(uint8_t fb_id);
std::chrono::milliseconds defaultTimeoutForMechanismCommand(uint8_t cmd_id);
std::optional<uint8_t> defaultSimulatedSuccessFeedbackForMechanismCommand(uint8_t cmd_id);

}  // namespace rc26_mechanism
