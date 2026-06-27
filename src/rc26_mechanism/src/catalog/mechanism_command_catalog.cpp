#include "rc26_mechanism/catalog/mechanism_command_catalog.hpp"

#include <algorithm>

namespace rc26_mechanism {

namespace {

const std::vector<MechanismCommandCatalogEntry>& buildCatalog() {
    static const std::vector<MechanismCommandCatalogEntry> catalog{};
    return catalog;
}

}  // namespace

const std::vector<MechanismCommandCatalogEntry>& mechanismCommandCatalog() {
    return buildCatalog();
}

const MechanismCommandCatalogEntry* findMechanismCommandCatalogEntry(uint8_t cmd_id) {
    const auto& catalog = mechanismCommandCatalog();
    const auto it = std::find_if(
        catalog.begin(), catalog.end(),
        [cmd_id](const MechanismCommandCatalogEntry& entry) { return entry.command_id == cmd_id; });
    return (it != catalog.end()) ? &(*it) : nullptr;
}

bool isExecuteSupportedMechanismCommand(uint8_t cmd_id) {
    const auto* entry = findMechanismCommandCatalogEntry(cmd_id);
    return entry && entry->execute_supported;
}

bool isTerminalSuccessFeedbackForMechanismCommand(uint8_t cmd_id, uint8_t fb_id) {
    const auto* entry = findMechanismCommandCatalogEntry(cmd_id);
    if (!entry) {
        return false;
    }
    return std::find(entry->terminal_success_feedback_ids.begin(),
                     entry->terminal_success_feedback_ids.end(),
                     fb_id) != entry->terminal_success_feedback_ids.end();
}

bool isAnyMechanismTerminalSuccessFeedback(uint8_t fb_id) {
    const auto& catalog = mechanismCommandCatalog();
    return std::any_of(
        catalog.begin(), catalog.end(),
        [fb_id](const MechanismCommandCatalogEntry& entry) {
            return std::find(entry.terminal_success_feedback_ids.begin(),
                             entry.terminal_success_feedback_ids.end(),
                             fb_id) != entry.terminal_success_feedback_ids.end();
        });
}

bool isTerminalMechanismFeedback(uint8_t fb_id) {
    return isAnyMechanismTerminalSuccessFeedback(fb_id);
}

std::chrono::milliseconds defaultTimeoutForMechanismCommand(uint8_t cmd_id) {
    const auto* entry = findMechanismCommandCatalogEntry(cmd_id);
    return entry ? entry->default_timeout : std::chrono::seconds(8);
}

std::optional<uint8_t> defaultSimulatedSuccessFeedbackForMechanismCommand(uint8_t cmd_id) {
    const auto* entry = findMechanismCommandCatalogEntry(cmd_id);
    if (!entry || entry->terminal_success_feedback_ids.empty()) {
        return std::nullopt;
    }
    return entry->terminal_success_feedback_ids.front();
}

}  // namespace rc26_mechanism
