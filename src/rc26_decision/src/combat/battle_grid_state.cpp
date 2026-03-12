#include "rc26_decision/combat/battle_grid_state.hpp"

namespace rc26_decision {

BattleGridState::BattleGridState() {
    reset();
}

void BattleGridState::reset(SlotState initial_state) {
    for (int grid = 1; grid <= kGridCount; ++grid) {
        for (int layer = 1; layer <= kLayerCount; ++layer) {
            slots_[static_cast<size_t>(grid)][static_cast<size_t>(layer)] = initial_state;
        }
    }
}

bool BattleGridState::setSlotState(int grid_position, int layer, SlotState state) {
    if (!isValidGridPosition(grid_position) || !isValidLayer(layer)) {
        return false;
    }
    slots_[static_cast<size_t>(grid_position)][static_cast<size_t>(layer)] = state;
    return true;
}

bool BattleGridState::markOccupied(int grid_position, int layer) {
    return setSlotState(grid_position, layer, SlotState::kOccupied);
}

bool BattleGridState::markEmpty(int grid_position, int layer) {
    return setSlotState(grid_position, layer, SlotState::kEmpty);
}

BattleGridState::SlotState BattleGridState::slotState(int grid_position, int layer) const {
    if (!isValidGridPosition(grid_position) || !isValidLayer(layer)) {
        return SlotState::kUnknown;
    }
    return slots_[static_cast<size_t>(grid_position)][static_cast<size_t>(layer)];
}

std::optional<int> BattleGridState::selectPlacementLayer(int grid_position, bool is_lifted) const {
    if (!isValidGridPosition(grid_position)) {
        return std::nullopt;
    }

    const int max_reachable_layer = is_lifted ? kLayerCount : 1;
    for (int layer = 1; layer <= max_reachable_layer; ++layer) {
        const auto state = slotState(grid_position, layer);
        if (state == SlotState::kEmpty) {
            return layer;
        }
    }
    return std::nullopt;
}

bool BattleGridState::isValidGridPosition(int grid_position) {
    return grid_position >= 1 && grid_position <= kGridCount;
}

bool BattleGridState::isValidLayer(int layer) {
    return layer >= 1 && layer <= kLayerCount;
}

}  // namespace rc26_decision
