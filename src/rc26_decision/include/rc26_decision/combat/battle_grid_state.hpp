// 对抗区九宫格离散状态模型
#pragma once

#include <array>
#include <cstdint>
#include <optional>

namespace rc26_decision {

class BattleGridState {
public:
    enum class SlotState : uint8_t {
        kUnknown = 0,
        kEmpty = 1,
        kOccupied = 2,
    };

    BattleGridState();

    void reset(SlotState initial_state = SlotState::kEmpty);

    bool setSlotState(int grid_position, int layer, SlotState state);
    bool markOccupied(int grid_position, int layer);
    bool markEmpty(int grid_position, int layer);

    SlotState slotState(int grid_position, int layer) const;

    // 根据当前离散状态和是否被抬起，决策可放置层。
    // 地面状态仅允许 1 层；被抬起状态允许 1~3 层。
    std::optional<int> selectPlacementLayer(int grid_position, bool is_lifted) const;

    static bool isValidGridPosition(int grid_position);
    static bool isValidLayer(int layer);

private:
    static constexpr int kGridCount = 9;
    static constexpr int kLayerCount = 3;

    // [grid_position 1..9][layer 1..3]
    std::array<std::array<SlotState, kLayerCount + 1>, kGridCount + 1> slots_{};
};

}  // namespace rc26_decision
