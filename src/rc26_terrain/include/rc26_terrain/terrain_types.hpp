#pragma once

#include <cstdint>

namespace rc26_terrain {

enum class TerrainCellState : uint8_t {
    UNKNOWN = 0,
    GROUND = 1,
    CLIMBABLE_STEP = 2,
    FATAL_DROP = 3,
    OBSTACLE = 4,
    KFS_OCCUPIED = 5,
};

}  // namespace rc26_terrain
