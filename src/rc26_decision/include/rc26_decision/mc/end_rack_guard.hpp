#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

namespace rc26_decision {

struct TipPose2D {
    double x;
    double y;
    double r;
};

struct ToolFootprint2D {
    double cx;
    double cy;
    double hw;
    double hh;
};

inline int countOverlaps(const ToolFootprint2D& footprint, const std::vector<TipPose2D>& tips) {
    int overlaps = 0;
    for (const auto& tip : tips) {
        const double nearest_x =
            std::clamp(tip.x, footprint.cx - footprint.hw, footprint.cx + footprint.hw);
        const double nearest_y =
            std::clamp(tip.y, footprint.cy - footprint.hh, footprint.cy + footprint.hh);
        const double dx = tip.x - nearest_x;
        const double dy = tip.y - nearest_y;
        if (dx * dx + dy * dy <= tip.r * tip.r) {
            ++overlaps;
        }
    }
    return overlaps;
}

}  // namespace rc26_decision
