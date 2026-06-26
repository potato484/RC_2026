#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "rc26_vision/shared/contracts/vision_types.hpp"

namespace rc26_vision {

struct VisualTargetSnapshot {
    std::string label;
    double distance_m{0.0};
    double score{0.0};
    int64_t sequence{0};
    double x1{0.0};
    double y1{0.0};
    double x2{0.0};
    double y2{0.0};
};

std::string visualTargetLabel(const Detection& detection);
VisualTargetSnapshot makeVisualTargetSnapshot(const Detection& detection, int64_t sequence);
double bboxIou(const VisualTargetSnapshot& a, const VisualTargetSnapshot& b);
bool isSameVisualTarget(const VisualTargetSnapshot& reference,
                        const VisualTargetSnapshot& candidate,
                        double iou_threshold);
bool isIgnoredVisualTarget(const VisualTargetSnapshot& candidate,
                           const std::vector<VisualTargetSnapshot>& ignored_targets,
                           double iou_threshold);

}  // namespace rc26_vision
