#include "rc26_vision/shared/target/visual_target_match.hpp"

#include <algorithm>
#include <string>

namespace rc26_vision {

std::string visualTargetLabel(const Detection& detection)
{
    if (!detection.class_name.empty()) {
        return detection.class_name;
    }
    return "class_" + std::to_string(detection.class_id);
}

VisualTargetSnapshot makeVisualTargetSnapshot(const Detection& detection, int64_t sequence)
{
    VisualTargetSnapshot snapshot;
    snapshot.label = visualTargetLabel(detection);
    snapshot.score = detection.score;
    snapshot.sequence = sequence;
    snapshot.x1 = detection.x1;
    snapshot.y1 = detection.y1;
    snapshot.x2 = detection.x2;
    snapshot.y2 = detection.y2;
    return snapshot;
}

double bboxIou(const VisualTargetSnapshot& a, const VisualTargetSnapshot& b)
{
    const double ax1 = std::min(a.x1, a.x2);
    const double ay1 = std::min(a.y1, a.y2);
    const double ax2 = std::max(a.x1, a.x2);
    const double ay2 = std::max(a.y1, a.y2);
    const double bx1 = std::min(b.x1, b.x2);
    const double by1 = std::min(b.y1, b.y2);
    const double bx2 = std::max(b.x1, b.x2);
    const double by2 = std::max(b.y1, b.y2);

    const double intersection_w = std::max(0.0, std::min(ax2, bx2) - std::max(ax1, bx1));
    const double intersection_h = std::max(0.0, std::min(ay2, by2) - std::max(ay1, by1));
    const double intersection_area = intersection_w * intersection_h;
    const double a_area = std::max(0.0, ax2 - ax1) * std::max(0.0, ay2 - ay1);
    const double b_area = std::max(0.0, bx2 - bx1) * std::max(0.0, by2 - by1);
    const double union_area = a_area + b_area - intersection_area;
    if (union_area <= 0.0) {
        return 0.0;
    }
    return intersection_area / union_area;
}

bool isSameVisualTarget(const VisualTargetSnapshot& reference,
                        const VisualTargetSnapshot& candidate,
                        double iou_threshold)
{
    return reference.label == candidate.label &&
           bboxIou(reference, candidate) >= std::max(0.0, iou_threshold);
}

bool isIgnoredVisualTarget(const VisualTargetSnapshot& candidate,
                           const std::vector<VisualTargetSnapshot>& ignored_targets,
                           double iou_threshold)
{
    return std::any_of(
        ignored_targets.begin(), ignored_targets.end(),
        [&candidate, iou_threshold](const VisualTargetSnapshot& ignored) {
            return isSameVisualTarget(ignored, candidate, iou_threshold);
        });
}

}  // namespace rc26_vision
