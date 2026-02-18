#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace rc26_vision {

enum class AttributeKind : int {
    Unknown = 0,
    R_R1 = 1,
    B_R1 = 2,
    Truth = 3,
    False = 4
};

struct Detection {
    float x1, y1, x2, y2;
    float score;
    int class_id;
    std::string class_name;
};

struct TargetResult {
    bool has_target = false;
    AttributeKind attr_kind = AttributeKind::Unknown;
    double distance_m = 0.0;
    double score = 0.0;
    int bbox_cx = 0;
    int bbox_cy = 0;
    int64_t timestamp_ns = 0;
};

using ResultCallback = std::function<void(const TargetResult&)>;

}  // namespace rc26_vision
