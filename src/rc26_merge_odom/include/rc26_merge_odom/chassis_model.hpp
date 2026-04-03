#pragma once

#include <algorithm>
#include <cctype>
#include <string>

namespace rc26_merge_odom {

enum class ChassisModel {
    kMecanum4Wheel,
    kTrackedDiff,
};

inline std::string normalizeChassisModel(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (value == "tracked" || value == "tracked_diff" || value == "track" || value == "diff" ||
        value == "differential") {
        return "tracked_diff";
    }
    return "tracked_diff";
}

inline ChassisModel parseChassisModel(const std::string& value) {
    return normalizeChassisModel(value) == "tracked_diff" ? ChassisModel::kTrackedDiff
                                                           : ChassisModel::kMecanum4Wheel;
}

inline const char* chassisModelName(ChassisModel model) {
    switch (model) {
        case ChassisModel::kTrackedDiff:
            return "tracked_diff";
        case ChassisModel::kMecanum4Wheel:
            return "mecanum_4wheel";
        default:
            return "tracked_diff";
    }
}

inline bool isTrackedDiffModel(ChassisModel model) {
    return model == ChassisModel::kTrackedDiff;
}

}  // namespace rc26_merge_odom
