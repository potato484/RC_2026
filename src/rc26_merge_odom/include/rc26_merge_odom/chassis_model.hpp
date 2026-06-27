#pragma once

#include <algorithm>
#include <cctype>
#include <string>

namespace rc26_merge_odom {

enum class ChassisModel {
    kMecanum4Wheel,
};

inline std::string normalizeChassisModel(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (value == "mecanum" || value == "mecanum_4wheel" || value == "omni" || value == "holonomic") {
        return "mecanum_4wheel";
    }
    return "mecanum_4wheel";
}

inline ChassisModel parseChassisModel(const std::string& value) {
    (void)value;
    return ChassisModel::kMecanum4Wheel;
}

inline const char* chassisModelName(ChassisModel model) {
    switch (model) {
        case ChassisModel::kMecanum4Wheel:
            return "mecanum_4wheel";
        default:
            return "mecanum_4wheel";
    }
}

inline bool isTrackedDiffModel(ChassisModel model) {
    (void)model;
    return false;
}

}  // namespace rc26_merge_odom
