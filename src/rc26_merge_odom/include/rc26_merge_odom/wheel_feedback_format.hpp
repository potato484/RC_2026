#pragma once

#include <algorithm>
#include <cctype>
#include <string>

namespace rc26_merge_odom {

enum class WheelFeedbackFormat {
    kLegacy4Wheel16B,
    kTrackedLeftRight8B,
};

inline std::string normalizeWheelFeedbackFormat(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (value == "tracked_lr_8b" || value == "tracked_lr" || value == "left_right_8b" || value == "left_right" ||
        value == "lr_8b" || value == "lr") {
        return "tracked_lr_8b";
    }
    return "legacy_4wheel_16b";
}

inline WheelFeedbackFormat parseWheelFeedbackFormat(const std::string& value) {
    return normalizeWheelFeedbackFormat(value) == "tracked_lr_8b" ? WheelFeedbackFormat::kTrackedLeftRight8B
                                                                  : WheelFeedbackFormat::kLegacy4Wheel16B;
}

inline const char* wheelFeedbackFormatName(WheelFeedbackFormat format) {
    switch (format) {
        case WheelFeedbackFormat::kTrackedLeftRight8B:
            return "tracked_lr_8b";
        case WheelFeedbackFormat::kLegacy4Wheel16B:
        default:
            return "legacy_4wheel_16b";
    }
}

}  // namespace rc26_merge_odom
