#pragma once

#include <algorithm>
#include <cctype>
#include <string>

namespace rc26_merge_odom {

enum class WheelFeedbackFormat {
    kLegacy4Wheel16B,
};

inline std::string normalizeWheelFeedbackFormat(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (value == "legacy_4wheel_16b" || value == "legacy_4wheel" || value == "4wheel_16b" || value == "4wheel") {
        return "legacy_4wheel_16b";
    }
    return "legacy_4wheel_16b";
}

inline WheelFeedbackFormat parseWheelFeedbackFormat(const std::string& value) {
    (void)value;
    return WheelFeedbackFormat::kLegacy4Wheel16B;
}

inline const char* wheelFeedbackFormatName(WheelFeedbackFormat format) {
    switch (format) {
        case WheelFeedbackFormat::kLegacy4Wheel16B:
        default:
            return "legacy_4wheel_16b";
    }
}

}  // namespace rc26_merge_odom
