#pragma once

#include <cstdint>
#include <cstring>
#include <vector>

namespace rc26_merge_odom {

struct WheelSpeedPayload {
    double v_fl = 0.0;
    double v_rl = 0.0;
    double v_rr = 0.0;
    double v_fr = 0.0;
};

inline bool parseWheelSpeedPayload(const std::vector<uint8_t>& payload, WheelSpeedPayload& out) {
    constexpr size_t kPayloadSize = sizeof(float) * 4U;
    if (payload.size() != kPayloadSize) {
        return false;
    }

    float v_fl = 0.0f;
    float v_rl = 0.0f;
    float v_rr = 0.0f;
    float v_fr = 0.0f;
    std::memcpy(&v_fl, &payload[0], sizeof(float));
    std::memcpy(&v_rl, &payload[4], sizeof(float));
    std::memcpy(&v_rr, &payload[8], sizeof(float));
    std::memcpy(&v_fr, &payload[12], sizeof(float));

    out.v_fl = static_cast<double>(v_fl);
    out.v_rl = static_cast<double>(v_rl);
    out.v_rr = static_cast<double>(v_rr);
    out.v_fr = static_cast<double>(v_fr);
    return true;
}

}  // namespace rc26_merge_odom
