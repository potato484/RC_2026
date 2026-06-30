#pragma once

#include "rc26_vision/inference/runtime/vision_inference_manager.hpp"

namespace rc26_vision::runtime_detail {

inline bool fillSnapshotColorFromDisplayFrame(
    VisionInferenceManager::FrameSnapshot& snapshot,
    const cv::Mat& display_frame) {
    if ((snapshot.has_color && !snapshot.color_bgr.empty()) ||
        !snapshot.has_display || display_frame.empty()) {
        return false;
    }

    snapshot.color_bgr = display_frame.clone();
    snapshot.color_stamp_ns = snapshot.display_stamp_ns;
    snapshot.has_color = !snapshot.color_bgr.empty();
    return snapshot.has_color;
}

}  // namespace rc26_vision::runtime_detail
