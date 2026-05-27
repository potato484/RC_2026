#pragma once

namespace rc26_vision {

struct YoloImageTransform {
    int src_w{0};
    int src_h{0};
    float scale_x{1.0F};
    float scale_y{1.0F};
    int pad_x{0};
    int pad_y{0};
    bool letterbox{false};
};

}  // namespace rc26_vision
