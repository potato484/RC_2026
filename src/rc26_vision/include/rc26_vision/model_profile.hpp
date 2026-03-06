#pragma once

#include <string>
#include <vector>
#include <optional>

namespace rc26_vision {

enum class EngineType {
    LocalOnnx,
    AidLite
};

struct AidLiteConfig {
    std::string framework_type;
    std::string accelerate_type;
    std::vector<int> input_shape;
    std::vector<std::vector<int>> output_shapes;
};

struct ModelProfile {
    std::string id;
    EngineType engine = EngineType::LocalOnnx;
    std::string model_path;
    std::vector<std::string> labels;
    float conf_thresh = 0.5f;
    float iou_thresh = 0.45f;
    int input_w = 640;
    int input_h = 640;
    std::optional<AidLiteConfig> aidlite;
};

}  // namespace rc26_vision
