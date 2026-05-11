#pragma once

#include <string>
#include <vector>
#include <optional>

namespace rc26_vision {

enum class EngineType {
    LocalOnnx,
    AidLite,
    AidLiteQnnYolo
};

struct AidLiteConfig {
    std::string framework_type;
    std::string accelerate_type;
    std::vector<int> input_shape;
    std::vector<std::vector<int>> output_shapes;
    std::string input_name;
    std::string output_name;
    std::string resize_mode{"stretch"};
    std::string padding_color{"114"};
    double input_scale{1.0 / 255.0};
    double input_offset{0.0};
    double output_scale{1.0};
    double output_offset{0.0};
    bool split_output_quantization{false};
    double bbox_output_scale{1.0};
    double bbox_output_offset{0.0};
    double score_output_scale{1.0};
    double score_output_offset{0.0};
    int num_classes{0};
    bool use_dsp{true};
    bool enable_cpu_fallback{true};
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
