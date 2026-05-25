#include "rc26_vision/runtime/inference_engine_factory.hpp"

#include <iostream>
#include <stdexcept>

#include "rc26_vision/engines/aidlite_engine.hpp"
#include "rc26_vision/engines/opencv_onnx_engine.hpp"
#include "rc26_vision/runtime/inference_backend_resolver.hpp"

namespace rc26_vision {

namespace {

AidLiteConfig makeDefaultOnnxCpuConfig() {
    AidLiteConfig config;
    config.framework_type = "onnx";
    config.accelerate_type = "cpu";
    return config;
}

AidLiteConfig normalizeAidLiteConfig(const ModelProfile& profile) {
    AidLiteConfig config = profile.aidlite.value_or(makeDefaultOnnxCpuConfig());
    if (config.framework_type.empty()) {
        config.framework_type = "onnx";
    }
    if (config.accelerate_type.empty()) {
        config.accelerate_type = "cpu";
    }
    return config;
}

}  // namespace

InferenceEnginePtr createInferenceEngine(const ModelProfile& profile) {
    const InferenceBackendSelection selection = resolveInferenceBackend(profile);
    std::cout << formatInferenceBackendSelectionLog(profile, selection) << std::endl;

    switch (selection.resolved_engine) {
        case EngineType::LocalOnnx: {
            const AidLiteConfig config = normalizeAidLiteConfig(profile);
            return std::make_unique<OpenCvOnnxEngine>(
                profile.model_path,
                profile.labels,
                config,
                profile.conf_thresh,
                profile.iou_thresh,
                profile.input_w,
                profile.input_h);
        }
        case EngineType::AidLite: {
            const AidLiteConfig config = normalizeAidLiteConfig(profile);
            return std::make_unique<AidLiteEngine>(
                profile.model_path,
                profile.labels,
                config,
                profile.conf_thresh,
                profile.iou_thresh,
                profile.input_w,
                profile.input_h);
        }
        case EngineType::AidLiteQnnYolo:
            throw std::runtime_error(
                "Profile '" + profile.id +
                "' requests the tip/QNN test backend, which is isolated from the default rc26_vision runtime path.");
        case EngineType::Auto:
            break;
    }

    throw std::runtime_error("Unsupported vision engine profile: " + profile.id);
}

}  // namespace rc26_vision
