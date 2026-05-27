#pragma once

#include <string>

#include "rc26_vision/inference/config/model_profile.hpp"

namespace rc26_vision {

struct InferenceBackendRuntimeInfo {
    bool aidlite_paths_detected{false};
    bool aidlite_compiled{false};
};

struct InferenceBackendSelection {
    EngineType requested_engine{EngineType::Auto};
    EngineType resolved_engine{EngineType::LocalOnnx};
    bool aidlite_paths_detected{false};
    bool aidlite_compiled{false};
    std::string reason;
};

bool isAidLiteBackendCompiled();
InferenceBackendRuntimeInfo detectInferenceBackendRuntimeInfo();
InferenceBackendSelection resolveInferenceBackend(const ModelProfile& profile);
InferenceBackendSelection resolveInferenceBackend(const ModelProfile& profile,
                                                  const InferenceBackendRuntimeInfo& runtime_info);
std::string formatInferenceBackendSelectionLog(const ModelProfile& profile,
                                               const InferenceBackendSelection& selection);

}  // namespace rc26_vision
