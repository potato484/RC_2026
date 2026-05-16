#pragma once

#include "rc26_vision/engines/inference_engine.hpp"
#include "rc26_vision/runtime/model_profile.hpp"

namespace rc26_vision {

InferenceEnginePtr createInferenceEngine(const ModelProfile& profile);

}  // namespace rc26_vision
