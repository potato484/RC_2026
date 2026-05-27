#pragma once

#include "rc26_vision/inference/config/model_profile.hpp"
#include "rc26_vision/inference/contracts/inference_engine.hpp"

namespace rc26_vision {

InferenceEnginePtr createInferenceEngine(const ModelProfile& profile);

}  // namespace rc26_vision
