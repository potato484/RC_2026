#pragma once

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "rc26_vision/shared/vision_types.hpp"

namespace rc26_vision {

class InferenceEngine {
public:
    virtual ~InferenceEngine() = default;

    virtual std::vector<Detection> infer(const cv::Mat& image) = 0;

    virtual void setConfThresh(float thresh) = 0;
    virtual void setIouThresh(float thresh) = 0;
    virtual float getConfThresh() const = 0;
    virtual float getIouThresh() const = 0;

protected:
    InferenceEngine() = default;
    InferenceEngine(const InferenceEngine&) = delete;
    InferenceEngine& operator=(const InferenceEngine&) = delete;
};

using InferenceEnginePtr = std::unique_ptr<InferenceEngine>;

}  // namespace rc26_vision
