#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "rc26_vision/inference/contracts/inference_engine.hpp"
#include "rc26_vision/shared/contracts/vision_types.hpp"

namespace rc26_vision {

class YoloEngine : public InferenceEngine {
public:
    explicit YoloEngine(const std::string& model_path,
                        const std::vector<std::string>& class_names,
                        float conf_thresh = 0.5f,
                        float iou_thresh = 0.45f,
                        int input_w = 640,
                        int input_h = 640);
    ~YoloEngine() override;

    std::vector<Detection> infer(const cv::Mat& image) override;

    void setConfThresh(float thresh) override;
    void setIouThresh(float thresh) override;
    float getConfThresh() const override;
    float getIouThresh() const override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace rc26_vision
