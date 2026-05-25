#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "rc26_vision/inference/inference_engine.hpp"
#include "rc26_vision/inference/model_profile.hpp"

namespace rc26_vision {

// Local ONNX inference backend. The current implementation is backed by ONNX Runtime.
class OpenCvOnnxEngine : public InferenceEngine {
public:
    explicit OpenCvOnnxEngine(const std::string& model_path,
                              const std::vector<std::string>& class_names,
                              const AidLiteConfig& config,
                              float conf_thresh = 0.5f,
                              float iou_thresh = 0.45f,
                              int input_w = 640,
                              int input_h = 640);
    ~OpenCvOnnxEngine() override;

    std::vector<Detection> infer(const cv::Mat& image) override;

    void setConfThresh(float thresh) override;
    void setIouThresh(float thresh) override;
    float getConfThresh() const override;
    float getIouThresh() const override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;

    std::vector<std::string> class_names_;
    std::atomic<float> conf_thresh_;
    std::atomic<float> iou_thresh_;
    int input_w_{640};
    int input_h_{640};
};

}  // namespace rc26_vision
