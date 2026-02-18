#pragma once

#include <string>
#include <vector>
#include <memory>
#include <atomic>
#include <cstdint>
#include <opencv2/core.hpp>
#include "rc26_vision/inference_engine.hpp"
#include "rc26_vision/types.hpp"

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
    cv::Mat preprocess(const cv::Mat& image);
    std::vector<Detection> postprocess(const std::vector<float>& output,
                                        const std::vector<int64_t>& output_shape,
                                        int orig_w, int orig_h);
    void nms(std::vector<Detection>& detections);

    struct Impl;
    std::unique_ptr<Impl> impl_;

    std::vector<std::string> class_names_;
    std::atomic<float> conf_thresh_;
    std::atomic<float> iou_thresh_;
    int input_w_ = 640;
    int input_h_ = 640;
};

}  // namespace rc26_vision
