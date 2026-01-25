#pragma once

#include <string>
#include <vector>
#include <memory>
#include <atomic>
#include <cstdint>
#include <opencv2/core.hpp>
#include "rc26_vision/types.hpp"

namespace rc26_vision {

class YoloEngine {
public:
    explicit YoloEngine(const std::string& model_path,
                        const std::vector<std::string>& class_names,
                        float conf_thresh = 0.5f,
                        float iou_thresh = 0.45f);
    ~YoloEngine();

    YoloEngine(const YoloEngine&) = delete;
    YoloEngine& operator=(const YoloEngine&) = delete;

    std::vector<Detection> infer(const cv::Mat& image);

    void setConfThresh(float thresh);
    void setIouThresh(float thresh);
    float getConfThresh() const;
    float getIouThresh() const;

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
