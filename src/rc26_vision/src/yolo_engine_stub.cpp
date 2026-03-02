#include "rc26_vision/yolo_engine.hpp"

#include <stdexcept>
#include <utility>

namespace rc26_vision {

struct YoloEngine::Impl {};

YoloEngine::YoloEngine(const std::string& model_path,
                       const std::vector<std::string>& class_names,
                       float conf_thresh,
                       float iou_thresh,
                       int input_w,
                       int input_h)
    : impl_(std::make_unique<Impl>()),
      class_names_(class_names),
      conf_thresh_(conf_thresh),
      iou_thresh_(iou_thresh),
      input_w_(input_w),
      input_h_(input_h) {
    (void)model_path;
    throw std::runtime_error(
        "YoloEngine requires ONNX Runtime, but ONNX Runtime was not found at build time.");
}

YoloEngine::~YoloEngine() = default;

std::vector<Detection> YoloEngine::infer(const cv::Mat& image) {
    (void)image;
    return {};
}

void YoloEngine::setConfThresh(float thresh) { conf_thresh_.store(thresh); }
void YoloEngine::setIouThresh(float thresh) { iou_thresh_.store(thresh); }
float YoloEngine::getConfThresh() const { return conf_thresh_.load(); }
float YoloEngine::getIouThresh() const { return iou_thresh_.load(); }

cv::Mat YoloEngine::preprocess(const cv::Mat& image) {
    (void)image;
    return {};
}

std::vector<Detection> YoloEngine::postprocess(const std::vector<float>& output,
                                               const std::vector<int64_t>& output_shape,
                                               int orig_w, int orig_h) {
    (void)output;
    (void)output_shape;
    (void)orig_w;
    (void)orig_h;
    return {};
}

void YoloEngine::nms(std::vector<Detection>& detections) {
    (void)detections;
}

}  // namespace rc26_vision
