#include "rc26_vision/aidlite_engine.hpp"

#include <stdexcept>

namespace rc26_vision {

namespace {

[[noreturn]] void throwAidLiteUnavailable() {
    throw std::runtime_error(
        "AidLite backend is unavailable: install AidLux/AidLite headers and library under /usr/local to enable rc26_vision inference.");
}

}  // namespace

struct AidLiteEngine::Impl {};

AidLiteEngine::AidLiteEngine(const std::string& model_path,
                             const std::vector<std::string>& class_names,
                             const AidLiteConfig& config,
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
    (void)config;
    throwAidLiteUnavailable();
}

AidLiteEngine::~AidLiteEngine() = default;

std::vector<Detection> AidLiteEngine::infer(const cv::Mat& image) {
    (void)image;
    return {};
}

void AidLiteEngine::setConfThresh(float thresh) { conf_thresh_.store(thresh); }

void AidLiteEngine::setIouThresh(float thresh) { iou_thresh_.store(thresh); }

float AidLiteEngine::getConfThresh() const { return conf_thresh_.load(); }

float AidLiteEngine::getIouThresh() const { return iou_thresh_.load(); }

cv::Mat AidLiteEngine::preprocess(const cv::Mat& image) {
    (void)image;
    return {};
}

std::vector<Detection> AidLiteEngine::postprocess(const std::vector<float>& output,
                                                  const std::vector<int64_t>& output_shape,
                                                  int orig_w,
                                                  int orig_h) {
    (void)output;
    (void)output_shape;
    (void)orig_w;
    (void)orig_h;
    return {};
}

void AidLiteEngine::nms(std::vector<Detection>& detections) { (void)detections; }

}  // namespace rc26_vision
