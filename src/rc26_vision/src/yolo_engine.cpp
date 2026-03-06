#include "rc26_vision/yolo_engine.hpp"

#include "rc26_vision/aidlite_engine.hpp"

#include <memory>
#include <stdexcept>
#include <utility>

namespace rc26_vision {

struct YoloEngine::Impl {
    std::unique_ptr<AidLiteEngine> engine;
};

YoloEngine::YoloEngine(const std::string& model_path,
                       const std::vector<std::string>& class_names,
                       float conf_thresh,
                       float iou_thresh,
                       int input_w,
                       int input_h)
    : impl_(std::make_unique<Impl>()) {
    AidLiteConfig config;
    config.framework_type = "onnx";
    config.accelerate_type = "cpu";

    impl_->engine = std::make_unique<AidLiteEngine>(
        model_path, class_names, config, conf_thresh, iou_thresh, input_w, input_h);
}

YoloEngine::~YoloEngine() = default;

std::vector<Detection> YoloEngine::infer(const cv::Mat& image) {
    if (!impl_ || !impl_->engine) {
        return {};
    }
    return impl_->engine->infer(image);
}

void YoloEngine::setConfThresh(float thresh) {
    if (impl_ && impl_->engine) {
        impl_->engine->setConfThresh(thresh);
    }
}

void YoloEngine::setIouThresh(float thresh) {
    if (impl_ && impl_->engine) {
        impl_->engine->setIouThresh(thresh);
    }
}

float YoloEngine::getConfThresh() const {
    if (!impl_ || !impl_->engine) {
        return 0.0f;
    }
    return impl_->engine->getConfThresh();
}

float YoloEngine::getIouThresh() const {
    if (!impl_ || !impl_->engine) {
        return 0.0f;
    }
    return impl_->engine->getIouThresh();
}

}  // namespace rc26_vision
