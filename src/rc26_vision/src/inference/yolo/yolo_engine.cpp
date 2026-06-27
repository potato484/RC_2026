#include "rc26_vision/inference/yolo/yolo_engine.hpp"

#include "rc26_vision/inference/runtime/engine_factory.hpp"

#include <memory>
#include <stdexcept>
#include <utility>

namespace rc26_vision {

struct YoloEngine::Impl {
    InferenceEnginePtr engine;
};

YoloEngine::YoloEngine(const std::string& model_path,
                       const std::vector<std::string>& class_names,
                       float conf_thresh,
                       float iou_thresh,
                       int input_w,
                       int input_h)
    : impl_(std::make_unique<Impl>()) {
    ModelProfile profile;
    profile.id = model_path;
    profile.engine = EngineType::Auto;
    profile.model_path = model_path;
    profile.labels = class_names;
    profile.conf_thresh = conf_thresh;
    profile.iou_thresh = iou_thresh;
    profile.input_w = input_w;
    profile.input_h = input_h;
    impl_->engine = createInferenceEngine(profile);
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
