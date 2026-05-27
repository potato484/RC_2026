#include "rc26_vision/inference/onnx/onnx_runtime_engine.hpp"

#include <stdexcept>

namespace rc26_vision {

namespace {

[[noreturn]] void throwOnnxRuntimeUnavailable() {
    throw std::runtime_error(
        "ONNX Runtime 后端不可用：当前 rc26_vision 构建未启用 ONNX Runtime C++ 支持。请安装 "
        "ONNX Runtime C++ 头文件和库，或在 AidLux 环境使用 engine: auto/aidlite。");
}

}  // namespace

struct OpenCvOnnxEngine::Impl {};

OpenCvOnnxEngine::OpenCvOnnxEngine(const std::string& model_path,
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
    throwOnnxRuntimeUnavailable();
}

OpenCvOnnxEngine::~OpenCvOnnxEngine() = default;

std::vector<Detection> OpenCvOnnxEngine::infer(const cv::Mat& image) {
    (void)image;
    return {};
}

void OpenCvOnnxEngine::setConfThresh(float thresh) { conf_thresh_.store(thresh); }

void OpenCvOnnxEngine::setIouThresh(float thresh) { iou_thresh_.store(thresh); }

float OpenCvOnnxEngine::getConfThresh() const { return conf_thresh_.load(); }

float OpenCvOnnxEngine::getIouThresh() const { return iou_thresh_.load(); }

}  // namespace rc26_vision
