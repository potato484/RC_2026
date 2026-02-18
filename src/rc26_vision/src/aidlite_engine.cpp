#include "rc26_vision/aidlite_engine.hpp"

#include <aidlux/aidlite/aidlite.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <cstring>
#include <stdexcept>
#include <utility>

using namespace aidlite;

namespace rc26_vision {

namespace {

FrameworkType parseFrameworkType(const std::string& type) {
    if (type == "QNN" || type == "qnn") return FrameworkType::TYPE_QNN;
    if (type == "QNN236" || type == "qnn236") return FrameworkType::TYPE_QNN236;
    if (type == "QNN231" || type == "qnn231") return FrameworkType::TYPE_QNN231;
    if (type == "QNN229" || type == "qnn229") return FrameworkType::TYPE_QNN229;
    if (type == "QNN223" || type == "qnn223") return FrameworkType::TYPE_QNN223;
    if (type == "QNN216" || type == "qnn216") return FrameworkType::TYPE_QNN216;
    if (type == "SNPE" || type == "snpe") return FrameworkType::TYPE_SNPE;
    if (type == "SNPE2" || type == "snpe2") return FrameworkType::TYPE_SNPE2;
    if (type == "TFLITE" || type == "tflite") return FrameworkType::TYPE_TFLITE;
    if (type == "RKNN" || type == "rknn") return FrameworkType::TYPE_RKNN;
    if (type == "NCNN" || type == "ncnn") return FrameworkType::TYPE_NCNN;
    if (type == "MNN" || type == "mnn") return FrameworkType::TYPE_MNN;
    if (type == "ONNX" || type == "onnx") return FrameworkType::TYPE_ONNX;
    return FrameworkType::TYPE_DEFAULT;
}

AccelerateType parseAccelerateType(const std::string& type) {
    if (type == "CPU" || type == "cpu") return AccelerateType::TYPE_CPU;
    if (type == "GPU" || type == "gpu") return AccelerateType::TYPE_GPU;
    if (type == "DSP" || type == "dsp") return AccelerateType::TYPE_DSP;
    if (type == "NPU" || type == "npu") return AccelerateType::TYPE_NPU;
    return AccelerateType::TYPE_CPU;
}

}  // namespace

struct AidLiteEngine::Impl {
    std::unique_ptr<Interpreter> interpreter;
    AidLiteConfig config;
    std::vector<int64_t> output_shape;
};

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
    impl_->config = config;

    Model* model = Model::create_instance(model_path);
    if (!model) {
        throw std::runtime_error("AidLiteEngine: failed to create Model");
    }

    Config* cfg = Config::create_instance();
    if (!cfg) {
        delete model;
        throw std::runtime_error("AidLiteEngine: failed to create Config");
    }

    cfg->framework_type = parseFrameworkType(config.framework_type);
    cfg->accelerate_type = parseAccelerateType(config.accelerate_type);

    // Note: build_interpretper_from_model_and_config takes ownership of model/cfg on success
    auto interpreter = InterpreterBuilder::build_interpretper_from_model_and_config(model, cfg);
    if (!interpreter) {
        // On failure, ownership was not transferred - clean up manually
        delete cfg;
        delete model;
        throw std::runtime_error("AidLiteEngine: failed to build interpreter");
    }
    // model and cfg are now owned by interpreter - do not delete

    if (interpreter->init() != 0) {
        interpreter->destory();
        throw std::runtime_error("AidLiteEngine: interpreter init failed");
    }

    if (interpreter->load_model() != 0) {
        interpreter->destory();
        throw std::runtime_error("AidLiteEngine: load_model failed");
    }

    if (!config.output_shapes.empty()) {
        const auto& first_output = config.output_shapes[0];
        impl_->output_shape.reserve(first_output.size());
        for (int dim : first_output) {
            impl_->output_shape.push_back(static_cast<int64_t>(dim));
        }
    }

    impl_->interpreter = std::move(interpreter);
}

AidLiteEngine::~AidLiteEngine() {
    if (impl_ && impl_->interpreter) {
        impl_->interpreter->destory();
    }
}

void AidLiteEngine::setConfThresh(float thresh) { conf_thresh_.store(thresh); }
void AidLiteEngine::setIouThresh(float thresh) { iou_thresh_.store(thresh); }
float AidLiteEngine::getConfThresh() const { return conf_thresh_.load(); }
float AidLiteEngine::getIouThresh() const { return iou_thresh_.load(); }

cv::Mat AidLiteEngine::preprocess(const cv::Mat& image) {
    if (image.empty()) return {};

    cv::Mat rgb;
    switch (image.channels()) {
        case 3: cv::cvtColor(image, rgb, cv::COLOR_BGR2RGB); break;
        case 4: cv::cvtColor(image, rgb, cv::COLOR_BGRA2RGB); break;
        case 1: cv::cvtColor(image, rgb, cv::COLOR_GRAY2RGB); break;
        default: return {};
    }

    cv::Mat resized;
    cv::resize(rgb, resized, cv::Size(input_w_, input_h_), 0, 0, cv::INTER_LINEAR);

    cv::Mat float_img;
    resized.convertTo(float_img, CV_32F, 1.0 / 255.0);

    std::vector<cv::Mat> chw;
    cv::split(float_img, chw);
    if (chw.size() != 3) return {};

    int sizes[4] = {1, 3, input_h_, input_w_};
    cv::Mat blob(4, sizes, CV_32F);

    const size_t plane_bytes = static_cast<size_t>(input_h_ * input_w_) * sizeof(float);
    for (int c = 0; c < 3; ++c) {
        int idx[4] = {0, c, 0, 0};
        std::memcpy(blob.ptr<float>(idx), chw[c].data, plane_bytes);
    }
    return blob;
}

std::vector<Detection> AidLiteEngine::infer(const cv::Mat& image) {
    if (image.empty() || !impl_->interpreter) return {};

    try {
        const int orig_w = image.cols;
        const int orig_h = image.rows;

        cv::Mat blob = preprocess(image);
        if (blob.empty()) return {};
        if (!blob.isContinuous()) blob = blob.clone();

        int result = impl_->interpreter->set_input_tensor(0, blob.ptr<float>());
        if (result != 0) return {};

        result = impl_->interpreter->invoke();
        if (result != 0) return {};

        float* out_data = nullptr;
        uint32_t out_length = 0;
        result = impl_->interpreter->get_output_tensor(0, reinterpret_cast<void**>(&out_data), &out_length);
        if (result != 0 || !out_data || out_length == 0) return {};

        size_t num_floats = out_length / sizeof(float);
        std::vector<float> output(out_data, out_data + num_floats);

        std::vector<int64_t> output_shape = impl_->output_shape;
        if (output_shape.empty()) {
            int64_t num_classes = static_cast<int64_t>(class_names_.size());
            int64_t channels = num_classes + 4;
            int64_t num_boxes = static_cast<int64_t>(num_floats) / channels;
            output_shape = {1, channels, num_boxes};
        }

        std::vector<Detection> detections = postprocess(output, output_shape, orig_w, orig_h);
        nms(detections);
        return detections;
    } catch (...) {
        return {};
    }
}

std::vector<Detection> AidLiteEngine::postprocess(
    const std::vector<float>& output,
    const std::vector<int64_t>& output_shape,
    int orig_w, int orig_h) {
    std::vector<Detection> detections;
    if (output.empty() || orig_w <= 0 || orig_h <= 0) return detections;

    const float conf_thresh = conf_thresh_.load(std::memory_order_relaxed);
    const int64_t num_classes = static_cast<int64_t>(class_names_.size());
    const int64_t expected_v8_c = num_classes + 4;
    const int64_t expected_v5_c = num_classes + 5;

    int64_t num_boxes = 0, channels = 0;
    bool channels_first = false;

    if (output_shape.size() == 3) {
        const int64_t d1 = output_shape[1], d2 = output_shape[2];
        if (d1 == expected_v8_c || d1 == expected_v5_c) {
            channels_first = true; channels = d1; num_boxes = d2;
        } else if (d2 == expected_v8_c || d2 == expected_v5_c) {
            channels_first = false; num_boxes = d1; channels = d2;
        } else if (d1 > 0 && d2 > 0) {
            channels_first = (d1 < d2);
            channels = channels_first ? d1 : d2;
            num_boxes = channels_first ? d2 : d1;
        }
    } else if (output_shape.size() == 2) {
        num_boxes = output_shape[0]; channels = output_shape[1];
    }

    if (num_boxes <= 0 || channels <= 0) return detections;
    if (static_cast<size_t>(num_boxes * channels) > output.size()) return detections;

    const float scale_x = static_cast<float>(orig_w) / static_cast<float>(input_w_);
    const float scale_y = static_cast<float>(orig_h) / static_cast<float>(input_h_);

    bool has_objectness = (channels == expected_v5_c);
    int64_t cls_offset = has_objectness ? 5 : 4;
    const int64_t out_num_classes = channels - cls_offset;
    if (out_num_classes <= 0) return detections;

    auto get_val = [&](int64_t box_idx, int64_t attr_idx) -> float {
        if (channels_first) return output[static_cast<size_t>(attr_idx * num_boxes + box_idx)];
        return output[static_cast<size_t>(box_idx * channels + attr_idx)];
    };

    for (int64_t i = 0; i < num_boxes; ++i) {
        float x = get_val(i, 0), y = get_val(i, 1);
        float w = get_val(i, 2), h = get_val(i, 3);

        float obj = has_objectness ? get_val(i, 4) : 1.0f;

        int best_class = -1;
        float best_score = 0.0f;
        for (int64_t c = 0; c < out_num_classes; ++c) {
            float s = get_val(i, cls_offset + c);
            if (s > best_score) { best_score = s; best_class = static_cast<int>(c); }
        }

        float score = has_objectness ? (obj * best_score) : best_score;
        if (score < conf_thresh) continue;

        float x1 = (x - w / 2.0f) * scale_x;
        float y1 = (y - h / 2.0f) * scale_y;
        float x2 = (x + w / 2.0f) * scale_x;
        float y2 = (y + h / 2.0f) * scale_y;

        x1 = std::max(0.0f, std::min(x1, static_cast<float>(orig_w - 1)));
        y1 = std::max(0.0f, std::min(y1, static_cast<float>(orig_h - 1)));
        x2 = std::max(0.0f, std::min(x2, static_cast<float>(orig_w - 1)));
        y2 = std::max(0.0f, std::min(y2, static_cast<float>(orig_h - 1)));
        if (x2 <= x1 || y2 <= y1) continue;

        Detection det;
        det.x1 = x1; det.y1 = y1; det.x2 = x2; det.y2 = y2;
        det.score = score; det.class_id = best_class;
        if (best_class >= 0 && best_class < static_cast<int>(class_names_.size())) {
            det.class_name = class_names_[best_class];
        }
        detections.push_back(std::move(det));
    }
    return detections;
}

void AidLiteEngine::nms(std::vector<Detection>& detections) {
    if (detections.empty()) return;

    const float iou_thresh = iou_thresh_.load(std::memory_order_relaxed);

    std::sort(detections.begin(), detections.end(),
              [](const Detection& a, const Detection& b) { return a.score > b.score; });

    auto iou = [](const Detection& a, const Detection& b) -> float {
        const float xx1 = std::max(a.x1, b.x1);
        const float yy1 = std::max(a.y1, b.y1);
        const float xx2 = std::min(a.x2, b.x2);
        const float yy2 = std::min(a.y2, b.y2);
        const float w = std::max(0.0f, xx2 - xx1);
        const float h = std::max(0.0f, yy2 - yy1);
        const float inter = w * h;
        const float area_a = std::max(0.0f, a.x2 - a.x1) * std::max(0.0f, a.y2 - a.y1);
        const float area_b = std::max(0.0f, b.x2 - b.x1) * std::max(0.0f, b.y2 - b.y1);
        const float uni = area_a + area_b - inter;
        return (uni <= 0.0f) ? 0.0f : (inter / uni);
    };

    std::vector<Detection> kept;
    kept.reserve(detections.size());

    for (const auto& det : detections) {
        bool keep = true;
        for (const auto& prev : kept) {
            if (det.class_id != prev.class_id) continue;
            if (iou(det, prev) > iou_thresh) { keep = false; break; }
        }
        if (keep) kept.push_back(det);
    }
    detections.swap(kept);
}

}  // namespace rc26_vision
