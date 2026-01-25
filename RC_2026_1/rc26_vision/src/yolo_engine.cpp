#include "rc26_vision/yolo_engine.hpp"

#include <onnxruntime_cxx_api.h>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <cstring>
#include <utility>

namespace rc26_vision {

struct YoloEngine::Impl {
    Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "YoloEngine"};
    Ort::SessionOptions session_options;
    std::unique_ptr<Ort::Session> session;
    Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(
        OrtArenaAllocator, OrtMemTypeDefault);

    std::vector<std::string> input_names;
    std::vector<std::string> output_names;
    std::vector<const char*> input_name_ptrs;
    std::vector<const char*> output_name_ptrs;
};

YoloEngine::YoloEngine(const std::string& model_path,
                       const std::vector<std::string>& class_names,
                       float conf_thresh,
                       float iou_thresh)
    : impl_(std::make_unique<Impl>()),
      class_names_(class_names),
      conf_thresh_(conf_thresh),
      iou_thresh_(iou_thresh) {
    impl_->session_options.SetIntraOpNumThreads(2);
    impl_->session_options.SetInterOpNumThreads(1);
    impl_->session = std::make_unique<Ort::Session>(
        impl_->env, model_path.c_str(), impl_->session_options);

    Ort::AllocatorWithDefaultOptions allocator;

    const size_t num_inputs = impl_->session->GetInputCount();
    impl_->input_names.reserve(num_inputs);
    for (size_t i = 0; i < num_inputs; ++i) {
        auto name_ptr = impl_->session->GetInputNameAllocated(i, allocator);
        impl_->input_names.emplace_back(name_ptr ? name_ptr.get() : "");
    }
    for (const auto& name : impl_->input_names) {
        impl_->input_name_ptrs.push_back(name.c_str());
    }

    const size_t num_outputs = impl_->session->GetOutputCount();
    impl_->output_names.reserve(num_outputs);
    for (size_t i = 0; i < num_outputs; ++i) {
        auto name_ptr = impl_->session->GetOutputNameAllocated(i, allocator);
        impl_->output_names.emplace_back(name_ptr ? name_ptr.get() : "");
    }
    for (const auto& name : impl_->output_names) {
        impl_->output_name_ptrs.push_back(name.c_str());
    }
}

YoloEngine::~YoloEngine() = default;

void YoloEngine::setConfThresh(float thresh) { conf_thresh_.store(thresh); }
void YoloEngine::setIouThresh(float thresh) { iou_thresh_.store(thresh); }
float YoloEngine::getConfThresh() const { return conf_thresh_.load(); }
float YoloEngine::getIouThresh() const { return iou_thresh_.load(); }

cv::Mat YoloEngine::preprocess(const cv::Mat& image) {
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

std::vector<Detection> YoloEngine::infer(const cv::Mat& image) {
    if (image.empty()) return {};

    try {
        const int orig_w = image.cols;
        const int orig_h = image.rows;

        cv::Mat blob = preprocess(image);
        if (blob.empty()) return {};
        if (!blob.isContinuous()) blob = blob.clone();

        const std::array<int64_t, 4> input_shape{1, 3, input_h_, input_w_};
        const size_t input_tensor_size = static_cast<size_t>(blob.total());

        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
            impl_->memory_info, blob.ptr<float>(), input_tensor_size,
            input_shape.data(), input_shape.size());

        if (impl_->input_name_ptrs.empty() || impl_->output_name_ptrs.empty()) {
            return {};
        }

        std::vector<Ort::Value> output_tensors = impl_->session->Run(
            Ort::RunOptions{nullptr},
            impl_->input_name_ptrs.data(), &input_tensor, 1,
            impl_->output_name_ptrs.data(), impl_->output_name_ptrs.size());

        if (output_tensors.empty() || !output_tensors[0].IsTensor()) return {};

        auto shape_info = output_tensors[0].GetTensorTypeAndShapeInfo();
        std::vector<int64_t> output_shape = shape_info.GetShape();
        const size_t output_count = shape_info.GetElementCount();
        const float* output_data = output_tensors[0].GetTensorData<float>();
        std::vector<float> output(output_data, output_data + output_count);

        std::vector<Detection> detections = postprocess(output, output_shape, orig_w, orig_h);
        nms(detections);
        return detections;
    } catch (const Ort::Exception&) {
        return {};
    } catch (const std::exception&) {
        return {};
    }
}

std::vector<Detection> YoloEngine::postprocess(
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

void YoloEngine::nms(std::vector<Detection>& detections) {
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
