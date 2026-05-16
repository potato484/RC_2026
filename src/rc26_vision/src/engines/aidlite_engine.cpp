#include "rc26_vision/engines/aidlite_engine.hpp"

#include <aidlux/aidlite/aidlite.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <utility>

using namespace Aidlux::Aidlite;

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

int parsePaddingValue(const std::string& value) {
    std::string lower = value;
    std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    if (lower.empty() || lower == "black") return 0;
    if (lower == "white") return 255;
    if (lower == "gray" || lower == "grey" || lower == "gray114" || lower == "grey114") return 114;
    try {
        return std::clamp(std::stoi(lower), 0, 255);
    } catch (...) {
        return 114;
    }
}

std::size_t bytesPerElement(DataType dtype) {
    switch (dtype) {
        case DataType::TYPE_UINT8:
        case DataType::TYPE_INT8:
            return sizeof(uint8_t);
        case DataType::TYPE_UINT16:
        case DataType::TYPE_INT16:
        case DataType::TYPE_FLOAT16:
            return sizeof(uint16_t);
        default:
            return sizeof(float);
    }
}

}  // namespace

struct AidLiteEngine::Impl {
    std::unique_ptr<Interpreter> interpreter;
    AidLiteConfig config;
    DataType input_dtype{DataType::TYPE_FLOAT32};
    DataType output_dtype{DataType::TYPE_FLOAT32};
    std::string input_name;
    std::string output_name;
    bool input_channel_first{true};
    std::size_t output_channels{0};
    std::size_t output_predictions{0};
    bool output_channel_major{true};
    int padding_value{114};

    int last_src_w{0};
    int last_src_h{0};
    float last_scale_x{1.0F};
    float last_scale_y{1.0F};
    int last_pad_x{0};
    int last_pad_y{0};
    bool last_letterbox{false};

    std::vector<uint8_t> input_u8;
    std::vector<int8_t> input_i8;
    std::vector<uint16_t> input_u16;
    std::vector<int16_t> input_i16;
    std::vector<float> input_f32;
    std::vector<float> output_f32;
    std::vector<uint8_t> input_u8_lut;
    std::vector<int8_t> input_i8_lut;
    std::vector<uint16_t> input_u16_lut;
    std::vector<int16_t> input_i16_lut;
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
    impl_->input_name = config.input_name;
    impl_->output_name = config.output_name;
    impl_->padding_value = parsePaddingValue(config.padding_color);

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
    cfg->qnn_shared_buffer = 0;

    auto interpreter = InterpreterBuilder::build_interpretper_from_model_and_config(model, cfg);
    if (!interpreter) {
        delete cfg;
        delete model;
        throw std::runtime_error("AidLiteEngine: failed to build interpreter");
    }

    if (interpreter->init() != 0) {
        interpreter->destory();
        throw std::runtime_error("AidLiteEngine: interpreter init failed");
    }

    if (interpreter->load_model() != 0) {
        interpreter->destory();
        throw std::runtime_error("AidLiteEngine: load_model failed");
    }

    std::vector<std::vector<TensorInfo>> input_infos;
    if (interpreter->get_input_tensor_info(input_infos) == 0 &&
        !input_infos.empty() && !input_infos[0].empty()) {
        const TensorInfo& in = input_infos[0][0];
        impl_->input_dtype = in.element_type;
        if (impl_->input_name.empty()) {
            impl_->input_name = in.name;
        }
        if (in.shape.size() >= 4U) {
            const std::size_t dim1 = static_cast<std::size_t>(in.shape[1]);
            const std::size_t dim3 = static_cast<std::size_t>(in.shape[3]);
            impl_->input_channel_first = dim1 > 0U && dim1 <= 4U && dim3 > 4U;
            const std::size_t tensor_h = impl_->input_channel_first ?
                static_cast<std::size_t>(in.shape[2]) : static_cast<std::size_t>(in.shape[1]);
            const std::size_t tensor_w = impl_->input_channel_first ?
                static_cast<std::size_t>(in.shape[3]) : static_cast<std::size_t>(in.shape[2]);
            if (tensor_w > 0U && tensor_h > 0U) {
                input_w_ = static_cast<int>(tensor_w);
                input_h_ = static_cast<int>(tensor_h);
            }
        }
    }

    std::vector<std::vector<TensorInfo>> output_infos;
    if (interpreter->get_output_tensor_info(output_infos) == 0 &&
        !output_infos.empty() && !output_infos[0].empty()) {
        const TensorInfo& out = output_infos[0][0];
        impl_->output_dtype = out.element_type;
        if (impl_->output_name.empty()) {
            impl_->output_name = out.name;
        }
        if (out.shape.size() >= 3U) {
            const std::size_t dim1 = static_cast<std::size_t>(out.shape[1]);
            const std::size_t dim2 = static_cast<std::size_t>(out.shape[2]);
            if (dim1 <= 512U && dim2 > dim1) {
                impl_->output_channels = dim1;
                impl_->output_predictions = dim2;
                impl_->output_channel_major = true;
            } else {
                impl_->output_channels = dim2;
                impl_->output_predictions = dim1;
                impl_->output_channel_major = false;
            }
        }
    }

    if (impl_->output_channels == 0U || impl_->output_predictions == 0U) {
        const std::size_t channels =
            static_cast<std::size_t>((config.num_classes > 0) ? config.num_classes :
                                     static_cast<int>(class_names_.size())) + 4U;
        impl_->output_channels = channels;
        impl_->output_predictions = 8400U;
        impl_->output_channel_major = true;
    }

    const std::size_t input_elements = static_cast<std::size_t>(input_w_ * input_h_ * 3);
    impl_->input_u8.resize(input_elements);
    impl_->input_i8.resize(input_elements);
    impl_->input_u16.resize(input_elements);
    impl_->input_i16.resize(input_elements);
    impl_->input_f32.resize(input_elements);
    impl_->output_f32.resize(impl_->output_channels * impl_->output_predictions);

    impl_->input_u8_lut.resize(256U);
    impl_->input_i8_lut.resize(256U);
    impl_->input_u16_lut.resize(256U);
    impl_->input_i16_lut.resize(256U);
    for (int i = 0; i < 256; ++i) {
        const float real = static_cast<float>(i) / 255.0F;
        const int32_t q = static_cast<int32_t>(
            std::lround(real / static_cast<float>(config.input_scale) -
                        static_cast<float>(config.input_offset)));
        impl_->input_u8_lut[static_cast<std::size_t>(i)] =
            static_cast<uint8_t>(std::clamp(q, 0, 255));
        impl_->input_i8_lut[static_cast<std::size_t>(i)] =
            static_cast<int8_t>(std::clamp(q, -128, 127));
        impl_->input_u16_lut[static_cast<std::size_t>(i)] =
            static_cast<uint16_t>(std::clamp(q, 0, 65535));
        impl_->input_i16_lut[static_cast<std::size_t>(i)] =
            static_cast<int16_t>(std::clamp(q, -32768, 32767));
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

bool AidLiteEngine::preprocess(const cv::Mat& image) {
    if (image.empty()) return false;

    cv::Mat rgb;
    switch (image.channels()) {
        case 3: cv::cvtColor(image, rgb, cv::COLOR_BGR2RGB); break;
        case 4: cv::cvtColor(image, rgb, cv::COLOR_BGRA2RGB); break;
        case 1: cv::cvtColor(image, rgb, cv::COLOR_GRAY2RGB); break;
        default: return false;
    }

    impl_->last_src_w = image.cols;
    impl_->last_src_h = image.rows;
    impl_->last_pad_x = 0;
    impl_->last_pad_y = 0;
    impl_->last_letterbox = (impl_->config.resize_mode != "stretch");

    cv::Mat model_rgb;
    if (impl_->last_letterbox) {
        const float sx = static_cast<float>(input_w_) / static_cast<float>(image.cols);
        const float sy = static_cast<float>(input_h_) / static_cast<float>(image.rows);
        const float scale = std::min(sx, sy);
        const int new_w = std::max(1, static_cast<int>(std::round(image.cols * scale)));
        const int new_h = std::max(1, static_cast<int>(std::round(image.rows * scale)));
        impl_->last_scale_x = scale;
        impl_->last_scale_y = scale;
        impl_->last_pad_x = (input_w_ - new_w) / 2;
        impl_->last_pad_y = (input_h_ - new_h) / 2;
        model_rgb = cv::Mat(
            input_h_, input_w_, CV_8UC3,
            cv::Scalar(impl_->padding_value, impl_->padding_value, impl_->padding_value));
        cv::Mat resized;
        cv::resize(rgb, resized, cv::Size(new_w, new_h), 0.0, 0.0, cv::INTER_LINEAR);
        resized.copyTo(model_rgb(cv::Rect(impl_->last_pad_x, impl_->last_pad_y, new_w, new_h)));
    } else {
        impl_->last_scale_x = static_cast<float>(input_w_) / static_cast<float>(image.cols);
        impl_->last_scale_y = static_cast<float>(input_h_) / static_cast<float>(image.rows);
        cv::resize(rgb, model_rgb, cv::Size(input_w_, input_h_), 0.0, 0.0, cv::INTER_LINEAR);
    }

    if (!model_rgb.isContinuous()) {
        model_rgb = model_rgb.clone();
    }

    const std::size_t total = static_cast<std::size_t>(model_rgb.total() * model_rgb.channels());
    const uint8_t* ptr = model_rgb.ptr<uint8_t>(0);

    switch (impl_->input_dtype) {
        case DataType::TYPE_UINT8:
            for (std::size_t i = 0; i < total; ++i) impl_->input_u8[i] = impl_->input_u8_lut[ptr[i]];
            return true;
        case DataType::TYPE_INT8:
            for (std::size_t i = 0; i < total; ++i) impl_->input_i8[i] = impl_->input_i8_lut[ptr[i]];
            return true;
        case DataType::TYPE_UINT16:
            for (std::size_t i = 0; i < total; ++i) impl_->input_u16[i] = impl_->input_u16_lut[ptr[i]];
            return true;
        case DataType::TYPE_INT16:
            for (std::size_t i = 0; i < total; ++i) impl_->input_i16[i] = impl_->input_i16_lut[ptr[i]];
            return true;
        default:
            break;
    }

    if (!impl_->input_channel_first) {
        for (std::size_t i = 0; i < total; ++i) {
            impl_->input_f32[i] = static_cast<float>(ptr[i]) / 255.0F;
        }
        return true;
    }

    const std::size_t plane = static_cast<std::size_t>(input_w_ * input_h_);
    if (total != plane * 3U) return false;
    for (std::size_t y = 0; y < static_cast<std::size_t>(input_h_); ++y) {
        for (std::size_t x = 0; x < static_cast<std::size_t>(input_w_); ++x) {
            const std::size_t hwc_index = (y * static_cast<std::size_t>(input_w_) + x) * 3U;
            const std::size_t pixel_index = y * static_cast<std::size_t>(input_w_) + x;
            impl_->input_f32[0U * plane + pixel_index] =
                static_cast<float>(ptr[hwc_index + 0U]) / 255.0F;
            impl_->input_f32[1U * plane + pixel_index] =
                static_cast<float>(ptr[hwc_index + 1U]) / 255.0F;
            impl_->input_f32[2U * plane + pixel_index] =
                static_cast<float>(ptr[hwc_index + 2U]) / 255.0F;
        }
    }
    return true;
}

std::vector<Detection> AidLiteEngine::infer(const cv::Mat& image) {
    if (image.empty() || !impl_->interpreter) return {};

    try {
        if (!preprocess(image)) return {};

        void* input_ptr = impl_->input_f32.data();
        switch (impl_->input_dtype) {
            case DataType::TYPE_UINT8: input_ptr = impl_->input_u8.data(); break;
            case DataType::TYPE_INT8: input_ptr = impl_->input_i8.data(); break;
            case DataType::TYPE_UINT16: input_ptr = impl_->input_u16.data(); break;
            case DataType::TYPE_INT16: input_ptr = impl_->input_i16.data(); break;
            default: break;
        }

        int result = -1;
        if (!impl_->input_name.empty()) {
            result = impl_->interpreter->set_input_tensor(impl_->input_name, input_ptr);
        }
        if (result != 0) {
            result = impl_->interpreter->set_input_tensor(0, input_ptr);
        }
        if (result != 0) return {};

        result = impl_->interpreter->invoke();
        if (result != 0) return {};

        void* raw_output = nullptr;
        uint32_t output_length = 0;
        if (!impl_->output_name.empty()) {
            result = impl_->interpreter->get_output_tensor(impl_->output_name, &raw_output, &output_length);
        } else {
            result = -1;
        }
        if (result != 0) {
            result = impl_->interpreter->get_output_tensor(0, &raw_output, &output_length);
        }
        if (result != 0 || !raw_output) return {};

        const std::size_t expected_elements = impl_->output_channels * impl_->output_predictions;
        const std::size_t bpe = bytesPerElement(impl_->output_dtype);
        std::size_t available_elements = expected_elements;
        if (output_length != 0U) {
            const std::size_t len = static_cast<std::size_t>(output_length);
            if (len == expected_elements || len == expected_elements * bpe) {
                available_elements = expected_elements;
            } else if (len % bpe == 0U) {
                available_elements = len / bpe;
            } else {
                available_elements = len;
            }
        }
        if (available_elements < expected_elements) return {};

        impl_->output_f32.resize(expected_elements);
        auto dequantize = [&](float raw, std::size_t flat_index) -> float {
            double scale = impl_->config.output_scale;
            double offset = impl_->config.output_offset;
            if (impl_->config.split_output_quantization && impl_->output_channels >= 4U) {
                const std::size_t channel = impl_->output_channel_major ?
                    (flat_index / impl_->output_predictions) : (flat_index % impl_->output_channels);
                if (channel < 4U) {
                    scale = impl_->config.bbox_output_scale;
                    offset = impl_->config.bbox_output_offset;
                } else {
                    scale = impl_->config.score_output_scale;
                    offset = impl_->config.score_output_offset;
                }
            }
            return static_cast<float>((raw + static_cast<float>(offset)) * static_cast<float>(scale));
        };

        switch (impl_->output_dtype) {
            case DataType::TYPE_UINT8: {
                const auto* out = static_cast<const uint8_t*>(raw_output);
                for (std::size_t i = 0; i < expected_elements; ++i) {
                    impl_->output_f32[i] = dequantize(static_cast<float>(out[i]), i);
                }
                break;
            }
            case DataType::TYPE_INT8: {
                const auto* out = static_cast<const int8_t*>(raw_output);
                for (std::size_t i = 0; i < expected_elements; ++i) {
                    impl_->output_f32[i] = dequantize(static_cast<float>(out[i]), i);
                }
                break;
            }
            case DataType::TYPE_UINT16: {
                const auto* out = static_cast<const uint16_t*>(raw_output);
                for (std::size_t i = 0; i < expected_elements; ++i) {
                    impl_->output_f32[i] = dequantize(static_cast<float>(out[i]), i);
                }
                break;
            }
            case DataType::TYPE_INT16:
            case DataType::TYPE_FLOAT16: {
                const auto* out = static_cast<const int16_t*>(raw_output);
                for (std::size_t i = 0; i < expected_elements; ++i) {
                    impl_->output_f32[i] = dequantize(static_cast<float>(out[i]), i);
                }
                break;
            }
            default: {
                const auto* out = static_cast<const float*>(raw_output);
                std::copy(out, out + expected_elements, impl_->output_f32.begin());
                break;
            }
        }

        std::vector<Detection> detections = postprocess(impl_->output_f32, image.cols, image.rows);
        nms(detections);
        return detections;
    } catch (...) {
        return {};
    }
}

std::vector<Detection> AidLiteEngine::postprocess(
    const std::vector<float>& output,
    int orig_w, int orig_h) {
    std::vector<Detection> detections;
    if (output.empty() || orig_w <= 0 || orig_h <= 0) return detections;

    const float conf_thresh = conf_thresh_.load(std::memory_order_relaxed);
    const int64_t channels = static_cast<int64_t>(impl_->output_channels);
    const int64_t num_boxes = static_cast<int64_t>(impl_->output_predictions);
    const int64_t num_classes = std::max<int64_t>(
        1, (impl_->config.num_classes > 0) ? impl_->config.num_classes :
        static_cast<int64_t>(class_names_.size()));
    const int64_t expected_v8_c = num_classes + 4;
    const int64_t expected_v5_c = num_classes + 5;
    const bool has_objectness = (channels == expected_v5_c);
    const int64_t cls_offset = has_objectness ? 5 : 4;
    const int64_t out_num_classes = channels - cls_offset;
    if (num_boxes <= 0 || channels <= cls_offset || out_num_classes <= 0) return detections;

    auto get_val = [&](int64_t box_idx, int64_t attr_idx) -> float {
        if (impl_->output_channel_major) {
            return output[static_cast<std::size_t>(attr_idx * num_boxes + box_idx)];
        }
        return output[static_cast<std::size_t>(box_idx * channels + attr_idx)];
    };

    auto map_x = [&](float x) -> float {
        if (impl_->last_letterbox) {
            return (x - static_cast<float>(impl_->last_pad_x)) / impl_->last_scale_x;
        }
        return x / impl_->last_scale_x;
    };
    auto map_y = [&](float y) -> float {
        if (impl_->last_letterbox) {
            return (y - static_cast<float>(impl_->last_pad_y)) / impl_->last_scale_y;
        }
        return y / impl_->last_scale_y;
    };

    detections.reserve(static_cast<std::size_t>(num_boxes / 8));
    for (int64_t i = 0; i < num_boxes; ++i) {
        const float x = get_val(i, 0);
        const float y = get_val(i, 1);
        const float w = get_val(i, 2);
        const float h = get_val(i, 3);
        if (w <= 0.0F || h <= 0.0F) continue;

        const float obj = has_objectness ? get_val(i, 4) : 1.0F;
        int best_class = -1;
        float best_score = 0.0F;
        const int64_t class_limit = std::min<int64_t>(out_num_classes, num_classes);
        for (int64_t c = 0; c < class_limit; ++c) {
            const float s = get_val(i, cls_offset + c);
            if (s > best_score) {
                best_score = s;
                best_class = static_cast<int>(c);
            }
        }

        const float score = has_objectness ? (obj * best_score) : best_score;
        if (score < conf_thresh) continue;

        float x1 = map_x(x - w / 2.0F);
        float y1 = map_y(y - h / 2.0F);
        float x2 = map_x(x + w / 2.0F);
        float y2 = map_y(y + h / 2.0F);

        x1 = std::clamp(x1, 0.0F, static_cast<float>(orig_w - 1));
        y1 = std::clamp(y1, 0.0F, static_cast<float>(orig_h - 1));
        x2 = std::clamp(x2, 0.0F, static_cast<float>(orig_w - 1));
        y2 = std::clamp(y2, 0.0F, static_cast<float>(orig_h - 1));
        if (x2 <= x1 || y2 <= y1) continue;

        Detection det;
        det.x1 = x1;
        det.y1 = y1;
        det.x2 = x2;
        det.y2 = y2;
        det.score = score;
        det.class_id = best_class;
        if (best_class >= 0 && best_class < static_cast<int>(class_names_.size())) {
            det.class_name = class_names_[static_cast<std::size_t>(best_class)];
        }
        detections.push_back(std::move(det));
    }
    (void)expected_v8_c;
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
            if (iou(det, prev) > iou_thresh) {
                keep = false;
                break;
            }
        }
        if (keep) kept.push_back(det);
    }
    detections.swap(kept);
}

}  // namespace rc26_vision
