#include "rc26_vision/inference/onnx_runtime_engine.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <utility>
#include <vector>

#include <onnxruntime/onnxruntime_cxx_api.h>

#include "rc26_vision/postprocess/yolo_detection_postprocessor.hpp"
#include "rc26_vision/preprocess/yolo_image_preprocessor.hpp"

namespace rc26_vision {

namespace {

Ort::Env& runtimeEnv() {
    static Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "rc26_vision");
    return env;
}

std::size_t bytesPerElement(ONNXTensorElementDataType dtype) {
    switch (dtype) {
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT8:
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT8:
            return sizeof(uint8_t);
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT16:
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT16:
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT16:
            return sizeof(uint16_t);
        default:
            return sizeof(float);
    }
}

struct OutputTensorLayout {
    std::size_t channels{0U};
    std::size_t predictions{0U};
    bool channel_major{true};
};

OutputTensorLayout inferOutputTensorLayout(const std::vector<int64_t>& shape) {
    std::vector<int64_t> dims;
    dims.reserve(shape.size());
    for (const int64_t dim : shape) {
        if (dim > 0) {
            dims.push_back(dim);
        }
    }

    while (!dims.empty() && dims.front() == 1 && dims.size() > 2U) {
        dims.erase(dims.begin());
    }
    while (!dims.empty() && dims.back() == 1 && dims.size() > 2U) {
        dims.pop_back();
    }
    if (dims.size() < 2U) {
        throw std::runtime_error("ONNX Runtime 输出张量维度异常，无法解析 YOLO 输出。");
    }

    const std::size_t dim_a = static_cast<std::size_t>(dims[dims.size() - 2]);
    const std::size_t dim_b = static_cast<std::size_t>(dims[dims.size() - 1]);

    OutputTensorLayout layout;
    if (dim_a <= 512U && dim_b > dim_a) {
        layout.channels = dim_a;
        layout.predictions = dim_b;
        layout.channel_major = true;
    } else {
        layout.channels = dim_b;
        layout.predictions = dim_a;
        layout.channel_major = false;
    }
    return layout;
}

}  // namespace

struct OpenCvOnnxEngine::Impl {
    Ort::Session session{nullptr};
    AidLiteConfig config;
    ONNXTensorElementDataType input_dtype{ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT};
    ONNXTensorElementDataType output_dtype{ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT};
    std::string input_name;
    std::string output_name;
    bool input_channel_first{true};
    std::size_t output_channels{0U};
    std::size_t output_predictions{0U};
    bool output_channel_major{true};
    int padding_value{114};

    std::vector<int64_t> input_shape;
    std::vector<int64_t> output_shape;
    std::vector<float> input_f32;
    std::vector<uint8_t> input_u8;
    std::vector<int8_t> input_i8;
    std::vector<uint16_t> input_u16;
    std::vector<int16_t> input_i16;
    std::vector<float> output_f32;
    std::vector<uint8_t> input_u8_lut;
    std::vector<int8_t> input_i8_lut;
    std::vector<uint16_t> input_u16_lut;
    std::vector<int16_t> input_i16_lut;
};

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
    impl_->config = config;
    impl_->padding_value = parsePaddingValue(config.padding_color);

    Ort::SessionOptions session_options;
    session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    session_options.SetIntraOpNumThreads(1);
    session_options.SetInterOpNumThreads(1);
    session_options.DisableCpuMemArena();
    impl_->session = Ort::Session(runtimeEnv(), model_path.c_str(), session_options);

    Ort::AllocatorWithDefaultOptions allocator;
    auto input_name = impl_->session.GetInputNameAllocated(0, allocator);
    auto output_name = impl_->session.GetOutputNameAllocated(0, allocator);
    impl_->input_name = input_name.get();
    impl_->output_name = config.output_name.empty() ? output_name.get() : config.output_name;

    Ort::TypeInfo input_info = impl_->session.GetInputTypeInfo(0);
    auto input_type_info = input_info.GetTensorTypeAndShapeInfo();
    impl_->input_dtype = input_type_info.GetElementType();
    impl_->input_shape = input_type_info.GetShape();
    if (impl_->input_shape.size() >= 4U) {
        const int64_t dim1_raw = impl_->input_shape[1];
        const int64_t dim3_raw = impl_->input_shape[3];
        if (dim1_raw > 0 && dim1_raw <= 4 && (dim3_raw <= 0 || dim3_raw > 4)) {
            impl_->input_channel_first = true;
        } else if (dim3_raw > 0 && dim3_raw <= 4 && (dim1_raw <= 0 || dim1_raw > 4)) {
            impl_->input_channel_first = false;
        } else {
            impl_->input_channel_first = true;
        }
        const std::size_t tensor_h = impl_->input_channel_first
                                         ? static_cast<std::size_t>(std::max<int64_t>(impl_->input_shape[2], input_h_))
                                         : static_cast<std::size_t>(std::max<int64_t>(impl_->input_shape[1], input_h_));
        const std::size_t tensor_w = impl_->input_channel_first
                                         ? static_cast<std::size_t>(std::max<int64_t>(impl_->input_shape[3], input_w_))
                                         : static_cast<std::size_t>(std::max<int64_t>(impl_->input_shape[2], input_w_));
        if (tensor_w > 0U && tensor_h > 0U) {
            input_w_ = static_cast<int>(tensor_w);
            input_h_ = static_cast<int>(tensor_h);
        }
        impl_->input_shape[0] = (impl_->input_shape[0] > 0) ? impl_->input_shape[0] : 1;
        if (impl_->input_channel_first) {
            impl_->input_shape[1] = (impl_->input_shape[1] > 0) ? impl_->input_shape[1] : 3;
            impl_->input_shape[2] = input_h_;
            impl_->input_shape[3] = input_w_;
        } else {
            impl_->input_shape[1] = input_h_;
            impl_->input_shape[2] = input_w_;
            impl_->input_shape[3] = (impl_->input_shape[3] > 0) ? impl_->input_shape[3] : 3;
        }
    }

    Ort::TypeInfo output_info = impl_->session.GetOutputTypeInfo(0);
    auto output_type_info = output_info.GetTensorTypeAndShapeInfo();
    impl_->output_dtype = output_type_info.GetElementType();
    impl_->output_shape = output_type_info.GetShape();
    try {
        const OutputTensorLayout layout = inferOutputTensorLayout(impl_->output_shape);
        impl_->output_channels = layout.channels;
        impl_->output_predictions = layout.predictions;
        impl_->output_channel_major = layout.channel_major;
    } catch (...) {
        impl_->output_channels = static_cast<std::size_t>(
            std::max(1, (config.num_classes > 0) ? config.num_classes
                                                 : static_cast<int>(class_names_.size())) +
            4);
        impl_->output_predictions = 8400U;
        impl_->output_channel_major = true;
    }

    const std::size_t input_elements = static_cast<std::size_t>(input_w_ * input_h_ * 3);
    impl_->input_f32.resize(input_elements);
    impl_->input_u8.resize(input_elements);
    impl_->input_i8.resize(input_elements);
    impl_->input_u16.resize(input_elements);
    impl_->input_i16.resize(input_elements);
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
}

OpenCvOnnxEngine::~OpenCvOnnxEngine() = default;

std::vector<Detection> OpenCvOnnxEngine::infer(const cv::Mat& image) {
    if (image.empty() || !impl_) {
        return {};
    }

    try {
        cv::Mat model_rgb;
        YoloImageTransform transform;
        if (!prepareYoloInputImage(image,
                                   input_w_,
                                   input_h_,
                                   impl_->config.resize_mode,
                                   impl_->padding_value,
                                   model_rgb,
                                   transform)) {
            return {};
        }

        const std::size_t total = static_cast<std::size_t>(model_rgb.total() * model_rgb.channels());
        const uint8_t* ptr = model_rgb.ptr<uint8_t>(0);

        if (impl_->input_dtype == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
            if (!impl_->input_channel_first) {
                for (std::size_t i = 0; i < total; ++i) {
                    impl_->input_f32[i] = static_cast<float>(ptr[i]) / 255.0F;
                }
            } else {
                const std::size_t plane = static_cast<std::size_t>(input_w_ * input_h_);
                for (std::size_t y = 0; y < static_cast<std::size_t>(input_h_); ++y) {
                    for (std::size_t x = 0; x < static_cast<std::size_t>(input_w_); ++x) {
                        const std::size_t hwc_index =
                            (y * static_cast<std::size_t>(input_w_) + x) * 3U;
                        const std::size_t pixel_index =
                            y * static_cast<std::size_t>(input_w_) + x;
                        impl_->input_f32[0U * plane + pixel_index] =
                            static_cast<float>(ptr[hwc_index + 0U]) / 255.0F;
                        impl_->input_f32[1U * plane + pixel_index] =
                            static_cast<float>(ptr[hwc_index + 1U]) / 255.0F;
                        impl_->input_f32[2U * plane + pixel_index] =
                            static_cast<float>(ptr[hwc_index + 2U]) / 255.0F;
                    }
                }
            }
        } else if (impl_->input_dtype == ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT8) {
            for (std::size_t i = 0; i < total; ++i) impl_->input_u8[i] = impl_->input_u8_lut[ptr[i]];
        } else if (impl_->input_dtype == ONNX_TENSOR_ELEMENT_DATA_TYPE_INT8) {
            for (std::size_t i = 0; i < total; ++i) impl_->input_i8[i] = impl_->input_i8_lut[ptr[i]];
        } else if (impl_->input_dtype == ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT16) {
            for (std::size_t i = 0; i < total; ++i) impl_->input_u16[i] = impl_->input_u16_lut[ptr[i]];
        } else if (impl_->input_dtype == ONNX_TENSOR_ELEMENT_DATA_TYPE_INT16) {
            for (std::size_t i = 0; i < total; ++i) impl_->input_i16[i] = impl_->input_i16_lut[ptr[i]];
        } else {
            throw std::runtime_error("ONNX Runtime 当前不支持该输入张量类型。");
        }

        Ort::MemoryInfo mem_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        Ort::Value input_tensor{nullptr};
        if (impl_->input_dtype == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
            input_tensor = Ort::Value::CreateTensor<float>(
                mem_info,
                impl_->input_f32.data(),
                impl_->input_f32.size(),
                impl_->input_shape.data(),
                impl_->input_shape.size());
        } else if (impl_->input_dtype == ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT8) {
            input_tensor = Ort::Value::CreateTensor<uint8_t>(
                mem_info,
                impl_->input_u8.data(),
                impl_->input_u8.size(),
                impl_->input_shape.data(),
                impl_->input_shape.size());
        } else if (impl_->input_dtype == ONNX_TENSOR_ELEMENT_DATA_TYPE_INT8) {
            input_tensor = Ort::Value::CreateTensor<int8_t>(
                mem_info,
                impl_->input_i8.data(),
                impl_->input_i8.size(),
                impl_->input_shape.data(),
                impl_->input_shape.size());
        } else if (impl_->input_dtype == ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT16) {
            input_tensor = Ort::Value::CreateTensor<uint16_t>(
                mem_info,
                impl_->input_u16.data(),
                impl_->input_u16.size(),
                impl_->input_shape.data(),
                impl_->input_shape.size());
        } else {
            input_tensor = Ort::Value::CreateTensor<int16_t>(
                mem_info,
                impl_->input_i16.data(),
                impl_->input_i16.size(),
                impl_->input_shape.data(),
                impl_->input_shape.size());
        }

        const std::array<const char*, 1> input_names = {impl_->input_name.c_str()};
        const std::array<const char*, 1> output_names = {impl_->output_name.c_str()};
        auto outputs = impl_->session.Run(Ort::RunOptions{nullptr},
                                          input_names.data(),
                                          &input_tensor,
                                          input_names.size(),
                                          output_names.data(),
                                          output_names.size());
        if (outputs.empty()) {
            return {};
        }

        Ort::Value& output_tensor = outputs.front();
        auto output_info = output_tensor.GetTensorTypeAndShapeInfo();
        const std::size_t element_count = output_info.GetElementCount();
        const std::size_t expected_elements = impl_->output_channels * impl_->output_predictions;
        if (element_count < expected_elements) {
            return {};
        }

        impl_->output_f32.resize(expected_elements);
        auto dequantize = [&](float raw, std::size_t flat_index) -> float {
            double scale = impl_->config.output_scale;
            double offset = impl_->config.output_offset;
            if (impl_->config.split_output_quantization && impl_->output_channels >= 4U) {
                const std::size_t channel = impl_->output_channel_major
                                                ? (flat_index / impl_->output_predictions)
                                                : (flat_index % impl_->output_channels);
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
            case ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT8: {
                const auto* data = output_tensor.GetTensorData<uint8_t>();
                for (std::size_t i = 0; i < expected_elements; ++i) {
                    impl_->output_f32[i] = dequantize(static_cast<float>(data[i]), i);
                }
                break;
            }
            case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT8: {
                const auto* data = output_tensor.GetTensorData<int8_t>();
                for (std::size_t i = 0; i < expected_elements; ++i) {
                    impl_->output_f32[i] = dequantize(static_cast<float>(data[i]), i);
                }
                break;
            }
            case ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT16: {
                const auto* data = output_tensor.GetTensorData<uint16_t>();
                for (std::size_t i = 0; i < expected_elements; ++i) {
                    impl_->output_f32[i] = dequantize(static_cast<float>(data[i]), i);
                }
                break;
            }
            case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT16:
            case ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT16: {
                const auto* data = output_tensor.GetTensorData<int16_t>();
                for (std::size_t i = 0; i < expected_elements; ++i) {
                    impl_->output_f32[i] = dequantize(static_cast<float>(data[i]), i);
                }
                break;
            }
            case ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT: {
                const auto* data = output_tensor.GetTensorData<float>();
                std::memcpy(impl_->output_f32.data(), data, expected_elements * sizeof(float));
                break;
            }
            default:
                throw std::runtime_error("ONNX Runtime 当前不支持该输出张量类型。");
        }

        std::vector<Detection> detections = decodeYoloOutput(impl_->output_f32,
                                                             impl_->output_channels,
                                                             impl_->output_predictions,
                                                             impl_->output_channel_major,
                                                             class_names_,
                                                             conf_thresh_.load(),
                                                             image.cols,
                                                             image.rows,
                                                             transform,
                                                             impl_->config.num_classes);
        applyClassWiseNms(detections, iou_thresh_.load());
        return detections;
    } catch (...) {
        return {};
    }
}

void OpenCvOnnxEngine::setConfThresh(float thresh) { conf_thresh_.store(thresh); }
void OpenCvOnnxEngine::setIouThresh(float thresh) { iou_thresh_.store(thresh); }
float OpenCvOnnxEngine::getConfThresh() const { return conf_thresh_.load(); }
float OpenCvOnnxEngine::getIouThresh() const { return iou_thresh_.load(); }

}  // namespace rc26_vision
