#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "rc26_vision/runtime/types.hpp"

namespace rc26_vision {

struct YoloImageTransform {
    int src_w{0};
    int src_h{0};
    float scale_x{1.0F};
    float scale_y{1.0F};
    int pad_x{0};
    int pad_y{0};
    bool letterbox{false};
};

int parsePaddingValue(const std::string& value);
bool prepareYoloInputImage(const cv::Mat& image,
                           int input_w,
                           int input_h,
                           const std::string& resize_mode,
                           int padding_value,
                           cv::Mat& model_rgb,
                           YoloImageTransform& transform);
std::vector<Detection> decodeYoloOutput(const std::vector<float>& output,
                                        std::size_t output_channels,
                                        std::size_t output_predictions,
                                        bool output_channel_major,
                                        const std::vector<std::string>& class_names,
                                        float conf_thresh,
                                        int orig_w,
                                        int orig_h,
                                        const YoloImageTransform& transform,
                                        int num_classes_hint = 0);
void applyClassWiseNms(std::vector<Detection>& detections, float iou_thresh);

}  // namespace rc26_vision
