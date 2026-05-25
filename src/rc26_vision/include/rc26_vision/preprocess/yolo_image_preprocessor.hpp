#pragma once

#include <string>

#include <opencv2/core.hpp>

#include "rc26_vision/shared/yolo_transform.hpp"

namespace rc26_vision {

int parsePaddingValue(const std::string& value);
bool prepareYoloInputImage(const cv::Mat& image,
                           int input_w,
                           int input_h,
                           const std::string& resize_mode,
                           int padding_value,
                           cv::Mat& model_rgb,
                           YoloImageTransform& transform);

}  // namespace rc26_vision
