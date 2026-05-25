#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "rc26_vision/shared/vision_types.hpp"
#include "rc26_vision/shared/yolo_transform.hpp"

namespace rc26_vision {

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
