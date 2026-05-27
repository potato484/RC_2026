#include "rc26_vision/postprocess/yolo/yolo_detection_postprocessor.hpp"

#include <algorithm>
#include <utility>

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
                                        int num_classes_hint) {
    std::vector<Detection> detections;
    if (output.empty() || orig_w <= 0 || orig_h <= 0 || output_channels == 0U ||
        output_predictions == 0U) {
        return detections;
    }

    const int64_t channels = static_cast<int64_t>(output_channels);
    const int64_t num_boxes = static_cast<int64_t>(output_predictions);
    const int64_t num_classes = std::max<int64_t>(
        1, (num_classes_hint > 0) ? num_classes_hint : static_cast<int64_t>(class_names.size()));
    const int64_t expected_v5_c = num_classes + 5;
    const bool has_objectness = (channels == expected_v5_c);
    const int64_t cls_offset = has_objectness ? 5 : 4;
    const int64_t out_num_classes = channels - cls_offset;
    if (num_boxes <= 0 || channels <= cls_offset || out_num_classes <= 0) {
        return detections;
    }

    auto get_val = [&](int64_t box_idx, int64_t attr_idx) -> float {
        if (output_channel_major) {
            return output[static_cast<std::size_t>(attr_idx * num_boxes + box_idx)];
        }
        return output[static_cast<std::size_t>(box_idx * channels + attr_idx)];
    };

    auto map_x = [&](float x) -> float {
        if (transform.letterbox) {
            return (x - static_cast<float>(transform.pad_x)) / transform.scale_x;
        }
        return x / transform.scale_x;
    };
    auto map_y = [&](float y) -> float {
        if (transform.letterbox) {
            return (y - static_cast<float>(transform.pad_y)) / transform.scale_y;
        }
        return y / transform.scale_y;
    };

    detections.reserve(static_cast<std::size_t>(num_boxes / 8));
    for (int64_t i = 0; i < num_boxes; ++i) {
        const float x = get_val(i, 0);
        const float y = get_val(i, 1);
        const float w = get_val(i, 2);
        const float h = get_val(i, 3);
        if (w <= 0.0F || h <= 0.0F) {
            continue;
        }

        const float obj = has_objectness ? get_val(i, 4) : 1.0F;
        int best_class = -1;
        float best_score = 0.0F;
        const int64_t class_limit = std::min<int64_t>(out_num_classes, num_classes);
        for (int64_t c = 0; c < class_limit; ++c) {
            const float score = get_val(i, cls_offset + c);
            if (score > best_score) {
                best_score = score;
                best_class = static_cast<int>(c);
            }
        }

        const float score = has_objectness ? (obj * best_score) : best_score;
        if (score < conf_thresh) {
            continue;
        }

        float x1 = map_x(x - w / 2.0F);
        float y1 = map_y(y - h / 2.0F);
        float x2 = map_x(x + w / 2.0F);
        float y2 = map_y(y + h / 2.0F);

        x1 = std::clamp(x1, 0.0F, static_cast<float>(orig_w - 1));
        y1 = std::clamp(y1, 0.0F, static_cast<float>(orig_h - 1));
        x2 = std::clamp(x2, 0.0F, static_cast<float>(orig_w - 1));
        y2 = std::clamp(y2, 0.0F, static_cast<float>(orig_h - 1));
        if (x2 <= x1 || y2 <= y1) {
            continue;
        }

        Detection det;
        det.x1 = x1;
        det.y1 = y1;
        det.x2 = x2;
        det.y2 = y2;
        det.score = score;
        det.class_id = best_class;
        if (best_class >= 0 && best_class < static_cast<int>(class_names.size())) {
            det.class_name = class_names[static_cast<std::size_t>(best_class)];
        }
        detections.push_back(std::move(det));
    }

    return detections;
}

void applyClassWiseNms(std::vector<Detection>& detections, float iou_thresh) {
    if (detections.empty()) {
        return;
    }

    std::sort(detections.begin(), detections.end(),
              [](const Detection& a, const Detection& b) { return a.score > b.score; });

    auto iou = [](const Detection& a, const Detection& b) -> float {
        const float xx1 = std::max(a.x1, b.x1);
        const float yy1 = std::max(a.y1, b.y1);
        const float xx2 = std::min(a.x2, b.x2);
        const float yy2 = std::min(a.y2, b.y2);
        const float w = std::max(0.0F, xx2 - xx1);
        const float h = std::max(0.0F, yy2 - yy1);
        const float inter = w * h;
        const float area_a = std::max(0.0F, a.x2 - a.x1) * std::max(0.0F, a.y2 - a.y1);
        const float area_b = std::max(0.0F, b.x2 - b.x1) * std::max(0.0F, b.y2 - b.y1);
        const float uni = area_a + area_b - inter;
        return (uni <= 0.0F) ? 0.0F : (inter / uni);
    };

    std::vector<Detection> kept;
    kept.reserve(detections.size());
    for (const auto& det : detections) {
        bool keep = true;
        for (const auto& prev : kept) {
            if (det.class_id != prev.class_id) {
                continue;
            }
            if (iou(det, prev) > iou_thresh) {
                keep = false;
                break;
            }
        }
        if (keep) {
            kept.push_back(det);
        }
    }
    detections.swap(kept);
}

}  // namespace rc26_vision
