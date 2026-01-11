#include "rc26_perception/postprocess.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

#include <opencv2/dnn.hpp>

namespace rc26_perception {

// RC26 方块类别 (按YOLO训练类别顺序)
// R1: 手动机器人KFS (比赛LOGO)
// T_03~T_17: 自动机器人KFS (甲骨文字，共15种)
// F_18~F_32: 假KFS (小篆体文字，共15种)
const std::vector<std::string> kBlockClasses = {
    "R1",    // 0: 手动机器人KFS (比赛LOGO)
    "T_03",  // 1: 自动机器人KFS - 甲骨文字
    "T_04",  // 2: 自动机器人KFS - 甲骨文字
    "T_05",  // 3: 自动机器人KFS - 甲骨文字
    "T_06",  // 4: 自动机器人KFS - 甲骨文字
    "T_07",  // 5: 自动机器人KFS - 甲骨文字
    "T_08",  // 6: 自动机器人KFS - 甲骨文字
    "T_09",  // 7: 自动机器人KFS - 甲骨文字
    "T_10",  // 8: 自动机器人KFS - 甲骨文字
    "T_11",  // 9: 自动机器人KFS - 甲骨文字
    "T_12",  // 10: 自动机器人KFS - 甲骨文字
    "T_13",  // 11: 自动机器人KFS - 甲骨文字
    "T_14",  // 12: 自动机器人KFS - 甲骨文字
    "T_15",  // 13: 自动机器人KFS - 甲骨文字
    "T_16",  // 14: 自动机器人KFS - 甲骨文字
    "T_17",  // 15: 自动机器人KFS - 甲骨文字
    "F_18",  // 16: 假KFS - 小篆体文字
    "F_19",  // 17: 假KFS - 小篆体文字
    "F_20",  // 18: 假KFS - 小篆体文字
    "F_21",  // 19: 假KFS - 小篆体文字
    "F_22",  // 20: 假KFS - 小篆体文字
    "F_23",  // 21: 假KFS - 小篆体文字
    "F_24",  // 22: 假KFS - 小篆体文字
    "F_25",  // 23: 假KFS - 小篆体文字
    "F_26",  // 24: 假KFS - 小篆体文字
    "F_27",  // 25: 假KFS - 小篆体文字
    "F_28",  // 26: 假KFS - 小篆体文字
    "F_29",  // 27: 假KFS - 小篆体文字
    "F_30",  // 28: 假KFS - 小篆体文字
    "F_31",  // 29: 假KFS - 小篆体文字
    "F_32"   // 30: 假KFS - 小篆体文字
};

// COCO 80 类别 (用于通用模型调试)
const std::vector<std::string> kCOCO80 = {
    "person",         "bicycle",    "car",           "motorcycle",    "airplane",     "bus",           "train",
    "truck",          "boat",       "traffic light", "fire hydrant",  "stop sign",    "parking meter", "bench",
    "bird",           "cat",        "dog",           "horse",         "sheep",        "cow",           "elephant",
    "bear",           "zebra",      "giraffe",       "backpack",      "umbrella",     "handbag",       "tie",
    "suitcase",       "frisbee",    "skis",          "snowboard",     "sports ball",  "kite",          "baseball bat",
    "baseball glove", "skateboard", "surfboard",     "tennis racket", "bottle",       "wine glass",    "cup",
    "fork",           "knife",      "spoon",         "bowl",          "banana",       "apple",         "sandwich",
    "orange",         "broccoli",   "carrot",        "hot dog",       "pizza",        "donut",         "cake",
    "chair",          "couch",      "potted plant",  "bed",           "dining table", "toilet",        "tv",
    "laptop",         "mouse",      "remote",        "keyboard",      "cell phone",   "microwave",     "oven",
    "toaster",        "sink",       "refrigerator",  "book",          "clock",        "vase",          "scissors",
    "teddy bear",     "hair drier", "toothbrush"};

namespace postprocess {

void YoloPost::runCxcywh(float* boxes_ptr, uint32_t boxes_bytes, float* scores_ptr, uint32_t /*scores_bytes*/,
                         std::vector<DetectedObject>& objects, float conf_thres, float iou_thres, int /*input_size*/,
                         int orig_w, int orig_h, float scale, bool box_ch_first, bool sco_ch_first, int num_classes,
                         int max_det) {
    objects.clear();
    if (!boxes_ptr || !scores_ptr)
        return;

    const int floats_box = (int)(boxes_bytes / sizeof(float));
    if (floats_box % 4 != 0)
        return;
    const int anchors = floats_box / 4;

    auto get_box = [&](int i, int k) -> float {
        return box_ch_first ? boxes_ptr[k * anchors + i] : boxes_ptr[i * 4 + k];
    };
    auto get_score = [&](int i, int c) -> float {
        return sco_ch_first ? scores_ptr[c * anchors + i] : scores_ptr[i * num_classes + c];
    };

    std::vector<cv::Rect> boxes;
    std::vector<float> scores;
    std::vector<int> class_ids;
    std::vector<int> center_xs;
    std::vector<int> center_ys;
    boxes.reserve(anchors);
    scores.reserve(anchors);
    class_ids.reserve(anchors);
    center_xs.reserve(anchors);
    center_ys.reserve(anchors);

    for (int i = 0; i < anchors; ++i) {
        float best_s = 0.f;
        int best_c = -1;
        for (int c = 0; c < num_classes; ++c) {
            float s = get_score(i, c);
            if (s > best_s) {
                best_s = s;
                best_c = c;
            }
        }
        if (best_s < conf_thres)
            continue;

        float cx = get_box(i, 0);
        float cy = get_box(i, 1);
        float w = get_box(i, 2);
        float h = get_box(i, 3);

        float x1 = cx - 0.5f * w;
        float y1 = cy - 0.5f * h;
        float x2 = x1 + w;
        float y2 = y1 + h;

        float rx1 = x1 * scale;
        float ry1 = y1 * scale;
        float rx2 = x2 * scale;
        float ry2 = y2 * scale;

        rx1 = std::max(0.f, std::min(rx1, (float)orig_w - 1.f));
        ry1 = std::max(0.f, std::min(ry1, (float)orig_h - 1.f));
        rx2 = std::max(0.f, std::min(rx2, (float)orig_w - 1.f));
        ry2 = std::max(0.f, std::min(ry2, (float)orig_h - 1.f));

        int bw = (int)std::round(rx2 - rx1);
        int bh = (int)std::round(ry2 - ry1);
        if (bw <= 0 || bh <= 0)
            continue;

        int box_x = (int)std::round(rx1);
        int box_y = (int)std::round(ry1);

        int cx_orig = box_x + bw / 2;
        int cy_orig = box_y + bh / 2;

        boxes.emplace_back(box_x, box_y, bw, bh);
        scores.emplace_back(best_s);
        class_ids.emplace_back(best_c);
        center_xs.emplace_back(cx_orig);
        center_ys.emplace_back(cy_orig);
    }

    if (boxes.empty())
        return;

    std::vector<int> order(boxes.size());
    std::iota(order.begin(), order.end(), 0);
    const int take = std::min((int)order.size(), max_det);
    std::partial_sort(order.begin(), order.begin() + take, order.end(),
                      [&](int a, int b) { return scores[a] > scores[b]; });

    std::vector<cv::Rect> boxes_top;
    boxes_top.reserve(take);
    std::vector<float> scores_top;
    scores_top.reserve(take);
    std::vector<int> class_top;
    class_top.reserve(take);
    std::vector<int> cx_top;
    cx_top.reserve(take);
    std::vector<int> cy_top;
    cy_top.reserve(take);

    for (int i = 0; i < take; ++i) {
        int idx = order[i];
        boxes_top.push_back(boxes[idx]);
        scores_top.push_back(scores[idx]);
        class_top.push_back(class_ids[idx]);
        cx_top.push_back(center_xs[idx]);
        cy_top.push_back(center_ys[idx]);
    }

    std::vector<int> keep;
    cv::dnn::NMSBoxes(boxes_top, scores_top, conf_thres, iou_thres, keep);

    objects.reserve(keep.size());
    for (int k : keep) {
        DetectedObject obj;
        obj.box = boxes_top[k];
        obj.label = class_top[k];
        obj.score = scores_top[k];
        obj.center_x = cx_top[k];
        obj.center_y = cy_top[k];
        obj.distance = 0.0f;
        objects.push_back(obj);
    }
}

void fillDepth(DetectedObject& obj, const cv::Mat& depth_img, int radius) {
    int cx = obj.center_x;
    int cy = obj.center_y;

    cx = std::max(0, std::min(cx, depth_img.cols - 1));
    cy = std::max(0, std::min(cy, depth_img.rows - 1));

    float depth_sum = 0.f;
    int valid_count = 0;

    for (int dy = -radius; dy <= radius; ++dy) {
        for (int dx = -radius; dx <= radius; ++dx) {
            int px = cx + dx;
            int py = cy + dy;
            if (px >= 0 && px < depth_img.cols && py >= 0 && py < depth_img.rows) {
                uint16_t d = depth_img.at<uint16_t>(py, px);
                if (d > 0 && d < 10000) {
                    depth_sum += d;
                    valid_count++;
                }
            }
        }
    }

    if (valid_count > 0) {
        obj.distance = (depth_sum / valid_count) / 1000.0f;
    } else {
        obj.distance = 0.0f;
    }
}

void computeCameraPosition(DetectedObject& obj, float fx, float fy, float cx, float cy) {
    // 从像素坐标 + 深度 计算相机坐标系下的3D位置
    // X = (u - cx) * Z / fx
    // Y = (v - cy) * Z / fy
    // Z = distance
    if (obj.distance <= 0.0f) {
        return;
    }

    float Z = obj.distance;
    float X = (obj.center_x - cx) * Z / fx;
    float Y = (obj.center_y - cy) * Z / fy;

    // 存储到 box 中 (临时方案，后续使用独立字段)
    // 这里直接返回，由调用者处理
    (void)X;
    (void)Y;
    (void)Z;
}

}  // namespace postprocess
}  // namespace rc26_perception
