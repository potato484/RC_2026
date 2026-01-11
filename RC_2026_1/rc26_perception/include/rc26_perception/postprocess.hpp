#pragma once

#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

namespace rc26_perception {

// 检测目标结构体
struct DetectedObject {
    cv::Rect box;    // 边界框
    int label;       // 类别 ID
    float score;     // 置信度
    int center_x;    // 边界框中心点 x
    int center_y;    // 边界框中心点 y
    float distance;  // 深度距离 (米)
};

// RC26 方块类别名称 (需要训练自定义模型后更新)
// 占位：使用 COCO 类别，后续替换为比赛专用类别
extern const std::vector<std::string> kBlockClasses;

// COCO 80 类别 (用于通用模型)
extern const std::vector<std::string> kCOCO80;

namespace postprocess {

// YOLO 后处理类
class YoloPost {
public:
    // 执行后处理 (cxcywh 格式)
    static void runCxcywh(float* boxes_ptr, uint32_t boxes_bytes, float* scores_ptr, uint32_t scores_bytes,
                          std::vector<DetectedObject>& objects, float conf_thres, float iou_thres, int input_size,
                          int orig_w, int orig_h, float scale, bool box_ch_first, bool sco_ch_first,
                          int num_classes = 80, int max_det = 300);
};

// 从深度图获取目标深度值
void fillDepth(DetectedObject& obj, const cv::Mat& depth_img, int radius = 2);

// 计算相机坐标系下的3D位置
void computeCameraPosition(DetectedObject& obj, float fx, float fy,  // 相机焦距
                           float cx, float cy                        // 相机光心
);

}  // namespace postprocess
}  // namespace rc26_perception
