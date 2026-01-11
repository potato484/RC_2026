#pragma once

#include <opencv2/opencv.hpp>

namespace rc26_perception {
namespace preprocess {

// 左上角对齐 letterbox 填充
cv::Mat topleftLetterbox(const cv::Mat& src, int target, float& scale_out);

// BGR 图像转换为 RGB 归一化 blob
void toBlobRgb01(const cv::Mat& img_bgr, float* dst, bool nchw);

}  // namespace preprocess
}  // namespace rc26_perception
