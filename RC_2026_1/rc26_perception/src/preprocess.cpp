#include "rc26_perception/preprocess.hpp"
#include <algorithm>

namespace rc26_perception {
namespace preprocess {

cv::Mat topleftLetterbox(const cv::Mat& src, int target, float& scale_out) {
    const int h = src.rows, w = src.cols;
    const float scale1 = static_cast<float>(h) / target;
    const float scale2 = static_cast<float>(w) / target;
    const float scale = std::max(scale1, scale2);
    int new_w = static_cast<int>(w / scale);
    int new_h = static_cast<int>(h / scale);
    cv::Mat resized;
    cv::resize(src, resized, cv::Size(new_w, new_h), 0, 0, cv::INTER_LINEAR);
    cv::Mat out(target, target, CV_8UC3, cv::Scalar(0, 0, 0));
    resized.copyTo(out(cv::Rect(0, 0, new_w, new_h)));
    scale_out = scale;
    return out;
}

void toBlobRgb01(const cv::Mat& img_bgr, float* dst, bool nchw) {
    const int H = img_bgr.rows;
    const int W = img_bgr.cols;
    const float scale = 1.f / 255.f;

    if (nchw) {
        const int HW = H * W;
        int r_off = 0, g_off = HW, b_off = 2 * HW;
        for (int i = 0; i < H; ++i) {
            const uchar* p = img_bgr.ptr<uchar>(i);
            for (int j = 0; j < W; ++j) {
                const int pos = i * W + j;
                const int idx = j * 3;
                const float b = p[idx + 0] * scale;
                const float g = p[idx + 1] * scale;
                const float r = p[idx + 2] * scale;
                dst[r_off + pos] = r;
                dst[g_off + pos] = g;
                dst[b_off + pos] = b;
            }
        }
    } else {
        for (int i = 0; i < H; ++i) {
            const uchar* p = img_bgr.ptr<uchar>(i);
            for (int j = 0; j < W; ++j) {
                const int pos = (i * W + j) * 3;
                const float b = p[j * 3 + 0] * scale;
                const float g = p[j * 3 + 1] * scale;
                const float r = p[j * 3 + 2] * scale;
                dst[pos + 0] = r;
                dst[pos + 1] = g;
                dst[pos + 2] = b;
            }
        }
    }
}

}  // namespace preprocess
}  // namespace rc26_perception
