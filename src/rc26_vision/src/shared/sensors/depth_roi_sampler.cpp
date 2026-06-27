#include "rc26_vision/shared/sensors/depth_roi_sampler.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace rc26_vision {

std::optional<double> sampleMedianDepth(const cv::Mat& depth,
                                        int cx,
                                        int cy,
                                        const DepthRoiSamplerConfig& config) {
    if (depth.empty() || depth.rows <= 0 || depth.cols <= 0) {
        return std::nullopt;
    }
    if (depth.channels() != 1 || config.roi_size <= 0 || config.min_valid_count <= 0) {
        return std::nullopt;
    }

    const int half = config.roi_size / 2;
    cx = std::clamp(cx, 0, depth.cols - 1);
    cy = std::clamp(cy, 0, depth.rows - 1);

    const int x0 = std::max(0, cx - half);
    const int x1 = std::min(depth.cols - 1, cx + half);
    const int y0 = std::max(0, cy - half);
    const int y1 = std::min(depth.rows - 1, cy + half);
    if (x0 > x1 || y0 > y1) {
        return std::nullopt;
    }

    std::vector<double> samples;
    samples.reserve(static_cast<std::size_t>((x1 - x0 + 1) * (y1 - y0 + 1)));
    for (int y = y0; y <= y1; ++y) {
        for (int x = x0; x <= x1; ++x) {
            double z_m = 0.0;
            if (depth.type() == CV_16UC1) {
                const uint16_t z_mm = depth.at<uint16_t>(y, x);
                if (z_mm == 0U) {
                    continue;
                }
                z_m = static_cast<double>(z_mm) * 1e-3;
            } else if (depth.type() == CV_32FC1) {
                const float z = depth.at<float>(y, x);
                if (!std::isfinite(z) || z <= 0.0F) {
                    continue;
                }
                z_m = static_cast<double>(z);
            } else {
                return std::nullopt;
            }

            if (z_m < config.min_depth_m || z_m > config.max_depth_m) {
                continue;
            }
            samples.push_back(z_m);
        }
    }

    if (static_cast<int>(samples.size()) < config.min_valid_count) {
        return std::nullopt;
    }

    const auto mid = samples.begin() + static_cast<std::ptrdiff_t>(samples.size() / 2);
    std::nth_element(samples.begin(), mid, samples.end());
    return *mid;
}

}  // namespace rc26_vision
