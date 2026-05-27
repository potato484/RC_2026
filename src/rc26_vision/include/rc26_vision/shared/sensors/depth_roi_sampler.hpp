#pragma once

#include <limits>
#include <optional>

#include <opencv2/core.hpp>

namespace rc26_vision {

struct DepthRoiSamplerConfig {
    int roi_size{5};
    int min_valid_count{1};
    double min_depth_m{0.0};
    double max_depth_m{std::numeric_limits<double>::infinity()};
};

std::optional<double> sampleMedianDepth(const cv::Mat& depth,
                                        int cx,
                                        int cy,
                                        const DepthRoiSamplerConfig& config);

}  // namespace rc26_vision
