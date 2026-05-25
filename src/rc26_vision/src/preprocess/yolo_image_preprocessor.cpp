#include "rc26_vision/preprocess/yolo_image_preprocessor.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>

#include <opencv2/imgproc.hpp>

namespace rc26_vision {

int parsePaddingValue(const std::string& value) {
    std::string lower = value;
    std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    if (lower.empty() || lower == "black") return 0;
    if (lower == "white") return 255;
    if (lower == "gray" || lower == "grey" || lower == "gray114" || lower == "grey114") return 114;
    try {
        return std::clamp(std::stoi(lower), 0, 255);
    } catch (...) {
        return 114;
    }
}

bool prepareYoloInputImage(const cv::Mat& image,
                           int input_w,
                           int input_h,
                           const std::string& resize_mode,
                           int padding_value,
                           cv::Mat& model_rgb,
                           YoloImageTransform& transform) {
    if (image.empty() || input_w <= 0 || input_h <= 0) {
        return false;
    }

    cv::Mat rgb;
    switch (image.channels()) {
        case 3:
            cv::cvtColor(image, rgb, cv::COLOR_BGR2RGB);
            break;
        case 4:
            cv::cvtColor(image, rgb, cv::COLOR_BGRA2RGB);
            break;
        case 1:
            cv::cvtColor(image, rgb, cv::COLOR_GRAY2RGB);
            break;
        default:
            return false;
    }

    transform.src_w = image.cols;
    transform.src_h = image.rows;
    transform.pad_x = 0;
    transform.pad_y = 0;
    transform.letterbox = (resize_mode != "stretch");

    if (transform.letterbox) {
        const float sx = static_cast<float>(input_w) / static_cast<float>(image.cols);
        const float sy = static_cast<float>(input_h) / static_cast<float>(image.rows);
        const float scale = std::min(sx, sy);
        const int new_w = std::max(1, static_cast<int>(std::round(image.cols * scale)));
        const int new_h = std::max(1, static_cast<int>(std::round(image.rows * scale)));
        transform.scale_x = scale;
        transform.scale_y = scale;
        transform.pad_x = (input_w - new_w) / 2;
        transform.pad_y = (input_h - new_h) / 2;
        model_rgb = cv::Mat(
            input_h, input_w, CV_8UC3, cv::Scalar(padding_value, padding_value, padding_value));
        cv::Mat resized;
        cv::resize(rgb, resized, cv::Size(new_w, new_h), 0.0, 0.0, cv::INTER_LINEAR);
        resized.copyTo(model_rgb(cv::Rect(transform.pad_x, transform.pad_y, new_w, new_h)));
    } else {
        transform.scale_x = static_cast<float>(input_w) / static_cast<float>(image.cols);
        transform.scale_y = static_cast<float>(input_h) / static_cast<float>(image.rows);
        cv::resize(rgb, model_rgb, cv::Size(input_w, input_h), 0.0, 0.0, cv::INTER_LINEAR);
    }

    if (!model_rgb.isContinuous()) {
        model_rgb = model_rgb.clone();
    }
    return true;
}

}  // namespace rc26_vision
