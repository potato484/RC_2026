/// RC26_WS=${RC26_WS:-$HOME/RC_2026}
/// 当前默认按系统环境自动选择 AidLite / OpenCV ONNX 链路
/// ros2 run rc26_vision yolo_inference_test --model ${RC26_WS}/src/rc26_vision/models/kfs.onnx --input ${RC26_WS}/test/test14.png --show --show-width 960 --show-height 540
#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <cstdio>
#include <cmath>
#include <algorithm>

#include <opencv2/opencv.hpp>
#include "rc26_vision/engines/yolo_engine.hpp"

namespace {
constexpr const char* kWindowName = "YOLO Detection";

cv::Mat renderToCanvas(const cv::Mat& image, int canvas_width, int canvas_height) {
    if (image.empty()) return image;
    if (canvas_width <= 0 || canvas_height <= 0) return image;

    const int w = image.cols;
    const int h = image.rows;
    if (w <= 0 || h <= 0) return image;

    const double scale_w = static_cast<double>(canvas_width) / static_cast<double>(w);
    const double scale_h = static_cast<double>(canvas_height) / static_cast<double>(h);
    const double scale = std::min(scale_w, scale_h);  // 允许放大/缩小，尽量填满窗口

    cv::Mat resized;
    const int new_w = std::max(1, static_cast<int>(std::lround(w * scale)));
    const int new_h = std::max(1, static_cast<int>(std::lround(h * scale)));
    cv::resize(image, resized, cv::Size(new_w, new_h), 0, 0,
               (scale < 1.0) ? cv::INTER_AREA : cv::INTER_LINEAR);

    cv::Mat canvas(canvas_height, canvas_width, image.type(), cv::Scalar(24, 24, 24));
    const int x = std::max(0, (canvas_width - new_w) / 2);
    const int y = std::max(0, (canvas_height - new_h) / 2);
    resized.copyTo(canvas(cv::Rect(x, y, new_w, new_h)));
    return canvas;
}
}  // namespace

void drawDetections(cv::Mat& image, const std::vector<rc26_vision::Detection>& detections) {
    if (image.empty()) return;

    const int min_dim = std::min(image.cols, image.rows);
    const double font_scale = std::clamp(min_dim / 900.0, 0.7, 1.6);
    const int thickness = std::clamp(static_cast<int>(std::lround(font_scale * 2.0)), 2, 4);
    const int box_thickness = std::clamp(static_cast<int>(std::lround(font_scale * 2.5)), 2, 6);

    for (const auto& det : detections) {
        cv::Rect rect(
            static_cast<int>(det.x1),
            static_cast<int>(det.y1),
            static_cast<int>(det.x2 - det.x1),
            static_cast<int>(det.y2 - det.y1));

        cv::Scalar color(0, 255, 0);
        if (det.class_name.find("F_") == 0) color = cv::Scalar(0, 0, 255);  // 假目标红色
        if (det.class_name.find("T_") == 0) color = cv::Scalar(0, 255, 0);  // 真目标绿色

        cv::rectangle(image, rect, color, box_thickness, cv::LINE_AA);

        char label[96];
        snprintf(label, sizeof(label), "%s  %.1f%%", det.class_name.c_str(), det.score * 100.0);

        int baseline = 0;
        const int font_face = cv::FONT_HERSHEY_SIMPLEX;
        const cv::Size text_size = cv::getTextSize(label, font_face, font_scale, thickness, &baseline);

        const int pad = std::max(2, static_cast<int>(std::lround(font_scale * 6.0)));
        int x = rect.x;
        int y = rect.y - pad - baseline;
        if (y < 0) y = rect.y + rect.height + text_size.height + pad;
        y = std::clamp(y, text_size.height + pad, image.rows - 1);
        x = std::clamp(x, 0, std::max(0, image.cols - text_size.width - 2 * pad));

        const cv::Rect bg_rect(x, y - text_size.height - pad,
                               text_size.width + 2 * pad, text_size.height + 2 * pad);
        cv::rectangle(image, bg_rect, cv::Scalar(0, 0, 0), cv::FILLED, cv::LINE_AA);
        cv::rectangle(image, bg_rect, color, std::max(1, thickness - 1), cv::LINE_AA);

        cv::putText(image, label, cv::Point(x + pad, y),
                    font_face, font_scale, cv::Scalar(255, 255, 255), thickness, cv::LINE_AA);
    }
}

void printUsage(const char* prog) {
    std::cout << "用法: " << prog << " --model <onnx> --input <image/video> [options]\n"
              << "选项:\n"
              << "  --model <path>    ONNX 模型路径 (必需)\n"
              << "  --input <path>    输入图片或视频路径 (必需)\n"
              << "  --conf <float>    置信度阈值 (默认 0.5)\n"
              << "  --iou <float>     IOU 阈值 (默认 0.45)\n"
              << "  --show            显示检测结果窗口\n"
              << "  --show-width <n>  显示窗口最大宽度 (默认 1280)\n"
              << "  --show-height <n> 显示窗口最大高度 (默认 720)\n"
              << "  --save <path>     保存结果图片/视频\n"
              << "  --help            显示帮助\n";
}

int main(int argc, char** argv) {
    std::string model_path, input_path, save_path;
    float conf_thresh = 0.5f, iou_thresh = 0.45f;
    bool show_window = false;
    int show_max_width = 1280;
    int show_max_height = 720;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--model" && i + 1 < argc) model_path = argv[++i];
        else if (arg == "--input" && i + 1 < argc) input_path = argv[++i];
        else if (arg == "--conf" && i + 1 < argc) conf_thresh = std::stof(argv[++i]);
        else if (arg == "--iou" && i + 1 < argc) iou_thresh = std::stof(argv[++i]);
        else if (arg == "--save" && i + 1 < argc) save_path = argv[++i];
        else if (arg == "--show") show_window = true;
        else if (arg == "--show-width" && i + 1 < argc) show_max_width = std::stoi(argv[++i]);
        else if (arg == "--show-height" && i + 1 < argc) show_max_height = std::stoi(argv[++i]);
        else if (arg == "--help") { printUsage(argv[0]); return 0; }
    }

    if (model_path.empty() || input_path.empty()) {
        printUsage(argv[0]);
        return 1;
    }

    // 类别名称
    std::vector<std::string> class_names = {
        "R_R1", "B_R1",
        "T_03", "T_04", "T_05", "T_06", "T_07", "T_08", "T_09", "T_10",
        "T_11", "T_12", "T_13", "T_14", "T_15", "T_16", "T_17",
        "F_18", "F_19", "F_20", "F_21", "F_22", "F_23", "F_24", "F_25",
        "F_26", "F_27", "F_28", "F_29", "F_30", "F_31", "F_32"
    };

    // 加载模型
    std::cout << "[INFO] 加载模型: " << model_path << std::endl;
    rc26_vision::YoloEngine engine(model_path, class_names, conf_thresh);
    engine.setIouThresh(iou_thresh);
    std::cout << "[INFO] 模型加载成功 (conf=" << conf_thresh << ", iou=" << iou_thresh << ")\n";

    // 判断输入类型
    cv::Mat test_img = cv::imread(input_path);
    bool is_image = !test_img.empty();

    if (is_image) {
        // 图片模式
        std::cout << "[INFO] 处理图片: " << input_path << std::endl;

        auto t0 = std::chrono::high_resolution_clock::now();
        auto detections = engine.infer(test_img);
        auto t1 = std::chrono::high_resolution_clock::now();
        double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

        std::cout << "[INFO] 推理耗时: " << ms << " ms, 检测到 " << detections.size() << " 个目标\n";

        for (const auto& det : detections) {
            std::cout << "  - " << det.class_name << " (score=" << det.score
                      << ", bbox=[" << det.x1 << "," << det.y1 << "," << det.x2 << "," << det.y2 << "])\n";
        }

        drawDetections(test_img, detections);

        if (!save_path.empty()) {
            cv::imwrite(save_path, test_img);
            std::cout << "[INFO] 结果已保存: " << save_path << std::endl;
        }

        if (show_window) {
            cv::namedWindow(kWindowName, cv::WINDOW_NORMAL);
            if (show_max_width > 0 && show_max_height > 0) {
                cv::resizeWindow(kWindowName, show_max_width, show_max_height);
            }
            cv::imshow(kWindowName, renderToCanvas(test_img, show_max_width, show_max_height));
            cv::waitKey(0);
        }
    } else {
        // 视频模式
        cv::VideoCapture cap(input_path);
        if (!cap.isOpened()) {
            std::cerr << "[ERROR] 无法打开: " << input_path << std::endl;
            return 1;
        }

        std::cout << "[INFO] 处理视频: " << input_path << std::endl;

        cv::VideoWriter writer;
        if (!save_path.empty()) {
            int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
            double fps = cap.get(cv::CAP_PROP_FPS);
            int w = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
            int h = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
            writer.open(save_path, fourcc, fps, cv::Size(w, h));
        }

        cv::Mat frame;
        int frame_count = 0;
        double total_ms = 0;
        bool window_inited = false;

        while (cap.read(frame)) {
            auto t0 = std::chrono::high_resolution_clock::now();
            auto detections = engine.infer(frame);
            auto t1 = std::chrono::high_resolution_clock::now();
            double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
            total_ms += ms;
            frame_count++;

            drawDetections(frame, detections);

            if (writer.isOpened()) writer.write(frame);

            if (show_window) {
                if (!window_inited) {
                    cv::namedWindow(kWindowName, cv::WINDOW_NORMAL);
                    if (show_max_width > 0 && show_max_height > 0) {
                        cv::resizeWindow(kWindowName, show_max_width, show_max_height);
                    }
                    window_inited = true;
                }
                cv::imshow(kWindowName, renderToCanvas(frame, show_max_width, show_max_height));
                if (cv::waitKey(1) == 27) break;  // ESC 退出
            }

            if (frame_count % 30 == 0) {
                std::cout << "[INFO] 帧 " << frame_count << ", 平均耗时: "
                          << (total_ms / frame_count) << " ms\n";
            }
        }

        std::cout << "[INFO] 处理完成, 共 " << frame_count << " 帧, 平均耗时: "
                  << (total_ms / frame_count) << " ms\n";

        if (!save_path.empty()) {
            std::cout << "[INFO] 结果已保存: " << save_path << std::endl;
        }
    }

    return 0;
}
