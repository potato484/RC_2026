// RC26 感知可视化节点
// 使用 OpenCV DNN 加载 ONNX 模型，实时显示检测结果
// 用法: ros2 run rc26_perception visualization_node --ros-args -p model_path:=/path/to/yolov8s.onnx

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

#include "rc26_perception/postprocess.hpp"
#include "rc26_perception/preprocess.hpp"

#include <memory>
#include <vector>
#include <chrono>

namespace rc26_perception {

// 颜色表 (BGR)
const std::vector<cv::Scalar> kColors = {
    cv::Scalar(255, 56, 56),    // 红
    cv::Scalar(56, 255, 56),    // 绿
    cv::Scalar(56, 56, 255),    // 蓝
    cv::Scalar(255, 178, 56),   // 橙
    cv::Scalar(178, 56, 255),   // 紫
    cv::Scalar(56, 255, 255),   // 黄
    cv::Scalar(255, 56, 178),   // 粉
    cv::Scalar(56, 178, 255),   // 青
};

class VisualizationNode : public rclcpp::Node {
public:
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, sensor_msgs::msg::Image>;

    VisualizationNode() : Node("rc26_visualization_node") {
        // 声明参数
        this->declare_parameter<std::string>("model_path", "");
        this->declare_parameter<int>("input_size", 640);
        this->declare_parameter<double>("conf_thres", 0.45);
        this->declare_parameter<double>("iou_thres", 0.45);
        this->declare_parameter<int>("num_classes", 80);
        this->declare_parameter<bool>("use_custom_classes", false);
        this->declare_parameter<std::string>("window_name", "RC26 Detection");
        this->declare_parameter<bool>("show_fps", true);
        this->declare_parameter<bool>("show_depth", true);
        
        // 话题参数
        this->declare_parameter<std::string>("color_topic", "/camera/color/image_raw");
        this->declare_parameter<std::string>("depth_topic", "/camera/aligned_depth_to_color/image_raw");
        
        // 深度可视化参数
        this->declare_parameter<bool>("show_depth_window", true);   // 显示深度图窗口
        this->declare_parameter<int>("depth_colormap", 2);          // 深度色图: 0=灰度, 2=JET, 4=RAINBOW, 11=TURBO
        this->declare_parameter<double>("depth_vis_min", 0.1);      // 深度可视化最小值 (米)
        this->declare_parameter<double>("depth_vis_max", 5.0);      // 深度可视化最大值 (米)
        
        // 获取参数
        model_path_ = this->get_parameter("model_path").as_string();
        input_size_ = this->get_parameter("input_size").as_int();
        conf_thres_ = this->get_parameter("conf_thres").as_double();
        iou_thres_ = this->get_parameter("iou_thres").as_double();
        num_classes_ = this->get_parameter("num_classes").as_int();
        use_custom_classes_ = this->get_parameter("use_custom_classes").as_bool();
        window_name_ = this->get_parameter("window_name").as_string();
        show_fps_ = this->get_parameter("show_fps").as_bool();
        show_depth_ = this->get_parameter("show_depth").as_bool();
        
        std::string color_topic = this->get_parameter("color_topic").as_string();
        std::string depth_topic = this->get_parameter("depth_topic").as_string();
        
        // 获取深度可视化参数
        show_depth_window_ = this->get_parameter("show_depth_window").as_bool();
        depth_colormap_ = this->get_parameter("depth_colormap").as_int();
        depth_vis_min_ = this->get_parameter("depth_vis_min").as_double();
        depth_vis_max_ = this->get_parameter("depth_vis_max").as_double();

        // 初始化 OpenCV DNN
        if (!model_path_.empty()) {
            try {
                RCLCPP_INFO(this->get_logger(), "Loading ONNX model: %s", model_path_.c_str());
                net_ = cv::dnn::readNetFromONNX(model_path_);
                
                // 尝试使用 CUDA (如果可用)
                net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
                net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
                
                model_loaded_ = true;
                RCLCPP_INFO(this->get_logger(), "Model loaded successfully");
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to load model: %s", e.what());
                model_loaded_ = false;
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "No model_path specified, visualization only");
            model_loaded_ = false;
        }

        // 创建窗口
        cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
        if (show_depth_window_) {
            cv::namedWindow("RC26 Depth", cv::WINDOW_AUTOSIZE);
        }

        // 使用 message_filters 同步彩色图和深度图
        color_sub_.subscribe(this, color_topic, rmw_qos_profile_sensor_data);
        depth_sub_.subscribe(this, depth_topic, rmw_qos_profile_sensor_data);

        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), color_sub_, depth_sub_);
        sync_->registerCallback(std::bind(&VisualizationNode::imageCallback, this,
                                          std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "RC26 Visualization Node started");
        RCLCPP_INFO(this->get_logger(), "  Color topic: %s", color_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  Depth topic: %s", depth_topic.c_str());
        if (show_depth_window_) {
            RCLCPP_INFO(this->get_logger(), "  Depth window: ON (colormap=%d, range=%.1f-%.1fm)",
                        depth_colormap_, depth_vis_min_, depth_vis_max_);
        }
        RCLCPP_INFO(this->get_logger(), "  Press 'q' to quit");
    }

    ~VisualizationNode() {
        cv::destroyAllWindows();
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& color_msg,
                       const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg) {
        auto start_time = std::chrono::high_resolution_clock::now();
        
        // 转换彩色图像
        cv_bridge::CvImagePtr color_ptr;
        try {
            color_ptr = cv_bridge::toCvCopy(color_msg, sensor_msgs::image_encodings::BGR8);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Color cv_bridge exception: %s", e.what());
            return;
        }
        cv::Mat frame = color_ptr->image;
        if (frame.empty()) return;

        // 转换深度图像
        cv::Mat depth_img;
        if (show_depth_) {
            cv_bridge::CvImagePtr depth_ptr;
            try {
                depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
                depth_img = depth_ptr->image;
            } catch (const std::exception& e) {
                RCLCPP_WARN_ONCE(this->get_logger(), "Depth unavailable: %s", e.what());
            }
        }

        // 执行推理
        std::vector<DetectedObject> detections;
        if (model_loaded_) {
            runInference(frame, detections);
            
            // 填充深度信息
            if (!depth_img.empty()) {
                for (auto& det : detections) {
                    postprocess::fillDepth(det, depth_img, 2);
                }
            }
        }

        // 绘制检测结果
        drawDetections(frame, detections);

        // 计算并显示 FPS
        if (show_fps_) {
            auto end_time = std::chrono::high_resolution_clock::now();
            double ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
            double fps = 1000.0 / ms;
            
            // 更新平滑 FPS
            fps_smooth_ = 0.9 * fps_smooth_ + 0.1 * fps;
            
            char fps_text[64];
            snprintf(fps_text, sizeof(fps_text), "FPS: %.1f (%.1f ms)", fps_smooth_, ms);
            cv::putText(frame, fps_text, cv::Point(10, 30), 
                        cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
        }

        // 显示检测数量
        char det_text[64];
        snprintf(det_text, sizeof(det_text), "Detections: %zu", detections.size());
        cv::putText(frame, det_text, cv::Point(10, 60), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);

        // 显示图像
        cv::imshow(window_name_, frame);
        
        // 显示深度图窗口
        if (show_depth_window_ && !depth_img.empty()) {
            cv::Mat depth_vis = visualizeDepth(depth_img);
            cv::imshow("RC26 Depth", depth_vis);
        }
        
        // 检查按键
        int key = cv::waitKey(1);
        if (key == 'q' || key == 'Q' || key == 27) {  // q, Q, ESC
            RCLCPP_INFO(this->get_logger(), "Quit requested");
            rclcpp::shutdown();
        }
    }
    
    cv::Mat visualizeDepth(const cv::Mat& depth_img) {
        // 深度范围 (毫米)
        double min_mm = depth_vis_min_ * 1000.0;
        double max_mm = depth_vis_max_ * 1000.0;
        
        // 转换为浮点并归一化
        cv::Mat depth_float;
        depth_img.convertTo(depth_float, CV_32F);
        
        // 裁剪到可视化范围
        cv::Mat normalized;
        depth_float = cv::max(depth_float, min_mm);
        depth_float = cv::min(depth_float, max_mm);
        
        // 归一化到 0-255
        normalized = (depth_float - min_mm) / (max_mm - min_mm) * 255.0;
        normalized.convertTo(normalized, CV_8U);
        
        // 应用颜色映射
        cv::Mat colored;
        if (depth_colormap_ == 0) {
            // 灰度图
            cv::cvtColor(normalized, colored, cv::COLOR_GRAY2BGR);
        } else {
            // 使用指定的 colormap
            cv::applyColorMap(normalized, colored, depth_colormap_);
        }
        
        // 将无效深度 (0) 标记为黑色
        cv::Mat invalid_mask = (depth_img == 0);
        colored.setTo(cv::Scalar(0, 0, 0), invalid_mask);
        
        // 添加深度刻度条
        drawDepthColorbar(colored);
        
        return colored;
    }
    
    void drawDepthColorbar(cv::Mat& img) {
        int bar_width = 20;
        int bar_height = img.rows - 40;
        int bar_x = img.cols - bar_width - 10;
        int bar_y = 20;
        
        // 绘制颜色条
        for (int i = 0; i < bar_height; ++i) {
            float ratio = 1.0f - (float)i / bar_height;
            uint8_t val = (uint8_t)(ratio * 255);
            
            cv::Mat row_color;
            cv::Mat val_mat(1, 1, CV_8U, cv::Scalar(val));
            if (depth_colormap_ == 0) {
                row_color = cv::Mat(1, bar_width, CV_8UC3, cv::Scalar(val, val, val));
            } else {
                cv::Mat colored;
                cv::applyColorMap(val_mat, colored, depth_colormap_);
                cv::Vec3b color = colored.at<cv::Vec3b>(0, 0);
                row_color = cv::Mat(1, bar_width, CV_8UC3, cv::Scalar(color[0], color[1], color[2]));
            }
            row_color.copyTo(img(cv::Rect(bar_x, bar_y + i, bar_width, 1)));
        }
        
        // 绘制边框
        cv::rectangle(img, cv::Rect(bar_x, bar_y, bar_width, bar_height), cv::Scalar(255, 255, 255), 1);
        
        // 绘制刻度标签
        char label[32];
        snprintf(label, sizeof(label), "%.1fm", depth_vis_max_);
        cv::putText(img, label, cv::Point(bar_x - 35, bar_y + 10), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);
        snprintf(label, sizeof(label), "%.1fm", (depth_vis_min_ + depth_vis_max_) / 2);
        cv::putText(img, label, cv::Point(bar_x - 35, bar_y + bar_height / 2), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);
        snprintf(label, sizeof(label), "%.1fm", depth_vis_min_);
        cv::putText(img, label, cv::Point(bar_x - 35, bar_y + bar_height - 5), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);
    }

    void runInference(const cv::Mat& frame, std::vector<DetectedObject>& detections) {
        // 预处理：创建 blob
        cv::Mat blob;
        cv::dnn::blobFromImage(frame, blob, 1.0/255.0, 
                               cv::Size(input_size_, input_size_),
                               cv::Scalar(0, 0, 0), true, false);
        
        net_.setInput(blob);
        
        // 推理 - 使用 vector<Mat> 获取输出 (OpenCV 4.5.4 兼容)
        std::vector<cv::Mat> outputs;
        net_.forward(outputs, net_.getUnconnectedOutLayersNames());
        
        if (outputs.empty()) {
            RCLCPP_WARN(this->get_logger(), "No outputs from network");
            return;
        }
        
        cv::Mat output = outputs[0];
        
        // YOLOv8 输出格式: [1, 84, 8400] (84 = 4 box + 80 classes)
        // 处理多维 Mat
        int num_features = 84;  // 4 + num_classes
        
        // 如果是 3D [1, 84, 8400]，需要 reshape
        if (output.dims == 3) {
            num_features = output.size[1];
            output = output.reshape(1, num_features);
        } else if (output.dims == 2) {
            // 已经是 2D [84, 8400]
            num_features = output.rows;
        }
        
        // 转置为 [8400, 84]
        cv::transpose(output, output);
        
        int anchors = output.rows;  // 8400
        
        // 计算缩放比例
        float scale_x = (float)frame.cols / input_size_;
        float scale_y = (float)frame.rows / input_size_;
        
        // 解析输出
        float* data = (float*)output.data;
        
        std::vector<cv::Rect> boxes;
        std::vector<float> scores;
        std::vector<int> class_ids;
        std::vector<int> center_xs;
        std::vector<int> center_ys;
        
        int num_cols = 4 + num_classes_;  // 84
        for (int i = 0; i < anchors; ++i) {
            float* row = data + i * num_cols;
            
            // 获取 box 坐标 (cx, cy, w, h)
            float cx = row[0];
            float cy = row[1];
            float w = row[2];
            float h = row[3];
            
            // 找到最大类别分数
            float max_score = 0.0f;
            int max_class = -1;
            for (int c = 0; c < num_classes_; ++c) {
                float s = row[4 + c];
                if (s > max_score) {
                    max_score = s;
                    max_class = c;
                }
            }
            
            if (max_score < conf_thres_) continue;
            
            // 转换为原始图像坐标
            float x1 = (cx - w / 2) * scale_x;
            float y1 = (cy - h / 2) * scale_y;
            float x2 = (cx + w / 2) * scale_x;
            float y2 = (cy + h / 2) * scale_y;
            
            // 裁剪到图像边界
            x1 = std::max(0.f, std::min(x1, (float)frame.cols - 1));
            y1 = std::max(0.f, std::min(y1, (float)frame.rows - 1));
            x2 = std::max(0.f, std::min(x2, (float)frame.cols - 1));
            y2 = std::max(0.f, std::min(y2, (float)frame.rows - 1));
            
            int box_w = (int)(x2 - x1);
            int box_h = (int)(y2 - y1);
            if (box_w <= 0 || box_h <= 0) continue;
            
            boxes.emplace_back((int)x1, (int)y1, box_w, box_h);
            scores.push_back(max_score);
            class_ids.push_back(max_class);
            center_xs.push_back((int)((x1 + x2) / 2));
            center_ys.push_back((int)((y1 + y2) / 2));
        }
        
        // NMS
        std::vector<int> keep;
        cv::dnn::NMSBoxes(boxes, scores, (float)conf_thres_, (float)iou_thres_, keep);
        
        // 填充结果
        detections.reserve(keep.size());
        for (int idx : keep) {
            DetectedObject obj;
            obj.box = boxes[idx];
            obj.score = scores[idx];
            obj.label = class_ids[idx];
            obj.center_x = center_xs[idx];
            obj.center_y = center_ys[idx];
            obj.distance = 0.0f;
            detections.push_back(obj);
        }
    }

    void drawDetections(cv::Mat& frame, const std::vector<DetectedObject>& detections) {
        for (const auto& det : detections) {
            // 选择颜色
            cv::Scalar color = kColors[det.label % kColors.size()];
            
            // 绘制边界框
            cv::rectangle(frame, det.box, color, 2);
            
            // 获取类别名称
            std::string class_name;
            if (use_custom_classes_ && det.label < (int)kBlockClasses.size()) {
                class_name = kBlockClasses[det.label];
            } else if (det.label < (int)kCOCO80.size()) {
                class_name = kCOCO80[det.label];
            } else {
                class_name = "class_" + std::to_string(det.label);
            }
            
            // 构建标签文本
            char label_text[128];
            if (det.distance > 0.0f) {
                snprintf(label_text, sizeof(label_text), "%s: %.2f (%.2fm)", 
                         class_name.c_str(), det.score, det.distance);
            } else {
                snprintf(label_text, sizeof(label_text), "%s: %.2f", 
                         class_name.c_str(), det.score);
            }
            
            // 计算标签背景
            int baseline = 0;
            cv::Size text_size = cv::getTextSize(label_text, cv::FONT_HERSHEY_SIMPLEX, 
                                                  0.5, 1, &baseline);
            
            int label_y = std::max(det.box.y - 5, text_size.height + 5);
            
            // 绘制标签背景
            cv::rectangle(frame, 
                          cv::Point(det.box.x, label_y - text_size.height - 5),
                          cv::Point(det.box.x + text_size.width + 5, label_y + 5),
                          color, cv::FILLED);
            
            // 绘制标签文本
            cv::putText(frame, label_text, 
                        cv::Point(det.box.x + 2, label_y),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
            
            // 绘制中心点
            cv::circle(frame, cv::Point(det.center_x, det.center_y), 4, color, -1);
        }
    }

private:
    // OpenCV DNN
    cv::dnn::Net net_;
    bool model_loaded_ = false;
    
    // ROS message_filters
    message_filters::Subscriber<sensor_msgs::msg::Image> color_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    
    // 参数
    std::string model_path_;
    std::string window_name_;
    int input_size_ = 640;
    int num_classes_ = 80;
    double conf_thres_ = 0.45;
    double iou_thres_ = 0.45;
    bool use_custom_classes_ = false;
    bool show_fps_ = true;
    bool show_depth_ = true;
    double fps_smooth_ = 0.0;
    
    // 深度可视化参数
    bool show_depth_window_ = true;
    int depth_colormap_ = 2;  // cv::COLORMAP_JET
    double depth_vis_min_ = 0.1;
    double depth_vis_max_ = 5.0;
};

}  // namespace rc26_perception

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rc26_perception::VisualizationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
