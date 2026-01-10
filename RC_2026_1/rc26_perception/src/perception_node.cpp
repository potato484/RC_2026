// RC26 感知节点
// 订阅 D455 RGB-D 图像，执行 YOLO 推理，发布 BlockDetections

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "rc26_perception/yolo_engine.hpp"
#include "rc26_perception/preprocess.hpp"
#include "rc26_perception/postprocess.hpp"
#include "rc26_perception/msg/block_detection.hpp"
#include "rc26_perception/msg/block_detections.hpp"

#include <memory>
#include <vector>

namespace rc26_perception {

class PerceptionNode : public rclcpp::Node {
public:
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, sensor_msgs::msg::Image>;

    PerceptionNode() : Node("rc26_perception_node") {
        // 声明参数
        this->declare_parameter<std::string>("model_path", "");
        this->declare_parameter<int>("input_size", 640);
        this->declare_parameter<double>("conf_thres", 0.45);
        this->declare_parameter<double>("iou_thres", 0.45);
        this->declare_parameter<bool>("nchw", false);
        this->declare_parameter<int>("num_classes", 80);
        this->declare_parameter<bool>("use_custom_classes", false);
        
        // 话题参数
        this->declare_parameter<std::string>("color_topic", "/camera/color/image_raw");
        this->declare_parameter<std::string>("depth_topic", "/camera/aligned_depth_to_color/image_raw");
        this->declare_parameter<std::string>("output_topic", "/rc26/block_detections");
        
        // 相机内参 (D455 默认值，可通过参数覆盖)
        this->declare_parameter<double>("camera_fx", 386.0);
        this->declare_parameter<double>("camera_fy", 386.0);
        this->declare_parameter<double>("camera_cx", 320.0);
        this->declare_parameter<double>("camera_cy", 240.0);
        
        // TF 参数
        this->declare_parameter<std::string>("camera_frame", "d455_color_optical_frame");
        this->declare_parameter<std::string>("base_frame", "base_link");
        
        // 获取参数
        model_path_ = this->get_parameter("model_path").as_string();
        input_size_ = this->get_parameter("input_size").as_int();
        if (input_size_ <= 0 || input_size_ > 2048) {
            RCLCPP_WARN(this->get_logger(),
                "Invalid input_size=%d, fallback to 640", input_size_);
            input_size_ = 640;
        }
        conf_thres_ = this->get_parameter("conf_thres").as_double();
        iou_thres_ = this->get_parameter("iou_thres").as_double();
        nchw_ = this->get_parameter("nchw").as_bool();
        num_classes_ = this->get_parameter("num_classes").as_int();
        use_custom_classes_ = this->get_parameter("use_custom_classes").as_bool();
        
        color_topic_ = this->get_parameter("color_topic").as_string();
        depth_topic_ = this->get_parameter("depth_topic").as_string();
        output_topic_ = this->get_parameter("output_topic").as_string();
        
        camera_fx_ = this->get_parameter("camera_fx").as_double();
        camera_fy_ = this->get_parameter("camera_fy").as_double();
        camera_cx_ = this->get_parameter("camera_cx").as_double();
        camera_cy_ = this->get_parameter("camera_cy").as_double();
        
        camera_frame_ = this->get_parameter("camera_frame").as_string();
        base_frame_ = this->get_parameter("base_frame").as_string();

        // 初始化 TF
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 初始化推理引擎
        if (!model_path_.empty()) {
            YoloEngineConfig engine_cfg;
            engine_cfg.model_path = model_path_;
            engine_cfg.input_size = input_size_;
            engine_cfg.nchw = nchw_;

            engine_ = std::make_unique<YoloEngine>();
            if (!engine_->init(engine_cfg)) {
                RCLCPP_WARN(this->get_logger(), 
                    "YOLO engine init failed, running in pass-through mode");
                engine_.reset();
            }
        } else {
            RCLCPP_WARN(this->get_logger(), 
                "No model_path specified, running in pass-through mode");
        }

        // 分配输入 tensor 内存
        input_tensor_.resize((size_t)input_size_ * input_size_ * 3);

        // 使用 message_filters 同步彩色图和深度图
        color_sub_.subscribe(this, color_topic_, rmw_qos_profile_sensor_data);
        depth_sub_.subscribe(this, depth_topic_, rmw_qos_profile_sensor_data);

        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), color_sub_, depth_sub_);
        sync_->registerCallback(std::bind(&PerceptionNode::imageCallback, this,
                                          std::placeholders::_1, std::placeholders::_2));

        // 创建发布者
        detections_pub_ = this->create_publisher<msg::BlockDetections>(output_topic_, 10);

        RCLCPP_INFO(this->get_logger(), "RC26 Perception Node started");
        RCLCPP_INFO(this->get_logger(), "  Color topic: %s", color_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Depth topic: %s", depth_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Output topic: %s", output_topic_.c_str());
        if (engine_) {
            RCLCPP_INFO(this->get_logger(), "  Model: %s", model_path_.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), "  Mode: Pass-through (no inference)");
        }
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& color_msg,
                       const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg) {
        // 转换彩色图像
        cv_bridge::CvImagePtr color_ptr;
        try {
            color_ptr = cv_bridge::toCvCopy(color_msg, sensor_msgs::image_encodings::BGR8);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Color cv_bridge exception: %s", e.what());
            return;
        }
        cv::Mat color_img = color_ptr->image;
        if (color_img.empty()) return;

        // 转换深度图像
        cv_bridge::CvImagePtr depth_ptr;
        try {
            depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Depth cv_bridge exception: %s", e.what());
            return;
        }
        cv::Mat depth_img = depth_ptr->image;
        if (depth_img.empty()) return;

        // 准备输出消息
        auto out_msg = std::make_unique<msg::BlockDetections>();
        out_msg->header = color_msg->header;

        // 如果引擎可用，执行推理
        if (engine_ && engine_->isInitialized()) {
            // 预处理
            float scale = 1.f;
            cv::Mat in_img = preprocess::topleftLetterbox(color_img, input_size_, scale);
            preprocess::toBlobRgb01(in_img, input_tensor_.data(), nchw_);

            // 推理
            if (!engine_->infer(input_tensor_.data())) {
                RCLCPP_ERROR(this->get_logger(), "Inference failed");
                return;
            }

            // 获取输出
            float* box_data = nullptr;
            uint32_t box_bytes = 0;
            float* score_data = nullptr;
            uint32_t score_bytes = 0;
            if (!engine_->getOutput(box_data, box_bytes, score_data, score_bytes)) {
                RCLCPP_ERROR(this->get_logger(), "Get output failed");
                return;
            }

            // 后处理
            std::vector<DetectedObject> objs;
            postprocess::YoloPost::runCxcywh(
                box_data, box_bytes, score_data, score_bytes, objs,
                (float)conf_thres_, (float)iou_thres_,
                input_size_, color_img.cols, color_img.rows, scale,
                /*box_ch_first=*/true, /*sco_ch_first=*/true,
                num_classes_, /*max_det=*/300);

            // 填充深度值并转换为消息
            for (auto& obj : objs) {
                postprocess::fillDepth(obj, depth_img, 2);
                
                msg::BlockDetection det;
                
                // 类别名称
                if (use_custom_classes_ && obj.label < (int)kBlockClasses.size()) {
                    det.class_name = kBlockClasses[obj.label];
                } else if (obj.label < (int)kCOCO80.size()) {
                    det.class_name = kCOCO80[obj.label];
                } else {
                    det.class_name = "class_" + std::to_string(obj.label);
                }
                
                det.center_x = obj.center_x;
                det.center_y = obj.center_y;
                det.depth_m = obj.distance;
                
                out_msg->detections.push_back(det);
            }

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Published %zu detections", objs.size());
        }

        // 发布结果（即使没有检测到目标也发布空消息）
        detections_pub_->publish(std::move(out_msg));
    }

private:
    // 推理引擎
    std::unique_ptr<YoloEngine> engine_;
    std::vector<float> input_tensor_;

    // ROS message_filters
    message_filters::Subscriber<sensor_msgs::msg::Image> color_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    // ROS Publisher
    rclcpp::Publisher<msg::BlockDetections>::SharedPtr detections_pub_;

    // TF
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // 参数
    std::string model_path_;
    std::string color_topic_;
    std::string depth_topic_;
    std::string output_topic_;
    std::string camera_frame_;
    std::string base_frame_;
    int input_size_ = 640;
    int num_classes_ = 80;
    double conf_thres_ = 0.45;
    double iou_thres_ = 0.45;
    bool nchw_ = false;
    bool use_custom_classes_ = false;
    double camera_fx_ = 386.0;
    double camera_fy_ = 386.0;
    double camera_cx_ = 320.0;
    double camera_cy_ = 240.0;
};

}  // namespace rc26_perception

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<rc26_perception::PerceptionNode>());
    rclcpp::shutdown();
    return 0;
}
