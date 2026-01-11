// RC26 D455 相机驱动节点
// 发布彩色图像 + 对齐深度图像

#include <atomic>
#include <thread>

#include <cv_bridge/cv_bridge.h>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace rc26_perception {

class D455DriverNode : public rclcpp::Node {
public:
    D455DriverNode() : Node("d455_driver_node"), running_(true) {
        // 基本参数
        this->declare_parameter<int>("width", 640);
        this->declare_parameter<int>("height", 480);
        this->declare_parameter<int>("fps", 30);
        this->declare_parameter<std::string>("color_topic", "/camera/color/image_raw");
        this->declare_parameter<std::string>("depth_topic", "/camera/aligned_depth_to_color/image_raw");
        this->declare_parameter<std::string>("frame_id", "d455_color_optical_frame");

        // 流控制
        this->declare_parameter<bool>("enable_color", true);
        this->declare_parameter<bool>("enable_depth", true);
        this->declare_parameter<bool>("align_depth_to_color", true);

        // 深度参数
        this->declare_parameter<double>("depth_min", 0.1);   // 最小深度 (米)
        this->declare_parameter<double>("depth_max", 10.0);  // 最大深度 (米)
        this->declare_parameter<int>("depth_preset", 0);     // 深度预设 (0=默认, 1=高精度, 2=高密度)

        // 曝光参数
        this->declare_parameter<bool>("enable_auto_exposure", true);
        this->declare_parameter<int>("exposure_us", 8000);  // 手动曝光时间 (微秒)
        this->declare_parameter<int>("gain", 16);           // 增益

        // 激光参数 (D455 红外投射器)
        this->declare_parameter<bool>("enable_laser", true);
        this->declare_parameter<int>("laser_power", 150);  // 激光功率 (0-360)

        // 获取基本参数
        width_ = this->get_parameter("width").as_int();
        height_ = this->get_parameter("height").as_int();
        fps_ = this->get_parameter("fps").as_int();
        if (width_ <= 0 || height_ <= 0) {
            RCLCPP_WARN(this->get_logger(), "Invalid resolution %dx%d, fallback to 640x480", width_, height_);
            width_ = 640;
            height_ = 480;
        }
        if (fps_ <= 0 || fps_ > 120) {
            RCLCPP_WARN(this->get_logger(), "Invalid fps=%d, fallback to 30", fps_);
            fps_ = 30;
        }
        color_topic_ = this->get_parameter("color_topic").as_string();
        depth_topic_ = this->get_parameter("depth_topic").as_string();
        frame_id_ = this->get_parameter("frame_id").as_string();

        // 获取流控制参数
        enable_color_ = this->get_parameter("enable_color").as_bool();
        enable_depth_ = this->get_parameter("enable_depth").as_bool();
        align_depth_ = this->get_parameter("align_depth_to_color").as_bool();

        // 获取深度参数
        depth_min_ = this->get_parameter("depth_min").as_double();
        depth_max_ = this->get_parameter("depth_max").as_double();
        depth_preset_ = this->get_parameter("depth_preset").as_int();

        if (depth_min_ < 0.0) {
            depth_min_ = 0.0;
        }
        if (depth_max_ < depth_min_) {
            RCLCPP_WARN(this->get_logger(), "Invalid depth range %.2f-%.2f, fallback to 0.1-10.0", depth_min_,
                        depth_max_);
            depth_min_ = 0.1;
            depth_max_ = 10.0;
        }
        if (depth_max_ > 65.535) {
            depth_max_ = 65.535;
        }

        // 获取曝光参数
        enable_auto_exposure_ = this->get_parameter("enable_auto_exposure").as_bool();
        exposure_us_ = this->get_parameter("exposure_us").as_int();
        gain_ = this->get_parameter("gain").as_int();

        // 获取激光参数
        enable_laser_ = this->get_parameter("enable_laser").as_bool();
        laser_power_ = this->get_parameter("laser_power").as_int();

        // 创建发布者
        if (enable_color_) {
            color_pub_ = this->create_publisher<sensor_msgs::msg::Image>(color_topic_, rclcpp::SensorDataQoS());
        }
        if (enable_depth_) {
            depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>(depth_topic_, rclcpp::SensorDataQoS());
        }

        // 启动 RealSense
        try {
            rs2::config cfg;

            if (enable_color_) {
                cfg.enable_stream(RS2_STREAM_COLOR, width_, height_, RS2_FORMAT_BGR8, fps_);
            }
            if (enable_depth_) {
                cfg.enable_stream(RS2_STREAM_DEPTH, width_, height_, RS2_FORMAT_Z16, fps_);
            }

            rs2::pipeline_profile profile = pipe_.start(cfg);

            // 配置设备参数
            configureDevice(profile);

            RCLCPP_INFO(this->get_logger(), "RealSense D455 started: %dx%d@%dfps", width_, height_, fps_);
            RCLCPP_INFO(this->get_logger(), "  Color: %s, Depth: %s, Align: %s", enable_color_ ? "ON" : "OFF",
                        enable_depth_ ? "ON" : "OFF", align_depth_ ? "ON" : "OFF");
            if (enable_color_) {
                RCLCPP_INFO(this->get_logger(), "  Color topic: %s", color_topic_.c_str());
            }
            if (enable_depth_) {
                RCLCPP_INFO(this->get_logger(), "  Depth topic: %s", depth_topic_.c_str());
                RCLCPP_INFO(this->get_logger(), "  Depth range: %.2f - %.2f m", depth_min_, depth_max_);
            }
        } catch (const rs2::error& e) {
            RCLCPP_FATAL(this->get_logger(), "RealSense start failed: %s", e.what());
            throw;
        }

        // 创建对齐器（将深度对齐到彩色）
        if (align_depth_ && enable_color_ && enable_depth_) {
            align_ = std::make_unique<rs2::align>(RS2_STREAM_COLOR);
        }

        // 采集发布线程
        capture_thread_ = std::thread([this] { this->captureLoop(); });
    }

    ~D455DriverNode() override {
        running_ = false;
        if (capture_thread_.joinable()) {
            capture_thread_.join();
        }
        try {
            pipe_.stop();
        } catch (...) {}
    }

private:
    void configureDevice(const rs2::pipeline_profile& profile) {
        auto device = profile.get_device();

        // 配置深度传感器
        for (auto& sensor : device.query_sensors()) {
            // 深度传感器设置
            if (sensor.is<rs2::depth_sensor>()) {
                auto depth_sensor = sensor.as<rs2::depth_sensor>();

                // 设置深度预设
                if (sensor.supports(RS2_OPTION_VISUAL_PRESET)) {
                    try {
                        // 0=Custom, 1=Default, 2=Hand, 3=HighAccuracy, 4=HighDensity, 5=MediumDensity
                        int preset = 1;  // Default
                        if (depth_preset_ == 1)
                            preset = 3;  // High Accuracy
                        else if (depth_preset_ == 2)
                            preset = 4;  // High Density
                        sensor.set_option(RS2_OPTION_VISUAL_PRESET, preset);
                        RCLCPP_INFO(this->get_logger(), "  Depth preset: %d", preset);
                    } catch (...) {}
                }

                // 设置激光
                if (sensor.supports(RS2_OPTION_EMITTER_ENABLED)) {
                    try {
                        sensor.set_option(RS2_OPTION_EMITTER_ENABLED, enable_laser_ ? 1 : 0);
                    } catch (...) {}
                }
                if (enable_laser_ && sensor.supports(RS2_OPTION_LASER_POWER)) {
                    try {
                        sensor.set_option(RS2_OPTION_LASER_POWER, laser_power_);
                        RCLCPP_INFO(this->get_logger(), "  Laser power: %d", laser_power_);
                    } catch (...) {}
                }

                // 获取深度单位
                if (sensor.supports(RS2_OPTION_DEPTH_UNITS)) {
                    depth_scale_ = depth_sensor.get_depth_scale();
                    RCLCPP_INFO(this->get_logger(), "  Depth scale: %.6f m", depth_scale_);
                }
            }

            // 彩色传感器设置
            if (sensor.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE)) {
                try {
                    sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, enable_auto_exposure_ ? 1 : 0);
                    RCLCPP_INFO(this->get_logger(), "  Auto exposure: %s", enable_auto_exposure_ ? "ON" : "OFF");
                } catch (...) {}
            }

            if (!enable_auto_exposure_) {
                if (sensor.supports(RS2_OPTION_EXPOSURE)) {
                    try {
                        sensor.set_option(RS2_OPTION_EXPOSURE, exposure_us_);
                        RCLCPP_INFO(this->get_logger(), "  Exposure: %d us", exposure_us_);
                    } catch (...) {}
                }
                if (sensor.supports(RS2_OPTION_GAIN)) {
                    try {
                        sensor.set_option(RS2_OPTION_GAIN, gain_);
                        RCLCPP_INFO(this->get_logger(), "  Gain: %d", gain_);
                    } catch (...) {}
                }
            }
        }
    }

    void captureLoop() {
        rclcpp::Rate rate(fps_);

        // 深度范围过滤值 (毫米)
        uint16_t depth_min_mm = static_cast<uint16_t>(depth_min_ * 1000);
        uint16_t depth_max_mm = static_cast<uint16_t>(depth_max_ * 1000);

        while (rclcpp::ok() && running_) {
            try {
                rs2::frameset fs = pipe_.wait_for_frames(1000);

                // 对齐深度到彩色（如果启用）
                rs2::frameset processed_fs = fs;
                if (align_) {
                    processed_fs = align_->process(fs);
                }

                rs2::video_frame color_frame = processed_fs.get_color_frame();
                rs2::depth_frame depth_frame = processed_fs.get_depth_frame();

                bool has_color = enable_color_ && color_frame;
                bool has_depth = enable_depth_ && depth_frame;

                if (!has_color && !has_depth) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "No frames available");
                    rate.sleep();
                    continue;
                }

                // 获取当前时间戳（两个消息使用相同时间戳以便同步）
                auto stamp = this->now();

                // 发布彩色图像
                if (has_color && color_pub_) {
                    cv::Mat img(cv::Size(color_frame.get_width(), color_frame.get_height()), CV_8UC3,
                                (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
                    cv::Mat img_copy = img.clone();
                    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::BGR8, img_copy)
                                   .toImageMsg();
                    msg->header.stamp = stamp;
                    msg->header.frame_id = frame_id_;
                    color_pub_->publish(*msg);
                }

                // 发布深度图像（16UC1，单位毫米）
                if (has_depth && depth_pub_) {
                    cv::Mat depth_img(cv::Size(depth_frame.get_width(), depth_frame.get_height()), CV_16UC1,
                                      (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);
                    cv::Mat depth_copy = depth_img.clone();

                    // 应用深度范围过滤
                    if (depth_min_mm > 0 || depth_max_mm < 65535) {
                        cv::Mat mask = (depth_copy < depth_min_mm) | (depth_copy > depth_max_mm);
                        depth_copy.setTo(0, mask);
                    }

                    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::TYPE_16UC1,
                                                  depth_copy)
                                   .toImageMsg();
                    msg->header.stamp = stamp;
                    msg->header.frame_id = frame_id_;
                    depth_pub_->publish(*msg);
                }

            } catch (const rs2::error& e) {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "RealSense error: %s", e.what());
            } catch (const std::exception& e) {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Publish exception: %s", e.what());
            }
            rate.sleep();
        }
    }

    // RealSense
    rs2::pipeline pipe_;
    std::unique_ptr<rs2::align> align_;

    // ROS Publishers
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;

    // 线程控制
    std::atomic<bool> running_;
    std::thread capture_thread_;

    // 基本参数
    int width_{640};
    int height_{480};
    int fps_{30};
    std::string color_topic_;
    std::string depth_topic_;
    std::string frame_id_;

    // 流控制
    bool enable_color_{true};
    bool enable_depth_{true};
    bool align_depth_{true};

    // 深度参数
    double depth_min_{0.1};
    double depth_max_{10.0};
    int depth_preset_{0};
    double depth_scale_{0.001};

    // 曝光参数
    bool enable_auto_exposure_{true};
    int exposure_us_{8000};
    int gain_{16};

    // 激光参数
    bool enable_laser_{true};
    int laser_power_{150};
};

}  // namespace rc26_perception

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<rc26_perception::D455DriverNode>());
    } catch (const std::exception& e) {
        fprintf(stderr, "Fatal: %s\n", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
