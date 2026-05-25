#include "rc26_vision/postprocess/tip_localizer.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rc26_vision/inference/engine_factory.hpp"
#include "rc26_vision/inference/model_profile.hpp"
#include "rc26_vision/shared/depth_roi_sampler.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace rc26_vision {

namespace {

ModelProfile makeTipLocalizerProfile(const std::string& model_path,
                                     const std::vector<std::string>& class_names,
                                     float conf_thresh,
                                     float iou_thresh) {
    ModelProfile profile;
    profile.id = "tip_localizer_default";
    profile.engine = EngineType::Auto;
    profile.model_path = model_path;
    profile.labels = class_names;
    profile.conf_thresh = conf_thresh;
    profile.iou_thresh = iou_thresh;
    return profile;
}

}  // namespace

TipLocalizer::TipLocalizer(const rclcpp::NodeOptions& options)
    : rclcpp::Node("tip_localizer", options) {
    this->declare_parameter<std::string>("color_topic", "/camera/color/image_raw");
    this->declare_parameter<std::string>("depth_topic", "/camera/aligned_depth_to_color/image_raw");
    this->declare_parameter<std::string>("camera_info_topic", "/camera/color/camera_info");
    this->declare_parameter<std::string>("output_topic", "/vision/tip_detections");
    this->declare_parameter<double>("spacing_expected_m", spacing_expected_m_);
    this->declare_parameter<double>("spacing_tolerance_m", spacing_tolerance_m_);
    this->declare_parameter<double>("max_depth_m", max_depth_m_);
    this->declare_parameter<double>("conf_thresh", 0.25);
    this->declare_parameter<double>("iou_thresh", 0.45);
    this->declare_parameter<std::string>("model_path", "");
    this->declare_parameter<std::vector<std::string>>(
        "class_names", std::vector<std::string>{"tip0", "tip1", "tip2", "tip3", "tip4", "tip5"});

    this->get_parameter("spacing_expected_m", spacing_expected_m_);
    this->get_parameter("spacing_tolerance_m", spacing_tolerance_m_);
    this->get_parameter("max_depth_m", max_depth_m_);

    std::string model_path;
    this->get_parameter("model_path", model_path);
    if (model_path.empty()) {
        const auto share = ament_index_cpp::get_package_share_directory("rc26_vision");
        model_path = share + "/models/kfs.onnx";
    }

    std::vector<std::string> class_names;
    this->get_parameter("class_names", class_names);
    const auto conf_thresh = static_cast<float>(this->get_parameter("conf_thresh").as_double());
    const auto iou_thresh = static_cast<float>(this->get_parameter("iou_thresh").as_double());

    engine_ = createInferenceEngine(
        makeTipLocalizerProfile(model_path, class_names, conf_thresh, iou_thresh));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    const auto color_topic = this->get_parameter("color_topic").as_string();
    const auto depth_topic = this->get_parameter("depth_topic").as_string();
    const auto info_topic = this->get_parameter("camera_info_topic").as_string();
    const auto output_topic = this->get_parameter("output_topic").as_string();

    color_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        color_topic, rclcpp::SensorDataQoS(),
        std::bind(&TipLocalizer::onColor, this, std::placeholders::_1));
    depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        depth_topic, rclcpp::SensorDataQoS(),
        std::bind(&TipLocalizer::onDepth, this, std::placeholders::_1));
    info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        info_topic, rclcpp::SensorDataQoS(),
        std::bind(&TipLocalizer::onCameraInfo, this, std::placeholders::_1));

    detections_pub_ = this->create_publisher<rc26_interfaces::msg::TipDetectionArray>(
        output_topic, rclcpp::QoS(10).reliable());
}

void TipLocalizer::onDepth(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    if (!msg) {
        return;
    }
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    latest_depth_ = msg;
}

void TipLocalizer::onCameraInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg) {
    if (!msg) {
        return;
    }

    std::lock_guard<std::mutex> lock(sensor_mutex_);
    intrinsics_.fx = msg->k[0];
    intrinsics_.fy = msg->k[4];
    intrinsics_.cx = msg->k[2];
    intrinsics_.cy = msg->k[5];
    intrinsics_.dist = msg->d;
    intrinsics_.ready = intrinsics_.fx > 0.0 && intrinsics_.fy > 0.0;
}

std::optional<geometry_msgs::msg::PointStamped> TipLocalizer::toMapFrame(
    const geometry_msgs::msg::PointStamped& pt_camera) {
    try {
        const auto tf = tf_buffer_->lookupTransform(
            "map", pt_camera.header.frame_id, pt_camera.header.stamp, rclcpp::Duration::from_seconds(0.05));
        geometry_msgs::msg::PointStamped pt_map;
        tf2::doTransform(pt_camera, pt_map, tf);
        return pt_map;
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "tip_localizer TF error: %s", ex.what());
        return std::nullopt;
    }
}

bool TipLocalizer::validateSpacing(std::vector<rc26_interfaces::msg::TipDetection>& detections) const {
    std::vector<std::pair<uint8_t, geometry_msgs::msg::PointStamped>> indexed;
    indexed.reserve(detections.size());
    for (const auto& det : detections) {
        if (det.tip_index <= 5) {
            indexed.emplace_back(det.tip_index, det.position_map);
        }
    }

    if (indexed.size() < 2) {
        for (auto& det : detections) det.spacing_valid = true;
        return true;
    }

    std::sort(indexed.begin(), indexed.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });

    for (size_t i = 1; i < indexed.size(); ++i) {
        const int idx_gap = static_cast<int>(indexed[i].first) - static_cast<int>(indexed[i - 1].first);
        if (idx_gap <= 0) {
            continue;
        }
        const auto& p0 = indexed[i - 1].second.point;
        const auto& p1 = indexed[i].second.point;
        const double dist_xy = std::hypot(p1.x - p0.x, p1.y - p0.y);
        const double expected = spacing_expected_m_ * static_cast<double>(idx_gap);
        if (std::abs(dist_xy - expected) > spacing_tolerance_m_) {
            for (auto& det : detections) det.spacing_valid = false;
            return false;
        }
    }

    for (auto& det : detections) det.spacing_valid = true;
    return true;
}

void TipLocalizer::onColor(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    if (!msg || !engine_) {
        return;
    }

    sensor_msgs::msg::Image::ConstSharedPtr depth_msg;
    CameraIntrinsics intrinsics;
    {
        std::lock_guard<std::mutex> lock(sensor_mutex_);
        depth_msg = latest_depth_;
        intrinsics = intrinsics_;
    }

    if (!depth_msg || !intrinsics.ready) {
        return;
    }

    cv::Mat color;
    cv::Mat depth;
    try {
        color = cv_bridge::toCvCopy(msg, "bgr8")->image;
        depth = cv_bridge::toCvShare(depth_msg)->image;
    } catch (const cv_bridge::Exception&) {
        return;
    }

    const auto detections = engine_->infer(color);

    rc26_interfaces::msg::TipDetectionArray out;
    out.header = msg->header;
    out.header.frame_id = "map";
    out.detections.reserve(detections.size());

    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
        intrinsics.fx, 0.0, intrinsics.cx,
        0.0, intrinsics.fy, intrinsics.cy,
        0.0, 0.0, 1.0);
    cv::Mat dist_coeffs(intrinsics.dist);

    DepthRoiSamplerConfig depth_sampler_config;
    depth_sampler_config.roi_size = 5;
    depth_sampler_config.min_valid_count = 1;
    depth_sampler_config.max_depth_m = max_depth_m_;

    for (const auto& det : detections) {
        const int u = static_cast<int>((det.x1 + det.x2) * 0.5f);
        const int v = static_cast<int>((det.y1 + det.y2) * 0.5f);
        const auto depth_m = sampleMedianDepth(depth, u, v, depth_sampler_config);
        if (!depth_m) {
            continue;
        }

        std::vector<cv::Point2f> src_px{cv::Point2f(static_cast<float>(u), static_cast<float>(v))};
        std::vector<cv::Point2f> undistorted;
        cv::undistortPoints(src_px, undistorted, camera_matrix, dist_coeffs);
        if (undistorted.empty()) {
            continue;
        }

        geometry_msgs::msg::PointStamped pt_camera;
        pt_camera.header = msg->header;
        pt_camera.point.x = static_cast<double>(undistorted[0].x) * (*depth_m);
        pt_camera.point.y = static_cast<double>(undistorted[0].y) * (*depth_m);
        pt_camera.point.z = *depth_m;

        const auto pt_map = toMapFrame(pt_camera);
        if (!pt_map) {
            continue;
        }

        rc26_interfaces::msg::TipDetection tip;
        tip.tip_index = (det.class_id >= 0 && det.class_id <= 5)
            ? static_cast<uint8_t>(det.class_id)
            : static_cast<uint8_t>(255);
        tip.position_map = *pt_map;
        tip.confidence = det.score;
        tip.depth_m = static_cast<float>(*depth_m);
        tip.spacing_valid = true;
        out.detections.push_back(std::move(tip));
    }

    if (!validateSpacing(out.detections)) {
        out.detections.clear();
    }

    detections_pub_->publish(out);
}

}  // namespace rc26_vision
