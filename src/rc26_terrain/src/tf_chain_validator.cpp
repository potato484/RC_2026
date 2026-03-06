#include "rc26_terrain/tf_chain_validator.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <sstream>
#include <utility>

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace rc26_terrain {

namespace {

std::string joinStrings(const std::vector<std::string>& items, const std::string& delimiter) {
    std::ostringstream oss;
    for (size_t index = 0; index < items.size(); ++index) {
        if (index != 0U) {
            oss << delimiter;
        }
        oss << items[index];
    }
    return oss.str();
}

}  // namespace

TfChainValidator::TfChainValidator(TfChainSpec spec) : spec_(std::move(spec)) {}

TfValidationReport TfChainValidator::validate(
    const tf2::BufferCore& buffer, const tf2::TimePoint& lookup_time,
    std::optional<tf2::TimePoint> freshness_reference_time) const {
    std::string spec_error;
    auto available_frames = buffer.getAllFrameNames();
    std::sort(available_frames.begin(), available_frames.end());

    if (!isSpecValid(&spec_error)) {
        return makeFailure(TfChainStatusCode::kInvalidSpecification, std::move(spec_error), std::move(available_frames));
    }

    if (spec_.forbidden_frame && containsFrame(available_frames, *spec_.forbidden_frame)) {
        return makeFailure(
            TfChainStatusCode::kLegacyFrameDetected,
            "检测到禁用帧 '" + *spec_.forbidden_frame + "'，TF 树中存在遗留链路",
            std::move(available_frames));
    }

    struct RequiredLink {
        std::string target;
        std::string source;
        std::string description;
    };

    const std::array<RequiredLink, 5> required_links{{
        {spec_.map_frame, spec_.odom_frame, spec_.map_frame + " -> " + spec_.odom_frame},
        {spec_.odom_frame, spec_.base_frame, spec_.odom_frame + " -> " + spec_.base_frame},
        {spec_.base_frame, spec_.sensor_frame, spec_.base_frame + " -> " + spec_.sensor_frame},
        {spec_.map_frame, spec_.base_frame, spec_.map_frame + " -> " + spec_.base_frame},
        {spec_.map_frame, spec_.sensor_frame, spec_.map_frame + " -> " + spec_.sensor_frame},
    }};

    std::vector<std::string> missing_links;
    for (const auto& link : required_links) {
        std::string error;
        if (!buffer.canTransform(link.target, link.source, lookup_time, &error)) {
            if (!error.empty()) {
                missing_links.emplace_back(link.description + " [" + error + "]");
            } else {
                missing_links.emplace_back(link.description);
            }
        }
    }

    if (!missing_links.empty()) {
        return makeFailure(
            TfChainStatusCode::kMissingRequiredTransform,
            "关键 TF 链路不可用: " + joinStrings(missing_links, "; "),
            std::move(available_frames), std::move(missing_links));
    }

    TfValidationReport report;
    report.ok = true;
    report.code = TfChainStatusCode::kOk;
    report.message = "TF 链路验证通过";
    report.available_frames = available_frames;

    try {
        report.map_to_odom = buffer.lookupTransform(spec_.map_frame, spec_.odom_frame, lookup_time);
        report.odom_to_base = buffer.lookupTransform(spec_.odom_frame, spec_.base_frame, lookup_time);
        report.map_to_base = buffer.lookupTransform(spec_.map_frame, spec_.base_frame, lookup_time);
        report.base_to_sensor = buffer.lookupTransform(spec_.base_frame, spec_.sensor_frame, lookup_time);
        report.map_to_sensor = buffer.lookupTransform(spec_.map_frame, spec_.sensor_frame, lookup_time);
    } catch (const tf2::TransformException& ex) {
        return makeFailure(
            TfChainStatusCode::kMissingRequiredTransform,
            std::string("TF 查询在验证阶段失败: ") + ex.what(), std::move(available_frames));
    }

    if (spec_.expected_base_to_sensor) {
        const double pos_error = positionErrorM(*report.base_to_sensor, *spec_.expected_base_to_sensor);
        const double yaw_error = yawErrorRad(*report.base_to_sensor, *spec_.expected_base_to_sensor);
        if (pos_error > spec_.tolerance.position_m || yaw_error > spec_.tolerance.yaw_rad) {
            std::ostringstream oss;
            oss << "静态外参异常: position_error=" << pos_error << "m, yaw_error=" << yaw_error << "rad";
            return makeFailure(
                TfChainStatusCode::kUnexpectedStaticExtrinsic,
                oss.str(), std::move(available_frames));
        }
    }

    if (spec_.max_dynamic_age_sec) {
        const tf2::TimePoint reference_time = freshness_reference_time.value_or(lookup_time);
        if (reference_time != tf2::TimePointZero) {
            try {
                const auto latest_map_to_base = buffer.lookupTransform(spec_.map_frame, spec_.base_frame, tf2::TimePointZero);
                const auto latest_map_to_sensor = buffer.lookupTransform(spec_.map_frame, spec_.sensor_frame, tf2::TimePointZero);
                if (!isTransformFresh(latest_map_to_base, reference_time, *spec_.max_dynamic_age_sec) ||
                    !isTransformFresh(latest_map_to_sensor, reference_time, *spec_.max_dynamic_age_sec)) {
                    std::ostringstream oss;
                    oss << "动态 TF 超时: max_age=" << *spec_.max_dynamic_age_sec << "s";
                    return makeFailure(
                        TfChainStatusCode::kStaleTransform,
                        oss.str(), std::move(available_frames));
                }
            } catch (const tf2::TransformException& ex) {
                return makeFailure(
                    TfChainStatusCode::kMissingRequiredTransform,
                    std::string("TF 新鲜度检查失败: ") + ex.what(), std::move(available_frames));
            }
        }
    }

    return report;
}

bool TfChainValidator::containsFrame(const std::vector<std::string>& frames, const std::string& frame) {
    return std::find(frames.begin(), frames.end(), frame) != frames.end();
}

double TfChainValidator::stampToSec(const builtin_interfaces::msg::Time& stamp) {
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

double TfChainValidator::timePointToSec(const tf2::TimePoint& time_point) {
    return std::chrono::duration_cast<std::chrono::duration<double>>(time_point.time_since_epoch()).count();
}

double TfChainValidator::yawErrorRad(
    const geometry_msgs::msg::TransformStamped& actual, const tf2::Transform& expected) {
    tf2::Transform actual_tf;
    tf2::fromMsg(actual.transform, actual_tf);

    double actual_roll = 0.0;
    double actual_pitch = 0.0;
    double actual_yaw = 0.0;
    double expected_roll = 0.0;
    double expected_pitch = 0.0;
    double expected_yaw = 0.0;

    tf2::Matrix3x3(actual_tf.getRotation()).getRPY(actual_roll, actual_pitch, actual_yaw);
    tf2::Matrix3x3(expected.getRotation()).getRPY(expected_roll, expected_pitch, expected_yaw);

    return std::abs(std::remainder(actual_yaw - expected_yaw, 2.0 * M_PI));
}

double TfChainValidator::positionErrorM(
    const geometry_msgs::msg::TransformStamped& actual, const tf2::Transform& expected) {
    tf2::Transform actual_tf;
    tf2::fromMsg(actual.transform, actual_tf);

    const auto delta = actual_tf.getOrigin() - expected.getOrigin();
    return delta.length();
}

TfValidationReport TfChainValidator::makeFailure(
    TfChainStatusCode code, std::string message, std::vector<std::string> available_frames,
    std::vector<std::string> missing_links) {
    TfValidationReport report;
    report.ok = false;
    report.code = code;
    report.message = std::move(message);
    report.available_frames = std::move(available_frames);
    report.missing_links = std::move(missing_links);
    return report;
}

bool TfChainValidator::isSpecValid(std::string* reason) const {
    const std::array<std::pair<const char*, const std::string*>, 4> required_frames{{
        {"map_frame", &spec_.map_frame},
        {"odom_frame", &spec_.odom_frame},
        {"base_frame", &spec_.base_frame},
        {"sensor_frame", &spec_.sensor_frame},
    }};

    for (const auto& [name, value] : required_frames) {
        if (value->empty()) {
            if (reason != nullptr) {
                *reason = std::string(name) + " 不能为空";
            }
            return false;
        }
    }

    if (spec_.map_frame == spec_.odom_frame || spec_.map_frame == spec_.base_frame ||
        spec_.map_frame == spec_.sensor_frame || spec_.odom_frame == spec_.base_frame ||
        spec_.odom_frame == spec_.sensor_frame || spec_.base_frame == spec_.sensor_frame) {
        if (reason != nullptr) {
            *reason = "TF 规格非法: 必需帧名称不能重复";
        }
        return false;
    }

    if (spec_.forbidden_frame && spec_.forbidden_frame->empty()) {
        if (reason != nullptr) {
            *reason = "forbidden_frame 不能为空字符串";
        }
        return false;
    }

    if (spec_.forbidden_frame &&
        (*spec_.forbidden_frame == spec_.map_frame || *spec_.forbidden_frame == spec_.odom_frame ||
         *spec_.forbidden_frame == spec_.base_frame || *spec_.forbidden_frame == spec_.sensor_frame)) {
        if (reason != nullptr) {
            *reason = "forbidden_frame 与必需帧冲突";
        }
        return false;
    }

    if (spec_.max_dynamic_age_sec && *spec_.max_dynamic_age_sec < 0.0) {
        if (reason != nullptr) {
            *reason = "max_dynamic_age_sec 不能为负数";
        }
        return false;
    }

    if (spec_.tolerance.position_m < 0.0 || spec_.tolerance.yaw_rad < 0.0) {
        if (reason != nullptr) {
            *reason = "姿态容差不能为负数";
        }
        return false;
    }

    return true;
}

bool TfChainValidator::isTransformFresh(
    const geometry_msgs::msg::TransformStamped& transform, const tf2::TimePoint& reference_time,
    double max_age_sec) const {
    const double transform_sec = stampToSec(transform.header.stamp);
    const double reference_sec = timePointToSec(reference_time);
    return (reference_sec - transform_sec) <= max_age_sec;
}

}  // namespace rc26_terrain
