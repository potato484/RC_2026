#pragma once

#include <optional>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/buffer_core.h"

namespace rc26_terrain {

enum class TfChainStatusCode {
    kOk = 0,
    kInvalidSpecification,
    kMissingRequiredTransform,
    kLegacyFrameDetected,
    kUnexpectedStaticExtrinsic,
    kStaleTransform,
};

struct TfPoseTolerance {
    double position_m{1e-6};
    double yaw_rad{1e-6};
};

struct TfChainSpec {
    std::string map_frame{"map"};
    std::string odom_frame{"odom"};
    std::string base_frame{"base_link"};
    std::string sensor_frame{"livox_frame"};
    std::optional<std::string> forbidden_frame{"laser_link"};
    std::optional<tf2::Transform> expected_base_to_sensor{};
    std::optional<double> max_dynamic_age_sec{};
    TfPoseTolerance tolerance{};
};

struct TfValidationReport {
    bool ok{false};
    TfChainStatusCode code{TfChainStatusCode::kInvalidSpecification};
    std::string message;
    std::vector<std::string> available_frames;
    std::vector<std::string> missing_links;
    std::optional<geometry_msgs::msg::TransformStamped> map_to_odom;
    std::optional<geometry_msgs::msg::TransformStamped> odom_to_base;
    std::optional<geometry_msgs::msg::TransformStamped> map_to_base;
    std::optional<geometry_msgs::msg::TransformStamped> base_to_sensor;
    std::optional<geometry_msgs::msg::TransformStamped> map_to_sensor;
};

class TfChainValidator {
public:
    explicit TfChainValidator(TfChainSpec spec);

    TfValidationReport validate(
        const tf2::BufferCore& buffer, const tf2::TimePoint& lookup_time,
        std::optional<tf2::TimePoint> freshness_reference_time = std::nullopt) const;

private:
    static bool containsFrame(const std::vector<std::string>& frames, const std::string& frame);
    static double stampToSec(const builtin_interfaces::msg::Time& stamp);
    static double timePointToSec(const tf2::TimePoint& time_point);
    static double yawErrorRad(const geometry_msgs::msg::TransformStamped& actual, const tf2::Transform& expected);
    static double positionErrorM(const geometry_msgs::msg::TransformStamped& actual, const tf2::Transform& expected);
    static TfValidationReport makeFailure(
        TfChainStatusCode code, std::string message, std::vector<std::string> available_frames,
        std::vector<std::string> missing_links = {});

    bool isSpecValid(std::string* reason) const;
    bool isTransformFresh(
        const geometry_msgs::msg::TransformStamped& transform, const tf2::TimePoint& reference_time,
        double max_age_sec) const;

    TfChainSpec spec_;
};

}  // namespace rc26_terrain
