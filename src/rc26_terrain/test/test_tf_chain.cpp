#include <cmath>
#include <string>

#include "gtest/gtest.h"

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/buffer_core.h"
#include "tf2/time.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace {

builtin_interfaces::msg::Time makeStamp(int32_t sec, uint32_t nanosec = 0U) {
    builtin_interfaces::msg::Time t;
    t.sec = sec;
    t.nanosec = nanosec;
    return t;
}

geometry_msgs::msg::TransformStamped makeTf(const std::string& parent, const std::string& child, const tf2::Transform& tf,
                                            const builtin_interfaces::msg::Time& stamp) {
    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = parent;
    msg.child_frame_id = child;
    msg.transform = tf2::toMsg(tf);
    return msg;
}

tf2::Transform makePlanar(double x, double y, double z, double yaw_rad) {
    tf2::Transform tf;
    tf.setOrigin(tf2::Vector3(x, y, z));
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_rad);
    tf.setRotation(q);
    return tf;
}

void expectTfNear(const geometry_msgs::msg::TransformStamped& actual, const tf2::Transform& expected,
                  double pos_eps = 1e-9, double yaw_eps = 1e-9) {
    tf2::Transform actual_tf;
    tf2::fromMsg(actual.transform, actual_tf);

    const auto da = actual_tf.getOrigin() - expected.getOrigin();
    EXPECT_NEAR(da.x(), 0.0, pos_eps);
    EXPECT_NEAR(da.y(), 0.0, pos_eps);
    EXPECT_NEAR(da.z(), 0.0, pos_eps);

    double ar = 0.0, ap = 0.0, ay = 0.0;
    double er = 0.0, ep = 0.0, ey = 0.0;
    tf2::Matrix3x3(actual_tf.getRotation()).getRPY(ar, ap, ay);
    tf2::Matrix3x3(expected.getRotation()).getRPY(er, ep, ey);

    // Only care about yaw for this test. Wrap to [-pi, pi].
    auto wrap = [](double a) {
        while (a > M_PI) a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    };
    EXPECT_NEAR(wrap(ay - ey), 0.0, yaw_eps);
}

}  // namespace

TEST(TfChainUnitTest, TopologyConnectedAndComposable) {
    tf2::BufferCore buffer(tf2::durationFromSec(10.0));

    // Static extrinsic: base_link -> livox_frame (pose of livox_frame in base_link).
    const tf2::Transform tf_base_livox = makePlanar(0.0, 0.0, 0.13, 0.0);
    ASSERT_TRUE(buffer.setTransform(makeTf("base_link", "livox_frame", tf_base_livox, makeStamp(0)), "test", true));

    // Dynamic: odom -> base_link.
    const tf2::Transform tf_odom_base = makePlanar(1.0, 2.0, 0.0, 0.1);
    ASSERT_TRUE(buffer.setTransform(makeTf("odom", "base_link", tf_odom_base, makeStamp(10)), "test", false));

    // Dynamic: map -> odom.
    const tf2::Transform tf_map_odom = makePlanar(5.0, -3.0, 0.0, -0.2);
    ASSERT_TRUE(buffer.setTransform(makeTf("map", "odom", tf_map_odom, makeStamp(10)), "test", false));

    const tf2::TimePoint t = tf2::timeFromSec(10.0);

    std::string err;
    EXPECT_TRUE(buffer.canTransform("map", "base_link", t, &err)) << err;
    EXPECT_TRUE(buffer.canTransform("map", "livox_frame", t, &err)) << err;

    // Validate composition: T_map_base = T_map_odom * T_odom_base.
    const tf2::Transform expected_map_base = tf_map_odom * tf_odom_base;
    const auto actual_map_base = buffer.lookupTransform("map", "base_link", t);
    expectTfNear(actual_map_base, expected_map_base, 1e-9, 1e-9);

    // Validate composition through static extrinsic: T_map_livox = T_map_base * T_base_livox.
    const tf2::Transform expected_map_livox = expected_map_base * tf_base_livox;
    const auto actual_map_livox = buffer.lookupTransform("map", "livox_frame", t);
    expectTfNear(actual_map_livox, expected_map_livox, 1e-9, 1e-9);
}

TEST(TfChainUnitTest, MissingStaticExtrinsicBreaksChain) {
    tf2::BufferCore buffer(tf2::durationFromSec(10.0));

    ASSERT_TRUE(buffer.setTransform(makeTf("odom", "base_link", makePlanar(0.0, 0.0, 0.0, 0.0), makeStamp(10)),
                                    "test", false));
    ASSERT_TRUE(
        buffer.setTransform(makeTf("map", "odom", makePlanar(0.0, 0.0, 0.0, 0.0), makeStamp(10)), "test", false));

    const tf2::TimePoint t = tf2::timeFromSec(10.0);
    std::string err;
    EXPECT_FALSE(buffer.canTransform("map", "livox_frame", t, &err));
    EXPECT_FALSE(err.empty());
}

