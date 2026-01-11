// RC2026 达妙IMU节点 - C++版本，输出话题DM_IMU
#include <cmath>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "rc26_merge_odom/imu/dm_imu_driver.hpp"

namespace {

std::array<double, 4> eulerToQuaternion(double roll, double pitch, double yaw) {
    double cy = std::cos(yaw * 0.5);
    double sy = std::sin(yaw * 0.5);
    double cp = std::cos(pitch * 0.5);
    double sp = std::sin(pitch * 0.5);
    double cr = std::cos(roll * 0.5);
    double sr = std::sin(roll * 0.5);

    double qw = cr * cp * cy + sr * sp * sy;
    double qx = sr * cp * cy - cr * sp * sy;
    double qy = cr * sp * cy + sr * cp * sy;
    double qz = cr * cp * sy - sr * sp * cy;

    return {qx, qy, qz, qw};
}

void normalizeQuaternion(double& qx, double& qy, double& qz, double& qw) {
    double n = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
    if (n < 1e-6) {
        qx = 0.0;
        qy = 0.0;
        qz = 0.0;
        qw = 1.0;
    } else {
        qx /= n;
        qy /= n;
        qz /= n;
        qw /= n;
    }
}

}  // namespace

class DmImuNode : public rclcpp::Node {
public:
    DmImuNode() : Node("dm_imu_node") {
        this->declare_parameter("port", "/dev/ttyACM0");
        this->declare_parameter("baudrate", 921600);
        this->declare_parameter("frame_id", "imu_link");
        this->declare_parameter("verbose", false);
        this->declare_parameter("qos_reliable", true);

        port_ = this->get_parameter("port").as_string();
        baudrate_ = this->get_parameter("baudrate").as_int();
        frame_id_ = this->get_parameter("frame_id").as_string();
        verbose_ = this->get_parameter("verbose").as_bool();
        bool qos_reliable = this->get_parameter("qos_reliable").as_bool();

        rclcpp::QoS qos(50);
        if (qos_reliable) {
            qos.reliable();
        } else {
            qos.best_effort();
        }

        pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("DM_IMU", qos);

        if (!driver_.open(port_, baudrate_)) {
            RCLCPP_FATAL(this->get_logger(), "Init serial failed: %s", driver_.lastError().c_str());
            throw std::runtime_error("Failed to open serial port");
        }

        RCLCPP_INFO(this->get_logger(), "Opened serial %s @ %d", port_.c_str(), baudrate_);

        timer_pub_ = this->create_wall_timer(std::chrono::milliseconds(5), std::bind(&DmImuNode::onTimerPublish, this));

        timer_stats_ = this->create_wall_timer(std::chrono::seconds(2), std::bind(&DmImuNode::onTimerStats, this));
    }

    ~DmImuNode() { driver_.close(); }

private:
    void onTimerPublish() {
        rc26_merge_odom::ImuData data = driver_.getLatestData();

        if (!data.euler_valid) {
            ++no_frame_ticks_;
            if (no_frame_ticks_ % 200 == 0 && verbose_) {
                RCLCPP_WARN(this->get_logger(), "No RPY frames yet from serial");
            }
            return;
        }

        // 检查是否有新数据
        if (data.euler_ts <= last_euler_ts_ && data.gyro_ts <= last_gyro_ts_ && data.accel_ts <= last_accel_ts_) {
            return;
        }

        last_euler_ts_ = data.euler_ts;
        last_gyro_ts_ = data.gyro_ts;
        last_accel_ts_ = data.accel_ts;

        // 角度转弧度
        double roll = data.euler[0] * M_PI / 180.0;
        double pitch = data.euler[1] * M_PI / 180.0;
        double yaw = data.euler[2] * M_PI / 180.0;

        auto [qx, qy, qz, qw] = eulerToQuaternion(roll, pitch, yaw);

        if (!std::isfinite(qx) || !std::isfinite(qy) || !std::isfinite(qz) || !std::isfinite(qw)) {
            qx = 0.0;
            qy = 0.0;
            qz = 0.0;
            qw = 1.0;
        } else {
            normalizeQuaternion(qx, qy, qz, qw);
        }

        auto msg = sensor_msgs::msg::Imu();
        msg.header.stamp = this->now();
        msg.header.frame_id = frame_id_;

        // Orientation
        msg.orientation.x = qx;
        msg.orientation.y = qy;
        msg.orientation.z = qz;
        msg.orientation.w = qw;
        msg.orientation_covariance[0] = 0.02;
        msg.orientation_covariance[4] = 0.02;
        msg.orientation_covariance[8] = 0.02;

        // Angular velocity (deg/s -> rad/s)
        if (data.gyro_valid) {
            msg.angular_velocity.x = data.gyro[0] * M_PI / 180.0;
            msg.angular_velocity.y = data.gyro[1] * M_PI / 180.0;
            msg.angular_velocity.z = data.gyro[2] * M_PI / 180.0;
            msg.angular_velocity_covariance[0] = 0.01;
            msg.angular_velocity_covariance[4] = 0.01;
            msg.angular_velocity_covariance[8] = 0.01;
        } else {
            for (int i = 0; i < 9; ++i) {
                msg.angular_velocity_covariance[i] = -1.0;
            }
        }

        // Linear acceleration (already in m/s²)
        if (data.accel_valid) {
            msg.linear_acceleration.x = data.accel[0];
            msg.linear_acceleration.y = data.accel[1];
            msg.linear_acceleration.z = data.accel[2];
            msg.linear_acceleration_covariance[0] = 0.1;
            msg.linear_acceleration_covariance[4] = 0.1;
            msg.linear_acceleration_covariance[8] = 0.1;
        } else {
            for (int i = 0; i < 9; ++i) {
                msg.linear_acceleration_covariance[i] = -1.0;
            }
        }

        pub_imu_->publish(msg);

        ++pub_count_;
        no_frame_ticks_ = 0;

        if (verbose_) {
            RCLCPP_INFO(this->get_logger(),
                        "#%u RPY=(%.2f, %.2f, %.2f) Gyro=(%.2f, %.2f, %.2f) Accel=(%.2f, %.2f, %.2f)", pub_count_,
                        data.euler[0], data.euler[1], data.euler[2], data.gyro[0], data.gyro[1], data.gyro[2],
                        data.accel[0], data.accel[1], data.accel[2]);
        }
    }

    void onTimerStats() {
        if (verbose_) {
            auto stats = driver_.getStats();
            RCLCPP_INFO(this->get_logger(), "[stats] ok=%u crc=%u short=%u nohdr=%u", stats.frames_ok,
                        stats.frames_crc_error, stats.frames_short, stats.frames_no_header);
        }
    }

    std::string port_;
    int baudrate_;
    std::string frame_id_;
    bool verbose_;

    rc26_merge_odom::DmImuDriver driver_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
    rclcpp::TimerBase::SharedPtr timer_pub_;
    rclcpp::TimerBase::SharedPtr timer_stats_;

    double last_euler_ts_ = 0.0;
    double last_gyro_ts_ = 0.0;
    double last_accel_ts_ = 0.0;
    uint32_t no_frame_ticks_ = 0;
    uint32_t pub_count_ = 0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<DmImuNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("dm_imu_node"), "Exception: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}
