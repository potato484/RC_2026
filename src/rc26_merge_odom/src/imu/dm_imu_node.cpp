// RC2026 达妙IMU节点 - C++版本，输出话题DM_IMU
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <deque>
#include <memory>
#include <vector>

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

double medianOf(std::vector<double> values) {
    if (values.empty()) {
        return 0.0;
    }

    const auto mid = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
    std::nth_element(values.begin(), mid, values.end());
    double median = *mid;
    if (values.size() % 2 == 0) {
        const auto lower_mid = std::max_element(values.begin(), mid);
        median = (*lower_mid + *mid) * 0.5;
    }
    return median;
}

std::size_t sanitizeHampelWindowSize(int window_size) {
    if (window_size < 3) {
        return 3;
    }
    if (window_size % 2 == 0) {
        return static_cast<std::size_t>(window_size + 1);
    }
    return static_cast<std::size_t>(window_size);
}

class HampelFilter {
public:
    void configure(std::size_t window_size, double nsigma, double epsilon) {
        window_size_ = std::max<std::size_t>(3, window_size);
        nsigma_ = nsigma;
        epsilon_ = epsilon;
        history_.clear();
    }

    double filter(double sample) {
        history_.push_back(sample);
        if (history_.size() > window_size_) {
            history_.pop_front();
        }

        if (history_.size() < window_size_) {
            return sample;
        }

        std::vector<double> window(history_.begin(), history_.end());
        const double median = medianOf(window);

        std::vector<double> deviations;
        deviations.reserve(window.size());
        for (double value : window) {
            deviations.push_back(std::abs(value - median));
        }
        const double mad = medianOf(std::move(deviations));

        if (mad > epsilon_ && std::abs(sample - median) > nsigma_ * mad) {
            return median;
        }
        return sample;
    }

private:
    std::size_t window_size_ = 5;
    double nsigma_ = 3.0;
    double epsilon_ = 1e-9;
    std::deque<double> history_;
};

}  // namespace

class DmImuNode : public rclcpp::Node {
public:
    DmImuNode() : Node("dm_imu_node") {
        this->declare_parameter("port", "/dev/ttyACM0");
        this->declare_parameter("baudrate", 921600);
        this->declare_parameter("frame_id", "imu_link");
        this->declare_parameter("verbose", false);
        this->declare_parameter("qos_reliable", true);
        this->declare_parameter("hampel_enable", true);
        this->declare_parameter("hampel_window_size", 5);
        this->declare_parameter("hampel_nsigma", 3.0);

        port_ = this->get_parameter("port").as_string();
        baudrate_ = this->get_parameter("baudrate").as_int();
        frame_id_ = this->get_parameter("frame_id").as_string();
        verbose_ = this->get_parameter("verbose").as_bool();
        bool qos_reliable = this->get_parameter("qos_reliable").as_bool();
        hampel_enable_ = this->get_parameter("hampel_enable").as_bool();
        hampel_window_size_ = static_cast<int>(this->get_parameter("hampel_window_size").as_int());
        hampel_nsigma_ = this->get_parameter("hampel_nsigma").as_double();

        const std::size_t sanitized_window_size = sanitizeHampelWindowSize(hampel_window_size_);
        if (sanitized_window_size != static_cast<std::size_t>(hampel_window_size_)) {
            RCLCPP_WARN(this->get_logger(), "hampel_window_size=%d invalid, adjusted to %zu", hampel_window_size_,
                        sanitized_window_size);
            hampel_window_size_ = static_cast<int>(sanitized_window_size);
        }
        if (hampel_nsigma_ <= 0.0) {
            RCLCPP_WARN(this->get_logger(), "hampel_nsigma=%.3f invalid, fallback to 3.0", hampel_nsigma_);
            hampel_nsigma_ = 3.0;
        }

        constexpr double hampel_epsilon = 1e-9;
        hampel_ax_.configure(sanitized_window_size, hampel_nsigma_, hampel_epsilon);
        hampel_ay_.configure(sanitized_window_size, hampel_nsigma_, hampel_epsilon);
        hampel_gz_.configure(sanitized_window_size, hampel_nsigma_, hampel_epsilon);

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
            const double gyro_x = data.gyro[0] * M_PI / 180.0;
            const double gyro_y = data.gyro[1] * M_PI / 180.0;
            double gyro_z = data.gyro[2] * M_PI / 180.0;
            if (hampel_enable_) {
                gyro_z = hampel_gz_.filter(gyro_z);
            }
            msg.angular_velocity.x = gyro_x;
            msg.angular_velocity.y = gyro_y;
            msg.angular_velocity.z = gyro_z;
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
            double accel_x = data.accel[0];
            double accel_y = data.accel[1];
            const double accel_z = data.accel[2];
            if (hampel_enable_) {
                accel_x = hampel_ax_.filter(accel_x);
                accel_y = hampel_ay_.filter(accel_y);
            }
            msg.linear_acceleration.x = accel_x;
            msg.linear_acceleration.y = accel_y;
            msg.linear_acceleration.z = accel_z;
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
    bool hampel_enable_ = true;
    int hampel_window_size_ = 5;
    double hampel_nsigma_ = 3.0;

    HampelFilter hampel_ax_;
    HampelFilter hampel_ay_;
    HampelFilter hampel_gz_;

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
