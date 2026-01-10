// RC2026 CAN里程计实现
// 从CAN总线读取四驱麦轮电机转速，计算并发布轮式里程计
#include "rc26_decision/gain_odom/can_odom.hpp"

#include <tf2/LinearMath/Quaternion.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cmath>
#include <cstring>

namespace rc26_decision
{

CanOdom::CanOdom(rclcpp::Node& node, Config config)
    : node_(node), config_(std::move(config))
{
    odom_pub_ = node_.create_publisher<nav_msgs::msg::Odometry>(config_.odom_topic, 10);

    last_update_time_ = std::chrono::steady_clock::now();

    if (!initCan())
    {
        RCLCPP_ERROR(node_.get_logger(), "CAN 接口初始化失败: %s", config_.can_interface.c_str());
        return;
    }

    running_ = true;
    can_thread_ = std::thread(&CanOdom::canThreadFunc, this);

    auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / static_cast<double>(config_.publish_rate_hz)));
    publish_timer_ = node_.create_wall_timer(period, std::bind(&CanOdom::publishOdometry, this));

    RCLCPP_INFO(
        node_.get_logger(),
        "CAN 里程计启动: interface=%s, odom_topic=%s, rate=%d Hz",
        config_.can_interface.c_str(),
        config_.odom_topic.c_str(),
        config_.publish_rate_hz);
}

CanOdom::~CanOdom()
{
    running_ = false;
    if (can_thread_.joinable())
    {
        can_thread_.join();
    }
    closeCan();
}

bool CanOdom::initCan()
{
    can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket_ < 0)
    {
        RCLCPP_ERROR(node_.get_logger(), "创建 CAN 套接字失败: %s", strerror(errno));
        return false;
    }

    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, config_.can_interface.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';

    if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0)
    {
        RCLCPP_ERROR(
            node_.get_logger(),
            "获取 CAN 接口索引失败: %s, %s",
            config_.can_interface.c_str(),
            strerror(errno));
        close(can_socket_);
        can_socket_ = -1;
        return false;
    }

    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(can_socket_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0)
    {
        RCLCPP_ERROR(node_.get_logger(), "绑定 CAN 套接字失败: %s", strerror(errno));
        close(can_socket_);
        can_socket_ = -1;
        return false;
    }

    // 设置CAN过滤器，只接收电调反馈报文 (0x201-0x204)
    struct can_filter filters[WHEEL_COUNT];
    for (int i = 0; i < WHEEL_COUNT; ++i)
    {
        filters[i].can_id = CAN_BASE_ID + i + 1;
        filters[i].can_mask = CAN_SFF_MASK;
    }
    setsockopt(can_socket_, SOL_CAN_RAW, CAN_RAW_FILTER, &filters, sizeof(filters));

    // 设置接收超时
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 50000;  // 50ms
    setsockopt(can_socket_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    return true;
}

void CanOdom::closeCan()
{
    if (can_socket_ >= 0)
    {
        close(can_socket_);
        can_socket_ = -1;
    }
}

void CanOdom::canThreadFunc()
{
    RCLCPP_DEBUG(node_.get_logger(), "CAN 接收线程启动");

    struct can_frame frame;

    while (running_)
    {
        ssize_t nbytes = read(can_socket_, &frame, sizeof(frame));
        if (nbytes < 0)
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
            {
                continue;
            }
            RCLCPP_WARN(node_.get_logger(), "CAN 读取错误: %s", strerror(errno));
            continue;
        }

        if (nbytes == sizeof(frame))
        {
            parseCanFrame(frame.can_id, frame.data, frame.can_dlc);
        }
    }

    RCLCPP_DEBUG(node_.get_logger(), "CAN 接收线程退出");
}

void CanOdom::parseCanFrame(uint32_t can_id, const uint8_t* data, uint8_t len)
{
    if (len < 8)
    {
        return;
    }

    // 电调ID = can_id - 0x200 (1~4)
    uint32_t motor_id = can_id - CAN_BASE_ID;
    if (motor_id < 1 || motor_id > WHEEL_COUNT)
    {
        return;
    }

    // 转换为数组索引 (0~3)
    uint8_t idx = static_cast<uint8_t>(motor_id - 1);

    // 解析数据
    uint16_t angle_raw = (static_cast<uint16_t>(data[0]) << 8) | data[1];
    int16_t rpm = static_cast<int16_t>((static_cast<uint16_t>(data[2]) << 8) | data[3]);
    int16_t current = static_cast<int16_t>((static_cast<uint16_t>(data[4]) << 8) | data[5]);
    uint8_t temperature = data[6];

    {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        motor_feedback_[idx].angle_raw = angle_raw;
        motor_feedback_[idx].rpm = rpm;
        motor_feedback_[idx].current = current;
        motor_feedback_[idx].temperature = temperature;
        motor_feedback_[idx].last_update = std::chrono::steady_clock::now();
    }
}

void CanOdom::wheelSpeedsToBodyVelocity(
    double v_fl, double v_rl, double v_rr, double v_fr,
    double& vx, double& vy, double& omega) const
{
    // 麦克纳姆轮逆运动学 (从轮速计算底盘速度)
    // 轮子布局 (俯视图):
    //   FL ---- FR
    //    |      |
    //    |      |
    //   RL ---- RR
    //
    // 麦轮滚子角度: 前左/后右为+45°，前右/后左为-45°
    //
    // 正运动学:
    // v_fl = vx - vy - (L+W)*omega
    // v_fr = vx + vy + (L+W)*omega
    // v_rl = vx + vy - (L+W)*omega
    // v_rr = vx - vy + (L+W)*omega
    //
    // 逆运动学 (最小二乘解):
    // vx = (v_fl + v_fr + v_rl + v_rr) / 4
    // vy = (-v_fl + v_fr + v_rl - v_rr) / 4
    // omega = (-v_fl + v_fr - v_rl + v_rr) / (4 * (L + W))

    double l_plus_w = (config_.wheel_base + config_.track_width) / 2.0;

    vx = (v_fl + v_fr + v_rl + v_rr) / 4.0;
    vy = (-v_fl + v_fr + v_rl - v_rr) / 4.0;
    omega = (-v_fl + v_fr - v_rl + v_rr) / (4.0 * l_plus_w);
}

void CanOdom::publishOdometry()
{
    auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now - last_update_time_).count();
    last_update_time_ = now;

    if (dt <= 0.0 || dt > 1.0)
    {
        return;
    }

    // 获取各轮转速 (RPM)
    std::array<int16_t, WHEEL_COUNT> rpm_values;
    {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        for (int i = 0; i < WHEEL_COUNT; ++i)
        {
            rpm_values[i] = motor_feedback_[i].rpm;
        }
    }

    // RPM -> 轮子线速度 (m/s)
    // v = rpm * 2 * pi * r / 60 / gear_ratio
    auto rpmToWheelSpeed = [this](int16_t rpm) -> double {
        return static_cast<double>(rpm) * 2.0 * M_PI * config_.wheel_radius /
               (60.0 * config_.gear_ratio);
    };

    double v_fl = rpmToWheelSpeed(rpm_values[FRONT_LEFT]);
    double v_rl = rpmToWheelSpeed(rpm_values[REAR_LEFT]);
    double v_rr = rpmToWheelSpeed(rpm_values[REAR_RIGHT]);
    double v_fr = rpmToWheelSpeed(rpm_values[FRONT_RIGHT]);

    // 计算底盘速度
    double vx, vy, omega;
    wheelSpeedsToBodyVelocity(v_fl, v_rl, v_rr, v_fr, vx, vy, omega);

    // 更新位姿 (使用中点法积分)
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);

        double half_dt = dt / 2.0;
        double mid_yaw = yaw_ + omega * half_dt;

        double cos_yaw = std::cos(mid_yaw);
        double sin_yaw = std::sin(mid_yaw);

        x_ += (vx * cos_yaw - vy * sin_yaw) * dt;
        y_ += (vx * sin_yaw + vy * cos_yaw) * dt;
        yaw_ += omega * dt;

        // 归一化 yaw 到 [-pi, pi]
        while (yaw_ > M_PI) yaw_ -= 2.0 * M_PI;
        while (yaw_ < -M_PI) yaw_ += 2.0 * M_PI;
    }

    // 发布里程计消息
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = node_.now();
    odom_msg.header.frame_id = config_.odom_frame;
    odom_msg.child_frame_id = config_.base_frame;

    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;
        odom_msg.pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw_);
        q.normalize();
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();
    }

    odom_msg.twist.twist.linear.x = vx;
    odom_msg.twist.twist.linear.y = vy;
    odom_msg.twist.twist.linear.z = 0.0;
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = omega;

    odom_pub_->publish(odom_msg);
}

void CanOdom::getPose(double& x, double& y, double& yaw) const
{
    std::lock_guard<std::mutex> lock(pose_mutex_);
    x = x_;
    y = y_;
    yaw = yaw_;
}

void CanOdom::reset()
{
    std::lock_guard<std::mutex> lock(pose_mutex_);
    x_ = 0.0;
    y_ = 0.0;
    yaw_ = 0.0;
    RCLCPP_INFO(node_.get_logger(), "里程计已重置");
}

}  // namespace rc26_decision
