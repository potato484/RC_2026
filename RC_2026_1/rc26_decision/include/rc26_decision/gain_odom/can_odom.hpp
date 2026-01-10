// RC2026 CAN里程计模块
// 从CAN总线读取四驱麦轮电机转速，计算并发布轮式里程计
#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace rc26_decision
{

// CAN电调反馈报文格式 (标识符 = 0x200 + 电调ID)
// DATA[0]: 转子机械角度高8位
// DATA[1]: 转子机械角度低8位 (范围0~8191对应0~360°)
// DATA[2]: 转子转速高8位
// DATA[3]: 转子转速低8位 (单位: RPM)
// DATA[4]: 实际转矩电流高8位
// DATA[5]: 实际转矩电流低8位
// DATA[6]: 电机温度
// DATA[7]: Null

constexpr uint32_t CAN_BASE_ID = 0x200;

// 轮子ID定义 (数组索引)
enum WheelID : uint8_t
{
    FRONT_LEFT  = 0,  // 左前 - 电调ID 1
    REAR_LEFT   = 1,  // 左后 - 电调ID 2
    REAR_RIGHT  = 2,  // 右后 - 电调ID 3
    FRONT_RIGHT = 3,  // 右前 - 电调ID 4
    WHEEL_COUNT = 4
};

struct MotorFeedback
{
    uint16_t angle_raw = 0;      // 原始角度值 (0~8191)
    int16_t rpm = 0;             // 转速 (RPM，有符号)
    int16_t current = 0;         // 转矩电流
    uint8_t temperature = 0;     // 电机温度
    std::chrono::steady_clock::time_point last_update;
};

class CanOdom
{
public:
    struct Config
    {
        // CAN接口
        std::string can_interface = "can0";

        // 麦轮底盘物理参数 (单位: 米)
        double wheel_radius = 0.07625;      // 轮子半径
        double wheel_base = 0.62326;          // 轴距 (前后轮中心距)
        double track_width = 0.7;         // 轮距 (左右轮中心距)

        // 电机减速比 (3591/187)
        double gear_ratio = 3591.0 / 187.0;

        // 里程计发布频率
        int publish_rate_hz = 50;

        // ROS话题和坐标系
        std::string odom_topic = "wheel_odom";
        std::string odom_frame = "odom";
        std::string base_frame = "base_link";
    };

    CanOdom(rclcpp::Node& node, Config config);
    ~CanOdom();

    CanOdom(const CanOdom&) = delete;
    CanOdom& operator=(const CanOdom&) = delete;

    // 获取当前位姿
    void getPose(double& x, double& y, double& yaw) const;

    // 重置里程计
    void reset();

private:
    rclcpp::Node& node_;
    Config config_;

    // CAN套接字
    int can_socket_ = -1;

    // 电机反馈数据
    std::array<MotorFeedback, WHEEL_COUNT> motor_feedback_;
    mutable std::mutex feedback_mutex_;

    // 位姿状态 (积分得到)
    double x_ = 0.0;
    double y_ = 0.0;
    double yaw_ = 0.0;
    mutable std::mutex pose_mutex_;

    // 上次更新时间
    std::chrono::steady_clock::time_point last_update_time_;

    // 线程控制
    std::atomic<bool> running_{false};
    std::thread can_thread_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

    // ROS发布器
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    // 初始化CAN接口
    bool initCan();
    void closeCan();

    // CAN接收线程
    void canThreadFunc();

    // 解析CAN帧
    void parseCanFrame(uint32_t can_id, const uint8_t* data, uint8_t len);

    // 计算并发布里程计
    void publishOdometry();

    // 麦轮逆运动学: 从轮速计算底盘速度
    void wheelSpeedsToBodyVelocity(
        double v_fl, double v_rl, double v_rr, double v_fr,
        double& vx, double& vy, double& omega) const;
};

}  // namespace rc26_decision
