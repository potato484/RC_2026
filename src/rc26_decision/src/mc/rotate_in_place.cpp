#include "rotate_in_place.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace rc26_decision {

namespace {
constexpr double kDeg2Rad = M_PI / 180.0;

// 从四元数提取偏航角 (yaw)
double yawFromQuaternion(const geometry_msgs::msg::Quaternion& q) {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

// 将角度规整到 [-π, π] 范围
double normalizeAngle(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}
}  // namespace

RotateInPlaceAction::RotateInPlaceAction(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config), start_time_(0, 0, RCL_ROS_TIME) {}

// onStart: 行为树首次进入本动作时调用一次。
// 负责：从黑板读取 mc_params → 计算目标角度和容差 → 创建 cmd_vel 发布器 →
//       订阅 mc_odom_topic，默认使用雷达标准 /odom → 通过 yaw 增量积分跟踪累计转角。
// 注意：/odom 是自动导航链的雷达里程计契约，merge_odom 仍只负责底盘执行和局部反馈链。
// 返回 RUNNING 后由 onRunning 接管持续控制。
BT::NodeStatus RotateInPlaceAction::onStart() {
    // 获取 ROS 节点指针
    if (!config().blackboard->get("node", node_) || node_ == nullptr) {
        return BT::NodeStatus::FAILURE;
    }
    // 读取武馆区运行参数（旋转角度、速度、方向等）
    if (!config().blackboard->get("mc_params", params_)) {
        RCLCPP_ERROR(node_->get_logger(), "武馆区原地旋转: 黑板缺少 mc_params");
        return BT::NodeStatus::FAILURE;
    }

    // 将配置中的度转换为弧度
    target_rad_ = std::abs(params_.rotate_angle_deg) * kDeg2Rad;
    signed_target_rad_ = (params_.rotate_direction >= 0 ? 1.0 : -1.0) * target_rad_;
    tolerance_rad_ = std::abs(params_.rotate_yaw_tolerance_deg) * kDeg2Rad;
    min_speed_radps_ =
        std::min(std::abs(params_.rotate_min_speed_radps), std::abs(params_.rotate_speed_radps));
    slowdown_rad_ = std::abs(params_.rotate_slowdown_angle_deg) * kDeg2Rad;
    accumulated_rad_ = 0.0;
    has_yaw_ = false;
    last_odom_tp_ = {};

    // 创建 cmd_vel 发布器，用于向底盘下发旋转角速度
    cmd_pub_ = node_->create_publisher<TwistMsg>(params_.rotate_cmd_vel_topic, rclcpp::QoS(10));
    // 订阅里程计话题，通过相邻帧 yaw 差值的增量积分计算已转过的角度
    odom_sub_ = node_->create_subscription<OdomMsg>(
        params_.odom_topic, rclcpp::QoS(rclcpp::KeepLast(10)),
        [this](const OdomMsg::SharedPtr msg) {
            const double yaw = yawFromQuaternion(msg->pose.pose.orientation);
            if (has_yaw_) {
                // 相邻两帧 yaw 差值（带角度归一化）累加到累计转角
                accumulated_rad_ += normalizeAngle(yaw - last_yaw_);
            }
            last_yaw_ = yaw;
            has_yaw_ = true;
            last_odom_tp_ = std::chrono::steady_clock::now();
        });

    start_time_ = node_->now();
    RCLCPP_INFO(node_->get_logger(),
                "武馆区原地旋转启动: 目标=%.1f° 速度=%.2frad/s min=%.2frad/s slowdown=%.1f° odom=%s",
                params_.rotate_angle_deg, params_.rotate_speed_radps, min_speed_radps_,
                params_.rotate_slowdown_angle_deg, params_.odom_topic.c_str());
    return BT::NodeStatus::RUNNING;
}

// onRunning: 行为树每次 tick 时调用（onStart 返回 RUNNING 之后）。
// 负责：检查超时 → 等待首个里程计到来 → 判断累计转角是否到达目标 →
//       未到达则持续发布旋转角速度到 cmd_vel。
BT::NodeStatus RotateInPlaceAction::onRunning() {
    // 超时保护：超过配置时间则停止旋转并返回 FAILURE
    if (params_.rotate_timeout_s > 0.0 &&
        (node_->now() - start_time_).seconds() > params_.rotate_timeout_s) {
        RCLCPP_WARN(node_->get_logger(), "武馆区原地旋转超时 %.1fs", params_.rotate_timeout_s);
        publishStop();
        odom_sub_.reset();
        return BT::NodeStatus::FAILURE;
    }

    // 尚未收到首个里程计消息，继续等待
    if (!has_yaw_) {
        return BT::NodeStatus::RUNNING;
    }

    const double odom_age_s =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - last_odom_tp_).count();
    if (odom_age_s > params_.rotate_odom_timeout_s) {
        publishStop();
        return BT::NodeStatus::RUNNING;
    }

    const double direction = signed_target_rad_ >= 0.0 ? 1.0 : -1.0;
    const double remaining_rad = direction * (signed_target_rad_ - accumulated_rad_);

    // 累计转角达到目标角度（按旋转方向判断剩余角度），旋转完成
    if (remaining_rad <= tolerance_rad_) {
        publishStop();
        odom_sub_.reset();
        RCLCPP_INFO(node_->get_logger(), "武馆区原地旋转完成: 累计=%.1f° 剩余=%.1f°",
                    accumulated_rad_ / kDeg2Rad, remaining_rad / kDeg2Rad);
        return BT::NodeStatus::SUCCESS;
    }

    double speed = std::abs(params_.rotate_speed_radps);
    if (slowdown_rad_ > tolerance_rad_ && remaining_rad < slowdown_rad_) {
        const double ratio = std::clamp(remaining_rad / slowdown_rad_, 0.0, 1.0);
        speed = min_speed_radps_ + (speed - min_speed_radps_) * ratio;
    }

    // 未完成：持续发布旋转角速度（方向由 mc_rotate_direction 控制）
    TwistMsg msg;
    msg.angular.z = direction * speed;
    cmd_pub_->publish(msg);
    return BT::NodeStatus::RUNNING;
}

// onHalted: 行为树被外部中断时调用（如 Sequence 中前序节点失败、树被 halt 等）。
// 负责：发送零速停止机器人旋转，释放里程计订阅，保证安全停机。
void RotateInPlaceAction::onHalted() {
    publishStop();
    odom_sub_.reset();
}

// 发布空 Twist 消息停止旋转
void RotateInPlaceAction::publishStop() {
    if (cmd_pub_) {
        cmd_pub_->publish(TwistMsg{});
    }
}

}  // namespace rc26_decision
