#include "rotate_in_place.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <string>

#include "rc26_decision/decision_failure.hpp"

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
//       订阅 mc_odom_topic，默认使用雷达标准 /odom。
// 未配置 target_yaw_rad 时沿用累计 yaw 增量的相对旋转；配置后切到绝对 yaw 对齐。
// 注意：/odom 是自动导航链的雷达里程计契约；底盘执行默认由 rc26_mcu_transport 消费 /cmd_vel。
// 返回 RUNNING 后由 onRunning 接管持续控制。
BT::NodeStatus RotateInPlaceAction::onStart() {
    // 获取 ROS 节点指针
    if (!config().blackboard->get("node", node_) || node_ == nullptr) {
        writeDecisionFailure(config().blackboard, "RotateInPlace", "运行上下文缺失：node 不可用");
        return BT::NodeStatus::FAILURE;
    }
    // 读取武馆区运行参数（旋转角度、速度、方向等）
    if (!config().blackboard->get("mc_params", params_)) {
        RCLCPP_ERROR(node_->get_logger(), "武馆区原地旋转: 黑板缺少 mc_params");
        writeDecisionFailure(config().blackboard, "RotateInPlace", "黑板缺少 mc_params");
        return BT::NodeStatus::FAILURE;
    }

    double requested_target_yaw = 0.0;
    absolute_target_mode_ =
        config().input_ports.find("target_yaw_rad") != config().input_ports.end();
    if (absolute_target_mode_) {
        const auto target_yaw_result = getInput("target_yaw_rad", requested_target_yaw);
        if (!target_yaw_result.has_value() || !std::isfinite(requested_target_yaw)) {
            RCLCPP_ERROR(node_->get_logger(), "武馆区原地旋转: target_yaw_rad 非法");
            writeDecisionFailure(config().blackboard, "RotateInPlace",
                                 "目标 yaw 参数 target_yaw_rad 非法");
            return BT::NodeStatus::FAILURE;
        }
        absolute_target_yaw_rad_ = normalizeAngle(requested_target_yaw);
    }

    // 将配置中的度转换为弧度；相对旋转模式继续使用这些参数，绝对 yaw 模式复用速度和容差。
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
            current_yaw_ = yaw;
            if (has_yaw_ && !absolute_target_mode_) {
                // 相邻两帧 yaw 差值（带角度归一化）累加到累计转角
                accumulated_rad_ += normalizeAngle(yaw - last_yaw_);
            }
            last_yaw_ = yaw;
            has_yaw_ = true;
            last_odom_tp_ = std::chrono::steady_clock::now();
        });

    start_time_ = node_->now();
    if (absolute_target_mode_) {
        RCLCPP_INFO(node_->get_logger(),
                    "武馆区绝对转向启动: target_yaw=%.4frad 速度=%.2frad/s min=%.2frad/s slowdown=%.1f° odom=%s",
                    absolute_target_yaw_rad_, params_.rotate_speed_radps, min_speed_radps_,
                    params_.rotate_slowdown_angle_deg, params_.odom_topic.c_str());
    } else {
        RCLCPP_INFO(node_->get_logger(),
                    "武馆区原地旋转启动: 目标=%.1f° 速度=%.2frad/s min=%.2frad/s slowdown=%.1f° odom=%s",
                    params_.rotate_angle_deg, params_.rotate_speed_radps, min_speed_radps_,
                    params_.rotate_slowdown_angle_deg, params_.odom_topic.c_str());
    }
    return BT::NodeStatus::RUNNING;
}

// onRunning: 行为树每次 tick 时调用（onStart 返回 RUNNING 之后）。
// 负责：检查超时 → 等待首个里程计到来 → 判断累计转角或绝对 yaw 是否到达目标 →
//       未到达则持续发布旋转角速度到 cmd_vel。
BT::NodeStatus RotateInPlaceAction::onRunning() {
    // 超时保护：超过配置时间则停止旋转并返回 FAILURE
    if (params_.rotate_timeout_s > 0.0 &&
        (node_->now() - start_time_).seconds() > params_.rotate_timeout_s) {
        RCLCPP_WARN(node_->get_logger(), "武馆区原地旋转超时 %.1fs", params_.rotate_timeout_s);
        writeDecisionFailure(config().blackboard, "RotateInPlace",
                             "原地旋转超时，超时_s=" +
                                 std::to_string(params_.rotate_timeout_s) +
                                 "，odom=" + params_.odom_topic +
                                 "，绝对目标模式=" +
                                 (absolute_target_mode_ ? "是" : "否") +
                                 "，当前yaw_rad=" +
                                 std::to_string(current_yaw_) +
                                 "，累计旋转_rad=" +
                                 std::to_string(accumulated_rad_));
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

    if (absolute_target_mode_) {
        const double yaw_error = normalizeAngle(absolute_target_yaw_rad_ - current_yaw_);
        const double remaining_rad = std::abs(yaw_error);

        if (remaining_rad <= tolerance_rad_) {
            publishStop();
            odom_sub_.reset();
            RCLCPP_INFO(node_->get_logger(),
                        "武馆区绝对转向完成: target_yaw=%.4frad current_yaw=%.4frad error=%.1f°",
                        absolute_target_yaw_rad_, current_yaw_, yaw_error / kDeg2Rad);
            return BT::NodeStatus::SUCCESS;
        }

        double speed = std::abs(params_.rotate_speed_radps);
        if (slowdown_rad_ > tolerance_rad_ && remaining_rad < slowdown_rad_) {
            const double ratio = std::clamp(remaining_rad / slowdown_rad_, 0.0, 1.0);
            speed = min_speed_radps_ + (speed - min_speed_radps_) * ratio;
        }

        TwistMsg msg;
        msg.angular.z = (yaw_error >= 0.0 ? 1.0 : -1.0) * speed;
        cmd_pub_->publish(msg);
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

RotateRetreatAction::RotateRetreatAction(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config) {}

BT::NodeStatus RotateRetreatAction::onStart() {
    if (!config().blackboard->get("node", node_) || node_ == nullptr) {
        writeDecisionFailure(config().blackboard, "RotateRetreat", "运行上下文缺失：node 不可用");
        return BT::NodeStatus::FAILURE;
    }
    if (!config().blackboard->get("mc_params", params_)) {
        writeDecisionFailure(config().blackboard, "RotateRetreat", "黑板缺少 mc_params");
        return BT::NodeStatus::FAILURE;
    }

    (void)getInput("retreat_x_m", retreat_x_m_);
    (void)getInput("retreat_y_m", retreat_y_m_);
    (void)getInput("cmd_vel_topic", cmd_vel_topic_);
    (void)getInput("odom_topic", odom_topic_);
    (void)getInput("max_speed_mps", max_speed_mps_);
    (void)getInput("min_speed_mps", min_speed_mps_);
    (void)getInput("xy_kp", xy_kp_);
    (void)getInput("heading_kp", heading_kp_);
    (void)getInput("heading_max_speed_radps", heading_max_speed_radps_);
    (void)getInput("xy_tolerance_m", xy_tolerance_m_);
    double yaw_tolerance_deg = 3.0;
    (void)getInput("yaw_tolerance_deg", yaw_tolerance_deg);
    (void)getInput("stable_ticks", stable_ticks_required_);
    (void)getInput("odom_timeout_s", odom_timeout_s_);
    (void)getInput("timeout_s", timeout_s_);

    if (!std::isfinite(retreat_x_m_) || !std::isfinite(retreat_y_m_)) {
        return failWithStop("旋转退让距离参数非法");
    }
    if (cmd_vel_topic_.empty()) {
        cmd_vel_topic_ = "cmd_vel";
    }
    if (odom_topic_.empty()) {
        odom_topic_ = "odom";
    }
    max_speed_mps_ = std::max(0.001, std::abs(max_speed_mps_));
    min_speed_mps_ = std::min(std::max(0.0, std::abs(min_speed_mps_)), max_speed_mps_);
    xy_kp_ = std::max(0.001, xy_kp_);
    heading_kp_ = std::max(0.0, heading_kp_);
    heading_max_speed_radps_ = std::max(0.0, std::abs(heading_max_speed_radps_));
    xy_tolerance_m_ = std::max(0.001, xy_tolerance_m_);
    yaw_tolerance_rad_ = std::max(0.0, yaw_tolerance_deg) * kDeg2Rad;
    stable_ticks_required_ = std::max(1, stable_ticks_required_);
    odom_timeout_s_ = std::max(0.001, odom_timeout_s_);
    timeout_s_ = std::max(0.001, timeout_s_);

    cmd_pub_ = node_->create_publisher<TwistMsg>(cmd_vel_topic_, rclcpp::QoS(10));
    odom_sub_ = node_->create_subscription<OdomMsg>(
        odom_topic_, rclcpp::QoS(rclcpp::KeepLast(10)),
        [this](const OdomMsg::SharedPtr msg) {
            current_x_ = msg->pose.pose.position.x;
            current_y_ = msg->pose.pose.position.y;
            current_yaw_ = yawFromQuaternion(msg->pose.pose.orientation);
            has_odom_ = true;
            last_odom_tp_ = std::chrono::steady_clock::now();
        });

    has_odom_ = false;
    target_ready_ = false;
    stable_ticks_ = 0;
    start_time_ = node_->now();
    publishStop();
    RCLCPP_INFO(node_->get_logger(),
                "武馆区旋转退让复合动作启动: retreat=(%.3f, %.3f)m cmd_vel=%s odom=%s",
                retreat_x_m_, retreat_y_m_, cmd_vel_topic_.c_str(), odom_topic_.c_str());
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus RotateRetreatAction::onRunning() {
    return tickTowardTarget();
}

void RotateRetreatAction::onHalted() {
    publishStop();
    releaseRuntime();
}

void RotateRetreatAction::publishStop() {
    if (cmd_pub_) {
        cmd_pub_->publish(TwistMsg{});
    }
}

void RotateRetreatAction::releaseRuntime() {
    odom_sub_.reset();
    cmd_pub_.reset();
    node_ = nullptr;
    has_odom_ = false;
    target_ready_ = false;
    stable_ticks_ = 0;
}

bool RotateRetreatAction::odomReady() const {
    if (!has_odom_) {
        return false;
    }
    const auto age_s =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - last_odom_tp_).count();
    return age_s <= odom_timeout_s_;
}

bool RotateRetreatAction::timedOut() const {
    return node_ != nullptr && (node_->now() - start_time_).seconds() > timeout_s_;
}

bool RotateRetreatAction::prepareTargetFromCurrentOdom() {
    if (!odomReady()) {
        return false;
    }
    start_x_ = current_x_;
    start_y_ = current_y_;
    start_yaw_ = current_yaw_;
    const double target_delta =
        (params_.rotate_direction >= 0 ? 1.0 : -1.0) * std::abs(params_.rotate_angle_deg) * kDeg2Rad;
    target_yaw_ = normalizeAngle(start_yaw_ + target_delta);
    const double c = std::cos(target_yaw_);
    const double s = std::sin(target_yaw_);
    target_x_ = start_x_ + retreat_x_m_ * c - retreat_y_m_ * s;
    target_y_ = start_y_ + retreat_x_m_ * s + retreat_y_m_ * c;
    target_ready_ = true;
    RCLCPP_INFO(node_->get_logger(),
                "武馆区旋转退让目标已捕获: start=(%.3f, %.3f, %.3f) target=(%.3f, %.3f, %.3f)",
                start_x_, start_y_, start_yaw_, target_x_, target_y_, target_yaw_);
    return true;
}

BT::NodeStatus RotateRetreatAction::tickTowardTarget() {
    if (node_ == nullptr || !cmd_pub_) {
        writeDecisionFailure(config().blackboard, "RotateRetreat",
                             "运行上下文缺失：node 或 cmd_vel 发布器不可用");
        return BT::NodeStatus::FAILURE;
    }
    if (timedOut()) {
        return failWithStop("旋转退让复合动作超时");
    }
    if (!target_ready_) {
        if (!odomReady()) {
            publishStop();
            return BT::NodeStatus::RUNNING;
        }
        if (!prepareTargetFromCurrentOdom()) {
            return failWithStop("旋转退让目标捕获失败");
        }
    }
    if (!odomReady()) {
        stable_ticks_ = 0;
        publishStop();
        return BT::NodeStatus::RUNNING;
    }

    const double error_world_x = target_x_ - current_x_;
    const double error_world_y = target_y_ - current_y_;
    const double distance = std::hypot(error_world_x, error_world_y);
    const double yaw_error = normalizeAngle(target_yaw_ - current_yaw_);

    if (distance <= xy_tolerance_m_ && std::abs(yaw_error) <= yaw_tolerance_rad_) {
        ++stable_ticks_;
        publishStop();
        if (stable_ticks_ >= stable_ticks_required_) {
            RCLCPP_INFO(node_->get_logger(),
                        "武馆区旋转退让复合动作完成: remaining=%.3fm yaw_error=%.1f°",
                        distance, yaw_error / kDeg2Rad);
            releaseRuntime();
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

    stable_ticks_ = 0;
    TwistMsg cmd;
    if (distance > xy_tolerance_m_) {
        const double c = std::cos(current_yaw_);
        const double s = std::sin(current_yaw_);
        double speed_x = xy_kp_ * (error_world_x * c + error_world_y * s);
        double speed_y = xy_kp_ * (-error_world_x * s + error_world_y * c);
        const double speed_norm = std::hypot(speed_x, speed_y);
        if (speed_norm > max_speed_mps_ && speed_norm > 1e-9) {
            const double scale = max_speed_mps_ / speed_norm;
            speed_x *= scale;
            speed_y *= scale;
        } else if (speed_norm < min_speed_mps_ && speed_norm > 1e-9) {
            const double scale = min_speed_mps_ / speed_norm;
            speed_x *= scale;
            speed_y *= scale;
        }
        cmd.linear.x = speed_x;
        cmd.linear.y = speed_y;
    }
    cmd.angular.z =
        std::clamp(heading_kp_ * yaw_error, -heading_max_speed_radps_, heading_max_speed_radps_);
    cmd_pub_->publish(cmd);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus RotateRetreatAction::failWithStop(const std::string& reason) {
    const std::string detail =
        reason + "，cmd_vel=" + cmd_vel_topic_ + "，odom=" + odom_topic_ +
        "，retreat_x_m=" + std::to_string(retreat_x_m_) +
        "，retreat_y_m=" + std::to_string(retreat_y_m_) +
        "，timeout_s=" + std::to_string(timeout_s_);
    if (node_) {
        RCLCPP_WARN(node_->get_logger(), "武馆区旋转退让复合动作失败: %s", detail.c_str());
    }
    writeDecisionFailure(config().blackboard, "RotateRetreat", detail);
    publishStop();
    releaseRuntime();
    return BT::NodeStatus::FAILURE;
}

}  // namespace rc26_decision
