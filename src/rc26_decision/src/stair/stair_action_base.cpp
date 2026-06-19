#include "rc26_decision/stair/stair_action_base.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <memory>
#include <utility>

namespace rc26_decision {

namespace {

// 台阶动作最小速度命令发布频率；避免参数为 0 时后续计算 1/rate 出现除零。
constexpr double kMinCommandRateHz = 1.0;
// 所有等待超时的最小值；避免配置为 0 或负数导致阶段立即异常结束。
constexpr double kMinTimeoutS = 0.001;

// 将内部枚举转换成日志里可读的英文标签，避免每个调用点重复写 switch。
const char *eventName(StairActionBase::WheelEvent event) {
  switch (event) {
  case StairActionBase::WheelEvent::FrontFirst:
    return "front_first";
  case StairActionBase::WheelEvent::FrontSecond:
    return "front_second";
  case StairActionBase::WheelEvent::Rear:
    return "rear";
  }
  return "unknown";
}

} // namespace

// 构造函数只初始化 BehaviorTree.CPP 基类和两个时间戳占位；ROS 资源在每次动作开始时创建。
StairActionBase::StairActionBase(const std::string &name,
                                 const BT::NodeConfig &config)
    : BT::StatefulActionNode(name, config), stage_start_(0, 0, RCL_ROS_TIME),
      last_drive_publish_(0, 0, RCL_ROS_TIME) {}

// 当前台阶节点没有 XML 端口；所有运行参数都来自黑板中的 stair_params。
BT::PortsList StairActionBase::providedPorts() { return {}; }

// setupRuntime() 是每次 StairClimb/StairDescend 进入 onStart() 后调用的公共初始化。
bool StairActionBase::setupRuntime(const char *action_label) {
  // 保存动作中文名，用于后续日志区分“上台阶”和“下台阶”。
  action_label_ = action_label ? action_label : "stair";
  // 从黑板取出 decision_node 的 ROS 节点指针；没有 node 就无法创建 publisher/client/subscription。
  if (!config().blackboard->get("node", node_) || node_ == nullptr) {
    // 不在这里打日志，是因为没有 node 时也没有 logger；由调用方返回 FAILURE。
    return false;
  }
  // 从黑板取台阶参数；这些参数由 loadStairParams() 在 decision_node 启动时写入。
  if (!config().blackboard->get("stair_params", params_)) {
    // 缺少参数说明 decision_node 初始化链不完整，动作不能安全运行。
    RCLCPP_ERROR(node_->get_logger(), "%s: 黑板缺少 stair_params",
                 action_label_.c_str());
    return false;
  }

  // 速度参数只保留绝对值；具体正负方向由上下台阶状态机显式决定。
  params_.drive_speed_mps = std::abs(params_.drive_speed_mps);
  // 发布频率做下限保护，保证 publishDrive() 的周期计算稳定。
  params_.command_rate_hz =
      std::max(kMinCommandRateHz, params_.command_rate_hz);
  // 推杆命令等待超时做下限保护，保证 tickCommand() 永远有明确超时语义。
  params_.command_timeout_s =
      std::max(kMinTimeoutS, params_.command_timeout_s);
  // 前轮激光事件等待超时做下限保护。
  params_.front_event_timeout_s =
      std::max(kMinTimeoutS, params_.front_event_timeout_s);
  // 后轮激光事件等待超时做下限保护。
  params_.rear_event_timeout_s =
      std::max(kMinTimeoutS, params_.rear_event_timeout_s);
  // 上台阶前推杆伸出后的零速等待允许为 0；小于 0 时夹到 0。
  params_.climb_front_extend_delay_s =
      std::max(0.0, params_.climb_front_extend_delay_s);
  // 上台阶组合推杆命令后的零速等待允许为 0。
  params_.climb_retract_rear_extend_delay_s =
      std::max(0.0, params_.climb_retract_rear_extend_delay_s);
  // 下台阶后推杆伸出后的零速等待允许为 0；小于 0 时夹到 0。
  params_.descend_rear_extend_delay_s =
      std::max(0.0, params_.descend_rear_extend_delay_s);
  // 下台阶组合推杆命令后的零速等待允许为 0。
  params_.descend_retract_front_extend_delay_s =
      std::max(0.0, params_.descend_retract_front_extend_delay_s);

  // 创建台阶动作自己的速度 publisher；动作结束或 halt 时会释放它。
  cmd_pub_ =
      node_->create_publisher<TwistMsg>(params_.cmd_vel_topic, rclcpp::QoS(10));
  // 创建共享串口推杆命令 service client；每条命令都通过 tickCommand() 异步发送。
  send_client_ =
      node_->create_client<SendCommandSrv>(params_.send_command_service);
  // 订阅 mechanism transport feedback；只关心 MCU 上报的三个激光高度突变事件。
  feedback_sub_ = node_->create_subscription<FeedbackMsg>(
      params_.feedback_topic, rclcpp::QoS(32).reliable(),
      [this](const FeedbackMsg::SharedPtr msg) {
        // feedback_id 是 uint8，先转成协议枚举，便于只匹配明确的台阶事件。
        const auto id = static_cast<FeedbackID>(msg->feedback_id);
        // 前轮第一个激光高度突变事件到达时，前轮第一事件计数加一。
        if (id == FeedbackID::FRONT_LASER_HEIGHT_JUMP) {
          front_first_event_count_.fetch_add(1, std::memory_order_relaxed);
        // 前轮第二个激光高度突变事件到达时，前轮第二事件计数加一。
        } else if (id == FeedbackID::FRONT_SECOND_LASER_HEIGHT_JUMP) {
          front_second_event_count_.fetch_add(1, std::memory_order_relaxed);
        // 后轮激光高度突变事件到达时，后轮事件计数加一。
        } else if (id == FeedbackID::REAR_LASER_HEIGHT_JUMP) {
          rear_event_count_.fetch_add(1, std::memory_order_relaxed);
        }
      });

  // 增加异步命令 generation，使上一轮尚未返回的 service 回调不会影响本轮动作。
  command_generation_.fetch_add(1, std::memory_order_relaxed);
  // 重置速度发布限频状态，确保进入动作后第一帧速度可以立即发出。
  has_last_drive_publish_ = false;
  // 记录当前阶段起点时间；后续每次 begin*() 会重新刷新。
  markStageStart();

  // 打印本次动作的关键运行入口，便于现场确认 topic/service/速度参数。
  RCLCPP_INFO(node_->get_logger(),
              "%s 启动: cmd_vel=%s service=%s feedback=%s speed=%.3fm/s",
              action_label_.c_str(), params_.cmd_vel_topic.c_str(),
              params_.send_command_service.c_str(),
              params_.feedback_topic.c_str(), params_.drive_speed_mps);
  // 初始化成功，具体状态机可以开始推进。
  return true;
}

// releaseRuntime() 释放本次动作生命周期内创建的 ROS 资源。
void StairActionBase::releaseRuntime() {
  // 让所有已经发出但尚未返回的 service 回调失效，防止回调写入已结束动作的状态。
  command_generation_.fetch_add(1, std::memory_order_relaxed);
  // 取消 feedback 订阅；动作结束后不再累计激光事件。
  feedback_sub_.reset();
  // 释放 service client；下一次动作重新创建。
  send_client_.reset();
  // 释放 cmd_vel publisher；释放前调用方应先 publishStop()。
  cmd_pub_.reset();
  // 清空 node_，让后续误调用工具函数时不会继续发布或计时。
  node_ = nullptr;
  // 清理命令等待状态，避免下一次动作读到旧的 accepted/rejected 标志。
  resetCommandState();
  resetCommandPairState();
}

// publishDrive() 发布一帧 x 方向速度，并按 stair_command_rate_hz 做限频。
void StairActionBase::publishDrive(double signed_speed_mps) {
  // 没有 publisher 或 node 说明动作尚未初始化或已经释放，直接忽略。
  if (!cmd_pub_ || !node_) {
    return;
  }

  // 用 ROS time 记录当前时刻，和上一帧发布时间比较。
  const auto now = node_->now();
  // 由配置频率换算最小发布周期。
  const double min_period_s = 1.0 / params_.command_rate_hz;
  // 如果上一帧刚发过且未到最小周期，本 tick 不重复发，避免刷爆 cmd_vel。
  if (has_last_drive_publish_ &&
      (now - last_drive_publish_).seconds() < min_period_s) {
    return;
  }

  // 构造 Twist，台阶动作只使用 linear.x，linear.y/angular.z 保持 0。
  TwistMsg msg;
  // signed_speed_mps 已经由调用方带正负号：上台阶为正，下台阶为负。
  msg.linear.x = signed_speed_mps;
  // 发布速度命令给底盘执行链。
  cmd_pub_->publish(msg);
  // 记录本次发布时间，供下一 tick 限频。
  last_drive_publish_ = now;
  // 标记已经发过速度，后续 tick 才需要比较时间间隔。
  has_last_drive_publish_ = true;
}

// publishStop() 发布一帧空 Twist，让底盘速度回到 0。
void StairActionBase::publishStop() {
  // 如果 publisher 已经释放，就没有可发布对象，直接返回。
  if (!cmd_pub_) {
    return;
  }
  // 空 Twist 的所有速度分量都是 0，用作安全停车命令。
  cmd_pub_->publish(TwistMsg{});
  // 清除限频状态，保证后续阶段第一帧速度或下一次停车可以立即发布。
  has_last_drive_publish_ = false;
}

// driveSpeedMagnitude() 返回参数中的正速度幅值；方向由调用方决定。
double StairActionBase::driveSpeedMagnitude() const {
  // 再取一次 abs 是防御式处理，避免未来绕过 setupRuntime() 时引入负幅值。
  return std::abs(params_.drive_speed_mps);
}

// beginCommand() 开始一个“下发推杆命令并等待 accepted”的阶段。
void StairActionBase::beginCommand(CommandID command_id, const char *label) {
  // 每个命令阶段单独切 generation，避免旧 service 回调跨阶段写入新状态。
  command_generation_.fetch_add(1, std::memory_order_relaxed);
  // 记录本阶段要发送的协议命令 ID。
  active_command_ = command_id;
  // 记录日志标签；没有传标签时使用 command 兜底。
  active_command_label_ = label ? label : "command";
  // 清理上一条命令的 sent/accepted/rejected 状态。
  resetCommandState();
  resetCommandPairState();
  // 从当前 tick 重新计时，用于 command_timeout_s。
  markStageStart();
  // 有 node 时打印命令 ID，便于现场和串口日志对照。
  if (node_) {
    RCLCPP_INFO(node_->get_logger(), "%s: 下发 %s(0x%02X)",
                action_label_.c_str(), active_command_label_.c_str(),
                static_cast<unsigned int>(static_cast<uint8_t>(command_id)));
  }
}

// tickCommand() 推进当前命令阶段：首次调用发 service，后续调用等 accepted/rejected/timeout。
StairActionBase::StepStatus StairActionBase::tickCommand() {
  // node/client 不存在说明动作生命周期异常，直接返回 Failure。
  if (!node_ || !send_client_) {
    return StepStatus::Failure;
  }

  // 如果异步 service 回调已经写入结果，本 tick 直接把结果映射成阶段状态。
  if (command_response_seen_.load(std::memory_order_relaxed)) {
    // accepted=true 表示 bridge 已经通过可靠 sendCommand() 收到 MCU 通用 ACK。
    if (command_accepted_.load(std::memory_order_relaxed)) {
      RCLCPP_INFO(node_->get_logger(), "%s: %s accepted",
                  action_label_.c_str(), active_command_label_.c_str());
      return StepStatus::Success;
    }
    // accepted=false 表示 service 拒绝或回调异常，状态机应停车失败。
    RCLCPP_WARN(node_->get_logger(), "%s: %s rejected",
                action_label_.c_str(), active_command_label_.c_str());
    return StepStatus::Failure;
  }

  // 如果从 beginCommand() 到现在超过命令超时，也视为失败。
  if (elapsedSinceStageStart() > params_.command_timeout_s) {
    RCLCPP_WARN(node_->get_logger(), "%s: %s 超时 %.2fs",
                action_label_.c_str(), active_command_label_.c_str(),
                params_.command_timeout_s);
    return StepStatus::Failure;
  }

  // command_sent_=false 表示本阶段还没有真正发出 service 请求。
  if (!command_sent_) {
    // service 未就绪时不消耗发送机会，继续 RUNNING 等待。
    if (!send_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "%s: 等待服务 %s",
                           action_label_.c_str(),
                           params_.send_command_service.c_str());
      return StepStatus::Running;
    }

    // 构造共享串口命令请求。
    auto request = std::make_shared<SendCommandSrv::Request>();
    // 写入当前阶段命令 ID，例如 FRONT_PUSHROD_EXTEND。
    request->command_id = static_cast<uint8_t>(active_command_);
    // 台阶推杆命令 v1 不需要 payload，保持空数组。
    request->payload.clear();
    // 捕获当前 generation；如果动作 halt/release 后 generation 改变，旧回调会被丢弃。
    const uint64_t token =
        command_generation_.load(std::memory_order_relaxed);

    // async_send_request 本身可能抛异常，因此用 try/catch 把异常收敛成阶段失败。
    try {
      // 异步发送 service，避免在 BT tick 线程里阻塞等待 MCU ACK。
      send_client_->async_send_request(
          request, [this, token](rclcpp::Client<SendCommandSrv>::SharedFuture future) {
            // 如果动作已经结束或进入新命令阶段，丢弃这个旧回调。
            if (token !=
                command_generation_.load(std::memory_order_relaxed)) {
              return;
            }
            // future.get() 可能因 executor/transport 异常抛出，因此单独保护。
            try {
              // 获取 service response。
              const auto response = future.get();
              // accepted=true 是本状态机推进到下一阶段的唯一命令成功条件。
              const bool accepted = response && response->accepted;
              // 写入 accepted 标志；onRunning() 后续 tick 会读取。
              command_accepted_.store(accepted, std::memory_order_relaxed);
              // 写入 rejected 标志，主要用于调试和状态完整性。
              command_rejected_.store(!accepted, std::memory_order_relaxed);
            } catch (const std::exception &) {
              // 回调异常时当作命令失败。
              command_accepted_.store(false, std::memory_order_relaxed);
              // 标记 rejected，保持状态一致。
              command_rejected_.store(true, std::memory_order_relaxed);
            }
            // 最后写 response_seen，确保主 tick 看到结果时 accepted/rejected 已经写完。
            command_response_seen_.store(true, std::memory_order_relaxed);
          });
    } catch (const std::exception &e) {
      // service 调用本身失败，立即返回 Failure。
      RCLCPP_WARN(node_->get_logger(), "%s: %s async_send_request 失败: %s",
                  action_label_.c_str(), active_command_label_.c_str(),
                  e.what());
      return StepStatus::Failure;
    }
    // 标记本阶段已经发出请求；之后 tick 只等待回调或超时，不重复发送。
    command_sent_ = true;
  }

  // 当前命令还在等待 service response。
  (void)command_rejected_;
  return StepStatus::Running;
}

// beginCommandPair() 开始一个“同一 tick 连续下发两条推杆命令并等待都 accepted”的阶段。
void StairActionBase::beginCommandPair(CommandID first_command_id,
                                       const char *first_label,
                                       CommandID second_command_id,
                                       const char *second_label) {
  // 单命令和双命令共用 generation；进入新阶段后，旧异步回调都不能再写当前状态。
  command_generation_.fetch_add(1, std::memory_order_relaxed);
  resetCommandState();
  resetCommandPairState();

  command_pair_[0].command_id = first_command_id;
  command_pair_[0].label = first_label ? first_label : "first_command";
  command_pair_[1].command_id = second_command_id;
  command_pair_[1].label = second_label ? second_label : "second_command";
  command_pair_active_ = true;
  markStageStart();

  if (node_) {
    RCLCPP_INFO(node_->get_logger(),
                "%s: 同步下发 %s(0x%02X) + %s(0x%02X)",
                action_label_.c_str(), command_pair_[0].label.c_str(),
                static_cast<unsigned int>(
                    static_cast<uint8_t>(first_command_id)),
                command_pair_[1].label.c_str(),
                static_cast<unsigned int>(
                    static_cast<uint8_t>(second_command_id)));
  }
}

// tickCommandPair() 推进双命令阶段：同一 tick 发送两条异步请求，等待两条都 accepted。
StairActionBase::StepStatus StairActionBase::tickCommandPair() {
  if (!node_ || !send_client_ || !command_pair_active_) {
    return StepStatus::Failure;
  }

  for (const auto &slot : command_pair_) {
    if (slot.response_seen.load(std::memory_order_relaxed) &&
        !slot.accepted.load(std::memory_order_relaxed)) {
      RCLCPP_WARN(node_->get_logger(), "%s: %s rejected",
                  action_label_.c_str(), slot.label.c_str());
      return StepStatus::Failure;
    }
  }

  const bool first_done =
      command_pair_[0].response_seen.load(std::memory_order_relaxed) &&
      command_pair_[0].accepted.load(std::memory_order_relaxed);
  const bool second_done =
      command_pair_[1].response_seen.load(std::memory_order_relaxed) &&
      command_pair_[1].accepted.load(std::memory_order_relaxed);
  if (first_done && second_done) {
    RCLCPP_INFO(node_->get_logger(), "%s: %s + %s accepted",
                action_label_.c_str(), command_pair_[0].label.c_str(),
                command_pair_[1].label.c_str());
    command_pair_active_ = false;
    return StepStatus::Success;
  }

  if (elapsedSinceStageStart() > params_.command_timeout_s) {
    RCLCPP_WARN(node_->get_logger(), "%s: %s + %s 超时 %.2fs",
                action_label_.c_str(), command_pair_[0].label.c_str(),
                command_pair_[1].label.c_str(), params_.command_timeout_s);
    return StepStatus::Failure;
  }

  if (!send_client_->service_is_ready()) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                         "%s: 等待服务 %s", action_label_.c_str(),
                         params_.send_command_service.c_str());
    return StepStatus::Running;
  }

  bool send_failed = false;
  for (std::size_t index = 0; index < 2; ++index) {
    if (!command_pair_[index].sent) {
      send_failed = !sendPairCommand(index) || send_failed;
    }
  }
  if (send_failed) {
    return StepStatus::Failure;
  }

  return StepStatus::Running;
}

// beginEventWait() 开始一个“等待指定激光高度突变”的阶段。
void StairActionBase::beginEventWait(WheelEvent event, double timeout_s,
                                     const char *label) {
  // 记录当前等待的是前轮第一、前轮第二还是后轮事件。
  active_event_ = event;
  // 记录日志标签；没有显式标签时根据事件类型生成默认名称。
  active_event_label_ = label ? label : eventName(event);
  // 记录本阶段的事件超时，并做最小值保护。
  active_event_timeout_s_ = std::max(kMinTimeoutS, timeout_s);
  // 记录三个事件基线；防止上一阶段或启动前的旧事件误触发本阶段。
  front_first_event_baseline_ =
      front_first_event_count_.load(std::memory_order_relaxed);
  front_second_event_baseline_ =
      front_second_event_count_.load(std::memory_order_relaxed);
  rear_event_baseline_ = rear_event_count_.load(std::memory_order_relaxed);
  // 从当前 tick 开始计算事件等待超时。
  markStageStart();
  // 清除速度限频状态，保证进入行驶阶段后第一帧速度可以立即发布。
  has_last_drive_publish_ = false;

  // 打印等待事件，便于现场对照 MCU 上报 0x17/0x18。
  if (node_) {
    RCLCPP_INFO(node_->get_logger(), "%s: 等待 %s 激光高度突变",
                action_label_.c_str(), active_event_label_.c_str());
  }
}

// tickEventWait() 检查当前等待的激光事件是否到达或超时。
StairActionBase::StepStatus StairActionBase::tickEventWait() {
  // eventReceived() 为 true 表示订阅回调已经累计到本阶段之后的新事件。
  if (eventReceived()) {
    // 记录收到事件的日志，便于确认状态机推进原因。
    if (node_) {
      RCLCPP_INFO(node_->get_logger(), "%s: 收到 %s 激光高度突变",
                  action_label_.c_str(), active_event_label_.c_str());
    }
    // 事件满足，当前阶段成功。
    return StepStatus::Success;
  }

  // 如果事件迟迟不到且超过超时时间，当前阶段失败。
  if (elapsedSinceStageStart() > active_event_timeout_s_) {
    // 打印超时日志，指出等待的是 front 还是 rear。
    if (node_) {
      RCLCPP_WARN(node_->get_logger(), "%s: 等待 %s 激光高度突变超时 %.2fs",
                  action_label_.c_str(), active_event_label_.c_str(),
                  active_event_timeout_s_);
    }
    // 事件等待失败，具体动作会停车并返回 BT FAILURE。
    return StepStatus::Failure;
  }

  // 事件未到且未超时，继续 RUNNING。
  return StepStatus::Running;
}

// beginTimedDrive() 开始一个“固定时间持续发布速度”的阶段。
void StairActionBase::beginTimedDrive(double signed_speed_mps,
                                      double duration_s,
                                      const char *label) {
  // 保存本阶段要发布的带符号速度。
  timed_drive_speed_mps_ = signed_speed_mps;
  // 保存持续时间；负数按 0 处理，表示下一 tick 立即完成。
  timed_drive_duration_s_ = std::max(0.0, duration_s);
  // 保存日志标签。
  timed_drive_label_ = label ? label : "timed_drive";
  // 从当前 tick 开始计时。
  markStageStart();
  // 清除限频状态，保证定时行驶第一帧速度可以立即发布。
  has_last_drive_publish_ = false;

  // 打印定时行驶阶段的持续时间，便于现场调参。
  if (node_) {
    RCLCPP_INFO(node_->get_logger(), "%s: 开始 %s %.2fs",
                action_label_.c_str(), timed_drive_label_.c_str(),
                timed_drive_duration_s_);
  }
}

// tickTimedDrive() 推进定时行驶阶段。
StairActionBase::StepStatus StairActionBase::tickTimedDrive() {
  // 已达到持续时间就结束当前阶段。
  if (elapsedSinceStageStart() >= timed_drive_duration_s_) {
    return StepStatus::Success;
  }

  // 时间未到，继续按限频发布当前阶段速度。
  publishDrive(timed_drive_speed_mps_);
  // 定时行驶尚未完成。
  return StepStatus::Running;
}

// beginZeroHold() 开始一个“持续发布零速并等待固定时长”的阶段。
void StairActionBase::beginZeroHold(double duration_s, const char *label) {
  zero_hold_duration_s_ = std::max(0.0, duration_s);
  zero_hold_label_ = label ? label : "zero_hold";
  markStageStart();
  has_last_drive_publish_ = false;

  if (node_) {
    RCLCPP_INFO(node_->get_logger(), "%s: 开始零速等待 %s %.2fs",
                action_label_.c_str(), zero_hold_label_.c_str(),
                zero_hold_duration_s_);
  }

  publishStop();
}

// tickZeroHold() 延时期间每 tick 都补零速，确保底盘不会沿用上一阶段速度。
StairActionBase::StepStatus StairActionBase::tickZeroHold() {
  publishStop();
  if (elapsedSinceStageStart() >= zero_hold_duration_s_) {
    return StepStatus::Success;
  }
  return StepStatus::Running;
}

BT::NodeStatus StairActionBase::failWithStop(const char *reason) {
  // 失败入口统一打印原因，让上台阶/下台阶不用在每个分支重复写停车收尾。
  if (node_) {
    RCLCPP_WARN(node_->get_logger(), "%s 失败: %s", action_label_.c_str(),
                reason ? reason : "unknown");
  }
  // 任何失败都先发布零速，确保离开动作前底盘没有继续运动。
  publishStop();
  // 释放本次动作创建的 ROS 资源，并使未完成异步回调失效。
  releaseRuntime();
  // 将公共失败语义映射为 BT FAILURE。
  return BT::NodeStatus::FAILURE;
}

double StairActionBase::elapsedSinceStageStart() const {
  // node_ 为空表示动作未初始化或已释放，此时不给调用方制造异常时间值。
  if (!node_) {
    return 0.0;
  }
  // 当前 ROS time 减去阶段起始时间，就是本阶段已经持续的秒数。
  return (node_->now() - stage_start_).seconds();
}

void StairActionBase::markStageStart() {
  // 只有 node_ 有效时才可读取 ROS time。
  if (node_) {
    // 记录阶段起始时间；命令等待、事件等待、定时行驶都复用这个时间戳做超时判断。
    stage_start_ = node_->now();
  }
}

void StairActionBase::resetCommandState() {
  // 当前阶段尚未发送 service 请求。
  command_sent_ = false;
  // 当前阶段尚未收到 service 回调。
  command_response_seen_.store(false, std::memory_order_relaxed);
  // 当前阶段尚未确认 accepted。
  command_accepted_.store(false, std::memory_order_relaxed);
  // 当前阶段尚未确认 rejected。
  command_rejected_.store(false, std::memory_order_relaxed);
}

void StairActionBase::resetCommandPairState() {
  command_pair_active_ = false;
  for (auto &slot : command_pair_) {
    slot.command_id = CommandID::STOP;
    slot.label.clear();
    slot.sent = false;
    slot.response_seen.store(false, std::memory_order_relaxed);
    slot.accepted.store(false, std::memory_order_relaxed);
    slot.rejected.store(false, std::memory_order_relaxed);
  }
}

bool StairActionBase::sendPairCommand(std::size_t index) {
  if (index >= 2 || !node_ || !send_client_) {
    return false;
  }

  auto &slot = command_pair_[index];
  auto request = std::make_shared<SendCommandSrv::Request>();
  request->command_id = static_cast<uint8_t>(slot.command_id);
  request->payload.clear();
  const uint64_t token = command_generation_.load(std::memory_order_relaxed);

  try {
    send_client_->async_send_request(
        request,
        [this, token, index](
            rclcpp::Client<SendCommandSrv>::SharedFuture future) {
          if (token != command_generation_.load(std::memory_order_relaxed) ||
              index >= 2) {
            return;
          }

          try {
            const auto response = future.get();
            const bool accepted = response && response->accepted;
            command_pair_[index].accepted.store(accepted,
                                                std::memory_order_relaxed);
            command_pair_[index].rejected.store(!accepted,
                                                std::memory_order_relaxed);
          } catch (const std::exception &) {
            command_pair_[index].accepted.store(false,
                                                std::memory_order_relaxed);
            command_pair_[index].rejected.store(true,
                                                std::memory_order_relaxed);
          }
          command_pair_[index].response_seen.store(true,
                                                   std::memory_order_relaxed);
        });
  } catch (const std::exception &e) {
    RCLCPP_WARN(node_->get_logger(), "%s: %s async_send_request 失败: %s",
                action_label_.c_str(), slot.label.c_str(), e.what());
    return false;
  }

  slot.sent = true;
  return true;
}

bool StairActionBase::eventReceived() const {
  switch (active_event_) {
  case WheelEvent::FrontFirst:
    return front_first_event_count_.load(std::memory_order_relaxed) >
           front_first_event_baseline_;
  case WheelEvent::FrontSecond:
    return front_second_event_count_.load(std::memory_order_relaxed) >
           front_second_event_baseline_;
  case WheelEvent::Rear:
    return rear_event_count_.load(std::memory_order_relaxed) >
           rear_event_baseline_;
  }
  return false;
}

} // namespace rc26_decision
