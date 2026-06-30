#include "rc26_decision/stair/stair_descend.hpp"

namespace rc26_decision {

// 构造函数只连接 BT 节点配置和公共基类；实际资源由每次 onStart() 重新初始化。
StairDescendAction::StairDescendAction(const std::string &name,
                                       const BT::NodeConfig &config)
    : StairActionBase(name, config) {}

// onStart() 是下台阶独立动作第一次被 tick 时的入口。
BT::NodeStatus StairDescendAction::onStart() {
  // 初始化 node、stair_params、cmd_vel publisher、推杆 service client 和 feedback subscriber。
  if (!setupRuntime("下台阶")) {
    // 缺少运行上下文时不允许发布运动命令，直接 FAILURE。
    return BT::NodeStatus::FAILURE;
  }

  // 清掉上一轮下台阶完成标记，保证本轮结果由当前状态机写入。
  config().blackboard->set("stair_descend_done", false);
  // 下台阶要求后轮朝前，第一阶段先负向行驶，等待后轮激光高度突变。
  phase_ = Phase::DriveUntilRearEvent;
  // 设置后轮事件等待基线和超时时间；只接受本动作开始后新来的 0x05。
  beginEventWait(WheelEvent::Rear, params_.rear_event_timeout_s, "rear");
  // 返回 RUNNING，让后续 tick 持续发布 x 负向速度。
  return BT::NodeStatus::RUNNING;
}

// onRunning() 每个 BT tick 推进一步下台阶状态机；所有等待都非阻塞。
BT::NodeStatus StairDescendAction::onRunning() {
  // phase_ 表示当前处于“行驶等事件”“等命令 accepted”或“零速等待”的哪一步。
  switch (phase_) {
  case Phase::DriveUntilRearEvent:
    // 第一阶段：后轮在前，持续发布 x 负方向速度靠近下阶梯边缘。
    publishDrive(-params_.descend_rear_drive_speed_mps);
    // 检查 MCU 是否已经上报后轮激光测距高度突变。
    switch (tickEventWait()) {
    case StepStatus::Success:
      // 后轮突变到达，先停车，再切换后推杆状态。
      publishStop();
      // 下一阶段伸出后推杆，支撑已经先进入下台阶过程的后轮侧。
      phase_ = Phase::SendRearExtend;
      // 下发 REAR_PUSHROD_EXTEND；等待 accepted 后先零速延时。
      beginCommand(CommandID::REAR_PUSHROD_EXTEND, "REAR_PUSHROD_EXTEND");
      break;
    case StepStatus::Failure:
      // 后轮事件超时，说明没有到达预期边缘或传感器异常，停车失败。
      return failWithStop("等待后轮激光高度突变超时");
    case StepStatus::Running:
      // 后轮事件尚未到达，保持 RUNNING，下个 tick 继续发 x 负向速度。
      break;
    }
    break;

  case Phase::SendRearExtend:
    // 第二阶段：等待后推杆伸出命令被共享串口 transport 接受，等待期间必须保持零速。
    publishStop();
    switch (tickCommand()) {
    case StepStatus::Success:
      // 后推杆已伸出，先按配置零速等待机构稳定；等待期间机器人不能移动。
      phase_ = Phase::HoldAfterRearExtend;
      beginZeroHold(params_.descend_rear_extend_delay_s,
                    "rear_extend_settle");
      break;
    case StepStatus::Failure:
      // 后推杆伸出失败时不再继续运动，防止车体失去支撑。
      return failWithStop("REAR_PUSHROD_EXTEND 命令失败");
    case StepStatus::Running:
      // service 仍在等待 accepted，本 tick 不阻塞。
      break;
    }
    break;

  case Phase::HoldAfterRearExtend:
    // 第三阶段：后推杆伸出后零速等待，持续覆盖上一阶段可能残留的速度。
    switch (tickZeroHold()) {
    case StepStatus::Success:
      // 延时结束后继续 x 负方向行驶，直到前轮第二个激光测距模块 0x07 检测到高度突变。
      phase_ = Phase::DriveUntilFrontSecondEvent;
      // 下台阶全链路不需要 0x04；这里只接受此后新来的前轮第二激光 0x07。
      beginEventWait(WheelEvent::FrontSecond, params_.front_event_timeout_s,
                     "front_second");
      beginDriveProfile(params_.descend_front_second_drive_profile,
                        "descend_front_second");
      break;
    case StepStatus::Failure:
      return failWithStop("后推杆伸出后零速等待失败");
    case StepStatus::Running:
      break;
    }
    break;

  case Phase::DriveUntilFrontSecondEvent:
    // 第四阶段：按前轮第二激光段速度规划发布 x 负方向速度，等待 0x07。
    publishProfiledDrive(-1.0);
    // 检查 MCU 是否已经上报前轮第二个激光测距模块高度突变 0x07。
    switch (tickEventWait()) {
    case StepStatus::Success:
      // 前轮第二激光 0x07 事件到达后立即停车，同一 tick 连续发送后收和前伸两条命令。
      publishStop();
      phase_ = Phase::SendRearRetractAndFrontExtend;
      beginCommandPair(CommandID::REAR_PUSHROD_RETRACT,
                       "REAR_PUSHROD_RETRACT",
                       CommandID::FRONT_PUSHROD_EXTEND,
                       "FRONT_PUSHROD_EXTEND");
      switch (tickCommandPair()) {
      case StepStatus::Success:
        phase_ = Phase::HoldAfterRearRetractAndFrontExtend;
        beginZeroHold(params_.descend_retract_front_extend_delay_s,
                      "rear_retract_front_extend_settle");
        break;
      case StepStatus::Failure:
        return failWithStop(
            "REAR_PUSHROD_RETRACT + FRONT_PUSHROD_EXTEND 并发命令失败");
      case StepStatus::Running:
        break;
      }
      break;
    case StepStatus::Failure:
      // 前轮第二激光事件超时，说明下台阶未按预期推进，停车失败。
      return failWithStop("等待前轮第二激光高度突变超时");
    case StepStatus::Running:
      // 前轮第二激光事件尚未到达，保持 RUNNING，下个 tick 继续发布负向速度。
      break;
    }
    break;

  case Phase::SendRearRetractAndFrontExtend:
    // 第五阶段：等待后推杆收回和前推杆伸出都 accepted；等待期间必须保持零速。
    publishStop();
    switch (tickCommandPair()) {
    case StepStatus::Success:
      // 两条命令都 accepted 后，再零速等待前推杆到位。
      phase_ = Phase::HoldAfterRearRetractAndFrontExtend;
      beginZeroHold(params_.descend_retract_front_extend_delay_s,
                    "rear_retract_front_extend_settle");
      break;
    case StepStatus::Failure:
      // 任一组合命令失败时不再继续运动，防止机构姿态不确定。
      return failWithStop("REAR_PUSHROD_RETRACT + FRONT_PUSHROD_EXTEND 并发命令失败");
    case StepStatus::Running:
      break;
    }
    break;

  case Phase::HoldAfterRearRetractAndFrontExtend:
    // 第六阶段：组合推杆命令后零速等待；这段期间忽略任何旧激光突变。
    switch (tickZeroHold()) {
    case StepStatus::Success:
      // 延时结束后以更低 x 负向速度继续行驶固定时间，时间到后触发前推杆收回。
      phase_ = Phase::TimedDriveBeforeFrontRetract;
      beginTimedDrive(-params_.descend_front_retract_timed_drive_speed_mps,
                      params_.descend_front_retract_drive_duration_s,
                      "front_retract_trigger_drive");
      break;
    case StepStatus::Failure:
      return failWithStop("后推杆收回和前推杆伸出后零速等待失败");
    case StepStatus::Running:
      break;
    }
    break;

  case Phase::TimedDriveBeforeFrontRetract:
    // 第七阶段：收到 0x07 并完成后收+前伸后，以 x 负向 0.025m/s 默认速度持续 4s。
    switch (tickTimedDrive()) {
    case StepStatus::Success:
      // 定时行驶完成后，收回前推杆作为下台阶最后动作。
      publishStop();
      phase_ = Phase::SendFrontRetract;
      beginCommand(CommandID::FRONT_PUSHROD_RETRACT,
                   "FRONT_PUSHROD_RETRACT");
      break;
    case StepStatus::Failure:
      return failWithStop("前推杆收回触发定时行驶失败");
    case StepStatus::Running:
      break;
    }
    break;

  case Phase::SendFrontRetract:
    // 第八阶段：等待前推杆收回 accepted，accepted 后进入最后零速等待。
    publishStop();
    switch (tickCommand()) {
    case StepStatus::Success:
      // 收到 accepted 后按配置继续零速等待，让前推杆收回动作有稳定时间。
      phase_ = Phase::HoldAfterFrontRetract;
      beginZeroHold(params_.descend_front_retract_delay_s,
                    "front_retract_settle");
      break;
    case StepStatus::Failure:
      return failWithStop("FRONT_PUSHROD_RETRACT 命令失败");
    case StepStatus::Running:
      break;
    }
    break;

  case Phase::HoldAfterFrontRetract:
    // 第九阶段：前推杆收回 accepted 后零速等待；等待结束才向 BT 报告成功。
    switch (tickZeroHold()) {
    case StepStatus::Success:
      // 写黑板完成标记，表示本次下台阶状态机完整走完。
      config().blackboard->set("stair_descend_done", true);
      // 进入 Done，避免重复收尾。
      phase_ = Phase::Done;
      // 释放本次动作资源。
      releaseRuntime();
      // 下台阶动作成功。
      return BT::NodeStatus::SUCCESS;
    case StepStatus::Failure:
      return failWithStop("前推杆收回后零速等待失败");
    case StepStatus::Running:
      break;
    }
    break;

  case Phase::Done:
    // Done 表示动作已经成功收尾；如果 BT 再 tick 到这里，保持 SUCCESS。
    return BT::NodeStatus::SUCCESS;
  }

  // 没有成功或失败返回时，说明当前阶段仍在等待，行为树继续 RUNNING。
  return BT::NodeStatus::RUNNING;
}

// onHalted() 处理外部中断。
void StairDescendAction::onHalted() {
  // 中断时只发布零速，不补发任何推杆命令，避免未知姿态下误动作。
  publishStop();
  // 释放订阅、client、publisher，并让未返回的异步回调失效。
  releaseRuntime();
  // 清理阶段状态，防止残留阶段影响下一次动作。
  phase_ = Phase::Done;
}

} // namespace rc26_decision
