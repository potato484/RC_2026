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
  // 设置后轮事件等待基线和超时时间；只接受本动作开始后新来的 0x18。
  beginEventWait(WheelEvent::Rear, params_.rear_event_timeout_s, "rear");
  // 返回 RUNNING，让后续 tick 持续发布 x 负向速度。
  return BT::NodeStatus::RUNNING;
}

// onRunning() 每个 BT tick 推进一步下台阶状态机；所有等待都非阻塞。
BT::NodeStatus StairDescendAction::onRunning() {
  // phase_ 表示当前处于“等事件”“等命令 accepted”或“定时行驶”的哪一步。
  switch (phase_) {
  case Phase::DriveUntilRearEvent:
    // 第一阶段：后轮在前，持续发布 x 负方向速度靠近下阶梯边缘。
    publishDrive(-driveSpeedMagnitude());
    // 检查 MCU 是否已经上报后轮激光测距高度突变。
    switch (tickEventWait()) {
    case StepStatus::Success:
      // 后轮突变到达，先停车，再切换后推杆状态。
      publishStop();
      // 下一阶段伸出后推杆，支撑已经先进入下台阶过程的后轮侧。
      phase_ = Phase::SendRearExtend;
      // 下发 REAR_PUSHROD_EXTEND；等待 accepted 后继续负向行驶。
      beginCommand(CommandID::REAR_PUSHROD_EXTEND, "REAR_PUSHROD_EXTEND");
      break;
    case StepStatus::Failure:
      // 后轮事件超时，说明没有到达预期边缘或传感器异常，停车失败。
      return failWithStop("rear laser event timeout");
    case StepStatus::Running:
      // 后轮事件尚未到达，保持 RUNNING，下个 tick 继续发 x 负向速度。
      break;
    }
    break;

  case Phase::SendRearExtend:
    // 第二阶段：等待后推杆伸出命令被共享串口 transport 接受。
    switch (tickCommand()) {
    case StepStatus::Success:
      // 后推杆已伸出，继续负向行驶，直到前轮也检测到高度突变。
      phase_ = Phase::DriveUntilFrontEvent;
      // 设置前轮事件等待基线和超时；只接受此后新来的 0x17。
      beginEventWait(WheelEvent::Front, params_.front_event_timeout_s,
                     "front");
      break;
    case StepStatus::Failure:
      // 后推杆伸出失败时不再继续运动，防止车体失去支撑。
      return failWithStop("REAR_PUSHROD_EXTEND failed");
    case StepStatus::Running:
      // service 仍在等待 accepted，本 tick 不阻塞。
      break;
    }
    break;

  case Phase::DriveUntilFrontEvent:
    // 第三阶段：继续发布 x 负方向速度，等待前轮侧越过下台阶边缘。
    publishDrive(-driveSpeedMagnitude());
    // 检查 MCU 是否已经上报前轮激光测距高度突变。
    switch (tickEventWait()) {
    case StepStatus::Success:
      // 前轮事件到达，先停车，避免边行驶边切换推杆。
      publishStop();
      // 下一阶段先收回后推杆。
      phase_ = Phase::SendRearRetract;
      // 下发 REAR_PUSHROD_RETRACT；accepted 后再伸前推杆。
      beginCommand(CommandID::REAR_PUSHROD_RETRACT,
                   "REAR_PUSHROD_RETRACT");
      break;
    case StepStatus::Failure:
      // 前轮事件超时，说明下台阶未按预期推进，停车失败。
      return failWithStop("front laser event timeout");
    case StepStatus::Running:
      // 前轮事件尚未到达，保持 RUNNING，下个 tick 继续发布负向速度。
      break;
    }
    break;

  case Phase::SendRearRetract:
    // 第四阶段：等待后推杆收回 accepted。
    switch (tickCommand()) {
    case StepStatus::Success:
      // 后推杆已收回，下一步伸出前推杆。
      phase_ = Phase::SendFrontExtend;
      // 下发 FRONT_PUSHROD_EXTEND；这是下台阶流程末段的前推杆动作。
      beginCommand(CommandID::FRONT_PUSHROD_EXTEND,
                   "FRONT_PUSHROD_EXTEND");
      break;
    case StepStatus::Failure:
      // 后推杆收回失败，机构状态不确定，停车失败。
      return failWithStop("REAR_PUSHROD_RETRACT failed");
    case StepStatus::Running:
      // 继续等待后推杆收回 accepted。
      break;
    }
    break;

  case Phase::SendFrontExtend:
    // 第五阶段：等待前推杆伸出命令 accepted。
    switch (tickCommand()) {
    case StepStatus::Success:
      // 前推杆伸出 accepted 后，进入最后一小段 x 负方向定时行驶。
      phase_ = Phase::FinishDrive;
      // 这段行驶暂不接定位闭环，只按参数配置的时间推进。
      beginTimedDrive(-driveSpeedMagnitude(),
                      params_.descend_finish_drive_time_s,
                      "descend finish drive");
      break;
    case StepStatus::Failure:
      // 前推杆伸出失败，停车失败。
      return failWithStop("FRONT_PUSHROD_EXTEND failed");
    case StepStatus::Running:
      // 等待前推杆伸出 accepted。
      break;
    }
    break;

  case Phase::FinishDrive:
    // 第六阶段：按固定时间继续发布 x 负方向速度，帮助车体完成落到下一层。
    switch (tickTimedDrive()) {
    case StepStatus::Success:
      // 定时行驶结束后必须停车，避免独立动作结束后继续保留速度。
      publishStop();
      // 写黑板完成标记，表示本次下台阶状态机完整走完。
      config().blackboard->set("stair_descend_done", true);
      // 进入 Done，避免重复收尾。
      phase_ = Phase::Done;
      // 释放本次动作资源。
      releaseRuntime();
      // 下台阶动作成功。
      return BT::NodeStatus::SUCCESS;
    case StepStatus::Failure:
      // tickTimedDrive 当前不会主动失败；保留分支用于未来加入定位判定。
      return failWithStop("descend finish drive failed");
    case StepStatus::Running:
      // 定时行驶尚未结束，下个 tick 继续发负向速度。
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
