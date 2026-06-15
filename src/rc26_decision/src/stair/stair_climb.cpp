#include "rc26_decision/stair/stair_climb.hpp"

namespace rc26_decision {

// 构造函数只把 BT 节点名和配置交给公共基类；真正的运行时资源在 onStart() 里按每次动作重新创建。
StairClimbAction::StairClimbAction(const std::string &name,
                                   const BT::NodeConfig &config)
    : StairActionBase(name, config) {}

// onStart() 是行为树第一次 tick 到 StairClimb 时调用的入口。
BT::NodeStatus StairClimbAction::onStart() {
  // 初始化 ROS 节点指针、台阶参数、cmd_vel publisher、共享串口 service client 和 feedback subscriber。
  if (!setupRuntime("上台阶")) {
    // 如果黑板缺少 node/stair_params 等关键上下文，动作无法安全运行，直接失败。
    return BT::NodeStatus::FAILURE;
  }

  // 本次上台阶刚开始，先把黑板上的完成标记清成 false，避免读到上一轮残留结果。
  config().blackboard->set("stair_climb_done", false);
  // 上台阶第一步必须先伸出前推杆，为前轮上台阶做支撑。
  phase_ = Phase::SendFrontExtend;
  // 下发 FRONT_PUSHROD_EXTEND；tickCommand() 后续会等待 /mechanism/send_command accepted=true。
  beginCommand(CommandID::FRONT_PUSHROD_EXTEND, "FRONT_PUSHROD_EXTEND");
  // 命令是异步等待的，所以第一次进入动作后返回 RUNNING，让行为树下一 tick 继续推进。
  return BT::NodeStatus::RUNNING;
}

// onRunning() 每个 BT tick 调一次；这里用 phase_ 实现非阻塞状态机。
BT::NodeStatus StairClimbAction::onRunning() {
  // 根据当前阶段决定本 tick 要做的是等命令 accepted、发速度、等激光事件，还是收尾。
  switch (phase_) {
  case Phase::SendFrontExtend:
    // 第一阶段：等待前推杆伸出命令被共享串口 transport 接受。
    switch (tickCommand()) {
    case StepStatus::Success:
      // 前推杆命令 accepted 后，进入前轮越过阶梯边缘的行驶阶段。
      phase_ = Phase::DriveUntilFrontEvent;
      // 记录当前前轮事件计数作为基线，只接受此后新来的 FRONT_LASER_HEIGHT_JUMP。
      beginEventWait(WheelEvent::Front, params_.front_event_timeout_s,
                     "front");
      break;
    case StepStatus::Failure:
      // 命令被拒绝或等待超时，发布零速并返回 FAILURE。
      return failWithStop("FRONT_PUSHROD_EXTEND failed");
    case StepStatus::Running:
      // service 尚未返回或尚未就绪，本 tick 不阻塞，继续 RUNNING。
      break;
    }
    break;

  case Phase::DriveUntilFrontEvent:
    // 第二阶段：前推杆已伸出，持续发布 x 正方向速度推动前轮上台阶。
    publishDrive(driveSpeedMagnitude());
    // 同时检查 MCU 是否已经上报前轮激光测距高度突变。
    switch (tickEventWait()) {
    case StepStatus::Success:
      // 前轮事件到达，先停车，避免切换推杆状态时底盘继续顶着台阶边缘。
      publishStop();
      // 前轮已经越过突变点，准备收回前推杆。
      phase_ = Phase::SendFrontRetract;
      // 下发 FRONT_PUSHROD_RETRACT；后续 tick 等待 accepted。
      beginCommand(CommandID::FRONT_PUSHROD_RETRACT,
                   "FRONT_PUSHROD_RETRACT");
      break;
    case StepStatus::Failure:
      // 等不到前轮突变，说明流程卡住或传感器异常，安全停车并失败。
      return failWithStop("front laser event timeout");
    case StepStatus::Running:
      // 还没等到前轮突变，本 tick 保持 RUNNING，下个 tick 继续发速度。
      break;
    }
    break;

  case Phase::SendFrontRetract:
    // 第三阶段：等待前推杆收回命令 accepted。
    switch (tickCommand()) {
    case StepStatus::Success:
      // 前推杆已确认收回，下一步伸后推杆，准备让后轮上台阶。
      phase_ = Phase::SendRearExtend;
      // 下发 REAR_PUSHROD_EXTEND；命令 accepted 后再继续行驶。
      beginCommand(CommandID::REAR_PUSHROD_EXTEND, "REAR_PUSHROD_EXTEND");
      break;
    case StepStatus::Failure:
      // 前推杆收回失败时不再继续运动，防止机构姿态不确定。
      return failWithStop("FRONT_PUSHROD_RETRACT failed");
    case StepStatus::Running:
      // 命令仍在等待 accepted，动作继续 RUNNING。
      break;
    }
    break;

  case Phase::SendRearExtend:
    // 第四阶段：等待后推杆伸出命令 accepted。
    switch (tickCommand()) {
    case StepStatus::Success:
      // 后推杆已伸出，进入后轮越过阶梯边缘的行驶阶段。
      phase_ = Phase::DriveUntilRearEvent;
      // 记录后轮事件计数基线，只接受后续新的 REAR_LASER_HEIGHT_JUMP。
      beginEventWait(WheelEvent::Rear, params_.rear_event_timeout_s, "rear");
      break;
    case StepStatus::Failure:
      // 后推杆伸出失败，继续上台阶不安全，停车失败。
      return failWithStop("REAR_PUSHROD_EXTEND failed");
    case StepStatus::Running:
      // 继续等待命令 accepted。
      break;
    }
    break;

  case Phase::DriveUntilRearEvent:
    // 第五阶段：后推杆已伸出，继续发布 x 正方向速度推动后轮上台阶。
    publishDrive(driveSpeedMagnitude());
    // 检查 MCU 是否已经上报后轮激光测距高度突变。
    switch (tickEventWait()) {
    case StepStatus::Success:
      // 后轮事件到达，完整上台阶运动已经越过关键边缘，先停车。
      publishStop();
      // 最后收回后推杆，结束台阶动作。
      phase_ = Phase::SendRearRetract;
      // 下发 REAR_PUSHROD_RETRACT；accepted 后才能宣布 SUCCESS。
      beginCommand(CommandID::REAR_PUSHROD_RETRACT,
                   "REAR_PUSHROD_RETRACT");
      break;
    case StepStatus::Failure:
      // 后轮事件超时，说明后轮未按预期上台阶或传感器没有反馈，停车失败。
      return failWithStop("rear laser event timeout");
    case StepStatus::Running:
      // 后轮事件尚未到达，保持 RUNNING，下个 tick 继续发 x 正向速度。
      break;
    }
    break;

  case Phase::SendRearRetract:
    // 第六阶段：等待后推杆收回 accepted，作为上台阶最后的收尾确认。
    switch (tickCommand()) {
    case StepStatus::Success:
      // 收到 accepted 后再补一帧零速，确保底盘命令权离开前处于停止状态。
      publishStop();
      // 写黑板完成标记，供独立树或后续调试读取本次动作结果。
      config().blackboard->set("stair_climb_done", true);
      // 进入 Done，避免同一个节点实例被重复 tick 时又执行收尾逻辑。
      phase_ = Phase::Done;
      // 释放订阅、client、publisher；本次动作生命周期结束。
      releaseRuntime();
      // 上台阶动作完整成功。
      return BT::NodeStatus::SUCCESS;
    case StepStatus::Failure:
      // 后推杆收回失败时仍然停车，并把动作结果报告为 FAILURE。
      return failWithStop("REAR_PUSHROD_RETRACT failed");
    case StepStatus::Running:
      // 继续等待后推杆收回命令 accepted。
      break;
    }
    break;

  case Phase::Done:
    // Done 表示动作已经成功收尾；如果 BT 再 tick 到这里，保持 SUCCESS。
    return BT::NodeStatus::SUCCESS;
  }

  // 以上阶段只要没有返回 SUCCESS/FAILURE，就说明动作还在进行。
  return BT::NodeStatus::RUNNING;
}

// onHalted() 在行为树被外部中断或父节点停止时调用。
void StairClimbAction::onHalted() {
  // 中断时只负责让底盘停住，不额外补发推杆命令，避免未知姿态下误动作。
  publishStop();
  // 释放本次动作创建的 ROS 资源，并使未返回的异步 service 回调失效。
  releaseRuntime();
  // 标记为 Done，防止同一实例后续残留在旧阶段。
  phase_ = Phase::Done;
}

} // namespace rc26_decision
