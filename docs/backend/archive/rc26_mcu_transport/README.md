# rc26_mcu_transport

`rc26_mcu_transport` 是当前 R2 目标 MCU 唯一共享串口 owner，提供机构 raw transport 和 `/cmd_vel -> POSE_TARGET(0x0C)` 底盘执行。

## 运行边界

- `rc26_bringup` 在自动链中启动本包，`start_r2_teleop.sh` 在遥控链中启动本包。
- 上层只能通过 `/mechanism/send_command`、`/mechanism/command_feedback` 与 `/cmd_vel` 使用目标 MCU，不得再次直连同一串口。
- 机构动作完成、比赛阶段容错和视觉验证由上层状态机负责；本包不伪造完成反馈。
- 串口不可用时节点保持存活并重试打开，service 拒绝发送，底盘 consumer 节流报告失败。

## 契约

`/mechanism/send_command` 保持 raw `uint8 command_id`。`wait_ack=true` 等待通用 `ACK(0x00)`，`wait_ack=false` 只确认写入。纯 ACK 重试耗尽不触发重连；真实读写、EOF、`epoll` error/hup 才触发重连。

`/mechanism/command_feedback` 只发布 `0x02~0x07`、`0x09~0x0D`、`0x10~0x15` 中正式定义的业务反馈，退役 `0x0F` 除外；两字节 `0xFE` 作为机械臂状态/错误发布。未知或退役反馈被丢弃并计入 `unsupported_feedback_drop_count`，最近 ID 写入 `last_unsupported_feedback_id`。

三个人工限位反馈为上行 `0x06/0x10/0x13`。下行 `0x10/0x13` 分别仍是比赛开始和第二预选赛放置命令，上下行语义不得混读。

## 本轮同步

2026-07-18：删除无调用的心跳、历史轮速和旧 second 高抬完成反馈路径；将上行透传改成正式业务 allowlist。机构生命周期占位包和历史底盘串口桥已删除，本包成为文档化的机构与底盘 MCU 执行唯一边界。
