# rc26_mechanism

## 模块定位

`rc26_mechanism` 当前是 R2 机构共享串口链路的轻量生命周期占位包。旧的高层机构 action 语义已经下线，真实机构命令统一通过 `rc26_mcu_transport` 提供的 raw transport service 下发。

## 当前实现

- 构建方式：组件库 + 独立可执行
- 导出节点：`mechanism_server_node`
- 启动文件：`launch/mechanism.launch.py`
- 默认 HAL：`shared_serial`

当前实现只保留最小真机链路：

- `mechanism_lifecycle_server.hpp/.cpp`：轻量 lifecycle 组件，只校验 `hal_type:=shared_serial` 并持有共享串口 HAL。
- `shared_serial_mechanism_hal.hpp/.cpp`：通过 `/mechanism/send_command` 和 `/mechanism/command_feedback` 复用 `rc26_mcu_transport` 已打开的目标 MCU 串口。
- `mechanism_command_catalog.hpp/.cpp`：保留空 catalog，给旧调用点提供编译兼容；当前不再注册任何高层命令完成语义。
- `test/catalog`：断言 catalog 为空、旧高层命令不可执行。

`mechanism.launch.py` 默认同时启动 `rc26_mcu_transport`；当 provider 已由 bringup、遥控脚本或其它入口启动时，应传 `start_mcu_transport:=false`，避免重复打开同一物理串口。

## 当前对外接口

`rc26_mechanism` 当前不再暴露高层 action，也不再发布机构状态 topic：

- 不创建旧抓端头 action endpoint
- 不创建旧组装 action endpoint
- 不创建旧通用执行 action endpoint
- 不发布 `/mechanism/status`

当前机构/底盘共享串口入口由 `rc26_mcu_transport` 提供：

- service `/mechanism/send_command`
- topic `/mechanism/command_feedback`

上层若要发送 `GRAB_TIP(0x01)`、`GRAB_KFS_DOWN(0x02)`、`GRAB_KFS_UP(0x03)`、`ENTRY_GRAB_KFS_UP(0x0F)`、`ARM_SECOND_LOWER(0x0E)`、`PLACE_KFS_GRID(0x06)`、推杆命令或其它原始机构命令，都直接调用 `/mechanism/send_command`。service `accepted=true` 只表示目标 MCU 已返回通用 `ACK(0x00)`，不表示动作已经完成。

## 协议口径

旧组装动作、通用 KFS 夹取动作、旧动作完成反馈和即时负确认语义已经从 `rc26_serial/protocol.hpp` 删除。当前机制层不再维护“命令对应哪些完成反馈”的目录。

当前与机构相关的原始 ID 口径：

- `GRAB_TIP = 0x01`
- `GRAB_KFS_DOWN = 0x02`
- `GRAB_KFS_UP = 0x03`
- `ARM_RAISE = 0x04`
- `ARM_LOWER = 0x05`
- `PLACE_KFS_GRID = 0x06`
- `FRONT_PUSHROD_EXTEND = 0x08`
- `FRONT_PUSHROD_RETRACT = 0x09`
- `REAR_PUSHROD_EXTEND = 0x0A`
- `REAR_PUSHROD_RETRACT = 0x0B`
- `POSE_TARGET = 0x0C`
- `ARM_HIGH_RAISE = 0x0D`
- `ARM_SECOND_LOWER = 0x0E`
- `ENTRY_GRAB_KFS_UP = 0x0F`

当前业务反馈口径：

- `ARM_RAISE_DONE = 0x02`
- `ARM_LOWER_DONE = 0x03`
- `FRONT_LASER_HEIGHT_JUMP = 0x04`
- `REAR_LASER_HEIGHT_JUMP = 0x05`
- `FRONT_LIMIT_SWITCH_TRIGGERED = 0x06`
- `FRONT_SECOND_LASER_HEIGHT_JUMP = 0x07`
- `ARM_HIGH_RAISE_DONE = 0x09`
- `ARM_SECOND_LOWER_DONE = 0x0A`
- `ENTRY_GRAB_KFS_UP_DONE = 0x0B`

KFS 阶梯等待测试链不因 `R_R1/B_R1` 阻塞标签发送夹取命令。`ARM_RAISE(0x04)` / `ARM_LOWER(0x05)` 由决策层直接通过 `/mechanism/send_command` 发送，并等待同 `seq` 的 `ARM_RAISE_DONE(0x02)` / `ARM_LOWER_DONE(0x03)`。向下夹取在锁定视觉深度并计算开环距离后，会先发送 `ARM_SECOND_LOWER(0x0E)` 并等待同 `seq` 的 `ARM_SECOND_LOWER_DONE(0x0A)`，确认后才前进或直接夹取。梅林预选赛入口高侧夹取使用 `ENTRY_GRAB_KFS_UP(0x0F)` 并等待同 `seq` 的 `ENTRY_GRAB_KFS_UP_DONE(0x0B)` 后进入视觉消失验证。`GRAB_KFS_UP(0x03)` 与 `GRAB_KFS_DOWN(0x02)` 已由梅林预选赛和 `rc26_vision` 独立 KFS action test 作为 raw transport 命令使用；本包不把 service ACK 解释为物理夹取成功，成功判定由上层视觉消失验证完成。

`PLACE_KFS_GRID(0x06)` 若仍需发送，只能走 raw `/mechanism/send_command`，不再有高层“完成反馈即成功”的封装。

## 真实部署口径

- 真机上由 `rc26_mcu_transport` 作为目标 MCU 串口 owner。
- `rc26_mechanism` 通过 `shared_serial` 复用 `rc26_mcu_transport` 的 ROS service/topic。
- 只要 `/mechanism/send_command` 或 `/mechanism/command_feedback` 异常，优先排查 `rc26_mcu_transport` 是否已启动并打开目标 MCU 串口。
- 当前只支持 `hal_type:=shared_serial`；其它 `hal_type` 会在 `configure` 阶段失败。

## 维护规则

新增机构原始命令时，先在 `rc26_serial/protocol.hpp` 定义 ID，再由需要该命令的上层直接调用 `/mechanism/send_command`。只有确实需要恢复高层动作语义时，才重新设计 action、完成反馈、超时规则和中间层文档契约。

## 本轮同步

2026-06-30 同步：协议真源新增梅林预选赛入口高侧 KFS 夹取 `ENTRY_GRAB_KFS_UP(0x0F)` / `ENTRY_GRAB_KFS_UP_DONE(0x0B)`。`rc26_mechanism` 仍保持 shared-serial 占位，不恢复高层 catalog；完成反馈由上层按同 `seq` 自行解释。

2026-06-27 同步：KFS 向下夹取新增 `ARM_SECOND_LOWER(0x0E)` / `ARM_SECOND_LOWER_DONE(0x0A)`，仍由上层经 `/mechanism/send_command` 和 `/mechanism/command_feedback` 解释，本包不恢复高层动作 catalog。

2026-06-26 同步：删除旧高层 action server 注册、旧执行上下文使用路径、旧 catalog 条目和依赖旧完成反馈的测试；catalog 当前为空。`rc26_mechanism` 保留为最小 shared-serial lifecycle 占位包，真实命令入口收口到 `rc26_mcu_transport` 的 raw transport。
