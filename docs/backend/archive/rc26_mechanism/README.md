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

上层若要发送 `GRAB_TIP(0x01)`、`GRAB_KFS_DOWN(0x02)`、`GRAB_KFS_UP(0x03)`、`PLACE_KFS_GRID(0x06)`、推杆命令或其它原始机构命令，都直接调用 `/mechanism/send_command`。service `accepted=true` 只表示目标 MCU 已返回通用 `ACK(0x00)`，不表示动作已经完成。

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

当前业务反馈口径：

- `ARM_RAISE_DONE = 0x02`
- `ARM_LOWER_DONE = 0x03`
- `FRONT_LASER_HEIGHT_JUMP = 0x04`
- `REAR_LASER_HEIGHT_JUMP = 0x05`
- `FRONT_LIMIT_SWITCH_TRIGGERED = 0x06`
- `FRONT_SECOND_LASER_HEIGHT_JUMP = 0x07`

KFS 阶梯等待测试链不把上下阶梯夹取预留命令加入 catalog。`ARM_RAISE(0x04)` / `ARM_LOWER(0x05)` 由决策层直接通过 `/mechanism/send_command` 发送，并等待同 `seq` 的 `ARM_RAISE_DONE(0x02)` / `ARM_LOWER_DONE(0x03)`。`GRAB_KFS_UP(0x03)` 与 `GRAB_KFS_DOWN(0x02)` 只作为后续本车目标标签明确后的 raw transport 预留能力。

`PLACE_KFS_GRID(0x06)` 若仍需发送，只能走 raw `/mechanism/send_command`，不再有高层“完成反馈即成功”的封装。

## 真实部署口径

- 真机上由 `rc26_mcu_transport` 作为目标 MCU 串口 owner。
- `rc26_mechanism` 通过 `shared_serial` 复用 `rc26_mcu_transport` 的 ROS service/topic。
- 只要 `/mechanism/send_command` 或 `/mechanism/command_feedback` 异常，优先排查 `rc26_mcu_transport` 是否已启动并打开目标 MCU 串口。
- 当前只支持 `hal_type:=shared_serial`；其它 `hal_type` 会在 `configure` 阶段失败。

## 维护规则

新增机构原始命令时，先在 `rc26_serial/protocol.hpp` 定义 ID，再由需要该命令的上层直接调用 `/mechanism/send_command`。只有确实需要恢复高层动作语义时，才重新设计 action、完成反馈、超时规则和中间层文档契约。

## 本轮同步

2026-06-26 同步：删除旧高层 action server 注册、旧执行上下文使用路径、旧 catalog 条目和依赖旧完成反馈的测试；catalog 当前为空。`rc26_mechanism` 保留为最小 shared-serial lifecycle 占位包，真实命令入口收口到 `rc26_mcu_transport` 的 raw transport。
