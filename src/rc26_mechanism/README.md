# RC 2026 Mechanism 模块 (rc26_mechanism)

## 模块简介

`rc26_mechanism` 当前是 R2 机构共享串口链路的最小生命周期占位包。旧的高层 action 语义已经下线，真实机构命令通过 `rc26_mcu_transport` 提供的 raw transport service 下发。

当前实现已经按“真机最小可用链路”收口：

- 只保留 `shared_serial` 真机 HAL
- `launch/mechanism.launch.py` 默认同时启动 `rc26_mcu_transport`；如果目标 MCU transport 已经由 bringup 或其它入口启动，可显式传 `start_mcu_transport:=false`
- 不再创建旧抓端头、旧组装或旧通用执行 action endpoint
- 不再发布 `/mechanism/status`

## 核心设计

1. **共享串口优先**：真实部署只支持 `shared_serial`，通过 `/mechanism/send_command` 与 `/mechanism/command_feedback` 复用 `rc26_mcu_transport` 已经持有的目标 MCU 串口。
2. **raw transport 收口**：本包不再赋予“什么反馈算动作完成”的高层语义；所有机构命令都由上层直接调用 `/mechanism/send_command`。
3. **旧 action 下线**：旧抓端头、旧组装与旧通用执行 action 已经从 `rc26_interfaces` 生成清单中移除。

## 目录结构

- `include/rc26_mechanism/nodes` + `src/nodes`：轻量生命周期节点
- `include/rc26_mechanism/catalog` + `src/catalog`：空机构命令目录，保留给旧调用点编译兼容
- `include/rc26_mechanism/runtime`：归档运行时辅助类型，当前节点不再使用
- `include/rc26_mechanism/hal/contracts`：HAL 抽象接口
- `include/rc26_mechanism/hal/shared_serial` + `src/hal/shared_serial`：真实共享串口桥接实现
- `test/catalog`：空命令目录回归测试

## 当前对外语义

- 当前没有高层 action 对外语义。
- `mechanism_command_catalog()` 返回空目录；`GRAB_TIP`、`PLACE_KFS_GRID`、上下 KFS 夹取和推杆命令若要发送，都直接调用 `/mechanism/send_command`。
- 旧通用 KFS 夹取、旧组装动作、旧专用完成反馈和旧即时负确认已从串口协议删除。

KFS 阶梯等待测试链当前不把新的上下阶梯夹取预留命令加入 catalog。`ARM_RAISE(0x04)`、`ARM_LOWER(0x05)` 由决策层测试节点直接通过 `/mechanism/send_command` 发送，并等待同 `seq` 的 `ARM_RAISE_DONE(0x02)` / `ARM_LOWER_DONE(0x03)`；`GRAB_KFS_UP(0x03)` 与 `GRAB_KFS_DOWN(0x02)` 只作为后续本车目标标签明确后的串口预留能力。

## 维护规则

新增机构命令时，固定按下面的顺序维护：

1. 在 `rc26_serial/protocol.hpp` 增加新的 `CommandID` / `FeedbackID`
2. 上层直接调用 `/mechanism/send_command` 发送原始命令 ID
3. 只有确实需要恢复高层动作语义时，才重新设计 action、完成反馈和文档契约

## 注意事项

- 当前真实部署只支持 `hal_type:=shared_serial`；其它 `hal_type` 会在 `configure` 阶段直接失败。
- `shared_serial` 复用的是 `rc26_mcu_transport` 提供的 `/mechanism/send_command` 与 `/mechanism/command_feedback`；涉及机构动作的运行链必须先启动或同时启动该目标 MCU 串口 owner。
- 单独启动 `ros2 launch rc26_mechanism mechanism.launch.py` 时会默认同步启动 `rc26_mcu_transport`；若该 provider 已存在，请传 `start_mcu_transport:=false`，避免重复打开同一物理串口。
- 前/后推杆 sidecar 命令不再属于 `rc26_mechanism` 的业务命令目录；遥控链当前直接调用 `/mechanism/send_command`。
- 当前机构节点不再维护端头状态机，也不再发布 `/mechanism/status`、端头姿态、装配计数或通信健康统计。
- 包目录已经物理清理掉历史残留空目录 `include/rc26_mechanism/hal/{fault,replay,sim}`、`src/hal/{fault,replay,sim}` 与 `launch/__pycache__`，当前源码树只保留最小真机链路对应的目录。
- 根目录集中式调试文档已删除；如需补充开发或实车命令行验证，应直接维护在本 README 或包内脚本中。
