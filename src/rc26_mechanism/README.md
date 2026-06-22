# RC 2026 Mechanism 模块 (rc26_mechanism)

## 模块简介

`rc26_mechanism` 是 R2 自动机器人的最小机构执行边界，负责把上层动作语义可靠地下发给下位机，并收敛执行结果。

当前实现已经按“真机最小可用链路”收口：

- 只保留 `shared_serial` 真机 HAL
- `launch/mechanism.launch.py` 默认同时启动 `rc26_mcu_transport`；如果目标 MCU transport 已经由 bringup 或其它入口启动，可显式传 `start_mcu_transport:=false`
- 只保留 3 个动作入口：
  - `/mechanism/grab_tip`
  - `/mechanism/assemble_weapon`
  - `/mechanism/run_command`
- `/mechanism/status` 只保留 3 个观测字段：
  - `hal_open`
  - `last_error_code`
  - `current_cmd_id`

## 核心设计

1. **生命周期门禁化管理**：只有节点处于 `active` 且 HAL 已打开时才接收机构动作；退活、错误或取消时会清退执行上下文并发送 `STOP`。
2. **集中命令目录真源**：`mechanism_command_catalog` 统一描述命令是否允许走 `/mechanism/run_command`、什么反馈算成功、默认 timeout 是多少。
3. **共享串口优先**：真实部署只支持 `shared_serial`，通过 `/mechanism/send_command` 与 `/mechanism/command_feedback` 复用 `rc26_mcu_transport` 已经持有的目标 MCU 串口。
4. **单动作串行执行**：同一时刻只允许一个机构动作执行，避免上层并发 goal 把机构链路打乱。
5. **反馈收敛保留**：继续保留 `pending_contexts_`、`buffered_feedbacks_`、早到成功反馈与超时处理；当前 MCU 不再回 `ACTION_FAIL/ERROR`，所以 mechanism 侧只按命令专属完成反馈或超时来收敛结果。

## 目录结构

- `include/rc26_mechanism/nodes` + `src/nodes`：生命周期节点与 action 服务端
- `include/rc26_mechanism/catalog` + `src/catalog`：机构命令目录真源
- `include/rc26_mechanism/runtime`：`CommandContext` 等运行时辅助类型
- `include/rc26_mechanism/hal/contracts`：HAL 抽象接口
- `include/rc26_mechanism/hal/shared_serial` + `src/hal/shared_serial`：真实共享串口桥接实现
- `test/catalog`、`test/transport`：命令目录与共享 transport 回归测试

## 当前对外语义

- `GrabTip.action`：保留专用抓端头入口，作为标准专用动作例子。
- `AssembleWeapon.action`：保留专用组装入口。
- `ExecuteMechanism.action`：只继续承接通用机构命令。
- 当前 catalog 中只保留 4 条机构业务命令：
  - `GRAB_TIP`
  - `ASSEMBLE_WEAPON`
  - `GRAB_KFS`
  - `PLACE_KFS_GRID`
- 其中允许走 `/mechanism/run_command` 的只有：
  - `GRAB_KFS`
  - `PLACE_KFS_GRID`

也就是说，KFS 抓取和九宫格放置当前统一通过 `/mechanism/run_command` 下发；`grab_tip` 与 `assemble_weapon` 继续保留专用 action。

## 维护规则

新增机构命令时，固定按下面的顺序维护：

1. 在 `rc26_serial/protocol.hpp` 增加新的 `CommandID` / `FeedbackID`
2. 在 `rc26_mechanism/catalog/mechanism_command_catalog.*` 增加命令目录项
3. 如果只需要通用执行，直接调用 `/mechanism/run_command`
4. 只有确实需要更强业务语义时，才再新增专用 action 包装

## 注意事项

- 当前真实部署只支持 `hal_type:=shared_serial`；其它 `hal_type` 会在 `configure` 阶段直接失败。
- `shared_serial` 复用的是 `rc26_mcu_transport` 提供的 `/mechanism/send_command` 与 `/mechanism/command_feedback`；涉及机构动作的运行链必须先启动或同时启动该目标 MCU 串口 owner。
- 单独启动 `ros2 launch rc26_mechanism mechanism.launch.py` 时会默认同步启动 `rc26_mcu_transport`；若该 provider 已存在，请传 `start_mcu_transport:=false`，避免重复打开同一物理串口。
- 前/后推杆 sidecar 命令不再属于 `rc26_mechanism` 的业务命令目录；遥控链当前直接调用 `/mechanism/send_command`。
- 当前机构节点不再维护端头状态机，也不再通过 `/mechanism/status` 发布端头姿态、装配计数或通信健康统计。
- 包目录已经物理清理掉历史残留空目录 `include/rc26_mechanism/hal/{fault,replay,sim}`、`src/hal/{fault,replay,sim}` 与 `launch/__pycache__`，当前源码树只保留最小真机链路对应的目录。
- 根目录集中式调试文档已删除；如需补充开发或实车命令行验证，应直接维护在本 README 或包内脚本中。
